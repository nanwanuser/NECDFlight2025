/**
  ******************************************************************************
  * @file    ground_station_node.cpp
  * @brief   地面站通信ROS节点
  * @version V2.5.3
  * @date    2025-08-01
  ******************************************************************************
  */

#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <set>
#include <cmath>

#include "ground_station_comm/data_communication_pkg.hpp"
#include "ground_station_comm/trajectory_mapper.hpp"
#include "ground_station_comm/AnimalData.h"

class GroundStationNode {
private:
    // ROS相关
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber pose_sub_;
    ros::Subscriber animal_sub_;
    ros::Subscriber state_sub_;
    ros::Publisher cmd_pub_;
    ros::Publisher start_detect_pub_;
    ros::Timer serial_timer_;
    ros::Timer control_timer_;

    // MAVROS服务客户端
    ros::ServiceClient arming_client_;
    ros::ServiceClient takeoff_client_;
    ros::ServiceClient set_mode_client_;
    ros::ServiceClient land_client_;

    // 串口相关
    std::unique_ptr<serial::Serial> serial_port_;
    std::string serial_device_;
    int serial_baudrate_;

    // 通信协议
    DataComm::DataCommProtocol comm_protocol_;

    // 轨迹映射
    GroundStation::TrajectoryMapper trajectory_mapper_;
    std::vector<geometry_msgs::PoseStamped> waypoints_;
    std::mutex waypoints_mutex_;

    // 状态管理
    enum class FlightState {
        IDLE,                    // 空闲
        ARMING,                  // 解锁中
        TAKING_OFF,              // 起飞中
        FOLLOWING_TRAJECTORY,    // 按照预定航线飞行
        WAYPOINT_HOVER_PRE,      // 航点到达后的第一次悬停（发送检测前）
        WAYPOINT_HOVER_POST,     // 航点到达后的第二次悬停（发送检测后）
        MOVING_TO_GRID_CENTER,   // 移动到网格中心
        SCANNING_GRID,          // 在网格内扫描动物
        SCAN_HOVERING,          // 扫描间的短暂悬停
        MOVING_TO_NEXT_VECTOR,  // 移动到下一个扫描向量
        LANDING                 // 斜飞降落
    };

    std::atomic<FlightState> flight_state_;
    std::atomic<int> current_waypoint_index_;
    std::atomic<bool> is_flying_;
    std::atomic<bool> trajectory_received_;
    geometry_msgs::PoseStamped current_pose_;
    std::mutex pose_mutex_;

    // MAVROS状态
    mavros_msgs::State current_mavros_state_;
    std::mutex mavros_state_mutex_;

    // 当前目标位置（用于连续发布）
    geometry_msgs::PoseStamped current_target_;
    std::mutex target_mutex_;
    std::atomic<bool> has_target_;

    // 降落相关
    ros::Time landing_start_time_;
    std::atomic<bool> is_landing_;
    geometry_msgs::Point landing_start_position_;  // 降落开始位置
    double landing_start_height_;                   // 降落开始高度

    // 动物检测相关
    ground_station_comm::AnimalData latest_animal_data_;
    std::mutex animal_data_mutex_;
    std::atomic<bool> animal_data_received_;
    std::vector<geometry_msgs::Point> scan_vectors_;  // 扫描向量
    size_t current_vector_index_;
    geometry_msgs::Point scan_start_position_;        // 扫描起始位置（网格中心）
    geometry_msgs::Point current_scan_target_;        // 当前扫描目标位置
    geometry_msgs::Point grid_center_;                // 当前网格中心
    ros::Time hover_start_time_;                      // 悬停开始时间
    static constexpr double HOVER_DURATION = 1.2;     // 扫描悬停持续时间（秒）
    static constexpr double WAYPOINT_HOVER_DURATION = 0.25;  // 航点悬停持续时间（秒）
    ros::Time waypoint_hover_start_time_;             // 航点悬停开始时间
    bool detect_signal_sent_;                         // 检测信号是否已发送
    bool should_scan_after_hover_;                    // 悬停后是否需要扫描
    uint8_t current_scanning_grid_;                   // 当前正在扫描的网格编号

    // 已访问网格跟踪
    std::set<uint8_t> visited_grids_;
    std::mutex visited_grids_mutex_;
    uint8_t current_grid_number_;

    // 已检测航点跟踪
    std::set<size_t> detected_waypoints_;
    std::mutex detected_waypoints_mutex_;

    // 已扫描网格跟踪
    std::set<uint8_t> scanned_grids_;
    std::mutex scanned_grids_mutex_;

    // 位置到达阈值
    double position_threshold_;
    static constexpr double GRID_BOUNDARY_THRESHOLD = 0.1;  // 10cm
    static constexpr double FIXED_HEIGHT = 0.9;             // 固定飞行高度0.9米
    static constexpr double LANDING_HEIGHT = 0.1;           // 降落目标高度0.1米
    static constexpr double TAKEOFF_HEIGHT = 0.9;           // 起飞高度0.9米

    // 命令定义
    static constexpr uint8_t CMD_TRAJECTORY = 0x01;
    static constexpr uint8_t CMD_TAKEOFF = 0x02;      // 复用作为降落通知命令
    static constexpr uint8_t CMD_ANIMAL_DATA = 0x03;  // 新增：动物数据命令
    static constexpr uint8_t TAKEOFF_DATA = 0x11;
    static constexpr uint8_t LANDING_DATA = 0xFF;     // 新增：降落数据

public:
    GroundStationNode() :
        pnh_("~"),
        flight_state_(FlightState::IDLE),
        current_waypoint_index_(0),
        is_flying_(false),
        trajectory_received_(false),
        has_target_(false),
        is_landing_(false),
        animal_data_received_(false),
        current_vector_index_(0),
        current_grid_number_(0),
        landing_start_height_(FIXED_HEIGHT),
        detect_signal_sent_(false),
        should_scan_after_hover_(false),
        current_scanning_grid_(0) {

        // 读取参数
        pnh_.param<std::string>("serial_device", serial_device_, "/dev/ttyUSB0");
        pnh_.param<int>("serial_baudrate", serial_baudrate_, 115200);
        pnh_.param<double>("position_threshold", position_threshold_, 0.3);

        // 初始化串口
        initSerial();

        // 初始化通信协议
        initProtocol();

        // 初始化ROS接口 - MAVROS话题
        pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 10,
                                  &GroundStationNode::poseCallback, this);
        state_sub_ = nh_.subscribe("/mavros/state", 10,
                                   &GroundStationNode::mavrosStateCallback, this);
        animal_sub_ = nh_.subscribe("/animal", 10,
                                   &GroundStationNode::animalDataCallback, this);

        // 发布到MAVROS的位置控制话题
        cmd_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
        start_detect_pub_ = nh_.advertise<std_msgs::Bool>("/start_detect", 10);

        // MAVROS服务客户端
        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        takeoff_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
        land_client_ = nh_.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");

        // 定时器 - 控制定时器用于连续发布目标位置
        serial_timer_ = nh_.createTimer(ros::Duration(0.01),
                                       &GroundStationNode::serialTimerCallback, this);
        control_timer_ = nh_.createTimer(ros::Duration(0.05),  // 20Hz发布频率
                                        &GroundStationNode::controlTimerCallback, this);

        ROS_INFO("Ground Station Node initialized");
        ROS_INFO("Publishing setpoints to: /mavros/setpoint_position/local");
        ROS_INFO("Flight height: %.1f meters, Landing height: %.1f meters", FIXED_HEIGHT, LANDING_HEIGHT);
    }

private:
    /**
     * @brief 初始化串口
     */
    void initSerial() {
        try {
            serial_port_.reset(new serial::Serial(
                serial_device_, serial_baudrate_,
                serial::Timeout::simpleTimeout(100)));

            if (serial_port_->isOpen()) {
                ROS_INFO("Serial port %s opened at %d baud",
                        serial_device_.c_str(), serial_baudrate_);
            } else {
                ROS_ERROR("Failed to open serial port %s", serial_device_.c_str());
            }
        } catch (serial::IOException& e) {
            ROS_ERROR("Serial port exception: %s", e.what());
        }
    }

    /**
     * @brief 初始化通信协议
     */
    void initProtocol() {
        // 设置发送回调
        comm_protocol_.setTransmitCallback(
            [this](const uint8_t* data, uint16_t len) {
                if (serial_port_ && serial_port_->isOpen()) {
                    serial_port_->write(data, len);
                }
            });

        // 设置接收回调
        comm_protocol_.setPacketCallback(
            [this](uint8_t cmd, const uint8_t* data, uint16_t len) {
                handlePacket(cmd, data, len);
            });

        comm_protocol_.init();
    }

    /**
     * @brief 处理接收到的数据包
     */
    void handlePacket(uint8_t cmd, const uint8_t* data, uint16_t len) {
        switch (cmd) {
            case CMD_TRAJECTORY:
                handleTrajectoryData(data, len);
                break;

            case CMD_TAKEOFF:
                handleTakeoffCommand(data, len);
                break;

            default:
                ROS_WARN("Unknown command: 0x%02X", cmd);
                break;
        }
    }

    /**
     * @brief 处理轨迹数据
     */
    void handleTrajectoryData(const uint8_t* data, uint16_t len) {
        ROS_INFO("Received trajectory data with %d waypoints", len);

        // 检查是否至少有2个航点（因为需要在倒数第二个点降落）
        if (len < 2) {
            ROS_WARN("Trajectory must have at least 2 waypoints for landing logic");
            return;
        }

        // 解析轨迹
        auto new_waypoints = trajectory_mapper_.parseTrajectory(data, len);

        // 确保所有航点高度为0.9米
        for (auto& waypoint : new_waypoints) {
            waypoint.pose.position.z = FIXED_HEIGHT;
        }

        {
            std::lock_guard<std::mutex> lock(waypoints_mutex_);
            waypoints_ = new_waypoints;
            current_waypoint_index_ = 0;
            trajectory_received_ = true;
        }

        // 清空已检测航点记录
        {
            std::lock_guard<std::mutex> lock(detected_waypoints_mutex_);
            detected_waypoints_.clear();
        }

        // 清空已扫描网格记录
        {
            std::lock_guard<std::mutex> lock(scanned_grids_mutex_);
            scanned_grids_.clear();
        }

        // 如果已经在飞行，设置第一个航点为目标
        if (is_flying_ && !waypoints_.empty()) {
            setCurrentTarget();
        }
    }

    /**
     * @brief 处理起飞命令 - 使用MAVROS
     */
    void handleTakeoffCommand(const uint8_t* data, uint16_t len) {
        if (len != 1 || data[0] != TAKEOFF_DATA) {
            ROS_WARN("Invalid takeoff command data");
            return;
        }

        ROS_INFO("Received takeoff command, initiating MAVROS takeoff sequence");

        // 开始起飞流程
        flight_state_ = FlightState::ARMING;

        // 设置起飞位置作为第一个目标
        geometry_msgs::PoseStamped takeoff_pose;
        takeoff_pose.header.frame_id = "map";
        takeoff_pose.header.stamp = ros::Time::now();

        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            takeoff_pose.pose.position.x = current_pose_.pose.position.x;
            takeoff_pose.pose.position.y = current_pose_.pose.position.y;
        }
        takeoff_pose.pose.position.z = TAKEOFF_HEIGHT;
        takeoff_pose.pose.orientation.w = 1.0;

        {
            std::lock_guard<std::mutex> target_lock(target_mutex_);
            current_target_ = takeoff_pose;
        }
        has_target_ = true;

        // 执行起飞序列
        performTakeoffSequence();
    }

    /**
     * @brief 执行MAVROS起飞序列
     */
    void performTakeoffSequence() {
        // 等待FCU连接
        while (ros::ok()) {
            std::lock_guard<std::mutex> lock(mavros_state_mutex_);
            if (current_mavros_state_.connected) {
                break;
            }
            ROS_INFO_ONCE("Waiting for FCU connection...");
            ros::Duration(0.5).sleep();
        }

        // 发送一些setpoints在切换到OFFBOARD模式之前
        ROS_INFO("Sending initial setpoints before OFFBOARD mode");
        for (int i = 0; ros::ok() && i < 10; i++) {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "map";
            {
                std::lock_guard<std::mutex> lock(pose_mutex_);
                pose.pose.position.x = current_pose_.pose.position.x;
                pose.pose.position.y = current_pose_.pose.position.y;
            }
            pose.pose.position.z = TAKEOFF_HEIGHT;
            pose.pose.orientation.w = 1.0;
            cmd_pub_.publish(pose);
            ros::Duration(0.1).sleep();
        }

        // 设置OFFBOARD模式
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";

        if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
            ROS_INFO("OFFBOARD mode enabled");
        } else {
            ROS_WARN("Failed to set OFFBOARD mode, continuing anyway");
        }

        // 解锁
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;

        if (arming_client_.call(arm_cmd) && arm_cmd.response.success) {
            ROS_INFO("Vehicle armed");
        } else {
            ROS_WARN("Arming failed, continuing anyway");
        }

        // 切换到起飞状态
        flight_state_ = FlightState::TAKING_OFF;
        is_flying_ = true;

        ROS_INFO("Takeoff sequence complete, ascending to %.1f meters", TAKEOFF_HEIGHT);
    }

    /**
     * @brief 设置当前目标航点
     */
    void setCurrentTarget() {
        std::lock_guard<std::mutex> lock(waypoints_mutex_);

        size_t index = current_waypoint_index_.load();
        if (index >= waypoints_.size()) {
            ROS_WARN("No more waypoints to set as target");
            has_target_ = false;
            return;
        }

        // 设置当前目标
        {
            std::lock_guard<std::mutex> target_lock(target_mutex_);
            current_target_ = waypoints_[index];
            // 确保高度为0.9米
            current_target_.pose.position.z = FIXED_HEIGHT;
        }
        has_target_ = true;

        ROS_INFO("Set waypoint %zu/%zu as current target: (%.2f, %.2f, %.2f)",
                index + 1, waypoints_.size(),
                waypoints_[index].pose.position.x,
                waypoints_[index].pose.position.y,
                FIXED_HEIGHT);
    }

    /**
     * @brief 移动到下一个航点
     */
    void moveToNextWaypoint() {
        // 移动到下一个航点
        current_waypoint_index_++;

        // 设置下一个航点为目标
        std::lock_guard<std::mutex> lock(waypoints_mutex_);
        if (current_waypoint_index_ < waypoints_.size()) {
            flight_state_ = FlightState::FOLLOWING_TRAJECTORY;
            lock.~lock_guard();
            setCurrentTarget();
        } else {
            ROS_INFO("All waypoints completed");
            is_flying_ = false;
            flight_state_ = FlightState::IDLE;
            has_target_ = false;
        }
    }

    /**
     * @brief 设置网格中心目标
     */
    void setGridCenterTarget() {
        // 获取当前网格中心
        geometry_msgs::Point current_position;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            current_position = current_pose_.pose.position;
        }

        uint8_t grid_number = trajectory_mapper_.positionToGridNumber(current_position);
        grid_center_ = trajectory_mapper_.getGridCenter(grid_number);
        current_scanning_grid_ = grid_number;  // 记录当前正在扫描的网格

        // 设置网格中心为目标
        geometry_msgs::PoseStamped target;
        target.header.frame_id = "map";
        target.header.stamp = ros::Time::now();
        target.pose.position = grid_center_;
        target.pose.position.z = FIXED_HEIGHT;  // 保持固定高度
        target.pose.orientation.w = 1.0;

        {
            std::lock_guard<std::mutex> target_lock(target_mutex_);
            current_target_ = target;
        }
        has_target_ = true;

        ROS_INFO("Moving to grid center of grid %d: (%.2f, %.2f, %.2f)",
                grid_number, grid_center_.x, grid_center_.y, FIXED_HEIGHT);
    }

    /**
     * @brief 设置扫描目标
     */
    void setScanTarget() {
        if (current_vector_index_ >= scan_vectors_.size()) {
            ROS_INFO("Scan complete for grid %d, marking as scanned", current_scanning_grid_);

            // 标记当前网格为已扫描
            {
                std::lock_guard<std::mutex> lock(scanned_grids_mutex_);
                scanned_grids_.insert(current_scanning_grid_);
            }

            // 移动到下一个航点
            flight_state_ = FlightState::FOLLOWING_TRAJECTORY;
            moveToNextWaypoint();
            return;
        }

        // 计算目标位置
        geometry_msgs::PoseStamped target;
        target.header.frame_id = "map";
        target.header.stamp = ros::Time::now();

        // 从网格中心向扫描方向移动
        double scan_distance = 0.25;  // 向外扫描25cm（减小扫描距离）
        current_scan_target_.x = grid_center_.x + scan_vectors_[current_vector_index_].x * scan_distance;
        current_scan_target_.y = grid_center_.y + scan_vectors_[current_vector_index_].y * scan_distance;
        current_scan_target_.z = FIXED_HEIGHT;  // 保持固定高度

        target.pose.position = current_scan_target_;
        target.pose.orientation.w = 1.0;

        // 设置为当前目标
        {
            std::lock_guard<std::mutex> target_lock(target_mutex_);
            current_target_ = target;
        }
        has_target_ = true;

        ROS_INFO("Set scan target %zu/%zu as current target: (%.2f, %.2f, %.2f)",
                current_vector_index_ + 1, scan_vectors_.size(),
                current_scan_target_.x,
                current_scan_target_.y,
                FIXED_HEIGHT);
    }

    /**
     * @brief 设置降落目标（斜飞到0,0,0.1）
     */
    void setLandingTarget() {
        geometry_msgs::PoseStamped target;
        target.header.frame_id = "map";
        target.header.stamp = ros::Time::now();

        // 目标为(0,0,0.1)
        target.pose.position.x = 0.0;
        target.pose.position.y = 0.0;
        target.pose.position.z = LANDING_HEIGHT;  // 0.1米
        target.pose.orientation.w = 1.0;

        // 设置为当前目标
        {
            std::lock_guard<std::mutex> target_lock(target_mutex_);
            current_target_ = target;
        }
        has_target_ = true;

        ROS_INFO("Set landing target: (0.0, 0.0, %.1f) - diagonal descent to origin", LANDING_HEIGHT);
    }

    /**
     * @brief 发送降落通知到地面站
     */
    void sendLandingNotification() {
        uint8_t landing_data = LANDING_DATA;  // 0xFF

        // 使用CMD_TAKEOFF(0x02)作为命令字，发送0xFF数据
        comm_protocol_.send(CMD_TAKEOFF, &landing_data, 1);

        ROS_INFO("Sent landing notification to ground station: CMD=0x%02X, Data=0x%02X",
                CMD_TAKEOFF, landing_data);
    }

    /**
     * @brief MAVROS状态回调
     */
    void mavrosStateCallback(const mavros_msgs::State::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mavros_state_mutex_);
        current_mavros_state_ = *msg;
    }

    /**
     * @brief 动物数据回调函数
     * 注意：这个回调是在接收到动物检测结果时触发的
     */
    void animalDataCallback(const ground_station_comm::AnimalData::ConstPtr& msg) {
        {
            std::lock_guard<std::mutex> lock(animal_data_mutex_);
            latest_animal_data_ = *msg;
            animal_data_received_ = true;
        }

        // 向地面站发送动物数据
        sendAnimalDataToGroundStation();

        // 检查是否需要进入扫描模式
        checkAndStartScanning();
    }

    /**
     * @brief 向地面站发送动物数据
     */
    void sendAnimalDataToGroundStation() {
        uint8_t data[6];  // 1字节网格标号 + 5字节动物数量

        // 获取当前网格标号
        geometry_msgs::Point current_position;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            current_position = current_pose_.pose.position;
        }

        data[0] = trajectory_mapper_.positionToGridNumber(current_position);

        // 添加动物数量
        {
            std::lock_guard<std::mutex> lock(animal_data_mutex_);
            data[1] = latest_animal_data_.peacock;
            data[2] = latest_animal_data_.wolf;
            data[3] = latest_animal_data_.monkey;
            data[4] = latest_animal_data_.tiger;
            data[5] = latest_animal_data_.elephant;
        }

        // 发送数据
        comm_protocol_.send(CMD_ANIMAL_DATA, data, 6);

        ROS_INFO("Sent animal data to ground station: Grid=%d, Animals=[%d,%d,%d,%d,%d]",
                data[0], data[1], data[2], data[3], data[4], data[5]);
    }

    /**
     * @brief 检查并开始扫描
     */
    void checkAndStartScanning() {
        std::lock_guard<std::mutex> lock(animal_data_mutex_);

        // 计算动物总数
        uint8_t total_animals = latest_animal_data_.peacock +
                               latest_animal_data_.wolf +
                               latest_animal_data_.monkey +
                               latest_animal_data_.tiger +
                               latest_animal_data_.elephant;

        // 获取当前网格编号
        geometry_msgs::Point current_position;
        {
            std::lock_guard<std::mutex> lock_pose(pose_mutex_);
            current_position = current_pose_.pose.position;
        }
        uint8_t grid_number = trajectory_mapper_.positionToGridNumber(current_position);

        // 检查当前网格是否已经扫描过
        bool already_scanned = false;
        {
            std::lock_guard<std::mutex> lock_scan(scanned_grids_mutex_);
            already_scanned = (scanned_grids_.find(grid_number) != scanned_grids_.end());
        }

        if (already_scanned) {
            ROS_INFO("Grid %d already scanned, skipping scan even though %d animals detected",
                    grid_number, total_animals);
            should_scan_after_hover_ = false;
        } else if (total_animals > 0) {
            ROS_INFO("Detected %d animals in grid %d, will start grid scanning after hover",
                    total_animals, grid_number);

            // 生成扫描向量
            generateScanVectors(total_animals);

            // 标记悬停后需要扫描
            should_scan_after_hover_ = true;
        } else {
            ROS_INFO("No animals detected in grid %d", grid_number);
            should_scan_after_hover_ = false;
        }
    }

    /**
     * @brief 生成扫描向量
     * @note 调用此函数前必须已经持有 animal_data_mutex_ 锁
     */
    void generateScanVectors(uint8_t count) {
        scan_vectors_.clear();

        // 根据动物坐标生成扫描向量
        for (const auto& coord : latest_animal_data_.coordinates) {
            geometry_msgs::Point vec;
            // 归一化向量方向
            double length = sqrt(coord.x * coord.x + coord.y * coord.y);
            if (length > 0) {
                vec.x = coord.x / length;
                vec.y = coord.y / length;
                vec.z = 0;  // 保持高度不变
                scan_vectors_.push_back(vec);
            }
        }

        // 如果坐标数量不足，使用默认的扫描模式
        if (scan_vectors_.size() < count) {
            double angle_step = 2 * M_PI / count;
            for (size_t i = scan_vectors_.size(); i < count; ++i) {
                geometry_msgs::Point vec;
                vec.x = cos(i * angle_step);
                vec.y = sin(i * angle_step);
                vec.z = 0;
                scan_vectors_.push_back(vec);
            }
        }

        ROS_INFO("Generated %zu scan vectors", scan_vectors_.size());
    }

    /**
     * @brief 位置回调函数
     */
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            current_pose_ = *msg;
        }

        // 根据飞行状态进行相应处理
        switch (flight_state_.load()) {
            case FlightState::TAKING_OFF:
                checkTakeoffComplete();
                break;

            case FlightState::FOLLOWING_TRAJECTORY:
                if (is_flying_ && trajectory_received_) {
                    checkWaypointReached();
                }
                break;

            case FlightState::WAYPOINT_HOVER_PRE:
                checkWaypointPreHoverComplete();
                break;

            case FlightState::WAYPOINT_HOVER_POST:
                checkWaypointPostHoverComplete();
                break;

            case FlightState::MOVING_TO_GRID_CENTER:
                checkGridCenterReached();
                break;

            case FlightState::SCANNING_GRID:
                checkScanTargetReached();
                break;

            case FlightState::SCAN_HOVERING:
                checkHoverComplete();
                break;

            case FlightState::LANDING:
                checkLandingProgress();
                break;

            default:
                break;
        }
    }

    /**
     * @brief 检查起飞是否完成
     */
    void checkTakeoffComplete() {
        std::lock_guard<std::mutex> lock(pose_mutex_);

        // 检查是否达到起飞高度
        if (current_pose_.pose.position.z >= TAKEOFF_HEIGHT - 0.1) {
            ROS_INFO("Takeoff complete, reached target height");

            // 切换到轨迹跟踪状态
            flight_state_ = FlightState::FOLLOWING_TRAJECTORY;

            // 如果已有轨迹，设置第一个航点为目标
            if (trajectory_received_ && !waypoints_.empty()) {
                // 解锁后设置目标
                lock.~lock_guard();
                setCurrentTarget();
            }
        }
    }

    /**
     * @brief 检查是否到达网格中心
     */
    void checkGridCenterReached() {
        std::lock_guard<std::mutex> lock(pose_mutex_);

        // 计算与网格中心的距离
        double dx = current_pose_.pose.position.x - grid_center_.x;
        double dy = current_pose_.pose.position.y - grid_center_.y;
        double distance = sqrt(dx*dx + dy*dy);

        if (distance < 0.1) {  // 10cm内认为到达
            ROS_INFO("Reached grid center, starting scan");

            // 切换到扫描状态
            flight_state_ = FlightState::SCANNING_GRID;
            scan_start_position_ = grid_center_;  // 扫描起始位置设为网格中心

            // 设置第一个扫描目标
            lock.~lock_guard();
            setScanTarget();
        }
    }

    /**
     * @brief 检查是否到达扫描目标
     */
    void checkScanTargetReached() {
        std::lock_guard<std::mutex> lock(pose_mutex_);

        // 计算与目标的距离
        double dx = current_pose_.pose.position.x - current_scan_target_.x;
        double dy = current_pose_.pose.position.y - current_scan_target_.y;
        double distance = sqrt(dx*dx + dy*dy);

        if (distance < 0.05) {  // 5cm内认为到达
            ROS_INFO("Reached scan target %zu, starting hover", current_vector_index_ + 1);

            // 切换到悬停状态
            flight_state_ = FlightState::SCAN_HOVERING;
            hover_start_time_ = ros::Time::now();
        }
    }

    /**
     * @brief 检查悬停是否完成
     */
    void checkHoverComplete() {
        double hover_duration = (ros::Time::now() - hover_start_time_).toSec();

        if (hover_duration >= HOVER_DURATION) {
            ROS_INFO("Hover complete for vector %zu", current_vector_index_ + 1);

            current_vector_index_++;

            if (current_vector_index_ < scan_vectors_.size()) {
                // 继续扫描下一个向量
                flight_state_ = FlightState::SCANNING_GRID;
                setScanTarget();
            } else {
                // 所有扫描完成
                ROS_INFO("All scan vectors completed for grid %d", current_scanning_grid_);

                // 标记当前网格为已扫描
                {
                    std::lock_guard<std::mutex> lock(scanned_grids_mutex_);
                    scanned_grids_.insert(current_scanning_grid_);
                }

                // 移动到下一个航点
                flight_state_ = FlightState::FOLLOWING_TRAJECTORY;
                moveToNextWaypoint();
            }
        }
    }

    /**
     * @brief 检查是否到达航点
     */
    void checkWaypointReached() {
        std::lock_guard<std::mutex> lock_wp(waypoints_mutex_);
        std::lock_guard<std::mutex> lock_pose(pose_mutex_);

        size_t index = current_waypoint_index_.load();
        if (index >= waypoints_.size()) {
            return;
        }

        // 检查当前航点是否已经检测过
        {
            std::lock_guard<std::mutex> lock_detected(detected_waypoints_mutex_);
            if (detected_waypoints_.find(index) != detected_waypoints_.end()) {
                // 已经检测过，直接移动到下一个航点
                ROS_INFO("Waypoint %zu already detected, moving to next", index + 1);
                lock_wp.~lock_guard();
                lock_pose.~lock_guard();
                lock_detected.~lock_guard();
                moveToNextWaypoint();
                return;
            }
        }

        // 计算与目标航点的距离
        double dx = current_pose_.pose.position.x - waypoints_[index].pose.position.x;
        double dy = current_pose_.pose.position.y - waypoints_[index].pose.position.y;
        double distance = sqrt(dx*dx + dy*dy);  // 只考虑水平距离

        // 如果到达航点
        if (distance < position_threshold_) {
            ROS_INFO("Reached waypoint %zu/%zu", index + 1, waypoints_.size());

            // 标记当前航点为已检测
            {
                std::lock_guard<std::mutex> lock_detected(detected_waypoints_mutex_);
                detected_waypoints_.insert(index);
            }

            // 检查是否到达倒数第二个点
            if (index == waypoints_.size() - 2) {
                ROS_INFO("Reached second to last waypoint, starting diagonal descent to (0,0,0.1)");

                // 记录降落开始位置
                landing_start_position_ = current_pose_.pose.position;

                // 切换到降落状态
                flight_state_ = FlightState::LANDING;
                is_landing_ = true;
                landing_start_time_ = ros::Time::now();

                // 发送降落通知到地面站
                sendLandingNotification();

                // 设置降落目标
                lock_wp.~lock_guard();
                lock_pose.~lock_guard();
                setLandingTarget();

                ROS_INFO("Starting diagonal descent from (%.2f, %.2f, %.2f) to (0, 0, %.1f)",
                        landing_start_position_.x,
                        landing_start_position_.y,
                        landing_start_position_.z,
                        LANDING_HEIGHT);
                return;
            }

            // 如果是第一个航点，直接进入下一个航点，不进行悬停和检测
            if (index == 0) {
                ROS_INFO("First waypoint reached, moving to next waypoint without detection");

                lock_wp.~lock_guard();
                lock_pose.~lock_guard();
                moveToNextWaypoint();
            } else {
                // 非第一个航点，进入悬停流程
                ROS_INFO("Starting pre-detection hover at waypoint %zu", index + 1);

                // 切换到第一次悬停状态
                flight_state_ = FlightState::WAYPOINT_HOVER_PRE;
                waypoint_hover_start_time_ = ros::Time::now();
                detect_signal_sent_ = false;
                should_scan_after_hover_ = false;
            }
        }
    }

    /**
     * @brief 检查航点第一次悬停是否完成
     */
    void checkWaypointPreHoverComplete() {
        double hover_duration = (ros::Time::now() - waypoint_hover_start_time_).toSec();

        if (hover_duration >= WAYPOINT_HOVER_DURATION) {
            ROS_INFO("Pre-detection hover complete, sending start_detect=true then false");

            // 发送检测信号true
            std_msgs::Bool detect_msg;
            detect_msg.data = true;
            start_detect_pub_.publish(detect_msg);

            // 立即发送false
            detect_msg.data = false;
            start_detect_pub_.publish(detect_msg);

            detect_signal_sent_ = true;

            // 记录当前网格为已访问
            {
                geometry_msgs::Point current_position;
                {
                    std::lock_guard<std::mutex> lock(pose_mutex_);
                    current_position = current_pose_.pose.position;
                }
                uint8_t grid_number = trajectory_mapper_.positionToGridNumber(current_position);
                if (grid_number != 0) {
                    std::lock_guard<std::mutex> lock(visited_grids_mutex_);
                    visited_grids_.insert(grid_number);
                }
            }

            // 切换到第二次悬停状态
            flight_state_ = FlightState::WAYPOINT_HOVER_POST;
            waypoint_hover_start_time_ = ros::Time::now();
        }
    }

    /**
     * @brief 检查航点第二次悬停是否完成
     */
    void checkWaypointPostHoverComplete() {
        double hover_duration = (ros::Time::now() - waypoint_hover_start_time_).toSec();

        if (hover_duration >= WAYPOINT_HOVER_DURATION) {
            ROS_INFO("Post-detection hover complete");

            // 检查是否需要扫描
            if (should_scan_after_hover_) {
                ROS_INFO("Animal detected, starting grid scanning");

                // 切换到移动到网格中心状态
                flight_state_ = FlightState::MOVING_TO_GRID_CENTER;
                current_vector_index_ = 0;

                // 设置网格中心为目标
                setGridCenterTarget();

                // 重置标志
                should_scan_after_hover_ = false;
                animal_data_received_ = false;
            } else {
                // 没有检测到动物或网格已扫描，继续下一个航点
                ROS_INFO("No animal detection, no animals found, or grid already scanned - moving to next waypoint");
                moveToNextWaypoint();
            }
        }
    }

    /**
     * @brief 检查降落进度
     */
    void checkLandingProgress() {
        std::lock_guard<std::mutex> lock(pose_mutex_);

        // 计算与目标的3D距离
        double dx = current_pose_.pose.position.x - 0.0;
        double dy = current_pose_.pose.position.y - 0.0;
        double dz = current_pose_.pose.position.z - LANDING_HEIGHT;
        double distance_to_target = sqrt(dx*dx + dy*dy + dz*dz);

        // 输出降落进度
        static int progress_counter = 0;
        if (++progress_counter >= 20) {  // 每秒输出一次
            ROS_INFO("Landing progress: Distance to (0,0,%.1f): %.2f m, Current height: %.2f m",
                     LANDING_HEIGHT, distance_to_target, current_pose_.pose.position.z);
            progress_counter = 0;
        }

        // 当接近目标位置时，调用MAVROS降落服务
        if (distance_to_target < 0.15) {  // 15cm内
            ROS_INFO("Close to landing position, calling MAVROS land service");

            // 调用MAVROS降落服务
            mavros_msgs::CommandTOL land_cmd;
            land_cmd.request.altitude = 0;
            land_cmd.request.latitude = 0;
            land_cmd.request.longitude = 0;
            land_cmd.request.min_pitch = 0;
            land_cmd.request.yaw = 0;

            if (land_client_.call(land_cmd) && land_cmd.response.success) {
                ROS_INFO("MAVROS land command sent successfully");
            } else {
                ROS_WARN("MAVROS land command failed, vehicle should still land in OFFBOARD mode");
            }

            // 结束飞行任务
            is_flying_ = false;
            trajectory_received_ = false;
            flight_state_ = FlightState::IDLE;
            has_target_ = false;
            is_landing_ = false;

            ROS_INFO("Flight mission completed, landing initiated");
        }
    }

    /**
     * @brief 串口定时器回调
     */
    void serialTimerCallback(const ros::TimerEvent& event) {
        if (!serial_port_ || !serial_port_->isOpen()) {
            return;
        }

        // 读取串口数据
        if (serial_port_->available()) {
            std::vector<uint8_t> buffer;
            size_t bytes_read = serial_port_->read(buffer, serial_port_->available());

            // 逐字节解析
            for (size_t i = 0; i < bytes_read; i++) {
                comm_protocol_.parseByte(buffer[i]);
            }
        }
    }

    /**
     * @brief 控制定时器回调 - 连续发布当前目标位置
     */
    void controlTimerCallback(const ros::TimerEvent& event) {
        // 如果有目标且正在飞行，连续发布目标位置
        if (has_target_ && is_flying_) {
            std::lock_guard<std::mutex> lock(target_mutex_);

            // 更新时间戳
            current_target_.header.stamp = ros::Time::now();
            current_target_.header.frame_id = "map";

            // 发布目标位置到MAVROS
            cmd_pub_.publish(current_target_);

            // 调试信息（降低输出频率）
            static int counter = 0;
            if (++counter >= 40) {  // 每2秒输出一次（20Hz * 2s = 40）
                const char* state_str = "";
                switch (flight_state_.load()) {
                    case FlightState::ARMING:
                        state_str = "ARMING";
                        break;
                    case FlightState::TAKING_OFF:
                        state_str = "TAKING_OFF";
                        break;
                    case FlightState::FOLLOWING_TRAJECTORY:
                        state_str = "FOLLOWING_TRAJECTORY";
                        break;
                    case FlightState::WAYPOINT_HOVER_PRE:
                        state_str = "WAYPOINT_HOVER_PRE";
                        break;
                    case FlightState::WAYPOINT_HOVER_POST:
                        state_str = "WAYPOINT_HOVER_POST";
                        break;
                    case FlightState::MOVING_TO_GRID_CENTER:
                        state_str = "MOVING_TO_GRID_CENTER";
                        break;
                    case FlightState::SCANNING_GRID:
                        state_str = "SCANNING_GRID";
                        break;
                    case FlightState::SCAN_HOVERING:
                        state_str = "SCAN_HOVERING";
                        break;
                    case FlightState::LANDING:
                        state_str = "LANDING";
                        break;
                    default:
                        state_str = "IDLE";
                }

                // 获取已扫描网格数量
                size_t scanned_count = 0;
                {
                    std::lock_guard<std::mutex> lock(scanned_grids_mutex_);
                    scanned_count = scanned_grids_.size();
                }

                ROS_INFO("State: %s, Publishing target: (%.2f, %.2f, %.2f), Scanned grids: %zu",
                         state_str,
                         current_target_.pose.position.x,
                         current_target_.pose.position.y,
                         current_target_.pose.position.z,
                         scanned_count);
                counter = 0;
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ground_station_node");

    try {
        GroundStationNode node;
        ros::spin();
    } catch (std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
    }

    return 0;
}