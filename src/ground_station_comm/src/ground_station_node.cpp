/**
  ******************************************************************************
  * @file    ground_station_node.cpp
  * @brief   地面站通信ROS节点
  * @version V2.0.0
  * @date    2025-07-31
  ******************************************************************************
  */

#include <ros/ros.h>
#include <serial/serial.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
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
    ros::Publisher cmd_pub_;
    ros::Publisher takeoff_land_pub_;
    ros::Publisher start_detect_pub_;
    ros::Timer serial_timer_;
    ros::Timer control_timer_;

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
        FOLLOWING_TRAJECTORY,    // 按照预定航线飞行
        SCANNING_GRID,          // 在网格内扫描动物
        MOVING_TO_NEXT_VECTOR   // 移动到下一个扫描向量
    };

    std::atomic<FlightState> flight_state_;
    std::atomic<int> current_waypoint_index_;
    std::atomic<bool> is_flying_;
    std::atomic<bool> trajectory_received_;
    geometry_msgs::PoseStamped current_pose_;
    std::mutex pose_mutex_;

    // 动物检测相关
    ground_station_comm::AnimalData latest_animal_data_;
    std::mutex animal_data_mutex_;
    std::atomic<bool> animal_data_received_;
    std::vector<geometry_msgs::Point> scan_vectors_;  // 扫描向量
    size_t current_vector_index_;
    geometry_msgs::Point scan_start_position_;        // 扫描起始位置
    geometry_msgs::Point current_scan_target_;        // 当前扫描目标位置

    // 已访问网格跟踪
    std::set<uint8_t> visited_grids_;
    std::mutex visited_grids_mutex_;
    uint8_t current_grid_number_;

    // 位置到达阈值
    double position_threshold_;
    static constexpr double GRID_BOUNDARY_THRESHOLD = 0.1;  // 10cm

    // 命令定义
    static constexpr uint8_t CMD_TRAJECTORY = 0x01;
    static constexpr uint8_t CMD_TAKEOFF = 0x02;
    static constexpr uint8_t CMD_ANIMAL_DATA = 0x03;  // 新增：动物数据命令
    static constexpr uint8_t TAKEOFF_DATA = 0x11;

public:
    GroundStationNode() :
        pnh_("~"),
        flight_state_(FlightState::IDLE),
        current_waypoint_index_(0),
        is_flying_(false),
        trajectory_received_(false),
        animal_data_received_(false),
        current_vector_index_(0),
        current_grid_number_(0) {

        // 读取参数
        pnh_.param<std::string>("serial_device", serial_device_, "/dev/ttyUSB0");
        pnh_.param<int>("serial_baudrate", serial_baudrate_, 115200);
        pnh_.param<double>("position_threshold", position_threshold_, 0.3);

        // 初始化串口
        initSerial();

        // 初始化通信协议
        initProtocol();

        // 初始化ROS接口
        pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 10,
                                  &GroundStationNode::poseCallback, this);
        animal_sub_ = nh_.subscribe("/animal", 10,
                                   &GroundStationNode::animalDataCallback, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/position_cmd", 10);
        takeoff_land_pub_ = nh_.advertise<std_msgs::Bool>("/px4ctrl/takeoff_land", 10);
        start_detect_pub_ = nh_.advertise<std_msgs::Bool>("/start_detect", 10);

        // 定时器
        serial_timer_ = nh_.createTimer(ros::Duration(0.01),
                                       &GroundStationNode::serialTimerCallback, this);
        control_timer_ = nh_.createTimer(ros::Duration(0.1),
                                        &GroundStationNode::controlTimerCallback, this);

        ROS_INFO("Ground Station Node initialized");
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

        {
            std::lock_guard<std::mutex> lock(waypoints_mutex_);
            waypoints_ = new_waypoints;
            current_waypoint_index_ = 0;
            trajectory_received_ = true;
        }

        // 如果已经在飞行，发布第一个航点
        if (is_flying_ && !waypoints_.empty()) {
            publishNextWaypoint();
        }
    }

    /**
     * @brief 处理起飞命令
     */
    void handleTakeoffCommand(const uint8_t* data, uint16_t len) {
        if (len != 1 || data[0] != TAKEOFF_DATA) {
            ROS_WARN("Invalid takeoff command data");
            return;
        }

        ROS_INFO("Received takeoff command");

        // 发布起飞命令
        std_msgs::Bool takeoff_msg;
        takeoff_msg.data = true;
        takeoff_land_pub_.publish(takeoff_msg);

        is_flying_ = true;
        flight_state_ = FlightState::FOLLOWING_TRAJECTORY;

        // 如果已有轨迹，发布第一个航点
        if (trajectory_received_ && !waypoints_.empty()) {
            ros::Duration(2.0).sleep(); // 等待起飞稳定
            publishNextWaypoint();
        }
    }

    /**
     * @brief 发布下一个航点
     */
    void publishNextWaypoint() {
        std::lock_guard<std::mutex> lock(waypoints_mutex_);

        size_t index = current_waypoint_index_.load();
        if (index >= waypoints_.size()) {
            ROS_WARN("No more waypoints to publish");
            return;
        }

        // 发布航点
        cmd_pub_.publish(waypoints_[index]);
        ROS_INFO("Published waypoint %zu/%zu: (%.2f, %.2f, %.2f)",
                index + 1, waypoints_.size(),
                waypoints_[index].pose.position.x,
                waypoints_[index].pose.position.y,
                waypoints_[index].pose.position.z);
    }

    /**
     * @brief 动物数据回调函数
     */
    void animalDataCallback(const ground_station_comm::AnimalData::ConstPtr& msg) {
        {
            std::lock_guard<std::mutex> lock(animal_data_mutex_);
            latest_animal_data_ = *msg;
            animal_data_received_ = true;
        }

        // 向地面站发送动物数据
        sendAnimalDataToGroundStation();

        // 发送 false 到 /start_detect
        std_msgs::Bool detect_msg;
        detect_msg.data = false;
        start_detect_pub_.publish(detect_msg);

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

        if (total_animals > 0) {
            ROS_INFO("Detected %d animals, starting grid scanning", total_animals);

            // 生成扫描向量
            generateScanVectors(total_animals);

            // 记录扫描起始位置
            {
                std::lock_guard<std::mutex> lock_pose(pose_mutex_);
                scan_start_position_ = current_pose_.pose.position;
            }

            // 切换到扫描状态
            flight_state_ = FlightState::SCANNING_GRID;
            current_vector_index_ = 0;

            // 发布第一个扫描目标
            publishNextScanTarget();
        } else {
            ROS_INFO("No animals detected, continuing trajectory");
            flight_state_ = FlightState::FOLLOWING_TRAJECTORY;
        }
    }

    /**
     * @brief 生成扫描向量
     */
    void generateScanVectors(uint8_t count) {
        scan_vectors_.clear();

        // 根据动物坐标生成扫描向量
        std::lock_guard<std::mutex> lock(animal_data_mutex_);

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
     * @brief 发布下一个扫描目标
     */
    void publishNextScanTarget() {
        if (current_vector_index_ >= scan_vectors_.size()) {
            ROS_INFO("Scan complete, returning to trajectory");
            flight_state_ = FlightState::FOLLOWING_TRAJECTORY;
            return;
        }

        // 计算目标位置
        geometry_msgs::PoseStamped target;
        target.header.frame_id = "map";
        target.header.stamp = ros::Time::now();

        // 从当前位置向扫描方向移动
        double scan_distance = 0.3;  // 向外扫描30cm
        current_scan_target_.x = scan_start_position_.x + scan_vectors_[current_vector_index_].x * scan_distance;
        current_scan_target_.y = scan_start_position_.y + scan_vectors_[current_vector_index_].y * scan_distance;
        current_scan_target_.z = scan_start_position_.z;  // 保持高度

        target.pose.position = current_scan_target_;
        target.pose.orientation.w = 1.0;

        cmd_pub_.publish(target);
        ROS_INFO("Published scan target %zu: (%.2f, %.2f, %.2f)",
                current_vector_index_ + 1,
                current_scan_target_.x,
                current_scan_target_.y,
                current_scan_target_.z);
    }

    /**
     * @brief 位置回调函数
     */
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            current_pose_ = *msg;
        }

        // 检查当前网格
        checkCurrentGrid();

        // 根据飞行状态进行相应处理
        switch (flight_state_.load()) {
            case FlightState::FOLLOWING_TRAJECTORY:
                if (is_flying_ && trajectory_received_) {
                    checkWaypointReached();
                }
                break;

            case FlightState::SCANNING_GRID:
                checkScanTargetReached();
                break;

            default:
                break;
        }
    }

    /**
     * @brief 检查当前网格
     */
    void checkCurrentGrid() {
        geometry_msgs::Point current_position;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            current_position = current_pose_.pose.position;
        }

        uint8_t grid_number = trajectory_mapper_.positionToGridNumber(current_position);

        if (grid_number != current_grid_number_ && grid_number != 0) {
            current_grid_number_ = grid_number;

            std::lock_guard<std::mutex> lock(visited_grids_mutex_);
            bool is_new_grid = visited_grids_.find(grid_number) == visited_grids_.end();

            // 发送检测信号
            std_msgs::Bool detect_msg;
            detect_msg.data = is_new_grid;
            start_detect_pub_.publish(detect_msg);

            if (is_new_grid) {
                visited_grids_.insert(grid_number);
                ROS_INFO("Entered new grid %d, sending start_detect=true", grid_number);
            } else {
                ROS_INFO("Re-entered grid %d, sending start_detect=false", grid_number);
            }
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

        // 检查是否接近网格边界
        bool near_boundary = trajectory_mapper_.isNearGridBoundary(
            current_pose_.pose.position, GRID_BOUNDARY_THRESHOLD);

        if (distance < 0.05 || near_boundary) {
            if (near_boundary) {
                ROS_INFO("Approaching grid boundary, moving to next scan vector");
            } else {
                ROS_INFO("Reached scan target %zu", current_vector_index_ + 1);
            }

            current_vector_index_++;

            if (current_vector_index_ < scan_vectors_.size()) {
                publishNextScanTarget();
            } else {
                ROS_INFO("All scan vectors completed, returning to trajectory");
                flight_state_ = FlightState::FOLLOWING_TRAJECTORY;
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

        // 计算与目标航点的距离
        double dx = current_pose_.pose.position.x - waypoints_[index].pose.position.x;
        double dy = current_pose_.pose.position.y - waypoints_[index].pose.position.y;
        double dz = current_pose_.pose.position.z - waypoints_[index].pose.position.z;
        double distance = sqrt(dx*dx + dy*dy + dz*dz);

        // 如果到达航点
        if (distance < position_threshold_) {
            ROS_INFO("Reached waypoint %zu/%zu", index + 1, waypoints_.size());

            // 检查是否到达倒数第二个点
            if (index == waypoints_.size() - 2) {
                ROS_INFO("Reached second to last waypoint, sending land command and stopping");

                // 发送降落命令
                std_msgs::Bool land_msg;
                land_msg.data = false;
                takeoff_land_pub_.publish(land_msg);

                // 结束飞行任务
                is_flying_ = false;
                trajectory_received_ = false;
                flight_state_ = FlightState::IDLE;
                ROS_INFO("Flight mission completed, landing initiated");
                return;
            }

            // 移动到下一个航点
            current_waypoint_index_++;

            // 发布下一个航点
            if (current_waypoint_index_ < waypoints_.size()) {
                // 解锁后发布，避免死锁
                lock_wp.~lock_guard();
                lock_pose.~lock_guard();
                publishNextWaypoint();
            } else {
                ROS_INFO("All waypoints completed");
                is_flying_ = false;
                flight_state_ = FlightState::IDLE;
            }
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
     * @brief 控制定时器回调
     */
    void controlTimerCallback(const ros::TimerEvent& event) {
        // 可以在这里添加额外的控制逻辑
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