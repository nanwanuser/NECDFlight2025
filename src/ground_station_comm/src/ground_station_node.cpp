/**
  ******************************************************************************
  * @file    ground_station_node.cpp
  * @brief   地面站通信ROS节点
  * @version V1.0.0
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

#include "ground_station_comm/data_communication_pkg.hpp"
#include "ground_station_comm/trajectory_mapper.hpp"

class GroundStationNode {
private:
    // ROS相关
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber pose_sub_;
    ros::Publisher cmd_pub_;
    ros::Publisher takeoff_land_pub_;
    ros::Timer serial_timer_;

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
    std::atomic<int> current_waypoint_index_;
    std::atomic<bool> is_flying_;
    std::atomic<bool> trajectory_received_;
    geometry_msgs::PoseStamped current_pose_;
    std::mutex pose_mutex_;

    // 位置到达阈值
    double position_threshold_;

    // 命令定义
    static constexpr uint8_t CMD_TRAJECTORY = 0x01;
    static constexpr uint8_t CMD_TAKEOFF = 0x02;
    static constexpr uint8_t TAKEOFF_DATA = 0x11;

public:
    GroundStationNode() :
        pnh_("~"),
        current_waypoint_index_(0),
        is_flying_(false),
        trajectory_received_(false) {

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
        cmd_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/position_cmd", 10);
        takeoff_land_pub_ = nh_.advertise<std_msgs::Bool>("/px4ctrl/takeoff_land", 10);

        // 定时器读取串口数据
        serial_timer_ = nh_.createTimer(ros::Duration(0.01),
                                       &GroundStationNode::serialTimerCallback, this);

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
     * @brief 位置回调函数
     */
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            current_pose_ = *msg;
        }

        // 检查是否到达当前航点
        if (is_flying_ && trajectory_received_) {
            checkWaypointReached();
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