#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include "stm32_comm/data_comm_protocol.hpp"
#include <memory>
#include <sstream>
#include <iomanip>
#include <thread>
#include <atomic>

class STM32Node {
public:
    STM32Node(ros::NodeHandle& nh, ros::NodeHandle& nh_private) 
        : nh_(nh), nh_private_(nh_private) {
        
        // 获取参数
        std::string port;
        int baudrate_int;  // 先用 int 接收参数
        nh_private_.param<std::string>("port", port, "/dev/ttyUSB0");
        nh_private_.param<int>("baudrate", baudrate_int, 115200);
        
        unsigned int baudrate = static_cast<unsigned int>(baudrate_int);  // 转换为 unsigned int
        
        // 初始化通信
        try {
            comm_ = std::make_unique<DataCommProtocol>(port, baudrate);
            comm_->setPacketCallback(
                std::bind(&STM32Node::packetHandler, this, 
                         std::placeholders::_1, std::placeholders::_2)
            );
            ROS_INFO("STM32 communication initialized on %s @ %u", 
                     port.c_str(), baudrate);
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to initialize communication: %s", e.what());
            throw;
        }
        
        // 订阅者示例 - LED控制
        led_sub_ = nh_.subscribe("/stm32/led_cmd", 10, 
                                &STM32Node::ledCallback, this);
        
        // 发布者示例 - 状态信息
        status_pub_ = nh_.advertise<std_msgs::String>("/stm32/status", 10);
        
        // 定时器示例 - 定期请求数据
       // timer_ = nh_.createTimer(ros::Duration(1.0), 
        //                        &STM32Node::timerCallback, this);
        
        // 启动接收线程
        receive_thread_ = std::thread(&STM32Node::receiveThread, this);
    }
    
    ~STM32Node() {
        running_ = false;
        if (receive_thread_.joinable()) {
            receive_thread_.join();
        }
    }
    
private:
    // 命令定义
    enum Commands {
        CMD_SENSOR_DATA = 0x01,
        CMD_LED_CONTROL = 0x02,
        CMD_MOTOR_CONTROL = 0x03,
        CMD_STATUS = 0x04
    };
    
    // 辅助函数：将数据转换为十六进制字符串
    std::string dataToHexString(const std::vector<uint8_t>& data) {
        std::stringstream ss;
        ss << std::hex << std::setfill('0');
        for (size_t i = 0; i < data.size(); ++i) {
            if (i > 0) ss << " ";
            ss << std::setw(2) << static_cast<int>(data[i]);
        }
        return ss.str();
    }
    
    void packetHandler(uint8_t cmd, const std::vector<uint8_t>& data) {
        // 显示接收到的数据包详情
        ROS_INFO("========== RECEIVED PACKET ==========");
        ROS_INFO("Command: 0x%02X", cmd);
        ROS_INFO("Data length: %zu bytes", data.size());
        if (!data.empty()) {
            ROS_INFO("Data content: [%s]", dataToHexString(data).c_str());
        }
        ROS_INFO("=====================================");
        
        // 处理接收到的数据包
        switch (cmd) {
            case CMD_SENSOR_DATA:
                ROS_INFO("Processing sensor data...");
                break;
                
            case CMD_STATUS:
                if (!data.empty()) {
                    std::string status(data.begin(), data.end());
                    std_msgs::String msg;
                    msg.data = status;
                    status_pub_.publish(msg);
                    ROS_INFO("Status message: %s", status.c_str());
                }
                break;
                
            default:
                ROS_WARN("Unknown command: 0x%02X", cmd);
                break;
        }
    }
    
    void ledCallback(const std_msgs::Bool::ConstPtr& msg) {
        // 发送LED控制命令
        std::vector<uint8_t> data = {msg->data ? uint8_t(0x01) : uint8_t(0x00)};
        
        ROS_INFO("LED control request: %s", msg->data ? "ON" : "OFF");
        
        // 使用新的 sendWithLog 函数，自动显示实际发送的数据包
        if (comm_->sendWithLog(CMD_LED_CONTROL, data, true)) {
            ROS_INFO("LED command sent successfully!");
        } else {
            ROS_ERROR("Failed to send LED command");
        }
    }
    
    void timerCallback(const ros::TimerEvent& event) {
        // 定期请求传感器数据
        std::vector<uint8_t> empty_data;  // 空数据
        
        ROS_INFO("Requesting sensor data...");
        
        // 使用新的 sendWithLog 函数
        if (!comm_->sendWithLog(CMD_SENSOR_DATA, empty_data, true)) {
            ROS_ERROR("Failed to request sensor data");
        }
    }
    
    void receiveThread() {
        ros::Rate rate(100);  // 100Hz
        
        while (running_ && ros::ok()) {
            // 处理接收数据
            size_t bytes = comm_->processReceive();
            if (bytes > 0) {
                ROS_DEBUG("Processed %zu bytes", bytes);
            }
            rate.sleep();
        }
    }
    
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    std::unique_ptr<DataCommProtocol> comm_;
    
    ros::Subscriber led_sub_;
    ros::Publisher status_pub_;
    ros::Timer timer_;
    
    std::thread receive_thread_;
    std::atomic<bool> running_{true};
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "stm32_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    try {
        STM32Node node(nh, nh_private);
        
        // 使用多线程spinner
        ros::MultiThreadedSpinner spinner(2);
        spinner.spin();
        
    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
        return 1;
    }
    
    return 0;
}