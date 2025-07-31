#include <ros/ros.h>
#include <yolo11_ros/AnimalData.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <cmath>

class AnimalDetectionPublisher
{
private:
    ros::NodeHandle nh_;
    ros::Publisher animal_pub_;
    ros::Publisher start_detect_pub_;  // 新增：用于发送false信号
    ros::Subscriber start_detect_sub_; // 新增：订阅start_detect话题
    
    // 图像尺寸（可修改）
    const double IMAGE_WIDTH = 640.0;
    const double IMAGE_HEIGHT = 320.0;
    
    // 将像素坐标转换为单位向量
    geometry_msgs::Point pixelToUnitVector(double pixel_x, double pixel_y) {
        // 转换到相对于图像中心的坐标
        double x = pixel_x - IMAGE_WIDTH / 2.0;
        double y = pixel_y - IMAGE_HEIGHT / 2.0;
        
        // 归一化为单位向量
        double magnitude = std::sqrt(x*x + y*y);
        geometry_msgs::Point unit_vector;
        if (magnitude > 0.0) {
            unit_vector.x = x / magnitude;
            unit_vector.y = y / magnitude;
            unit_vector.z = 0.0;
        } else {
            unit_vector.x = 0.0;
            unit_vector.y = 0.0;
            unit_vector.z = 0.0;
        }
        return unit_vector;
    }
    
    // 新增：生成测试数据并发布
    void performDetectionAndPublish() {
        ROS_INFO("Received detection trigger - taking photo and processing...");
        
        // 模拟拍照延时
        ros::Duration(0.1).sleep();
        
        // 生成随机测试数据
        int peacock = rand() % 3;
        int wolf = rand() % 3;
        int monkey = rand() % 3;
        int tiger = rand() % 3;
        int elephant = rand() % 3;
        
        // 计算总数
        int total = peacock + wolf + monkey + tiger + elephant;
        
        // 生成对应数量的随机像素坐标
        std::vector<std::pair<double, double>> coords;
        for (int i = 0; i < total; i++) {
            double x = rand() % 640; // 0-639
            double y = rand() % 320; // 0-319
            coords.push_back({x, y});
        }
        
        // 发布检测结果到/animal_detection
        publish(peacock, wolf, monkey, tiger, elephant, coords);
        
        // 发布完成后，向/start_detect发送false
        std_msgs::Bool false_msg;
        false_msg.data = false;
        start_detect_pub_.publish(false_msg);
        ROS_INFO("Detection completed, sent false to /start_detect");
    }
    
    // 新增：start_detect话题的回调函数
    void startDetectCallback(const std_msgs::Bool::ConstPtr& msg) {
        if (msg->data == true) {
            ROS_INFO("Received start_detect = true, starting detection...");
            performDetectionAndPublish();
        } else {
            ROS_DEBUG("Received start_detect = false, ignoring...");
        }
    }

public:
    AnimalDetectionPublisher() {
        // 原有的发布器
        animal_pub_ = nh_.advertise<yolo11_ros::AnimalData>("/animal_detection", 10);
        
        // 新增：start_detect相关的发布器和订阅器
        start_detect_pub_ = nh_.advertise<std_msgs::Bool>("/start_detect", 10);
        start_detect_sub_ = nh_.subscribe("/start_detect", 10, &AnimalDetectionPublisher::startDetectCallback, this);
        
        ROS_INFO("Animal Detection Publisher initialized");
        ROS_INFO("Subscribed to /start_detect topic");
        ROS_INFO("Will publish to /animal_detection and /start_detect");
        
        srand(time(nullptr)); // 初始化随机数种子
    }
    
    // 核心发布函数
    // animal_counts: [peacock, wolf, monkey, tiger, elephant] 的数量数组
    // pixel_coords: 像素坐标对的vector [(x1,y1), (x2,y2), ...]
    void publishDetection(const std::vector<int>& animal_counts,
                         const std::vector<std::pair<double, double>>& pixel_coords) {
        yolo11_ros::AnimalData msg;
        
        // 设置时间戳和坐标系
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "camera_frame";
        
        // 设置各动物数量
        msg.peacock = animal_counts[0];
        msg.wolf = animal_counts[1];
        msg.monkey = animal_counts[2];
        msg.tiger = animal_counts[3];
        msg.elephant = animal_counts[4];
        
        // 转换像素坐标为单位向量并添加到消息
        for (const auto& coord : pixel_coords) {
            geometry_msgs::Point unit_vec = pixelToUnitVector(coord.first, coord.second);
            msg.coordinates.push_back(unit_vec);
        }
        
        // 发布消息
        animal_pub_.publish(msg);
        ROS_INFO("Published: P=%d W=%d M=%d T=%d E=%d, Total=%zu",
                msg.peacock, msg.wolf, msg.monkey, msg.tiger, msg.elephant,
                msg.coordinates.size());
    }
    
    // 简化版本：直接传入所有参数
    void publish(int peacock, int wolf, int monkey, int tiger, int elephant,
                const std::vector<std::pair<double, double>>& pixel_coords) {
        std::vector<int> counts = {peacock, wolf, monkey, tiger, elephant};
        publishDetection(counts, pixel_coords);
    }
};

// 修改后的主函数
int main(int argc, char** argv)
{
    ros::init(argc, argv, "yolo11_ros_node");
    
    AnimalDetectionPublisher publisher;
    
    ROS_INFO("YOLO11 ROS Node started. Waiting for /start_detect messages...");
    ROS_INFO("Send 'rostopic pub /start_detect std_msgs/Bool \"data: true\"' to trigger detection");
    
    // 使用ros::spin()替代原来的循环，等待回调
    ros::spin();
    
    return 0;
}

/* 使用示例：在你的YOLO检测代码中
// 创建发布器
AnimalDetectionPublisher publisher;
// 假设YOLO检测到：1只孔雀在(100,150)，2只狼在(200,100)和(300,200)
std::vector<std::pair<double, double>> detected_coords = {
    {100, 150}, // 孔雀
    {200, 100}, // 狼1
    {300, 200}  // 狼2
};
// 发布检测结果
publisher.publish(1, 2, 0, 0, 0, detected_coords);
*/