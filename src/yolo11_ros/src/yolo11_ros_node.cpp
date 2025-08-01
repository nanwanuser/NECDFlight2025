#include <ros/ros.h>
#include <yolo11_ros/AnimalData.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <vector>
#include <cmath>
#include <string>
#include <chrono>
#include <iostream>
#include <map>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

// YOLO11 RKNN相关头文件
#include "yolo11.h"

class YOLO11ROSNode
{
private:
    ros::NodeHandle nh_;
    ros::Publisher animal_pub_;
    ros::Publisher start_detect_pub_;
    ros::Publisher image_pub_;
    ros::Subscriber start_detect_sub_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_transport_pub_;
    
    // YOLO11 RKNN相关
    rknn_app_context_t rknn_app_ctx_;
    std::string model_path_;
    
    // 摄像头相关
    cv::VideoCapture cap_;
    int camera_index_;
    bool camera_opened_;
    bool should_publish_data_;  // 新增：是否应该发布数据的标志
    
    // 图像尺寸 - 摄像头实际支持的分辨率
    const double IMAGE_WIDTH = 640.0;
    const double IMAGE_HEIGHT = 480.0;  // 🔥 改回480，匹配摄像头支持的分辨率
    
    // 类别映射 - 根据你的5种动物调整
    std::map<int, std::string> class_to_animal_ = {
        {0, "d_elephant"},   // 类别0对应大象
        {1, "h_monkey"},     // 类别1对应猴子
        {2, "k_kongque"},    // 类别2对应孔雀
        {3, "l_tiger"},      // 类别3对应老虎
        {4, "ll_wolf"}       // 类别4对应狼
    };

    // 将像素坐标转换为单位向量
    geometry_msgs::Point pixelToUnitVector(double pixel_x, double pixel_y) {
        double x = pixel_x - IMAGE_WIDTH / 2.0;
        double y = IMAGE_HEIGHT / 2.0 - pixel_y;
        
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
    
    // 🔥 新增：在图像上绘制坐标系标注
    void drawCoordinateSystem(cv::Mat& mat, bool refresh_flag = false) {
        // 计算图像中心点
        int center_x = static_cast<int>(IMAGE_WIDTH / 2.0);
        int center_y = static_cast<int>(IMAGE_HEIGHT / 2.0);
        
        // 设置坐标轴长度
        int axis_length = 80;
        
        // 颜色定义
        cv::Scalar center_color = cv::Scalar(0, 0, 255);     // 红色中心点
        cv::Scalar x_axis_color = cv::Scalar(0, 255, 0);     // 绿色X轴
        cv::Scalar y_axis_color = cv::Scalar(255, 0, 0);     // 蓝色Y轴
        cv::Scalar text_color = cv::Scalar(255, 255, 255);   // 白色文字
        cv::Scalar bg_color = cv::Scalar(0, 0, 0);           // 黑色背景
        
        // 如果刷新标志为true，绘制更突出的坐标系
        if (refresh_flag) {
            center_color = cv::Scalar(0, 255, 255);  // 黄色中心点
            axis_length = 100;  // 更长的坐标轴
        }
        
        // 绘制中心点
        cv::circle(mat, cv::Point(center_x, center_y), 5, center_color, -1);
        
        // 绘制X轴 (水平向右为正)
        cv::arrowedLine(mat, 
                       cv::Point(center_x, center_y), 
                       cv::Point(center_x + axis_length, center_y),
                       x_axis_color, 2, 8, 0, 0.3);
        
        // 绘制Y轴 (垂直向上为正)
        cv::arrowedLine(mat, 
                       cv::Point(center_x, center_y), 
                       cv::Point(center_x, center_y - axis_length),
                       y_axis_color, 2, 8, 0, 0.3);
        
        // 绘制坐标轴标签
        cv::putText(mat, "+X", cv::Point(center_x + axis_length + 5, center_y + 5),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, x_axis_color, 1);
        cv::putText(mat, "+Y", cv::Point(center_x + 5, center_y - axis_length - 5),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, y_axis_color, 1);
        
        // 绘制象限标识
        cv::putText(mat, "(-1,+1)", cv::Point(center_x - 60, center_y - 40),
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, text_color, 1);
        cv::putText(mat, "(+1,+1)", cv::Point(center_x + 20, center_y - 40),
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, text_color, 1);
        cv::putText(mat, "(-1,-1)", cv::Point(center_x - 60, center_y + 50),
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, text_color, 1);
        cv::putText(mat, "(+1,-1)", cv::Point(center_x + 20, center_y + 50),
                   cv::FONT_HERSHEY_SIMPLEX, 0.4, text_color, 1);
        
        // 绘制中心坐标
        cv::putText(mat, "(0,0)", cv::Point(center_x + 8, center_y - 8),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, center_color, 1);
        
        // 如果刷新标志为true，添加额外的刷新提示
        if (refresh_flag) {
            // 绘制半透明背景
            cv::Rect text_bg(10, 10, 200, 30);
            cv::rectangle(mat, text_bg, bg_color, -1);
            
            cv::putText(mat, "COORDINATE REFRESHED!", cv::Point(15, 30),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
        }
    }
    
    // 🔥 新增：绘制检测到的动物的单位向量
    void drawAnimalVectors(cv::Mat& mat, const std::vector<std::pair<double, double>>& coords) {
        int center_x = static_cast<int>(IMAGE_WIDTH / 2.0);
        int center_y = static_cast<int>(IMAGE_HEIGHT / 2.0);
        
        for (size_t i = 0; i < coords.size(); i++) {
            double pixel_x = coords[i].first;
            double pixel_y = coords[i].second;
            
            // 计算单位向量
            geometry_msgs::Point unit_vec = pixelToUnitVector(pixel_x, pixel_y);
            
            // 计算在图像上的显示位置 (缩放单位向量用于显示)
            int vec_end_x = center_x + static_cast<int>(unit_vec.x * 60);
            int vec_end_y = center_y - static_cast<int>(unit_vec.y * 60);  // 注意Y轴翻转
            
            // 绘制从中心到目标的向量
            cv::arrowedLine(mat,
                           cv::Point(center_x, center_y),
                           cv::Point(vec_end_x, vec_end_y),
                           cv::Scalar(255, 255, 0), 2, 8, 0, 0.4);  // 青色箭头
            
            // 在向量末端显示坐标值
            char coord_text[64];
            sprintf(coord_text, "(%.2f,%.2f)", unit_vec.x, unit_vec.y);
            cv::putText(mat, coord_text,
                       cv::Point(vec_end_x + 5, vec_end_y - 5),
                       cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 0), 1);
        }
    }
    
    // 将OpenCV Mat转换为image_buffer_t
    int matToImageBuffer(cv::Mat& mat, image_buffer_t* img_buf) {
        if (mat.empty()) {
            return -1;
        }
        
        cv::Mat rgb_mat;
        cv::cvtColor(mat, rgb_mat, cv::COLOR_BGR2RGB);
        
        img_buf->width = rgb_mat.cols;
        img_buf->height = rgb_mat.rows;
        img_buf->format = IMAGE_FORMAT_RGB888;
        img_buf->size = rgb_mat.cols * rgb_mat.rows * 3;
        img_buf->virt_addr = (unsigned char*)malloc(img_buf->size);
        
        if (img_buf->virt_addr == NULL) {
            return -1;
        }
        
        memcpy(img_buf->virt_addr, rgb_mat.data, img_buf->size);
        return 0;
    }
    
    // 在OpenCV Mat上绘制检测框
    void drawObjectsOnMat(cv::Mat& mat, object_detect_result_list* od_results) {
        char text[256];
        
        for (int i = 0; i < od_results->count; i++) {
            object_detect_result* det_result = &(od_results->results[i]);
            
            // 绘制矩形框
            cv::rectangle(mat, 
                         cv::Point(det_result->box.left, det_result->box.top),
                         cv::Point(det_result->box.right, det_result->box.bottom),
                         cv::Scalar(0, 255, 0), 2);
            
            // 绘制标签和置信度
            sprintf(text, "%s %.1f%%", coco_cls_to_name(det_result->cls_id), 
                    det_result->prop * 100);
            
            cv::putText(mat, text,
                       cv::Point(det_result->box.left, det_result->box.top - 5),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
            
            ROS_INFO("Detected: %s @ (%d %d %d %d) %.3f", 
                   coco_cls_to_name(det_result->cls_id),
                   det_result->box.left, det_result->box.top,
                   det_result->box.right, det_result->box.bottom,
                   det_result->prop);
        }
    }
    
    // 统计检测结果中的动物数量
    void countAnimals(object_detect_result_list* od_results, 
                     int& peacock, int& wolf, int& monkey, int& tiger, int& elephant,
                     std::vector<std::pair<double, double>>& coords) {
        peacock = wolf = monkey = tiger = elephant = 0;
        coords.clear();
        
        for (int i = 0; i < od_results->count; i++) {
            object_detect_result* det_result = &(od_results->results[i]);
            
            // 计算中心点坐标
            double center_x = (det_result->box.left + det_result->box.right) / 2.0;
            double center_y = (det_result->box.top + det_result->box.bottom) / 2.0;
            coords.push_back({center_x, center_y});
            
            // 根据类别ID统计数量
            int cls_id = det_result->cls_id;
            std::string class_name = coco_cls_to_name(cls_id);
            
            if (cls_id == 0 || class_name.find("d_elephant") != std::string::npos) {
                elephant++;
            } else if (cls_id == 1 || class_name.find("h_monkey") != std::string::npos) {
                monkey++;
            } else if (cls_id == 2 || class_name.find("k_kongque") != std::string::npos) {
                peacock++;
            } else if (cls_id == 3 || class_name.find("l_tiger") != std::string::npos) {
                tiger++;
            } else if (cls_id == 4 || class_name.find("ll_wolf") != std::string::npos) {
                wolf++;
            }
            
            // 调试输出
            ROS_DEBUG("Animal detected: cls_id=%d, name=%s, count: E=%d H=%d K=%d L=%d LL=%d", 
                     cls_id, class_name.c_str(), elephant, monkey, peacock, tiger, wolf);
        }
    }
    
    // 连续检测并显示
    void continuousDetection() {
        if (!camera_opened_ || !cap_.isOpened()) {
            ROS_ERROR("Camera is not opened!");
            return;
        }
        
        cv::Mat frame;
        cap_ >> frame;
        if (frame.empty()) {
            return;
        }
        
        // 🔥 获取实际帧的尺寸
        int actual_width = frame.cols;
        int actual_height = frame.rows;
        
        // 转换为image_buffer_t格式
        image_buffer_t src_image;
        memset(&src_image, 0, sizeof(image_buffer_t));
        
        int ret = matToImageBuffer(frame, &src_image);
        if (ret != 0) {
            return;
        }
        
        // 执行YOLO推理
        object_detect_result_list od_results;
        ret = inference_yolo11_model(&rknn_app_ctx_, &src_image, &od_results);
        
        std::vector<std::pair<double, double>> coords;
        
        if (ret == 0) {
            // 🔥 调整检测框坐标到实际图像尺寸
            adjustDetectionBoxes(&od_results, actual_width, actual_height);
            
            // 在图像上绘制检测结果
            drawObjectsOnMat(frame, &od_results);
            
            // 如果收到发布标志，则发布数据并获取坐标
            if (should_publish_data_) {
                // 统计动物数量并发布
                int peacock, wolf, monkey, tiger, elephant;
                countAnimals(&od_results, peacock, wolf, monkey, tiger, elephant, coords);
                
                publishAnimalData(peacock, wolf, monkey, tiger, elephant, coords);
                publishImage(frame);
                
                // 发布完成信号
                std_msgs::Bool false_msg;
                false_msg.data = false;
                start_detect_pub_.publish(false_msg);
                ROS_INFO("Detection data published, sent false to /start_detect");
                
                should_publish_data_ = false;  // 重置标志
            } else {
                // 如果没有发布标志，仍然计算坐标用于显示
                int peacock, wolf, monkey, tiger, elephant;
                countAnimals(&od_results, peacock, wolf, monkey, tiger, elephant, coords);
            }
        }
        
        // 🔥 始终绘制坐标系，如果刷新标志为true则高亮显示
        drawCoordinateSystem(frame, should_publish_data_);
        
        // 🔥 绘制检测到的动物的单位向量
        if (!coords.empty()) {
            drawAnimalVectors(frame, coords);
        }
        
        // 显示结果窗口
        cv::imshow("YOLO11 Detection Results", frame);
        cv::waitKey(1);
        
        // 释放内存
        free(src_image.virt_addr);
    }
    
    // 🔥 新增：调整检测框坐标
    void adjustDetectionBoxes(object_detect_result_list* od_results, int actual_width, int actual_height) {
        // 如果实际尺寸与设定尺寸不同，需要缩放坐标
        float scale_x = (float)actual_width / IMAGE_WIDTH;
        float scale_y = (float)actual_height / IMAGE_HEIGHT;
        
        for (int i = 0; i < od_results->count; i++) {
            object_detect_result* det_result = &(od_results->results[i]);
            
            // 缩放检测框坐标
            det_result->box.left = (int)(det_result->box.left * scale_x);
            det_result->box.top = (int)(det_result->box.top * scale_y);
            det_result->box.right = (int)(det_result->box.right * scale_x);
            det_result->box.bottom = (int)(det_result->box.bottom * scale_y);
            
            // 确保坐标在图像范围内
            det_result->box.left = std::max(0, std::min(det_result->box.left, actual_width - 1));
            det_result->box.top = std::max(0, std::min(det_result->box.top, actual_height - 1));
            det_result->box.right = std::max(0, std::min(det_result->box.right, actual_width - 1));
            det_result->box.bottom = std::max(0, std::min(det_result->box.bottom, actual_height - 1));
        }
        
        // 调试输出
        if (od_results->count > 0) {
            ROS_DEBUG("Frame size: %dx%d, Scale: %.2fx%.2f", 
                     actual_width, actual_height, scale_x, scale_y);
        }
    }
    
    // 执行检测并发布结果（保留原函数但简化）
    void performDetectionAndPublish() {
        ROS_INFO("Received detection trigger - will publish data on next detection");
        should_publish_data_ = true;  // 设置发布标志
    }
    
    // 发布图像
    void publishImage(const cv::Mat& image) {
        try {
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
            msg->header.stamp = ros::Time::now();
            msg->header.frame_id = "camera_frame";
            image_transport_pub_.publish(msg);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }
    
    // 发布动物检测数据
    void publishAnimalData(int peacock, int wolf, int monkey, int tiger, int elephant,
                          const std::vector<std::pair<double, double>>& pixel_coords) {
        yolo11_ros::AnimalData msg;
        
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "camera_frame";
        
        msg.peacock = peacock;
        msg.wolf = wolf;
        msg.monkey = monkey;
        msg.tiger = tiger;
        msg.elephant = elephant;
        
        // 转换像素坐标为单位向量
        for (const auto& coord : pixel_coords) {
            geometry_msgs::Point unit_vec = pixelToUnitVector(coord.first, coord.second);
            msg.coordinates.push_back(unit_vec);
        }
        
        animal_pub_.publish(msg);
        ROS_INFO("Published animal data: Elephant=%d Monkey=%d Peacock=%d Tiger=%d Wolf=%d, Total=%zu",
                msg.elephant, msg.monkey, msg.peacock, msg.tiger, msg.wolf,
                msg.coordinates.size());
    }
    
    // start_detect话题回调函数
    void startDetectCallback(const std_msgs::Bool::ConstPtr& msg) {
        if (msg->data == true) {
            ROS_INFO("Received start_detect = true, starting detection...");
            performDetectionAndPublish();
        }
    }

public:
    YOLO11ROSNode() : it_(nh_), camera_opened_(false), should_publish_data_(false) {
        // 获取参数
        nh_.param<std::string>("model_path", model_path_, 
            "/home/orangepi/NECDFlight2025/src/yolo11_ros/model/fix_gaoqing.rknn");
        nh_.param<int>("camera_index", camera_index_, 0);
        
        // 初始化发布器和订阅器
        animal_pub_ = nh_.advertise<yolo11_ros::AnimalData>("/animal_detection", 10);
        start_detect_pub_ = nh_.advertise<std_msgs::Bool>("/start_detect", 10);
        start_detect_sub_ = nh_.subscribe("/start_detect", 10, 
            &YOLO11ROSNode::startDetectCallback, this);
        image_transport_pub_ = it_.advertise("/yolo11/detection_image", 1);
        
        // 初始化YOLO11模型
        if (initYOLOModel() != 0) {
            ROS_FATAL("Failed to initialize YOLO11 model!");
            ros::shutdown();
            return;
        }
        
        // 初始化摄像头
        if (initCamera() != 0) {
            ROS_FATAL("Failed to initialize camera!");
            ros::shutdown();
            return;
        }
        
        ROS_INFO("YOLO11 ROS Node initialized successfully");
        ROS_INFO("Model path: %s", model_path_.c_str());
        ROS_INFO("Camera index: %d", camera_index_);
        ROS_INFO("Animal classes: 0=d_elephant, 1=h_monkey, 2=k_kongque, 3=l_tiger, 4=ll_wolf");
        ROS_INFO("Subscribed to /start_detect topic");
        ROS_INFO("Publishing to /animal_detection and /yolo11/detection_image");
    }
    
    ~YOLO11ROSNode() {
        if (camera_opened_) {
            cap_.release();
        }
        cv::destroyAllWindows();
        
        // 释放YOLO模型资源
        release_yolo11_model(&rknn_app_ctx_);
        deinit_post_process();
    }
    
    // 初始化YOLO模型
    int initYOLOModel() {
        memset(&rknn_app_ctx_, 0, sizeof(rknn_app_context_t));
        
        // 初始化后处理
        init_post_process();
        
        // 初始化YOLO模型
        int ret = init_yolo11_model(model_path_.c_str(), &rknn_app_ctx_);
        if (ret != 0) {
            ROS_ERROR("init_yolo11_model fail! ret=%d model_path=%s", ret, model_path_.c_str());
            return -1;
        }
        
        ROS_INFO("YOLO11 model initialized successfully");
        return 0;
    }
    
    // 初始化摄像头
    int initCamera() {
        cap_.open(camera_index_);
        if (!cap_.isOpened()) {
            ROS_ERROR("Failed to open camera %d", camera_index_);
            return -1;
        }
        
        // 设置摄像头分辨率
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, IMAGE_WIDTH);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, IMAGE_HEIGHT);
        
        // 🔥 获取实际设置的分辨率
        double actual_width = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
        double actual_height = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
        
        camera_opened_ = true;
        ROS_INFO("Camera opened successfully");
        ROS_INFO("Requested resolution: %.0fx%.0f", IMAGE_WIDTH, IMAGE_HEIGHT);
        ROS_INFO("Actual resolution: %.0fx%.0f", actual_width, actual_height);
        
        if (actual_width != IMAGE_WIDTH || actual_height != IMAGE_HEIGHT) {
            ROS_WARN("Camera resolution mismatch! This may cause detection box offset.");
            ROS_WARN("Consider updating IMAGE_WIDTH and IMAGE_HEIGHT constants.");
        }
        
        return 0;
    }
    
    // 主循环
    void spin() {
        ros::Rate loop_rate(30);  // 30 FPS
        while (ros::ok()) {
            continuousDetection();  // 连续检测
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "yolo11_ros_node");
    
    try {
        YOLO11ROSNode node;
        ROS_INFO("YOLO11 ROS Node started. Waiting for /start_detect messages...");
        ROS_INFO("Send 'rostopic pub /start_detect std_msgs/Bool \"data: true\"' to trigger detection");
        node.spin();
    } catch (const std::exception& e) {
        ROS_FATAL("Exception in YOLO11 ROS Node: %s", e.what());
        return -1;
    }
    
    return 0;
}
