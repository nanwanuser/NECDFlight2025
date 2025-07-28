#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <zbar.h>
#include <vector>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <unistd.h>  
#include <sys/stat.h>

class QRLogisticsDetector {
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport image_transport_;
    image_transport::Publisher image_pub_;
    ros::Subscriber pose_sub_;
    
    cv::VideoCapture cap_;
    std::string camera_device_;
    bool show_image_;
    
    zbar::ImageScanner scanner_;
    
    geometry_msgs::PoseStamped current_pose_;
    
    // 用于控制相同内容的输出频率
    std::string last_qr_content_;
    ros::Time last_output_time_;

    // 核心方法
    std::string detectQR(cv::Mat &frame);
    bool processQRContent(const std::string& content);
    void drawZBarResult(cv::Mat &frame, zbar::SymbolIterator symbol);
    void publishImage(const cv::Mat& frame);
    bool initializeCamera();
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    
    // 摄像头相关方法
    std::vector<std::string> scanAvailableCameras();
    bool tryOpenCamera(const std::string& device, int backend = -1);
    bool initializeCameraWithRetry();
    bool isDeviceAccessible(const std::string& device);
    void killCameraProcesses();

public:
    QRLogisticsDetector(ros::NodeHandle &nh);
    void run();
};

QRLogisticsDetector::QRLogisticsDetector(ros::NodeHandle &nh) : 
    nh_(nh), image_transport_(nh)
{
    scanner_.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 1);
    nh_.param<std::string>("camera_device", camera_device_, "/dev/video2");
    nh_.param<bool>("show_image", show_image_, true);
    
    pose_sub_ = nh_.subscribe("/mavros/local_position/pose", 1, 
                            &QRLogisticsDetector::poseCallback, this);
    image_pub_ = image_transport_.advertise("camera/image", 1);
    
    nh_.setParam("/qr_detected", false);
    
    // 初始化时间控制变量
    last_qr_content_ = "";
    last_output_time_ = ros::Time(0);
    
    ROS_INFO("QR检测器初始化完成");
}

std::vector<std::string> QRLogisticsDetector::scanAvailableCameras() {
    std::vector<std::string> available_cameras;
    
    // 扫描 /dev/video* 设备
    for (int i = 0; i < 10; i++) {
        std::string device = "/dev/video" + std::to_string(i);
        if (isDeviceAccessible(device)) {
            available_cameras.push_back(device);
        }
    }
    
    // 数字索引
    for (int i = 0; i < 5; i++) {
        available_cameras.push_back(std::to_string(i));
    }
    
    return available_cameras;
}

bool QRLogisticsDetector::isDeviceAccessible(const std::string& device) {
    struct stat buffer;
    return (stat(device.c_str(), &buffer) == 0);
}

void QRLogisticsDetector::killCameraProcesses() {
    ROS_WARN("释放摄像头资源...");
    system("pkill -f cheese 2>/dev/null");
    system("pkill -f guvcview 2>/dev/null");
    system("pkill -f vlc 2>/dev/null");
    usleep(1000000);
}

bool QRLogisticsDetector::tryOpenCamera(const std::string& device, int backend) {
    cv::VideoCapture test_cap;
    
    try {
        if (device.find("video") != std::string::npos) {
            if (backend >= 0) {
                test_cap.open(device, backend);
            } else {
                test_cap.open(device, cv::CAP_V4L2);
            }
        } else {
            int device_id = std::stoi(device);
            if (backend >= 0) {
                test_cap.open(device_id, backend);
            } else {
                test_cap.open(device_id);
            }
        }
        
        if (test_cap.isOpened()) {
            cv::Mat test_frame;
            for (int i = 0; i < 5; i++) {
                test_cap >> test_frame;
                if (!test_frame.empty()) {
                    cap_ = std::move(test_cap);
                    camera_device_ = device;
                    return true;
                }
                usleep(100000);
            }
            test_cap.release();
        }
    } catch (const std::exception& e) {
        ROS_DEBUG("设备 %s 打开失败: %s", device.c_str(), e.what());
    }
    
    return false;
}

bool QRLogisticsDetector::initializeCameraWithRetry() {
    const int max_retries = 3;
    const int retry_delay_ms = 2000;
    
    ROS_INFO("开始摄像头初始化: %s", camera_device_.c_str());
    
    for (int retry = 0; retry < max_retries; retry++) {
        if (retry > 0) {
            ROS_WARN("摄像头重试 %d/%d", retry + 1, max_retries);
            killCameraProcesses();
            usleep(retry_delay_ms * 1000);
        }
        
        std::vector<int> backends = {cv::CAP_V4L2, cv::CAP_ANY, cv::CAP_GSTREAMER};
        
        // 尝试用户指定设备
        for (int backend : backends) {
            ROS_INFO("尝试打开 %s (backend: %d)", camera_device_.c_str(), backend);
            if (tryOpenCamera(camera_device_, backend)) {
                ROS_INFO("成功打开摄像头: %s", camera_device_.c_str());
                return true;
            }
        }
        
        // 扫描所有可用摄像头
        ROS_WARN("扫描可用摄像头...");
        std::vector<std::string> available_cameras = scanAvailableCameras();
        ROS_INFO("发现 %zu 个设备", available_cameras.size());
        
        for (const std::string& device : available_cameras) {
            if (device == camera_device_) continue;
            
            for (int backend : backends) {
                ROS_INFO("尝试设备: %s (backend: %d)", device.c_str(), backend);
                if (tryOpenCamera(device, backend)) {
                    ROS_INFO("成功打开摄像头: %s", device.c_str());
                    return true;
                }
            }
        }
    }
    
    return false;
}

bool QRLogisticsDetector::initializeCamera() {
    if (!initializeCameraWithRetry()) {
        ROS_ERROR("摄像头初始化失败");
        system("ls -la /dev/video* 2>/dev/null || echo '未找到摄像头设备'");
        return false;
    }
    
    // 设置摄像头参数
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap_.set(cv::CAP_PROP_FPS, 30);
    cap_.set(cv::CAP_PROP_AUTOFOCUS, 0);
    
    ROS_INFO("摄像头初始化成功: %s", camera_device_.c_str());
    ROS_INFO("分辨率: %dx%d, FPS: %.1f", 
             (int)cap_.get(cv::CAP_PROP_FRAME_WIDTH),
             (int)cap_.get(cv::CAP_PROP_FRAME_HEIGHT),
             cap_.get(cv::CAP_PROP_FPS));
    
    return true;
}

void QRLogisticsDetector::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pose_ = *msg;
}

void QRLogisticsDetector::run() {
    if (!initializeCamera()) {
        ROS_ERROR("摄像头初始化失败，程序退出");
        return;
    }

    ros::Rate loop_rate(30);
    cv::Mat frame;
    int empty_frame_count = 0;
    const int max_empty_frames = 150; // 5秒
    
    while (ros::ok()) {
        cap_ >> frame;
        if (frame.empty()) {
            empty_frame_count++;
            if (empty_frame_count >= max_empty_frames) {
                ROS_WARN("重新初始化摄像头...");
                cap_.release();
                if (initializeCamera()) {
                    empty_frame_count = 0;
                    ROS_INFO("摄像头重新初始化成功");
                } else {
                    ROS_ERROR("摄像头重新初始化失败");
                    break;
                }
            } else {
                ROS_WARN_THROTTLE(5, "空帧 (%d/%d)", empty_frame_count, max_empty_frames);
            }
            continue;
        }
        
        empty_frame_count = 0;

        std::string qr_content = detectQR(frame);
        
        if (!qr_content.empty()) {
            ros::Time current_time = ros::Time::now();
            bool should_output = false;
            
            // 如果是不同内容，或者相同内容但超过1秒，则输出
            if (qr_content != last_qr_content_) {
                should_output = true;
                last_qr_content_ = qr_content;
            } else if ((current_time - last_output_time_).toSec() >= 1.0) {
                should_output = true;
            }
            
            if (should_output) {
                last_output_time_ = current_time;
                nh_.setParam("/qr_detected", true);
                nh_.setParam("/qr_content", qr_content);
                
                if (processQRContent(qr_content)) {
                    ROS_INFO("二维码处理成功: %s", qr_content.c_str());
                } else {
                    ROS_WARN("二维码处理失败: %s", qr_content.c_str());
                }
            }
        }

        publishImage(frame);
        
        if (show_image_) {
            cv::imshow("QR检测", frame);
            if (cv::waitKey(30) == 27) break;
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    cap_.release();
    if (show_image_) cv::destroyAllWindows();
}

std::string QRLogisticsDetector::detectQR(cv::Mat &frame) {
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, cv::Size(3, 3), 0);
    cv::threshold(gray, gray, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    
    zbar::Image image(gray.cols, gray.rows, "Y800", gray.data, gray.cols * gray.rows);
    int scan_result = scanner_.scan(image);
    
    if (scan_result > 0) {
        for (zbar::Image::SymbolIterator symbol = image.symbol_begin(); 
             symbol != image.symbol_end(); ++symbol) {
            std::string content = symbol->get_data();
            
            // 基本内容检查（可根据实际需要修改）
            if (!content.empty()) {
                drawZBarResult(frame, symbol);
                return content;
            }
        }
    }
    return "";
}

bool QRLogisticsDetector::processQRContent(const std::string& content) {
    ROS_INFO_STREAM("检测到二维码: " << content);
    
    // TODO: 根据实际赛题要求在这里添加具体的处理逻辑
    // 例如：
    // - 解析任务格式
    // - 设置航点参数
    // - 触发特定行为
    
    // 示例：简单地将内容设置为ROS参数
    nh_.setParam("/qr_content", content);
    
    return true;
}

void QRLogisticsDetector::drawZBarResult(cv::Mat &frame, zbar::SymbolIterator symbol) {
    std::vector<cv::Point> points;
    for (int i = 0; i < symbol->get_location_size(); i++) {
        points.push_back(cv::Point(
            symbol->get_location_x(i),
            symbol->get_location_y(i)));
    }
    
    cv::polylines(frame, points, true, cv::Scalar(0, 255, 0), 2);
    cv::putText(frame, symbol->get_data(), 
               points[0], cv::FONT_HERSHEY_SIMPLEX, 0.5, 
               cv::Scalar(0, 0, 255), 1);
}

void QRLogisticsDetector::publishImage(const cv::Mat& frame) {
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(
        std_msgs::Header(), "bgr8", frame).toImageMsg();
    image_pub_.publish(msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "qr_logistics_detector");
    setlocale(LC_ALL,"");
    ros::NodeHandle nh("~");
    QRLogisticsDetector detector(nh);
    detector.run();
    return 0;
 