#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaarithm.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "vision_msgs/msg/point2_d.hpp" 
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "cv_bridge/cv_bridge.h"

using std::placeholders::_1;
using namespace cv;
using namespace std::chrono_literals;

/*
视觉目标检测节点: 负责从摄像头捕获图像，通过 HSV 颜色空间提取特定颜色特征，
并利用几何形态学和仿射变换识别并截取目标数字区域。
*/
 
class TargetDetectionNode : public rclcpp::Node
{
public:
    TargetDetectionNode() : Node("target_detection_node")
    {
	// 1. 初始化 QoS 配置 
        const rmw_qos_profile_t custom_qos = {
            RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10,
            RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            RMW_QOS_POLICY_DURABILITY_VOLATILE,
            RMW_DURATION_INFINITE,
            RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, false
        };
        rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(custom_qos), custom_qos);
        
	// 2. 创建发布者与订阅者
        center_pub_ = this->create_publisher<vision_msgs::msg::Point2D>("target_center", qos_profile); 
        image_pub_  = this->create_publisher<sensor_msgs::msg::Image>("target_nums", qos_profile); 
        status_sub_ = this->create_subscription<std_msgs::msg::String>(       
            "status", qos_profile, std::bind(&TargetDetectionNode::status_callback, this, _1));

	// 3. 初始化摄像头及参数
        init_camera();

        // 4. 创建图像处理定时器
        process_timer_ = this->create_wall_timer(
            5ms, std::bind(&TargetDetectionNode::process_frame, this));

        RCLCPP_INFO(this->get_logger(), "TargetDetection Node Initialized Successfully.");
    }

private:
    // ROS2 接口
    rclcpp::Publisher<vision_msgs::msg::Point2D>::SharedPtr center_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
    rclcpp::TimerBase::SharedPtr process_timer_;

    // 相关变量
    VideoCapture capture_;
    Point2f target_center_;
    Mat num_roi_;               // 最终截取到的数字 ROI 区域
    short log_counter_ = 0;
    bool is_active_ = false;    // 节点是否处于活跃工作状态
    bool is_processing_ = false;// 防止定时器重入导致的处理冲突
    
    // 初始化摄像头硬件参数
    void init_camera()
    {
        capture_.open(3, CAP_V4L2);
        if (!capture_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera.");
            return;
        }
        VideoWriter vw;
        capture_.set(CAP_PROP_FOURCC, vw.fourcc('M','J','P','G'));
        capture_.set(CAP_PROP_FRAME_WIDTH, 1280);
        capture_.set(CAP_PROP_FRAME_HEIGHT, 720);
        capture_.set(CAP_PROP_FPS, 30);
        capture_.set(CAP_PROP_GAIN, 10);
    }
    
    // 发布目标中心点坐标
    void publish_target_center()                                                       
    {
        auto msg = vision_msgs::msg::Point2D();                                      
        msg.x = target_center_.x;
        msg.y = target_center_.y;
        center_pub_->publish(msg);                                                
    }
    
    // 发布处理后的目标数字图像
    void publish_digit_image()                                                       
    {
        if (num_roi_.empty()) return;
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", num_roi_).toImageMsg();
        image_pub_->publish(*msg.get());
    }

    // 状态机回调函数，控制视觉处理的启停
    void status_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (++log_counter_ % 5 == 0) {
            RCLCPP_INFO(this->get_logger(), "Current Status: '%s'", msg->data.c_str());  
            log_counter_ = 0;
        }
        
        std::string status_str = msg->data;
        is_active_ = (status_str == "FORWARD" || status_str == "SEARCH");
    }
    
    // 核心视觉处理回调函数
    void process_frame()
    {
        // 检查运行状态与防止重入
        if (!is_active_ || is_processing_) return;
        
        is_processing_ = true;
        auto start_time = std::chrono::high_resolution_clock::now();

        Mat frame, red_and_blue;
        cuda::GpuMat d_frame, d_HSV_frame, d_mask1, d_mask3, d_mask4, d_red_and_blue, d_rotated;
        cuda::Stream stream;
        capture_ >> frame;
        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Empty image frame received.");
            is_processing_ = false;
            return;
        }
        d_frame.upload(frame);
        frame.release();
        
        // 1. 颜色空间转换与掩膜提取 (提取特定红色和蓝色区域)
        cuda::cvtColor(d_frame, d_HSV_frame, COLOR_BGR2HSV, 0, stream);
        cuda::inRange(d_HSV_frame, Scalar(0,100,100), Scalar(5,255,255), d_mask1, stream);
        cuda::inRange(d_HSV_frame, Scalar(95,110,120), Scalar(125,255,255), d_mask3, stream);
        cuda::inRange(d_HSV_frame, Scalar(175,100,100), Scalar(180,255,255), d_mask4, stream);
        d_HSV_frame.release();
        cuda::bitwise_or(d_mask3, d_mask1, d_red_and_blue, noArray(), stream);
        cuda::bitwise_or(d_mask4, d_red_and_blue, d_red_and_blue, noArray(), stream);
        d_red_and_blue.download(red_and_blue, stream);
        d_red_and_blue.release();
        stream.waitForCompletion();
              
        // 2. 轮廓查找与面积滤波
        std::vector<std::vector<Point>> contours;
        findContours(red_and_blue, contours, RETR_TREE, CHAIN_APPROX_NONE);
        
        if (contours.empty()) {
            is_processing_ = false;
            return;
        }

        // 过滤微小噪点轮廓
        for (size_t i = 0; i < contours.size(); ++i) {
            if (contourArea(contours[i]) < 30) {
                drawContours(red_and_blue, contours, static_cast<int>(i), Scalar(0, 0, 0), -1);
            }
        }
        // 3. 形态学闭运算，连接断裂的特征区域
        Mat kernel = getStructuringElement(MORPH_RECT, Size(35, 35));
        morphologyEx(red_and_blue, red_and_blue, MORPH_CLOSE, kernel, Point(-1, -1), 1);
                     
        // 4. 二次轮廓查找并寻找最大包络
        std::vector<std::vector<Point>> refined_contours;
        findContours(red_and_blue, refined_contours, RETR_TREE, CHAIN_APPROX_SIMPLE);
        
        if (refined_contours.empty()) {
            is_processing_ = false;
            return;
        }
        
	    double max_area = 0;
        int max_idx = 0;
        for (size_t i = 0; i < refined_contours.size(); ++i) {
            double area = contourArea(refined_contours[i]);
            if (area > max_area) {
                max_area = area;
                max_idx = static_cast<int>(i);
            }
        }

        // 面积阈值判定
        if (max_area < 300) {
            is_processing_ = false;
            return;
        }
        
        // 5. 几何特征提取与姿态结算
        RotatedRect min_rect = minAreaRect(refined_contours[max_idx]);
        Rect bound_rect = min_rect.boundingRect();
        
        // 边界检查保护
        if (bound_rect.tl().x <= 0 || bound_rect.tl().y <= 0 || 
            bound_rect.br().x >= red_and_blue.cols || bound_rect.br().y >= red_and_blue.rows) {
            RCLCPP_WARN(this->get_logger(), "Target minRect is out of image boundary.");
            is_processing_ = false;
            return;
        }
        
        Point2f rect_points[4];
        min_rect.points(rect_points);
        float angle = min_rect.angle;
        
        if (angle >= 89.0f) {
            is_processing_ = false;
            return;
        }
        
        // 分析角点区域颜色，确定目标的上下朝向
        float corner_intensities[4] = {0};
        for (int i = 0; i < 4; ++i) {
            Rect corner_rect(rect_points[i].x - 10, rect_points[i].y - 10, 20, 20);
            
            // 角点越界检查
            if (corner_rect.tl().x <= 0 || corner_rect.tl().y <= 0 || 
                corner_rect.br().x >= red_and_blue.cols || corner_rect.br().y >= red_and_blue.rows) {
                is_processing_ = false;
                return;
            }
            Mat roi = red_and_blue(corner_rect);
            corner_intensities[i] = mean(roi)[0];
        }
        
        // 找出颜色强度最高的两个角点（推测为目标底部）
        int bot1 = 0, bot2 = 0;
        float max_val = 0;
        for (int i = 0; i < 4; ++i) {
            if (corner_intensities[i] > max_val) {
                max_val = corner_intensities[i];
                bot1 = i;
            }
        }
        max_val = 0;
        for (int i = 0; i < 4; ++i) {
            if (corner_intensities[i] > max_val && i != bot1) {
                max_val = corner_intensities[i];
                bot2 = i;
            }
        }
                            
        // 计算目标几何中心
        float top_x = 0, top_y = 0;
        for (int i = 0; i < 4; ++i) {
            if (i != bot1 && i != bot2) {
                top_x += rect_points[i].x;
                top_y += rect_points[i].y;
            }
        }
        Point2f top_center(top_x / 2.0f, top_y / 2.0f);
        Point2f bot_center((rect_points[bot1].x + rect_points[bot2].x) / 2.0f, 
                           (rect_points[bot1].y + rect_points[bot2].y) / 2.0f);
        
        Point2f target_mid((top_center.x + rect_points[bot1].x + rect_points[bot2].x) / 3.0f, 
                           (top_center.y + rect_points[bot1].y + rect_points[bot2].y) / 3.0f);
                           
        target_center_ = target_mid;
        publish_target_center();
                            
        // 6. 仿射变换校正视角
        Mat rot_mat;
        if (top_center.x > bot_center.x && top_center.y < bot_center.y) {
            rot_mat = getRotationMatrix2D(target_mid, angle, 1.0);
        } else if (top_center.x < bot_center.x && top_center.y < bot_center.y) {
            rot_mat = getRotationMatrix2D(target_mid, 270.0 + angle, 1.0);
        } else if (top_center.x < bot_center.x && top_center.y > bot_center.y) {
            rot_mat = getRotationMatrix2D(target_mid, 180.0 + angle, 1.0);
        } else if (top_center.x > bot_center.x && top_center.y > bot_center.y) {
            rot_mat = getRotationMatrix2D(target_mid, 90.0 + angle, 1.0);
        } else {
            is_processing_ = false;
            return;
        }
        
        cuda::warpAffine(
            d_frame, d_rotated, rot_mat, d_frame.size(), INTER_LINEAR, 0, Scalar(), stream);
	    d_frame.release();
	
	    // 7. 截取中心特征区域 (ROI)
        int length = std::min(min_rect.size.height, min_rect.size.width);
        Rect num_area(target_mid.x - length / 4.0f, target_mid.y - length / 3.8f, 
                      length / 1.8f, length / 1.8f);

        if (num_area.tl().x <= 0 || num_area.tl().y <= 0 || 
            num_area.br().x >= d_rotated.cols || num_area.br().y >= d_rotated.rows) {
            RCLCPP_WARN(this->get_logger(), "Number ROI is out of boundary.");
            is_processing_ = false;
            return;
        }
        
        d_rotated = d_rotated(num_area);
        cuda::resize(d_rotated,d_rotated,Size(256,256),0.0,0.0,1,stream);
        
        // 8. 颜色过滤以凸显目标数字
        cuda::cvtColor(d_rotated,d_HSV_frame,COLOR_BGR2HSV,0,stream);
        cuda::inRange(d_HSV_frame, Scalar(0,0,135),Scalar(180,255,255),d_mask1,stream);
        d_rotated.setTo(Scalar(255, 255, 255), d_mask1);
        d_rotated.download(num_roi_,stream);
        stream.waitForCompletion();
        publish_digit_image();
	    num_roi_.release();
	
	// 性能统计
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed_time = end_time - start_time;
        RCLCPP_INFO(this->get_logger(), "Frame processed in %.2f ms", elapsed_time.count());
	    is_processing_ = false;
    }

};

int main(int argc, char * argv[])                      
{
    // ROS2 C++接口初始化
    rclcpp::init(argc, argv);  
    auto node = std::make_shared<TargetDetectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}       
