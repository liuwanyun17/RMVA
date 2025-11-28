#include <cv_bridge/cv_bridge.h>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <referee_pkg/msg/multi_object.hpp>
#include <referee_pkg/msg/object.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>
#include "sensor_msgs/msg/image.hpp"

using namespace std;
using namespace rclcpp;
using namespace cv;

class MultiArmorNode : public rclcpp::Node {
 public:
  MultiArmorNode(string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "Initializing MutiArmorNode");

    Image_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        bind(&MultiArmorNode::callback_camera, this, std::placeholders::_1));

    Target_pub = this->create_publisher<referee_pkg::msg::MultiObject>(
        "/vision/target", 10);

    cv::namedWindow("Detection Result", cv::WINDOW_AUTOSIZE);
    RCLCPP_INFO(this->get_logger(), "MutiArmorNode initialized successfully");
  }

  ~MultiArmorNode() { cv::destroyWindow("Detection Result"); }

 private:
  void callback_camera(sensor_msgs::msg::Image::SharedPtr msg);
  vector<Point2f> CalculateProjdistanceToFindPoints(const vector<Point>& red_corners
                                                ,const Point2f& center,
                                                const Rect& boundingRect,
                                               const RotatedRect& rotated_boundingRect);
  vector<Point2f> sortArmorCorners(const vector<Point2f>& corners);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr Image_sub;
  rclcpp::Publisher<referee_pkg::msg::MultiObject>::SharedPtr Target_pub;
  vector<Point2f> Point_V;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MultiArmorNode>("MutiArmorNode");
  RCLCPP_INFO(node->get_logger(), "Starting MutiArmorNode");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
vector<Point2f> MultiArmorNode::CalculateProjdistanceToFindPoints(const vector<Point>& red_corners,
                                                             const Point2f& center,
                                                             const Rect& boundingRect,
                                                             const RotatedRect& rotated_boundingRect) {
   float angle = rotated_boundingRect.angle;
if (angle < -45) {
    angle += 90;
} else if (angle > 45) {
    angle -= 90;
}
angle = angle * CV_PI / 180.0;  // 获取偏转角度并转换为弧度

    Size2f size = rotated_boundingRect.size;

    //计算灯条的方向
    Point2f dir_vector(cos(angle), sin(angle));
    if (size.width < size.height) {
        dir_vector = Point2f(-sin(angle), cos(angle)); // 垂直方向
    }
    else {
        dir_vector = Point2f(cos(angle), sin(angle)); // 水平方向
    }
    
    //方向向量归一化为单位向量，确保后面的投影距离计算正确
    double dir_length = sqrt(dir_vector.x * dir_vector.x + dir_vector.y * dir_vector.y);
    dir_vector.x /= dir_length;
    dir_vector.y /= dir_length;

    //计算每一个点到中心点的沿灯条方向投影距离

    double max_projdistance_top = 1e9; 
    double max_projdistance_bottom = -1e9;
    vector<Point2f> extreme_points(2 , Point2f(0,0)); 
    
    for(const auto& corner : red_corners){
        Point2f corner2f(boundingRect.x + corner.x, boundingRect.y + corner.y);
        Point2f vec_to_center = corner2f - center;
        double proj_distance = vec_to_center.dot(dir_vector);// 投影距离，dot表示点积

      
            
            if(proj_distance>max_projdistance_top){
                max_projdistance_top=proj_distance;
                extreme_points[0] = corner2f;
            }
              
          
            else (proj_distance>max_projdistance_bottom);
            {
                max_projdistance_bottom=proj_distance;
            extreme_points[1] = corner2f;  
          }
    }
    return extreme_points;
}


    

vector<Point2f> MultiArmorNode::sortArmorCorners(const vector<Point2f>& corners) {
    vector<Point2f> sorted(4);
    Point2f center(0,0);
    for (const auto& corner : corners) {
        center += corner;
    }
    center /= 4.0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
    //范围for循环
    for (const auto& corner : corners) {
        if (corner.x < center.x && corner.y > center.y) {
            sorted[0] = corner;  // 左下
        } else if (corner.x > center.x && corner.y > center.y) {
            sorted[1] = corner;  // 右下
        } else if (corner.x > center.x && corner.y < center.y) {
            sorted[2] = corner;  // 右上
        } else if (corner.x < center.x && corner.y < center.y) {
            sorted[3] = corner;  // 左上
        }
    }
    return sorted;
}



void MultiArmorNode::callback_camera(sensor_msgs::msg::Image::SharedPtr msg) {
  try {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_ptr->image;

    if (image.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty image");
      return;
    }

    cv::Mat result_image = image.clone();
    
    // 模糊处理
    cv::Mat blurred;
    cv::GaussianBlur(image, blurred, cv::Size(15, 15), 0);
    
    // HSV转换
    cv::Mat hsv;
    cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);
    
    // 红色范围
    cv::Mat mask1, mask2, mask_wide;
    
    // 宽红色范围
    cv::inRange(hsv, cv::Scalar(0, 50, 50), cv::Scalar(20, 255, 255), mask1);
    cv::inRange(hsv, cv::Scalar(160, 50, 50), cv::Scalar(180, 255, 255), mask2);
    mask_wide = mask1 | mask2;
   
    
    cv::Mat mask3;
    cv::inRange(hsv,cv::Scalar(0,0,0),cv::Scalar(180,255,50),mask3);
    cv::imshow("mask3",mask3);
    Mat mask_total = mask_wide | mask3;
    cv::imshow("mask_total",mask_total);
    
    
    // 对 mask_wide 进行形态学操作
    cv::Mat mask_for_contours = mask_total.clone();
    cv::Mat kernel_dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::dilate(mask_for_contours, mask_for_contours, kernel_dilate);
    cv::imshow("mask_for_contours",mask_for_contours);
    cv::Mat kernel_close = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15));
    cv::morphologyEx(mask_for_contours, mask_for_contours, cv::MORPH_CLOSE, kernel_close);
    
    // 找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask_for_contours, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    RCLCPP_INFO(this->get_logger(), "Found %ld contours in mask_wide", contours.size());

    
    
    Point_V.clear();
    
    std::vector<cv::Point2f> all_points;  // 存储所有区域的最高点和最低点

    int point_counter = 1;

    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        
        if (area < 50) continue;

        
        // 找到轮廓的所有点
        std::vector<cv::Point> contour_points = contours[i];
        if (contour_points.empty()) continue;

        //确定边界矩形
        cv::RotatedRect rotated_boundingRect = cv::minAreaRect(contours[i]);
        cv::Rect boundingRect = rotated_boundingRect.boundingRect();

        cv::Point2f armor_center = rotated_boundingRect.center;  
        


        //设置装甲板区域为感兴趣区域roi
        cv::Mat roi = hsv(boundingRect); 
        cv::Mat redmask1, redmask2, redmask;
    
        // 宽红色范围
        cv::inRange(roi, cv::Scalar(0, 50, 50), cv::Scalar(20, 255, 255), redmask1);
        cv::inRange(roi, cv::Scalar(160, 50, 50), cv::Scalar(180, 255, 255), redmask2);
        redmask= redmask1 | redmask2;

        std::vector<std::vector<cv::Point>> redcontours;
        cv::findContours(redmask, redcontours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (size_t k = 0; k < redcontours.size(); k++) {
        double redarea = cv::contourArea(redcontours[k]);
        
        if (redarea < 50) continue;

        std::vector<cv::Point> redcontour_points = redcontours[k];
        if (redcontour_points.empty()) continue;

        std::vector<cv::Point2f> collected_points = CalculateProjdistanceToFindPoints(
        redcontour_points, armor_center, boundingRect, rotated_boundingRect);
        


      //   // 按y坐标排序找到最高点和最低点
        // std::vector<cv::Point> sorted_points = redcontour_points;
      //   std::sort(sorted_points.begin(), sorted_points.end(), 
      //            [](const cv::Point& a, const cv::Point& b) {
      //             return a.y<b.y;
      //             });
        
      //   // 最高点（y最小）
      //   cv::Point highest_point = sorted_points[0];
      //   // 最低点（y最大）
      //   cv::Point lowest_point = sorted_points[sorted_points.size() - 1];

        // 存储到所有点集合中
        all_points.push_back(collected_points[0]);
        all_points.push_back(collected_points[1]);
        
        RCLCPP_INFO(this->get_logger(), 
                   "Region %ld: Highest=(%f,%f), Lowest=(%f,%f)", 
                   i, collected_points[0].x, collected_points[0].y, collected_points[1].x, collected_points[1].y); 
      }             
    // 如果有两个区域有4个点（两个区域的最高点和最低点），按左下、右下、右上、左上排序
    if (all_points.size() ==  4) {
        // 使用排序函数按左下、右下、右上、左上排序
        std::vector<cv::Point2f> sorted_points = sortArmorCorners(all_points);
        
        // 绘制红色空心圆圈和数字标签 
        for (int j = 0; j < 4; j++) {
            
            cv::circle(result_image, sorted_points[j], 3, cv::Scalar(0, 0, 255), 3);
            
            // 添加数字标签 1,2,3,4
            string point_text = to_string(point_counter);
            cv::putText(result_image, point_text,
                       cv::Point(sorted_points[j].x + 3, sorted_points[j].y),
                       cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
            
            Point_V.push_back(sorted_points[j]);
            point_counter ++;
            
        }
        // sorted_points.clear();
        all_points.clear();
        RCLCPP_INFO(this->get_logger(), "Marked 4 points from 2 regions");
    } else {
        RCLCPP_WARN(this->get_logger(), "Need exactly 4 points (2 regions), but got %ld points", all_points.size());
    }
  }
    // 显示带圆圈的检测结果
    cv::imshow("Detection Result", result_image);
    
    cv::waitKey(1);  // 添加这一行以更新显示

// 创建并发布装甲板消息
referee_pkg::msg::MultiObject msg_object;
msg_object.header = msg->header;
msg_object.num_objects = Point_V.size() / 4;  // 每个装甲板4个角点

vector<string> types = {"armor_red_1","armor_red_2"};  // 修改为装甲板类型

for (int k = 0; k < msg_object.num_objects; k++) {
  referee_pkg::msg::Object obj;
  obj.target_type = (k < types.size()) ? types[k] : "armor";  // 设为armor

  for (int j = 0; j < 4; j++) {
    int index = 4 * k + j;
    if (index < Point_V.size()) {
      geometry_msgs::msg::Point corner;
      corner.x = Point_V[index].x;
      corner.y = Point_V[index].y;
      corner.z = 0.0;
      obj.corners.push_back(corner);
    }
  }

  msg_object.objects.push_back(obj);
}

Target_pub->publish(msg_object);
RCLCPP_INFO(this->get_logger(), "Published %d armor targets",
            msg_object.num_objects);
    
  } catch (const cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
  }
}


//////////////////////////////////////////////////////
#include <cv_bridge/cv_bridge.h>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <referee_pkg/msg/multi_object.hpp>
#include <referee_pkg/msg/object.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>
#include "sensor_msgs/msg/image.hpp"

using namespace std;
using namespace rclcpp;
using namespace cv;


class DigitClassifier {
private:
    cv::dnn::Net net; //初始化神经网络
    bool is_loaded; // 标志位，指示模型是否已加载
    
public:
    DigitClassifier() : is_loaded(false) {}// 构造函数初始化标志位为false
    
    // 加载神经网络模型函数部分
    bool loadModel(const std::string& model_path) {
        try {
            net = cv::dnn::readNetFromONNX("/home/liuwanyun/Vision_Arena_2025/src/player_pkg/models/digit_model.onnx");
            is_loaded = true;
            RCLCPP_INFO(rclcpp::get_logger("DigitClassifier"), "神经网络模型加载成功");
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("DigitClassifier"), "模型加载失败: %s", e.what());
            return false;
        }
    }
    
    // 动态识别中判断是否为有数字面的正面装甲板
    bool isFrontView(const cv::Mat& armor_roi) {

        //异常处理
        if (!is_loaded || armor_roi.empty()) return false;
        
        try {
            // 预处理装甲板区域
            cv::Mat processed = preprocessROI(armor_roi);
            
            // 神经网络推理
            net.setInput(processed);
            cv::Mat output = net.forward();
            
            // 获取预测结果和置信度
            cv::Point class_id;
            double confidence;
            cv::minMaxLoc(output, nullptr, &confidence, nullptr, &class_id);
            
            int predicted_digit = class_id.x;
            
            RCLCPP_DEBUG(rclcpp::get_logger("DigitClassifier"), 
                        "数字识别结果: %d, 置信度: %.3f", predicted_digit, confidence);
            
            // 如果能识别到数字0-9且置信度足够高，认为是正面
            return (predicted_digit >= 0 && predicted_digit <= 9 && confidence > 0.7);
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("DigitClassifier"), "推理错误: %s", e.what());
            return false;
        }
    }
    
private:
    //图像预处理函数
    cv::Mat preprocessROI(const cv::Mat& roi) {
        cv::Mat processed;
        
        //转换为灰度图
        if (roi.channels() == 3) {
            cv::cvtColor(roi, processed, cv::COLOR_BGR2GRAY);
        } else {
            processed = roi.clone();
        }
        
        //调整大小到网络输入尺寸 (28x28)
        cv::resize(processed, processed, cv::Size(28, 28));
        
        //归一化到 [0, 1]，计算更稳定
        processed.convertTo(processed, CV_32F, 1.0 / 255.0);
        
        //重塑为网络输入格式 (1, 1, 28, 28)
        cv::Mat blob = cv::dnn::blobFromImage(processed);
        return blob;
    }
};

class MultiArmorNode : public rclcpp::Node {
 public:
  MultiArmorNode(string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "Initializing MutiArmorNode");

    std::string model_path = "/home/liuwanyun/Vision_Arena_2025/src/player_pkg/models/digit_model.onnx"; 
    bool model_loaded = digit_classifier.loadModel(model_path);
    if (!model_loaded) {
        RCLCPP_WARN(this->get_logger(), "神经网络模型加载失败，将返回所有检测到的装甲板");
    }

    Image_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        bind(&MultiArmorNode::callback_camera, this, std::placeholders::_1));

    Target_pub = this->create_publisher<referee_pkg::msg::MultiObject>(
        "/vision/target", 10);

    cv::namedWindow("Detection Result", cv::WINDOW_AUTOSIZE);
    RCLCPP_INFO(this->get_logger(), "MutiArmorNode initialized successfully");
  }

  ~MultiArmorNode() { cv::destroyWindow("Detection Result"); }

 private:
  void callback_camera(sensor_msgs::msg::Image::SharedPtr msg);
  vector<Point2f> calculateAbstractRectanglePoints(const vector<Point>& contour);
  vector<Point2f> sortArmorCorners(const vector<Point2f>& corners);

  cv::Mat extractDigitROI(const cv::Mat& image, const std::vector<cv::Point2f>& corners);
  DigitClassifier digit_classifier;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr Image_sub;
  rclcpp::Publisher<referee_pkg::msg::MultiObject>::SharedPtr Target_pub;
  vector<Point2f> Point_V;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MultiArmorNode>("MutiArmorNode");
  RCLCPP_INFO(node->get_logger(), "Starting MutiArmorNode");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
cv::Mat MultiArmorNode::extractDigitROI(const cv::Mat& image, const std::vector<cv::Point2f>& corners) {
    if (corners.size() != 4) return cv::Mat();
    
    // 计算边界矩形
    std::vector<cv::Point2f> sorted_corners = sortArmorCorners(corners);
    
    // 找到最小包围矩形
    cv::Rect roi_rect = cv::boundingRect(sorted_corners);
    
    // 确保不超出图像边界
    roi_rect &= cv::Rect(0, 0, image.cols, image.rows);
    
    if (roi_rect.area() <= 0) return cv::Mat();
    
    // 提取ROI区域
    cv::Mat digit_roi = image(roi_rect);
    
    return digit_roi;
}

vector<Point2f> MultiArmorNode::sortArmorCorners(const vector<Point2f>& corners) {
    vector<Point2f> sorted(4);
    Point2f center(0,0);
    for (const auto& corner : corners) {
        center += corner;
    }
    center /= 4.0;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
    //范围for循环
    for (const auto& corner : corners) {
        if (corner.x < center.x && corner.y > center.y) {
            sorted[0] = corner;  // 左下
        } else if (corner.x > center.x && corner.y > center.y) {
            sorted[1] = corner;  // 右下
        } else if (corner.x > center.x && corner.y < center.y) {
            sorted[2] = corner;  // 右上
        } else if (corner.x < center.x && corner.y < center.y) {
            sorted[3] = corner;  // 左上
        }
    }
    return sorted;
}


void MultiArmorNode::callback_camera(sensor_msgs::msg::Image::SharedPtr msg) {
  try {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_ptr->image;

    if (image.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty image");
      return;
    }

    cv::Mat result_image = image.clone();
    
    // 模糊处理
    cv::Mat blurred;
    cv::GaussianBlur(image, blurred, cv::Size(15, 15), 0);
    
    // HSV转换
    cv::Mat hsv;
    cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);
    
    // 红色范围
    cv::Mat mask1, mask2, mask_wide;
    
    // 宽红色范围
    cv::inRange(hsv, cv::Scalar(0, 50, 50), cv::Scalar(20, 255, 255), mask1);
    cv::inRange(hsv, cv::Scalar(160, 50, 50), cv::Scalar(180, 255, 255), mask2);
    mask_wide = mask1 | mask2;
   
    
    cv::Mat mask3;
    cv::inRange(hsv,cv::Scalar(0,0,0),cv::Scalar(180,255,50),mask3);
    cv::imshow("mask3",mask3);
    Mat mask_total = mask_wide | mask3;
    cv::imshow("mask_total",mask_total);
    
    
    // 对 mask_wide 进行形态学操作
    cv::Mat mask_for_contours = mask_total.clone();
    cv::Mat kernel_dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::dilate(mask_for_contours, mask_for_contours, kernel_dilate);
    cv::imshow("mask_for_contours",mask_for_contours);
    cv::Mat kernel_close = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15));
    cv::morphologyEx(mask_for_contours, mask_for_contours, cv::MORPH_CLOSE, kernel_close);
    
    // 找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask_for_contours, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    RCLCPP_INFO(this->get_logger(), "Found %ld contours in mask_wide", contours.size());

    
    
    Point_V.clear();
    
    std::vector<cv::Point2f> all_points;  // 存储所有区域的最高点和最低点

    int point_counter = 1;

    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        
        if (area < 50) continue;

        
        // 找到轮廓的所有点
        std::vector<cv::Point> contour_points = contours[i];
        if (contour_points.empty()) continue;

        //确定边界矩形
        cv::Rect boundingRect = cv::boundingRect(contours[i]);
        
       
        //设置装甲板区域为感兴趣区域roi
        cv::Mat roi = hsv(boundingRect); 
        cv::Mat redmask1, redmask2, redmask;
    
        // 宽红色范围
        cv::inRange(roi, cv::Scalar(0, 50, 50), cv::Scalar(20, 255, 255), redmask1);
        cv::inRange(roi, cv::Scalar(160, 50, 50), cv::Scalar(180, 255, 255), redmask2);
        redmask= redmask1 | redmask2;

        std::vector<std::vector<cv::Point>> redcontours;
        cv::findContours(redmask, redcontours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (size_t k = 0; k < redcontours.size(); k++) {
        double redarea = cv::contourArea(redcontours[k]);
        
        if (redarea < 50) continue;

        std::vector<cv::Point> redcontour_points = redcontours[k];
        if (redcontour_points.empty()) continue;


        // 按y坐标排序找到最高点和最低点
        std::vector<cv::Point> sorted_points = redcontour_points;
        std::sort(sorted_points.begin(), sorted_points.end(), 
                 [](const cv::Point& a, const cv::Point& b) {
                  return a.y<b.y;
                  });
        
        // 最高点（y最小）
        cv::Point highest_point = sorted_points[0];
        // 最低点（y最大）
        cv::Point lowest_point = sorted_points[sorted_points.size() - 1];

        cv::Point2f raw_highest(highest_point.x + boundingRect.x, 
                                  highest_point.y + boundingRect.y);
        cv::Point2f raw_lowest(lowest_point.x + boundingRect.x, 
                                 lowest_point.y + boundingRect.y);
        
        // 存储到所有点集合中
        all_points.push_back(cv::Point2f(raw_highest.x, raw_highest.y));
        all_points.push_back(cv::Point2f(raw_lowest.x, raw_lowest.y));
        
        RCLCPP_INFO(this->get_logger(), 
                   "Region %d: Highest=(%d,%d), Lowest=(%d,%d)", 
                   i, raw_highest.x, raw_highest.y, raw_lowest.x, raw_lowest.y); 
      }             
    // 如果有两个区域有4个点（两个区域的最高点和最低点），按左下、右下、右上、左上排序
    if (all_points.size() ==  4) {
        // 使用排序函数按左下、右下、右上、左上排序
        std::vector<cv::Point2f> sorted_points = sortArmorCorners(all_points);
        
        // 绘制红色空心圆圈和数字标签 
        for (int j = 0; j < 4; j++) {
            
            cv::circle(result_image, sorted_points[j], 3, cv::Scalar(0, 0, 255), 3);
            
            // 添加数字标签 1,2,3,4
            string point_text = to_string(point_counter);
            cv::putText(result_image, point_text,
                       cv::Point(sorted_points[j].x + 3, sorted_points[j].y),
                       cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
            
            Point_V.push_back(sorted_points[j]);
            point_counter ++;
            
        }
        sorted_points.clear();
        all_points.clear();
        RCLCPP_INFO(this->get_logger(), "Marked 4 points from 2 regions");
    } else {
        RCLCPP_WARN(this->get_logger(), "Need exactly 4 points (2 regions), but got %d points", all_points.size());
    }
  }
    // 显示带圆圈的检测结果
    cv::imshow("Detection Result", result_image);
    
    cv::waitKey(1);  // 添加这一行以更新显示

// 创建并发布装甲板消息
referee_pkg::msg::MultiObject msg_object;
msg_object.header = msg->header;
msg_object.num_objects = Point_V.size() / 4;  // 每个装甲板4个角点

vector<string> types = {"armor_red_1","armor_red_2","armor_red_3","armor_red_4","armor_red_5"};  // 修改为装甲板类型

for (int k = 0; k < msg_object.num_objects; k++) {
  referee_pkg::msg::Object obj;
  obj.target_type = (k < types.size()) ? types[k] : "armor";  // 设为armor

  for (int j = 0; j < 4; j++) {
    int index = 4 * k + j;
    if (index < Point_V.size()) {
      geometry_msgs::msg::Point corner;
      corner.x = Point_V[index].x;
      corner.y = Point_V[index].y;
      corner.z = 0.0;
      obj.corners.push_back(corner);
    }
  }

  msg_object.objects.push_back(obj);
}

Target_pub->publish(msg_object);
RCLCPP_INFO(this->get_logger(), "Published %d armor targets",
            msg_object.num_objects);
    
  } catch (const cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
  }
}


