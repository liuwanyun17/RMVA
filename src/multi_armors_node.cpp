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
#include <map>


using namespace std;
using namespace rclcpp;
using namespace cv;

class DigitTemplateMatcher {
private:
    std::map<int, cv::Mat> templates;
    
public:
    DigitTemplateMatcher() {}
    
    bool loadTemplates(const std::string& template_dir) {
        templates.clear();
        
        for (int digit = 1; digit <= 5; digit++) {
            std::string path = template_dir + "/" + std::to_string(digit) + ".jpg";
            cv::Mat template_img = cv::imread(path, cv::IMREAD_GRAYSCALE);
            
            if (template_img.empty()) {
                RCLCPP_WARN(rclcpp::get_logger("DigitTemplateMatcher"), 
                           "无法加载模板: %s", path.c_str());
                continue;
            }
            
            // 调整模板尺寸
            cv::Mat resized_template;
            cv::resize(template_img, resized_template, cv::Size(30, 45));
            
            // 二值化
            cv::Mat binary;
            cv::threshold(resized_template, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
            templates[digit] = binary;
            
            RCLCPP_INFO(rclcpp::get_logger("DigitTemplateMatcher"), 
                       "加载数字 %d 模板", digit);
        }
        
        return !templates.empty();
    }
    
    std::string recognize(const cv::Mat& roi) {
        if (roi.empty() || templates.empty()) {
            return "unknown";
        }
        imshow("ROI for Recognition", roi);
        
        // 转换为灰度图
        cv::Mat gray;
        if (roi.channels() == 3) {
            cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = roi.clone();
        }
        
        int best_digit = -1;
        double best_score = 0.4;  // 置信度阈值

        // 与所有模板匹配
        for (const auto& [digit, template_img] : templates) {
            cv::Mat resized_roi;
            cv::resize(gray, resized_roi, template_img.size());
            
            // 二值化ROI
            cv::Mat roi_binary;
            cv::threshold(resized_roi, roi_binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
            
            cv::Mat result;
            cv::matchTemplate(roi_binary, template_img, result, cv::TM_CCOEFF_NORMED);
            
            double minVal, maxVal;
            cv::Point minLoc, maxLoc;
            cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);
            
            if (maxVal > best_score) {
                best_score = maxVal;
                best_digit = digit;
            }
        }
        
        if (best_digit != -1) {
            RCLCPP_INFO(rclcpp::get_logger("DigitTemplateMatcher"), 
                       "识别结果: %d, 置信度: %.3f", best_digit, best_score);
            return std::to_string(best_digit);
        }
        
        return "unknown";
    }
};

class MultiArmorNode : public rclcpp::Node {
 public:
  MultiArmorNode(string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "Initializing MultiArmorNode");

    std::string template_path = "./digit_templates";
    if (!digit_matcher.loadTemplates(template_path)) {
        RCLCPP_WARN(this->get_logger(), "数字模板加载失败");
    }


    Image_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        bind(&MultiArmorNode::callback_camera, this, std::placeholders::_1));

    Target_pub = this->create_publisher<referee_pkg::msg::MultiObject>(
        "/vision/target", 10);

    cv::namedWindow("Detection Result", cv::WINDOW_AUTOSIZE);
    RCLCPP_INFO(this->get_logger(), "MultiArmorNode initialized successfully");
  }

  ~MultiArmorNode() { cv::destroyWindow("Detection Result"); }

 private:
  void callback_camera(sensor_msgs::msg::Image::SharedPtr msg);
  vector<Point2f> sortArmorCorners(const vector<Point2f>& corners);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr Image_sub;
  rclcpp::Publisher<referee_pkg::msg::MultiObject>::SharedPtr Target_pub;
  vector<Point2f> Point_V;
  DigitTemplateMatcher digit_matcher;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MultiArmorNode>("MultiArmorNode");
  RCLCPP_INFO(node->get_logger(), "Starting MultiArmorNode");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
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
    cv::GaussianBlur(image, blurred, cv::Size(7, 7), 0);

    blurred.convertTo(blurred, -1, 1.5, 0);
    
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
    std::vector<std::string> detected_numbers;  // 存储检测到的数字

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


            // 计算装甲板中心
            cv::Point2f armor_center = (sorted_points[0] + sorted_points[1] + 
                                       sorted_points[2] + sorted_points[3]) / 4.0f;
            
            // 数字区域提取
            string detected_number = "unknown";
            cv::Rect number_roi_rect(
                armor_center.x - boundingRect.width / 3,
                armor_center.y - boundingRect.height / 2,
                boundingRect.width/1.5, 
                boundingRect.height);
            
            number_roi_rect = number_roi_rect & cv::Rect(0, 0, image.cols, image.rows);
            
            
                cv::Mat number_roi = image(number_roi_rect);
                detected_number = digit_matcher.recognize(number_roi);


         cv::putText(result_image, "Num:" + detected_number,
                           cv::Point(armor_center.x - 20, armor_center.y - 25),
                           cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
            
        
        detected_numbers.push_back(detected_number);
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
  // obj.target_type = (k < types.size()) ? types[k] : "armor";  // 设为armor


 string number = (k < detected_numbers.size()) ? detected_numbers[k] : "unknown";
        if (number != "unknown" && std::stoi(number) <= types.size()) {
            obj.target_type = types[std::stoi(number) - 1];
        } else {
            obj.target_type = "armor_red_unknown";
        }

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


