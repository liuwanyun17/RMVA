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

class ArmorNode : public rclcpp::Node {
 public:
  ArmorNode(string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "Initializing ArmorNode");

    Image_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        bind(&ArmorNode::callback_camera, this, std::placeholders::_1));

    Target_pub = this->create_publisher<referee_pkg::msg::MultiObject>(
        "/vision/target", 10);

    cv::namedWindow("Detection Result", cv::WINDOW_AUTOSIZE);
    RCLCPP_INFO(this->get_logger(), "ArmorNode initialized successfully");
  }

  ~ArmorNode() { cv::destroyWindow("Detection Result"); }

 private:
  void callback_camera(sensor_msgs::msg::Image::SharedPtr msg);
  vector<Point2f> calculateAbstractRectanglePoints(const vector<Point>& contour);
  vector<Point2f> sortArmorCorners(const vector<Point2f>& corners);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr Image_sub;
  rclcpp::Publisher<referee_pkg::msg::MultiObject>::SharedPtr Target_pub;
  vector<Point2f> Point_V;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmorNode>("ArmorNode");
  RCLCPP_INFO(node->get_logger(), "Starting ArmorNode");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

vector<Point2f> ArmorNode::sortArmorCorners(const vector<Point2f>& corners) {
    vector<Point2f> sorted(4);
    Point2f center(0, 0);
    
    for (const auto& corner : corners) {
        center += corner;
    }
    center /= 4.0;
    
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

vector<Point2f> ArmorNode::calculateAbstractRectanglePoints(const vector<Point>& contour) {
    vector<Point2f> abstract_points;
    if (contour.empty()) return abstract_points;
    
    Point2f top_left = contour[0], top_right = contour[0];
    Point2f bottom_left = contour[0], bottom_right = contour[0];
    
    for (const auto& point : contour) {
        if (point.y < top_left.y) {
            if (point.x < top_left.x) top_left = point;
        }
        if (point.y < top_right.y) {
            if (point.x > top_right.x) top_right = point;
        }
        if (point.y > bottom_left.y) {
            if (point.x < bottom_left.x) bottom_left = point;
        }
        if (point.y > bottom_right.y) {
            if (point.x > bottom_right.x) bottom_right = point;
        }
    }
    
    abstract_points.push_back(bottom_left);   // 左下 - 1
    abstract_points.push_back(bottom_right);  // 右下 - 2
    abstract_points.push_back(top_right);     // 右上 - 3
    abstract_points.push_back(top_left);      // 左上 - 4
    
    return abstract_points;
}

void ArmorNode::callback_camera(sensor_msgs::msg::Image::SharedPtr msg) {
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
    cv::GaussianBlur(image, blurred, cv::Size(3, 3), 0);

    // 提高对比度
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
    cv::imshow("mask",mask_wide);
    
    
    
    // 对 mask_wide 进行形态学操作
    cv::Mat mask_for_contours = mask_wide.clone();
    cv::Mat kernel_dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::dilate(mask_for_contours, mask_for_contours, kernel_dilate);
    cv::imshow("mask_for_contours",mask_for_contours);
    cv::Mat kernel_close = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
    cv::morphologyEx(mask_for_contours, mask_for_contours, cv::MORPH_CLOSE, kernel_close);
    
    // 找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask_for_contours, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    RCLCPP_INFO(this->get_logger(), "Found %d contours in mask_wide", contours.size());


    
    Point_V.clear();
    
    // 只处理前2个区域（两个红色区域）
    int regions_to_process = std::min(2, (int)contours.size());
    
    // 存储所有区域的最高点和最低点
    std::vector<cv::Point2f> all_points;  
    
    for (int i = 0; i < regions_to_process; i++) {
        double area = cv::contourArea(contours[i]);
        
        if (area < 50) continue;
        
        // 找到轮廓的所有点
        std::vector<cv::Point> contour_points = contours[i];
        if (contour_points.empty()) continue;
        
        // 按y坐标排序找到最高点和最低点
        std::vector<cv::Point> sorted_points = contour_points;
        std::sort(sorted_points.begin(), sorted_points.end(), 
                 [](const cv::Point& a, const cv::Point& b) { return a.y < b.y; });
        
        // 最高点（y最小）
        cv::Point highest_point = sorted_points[0];
        // 最低点（y最大）
        cv::Point lowest_point = sorted_points[sorted_points.size() - 1];
        
        // 存储到所有点集合中
        all_points.push_back(cv::Point2f(highest_point.x, highest_point.y));
        all_points.push_back(cv::Point2f(lowest_point.x, lowest_point.y));
        
        RCLCPP_INFO(this->get_logger(), 
                   "Region %d: Highest=(%d,%d), Lowest=(%d,%d)", 
                   i, highest_point.x, highest_point.y, lowest_point.x, lowest_point.y);
    }
    
    // 如果有4个点（两个区域的最高点和最低点），按左下、右下、右上、左上排序
    if (all_points.size() == 4) {
        // 使用排序函数按左下、右下、右上、左上排序
        std::vector<cv::Point2f> sorted_points = sortArmorCorners(all_points);
        
        // 绘制红色空心圆圈和数字标签 
        for (int j = 0; j < 4; j++) {
            
            cv::circle(result_image, sorted_points[j], 3, cv::Scalar(0, 0, 255), 3);
            
            // 添加数字标签 1,2,3,4
            string point_text = to_string(j + 1);
            cv::putText(result_image, point_text,
                       cv::Point(sorted_points[j].x + 3, sorted_points[j].y),
                       cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 3);
            
            Point_V.push_back(sorted_points[j]);
        }
        
        RCLCPP_INFO(this->get_logger(), "Marked 4 points from 2 regions");
    } else {
        RCLCPP_WARN(this->get_logger(), "Need exactly 4 points (2 regions), but got %d points", all_points.size());
    }
    
    // 显示带圆圈的检测结果
    cv::imshow("Detection Result", result_image);
    
    cv::waitKey(1);  // 添加这一行以更新显示
    
    // 创建并发布装甲板消息
referee_pkg::msg::MultiObject msg_object;
msg_object.header = msg->header;
msg_object.num_objects = Point_V.size() / 4;  // 每个装甲板4个角点

vector<string> types = {"armor_red_1"};  // 修改为装甲板类型

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
