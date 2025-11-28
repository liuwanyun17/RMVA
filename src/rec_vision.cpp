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

class Vision : public rclcpp::Node {
 public:
  Vision() : Node("rec_vision_node") {
    RCLCPP_INFO(this->get_logger(), "Initializing rec_vision_node");

    Image_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        bind(&Vision::callback_camera, this, std::placeholders::_1));

    Target_pub = this->create_publisher<referee_pkg::msg::MultiObject>(
        "/vision/target", 10);

    cv::namedWindow("Detection Result", cv::WINDOW_AUTOSIZE);

// cv::namedWindow("HSV Adjust", cv::WINDOW_AUTOSIZE);
// createTrackbar("H Min", "HSV Adjust", &h_min, 180);
// createTrackbar("H Max", "HSV Adjust", &h_max, 180);
// createTrackbar("S Min", "HSV Adjust", &s_min, 255);
// createTrackbar("S Max", "HSV Adjust", &s_max, 255);
// createTrackbar("V Min", "HSV Adjust", &v_min, 255);
// createTrackbar("V Max", "HSV Adjust", &v_max, 255);
    RCLCPP_INFO(this->get_logger(), "rec_vision_node initialized successfully");
  }

  ~Vision() { cv::destroyWindow("Detection Result"); }

 private:
   int h_min = 35, h_max = 85;
  int s_min = 50, s_max = 255;
  int v_min = 50, v_max = 255;
   
  void callback_camera(sensor_msgs::msg::Image::SharedPtr msg);

  // 稳定的矩形角点计算方法
  vector<Point2f> calculateStableRectanglePoints(const Rect& bbox);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr Image_sub;
  rclcpp::Publisher<referee_pkg::msg::MultiObject>::SharedPtr Target_pub;
  vector<Point2f> Point_V;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Vision>();
  RCLCPP_INFO(node->get_logger(), "Starting rec_vision_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

vector<Point2f> Vision::calculateStableRectanglePoints(const Rect& bbox) {
  vector<Point2f> points;

  // 简单稳定的几何计算，避免漂移
  
  points.push_back(Point2f(bbox.x , bbox.y+bbox.height));  // 左下点 (1)
  points.push_back(Point2f(bbox.x+bbox.width, bbox.y + bbox.height));  // 右下点 (2)
  points.push_back(Point2f(bbox.x+bbox.width, bbox.y ));  // 右上点 (3)
  points.push_back(Point2f(bbox.x, bbox.y ));  // 左上点 (4)

  return points;
}

void Vision::callback_camera(sensor_msgs::msg::Image::SharedPtr msg) {
  try {
    // 图像转换
    cv_bridge::CvImagePtr cv_ptr;

    if (msg->encoding == "rgb8" || msg->encoding == "R8G8B8") {
      cv::Mat image(msg->height, msg->width, CV_8UC3,
                    const_cast<unsigned char *>(msg->data.data()));
      cv::Mat bgr_image;
      cv::cvtColor(image, bgr_image, cv::COLOR_RGB2BGR);
      cv_ptr = std::make_shared<cv_bridge::CvImage>();
      cv_ptr->header = msg->header;
      cv_ptr->encoding = "bgr8";
      cv_ptr->image = bgr_image;
    } else {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }

    cv::Mat image = cv_ptr->image;

    if (image.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty image");
      return;
    }

    // 创建结果图像
    cv::Mat result_image = image.clone();

    // 转换到 HSV 空间
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);

    //绿色检测 - 使用稳定的范围
    cv::Mat mask;
    //cv::inRange(hsv, cv::Scalar(h_min, s_min, v_min), cv::Scalar(h_max, s_max, v_max),  mask);
    cv::inRange(hsv, cv::Scalar(35, 0, 87), cv::Scalar(180, 255, 255),  mask);
   cv::imshow("Mask", mask);

    // 适度的形态学操作
    cv::Mat kernel =
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

    // 找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    Point_V.clear();
    int valid_rectangles = 0;

    for (size_t i = 0; i < contours.size(); i++) {
      double area = cv::contourArea(contours[i]);
      if (area < 100) continue;

      Rect bbox=boundingRect(contours[i]);
      float width=bbox.width;
      float height=bbox.height;

    vector<Point2f> rect_points = calculateStableRectanglePoints(bbox);

        // 绘制检测到的矩形
 
      cv::rectangle(result_image,bbox.tl(),bbox.br(),Scalar(0,0,255),2);

        // 绘制矩形上的四个点
        vector<string> point_names = {"左下", "右下", "右上", "左上"};
        vector<cv::Scalar> point_colors = {
            cv::Scalar(255, 0, 0),    // 蓝色 
            cv::Scalar(0, 255, 0),    // 绿色 
            cv::Scalar(0, 255, 255),  // 黄色 
            cv::Scalar(255, 0, 255)   // 紫色 
        };

        for (int j = 0; j < 4; j++) {
          cv::circle(result_image, rect_points[j], 6, point_colors[j], -1);
          cv::circle(result_image, rect_points[j], 6, cv::Scalar(0, 0, 0), 2);

          // 标注序号
          string point_text = to_string(j + 1);
          cv::putText(
              result_image, point_text,
              cv::Point(rect_points[j].x + 10, rect_points[j].y - 10),
              cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 3);
          cv::putText(
              result_image, point_text,
              cv::Point(rect_points[j].x + 10, rect_points[j].y - 10),
              cv::FONT_HERSHEY_SIMPLEX, 0.6, point_colors[j], 2);

          // 添加到发送列表
          Point_V.push_back(rect_points[j]);

          RCLCPP_INFO(this->get_logger(),
                      "Rectangle %d, Point %d (%s): (%.1f, %.1f)",
                      valid_rectangles + 1, j + 1, point_names[j].c_str(),
                      rect_points[j].x, rect_points[j].y);
        }
            valid_rectangles++;
        
      }
    

    // 显示结果图像
    cv::imshow("Detection Result", result_image);
    cv::waitKey(1);

    // 创建并发布消息
    referee_pkg::msg::MultiObject msg_object;
    msg_object.header = msg->header;
    msg_object.num_objects = Point_V.size() / 4;

    vector<string> types = {"rect"};

    for (int k = 0; k < msg_object.num_objects; k++) {
      referee_pkg::msg::Object obj;
      obj.target_type = (k < types.size()) ? types[k] : "unknown";

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
    RCLCPP_INFO(this->get_logger(), "Published %d rectangle targets",
                msg_object.num_objects);

  } catch (const cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
  }
}