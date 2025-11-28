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
#include <opencv2/dnn.hpp> 

using namespace std;
using namespace rclcpp;
using namespace cv;

class MutiArmorNode : public rclcpp::Node {
 public:
  MutiArmorNode(string name) : Node(name) {
    
    RCLCPP_INFO(this->get_logger(), "Initializing MutiArmorNode");

    Image_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        bind(&MutiArmorNode::callback_camera, this, std::placeholders::_1));

    Target_pub = this->create_publisher<referee_pkg::msg::MultiObject>(
        "/vision/target", 10);

    cv::namedWindow("Detection Result", cv::WINDOW_AUTOSIZE);
    RCLCPP_INFO(this->get_logger(), "MutiArmorNode initialized successfully");
  }

  ~MutiArmorNode() { cv::destroyWindow("Detection Result"); }

 private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr Image_sub;
  rclcpp::Publisher<referee_pkg::msg::MultiObject>::SharedPtr Target_pub;
  vector<Point2f> Point_V;
  
  struct ArmorInfo {
      string number;
      vector<Point2f> corners;
  };
  vector<ArmorInfo> armor_list;
  vector<string> armor_numbers;
  
  void callback_camera(sensor_msgs::msg::Image::SharedPtr msg);
  vector<Point2f> sortArmorCorners(const vector<Point2f>& corners);
  string classifyNumber(const cv::Mat& roi);
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MutiArmorNode>("MutiArmorNode");
  RCLCPP_INFO(node->get_logger(), "Starting MutiArmorNode");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

vector<Point2f> MutiArmorNode::sortArmorCorners(const vector<Point2f>& corners) {
    vector<Point2f> sorted(4);
    Point2f center(0,0);
    for (const auto& corner : corners) {
        center += corner;
    }
    center /= 4.0;
    for (const auto& corner : corners) {
        if (corner.x < center.x && corner.y > center.y) {
            sorted[0] = corner;
        } else if (corner.x > center.x && corner.y > center.y) {
            sorted[1] = corner;
        } else if (corner.x > center.x && corner.y < center.y) {
            sorted[2] = corner;
        } else if (corner.x < center.x && corner.y < center.y) {
            sorted[3] = corner;
        }
    }
    return sorted;
}

string MutiArmorNode::classifyNumber(const cv::Mat& roi) {
    if (roi.empty()) return "unknown";
    
    // 最简单的预处理
    cv::Mat gray;
    if (roi.channels() == 3) {
        cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = roi.clone();
    }
    
    // 固定阈值提取白色数字
    cv::Mat binary;
    cv::threshold(gray, binary, 200, 255, cv::THRESH_BINARY);
    
    // 计算白色像素比例
    int white_pixels = cv::countNonZero(binary);
    double white_ratio = (double)white_pixels / (binary.rows * binary.cols);
    
    // 只基于白色比例分类
    if (white_ratio < 0.1 || white_ratio > 0.6) {
        return "unknown";
    }
    else if (white_ratio < 0.24) {
        return "1";  // 数字1最细，白色最少
    }
    else if (white_ratio < 0.30) {
        return "3";  // 数字3
    }
    else if (white_ratio < 0.36) {
        return "2";  // 数字2
    }
    else if (white_ratio < 0.42) {
        return "4";  // 数字4
    }
    else if (white_ratio < 0.50) {
        return "5";  // 数字5
    }
    else {
        return "unknown";
    }
}

void MutiArmorNode::callback_camera(sensor_msgs::msg::Image::SharedPtr msg) {
  try {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat image = cv_ptr->image;

    if (image.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty image");
      return;
    }

    cv::Mat result_image = image.clone();
    
    // 保持原有的图像处理流程
    cv::Mat blurred;
    cv::GaussianBlur(image, blurred, cv::Size(15, 15), 0);
    
    cv::Mat hsv;
    cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);
    
    cv::Mat mask1, mask2, mask_wide;
    cv::inRange(hsv, cv::Scalar(0, 50, 50), cv::Scalar(20, 255, 255), mask1);
    cv::inRange(hsv, cv::Scalar(160, 50, 50), cv::Scalar(180, 255, 255), mask2);
    mask_wide = mask1 | mask2;
    
    cv::Mat mask3;
    cv::inRange(hsv,cv::Scalar(0,0,0),cv::Scalar(180,255,50),mask3);
    Mat mask_total = mask_wide | mask3;
    
    cv::Mat mask_for_contours = mask_total.clone();
    cv::Mat kernel_dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::dilate(mask_for_contours, mask_for_contours, kernel_dilate);
    cv::Mat kernel_close = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15));
    cv::morphologyEx(mask_for_contours, mask_for_contours, cv::MORPH_CLOSE, kernel_close);
    
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask_for_contours, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    RCLCPP_INFO(this->get_logger(), "Found %ld contours", contours.size());

    Point_V.clear();
    armor_numbers.clear();
    armor_list.clear();
    
    vector<ArmorInfo> detected_armors;

    // 第一步：检测所有装甲板并识别中心数字
    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area < 100) continue;

        cv::Rect boundingRect = cv::boundingRect(contours[i]);
        
        double aspect_ratio = (double)boundingRect.width / boundingRect.height;
        if (aspect_ratio < 1.5 || aspect_ratio > 4.0) continue;
        
        cv::Mat roi = hsv(boundingRect); 
        cv::Mat redmask1, redmask2, redmask;
    
        cv::inRange(roi, cv::Scalar(0, 50, 50), cv::Scalar(20, 255, 255), redmask1);
        cv::inRange(roi, cv::Scalar(160, 50, 50), cv::Scalar(180, 255, 255), redmask2);
        redmask = redmask1 | redmask2;

        std::vector<std::vector<cv::Point>> redcontours;
        cv::findContours(redmask, redcontours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (redcontours.size() >= 2) {
            vector<Point2f> armor_points;
            
            for (size_t k = 0; k < min(redcontours.size(), size_t(2)); k++) {
                cv::Rect red_rect = cv::boundingRect(redcontours[k]);
                
                Point2f top_point(red_rect.x + red_rect.width/2 + boundingRect.x, 
                                 red_rect.y + boundingRect.y);
                Point2f bottom_point(red_rect.x + red_rect.width/2 + boundingRect.x, 
                                    red_rect.y + red_rect.height + boundingRect.y);
                
                armor_points.push_back(top_point);
                armor_points.push_back(bottom_point);
            }
            
            if (armor_points.size() == 4) {
                vector<Point2f> sorted_points = sortArmorCorners(armor_points);
                
                // 计算装甲板中心位置
                cv::Point2f armor_center = (sorted_points[0] + sorted_points[1] + 
                                           sorted_points[2] + sorted_points[3]) / 4.0f;
                
                // 数字区域提取
                string detected_number = "unknown";
                int number_width = 40;
                int number_height = 30;
                
                cv::Rect number_roi_rect(
                    armor_center.x - number_width/2,
                    armor_center.y - number_height/2,
                    number_width,
                    number_height
                );
                
                number_roi_rect = number_roi_rect & cv::Rect(0, 0, image.cols, image.rows);
                
                if (number_roi_rect.width > 10 && number_roi_rect.height > 10) {
                    cv::Mat number_roi = image(number_roi_rect);
                    
                    // // 在图像上标记数字识别区域（黄色矩形）
                //  cv::rectangle(result_image, number_roi_rect, cv::Scalar(0, 255, 255), 2);
                    
                //  使用改进的数字分类器识别中心数字
                     detected_number = classifyNumber(number_roi);
                    
                //     // 在装甲板中心显示识别到的数字
                //      cv::putText(result_image, "Num:" + detected_number,
                //                 cv::Point(armor_center.x - 20, armor_center.y - 20),
                //                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
                 }
                
                // 存储装甲板信息
                ArmorInfo armor;
                armor.number = detected_number;
                armor.corners = sorted_points;
                detected_armors.push_back(armor);
                
                RCLCPP_INFO(this->get_logger(), "Armor detected with number: %s", detected_number.c_str());
            }
        }
    }

    // 第二步：按识别到的数字从小到大排序
    std::sort(detected_armors.begin(), detected_armors.end(), 
        [](const ArmorInfo& a, const ArmorInfo& b) {
            if (a.number != "unknown" && b.number != "unknown") {
                int num_a = std::stoi(a.number);
                int num_b = std::stoi(b.number);
                return num_a < num_b;
            }
            if (a.number != "unknown" && b.number == "unknown") return true;
            if (a.number == "unknown" && b.number != "unknown") return false;
            return false;
        });

    // 第三步：按排序后的顺序进行连续编号
    int global_point_counter = 1;
    for (size_t i = 0; i < detected_armors.size(); i++) {
        vector<Point2f> sorted_points = detected_armors[i].corners;
        string detected_number = detected_armors[i].number;
        
        armor_list.push_back(detected_armors[i]);
        armor_numbers.push_back(detected_number);
        
        // 绘制红色圆圈和数字标签
        for (int j = 0; j < 4; j++) {
            cv::circle(result_image, sorted_points[j], 8, cv::Scalar(0, 0, 255), 2);
            
            string corner_text = to_string(global_point_counter);
            cv::putText(result_image, corner_text,
                       cv::Point(sorted_points[j].x + 10, sorted_points[j].y + 5),
                       cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
            
            Point_V.push_back(sorted_points[j]);
            global_point_counter++;
        }
        
        RCLCPP_INFO(this->get_logger(), "Sorted Armor: Number=%s, Points=%d-%d", 
                   detected_number.c_str(), global_point_counter-4, global_point_counter-1);
    }
    
    cv::imshow("Detection Result", result_image);
    cv::waitKey(1);

    // 发布消息
    if (!armor_list.empty()) {
        referee_pkg::msg::MultiObject msg_object;
        msg_object.header = msg->header;
        msg_object.num_objects = armor_list.size();

        for (int k = 0; k < msg_object.num_objects; k++) {
            referee_pkg::msg::Object obj;
            string number = armor_numbers[k];
            obj.target_type = "armor_red_" + number;

            for (int j = 0; j < 4; j++) {
                geometry_msgs::msg::Point corner;
                corner.x = armor_list[k].corners[j].x;
                corner.y = armor_list[k].corners[j].y;
                corner.z = 0.0;
                obj.corners.push_back(corner);
            }
            msg_object.objects.push_back(obj);
        }

        Target_pub->publish(msg_object);
        RCLCPP_INFO(this->get_logger(), "Published %d armor targets sorted by number", msg_object.num_objects);
    } else {
        RCLCPP_WARN(this->get_logger(), "No armor targets detected");
    }
    
  } catch (const cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
  }
}