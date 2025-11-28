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
#include <algorithm>

using namespace std;
using namespace rclcpp;
using namespace cv;

// 数字模板匹配类
class DigitTemplateMatcher {
private:
    std::map<int, cv::Mat> templates;
    bool debug_mode;
    double confidence_threshold;
    
public:
    DigitTemplateMatcher() : debug_mode(true), confidence_threshold(0.6) {}
    
    bool loadTemplates(const std::string& template_dir) {
        templates.clear();
        
        auto logger = rclcpp::get_logger("DigitTemplateMatcher");
        
        for (int digit = 1; digit <= 5; digit++) {
            std::string path = template_dir + "/" + std::to_string(digit) + ".jpg";
            cv::Mat template_img = cv::imread(path, cv::IMREAD_GRAYSCALE);
            
            if (template_img.empty()) {
                RCLCPP_WARN(logger, "无法加载模板: %s", path.c_str());
                continue;
            }
            
        cv::Mat resized_template;
        cv::Size target_size(40, 60); // 标准尺寸：宽40，高60
        
        cv::resize(template_img, resized_template, target_size);

            // 预处理模板：二值化
            cv::Mat binary;
            cv::threshold(resized_template, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
            templates[digit] = binary;
            
            RCLCPP_INFO(logger, "加载数字 %d 模板, 尺寸: %dx%d", 
                       digit, binary.cols, binary.rows);
            
            // 调试模式下显示模板
            if (debug_mode) {
                cv::imshow("Template " + std::to_string(digit), binary);
            }
        }
        
        bool success = !templates.empty();
        if (success) {
            RCLCPP_INFO(logger, "成功加载 %d 个数字模板", templates.size());
        } else {
            RCLCPP_ERROR(logger, "未加载任何模板");
        }
        
        return success;
    }
    
    std::string recognize(const cv::Mat& roi) {
        if (roi.empty()) {
            return "unknown";
        }
        
        if (templates.empty()) {
            return "unknown";
        }
        
        //转化为灰度图
        cv::Mat gray;
        if (roi.channels() == 3) {
            cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = roi.clone();
        }
        
        // 调试：显示原始ROI
        if (debug_mode) {
            cv::imshow("Original ROI", gray);
        }
        
        int best_digit = -1;
        double best_score = confidence_threshold;

        // 遍历所有模板进行匹配
        for (const auto& [digit, template_img] : templates) {
            cv::Mat resized_roi;
            cv::resize(gray, resized_roi, template_img.size());
            
            // 对ROI也进行二值化
            cv::Mat roi_binary;
            cv::threshold(resized_roi, roi_binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
            
            // 调试：显示处理后的ROI
            if (debug_mode && digit == 1) {
                cv::imshow("Resized ROI", resized_roi);
                cv::imshow("Binary ROI", roi_binary);
            }
            
            cv::Mat result;
            cv::matchTemplate(roi_binary, template_img, result, cv::TM_CCOEFF_NORMED);
            
            double minVal, maxVal;
            cv::Point minLoc, maxLoc;
            cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc);
            
            RCLCPP_DEBUG(rclcpp::get_logger("DigitTemplateMatcher"), 
                        "数字 %d 匹配分数: %.3f", digit, maxVal);
            
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

    // 初始化数字识别器
    std::string template_path = "./digit_templates"; // 模板目录路径
    if (!digit_matcher.loadTemplates(template_path)) {
        RCLCPP_WARN(this->get_logger(), "数字模板加载失败，将使用默认分类");
    }

    Image_sub = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        bind(&MultiArmorNode::callback_camera, this, std::placeholders::_1));

    Target_pub = this->create_publisher<referee_pkg::msg::MultiObject>(
        "/vision/target", 10);

    cv::namedWindow("Detection Result", cv::WINDOW_AUTOSIZE);
    RCLCPP_INFO(this->get_logger(), "MultiArmorNode initialized successfully");
  }

  ~MultiArmorNode() { 
    cv::destroyWindow("Detection Result");
    cv::destroyWindow("Digit ROI");
    cv::destroyWindow("Original ROI");
    cv::destroyWindow("Resized ROI");
    cv::destroyWindow("Binary ROI");
  }

 private:
  void callback_camera(sensor_msgs::msg::Image::SharedPtr msg);
  vector<Point2f> sortArmorCorners(const vector<Point2f>& corners);
  string classifyNumber(const cv::Mat& roi);
  string classifyByWhiteRatio(const cv::Mat& roi); // 备用分类方法

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr Image_sub;
  rclcpp::Publisher<referee_pkg::msg::MultiObject>::SharedPtr Target_pub;
  vector<Point2f> Point_V;
  DigitTemplateMatcher digit_matcher; // 数字识别器
  
  // 装甲板信息结构体
  struct ArmorInfo {
      string number;
      vector<Point2f> corners;
  };
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

string MultiArmorNode::classifyNumber(const cv::Mat& roi) {
    // 首先尝试模板匹配
    string result = digit_matcher.recognize(roi);
    if (result != "unknown") {
        return result;
    }
    
    // 如果模板匹配失败，使用白色比例作为备用方案
    return classifyByWhiteRatio(roi);
}

string MultiArmorNode::classifyByWhiteRatio(const cv::Mat& roi) {
    if (roi.empty()) return "unknown";
    
    cv::Mat gray, binary;
    if (roi.channels() == 3) {
        cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = roi.clone();
    }
    
    // 自适应阈值
    cv::adaptiveThreshold(gray, binary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, 
                         cv::THRESH_BINARY, 11, 2);
    
    // 计算白色像素比例
    int white_pixels = cv::countNonZero(binary);
    double white_ratio = (double)white_pixels / (binary.rows * binary.cols);
    
    RCLCPP_DEBUG(this->get_logger(), "White ratio: %.3f", white_ratio);
    
    // 基于白色比例分类
    if (white_ratio < 0.1 || white_ratio > 0.6) {
        return "unknown";
    }
    else if (white_ratio < 0.18) {
        return "1";
    }
    else if (white_ratio < 0.22) {
        return "3";
    }
    else if (white_ratio < 0.36) {
        return "2";
    }
    else if (white_ratio < 0.42) {
        return "4";
    }
    else if (white_ratio < 0.50) {
        return "5";
    }
    else {
        return "unknown";
    }
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
    cv::inRange(hsv, cv::Scalar(0, 50, 50), cv::Scalar(20, 255, 255), mask1);
    cv::inRange(hsv, cv::Scalar(160, 50, 50), cv::Scalar(180, 255, 255), mask2);
    mask_wide = mask1 | mask2;
    
    cv::Mat mask3;
    cv::inRange(hsv,cv::Scalar(0,0,0),cv::Scalar(180,255,50),mask3);
    cv::imshow("mask3",mask3);
    Mat mask_total = mask_wide | mask3;
    cv::imshow("mask_total",mask_total);
    
    // 形态学操作
    cv::Mat mask_for_contours = mask_total.clone();
    cv::Mat kernel_dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::dilate(mask_for_contours, mask_for_contours, kernel_dilate);
    cv::imshow("mask_for_contours",mask_for_contours);
    cv::Mat kernel_close = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15));
    cv::morphologyEx(mask_for_contours, mask_for_contours, cv::MORPH_CLOSE, kernel_close);
    
    // 找轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask_for_contours, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    RCLCPP_INFO(this->get_logger(), "Found %ld contours", contours.size());

    Point_V.clear();
    
    std::vector<ArmorInfo> detected_armors; // 存储检测到的装甲板

    int point_counter = 1;

    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area < 10) continue;

        cv::Rect boundingRect = cv::boundingRect(contours[i]);
        
        // 长宽比筛选
        double aspect_ratio = (double)boundingRect.width / boundingRect.height;
        if (aspect_ratio < 1.5 || aspect_ratio > 4.0) continue;
        
        cv::Mat roi = hsv(boundingRect); 
        cv::Mat redmask1, redmask2, redmask;
        cv::inRange(roi, cv::Scalar(0, 50, 50), cv::Scalar(20, 255, 255), redmask1);
        cv::inRange(roi, cv::Scalar(160, 50, 50), cv::Scalar(180, 255, 255), redmask2);
        redmask = redmask1 | redmask2;

        std::vector<std::vector<cv::Point>> redcontours;
        cv::findContours(redmask, redcontours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        std::vector<cv::Point2f> all_points;
        // 检测灯条并构建装甲板
        for (size_t k = 0; k < min(redcontours.size(), size_t(2)); k++) {
            cv::Rect red_rect = cv::boundingRect(redcontours[k]);
            
            Point2f top_point(red_rect.x + red_rect.width/2 + boundingRect.x, 
                             red_rect.y + boundingRect.y);
            Point2f bottom_point(red_rect.x + red_rect.width/2 + boundingRect.x, 
                                red_rect.y + red_rect.height + boundingRect.y);
            
            all_points.push_back(top_point);
            all_points.push_back(bottom_point);
        }
        RCLCPP_INFO(this->get_logger(),"检测到 %ld 个点",all_points.size());
        // 如果找到4个点（两个灯条），构建装甲板
        if (all_points.size() == 4) {
            vector<Point2f> sorted_points = sortArmorCorners(all_points);
            
            // 计算装甲板中心位置
            cv::Point2f armor_center = (sorted_points[0] + sorted_points[1] + 
                                       sorted_points[2] + sorted_points[3]) / 4.0f;
            
            // 数字区域提取
            string detected_number = "unknown";
            int number_width = 0.6*boundingRect.width;
            int number_height = 0.9*boundingRect.height;
            
            cv::Rect number_roi_rect(
                armor_center.x - number_width/2,
                armor_center.y - number_height/2,
                number_width,
                number_height
            );
            
            number_roi_rect = number_roi_rect & cv::Rect(0, 0, image.cols, image.rows);
            
            cv::Mat number_roi = image(number_roi_rect);
                
            // 数字识别
            detected_number = classifyNumber(number_roi);
                
            // 在图像上标记数字识别区域和结果
            cv::rectangle(result_image, number_roi_rect, cv::Scalar(0, 255, 255), 2);
            cv::putText(result_image, "Num:" + detected_number,
                        cv::Point(armor_center.x - 20, armor_center.y - 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
                
                // 调试：显示数字ROI
            cv::imshow("Digit ROI", number_roi);
            
            
            // 存储装甲板信息
            ArmorInfo armor;
            armor.number = detected_number;
            armor.corners = sorted_points;
            detected_armors.push_back(armor);
            
            // 绘制装甲板角点
            for (int j = 0; j < 4; j++) {
                cv::circle(result_image, sorted_points[j], 8, cv::Scalar(0, 0, 255), 2);
                string corner_text = to_string(point_counter);
                cv::putText(result_image, corner_text,
                           cv::Point(sorted_points[j].x + 10, sorted_points[j].y + 5),
                           cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
                Point_V.push_back(sorted_points[j]);
                point_counter++;
            }
            
            RCLCPP_INFO(this->get_logger(), "Armor detected with number: %s", detected_number.c_str());
        }
        
        all_points.clear();
    }
    
    cv::imshow("Detection Result", result_image);
    cv::waitKey(1);

    // 发布消息
    if (!detected_armors.empty()) {
        referee_pkg::msg::MultiObject msg_object;
        msg_object.header = msg->header;
        msg_object.num_objects = detected_armors.size();
        
        // 按数字排序装甲板
        std::sort(detected_armors.begin(), detected_armors.end(), 
            [](const ArmorInfo& a, const ArmorInfo& b) {
                if (a.number != "unknown" && b.number != "unknown") {
                    return std::stoi(a.number) < std::stoi(b.number);
                }
                return a.number != "unknown"; // 已知数字排在前面
            });
        
        vector<string> types = {"armor_red_1","armor_red_2","armor_red_3","armor_red_4","armor_red_5"};

        for (size_t k = 0; k < detected_armors.size(); k++) {
            referee_pkg::msg::Object obj;
            string number = detected_armors[k].number;
            
            // 根据识别到的数字设置类型
            if (number != "unknown" && std::stoi(number) <= types.size()) {
                obj.target_type = types[std::stoi(number) - 1];
            } else {
                obj.target_type = "armor_red_unknown";
            }

            for (int j = 0; j < 4; j++) {
                geometry_msgs::msg::Point corner;
                corner.x = detected_armors[k].corners[j].x;
                corner.y = detected_armors[k].corners[j].y;
                corner.z = 0.0;
                obj.corners.push_back(corner);
            }
            msg_object.objects.push_back(obj);
        }

        Target_pub->publish(msg_object);
        RCLCPP_INFO(this->get_logger(), "Published %d armor targets with number recognition", 
                   msg_object.num_objects);
    } else {
        RCLCPP_WARN(this->get_logger(), "No armor targets detected");
    }
    
  } catch (const cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
  }
}
// #include <cv_bridge/cv_bridge.h>
// #include <cmath>
// #include <geometry_msgs/msg/point.hpp>
// #include <memory>
// #include <opencv2/core.hpp>
// #include <opencv2/opencv.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp/timer.hpp>
// #include <referee_pkg/msg/multi_object.hpp>
// #include <referee_pkg/msg/object.hpp>
// #include <sensor_msgs/image_encodings.hpp>
// #include <std_msgs/msg/header.hpp>
// #include "sensor_msgs/msg/image.hpp"
// #include <opencv2/dnn.hpp> 

// using namespace std;
// using namespace rclcpp;
// using namespace cv;

// class MutiArmorNode : public rclcpp::Node {
//  public:
//   MutiArmorNode(string name) : Node(name) {
    
//     RCLCPP_INFO(this->get_logger(), "Initializing MutiArmorNode");

//     Image_sub = this->create_subscription<sensor_msgs::msg::Image>(
//         "/camera/image_raw", 10,
//         bind(&MutiArmorNode::callback_camera, this, std::placeholders::_1));

//     Target_pub = this->create_publisher<referee_pkg::msg::MultiObject>(
//         "/vision/target", 10);

//     cv::namedWindow("Detection Result", cv::WINDOW_AUTOSIZE);
//     RCLCPP_INFO(this->get_logger(), "MutiArmorNode initialized successfully");
//   }

//   ~MutiArmorNode() { cv::destroyWindow("Detection Result"); }

//  private:
//   rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr Image_sub;
//   rclcpp::Publisher<referee_pkg::msg::MultiObject>::SharedPtr Target_pub;
//   vector<Point2f> Point_V;
  
//   struct ArmorInfo {
//       string number;
//       vector<Point2f> corners;
//   };
//   vector<ArmorInfo> armor_list;
//   vector<string> armor_numbers;
  
//   void callback_camera(sensor_msgs::msg::Image::SharedPtr msg);
//   vector<Point2f> sortArmorCorners(const vector<Point2f>& corners);
//   string classifyNumber(const cv::Mat& roi);
// };

// int main(int argc, char **argv) {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<MutiArmorNode>("MutiArmorNode");
//   RCLCPP_INFO(node->get_logger(), "Starting MutiArmorNode");
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }

// vector<Point2f> MutiArmorNode::sortArmorCorners(const vector<Point2f>& corners) {
//     vector<Point2f> sorted(4);
//     Point2f center(0,0);
//     for (const auto& corner : corners) {
//         center += corner;
//     }
//     center /= 4.0;
//     for (const auto& corner : corners) {
//         if (corner.x < center.x && corner.y > center.y) {
//             sorted[0] = corner;
//         } else if (corner.x > center.x && corner.y > center.y) {
//             sorted[1] = corner;
//         } else if (corner.x > center.x && corner.y < center.y) {
//             sorted[2] = corner;
//         } else if (corner.x < center.x && corner.y < center.y) {
//             sorted[3] = corner;
//         }
//     }
//     return sorted;
// }

// string MutiArmorNode::classifyNumber(const cv::Mat& roi) {
//     if (roi.empty()) return "unknown";
    
//     // 最简单的预处理
//     cv::Mat gray;
//     if (roi.channels() == 3) {
//         cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
//     } else {
//         gray = roi.clone();
//     }
    
//     // 固定阈值提取白色数字
//     cv::Mat binary;
//     cv::threshold(gray, binary, 200, 255, cv::THRESH_BINARY);
    
//     // 计算白色像素比例
//     int white_pixels = cv::countNonZero(binary);
//     double white_ratio = (double)white_pixels / (binary.rows * binary.cols);
    
//     // 只基于白色比例分类
//     if (white_ratio < 0.1 || white_ratio > 0.6) {
//         return "unknown";
//     }
//     else if (white_ratio < 0.24) {
//         return "1";  // 数字1最细，白色最少
//     }
//     else if (white_ratio < 0.30) {
//         return "3";  // 数字3
//     }
//     else if (white_ratio < 0.36) {
//         return "2";  // 数字2
//     }
//     else if (white_ratio < 0.42) {
//         return "4";  // 数字4
//     }
//     else if (white_ratio < 0.50) {
//         return "5";  // 数字5
//     }
//     else {
//         return "unknown";
//     }
// }

// void MutiArmorNode::callback_camera(sensor_msgs::msg::Image::SharedPtr msg) {
//   try {
//     cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//     cv::Mat image = cv_ptr->image;

//     if (image.empty()) {
//       RCLCPP_WARN(this->get_logger(), "Received empty image");
//       return;
//     }

//     cv::Mat result_image = image.clone();
    
//     // 保持原有的图像处理流程
//     cv::Mat blurred;
//     cv::GaussianBlur(image, blurred, cv::Size(15, 15), 0);
    
//     cv::Mat hsv;
//     cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);
    
//     cv::Mat mask1, mask2, mask_wide;
//     cv::inRange(hsv, cv::Scalar(0, 50, 50), cv::Scalar(20, 255, 255), mask1);
//     cv::inRange(hsv, cv::Scalar(160, 50, 50), cv::Scalar(180, 255, 255), mask2);
//     mask_wide = mask1 | mask2;
    
//     cv::Mat mask3;
//     cv::inRange(hsv,cv::Scalar(0,0,0),cv::Scalar(180,255,50),mask3);
//     Mat mask_total = mask_wide | mask3;
    
//     cv::Mat mask_for_contours = mask_total.clone();
//     cv::Mat kernel_dilate = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
//     cv::dilate(mask_for_contours, mask_for_contours, kernel_dilate);
//     cv::Mat kernel_close = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15));
//     cv::morphologyEx(mask_for_contours, mask_for_contours, cv::MORPH_CLOSE, kernel_close);
    
//     std::vector<std::vector<cv::Point>> contours;
//     cv::findContours(mask_for_contours, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
//     RCLCPP_INFO(this->get_logger(), "Found %ld contours", contours.size());

//     Point_V.clear();
//     armor_numbers.clear();
//     armor_list.clear();
    
//     vector<ArmorInfo> detected_armors;

//     // 第一步：检测所有装甲板并识别中心数字
//     for (size_t i = 0; i < contours.size(); i++) {
//         double area = cv::contourArea(contours[i]);
//         if (area < 100) continue;

//         cv::Rect boundingRect = cv::boundingRect(contours[i]);
        
//         double aspect_ratio = (double)boundingRect.width / boundingRect.height;
//         if (aspect_ratio < 1.5 || aspect_ratio > 4.0) continue;
        
//         cv::Mat roi = hsv(boundingRect); 
//         cv::Mat redmask1, redmask2, redmask;
    
//         cv::inRange(roi, cv::Scalar(0, 50, 50), cv::Scalar(20, 255, 255), redmask1);
//         cv::inRange(roi, cv::Scalar(160, 50, 50), cv::Scalar(180, 255, 255), redmask2);
//         redmask = redmask1 | redmask2;

//         std::vector<std::vector<cv::Point>> redcontours;
//         cv::findContours(redmask, redcontours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

//         if (redcontours.size() >= 2) {
//             vector<Point2f> armor_points;
            
//             for (size_t k = 0; k < min(redcontours.size(), size_t(2)); k++) {
//                 cv::Rect red_rect = cv::boundingRect(redcontours[k]);
                
//                 Point2f top_point(red_rect.x + red_rect.width/2 + boundingRect.x, 
//                                  red_rect.y + boundingRect.y);
//                 Point2f bottom_point(red_rect.x + red_rect.width/2 + boundingRect.x, 
//                                     red_rect.y + red_rect.height + boundingRect.y);
                
//                 armor_points.push_back(top_point);
//                 armor_points.push_back(bottom_point);
//             }
            
//             if (armor_points.size() == 4) {
//                 vector<Point2f> sorted_points = sortArmorCorners(armor_points);
                
//                 // 计算装甲板中心位置
//                 cv::Point2f armor_center = (sorted_points[0] + sorted_points[1] + 
//                                            sorted_points[2] + sorted_points[3]) / 4.0f;
                
//                 // 数字区域提取
//                 string detected_number = "unknown";
//                 int number_width = 40;
//                 int number_height = 30;
                
//                 cv::Rect number_roi_rect(
//                     armor_center.x - number_width/2,
//                     armor_center.y - number_height/2,
//                     number_width,
//                     number_height
//                 );
                
//                 number_roi_rect = number_roi_rect & cv::Rect(0, 0, image.cols, image.rows);
                
//                 if (number_roi_rect.width > 10 && number_roi_rect.height > 10) {
//                     cv::Mat number_roi = image(number_roi_rect);
                    
//                     // // 在图像上标记数字识别区域（黄色矩形）
//                  cv::rectangle(result_image, number_roi_rect, cv::Scalar(0, 255, 255), 2);
                    
//                 //  使用改进的数字分类器识别中心数字
//                      detected_number = classifyNumber(number_roi);
                    
//                 //     // 在装甲板中心显示识别到的数字
//                      cv::putText(result_image, "Num:" + detected_number,
//                                 cv::Point(armor_center.x - 20, armor_center.y - 20),
//                                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
//                  }
                
//                 // 存储装甲板信息
//                 ArmorInfo armor;
//                 armor.number = detected_number;
//                 armor.corners = sorted_points;
//                 detected_armors.push_back(armor);
                
//                 RCLCPP_INFO(this->get_logger(), "Armor detected with number: %s", detected_number.c_str());
//             }
//         }
//     }

//     // 第二步：按识别到的数字从小到大排序
//     std::sort(detected_armors.begin(), detected_armors.end(), 
//         [](const ArmorInfo& a, const ArmorInfo& b) {
//             if (a.number != "unknown" && b.number != "unknown") {
//                 int num_a = std::stoi(a.number);
//                 int num_b = std::stoi(b.number);
//                 return num_a < num_b;
//             }
//             if (a.number != "unknown" && b.number == "unknown") return true;
//             if (a.number == "unknown" && b.number != "unknown") return false;
//             return false;
//         });

//     // 第三步：按排序后的顺序进行连续编号
//     int global_point_counter = 1;
//     for (size_t i = 0; i < detected_armors.size(); i++) {
//         vector<Point2f> sorted_points = detected_armors[i].corners;
//         string detected_number = detected_armors[i].number;
        
//         armor_list.push_back(detected_armors[i]);
//         armor_numbers.push_back(detected_number);
        
//         // 绘制红色圆圈和数字标签
//         for (int j = 0; j < 4; j++) {
//             cv::circle(result_image, sorted_points[j], 8, cv::Scalar(0, 0, 255), 2);
            
//             string corner_text = to_string(global_point_counter);
//             cv::putText(result_image, corner_text,
//                        cv::Point(sorted_points[j].x + 10, sorted_points[j].y + 5),
//                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
            
//             Point_V.push_back(sorted_points[j]);
//             global_point_counter++;
//         }
        
//         RCLCPP_INFO(this->get_logger(), "Sorted Armor: Number=%s, Points=%d-%d", 
//                    detected_number.c_str(), global_point_counter-4, global_point_counter-1);
//     }
    
//     cv::imshow("Detection Result", result_image);
//     cv::waitKey(1);

//     // 发布消息
//     if (!armor_list.empty()) {
//         referee_pkg::msg::MultiObject msg_object;
//         msg_object.header = msg->header;
//         msg_object.num_objects = armor_list.size();

//         for (int k = 0; k < msg_object.num_objects; k++) {
//             referee_pkg::msg::Object obj;
//             string number = armor_numbers[k];
//             obj.target_type = "armor_red_" + number;

//             for (int j = 0; j < 4; j++) {
//                 geometry_msgs::msg::Point corner;
//                 corner.x = armor_list[k].corners[j].x;
//                 corner.y = armor_list[k].corners[j].y;
//                 corner.z = 0.0;
//                 obj.corners.push_back(corner);
//             }
//             msg_object.objects.push_back(obj);
//         }

//         Target_pub->publish(msg_object);
//         RCLCPP_INFO(this->get_logger(), "Published %d armor targets sorted by number", msg_object.num_objects);
//     } else {
//         RCLCPP_WARN(this->get_logger(), "No armor targets detected");
//     }
    
//   } catch (const cv_bridge::Exception &e) {
//     RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
//   } catch (const std::exception &e) {
//     RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
//   }
// }