#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include "referee_pkg/srv/hit_armor.hpp"
#include <std_msgs/msg/header.hpp>


using namespace std;
using namespace rclcpp;

class ArmorShooter : public rclcpp::Node {
    public:
    ArmorShooter(string name) : Node(name) {
        RCLCPP_INFO(this->get_logger(), "Initializing ArmorShooter Node");
        Service_= this->create_service<referee_pkg::srv::HitArmor>("/referee/hit_arror",
            std::bind(&ArmorShooter::handle_shooter,this,
            std::placeholders::_1,std::placeholders::_2));
            RCLCPP_INFO(this->get_logger(), "ArmorShooter Node initialized successfully");
        } 
    private:
    rclcpp::Service<referee_pkg::srv::HitArmor>::SharedPtr Service_;

    void handle_shooter(const std::shared_ptr<referee_pkg::srv::HitArmor::Request> request,
                                std::shared_ptr<referee_pkg::srv::HitArmor::Response> response);
    geometry_msgs::msg::Point CalculateCenterPoint(const std::vector<geometry_msgs::msg::Point>& modelpoints);
};
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArmorShooter>("armor_shooter_node");
    RCLCPP_INFO(node->get_logger(), "Starting ArmorShooter Node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

geometry_msgs::msg::Point ArmorShooter::CalculateCenterPoint(const std::vector<geometry_msgs::msg::Point>& modelpoints){
    geometry_msgs::msg::Point center;
    for (auto const& corner : modelpoints){
        center.x += corner.x;
        center.y += corner.y;
        center.z += corner.z;
    }
    center.x /= 4;
    center.y /= 4;
    center.z /= 4;
    return center;
}

void ArmorShooter::handle_shooter(const std::shared_ptr<referee_pkg::srv::HitArmor::Request> request,
                                std::shared_ptr<referee_pkg::srv::HitArmor::Response> response){

RCLCPP_INFO(this->get_logger(), "Received hit armor request");
geometry_msgs::msg::Point center = CalculateCenterPoint(request->modelpoint);
RCLCPP_INFO(this->get_logger(), "Calculated center point: (%.2f, %.2f, %.2f)", center.x, center.y, center.z);

double v0 = 1.5 ; // 初始速度，单位：m/s
double g = request->g;
double h = center.z;
double d = sqrt(center.x * center.x + center.y * center.y);

double a =(g*d*d) / (2*v0*v0) ;
double b =-d;
double c=(g*d*d) / (2*v0*v0) + h;
double delta = b*b-4*a*c;

if (delta < 0) {
    RCLCPP_WARN(this->get_logger(), "No valid shooting angle found.");
    response->yaw = 0.0;
    response->pitch = 0.0;
    response->roll = 0.0;
    return;
}
else {
    double x1 = (-b + sqrt(delta)) / (2 * a);
    double x2 = (-b - sqrt(delta)) / (2 * a);
    
    double t1 = d/v0*cos(atan(x1));
    double t2 = d/v0*cos(atan(x2));
    
    if(t1<t2){
        response-> pitch = atan(x1);
    }
    else{
        response-> pitch = atan(x2);
    }

}
response-> yaw = atan2(center.y, center.x);
response-> roll = 0.0;
}