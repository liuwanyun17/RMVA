#include <rclcpp/rclcpp.hpp>
#include "referee_pkg/msg/race_stage.hpp"

class RaceStageSubscriber : public rclcpp::Node
{
public:
    RaceStageSubscriber() : Node("race_stage_subscriber")
    {
        // 创建订阅者
        subscription_ = this->create_subscription<referee_pkg::msg::RaceStage>(
            "/referee/race_stage",
            10,
            std::bind(&RaceStageSubscriber::race_stage_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "RaceStage 订阅者已启动，等待比赛阶段消息...");
    }

private:
    void race_stage_callback(const referee_pkg::msg::RaceStage::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "收到比赛阶段切换消息: 阶段 %d", msg->stage);
        
        // 根据比赛阶段执行不同的逻辑
        switch(msg->stage) {
            case 1:
                RCLCPP_INFO(this->get_logger(), "进入比赛阶段 1 - 准备阶段");
                handle_stage_1();
                break;
            case 2:
                RCLCPP_INFO(this->get_logger(), "进入比赛阶段 2 - 自瞄阶段");
                handle_stage_2();
                break;
            case 3:
                RCLCPP_INFO(this->get_logger(), "进入比赛阶段 3 - 小能量机关");
                handle_stage_3();
                break;
            case 4:
                RCLCPP_INFO(this->get_logger(), "进入比赛阶段 4 - 大能量机关");
                handle_stage_4();
                break;
            case 5:
                RCLCPP_INFO(this->get_logger(), "进入比赛阶段 5 - 结束阶段");
                handle_stage_5();
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "未知比赛阶段: %d", msg->stage);
                break;
        }
        
        // 保存当前阶段到成员变量
        current_stage_ = msg->stage;
    }

    void handle_stage_1() {
        // 阶段1：准备阶段
        // 可以执行初始化操作
    }

    void handle_stage_2() {
        // 阶段2：自瞄阶段
        // 启动装甲板识别节点
    }

    void handle_stage_3() {
        // 阶段3：小能量机关
        // 启动矩形识别节点
    }

    void handle_stage_4() {
        // 阶段4：大能量机关  
        // 启动球形识别节点
    }

    void handle_stage_5() {
        // 阶段5：结束阶段
        // 停止所有识别节点，清理资源
    }

    // 获取当前比赛阶段（供其他节点调用）
    int get_current_stage() const {
        return current_stage_;
    }

    rclcpp::Subscription<referee_pkg::msg::RaceStage>::SharedPtr subscription_;
    int current_stage_ = 0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RaceStageSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}