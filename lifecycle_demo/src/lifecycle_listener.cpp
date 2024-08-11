#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "lifecycle_msgs/msg/transition_event.hpp"

class LifecycleListener : public rclcpp::Node {
public:
    explicit LifecycleListener(const std::string& node_name) : Node(node_name) {
        // 上下游约定的 topic 名为 lifecycle_chatter
        sub_str_ = this->create_subscription<std_msgs::msg::String>(
            "lifecycle_chatter", 10, 
            [this](std_msgs::msg::String::ConstSharedPtr msg) {return this->str_cb(msg);});

        // 上游 lifecycle 节点名为 lifecycle_talker，因此他一定会附赠一个 lifecycle_talker/transition_event topic
        // 下游订阅这个 topic，可以获取到 lifecycle_talker 节点的状态变化信息
        sub_transition_ = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
            "lifecycle_talker/transition_event", 10,
            [this](lifecycle_msgs::msg::TransitionEvent::ConstSharedPtr msg) {return this->trans_cb(msg);});

        RCLCPP_INFO(this->get_logger(), "LifecycleListener constructor is called");
    }

    void str_cb(std_msgs::msg::String::ConstSharedPtr str_msg) {
        RCLCPP_INFO(this->get_logger(), "str_cb: %s", str_msg->data.c_str());
    }

    void trans_cb(lifecycle_msgs::msg::TransitionEvent::ConstSharedPtr state_msg) {
        RCLCPP_INFO(this->get_logger(), "trans_cb: transition from %s to %s", 
            state_msg->start_state.label.c_str(), state_msg->goal_state.label.c_str());
    }


private:
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> sub_str_;
    std::shared_ptr<rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>> sub_transition_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LifecycleListener>("lifecycle_listener"));
    rclcpp::shutdown();
}