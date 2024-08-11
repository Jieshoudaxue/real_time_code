#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "std_msgs/msg/string.hpp"

class LifecycleTalker : public rclcpp_lifecycle::LifecycleNode {
public:
    // 通常情况下，不推荐在构造函数内进行资源申请，有可能失败，但又无法获取返回值
    // 如果这里是普通的 node，初始化部分应放在单独的 init 函数中，并根据返回值判断是否初始化成功
    explicit LifecycleTalker(const std::string& node_name, bool is_intra_process = false) : 
                            rclcpp_lifecycle::LifecycleNode(node_name, 
                            rclcpp::NodeOptions().use_intra_process_comms(is_intra_process)) {
        RCLCPP_INFO(this->get_logger(), "LifecycleTalker constructor is called");
    }

    // 这是 configuring 过渡状态的回调函数，用于初始化节点，完成后将从 unconfigured 进入 inactive 状态
    // 这里将完成程序的初始化，申请各种资源，创建发布器和定时器
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& pre_state) {
        pub_str_ = this->create_publisher<std_msgs::msg::String>("lifecycle_chatter", 10);
        lc_timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() {return this->publish();});
        count_ = 0;

        RCLCPP_INFO(this->get_logger(), 
            "LifecycleTalker on_configure is called for initial, pre state is %s", pre_state.label().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    // 这是定时器的回调函数，也是节点处于 active 状态时的主要工作函数，理论上不应该有 log ，应是一个纯粹的工作函数
    void publish() {
        auto str_msg = std::make_unique<std_msgs::msg::String>();
        str_msg->data = "lifecycle say hello " + std::to_string(count_++);

        if (true == pub_str_->is_activated()) {
            RCLCPP_INFO(this->get_logger(), "LifecycleTalker publish: %s", str_msg->data.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "LifecycleTalker is deactivated");
        }

        // 如果节点不处于 active 状态，即使调用了 publish ，也不会发布消息，lifecycle_listener 的 log 会证明这一点
        // 这是 lifecycle node 非常巧妙的一个设计，可以实现在线启动和关停节点
        pub_str_->publish(std::move(str_msg));
    }

    // 这是 activating 过渡状态的回调函数，完成后将从 inactive 进入 active 状态
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& pre_state) {
        LifecycleNode::on_activate(pre_state);

        RCLCPP_INFO(this->get_logger(), 
            "LifecycleTalker on_activate is called, pre state is %s", pre_state.label().c_str());

        std::this_thread::sleep_for(std::chrono::seconds(2));

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    // 这是 deactivating 过渡状态的回调函数，完成后将从 active 返回 inactive 状态
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& pre_state) {
        LifecycleNode::on_deactivate(pre_state);

        RCLCPP_INFO(this->get_logger(), 
            "LifecycleTalker on_deactivate is called, pre state is %s", pre_state.label().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }    

    // 这是 cleaning 过渡状态的回调函数，完成后将从 inactive 返回 unconfigured 状态
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State& pre_state) {
        // 释放资源，清空智能指针
        pub_str_.reset();
        lc_timer_.reset();

        RCLCPP_INFO(this->get_logger(), 
            "LifecycleTalker on_cleanup is called, pre state is %s", pre_state.label().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }  

    // 这是 shuttingdown 过渡状态的回调函数，完成后将进入 finalized 状态
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State& pre_state) {
        // 释放资源，清空智能指针
        pub_str_.reset();
        lc_timer_.reset();

        RCLCPP_INFO(this->get_logger(), 
            "LifecycleTalker on_shutdown is called, pre state is %s", pre_state.label().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }  

private:
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_str_;
    std::shared_ptr<rclcpp::TimerBase> lc_timer_;
    size_t count_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    // 单线程执行器，用于 lifecycle_talker 执行，其在单个线程顺序地执行所有回调函数，
    // 满足 lifecycle node 对状态转换必须严格按照顺序执行的要求
    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<LifecycleTalker> lc_talker = std::make_shared<LifecycleTalker>("lifecycle_talker");

    exe.add_node(lc_talker->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();

    return 0;
}