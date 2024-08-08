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
    explicit LifecycleTalker(const std::string& node_name, bool is_intra_process = false) : 
                            rclcpp_lifecycle::LifecycleNode(node_name, 
                            rclcpp::NodeOptions().use_intra_process_comms(is_intra_process)) {
        RCLCPP_INFO(this->get_logger(), "LifecycleTalker constructor is called");
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& pre_state) {
        pub_str_ = this->create_publisher<std_msgs::msg::String>("lifecycle_chatter", 10);
        lc_timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() {return this->publish();});
        count_ = 0;

        RCLCPP_INFO(this->get_logger(), 
            "LifecycleTalker on_configure is called for initial, pre state is %s", pre_state.label().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    void publish() {
        auto str_msg = std::make_unique<std_msgs::msg::String>();
        str_msg->data = "lifecycle say hello " + std::to_string(count_++);

        if (true == pub_str_->is_activated()) {
            RCLCPP_INFO(this->get_logger(), "LifecycleTalker publish: %s", str_msg->data.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "LifecycleTalker is deactivated");
        }

        pub_str_->publish(std::move(str_msg));
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& pre_state) {
        LifecycleNode::on_activate(pre_state);

        RCLCPP_INFO(this->get_logger(), 
            "LifecycleTalker on_activate is called, pre state is %s", pre_state.label().c_str());

        std::this_thread::sleep_for(std::chrono::seconds(2));

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& pre_state) {
        LifecycleNode::on_deactivate(pre_state);

        RCLCPP_INFO(this->get_logger(), 
            "LifecycleTalker on_deactivate is called, pre state is %s", pre_state.label().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }    

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State& pre_state) {
        pub_str_.reset();
        lc_timer_.reset();

        RCLCPP_INFO(this->get_logger(), 
            "LifecycleTalker on_cleanup is called, pre state is %s", pre_state.label().c_str());

        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }  

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State& pre_state) {
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

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<LifecycleTalker> lc_talker = std::make_shared<LifecycleTalker>("lifecycle_talker");

    exe.add_node(lc_talker->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();

    return 0;
}