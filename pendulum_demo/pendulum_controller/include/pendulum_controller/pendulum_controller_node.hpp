#ifndef PENDULUM_CONTROLLER_NODE_HPP_
#define PENDULUM_CONTROLLER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <random>

#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcppmath/clamp.hpp"
#include "rclcpp/strategies/message_pool_memory_strategy.hpp"
#include "rclcpp/strategies/allocator_memory_strategy.hpp"

#include "pendulum_msg/msg/joint_state.hpp"
#include "pendulum_msg/msg/force_cmd.hpp"
#include "pendulum_msg/msg/teleop.hpp"


namespace pendulum_demo {
namespace pendulum_controller {

class PendulumController {
public:

};


class PendulumControllerNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit PendulumControllerNode(const std::string & node_name, 
                                const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    // rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    //     const rclcpp_lifecycle::State& pre_state) override;

    // rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    //     const rclcpp_lifecycle::State& pre_stae) override;

    // rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    //     const rclcpp_lifecycle::State& pre_state) override;

    // rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
    //     const rclcpp_lifecycle::State& pre_state) override;

    // rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
    //     const rclcpp_lifecycle::State& pre_state) override;

private:
    const std::string state_topic_name_;
    const std::string command_topic_name_;
    const std::string teleop_topic_name_;
    bool enable_topic_stats_;
    const std::string topic_stats_topic_name_;
    std::chrono::milliseconds topic_stats_publish_period_;
    std::chrono::milliseconds deadline_duration_;

    PendulumController pcontroller_;

    uint32_t num_missed_deadlines_pub_;
    uint32_t num_missed_deadlines_sub_;

    pendulum_msg::msg::ForceCmd force_cmd_msg_;

    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<pendulum_msg::msg::ForceCmd>> force_cmd_pub_;
    std::shared_ptr<rclcpp::Subscription<pendulum_msg::msg::JointState>> joint_state_sub_;
    std::shared_ptr<rclcpp::Subscription<pendulum_msg::msg::Teleop>> teleop_sub_;
};


}   // namespace pendulum_controller
}   // namespace pendulum_demo


#endif // PENDULUM_CONTROLLER_NODE_HPP_