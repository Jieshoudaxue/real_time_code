#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "pendulum_controller/pendulum_controller_node.hpp"

namespace pendulum_demo {
namespace pendulum_controller {

// PendulumControllerNode
PendulumControllerNode::PendulumControllerNode(const std::string & node_name, const rclcpp::NodeOptions & options) 
    : LifecycleNode(node_name, options),
    state_topic_name_(declare_parameter<std::string>("state_topic_name", "pendulum_joint_states")),
    command_topic_name_(declare_parameter<std::string>("command_topic_name", "joint_command")),
    teleop_topic_name_(declare_parameter<std::string>("teleop_topic_name", "teleop")),
    enable_topic_stats_(declare_parameter<bool>("enable_topic_stats", false)),
    topic_stats_topic_name_{declare_parameter<std::string>("topic_stats_topic_name", "controller_stats")},
    topic_stats_publish_period_{std::chrono::milliseconds{declare_parameter<std::uint16_t>("topic_stats_publish_period_ms", 1000U)}},
    deadline_duration_{std::chrono::milliseconds{declare_parameter<std::uint16_t>("deadline_duration_ms", 0U)}},
    pcontroller_(PendulumController::ControllerConfig(
        declare_parameter<std::vector<double>>("controller.feedback_matrix", {-10.0000, -51.5393, 356.8637, 154.4146}))),
    num_missed_deadlines_pub_{0U},
    num_missed_deadlines_sub_{0U} {

    // create teleoperation sub
    auto on_pendulum_teleop = [this](const pendulum_msg::msg::Teleop::SharedPtr msg) {
            pcontroller_.set_teleop(msg->cart_position, msg->cart_velocity);
        };
    teleop_sub_ = this->create_subscription<pendulum_msg::msg::Teleop>(
        teleop_topic_name_, rclcpp::QoS(10), on_pendulum_teleop);

    // create command pub
    rclcpp::PublisherOptions command_publisher_options;
    command_publisher_options.event_callbacks.deadline_callback =
        [this](rclcpp::QOSDeadlineOfferedInfo &) -> void {
            num_missed_deadlines_pub_++;
        };
    force_cmd_pub_ = this->create_publisher<pendulum_msg::msg::ForceCmd>(
        command_topic_name_,
        rclcpp::QoS(10).deadline(deadline_duration_),
        command_publisher_options);

    // create joint state sub
    auto on_sensor_message = [this](const pendulum_msg::msg::JointState::SharedPtr msg) {
            // update pendulum state
            pcontroller_.set_state(
                msg->cart_position, msg->cart_velocity,
                msg->pole_angle, msg->pole_velocity);

            // update pendulum controller output
            pcontroller_.update();

            // publish pendulum force command
            force_cmd_msg_.force = pcontroller_.get_force_command();
            force_cmd_pub_->publish(force_cmd_msg_);
        };

    rclcpp::SubscriptionOptions state_subscription_options;
    state_subscription_options.event_callbacks.deadline_callback =
        [this](rclcpp::QOSDeadlineRequestedInfo &) -> void {
            num_missed_deadlines_sub_++;
        };
    if (enable_topic_stats_) {
        state_subscription_options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
        state_subscription_options.topic_stats_options.publish_topic = topic_stats_topic_name_;
        state_subscription_options.topic_stats_options.publish_period = topic_stats_publish_period_;
    }

    using rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy;
    using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;
    auto state_msg_strategy =
        std::make_shared<MessagePoolMemoryStrategy<pendulum_msg::msg::JointState, 1>>();

    joint_state_sub_ = this->create_subscription<pendulum_msg::msg::JointState>(
        state_topic_name_,
        rclcpp::QoS(10).deadline(deadline_duration_),
        on_sensor_message,
        state_subscription_options,
        state_msg_strategy);

    RCLCPP_INFO(this->get_logger(), "PendulumControllerNode constructor");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
PendulumControllerNode::on_configure(const rclcpp_lifecycle::State& pre_state) {
    // 初始化阶段，重置 pcontroller_
    pcontroller_.reset();

    RCLCPP_INFO(this->get_logger(), 
        "LifecycleTalker on_configure is called for initial, pre state is %s", pre_state.label().c_str());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
PendulumControllerNode::on_activate(const rclcpp_lifecycle::State& pre_state) {
    // 告知下游，上游发布节点的 pub 已经激活
    force_cmd_pub_->on_activate();

    RCLCPP_INFO(this->get_logger(), 
        "LifecycleTalker on_activate is called, pre state is %s", pre_state.label().c_str());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
PendulumControllerNode::on_deactivate(const rclcpp_lifecycle::State& pre_state) {
    // 告知下游，上游节点的 pub 已经关闭
    force_cmd_pub_->on_deactivate();

    // 当节点关闭时，打印此时倒立摆的状态
    const auto state = pcontroller_.get_state();
    const auto teleoperation_command = pcontroller_.get_teleop();
    const double force_command = pcontroller_.get_force_command();

    RCLCPP_INFO(get_logger(), "Cart position = %lf", state.at(0));
    RCLCPP_INFO(get_logger(), "Cart velocity = %lf", state.at(1));
    RCLCPP_INFO(get_logger(), "Pole angle = %lf", state.at(2));
    RCLCPP_INFO(get_logger(), "Pole angular velocity = %lf", state.at(3));
    RCLCPP_INFO(get_logger(), "Teleoperation cart position = %lf", teleoperation_command.at(0));
    RCLCPP_INFO(get_logger(), "Teleoperation cart velocity = %lf", teleoperation_command.at(1));
    RCLCPP_INFO(get_logger(), "Force command = %lf", force_command);
    RCLCPP_INFO(get_logger(), "Publisher missed deadlines = %u", num_missed_deadlines_pub_);
    RCLCPP_INFO(get_logger(), "Subscription missed deadlines = %u", num_missed_deadlines_sub_);

    RCLCPP_INFO(this->get_logger(), 
        "LifecycleTalker on_deactivate is called, pre state is %s", pre_state.label().c_str());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
PendulumControllerNode::on_cleanup(const rclcpp_lifecycle::State& pre_state) {

    RCLCPP_INFO(this->get_logger(), 
        "LifecycleTalker on_cleanup is called, pre state is %s", pre_state.label().c_str());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumControllerNode::on_shutdown(const rclcpp_lifecycle::State& pre_state) {

    RCLCPP_INFO(this->get_logger(), 
        "LifecycleTalker on_shutdown is called, pre state is %s", pre_state.label().c_str());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


} // namespace pendulum_controller
} // namespace pendulum_demo


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    using pendulum_demo::pendulum_controller::PendulumControllerNode;
    auto controller_node = std::make_shared<PendulumControllerNode>("pendulum_controller_node");

    exe.add_node(controller_node->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();

    return 0;
}