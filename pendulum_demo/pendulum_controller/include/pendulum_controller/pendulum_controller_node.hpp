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
    class ControllerConfig {
    public:
        explicit ControllerConfig(std::vector<double> feedback_matrix) 
        : feedback_matrix_(std::move(feedback_matrix)) {}

        const std::vector<double> & get_feedback_matrix() const {
            return feedback_matrix_;
        }
    private:
        std::vector<double> feedback_matrix_;
    };

public:
    explicit PendulumController(const ControllerConfig & config)
    : cconfig_(config),
      state_{0.0, 0.0, M_PI, 0.0},
      reference_{0.0, 0.0, M_PI, 0.0} {}

    void reset() {
        set_state(0.0, 0.0, M_PI, 0.0);
        set_teleop(0.0, 0.0, M_PI, 0.0);
    }

    void update() {
        set_force_command(calculate(state_, reference_));
    }

    void set_teleop(double cart_pos, double cart_vel,
                    double pole_pos, double pole_vel) {
        reference_[0] = cart_pos;
        reference_[1] = cart_vel;
        reference_[2] = pole_pos;
        reference_[3] = pole_vel;
    }

    void set_teleop(double cart_pos, double cart_vel) {
        reference_[0] = cart_pos;
        reference_[1] = cart_vel;
    }

    void set_state(double cart_pos, double cart_vel, double pole_pos, double pole_vel) {
        state_ = {cart_pos, cart_vel, pole_pos, pole_vel};
    }

    void set_force_command(double force) {
        force_command_ = force;
    }

    std::vector<double> get_state() const {
        return state_;
    }

    std::vector<double> get_teleop() const {
        return reference_;
    }

    double get_force_command() const {
        return force_command_;
    }

private:
    double calculate(const std::vector<double> & state, const std::vector<double> & reference) const {
        double controller_output = 0.0;
        size_t dim = state.size();
        if ((dim != reference.size()) &&
            (dim != cconfig_.get_feedback_matrix().size()))
        {
            throw std::invalid_argument("wrong state size vector");
        }

        for (size_t i = 0; i < dim; i++) {
            controller_output += -cconfig_.get_feedback_matrix()[i] * (state[i] - reference[i]);
        }

        return controller_output;
    }

private:
    const ControllerConfig cconfig_;
    std::vector<double> state_;
    std::vector<double> reference_;
    double force_command_;
};


class PendulumControllerNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit PendulumControllerNode(const std::string & node_name, 
                                const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& pre_state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& pre_stae) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& pre_state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State& pre_state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State& pre_state) override;

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

    std::shared_ptr<rclcpp::Subscription<pendulum_msg::msg::Teleop>> teleop_sub_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<pendulum_msg::msg::ForceCmd>> force_cmd_pub_;
    pendulum_msg::msg::ForceCmd force_cmd_msg_;
    std::shared_ptr<rclcpp::Subscription<pendulum_msg::msg::JointState>> joint_state_sub_;
};


}   // namespace pendulum_controller
}   // namespace pendulum_demo


#endif // PENDULUM_CONTROLLER_NODE_HPP_