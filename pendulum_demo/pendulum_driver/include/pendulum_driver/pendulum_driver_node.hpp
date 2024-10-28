#ifndef PENDULUM_DRIVER_NODE_HPP_
#define PENDULUM_DRIVER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <random>

#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcppmath/clamp.hpp"

#include "pendulum_msg/msg/joint_state.hpp"
#include "pendulum_msg/msg/force_cmd.hpp"


namespace pendulum_demo {
namespace pendulum_driver {

static constexpr std::size_t STATE_DIMENSION = 4U;

struct PendulumState {
    double cart_position = 0.0;
    double cart_velocity = 0.0;
    double pole_angle = M_PI;
    double pole_velocity = 0.0;
    double cart_force = 0.0;
};

class PendulumConfig {
public:
    PendulumConfig(
        double pendulum_mass,
        double cart_mass,
        double pendulum_length,
        double damping_coefficient,
        double gravity,
        double max_cart_force,
        double noise_level,
        std::chrono::microseconds physics_update_period);

    double get_pendulum_mass() const;

    double get_cart_mass() const;

    double get_pendulum_length() const;

    double get_damping_coefficient() const;

    double get_gravity() const;

    double get_max_cart_force() const;

    double get_noise_level() const;

    std::chrono::microseconds get_physics_update_period() const;

private:
    double pendulum_mass_ = 1.0; // kg
    double cart_mass_ = 5.0; // kg
    double pendulum_length_ = 2.0; // m
    double damping_coefficient_ = 20.0;
    double gravity_ = -9.8;
    double max_cart_force_ = 1000.0;
    double noise_level_ = 1.0;
    std::chrono::microseconds physics_update_period_;
};

using derivativeF = std::function<double (const std::vector<double> & , double, size_t)>;

// Runge Kutta method: 
// https://www.cnblogs.com/neoLin/p/10999688.html
// https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
class RungeKutta {
public:
    explicit RungeKutta(size_t dimension) : dim_(dimension) {
        k1_.resize(dim_);
        k2_.resize(dim_);
        k3_.resize(dim_);
        k4_.resize(dim_);
        state_.resize(dim_);
    }

    // Time step using 4th-order Runge Kutta and trapezoidal rule
    // df: Derivative function pointing to the ODE equations to solve
    // y: Status vector with the previous status at input and next state at output.
    // h: Time step.
    // u: Single input in the equations.
    // throw std::invalid_argument If the state vector doesn't have wrong dimensions.
    void step(const derivativeF& df, std::vector<double>& y, double h, double u) {
        std::size_t i = 0U;

        // stage 1
        for (i = 0; i < dim_; i++) {
            k1_[i] = df(y, u, i);
        }

        // stage 2
        for (i = 0; i < dim_; i++) {
            state_[i] = y[i] + h * 0.5 * k1_[i];
        }
        for (i = 0; i < dim_; i++) {
            k2_[i] = df(state_, u, i);
        }

        // stage 3
        for (i = 0; i < dim_; i++) {
            state_[i] = y[i] + h * (0.5 * k2_[i]);
        }
        for (i = 0; i < dim_; i++) {
            k3_[i] = df(state_, u, i);
        }

        // stage 4
        for (i = 0; i < dim_; i++) {
            state_[i] = y[i] + h * (1.0 * k3_[i]);
        }
        for (i = 0; i < dim_; i++) {
            k4_[i] = df(state_, u, i);
        }

        // update next step
        for (i = 0; i < dim_; i++) {
            y[i] = y[i] + (h / 6.0) * (k1_[i] + 2 * k2_[i] + 2 * k3_[i] + k4_[i]);
        }
    }

private:
    // state[0]: cart position
    // state[1]: cart velocity
    // state[2]: pole position
    // state[3]: pole velocity
    std::vector<double> state_;

    std::vector<double> k1_;
    std::vector<double> k2_;
    std::vector<double> k3_;
    std::vector<double> k4_;

    const size_t dim_;
};

class PendulumDriver {
public:
    explicit PendulumDriver(const PendulumConfig & config);

    void set_state(double cart_pos, double cart_vel, double pole_pos, double pole_vel);

    void set_controller_cart_force(double force);

    void set_disturbance_force(double force);

    const PendulumState & get_state() const;

    double get_controller_cart_force() const;

    double get_disturbance_force() const;

    void update();

    void reset();

private:
    const PendulumConfig pendulum_config_;
    double time_step_;  // seconds
    PendulumState state_;

    RungeKutta ode_solver_;
    // state array for ODE solver
    // X[0]: cart position
    // X[1]: cart velocity
    // X[2]: pole position
    // X[3]: pole velocity
    std::vector<double> X_;

    double controller_force_ = 0.0;
    double disturbance_force_ = 0.0;

    // 用于产生随机噪声
    std::random_device rd_;
    std::mt19937 rand_gen_;
    std::uniform_real_distribution<double> noise_gen_;

    derivativeF derivative_function_;
};


class PendulumDriverNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit PendulumDriverNode(const std::string & node_name, 
                                const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void init_state_message();

    void create_command_subscription();

    void create_disturbance_subscription();

    void create_state_publisher();

    void log_driver_state();

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
    std::string state_topic_name_;
    std::string command_topic_name_;
    std::string disturbance_topic_name_;
    std::string cart_base_joint_name_;
    std::string pole_joint_name_;
    std::chrono::microseconds state_publish_period_;
    bool enable_topic_stats_;

    std::string topic_stats_topic_name_;
    std::chrono::milliseconds topic_stats_publish_period_;
    std::chrono::milliseconds deadline_duration_;

    PendulumDriver pdriver_;

    std::shared_ptr<rclcpp::Subscription<pendulum_msg::msg::ForceCmd>> force_cmd_sub_;
    std::shared_ptr<rclcpp::Subscription<pendulum_msg::msg::ForceCmd>> disturbance_sub_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<pendulum_msg::msg::JointState>> joint_state_pub_;

    rclcpp::TimerBase::SharedPtr joint_state_timer_;
    rclcpp::TimerBase::SharedPtr update_driver_timer_;
    pendulum_msg::msg::JointState joint_state_msg_;

    uint32_t num_missed_deadlines_pub_;
    uint32_t num_missed_deadlines_sub_;
};


}   // namespace pendulum_driver
}   // namespace pendulum_demo


#endif // PENDULUM_DRIVER_NODE_HPP_