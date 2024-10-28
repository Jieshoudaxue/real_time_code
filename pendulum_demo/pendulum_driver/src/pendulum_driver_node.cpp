#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "pendulum_driver/pendulum_driver_node.hpp"

namespace pendulum_demo {
namespace pendulum_driver {

// PendulumConfig
PendulumConfig::PendulumConfig(
        double pendulum_mass,
        double cart_mass,
        double pendulum_length,
        double damping_coefficient,
        double gravity,
        double max_cart_force,
        double noise_level,
        std::chrono::microseconds physics_update_period) :
    pendulum_mass_(pendulum_mass),
    cart_mass_(cart_mass),
    pendulum_length_(pendulum_length),
    damping_coefficient_(damping_coefficient),
    gravity_(gravity),
    max_cart_force_(max_cart_force),
    noise_level_(noise_level),
    physics_update_period_(physics_update_period) {}

double PendulumConfig::get_pendulum_mass() const {
    return pendulum_mass_;
}

double PendulumConfig::get_cart_mass() const {
    return cart_mass_;
}

double PendulumConfig::get_pendulum_length() const {
    return pendulum_length_;
}

double PendulumConfig::get_damping_coefficient() const {
    return damping_coefficient_;
}

double PendulumConfig::get_gravity() const {
    return gravity_;
}

double PendulumConfig::get_max_cart_force() const {
    return max_cart_force_;
}

double PendulumConfig::get_noise_level() const {
    return noise_level_;
}

std::chrono::microseconds PendulumConfig::get_physics_update_period() const {
    return physics_update_period_;
}


// PendulumDriver
PendulumDriver::PendulumDriver(const PendulumConfig & config) :
    pendulum_config_(config),
    ode_solver_(STATE_DIMENSION),
    X_{0.0, 0.0, M_PI, 0.0},
    controller_force_{0.0},
    disturbance_force_{0.0},
    rand_gen_(rd_()),
    noise_gen_(std::uniform_real_distribution<double>(
        -config.get_noise_level(), config.get_noise_level())) {
    
    // Calculate the controller timestep(sec) for discrete differentiation/integration
    time_step_ = pendulum_config_.get_physics_update_period().count() / (1000.0 * 1000.0);
    if (std::isnan(time_step_) || time_step_ == 0) {
        throw std::runtime_error("Invalid time_step_ calculated in PendulumDriver constructor");
    }

    derivative_function_ = [this](const std::vector<double>& y, double u, size_t i) -> double {
        const double m = pendulum_config_.get_pendulum_mass();
        const double M = pendulum_config_.get_cart_mass();
        const double L = pendulum_config_.get_pendulum_length();
        const double d = pendulum_config_.get_damping_coefficient();
        const double g = pendulum_config_.get_gravity();

        // y[0] : cart position
        // y[1] : cart velocity
        // y[2] : pole position
        // y[3] : pole velocity
        if ( i == 0 ) {
            return y[1];
        } else if ( i == 1 ) {
            double Sy = sin(y[2]);
            double Cy = cos(y[2]);
            double D = m * L * L * (M + m * (1 - Cy * Cy));
            // 返回小车加速度
            return (1 / D) *
                (-m * m * L * L * g * Cy * Sy + m * L * L * (m * L * y[3] * y[3] * Sy - d * y[1])) +
                m * L * L * (1 / D) * u;
        } else if ( i == 2 ) {
            return y[3];
        } else if ( i == 3 ) {
            double Sy = sin(y[2]);
            double Cy = cos(y[2]);
            double D = m * L * L * (M + m * (1 - Cy * Cy));
            // 返回摆杆角加速度
            return (1 / D) * ((m + M) * m * g * L * Sy - m * L * Cy * (m * L * y[3] * y[3] * Sy -
                d * y[1])) - m * L * Cy * (1 / D) * u + noise_gen_(rand_gen_);            
        } else {
            throw std::invalid_argument("received wrong index");
        }
    };
}

void PendulumDriver::set_controller_cart_force(double force) {
    controller_force_ = rcppmath::clamp(force, 
                            -pendulum_config_.get_max_cart_force(), 
                            pendulum_config_.get_max_cart_force());
}

void PendulumDriver::set_state(double cart_pos, double cart_vel, double pole_pos, double pole_vel) {
    state_.cart_position = cart_pos;
    state_.cart_velocity = cart_vel;
    state_.pole_angle = pole_pos;
    state_.pole_velocity = pole_vel;
}

void PendulumDriver::set_disturbance_force(double force) {
    disturbance_force_ = force;
}


const PendulumState & PendulumDriver::get_state() const {
    return state_;
}

double PendulumDriver::get_controller_cart_force() const {
    return controller_force_;
}


double PendulumDriver::get_disturbance_force() const {
    return disturbance_force_;
}

void PendulumDriver::update() {
    double cart_force = disturbance_force_ + controller_force_;
    ode_solver_.step(derivative_function_, X_, time_step_, cart_force);

    state_.cart_position = X_[0];
    state_.cart_velocity = X_[1];
    state_.pole_angle = X_[2];
    state_.pole_velocity = X_[3];
    state_.cart_force = cart_force;
}

void PendulumDriver::reset() {
    set_state(0.0, 0.0, M_PI, 0.0);
    set_disturbance_force(0.0);
    set_controller_cart_force(0.0);
    X_ = {0.0, 0.0, M_PI, 0.0};
}

// PendulumDriverNode





} // namespace pendulum_driver
} // namespace pendulum_demo


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    using pendulum_demo::pendulum_driver::PendulumDriverNode;
    auto driver_node = std::make_shared<PendulumDriverNode>("pendulum_driver");

    exe.add_node(driver_node->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();

    return 0;
}