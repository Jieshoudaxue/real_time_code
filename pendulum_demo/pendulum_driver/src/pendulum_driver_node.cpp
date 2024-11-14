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
PendulumDriverNode::PendulumDriverNode(const std::string & node_name, const rclcpp::NodeOptions & options) 
    : LifecycleNode(node_name, options),
    state_topic_name_(declare_parameter<std::string>("state_topic_name", "pendulum_joint_states")), 
    command_topic_name_(declare_parameter<std::string>("command_topic_name", "joint_command")),
    disturbance_topic_name_(declare_parameter<std::string>("disturbance_topic_name", "disturbance")),
    cart_base_joint_name_(declare_parameter<std::string>("cart_base_joint_name", "cart_base_joint")),
    pole_joint_name_(declare_parameter<std::string>("pole_joint_name", "pole_joint")),
    state_publish_period_(std::chrono::microseconds{
        declare_parameter<std::uint16_t>("state_publish_period_us", 1000U)}),
    enable_topic_stats_(declare_parameter<bool>("enable_topic_stats", false)),
    topic_stats_topic_name_(declare_parameter<std::string>("topic_stats_topic_name", "driver_stats")),
    topic_stats_publish_period_(std::chrono::milliseconds{
        declare_parameter<std::uint16_t>("topic_stats_publish_period_ms", 1000U)}),
    deadline_duration_(std::chrono::milliseconds{
        declare_parameter<std::uint16_t>("deadline_duration_ms", 0U)}),
    pdriver_(PendulumConfig(
        declare_parameter<double>("driver.pendulum_mass", 1.0),
        declare_parameter<double>("driver.cart_mass", 5.0),
        declare_parameter<double>("driver.pendulum_length", 2.0),
        declare_parameter<double>("driver.damping_coefficient", 20.0),
        declare_parameter<double>("driver.gravity", -9.8),
        declare_parameter<double>("driver.max_cart_force", 1000.0),
        declare_parameter<double>("driver.noise_level", 1.0),
        std::chrono::microseconds{state_publish_period_})),
    num_missed_deadlines_pub_{0U},
    num_missed_deadlines_sub_{0U} {

    // init state message
    joint_state_msg_.pole_angle = 0.0;
    joint_state_msg_.pole_velocity = 0.0;
    joint_state_msg_.cart_position = 0.0;
    joint_state_msg_.cart_velocity = 0.0;
    joint_state_msg_.cart_force = 0.0;

    // create state publisher
    rclcpp::PublisherOptions state_pub_options;
    state_pub_options.event_callbacks.deadline_callback = 
        [this](rmw_offered_deadline_missed_status_t &) -> void 
        {
            num_missed_deadlines_pub_++;
        };
    joint_state_pub_ = this->create_publisher<pendulum_msg::msg::JointState>(
        state_topic_name_,
        rclcpp::QoS(10).deadline(deadline_duration_),
        state_pub_options);

    // create command subscription
    auto on_force_received = [this](pendulum_msg::msg::ForceCmd::SharedPtr msg) {
        pdriver_.set_controller_cart_force(msg->force);
    };

    rclcpp::SubscriptionOptions force_subscription_options;
    force_subscription_options.event_callbacks.deadline_callback = 
        [this](rclcpp::QOSDeadlineRequestedInfo &) -> void 
        {
            num_missed_deadlines_sub_++;
        };
    if (enable_topic_stats_) {
        force_subscription_options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
        force_subscription_options.topic_stats_options.publish_topic = topic_stats_topic_name_;
        force_subscription_options.topic_stats_options.publish_period = topic_stats_publish_period_;
    }

    using rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy;
    auto force_msg_strategy = std::make_shared<MessagePoolMemoryStrategy<pendulum_msg::msg::ForceCmd, 1>>();

    force_cmd_sub_ = this->create_subscription<pendulum_msg::msg::ForceCmd>(
        command_topic_name_,
        rclcpp::QoS(10).deadline(deadline_duration_),
        on_force_received,
        force_subscription_options,
        force_msg_strategy
    );

    // create disturbance subscription
    disturbance_sub_ = this->create_subscription<pendulum_msg::msg::ForceCmd>(
        disturbance_topic_name_,
        rclcpp::QoS(10),
        [this](pendulum_msg::msg::ForceCmd::SharedPtr msg) {
            pdriver_.set_disturbance_force(msg->force);
        }
    );

    // create state timer callback
    joint_state_timer_ = this->create_wall_timer(state_publish_period_, [this]() {
        pdriver_.update();
        const auto state = pdriver_.get_state();
        joint_state_msg_.cart_position = state.cart_position;
        joint_state_msg_.cart_velocity = state.cart_velocity;
        joint_state_msg_.cart_force = state.cart_force;
        joint_state_msg_.pole_angle = state.pole_angle;
        joint_state_msg_.pole_velocity = state.pole_velocity;
        joint_state_pub_->publish(joint_state_msg_);
    });
    // 创建完状态 topic 发送计时器后，立即取消，将计时器启动交给 LifecycleNode 的状态机
    joint_state_timer_->cancel();
    
    RCLCPP_INFO(this->get_logger(), "PendulumDriverNode constructor");
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
PendulumDriverNode::on_configure(const rclcpp_lifecycle::State& pre_state) {
    // 初始化阶段，重置 PendulumDriver
    pdriver_.reset();

    RCLCPP_INFO(this->get_logger(), 
        "LifecycleTalker on_configure is called for initial, pre state is %s", pre_state.label().c_str());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
PendulumDriverNode::on_activate(const rclcpp_lifecycle::State& pre_state) {
    // 告知下游，上游发布节点的 pub 已经激活
    joint_state_pub_->on_activate();
    // 启动状态 topic 发送计时器
    joint_state_timer_->reset();

    RCLCPP_INFO(this->get_logger(), 
        "LifecycleTalker on_activate is called, pre state is %s", pre_state.label().c_str());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
PendulumDriverNode::on_deactivate(const rclcpp_lifecycle::State& pre_state) {
    // 当节点停止活跃时，取消计时器，停止发送状态 topic
    joint_state_timer_->cancel();
    // 告知下游，上游节点的 pub 已经关闭
    joint_state_pub_->on_deactivate();

    // 当节点关闭时，打印此时倒立摆的状态
    const auto state = pdriver_.get_state();
    const auto disturbance_force = pdriver_.get_disturbance_force();
    const double controller_force_command = pdriver_.get_controller_cart_force();
    RCLCPP_INFO(this->get_logger(), "Cart position = %lf", state.cart_position);
    RCLCPP_INFO(this->get_logger(), "Cart velocity = %lf", state.cart_velocity);
    RCLCPP_INFO(this->get_logger(), "Pole angle = %lf", state.pole_angle);
    RCLCPP_INFO(this->get_logger(), "Pole angular velocity = %lf", state.pole_velocity);
    RCLCPP_INFO(this->get_logger(), "Controller force command = %lf", controller_force_command);
    RCLCPP_INFO(this->get_logger(), "Disturbance force = %lf", disturbance_force);
    RCLCPP_INFO(this->get_logger(), "Publisher missed deadlines = %u", num_missed_deadlines_pub_);
    RCLCPP_INFO(this->get_logger(), "Subscription missed deadlines = %u", num_missed_deadlines_sub_);

    RCLCPP_INFO(this->get_logger(), 
        "LifecycleTalker on_deactivate is called, pre state is %s", pre_state.label().c_str());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
PendulumDriverNode::on_cleanup(const rclcpp_lifecycle::State& pre_state) {

    RCLCPP_INFO(this->get_logger(), 
        "LifecycleTalker on_cleanup is called, pre state is %s", pre_state.label().c_str());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
PendulumDriverNode::on_shutdown(const rclcpp_lifecycle::State& pre_state) {

    RCLCPP_INFO(this->get_logger(), 
        "LifecycleTalker on_shutdown is called, pre state is %s", pre_state.label().c_str());

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}



} // namespace pendulum_driver
} // namespace pendulum_demo


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    using pendulum_demo::pendulum_driver::PendulumDriverNode;
    auto driver_node = std::make_shared<PendulumDriverNode>("pendulum_driver_node");

    exe.add_node(driver_node->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();

    return 0;
}