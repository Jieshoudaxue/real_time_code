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
    : LifecycleNode(node_name, options) {

    RCLCPP_INFO(this->get_logger(), "PendulumControllerNode constructor");
}

} // namespace pendulum_controller
} // namespace pendulum_demo


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    using pendulum_demo::pendulum_controller::PendulumControllerNode;
    auto driver_node = std::make_shared<PendulumControllerNode>("pendulum_controller_node");

    exe.add_node(driver_node->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();

    return 0;
}