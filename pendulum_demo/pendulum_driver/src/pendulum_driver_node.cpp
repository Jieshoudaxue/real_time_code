#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace pendulum_demo {
namespace pendulum_driver {

class PendulumDriverNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit PendulumDriverNode(const std::string & node_name) :
        rclcpp_lifecycle::LifecycleNode(node_name) {
            RCLCPP_INFO(this->get_logger(), "PendulumDriverNode constructor is called");
        }

};


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