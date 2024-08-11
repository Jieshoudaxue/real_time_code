#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"

class LifecycleServiceClient : public rclcpp::Node {
public:
    explicit LifecycleServiceClient(const std::string & node_name) : Node(node_name) {
        RCLCPP_INFO(this->get_logger(), "LifecycleServiceClient constructor is called");
    }

    void init() {
        // client_get_state_topic 为 lifecycle_talker/get_state
        // client_change_state_topic 为 lifecycle_talker/change_state
        // 上游 lifecycle 节点名为 lifecycle_talker，因此他一定会附赠 lifecycle_talker/get_state 和 lifecycle_talker/change_state service
        // 触发器通过这两个 service ，可以驱动 lifecycle_talker 节点的状态变化
        client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(client_get_state_topic);
        client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(client_change_state_topic);
    }

    uint32_t get_state(std::chrono::seconds timeout = std::chrono::seconds(3)) {
        std::shared_ptr<lifecycle_msgs::srv::GetState::Request> get_state_req = 
            std::make_shared<lifecycle_msgs::srv::GetState::Request>();
        
        if (false == client_get_state_->wait_for_service(timeout)) {
            RCLCPP_ERROR(this->get_logger(), "service %s is not available", client_get_state_->get_service_name());
            // PRIMARY_STATE_UNKNOWN 为 0，表示未知状态
            return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        }

        auto future_result = client_get_state_->async_send_request(get_state_req);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result) == 
                                                        rclcpp::FutureReturnCode::SUCCESS) {
            // future_result 只能被调用一次，调用后会发生值转移，再次调用属于未定义行为
            // 因此这里缓存了 future_result 的值，免去二次调用
            auto tmp_result = future_result.get();
            RCLCPP_INFO(this->get_logger(), "node [%s] in state: %s", lifecycle_node.c_str(), 
                                                    tmp_result->current_state.label.c_str());
            return tmp_result->current_state.id;
        } else {
            RCLCPP_ERROR(this->get_logger(), "failed to call service %s", client_get_state_->get_service_name());
            return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        }
    }

    bool change_state(std::uint8_t transition_id, std::chrono::seconds timeout = std::chrono::seconds(3)) {
        std::shared_ptr<lifecycle_msgs::srv::ChangeState::Request> change_state_req = 
            std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        change_state_req->transition.id = transition_id;

        if (false == client_change_state_->wait_for_service(timeout)) {
            RCLCPP_ERROR(this->get_logger(), "service %s is not available", client_change_state_->get_service_name());
            return false;
        }

        auto future_result = client_change_state_->async_send_request(change_state_req);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result) ==
                                                        rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "node [%s] transition %d is successful", lifecycle_node.c_str(), transition_id);
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "failed to call service %s, transition %d is failed", 
                                                client_change_state_->get_service_name(), transition_id);
            return false;
        }
    }

private:
    const std::string lifecycle_node = "lifecycle_talker";
    const std::string client_get_state_topic = "lifecycle_talker/get_state";
    const std::string client_change_state_topic = "lifecycle_talker/change_state";

    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state_;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;
};

// 按照 lifecycle node 的状态转换顺序，
// 依次触发 configure, activate, deactivate, activate, deactivate, cleanup, shutdown 状态转换
void trigger_change(std::shared_ptr<LifecycleServiceClient>& lc_client) {
    rclcpp::WallRate time_between_state_change(0.2);   // 0.2hz
    
    using Transition = lifecycle_msgs::msg::Transition;
    using State = lifecycle_msgs::msg::State;

    // configure 状态转换
    // 完成后，lifecycle_talker 节点将从 unconfigured 状态转换到 inactive 状态
    {
        if (false == lc_client->change_state(Transition::TRANSITION_CONFIGURE)) {
            RCLCPP_ERROR(lc_client->get_logger(), "failed to trigger state: configure");
            return;
        }

        if (State::PRIMARY_STATE_UNKNOWN == lc_client->get_state()) {
            RCLCPP_INFO(lc_client->get_logger(), "failed to get state in configure");
            return;
        }
    }

    // activate 状态转换
    // 完成后，lifecycle_talker 节点将从 inactive 状态转换到 active 状态
    {
        time_between_state_change.sleep();
        if (false == rclcpp::ok()) {
            return;
        }

        if (false == lc_client->change_state(Transition::TRANSITION_ACTIVATE)) {
            RCLCPP_ERROR(lc_client->get_logger(), "failed to trigger state: activate");
            return;
        }
        if (State::PRIMARY_STATE_UNKNOWN == lc_client->get_state()) {
            return;
        }
    }

    // deactivate 状态转换
    // 完成后，lifecycle_talker 节点将从 active 状态转回到 inactive 状态
    {
        time_between_state_change.sleep();
        if (false == rclcpp::ok()) {
            return;
        }

        if (false == lc_client->change_state(Transition::TRANSITION_DEACTIVATE)) {
            RCLCPP_ERROR(lc_client->get_logger(), "failed to trigger state: deactivate");
            return;
        }
        if (State::PRIMARY_STATE_UNKNOWN == lc_client->get_state()) {
            return;
        }
    }

    // activate it again
    {
        time_between_state_change.sleep();
        if (false == rclcpp::ok()) {
            return;
        }

        if (false == lc_client->change_state(Transition::TRANSITION_ACTIVATE)) {
            RCLCPP_ERROR(lc_client->get_logger(), "failed to trigger state: activate");
            return;
        }
        if (State::PRIMARY_STATE_UNKNOWN == lc_client->get_state()) {
            return;
        }
    }

    // deactivate it again
    {
        time_between_state_change.sleep();
        if (false == rclcpp::ok()) {
            return;
        }

        if (false == lc_client->change_state(Transition::TRANSITION_DEACTIVATE)) {
            RCLCPP_ERROR(lc_client->get_logger(), "failed to trigger state: deactivate");
            return;
        }
        if (State::PRIMARY_STATE_UNKNOWN == lc_client->get_state()) {
            return;
        }
    }

    // cleanup 状态转换
    // 完成后，lifecycle_talker 节点将从 inactive 状态转回到 unconfigured 状态
    {
        time_between_state_change.sleep();
        if (false == rclcpp::ok()) {
            return;
        }

        if (false == lc_client->change_state(Transition::TRANSITION_CLEANUP)) {
            RCLCPP_ERROR(lc_client->get_logger(), "failed to trigger state: cleanup");
            return;
        }
        if (State::PRIMARY_STATE_UNKNOWN == lc_client->get_state()) {
            return;
        }
    }

    // shutdown 状态转换
    // 完成后，lifecycle_talker 节点将从 unconfigured 状态转回到 finalized 状态
    {
        time_between_state_change.sleep();
        if (false == rclcpp::ok()) {
            return;
        }

        if (false == lc_client->change_state(Transition::TRANSITION_UNCONFIGURED_SHUTDOWN)) {
            RCLCPP_ERROR(lc_client->get_logger(), "failed to trigger state: shutdown");
            return;
        }
        if (State::PRIMARY_STATE_UNKNOWN == lc_client->get_state()) {
            return;
        }
    }

}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    std::shared_ptr<LifecycleServiceClient> lc_client = std::make_shared<LifecycleServiceClient>("lifecycle_service_client");
    lc_client->init();

    trigger_change(lc_client);

    rclcpp::shutdown();

    return 0;
}