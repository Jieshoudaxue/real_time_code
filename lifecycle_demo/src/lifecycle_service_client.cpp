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
        client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(client_get_state_topic);
        client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(client_change_state_topic);
    }

    uint32_t get_state(std::chrono::seconds timeout = std::chrono::seconds(3)) {
        std::shared_ptr<lifecycle_msgs::srv::GetState::Request> get_state_req = 
            std::make_shared<lifecycle_msgs::srv::GetState::Request>();
        
        if (false == client_get_state_->wait_for_service(timeout)) {
            RCLCPP_ERROR(this->get_logger(), "service %s is not available", client_get_state_->get_service_name());
            return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
        }

        auto future_result = client_get_state_->async_send_request(get_state_req);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result) == 
                                                        rclcpp::FutureReturnCode::SUCCESS) {
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

void trigger_change(std::shared_ptr<LifecycleServiceClient>& lc_client) {
    rclcpp::WallRate time_between_state_change(0.2);   // 0.2hz
    
    using Transition = lifecycle_msgs::msg::Transition;
    using State = lifecycle_msgs::msg::State;

    // configure
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

    // activate
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

    // deactivate
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

    // cleanup
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

    // shutdown
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