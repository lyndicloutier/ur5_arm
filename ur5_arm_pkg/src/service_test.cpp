#include <rclcpp/rclcpp.hpp>
#include "ur5_arm_msgs/srv/ur5_service_message.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;
using UR5ServiceMessage = ur5_arm_msgs::srv::UR5ServiceMessage;

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // initialize the node
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("service_test");

    // create the client
    rclcpp::Client<UR5ServiceMessage>::SharedPtr client = node->create_client<UR5ServiceMessage>("posestamped");

    // make the request
    auto request = std::make_shared<UR5ServiceMessage::Request>();
    request->position.pose.orientation.w = 0.0;
    request->position.pose.position.x = 1.0;
    request->position.pose.position.y = 2.0;
    request->position.pose.position.z = 3.0;

    // wait for the service
    while (!client->wait_for_service()) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    // send the request to the server
    auto result = client->async_send_request(request);

    // Wait for the result
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Success...\n");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service posestamped...\n");
    }

    rclcpp::shutdown();
    return 0;
}