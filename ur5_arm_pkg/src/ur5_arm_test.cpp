// imports used
// #include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "ur5_arm_msgs/srv/ur5_service_message.hpp"
#include <moveit/move_group_interface/move_group_interface.h>

using UR5ServiceMessage = ur5_arm_msgs::srv::UR5ServiceMessage;

// class for the node
class UR5ArmTest : public rclcpp::Node {
public:
    UR5ArmTest(): rclcpp::Node("ur5_robot_arm", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
        service_ = create_service<UR5ServiceMessage>("posestamped", std::bind(&UR5ArmTest::listener_callback, this, std::placeholders::_1, std::placeholders::_2));
    }

    _Float64 get_position_x() const {
        return position_x_;
    }

    _Float64 get_position_y() const {
        return position_y_;
    }

    _Float64 get_position_z() const {
        return position_z_;
    }

    _Float64 get_orientation_w() const {
        return orientation_w_;
    }

    // geometry_msgs::msg::PoseStamped get_position() const {
        
    // } 

private:
    // function to receive the goal position from the service provider
    void listener_callback(const std::shared_ptr<UR5ServiceMessage::Request> request, const std::shared_ptr<UR5ServiceMessage::Response> response) {
        RCLCPP_INFO(get_logger(), "Request received...\n");

        orientation_w_ = request->position.pose.orientation.w;
        position_x_ = request->position.pose.position.x;
        position_y_ = request->position.pose.position.y;
        position_z_ = request->position.pose.position.z;

        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Position = (" + str(self.position_x) + ", " + str(self.position_y) 
        //                        + ", " + str(self.position_z) + ", " + str(self.orientation_w) 
        //                        + ") and frame_id = " + self.frame_id + "\n"");

        response->success = true;

        // return response;
    }

        // declare variables for goal position
    _Float64 orientation_w_;
    _Float64 position_x_;
    _Float64 position_y_;
    _Float64 position_z_;
    
    // create the service
    rclcpp::Service<UR5ServiceMessage>::SharedPtr service_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc,argv);

    auto ur5_arm_test = std::make_shared<UR5ArmTest>();
    
    // spin the node once
    // rclcpp::spin_some(ur5_arm_test);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Test...");

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(ur5_arm_test, "ur_manipulator");

    // Set a target Pose
    const auto pose_goal = [ur5_arm_test]{
        geometry_msgs::msg::PoseStamped msg;
        msg.pose.orientation.w = ur5_arm_test->get_orientation_w();
        msg.pose.position.x = ur5_arm_test->get_position_x();
        msg.pose.position.y = ur5_arm_test->get_position_y();
        msg.pose.position.z = ur5_arm_test->get_position_z();
        return msg;
    }();
    move_group_interface.setPoseTarget(pose_goal);

    // Create a plan to that target pose
    auto const [success, plan] = [&move_group_interface]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if(success) {
        move_group_interface.execute(plan);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Planing failed!");
    }

    rclcpp::spin(ur5_arm_test);
    rclcpp::shutdown();

    return 0;
}