#include "control_node.hpp"

#include <chrono>
#include <memory>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10);

    // 10Hz firing control loop
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&ControlNode::controlLoop, this));
}


void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(),"path size: %zu, first pos: %.2f, last pos: %.2f", 
    msg->poses.size(), msg->poses.front().pose.position.x,msg->poses.back().pose.position.x );
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {

    RCLCPP_INFO(this->get_logger(), "odom x = %.2f", msg->pose.pose.position.x);
}

void ControlNode::controlLoop() {

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.1;
    cmd.angular.z = 0.0;
    RCLCPP_INFO(this->get_logger(), "publishing cmd_vel: linear.x=%.2f angular.z=%.2f",
    cmd.linear.x, cmd.angular.z);


    cmd_vel_pub_->publish(cmd);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
