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
    current_path_ = msg;
    last_index_ = 0; // if new path, then start index for lookahead search

}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
    robot_theta_ = tf2::getYaw(msg->pose.pose.orientation);
    has_odom_ = true; // no control loop unless odom come throuhg
}

void ControlNode::controlLoop() {

    if (!current_path_ || current_path_->poses.empty() || !has_odom_) 
    {
        return;
    }

    // we have arrived
    if (control_.checkGoalReached(current_path_, robot_x_, robot_y_, goal_tolerance_)) 
    {
        geometry_msgs::msg::Twist stop_cmd;
        cmd_vel_pub_->publish(stop_cmd);
        return;
    }

    // calclate lookahead point in robot frame and set index for next search
    auto [lx, ly, found, updated_index] = control_.findLookaheadPoint(current_path_, robot_x_, robot_y_, robot_theta_, lookahead_distance_, last_index_);
    last_index_ = updated_index;

    //RCLCPP_INFO(this->get_logger(), "lookahead point: lx=%.2f ly=%.2f, index=%d",lx, ly, updated_index);

    auto [pp_linear, pp_angular] = control_.purePursuit(lx, ly, linear_speed_, max_angular_z_);
    //RCLCPP_INFO(this->get_logger(), "pure pursuit: linear_x=%.2f angular_z=%.2f", pp_linear, pp_angular);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = pp_linear;
    cmd.angular.z = pp_angular;
    //RCLCPP_INFO(this->get_logger(), "publishing cmd_vel: linear.x=%.2f angular.z=%.2f",cmd.linear.x, cmd.angular.z);


    cmd_vel_pub_->publish(cmd);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
