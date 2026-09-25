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
        last_omega_ = 0.0;
        return;
    }

    // calclate lookahead point in robot frame and set index for next search
    auto [lx, ly, found, updated_index] = control_.findLookaheadPoint(current_path_, robot_x_, robot_y_, robot_theta_, lookahead_distance_, last_index_);
    last_index_ = updated_index;

    if (!found) //stop publishing if close to final node
    {
        geometry_msgs::msg::Twist stop_cmd;
        cmd_vel_pub_->publish(stop_cmd);
        //last_omega_ = 0.0;
        return;
    }

    //RCLCPP_INFO(this->get_logger(), "lookahead point: lx=%.2f ly=%.2f, index=%d",lx, ly, updated_index);

    double heading_error = control_.computeHeadingError(lx, ly);

    // shoulndt be abrupt exit and enter into rotation - linear movement
    if (!rotating_in_place_ && std::abs(heading_error) > enter_rotate_threshold_) 
    {
        rotating_in_place_ = true;
    } 
    else if (rotating_in_place_ && std::abs(heading_error) < exit_rotate_threshold_) 
    {
        rotating_in_place_ = false;
        last_omega_ = 0.0; // stop after turn in place
    }

    // store output from core
    std::pair<double, double> linear_angular_vels;

    // for each state, use relevant control regime
    if (rotating_in_place_) 
    {
        linear_angular_vels = control_.turnInPlace(heading_error, rotate_angular_speed_);
    } 
    //else if (std::abs(heading_error) < small_heading_thresh_) 
    //{
    //  linear_angular_vels = control_.gentleTracking(heading_error, linear_speed_, small_heading_gain_);
    //} 
    else 
    {
        linear_angular_vels = control_.purePursuit(lx, ly, linear_speed_, max_angular_z_);
    }
    
    double desired_linear = linear_angular_vels.first;
    double desired_omega = linear_angular_vels.second;

    double omega = control_.rateLimit(desired_omega, last_omega_, max_delta_omega_);
    double linear = control_.rateLimit(desired_linear, last_linear_, max_delta_linear_);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear;
    cmd.angular.z = omega;
    //RCLCPP_INFO(this->get_logger(), "publishing cmd_vel: linear.x=%.2f angular.z=%.2f",cmd.linear.x, cmd.angular.z);


    cmd_vel_pub_->publish(cmd);

    last_linear_ = linear;
    last_omega_ = omega;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
