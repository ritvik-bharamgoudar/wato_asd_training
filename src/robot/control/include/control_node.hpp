#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "control_core.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    robot::ControlCore control_;

    // callbacks
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void controlLoop(); // fired by timer_ - handles extraction + core call + publish

    // ROS constructs
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Path::SharedPtr current_path_;

    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_theta_ = 0.0;
    bool has_odom_ = false;

    int last_index_ = 0;

    //pp parameters
    double lookahead_distance_ = 0.5;
    double linear_speed_ = 1.5;
    double max_angular_z_ = 2;
    double goal_tolerance_ = 0.15;

    double enter_rotate_threshold_ = M_PI / 2.0; // if heading error above, rotate in place
    double exit_rotate_threshold_ = M_PI / 6.0;  // must drop below to resume normal driving
    double rotate_angular_speed_ = 0.2;
    double small_heading_thresh_ = 0.25; // heading error below this, gentle tracking instead of pp
    double small_heading_gain_ = 0.5;

    bool rotating_in_place_ = false;

    double max_delta_omega_ = 0.3; // turning rate limit
    double last_omega_ = 0.0; // for rate limit previous tick comparison
    double last_linear_ = 0.0;
    double max_delta_linear_ = 0.2; // rate limit



};

#endif