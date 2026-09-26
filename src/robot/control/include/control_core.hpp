#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <tuple>
#include <utility>

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);

    // these are all called in node object so public
    std::tuple<double, double, bool, int> findLookaheadPoint(
    const nav_msgs::msg::Path::SharedPtr path,
    double robot_x, double robot_y, double robot_theta,
    double lookahead_distance, int start_index);

    std::pair<double, double> purePursuit(double lx, double ly, double linear_speed, double max_angular_z);
    
    bool checkGoalReached(const nav_msgs::msg::Path::SharedPtr path, double robot_x, double robot_y, double goal_tolerance);

    // if turn too tight or shallow - use different turning regimes
    double computeHeadingError(double lx, double ly);
    // rotate in place if too tight
    std::pair<double, double> turnInPlace(double heading_error, double rotate_angular_speed);
    
    // caps change in speeds
    double rateLimit(double desired_omega, double last_omega, double max_delta_omega);
  
    private:
    rclcpp::Logger logger_;

    double computeDistance(double x1, double y1, double x2, double y2);

};

} 

#endif 
