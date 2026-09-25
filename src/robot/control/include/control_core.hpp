#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);

    std::tuple<double, double, bool, int> findLookaheadPoint(
    const nav_msgs::msg::Path::SharedPtr path,
    double robot_x, double robot_y, double robot_theta,
    double lookahead_distance, int start_index);

    std::pair<double, double> purePursuit(double lx, double ly, double linear_speed, double max_angular_z);


  private:
    rclcpp::Logger logger_;

    double computeDistance(double x1, double y1, double x2, double y2);

};

} 

#endif 
