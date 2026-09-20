#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
 
#include "costmap_core.hpp"

//class definition/shape
class CostmapNode : public rclcpp::Node {
  public:
    //constructor
    CostmapNode();

  private: 
    // Place callback function here

    //private methods - used within this class only
    void publishCostmap();
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    // data which is stored in member variables 
    robot::CostmapCore costmap_;
    // Place these constructs here
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_scan_stamp_; // sync up published map with lidar input stamp
};
 
#endif 