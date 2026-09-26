#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "map_memory_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:

    void updateMap();
    void publishMap();
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    robot::MapMemoryCore map_memory_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_theta_ = 0.0;

    // used to store the most recent to costmap callback
    double costmap_pose_x_ = 0.0;
    double costmap_pose_y_ = 0.0;
    double costmap_theta_ = 0.0;

    double last_x_ = 0.0;
    double last_y_ = 0.0;

    bool has_last_pose_ = false;
    bool should_update_map_ = false;
    bool is_costmap_updated_ = false;

    nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;

    
};

#endif  
