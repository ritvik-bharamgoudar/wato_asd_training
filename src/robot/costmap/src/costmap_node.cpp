#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10,
    std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishCostmap, this));
}
 
// Define grid to publish an empty costmap placeholder
void CostmapNode::publishCostmap() {
  auto grid = nav_msgs::msg::OccupancyGrid();

  grid.header.stamp.sec = 0;
  grid.header.frame_id = "map";

  grid.info.resolution = resolution_;
  grid.info.width = width_;
  grid.info.height = height_;
  
  grid.info.origin.position.x = 0.0;
  grid.info.origin.position.y = 0.0;
  grid.info.origin.position.z = 0.0;

  grid.data.assign(grid.info.width * grid.info.height, 0);

  //message.data = "Goodbye, ROS 2.";
  RCLCPP_INFO(this->get_logger(), "placeholder costmap");
  costmap_pub_->publish(grid);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  costmap_.laserScan(msg->ranges, msg->angle_min, msg->angle_increment, msg->range_min, msg->range_max);
  RCLCPP_INFO(this->get_logger(), "scan with %zu number of ranges",msg->ranges.size());
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}