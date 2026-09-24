#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10); // initialise object with topic and queue length 10 messages

  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1)); // initialise laserCallback and store up to 10 messages 

  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishCostmap, this)); // publish costmap every 0.5 seconds

}
 
// data format from OccupancyGrid (1d int8_t array for grid)
void CostmapNode::publishCostmap() {
  auto grid = nav_msgs::msg::OccupancyGrid();

  grid.header.stamp = last_scan_stamp_; // lidar input time stamp
  grid.header.frame_id = "robot";

  grid.info.resolution = RES;
  grid.info.width = WIDTH;
  grid.info.height = HEIGHT;
  
  grid.info.origin.position.x = (WIDTH/2 * -1 * RES); 
  grid.info.origin.position.y = (HEIGHT/2 * -1 * RES); // lidar can be negative but grid is not, so origin taken bottom left relative to robot position
  grid.info.origin.position.z = 0.0;

  grid.data = costmap_.returnGrid();

  //RCLCPP_INFO(this->get_logger(), "costmap returned with %zu cells", costmap_.returnGrid().size()); 
  costmap_pub_->publish(grid);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  last_scan_stamp_ = msg->header.stamp; // store lidar time stamp in member variable
  costmap_.processScan(msg->ranges, msg->angle_min, msg->angle_increment, msg->range_min, msg->range_max); // pass in lidar msg to processScan
  //RCLCPP_INFO(this->get_logger(), "scan with %zu number of ranges",msg->ranges.size());
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}