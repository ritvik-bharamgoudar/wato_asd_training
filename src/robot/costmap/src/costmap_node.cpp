#include <chrono>
#include <memory>
 
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishCostmap, this));
}
 
// Define the timer to publish a message every 500ms
void CostmapNode::publishCostmap() {
  auto grid = nav_msgs::msg::OccupancyGrid();

  grid.header.stamp.sec = 0;
  grid.header.frame_id = "map";

  grid.info.resolution = 0.1;
  grid.info.width = 50;
  grid.info.height = 50;
  
  grid.info.origin.position.x = 0.0;
  grid.info.origin.position.y = 0.0;
  grid.info.origin.position.z = 0.0;

  grid.data.assign(grid.info.width * grid.info.height, 0);

  //message.data = "Goodbye, ROS 2.";
  RCLCPP_INFO(this->get_logger(), "placeholder costmap");
  costmap_pub_->publish(grid);
}
 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}