#include <chrono>
#include <memory>

#include "map_memory_node.hpp"


MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MappingNode::costmapCallback, this, std::placeholders::_1));
  
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
  "/odom/filtered", 10, std::bind(&MappingNode::odomCallback, this, std::placeholders::_1));

  // Initialize publisher
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // Initialize timer
  timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&MappingNode::updateMap, this));
}

// data format from OccupancyGrid (1d int8_t array for grid)
void MappingNode::publishMap() {
  auto grid = nav_msgs::msg::OccupancyGrid();

  grid.header.stamp = ; // lidar input time stamp
  grid.header.frame_id = ;

  grid.info.resolution = ;
  grid.info.width = ;
  grid.info.height = ;
  
  grid.info.origin.position.x = (/2 * -1 * ); 
  grid.info.origin.position.y = (/2 * -1 * ); // lidar can be negative but grid is not, so origin taken bottom left relative to robot position
  grid.info.origin.position.z = 0.0;

  grid.data = map_.returnMap();

  RCLCPP_INFO(this->get_logger(), "map returned with %zu cells", map_.returnMap().size()); 
  map_pub_->publish(grid);
}

void MappingNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = msg->grid.data;
  is_costmap_updated_ = true;
  RCLCPP_INFO(this->get_logger(), "costmap with %zu size",msg->grid.data.size());
}

void MappingNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;

  // distance travelled
  double dist = std::sqrt(std::pow(x - last_x_, 2)+ std::pow(y - last_y_, 2));
  if (dist >= DIST_THRESHOLD) {
    last_x_ = x;
    last_y_ = y;
    should_update_map_ = true
  }
  RCLCPP_INFO(this->get_logger(), "odom position: %zu ",msg->pose.pose.x);
}

void MappingNode::timerCall(is_costmap_updated, should_update_map_){
  if (is_costmap_updated && should_update_map_){
    void mergeCells()
  }
  

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
} 
 