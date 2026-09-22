#include <chrono>
#include <memory>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "map_memory_node.hpp"


MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
  "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  // Initialize publisher
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  // Initialize timer
  timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
}

/*
// data format from OccupancyGrid (1d int8_t array for grid)
void MapMemoryNode::publishMap() {
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
*/

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  latest_costmap_ = msg;
  is_costmap_updated_ = true;
  RCLCPP_INFO(this->get_logger(), "costmap with %zu size",msg->data.size());
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  robot_theta_ = tf2::getYaw(msg->pose.pose.orientation);

  if (!has_last_pose_){
    // only triggers on first odom msg received
    last_x_ = robot_x_;
    last_y_ = robot_y_;
    has_last_pose_ = true; 
    should_update_map_ = true; // do one update intially
    return;
  }

  // distance travelled
  double dist = std::sqrt(std::pow(robot_x_ - last_x_, 2)+ std::pow(robot_y_ - last_y_, 2));
  
  if (dist >= DIST_THRESHOLD) {
    last_x_ = robot_x_;
    last_y_ = robot_y_;
    should_update_map_ = true;
  }
  RCLCPP_INFO(this->get_logger(), "odom position: %f ",msg->pose.pose.position.x);
}

void MapMemoryNode::updateMap(){
  if (!is_costmap_updated_ || !should_update_map_){
    return; // need both true to continue
  }

  map_memory_.mergeCostmap(latest_costmap_, robot_x_, robot_y_, robot_theta_);

  should_update_map_ = false;
  is_costmap_updated_ = false;

  }
  



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
} 
 