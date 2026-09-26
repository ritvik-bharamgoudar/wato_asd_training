#include "map_memory_core.hpp"
#include <cmath>
#include <vector>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) : logger_(logger) 
{
  global_map_ = initialiseMap(); // set grid spanning full map to -1 everywhere 
  RCLCPP_INFO(logger_, "initial global map size=%zu", global_map_.size());
} 

std::vector<int8_t> MapMemoryCore::initialiseMap()
{
  hit_count_ = std::vector<int>((G_WIDTH * G_HEIGHT), 0);
  return std::vector<int8_t>((G_WIDTH * G_HEIGHT), -1);
}

// g: global, c: costmap, r: robot
void MapMemoryCore::mergeCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap_msg, double r_x, double r_y, double r_theta)
{
  double cos_t = std::cos(r_theta);
  double sin_t = std::sin(r_theta);

  //for each local cost: index->cell->cartesian->global frame->global cell
  int c_width = static_cast<int>(costmap_msg->info.width);
  for (size_t i=0; i < costmap_msg->data.size(); i++){
    int new_cost = costmap_msg->data[i];
    int c_row = static_cast<int>(i) / c_width;
    int c_col = static_cast<int>(i) % c_width;

    //costmap cell to cartesian coordinates
    double c_x = (c_col+0.5) * costmap_msg->info.resolution + costmap_msg->info.origin.position.x;
    double c_y = (c_row+0.5) * costmap_msg->info.resolution + costmap_msg->info.origin.position.y;

    // costmap frame to global frame - transform with robot pose
    double g_x = r_x + (c_x * cos_t) - (c_y * sin_t);
    double g_y = r_y + (c_x * sin_t) + (c_y * cos_t);

    int g_col = int(std::floor((g_x - G_ORIGIN_X) / G_RES));
    int g_row = int(std::floor((g_y - G_ORIGIN_Y) / G_RES));

    if (g_row >= 0 && g_row < G_HEIGHT && g_col >= 0 && g_col < G_WIDTH) {
      //global_map_[g_row * G_WIDTH + g_col] = static_cast<int8_t>(new_cost);
      assignWeightedCost(g_row, g_col, G_WIDTH, new_cost);
    }
  }
  //RCLCPP_INFO(logger_, "mergeCostmap called, costmap size=%zu, pose=(%f, %f, %f)", costmap_msg->data.size(), r_x, r_y, r_theta); 
}

// count how many times 
void MapMemoryCore::assignWeightedCost(int new_row, int new_col, int width, int new_cost)
{
    int i = (new_row * width + new_col);
    if (new_cost >= 0){
      if (new_cost == MAX_COST) {
          // increment if MAX_COST
          if (hit_count_[i] < HIT_CAP) {
              hit_count_[i] += 3; // weighted to favour obstacle detections than free
                                  // because lidar traces scraping obstacles were returning as free
          }
      } 
      else if (new_cost == FREE) {
          // if marked as free, decrement
          if (hit_count_[i] > 0) {
              hit_count_[i]--;
          }
      } 
      else {
          global_map_[i] = static_cast<int8_t>(new_cost); //cells from inflate obstacles
          return;
      }
      global_map_[i] = (hit_count_[i] >= HITS_REQUIRED) ? MAX_COST : FREE;
  }
}


std::vector<int8_t> MapMemoryCore::returnMap() const {
  return global_map_;
}
}