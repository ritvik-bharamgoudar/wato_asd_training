#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) : logger_(logger) {
  global_map_ = initialiseMap(); // set grid spanning full map to -1 everywhere 
  RCLCPP_INFO(logger_, "initial global map size=%zu", global_map_.size());
} 

void MapMemoryCore::mergeCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap_msg, double x, double y, double theta){
  RCLCPP_INFO(logger_, "mergeCostmap called, costmap size=%zu, pose=(%f, %f, %f)", costmap_msg->data.size(), x, y, theta);
}

std::vector<int8_t> MapMemoryCore::initialiseMap(){
  return std::vector<int8_t>((G_WIDTH * G_HEIGHT), -1);

}
}


/*
// Integrate the latest costmap into the global map
void mergeCostmap(costmap_msg, odom_msg) {
    // Transform and merge the latest costmap into the global map
    // (Implementation would handle grid alignment and merging logic)

    double robo_x = odom_msg.pose.pose.position.x
    double robo_y = odom_msg.pose.pose.position.y
    double theta = tf2::getYaw(msg.pose.pose.orientation)
    double cos_t = cos(theta) 
    double sin_t = sin(theta) // rotation vector

    for (size_t i = 0; i<costmap.data.size();i++){

      int new_cost = costmap.data[i]

      int row_c = int(i / costmap_WIDTH)
      int col_c = int(i % costmap_WIDTH) // as stored as 1d array where i = (row*width+col)

      double x_local = col_c * costmap_RES + costmap.origin.x
      double y_local = row_c * costmap_RES + costmap.origin.y

      double x_global = robo_x + x_local * cos_t - y_local*sin_t
      double y_global = robo_y + x_local * sin_t + y_local*cos_t

      int col_g = floor((x_global - global.origin.x) / GLOBAL_RES)
      int row_g = floor((y_global - global.origin.y) / GLOBAL_RES)

      if 0 <= row_g && row_g < G_HEIGHT && 0<=col_g && col_g < WIDTH:
        mergeCells(row_g, col_g, new_cost)


void mergeCells(row_g. col_g, new_cost)
        curr_cost = global_map_[row_g * G_WIDHT + col_g]
        if curr_cost == -1)
          global_map_[row_g * G_WIDHT + col_g] = new_cost
        else:
          double weighted_cost = NEW_COST_WEIGHT * new_cost + (1-NEWCOST_WEIGHT) * curr_cost
          global_map_[row_g * G_WIDHT + col_g] = int8_t(weighted_cost)
    }
}

*/

/* Pseudo code

// notes:
- Map updates when robot moved 1.5m
- Should publish asynchronously to costmap
- odometry used to track when moved 1.5m
- Confidence updates to combine incoming vs current data
- world map coarser than costmap




INPUTS:
/costmap
  grid.info.resolution = RES;
  grid.info.width = WIDTH;
  grid.info.height = HEIGHT;
  
  grid.info.origin.position.x = (WIDTH/2 * -1 * RES); 
  grid.info.origin.position.y = (HEIGHT/2 * -1 * RES); // lidar can be negative but grid is not, so origin taken bottom left relative to robot position
  grid.info.origin.position.z = 0.0;

  grid.data = costmap_.returnGrid();

/odom/filtered
nav_msgs/msg/Odometry
position
norm = 4.79
x
-4.699993388587701
y
8.281646459956102e-10
z
0.8999991642967193
orientation
rpy = [-0.00°, 0.00°, 0.00°]
x
-4.5989020245523627e-10
y
0.000003672988427466913
z
1.6777401303148919e-13
w
0.9999999999932546


OUTPUTS:
/map - nav_msgs::msg::OccupancyGrid()
  msg.header.stamp = 
  msg.header.frame_id = "sim_world"
  msg.info.resolution
  msg.info.widht
  msg.info.height
  msg.info.origin
  msg.data // 1D array

  CONSTANTS
  int G_WIDTH = full env
  int G_HEIGHT = full env
  double GLOBAL_RES = >0.1
  double DIST_THRESHOLD = 1.5 //metres
  double NEW_COST_WEIGHT = 0.7 // weighted average to incoming data

// Global map and robot position
nav_msgs::msg::OccupancyGrid global_map_;
double last_x, last_y;
bool is_costmap_updated_ = false;

// Callback for costmap updates
void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    // Store the latest costmap
    latest_costmap_ = costmap_msg;
    is_costmap_updated_ = true;
}

// Callback for odometry updates
void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double x = odom_msg->pose.pose.position.x;
    double y = odom_msg->pose.pose.position.y;

    // Compute distance traveled
    double dist = std::sqrt(std::pow(x - last_x, 2) + std::pow(y - last_y, 2));
    if (dist >= DISTANCE_THRESHOLD) {
        last_x = x;
        last_y = y;
        should_update_map_ = true;
    }
}

// Integrate the latest costmap into the global map
void mergeCostmap(costmap_msg, odom_msg) {
    // Transform and merge the latest costmap into the global map
    // (Implementation would handle grid alignment and merging logic)

    double robo_x = odom_msg.pose.pose.position.x
    double robo_y = odom_msg.pose.pose.position.y
    double theta = tf2::getYaw(msg.pose.pose.orientation)
    double cos_t = cos(theta) 
    double sin_t = sin(theta) // rotation vector

    for (size_t i = 0; i<costmap.data.size();i++){

      int new_cost = costmap.data[i]

      int row_c = int(i / costmap_WIDTH)
      int col_c = int(i % costmap_WIDTH) // as stored as 1d array where i = (row*width+col)

      double x_local = col_c * costmap_RES + costmap.origin.x
      double y_local = row_c * costmap_RES + costmap.origin.y

      double x_global = robo_x + x_local * cos_t - y_local*sin_t
      double y_global = robo_y + x_local * sin_t + y_local*cos_t

      int col_g = floor((x_global - global.origin.x) / GLOBAL_RES)
      int row_g = floor((y_global - global.origin.y) / GLOBAL_RES)

      if 0 <= row_g && row_g < G_HEIGHT && 0<=col_g && col_g < WIDTH:
        mergeCells(row_g, col_g, new_cost)


void mergeCells(row_g. col_g, new_cost)
        curr_cost = global_map_[row_g * G_WIDHT + col_g]
        if curr_cost == -1)
          global_map_[row_g * G_WIDHT + col_g] = new_cost
        else:
          double weighted_cost = NEW_COST_WEIGHT * new_cost + (1-NEWCOST_WEIGHT) * curr_cost
          global_map_[row_g * G_WIDHT + col_g] = int8_t(weighted_cost)
    }
}


void publishMap()




// Timer-based map update
void updateMap() {
    if (should_update_map_ && costmap_updated_) {
        integrateCostmap(latest_map_. latest_odom_);
        map_pub_->publish(global_map_);
        should_update_map_ = false;
        costmap_updated_ = false;
    }
}


*/