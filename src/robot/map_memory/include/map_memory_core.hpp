#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include "nav_msgs/msg/occupancy_grid.hpp"


const int G_WIDTH = 500; // cols = W / RES
const int G_HEIGHT = 500; // rows = H / RES
const double G_RES = 0.2; // size of once cell RESxRES metres
const double DIST_THRESHOLD = 1.5; // metres
const double NEW_COST_WEIGHT = 0.7; // for weighted averaging of incoming costmap

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    void mergeCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap, double robot_x, double robot_y, double robot_theta);

    std::vector<int8_t> returnMap() const;
  
    private:
    rclcpp::Logger logger_;

    std::vector<int8_t> global_map_;

    std::vector<int8_t> initialiseMap();





    
};

}  
 
#endif   
