#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include "nav_msgs/msg/occupancy_grid.hpp"


const int G_WIDTH = 400; // cols = W / RES
const int G_HEIGHT = 400; // rows = H / RES
const double G_RES = 0.1; // size of once cell RESxRES metres
const double G_ORIGIN_X = G_WIDTH / 2 * -1 * G_RES;
const double G_ORIGIN_Y = G_HEIGHT / 2 * -1 * G_RES;
const double DIST_THRESHOLD = 2; // metres
const int HITS_REQUIRED = 9; // cell needs a cost at least 3 times to reach global map
const int HIT_CAP = 40;
const int FREE = 0;
const int MAX_COST = 100;

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

    std::vector<int> hit_count_;

    std::vector<int8_t> initialiseMap();

    void assignWeightedCost(int new_row, int new_col, int width, int new_cost);

    
};

}  
 
#endif   
