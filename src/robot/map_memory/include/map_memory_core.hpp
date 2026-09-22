#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>

const int G_WIDTH = 500 // cols = W / RES
const int G_HEIGHT = 500 // rows = H / RES
const double G_RES = 0.2 // size of once cell RESxRES

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

  private:
    rclcpp::Logger logger_;

    double last_x_;
    double last_y_;
    bool costmap_updated_ = false;



    
};

}  
 
#endif   
