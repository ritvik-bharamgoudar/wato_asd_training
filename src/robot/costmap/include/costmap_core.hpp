#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    struct Point{
      float x;
      float y;
    };
    
    // laser scan data
    void laserScan(
      const std::vector<float>& radii, 
      float angle_min,
      float angle_increment,
      float range_min,
      float range_max);
    
    // define grid state
    void emptyGrid(int8_t value = 0);
    int width() const;
    int height() const;
    double resolution() const;

    Point polarToCartesain(double radius, float angle);
    void addCost(const std::vector<float>);


  private:
    rclcpp::Logger logger_;
    

    // grid state
    std::vector<int8_t> grid_;
    int width_;
    int height_;
    double resolution_;

};

}  

#endif  