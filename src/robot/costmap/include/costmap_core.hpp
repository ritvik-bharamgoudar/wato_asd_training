#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>

const int WIDTH = 200; // cells
const int HEIGHT = 200; // cells
const double RES = 0.1; // both grid size and res chosen after testing in map
const double INF_RADIUS = 1.0; // metres
const int MAX_COST = 100;


namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    void processScan(const std::vector<float> &ranges, double angle_min, double angle_increment, double range_min, double range_max);

    std::vector<int8_t> returnGrid();


  private:
    //member variables
    rclcpp::Logger logger_;

    std::vector<int8_t> occup_grid_;

    // methods
    std::vector<int8_t> initialiseCostmap();

    void convertToGrid(double range, double angle, int &col, int &row);

    void markObstacles(std::vector<int8_t> &grid,int row, int col, int cost);

    void inflateObstacles(std::vector<int8_t> &grid);
};

}  
#endif  