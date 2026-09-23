#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>

const int WIDTH = 100; // cells
const int HEIGHT = 100; // cells
const double RES = 0.2; // both grid size and res chosen after testing in map
const double INF_RADIUS = 1.0; // metres
const int8_t MAX_COST = 100;
const int8_t UNKNOWN = -1;
const int8_t FREE = 0;


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

    void convertToGrid(double range, double angle, int &row, int &col);

    void markObstacles(std::vector<int8_t> &grid,int row, int col, int cost);

    void markLine(std::vector<int8_t> &grid, int x0, int y0, int x1, int y1, int8_t cost, int8_t free);

    void inflateObstacles(std::vector<int8_t> &grid);
};

}  
#endif  