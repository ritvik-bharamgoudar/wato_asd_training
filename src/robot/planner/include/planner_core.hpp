#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <unordered_map>
#include <cstdint>

const int WIDTH = 200;
const int HEIGHT = 200;
const int8_t OBSTACLE = 100;
const double COST_WEIGHT = 0.05;


namespace robot
{

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger); 

    std::vector<int> searchAStar(const std::vector<int8_t>& grid, int start_idx, int goal_idx);
    

  private:
    rclcpp::Logger logger_;

    double calcHeuristic(int idx, int goal_idx);
    std::vector<int> getNeighbors(int idx, const std::vector<int8_t>& grid);
    std::vector<int> reconstructPath(int goal_idx, std::unordered_map<int,int>& came_from);

};

}  

#endif   
