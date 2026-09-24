#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

namespace robot
{

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

  private:
    rclcpp::Logger logger_;

    double calcHeuristic(int idx, int goal_idx);
    std::vector<int> getNeighbors(int idx, const std::vector<int8_t>& grid);
    std::vector<int> reconstructPath(int goal_idx, std::unordered_map<int,int>& came_from);
    std::vector<int> searchAStar(int start_idx, int goal_idx, const std::vector<int8_t>& grid);
    
};

}  

#endif   
