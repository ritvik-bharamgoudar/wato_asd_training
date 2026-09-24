#include "planner_core.hpp"
#include "vector"
#include "cmath"

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) : logger_(logger) {}

    double PlannerCore::calcHeuristic(int idx, int goal_idx) {
    // convert both indices back into (x,y) grid coordinate
    int x1 = idx % WIDTH;
    int y1 = idx / WIDTH;
    int x2 = goal_idx % WIDTH;
    int y2 = goal_idx / WIDTH;

    double dx = x1 - x2;
    double dy = y1 - y2;

    // euclidean distance - can move diagon alley
    return std::sqrt(dx * dx + dy * dy);
    }

    // for a given cell, return valid neighbours to add to search
    std::vector<int> PlannerCore::getNeighbors(int idx, const std::vector<int8_t>& grid) {
        std::vector<int> neighbors;

        // convert index to x (col), y(row)
        int x = idx % WIDTH;
        int y = idx / WIDTH;

        // 8 possible moves: 4 cardinal, 4 diagonal
        const std::array<std::pair<int,int>, 8> directions = {{
            {0,1}, {1,0}, {-1,0}, {0,-1},
            {1,1}, {-1,1}, {1,-1}, {-1,-1}
        }};

        // loop through each surrounding cell
        for (const auto& d : directions) {
            int nx = x + d.first;
            int ny = y + d.second;

            // check bounds
            if (nx >= 0 && nx < WIDTH && ny >= 0 && ny < HEIGHT) {
                int n_idx = ny * WIDTH + nx;

                // invalid if the cell is max cost
                if (grid[n_idx] != OBSTACLE) {
                    neighbors.push_back(n_idx);
                }
            }
        }
        return neighbors;
    }

    // hashmap stores the previous cell of each step - trace back to start
    std::vector<int> PlannerCore::reconstructPath(int goal_idx, std::unordered_map<int,int>& came_from) {
        std::vector<int> path;
        int current = goal_idx;
        path.push_back(current);

        while (came_from.find(current) != came_from.end()) {
            current = came_from[current];
            path.push_back(current);
        }

        // above set of nodes is goal-to-start; reverse for start to goal path
        std::reverse(path.begin(), path.end());
        return path;
    }

    // returns a path: list of grid cells from start to finish 
    std::vector<int> PlannerCore::searchAStar(const std::vector<int8_t>& grid, int start_idx, int goal_idx) {

        // open_set: a min-heap of (f_score, index) pairs - compared on f_score so top gives min f_score
        std::priority_queue<std::pair<double,int>, std::vector<std::pair<double,int>>, std::greater<>> open_set;

        // g_score: the cheapest accumulated cost found so far to reach each cell index
        std::unordered_map<int,double> g_score;

        // came_from: for each cell, which cell led to it on its current best route
        std::unordered_map<int,int> came_from;

        // closed: cells that have already been fully expanded
        std::unordered_set<int> closed;

        // start cell f_score is just the heuristic, since g=0
        g_score[start_idx] = 0.0;
        open_set.push({calcHeuristic(start_idx, goal_idx), start_idx});

        // no goal found if heap empty
        while (!open_set.empty()) {
            // pop the cell with the lowest f_score currently known
            auto [f, current] = open_set.top();
            open_set.pop();

            // if this cell is in closed - no need to expand it
            if (closed.count(current)) continue;

            // exit loop if current at goal and current would have lowest f score in heap
            if (current == goal_idx) return reconstructPath(current, came_from);

            // mark this cell as fully settled -- its g_score is now final
            closed.insert(current);

            // check every valid (in bounds, non-obstacle) neighbor
            for (int n_idx : getNeighbors(current, grid)) {
                if (closed.count(n_idx)) continue; // already settled, skip

                // check if step is diagonal or cardinal
                int cx = current % WIDTH, cy = current / WIDTH;
                int nx = n_idx % WIDTH,   ny = n_idx / WIDTH;
                bool diagonal = (cx != nx) && (cy != ny);

                // distance cost: 1 for cardinal, sqrt(2) for diagonal
                double step_cost = diagonal ? std::sqrt(2.0) : 1.0;

                // costmap penalty: cells near obstacles also carry costs (from inflation)
                // COST_WEIGHT empirically tuned
                double obst_proximity_cost = COST_WEIGHT * static_cast<double>(grid[n_idx]);

                // tentative g: proposed cost to reach this cell: current cost + step cost + any obstacle cost
                double tentative_g = g_score[current] + step_cost + obst_proximity_cost;

                // compare the proposed cost for cell to the g score in hashmap - and only take min
                if (g_score.find(n_idx) == g_score.end() || tentative_g < g_score[n_idx]) {
                    g_score[n_idx] = tentative_g;      // store lower g score
                    came_from[n_idx] = current;        // stepped into from current
                    // evaluate f = g + h and add to heap
                    open_set.push({tentative_g + calcHeuristic(n_idx, goal_idx), n_idx});
                }
            }
        }

        return {};
    }


} 
