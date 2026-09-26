#include "costmap_core.hpp"
#include <cmath>
#include <vector>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {
    occup_grid_ = initialiseCostmap(); // set to empty initially - 1D (row * WIDTH + col)
}


void CostmapCore::processScan(const std::vector<float> &ranges, double angle_min, double angle_increment, double range_min, double range_max) {
    
    occup_grid_ = initialiseCostmap(); // reset grid each call

    for (size_t i=0; i < ranges.size(); i++){
        double angle = angle_min + i * angle_increment;
        double range = ranges[i]; // extract polar coordinate
        int row;
        int col;

        // if nan, leave cells unmarked
        if (std::isnan(range)) {
            continue;
        }
        // if too close, then also leave unmarked
        if (range <= range_min) {
            continue;
        }
        // if beyond the max range mark as free including endpoint
        if (std::isinf(range) || range >= range_max) {
            convertToGrid(range_max, angle, row, col);
            markLine(occup_grid_, (WIDTH/2), (HEIGHT/2), col, row, FREE, FREE);
            continue;
        }

        convertToGrid(range, angle, row, col); // polar -> cartesian -> discrete grid cell
        markLine(occup_grid_, (WIDTH/2), (HEIGHT/2), col, row, MAX_COST, FREE); // set cost to any marked cells
        }
        
    inflateObstacles(occup_grid_); // set linearly decreasing cost to cells surrounding each obstacle
    }



std::vector<int8_t> CostmapCore::initialiseCostmap() {
    std::vector<int8_t> grid((HEIGHT * WIDTH), UNKNOWN); //1D (row * WIDTH + col)
    return grid;
}

void CostmapCore::convertToGrid(double range, double angle, int &row, int &col) {
    double x = range * std::cos(angle);
    double y = range * std::sin(angle);
    col = int(floor(x/RES + WIDTH/2));
    row = int(floor(y/RES + HEIGHT/2)); // convert to cell in grid frame, grid origin = robot_origin(x,y) - (WIDTH/2, HEIGHT/2)
}

// used in inflation to mark cells
void CostmapCore::markObstacles(std::vector<int8_t> &grid, int row, int col, int cost) {
    if (row >= 0 && row < HEIGHT && col >= 0 && col < WIDTH && cost > grid[row*WIDTH + col]) {
        grid[row*WIDTH + col] = cost;
    }
}


// steps along line towards endpoint (obstacle) marking as 0 (free cell)
void CostmapCore::markLine(std::vector<int8_t> &grid, int x0, int y0, int x1, int y1, int8_t cost, int8_t free) {
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);

    // step change in each axis: if +ve then +1, if -ve then -1, otherwise 0
    int step_x = (x1 > x0) ? 1 : (x1 < x0 ? -1 : 0);
    int step_y = (y1 > y0) ? 1 : (y1 < y0 ? -1 : 0);

    int x = x0;
    int y = y0;

    if (dx >= dy) {
        // x is dominant: it steps every iteration 
        // y is minor: it only steps once the running cost overflows dx
        int runningCost = 0;

        for (int i = 0; i <= dx; ++i) {

            // mark current cell free unless this is the final endpoint
            if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT){
            grid[y*WIDTH + x] = (x == x1 && y == y1) ? cost : free;
            }
            if (i == dx) break; // reached endpoint, stop stepping

            x += step_x;                 // dominant axis always advances
            runningCost += dy;       // accumulate the minor delta

            if (runningCost >= dx) { // full circle completed
                y += step_y;             // minor hand advances once
                runningCost -= dx;   // subtract the dominant delta, keep the remainder
            }
        }
    } else {
        // same as above but y is dominant, x is minor
        int runningCost = 0;
        for (int i = 0; i <= dy; ++i) {

            if (x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT){
            grid[y*WIDTH + x] = (x == x1 && y == y1) ? cost : free;
            }
            if (i == dy) break;
            y += step_y;
            runningCost += dx;
            if (runningCost >= dy) {
                x += step_x;
                runningCost -= dy;
            }
        }
    }
}



void CostmapCore::inflateObstacles(std::vector<int8_t> &grid){
    int radius_cells = int(floor(INF_RADIUS / RES)); // obstacle radius in cells
    std::vector<std::tuple<int,int>> obstaCells;
    for (int r=0; r<HEIGHT; r++){
        for (int c=0; c<WIDTH; c++){
            if (grid[r*WIDTH + c] == MAX_COST) {
                obstaCells.push_back({r,c}); // loop full grid and append any cells marked as obstacle
            }
        }
    }

    for (std::tuple<int,int> obst : obstaCells){ // each obstacle cell
        for (int d_row= (radius_cells * -1); d_row <= radius_cells; d_row++){ 
            for (int d_col = (radius_cells * -1); d_col <= radius_cells; d_col++){
                int window_row = std::get<0>(obst) + d_row; 
                int window_col = std::get<1>(obst) + d_col; // each cell of window, where window size is 2*radius_cells x 2* radius_cells centred on the obstacle cell

                if (0<=window_row && window_row<HEIGHT && 0<=window_col && window_col<WIDTH){
                    double dist = sqrt(pow(d_row * RES, 2) + (pow(d_col * RES, 2))); // euclidean distance
                    if (dist <= INF_RADIUS){
                        int obst_cost;
                        // account for robot body hitting obstacles
                        if (dist<=CLEARANCE)
                        {
                            obst_cost = MAX_COST;
                        }
                        else 
                        {
                        //double k = log(100.0) / INF_RADIUS;
                        //int obst_cost = int(MAX_COST * exp(-k*dist)); // cost reaches too far - not sure if affects A* heuristic
                        obst_cost = int(MAX_COST * (1 - ((dist-CLEARANCE)/(INF_RADIUS-CLEARANCE)))); // linearly decreasing cost from edge of CLEARNACE
                        }

                        markObstacles(grid, window_row, window_col, obst_cost);
                    }
                }

                }
            }
        }
}

std::vector<int8_t> CostmapCore::returnGrid() {
    return occup_grid_;
} // used by node to populate /costmap grid data

}     