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
        
        if (range > range_min && range < range_max){ // discard if out of Lidar range
            int row;
            int col;
            convertToGrid(range, angle, row, col); // polar -> cartesian -> discrete grid cell
            markObstacles(occup_grid_, row, col, MAX_COST); // set cost to any marked cells
            }
        }
    inflateObstacles(occup_grid_); // set linearly decreasing cost to cells surrounding each obstacle
}



std::vector<int8_t> CostmapCore::initialiseCostmap() {
    std::vector<int8_t> grid((HEIGHT * WIDTH), 0); //1D (row * WIDTH + col)
    return grid;
}

void CostmapCore::convertToGrid(double range, double angle, int &row, int &col) {
    double x = range * cos(angle);
    double y = range * sin(angle);
    col = int(floor(x/RES + WIDTH/2));
    row = int(floor(y/RES + HEIGHT/2)); // change to grid frame, grid origin = robot_origin(x,y) - (WIDTH/2, HEIGHT/2)
}

void CostmapCore::markObstacles(std::vector<int8_t> &grid,int row, int col, int cost) {
    if (row >= 0 && row < HEIGHT && col >= 0 && col < WIDTH && cost > grid[row*WIDTH + col]) {
        grid[row*WIDTH + col] = cost;
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
                        //double k = log(100.0) / INF_RADIUS;
                        //int obst_cost = int(MAX_COST * exp(-k*dist)); // cost reaches too far - not sure if affects A* heuristic
                        int obst_cost = int(MAX_COST * (1 - (dist/INF_RADIUS))); // linearly decreasing cost
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




/*
costmap pseudo code:

INPUT: laserscan(arr[ranges], float(min_angle). 
float(min_increment), float(range_min), float(range_max))

OUTPUT: OccupancyGrid(/costmap)
    requires:
    msg.header.stamp
    msg.header.frame_id //robot frame
    msg.info.resolution
    msg.info.width
    msg.info.height
    msg.info.origin // (0,0) to bottom left of robot - so x = -WIDTH/2 * RES, y = -HEIGHT/2 * RES
    msg.data // 1D array

CONSTANTS:
INT WIDTH = 100
INT HEGHT = 100
DOUBLE RES = 0.1
DOUBLE INF_RADIUS = 10.0 // metres
INT MAX_COST = 100

FUNCTION initialiseCostmap()
    MATRIX Occup_grid = zeros)WIDTH,HEIGHT)
    RETURN Occup_Grid

FUNCTOIN laserCallback(scan)

    grid = initialiseCostmap

    for (size_t i=0; i< scan-> ranges.size(); i++)
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];

    if (range < scan->range_max && range > scan->range_min)
        (row,col)= comvertToGrid(range,angle)

        FUNCTION convertToGrid(range, angle)
            DOUBLE x = range * cos(angle)
            DOUBLE y = range * sin(angle)
            INT col = floor(x / RES + WIDTH / 2)
            INT row = floor(y / RES + HEIGHT / 2)
            RETURN (row,col)


        if 0<=row<HEIGHT && 0<=col<WIDTH
            grid[row][col] = MAX_COST
        
    inflateObstacles(OccupGrid, INF_RADIUS)

    FUNCTION inflateObstacles(OccupGrid, INF_RADIUS)
        radius_cells = floor(INF_RADIUS / RES) //obstacle window in cell distance
        obstaCells = []
        for row in 0:HEIGHT-1
            for col in 0:WIDTH-1
                if grid[row][col] == MAX_COST
                    obstaCells.append((row,col)) // starter array of all obstacle cells
        
        for (o_row, o_col) in obstaCells
            for d_row in -cell_radius:cell_radius
                for d_col in -cell_radius:cell_radius // for a window around cell distance 
                    window_row = o_row + d_row
                    window_col = o_col + d_col // loop each cell of window

                    if 0<=window_row<HEIGHT && 0<=window_col<WIDTH
                        dist = sqrt((d_row * RES)^2 + (d_col * RES)^2)

                        if dist <= INF_RADIUS
                            cost = MAX_COST * (1 - dist/INF_RADIUS)
                            if cost > grid[window_row][window_col]
                                grid[window_row][window_col] = cost
        publishCostmap()

        FUNCTION publishCostmap()
            occupArray = []
            for row in 0:HEIGHT-1
                for col in 0:WIDTH-1
                    occupArray.append(occupGrid[row][col])
            RETURN occupArray

        publish occupArray

        
*/
