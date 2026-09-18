#include "costmap_core.hpp"
#include <cmath>

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

Point CostmapCore::polarToCartesian(double radius, float angle) {
            double x = radius * std::cos(angle);
            double y = radius * std::sin(angle);
            return Point{x,y};
        }

void CostmapCore::laserScan(
      const std::vector<float>& radii, 
      float angle_min,
      float angle_increment,
      float range_min,
      float range_max){

      for (int i =0; i < int(radii.size()); i++) {

        double radius = radii[i];
        float angle = angle_min + i * angle_increment;

        Point p = polarToCartesian(radius, angle);

}

} 
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

        markObstacle(OccupGrid. row, col)

        FUNCTION markObstacle(OccupGrid,row,col)
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
