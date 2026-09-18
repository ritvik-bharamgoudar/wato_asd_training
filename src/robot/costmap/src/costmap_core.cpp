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

OUTPUT: OccupancyGrid(costmap)

CONSTANTS:
WIDTH = 100
HEGHT = 100
RES = 0.1
INF_RADIUS = 10 // number of cells
MAX_COST = 100

FUNCTION initialiseCostmap()
    Occup_grid = zeros)WIDTH,HEIGHT)
    RETURN Occup_Grid

FUNCTOIN laserCallback(scan)

    grid = initialiseCostmap

    for (size_t i=0; i< scan-> ranges.size(); i++)
    double angle = scan->angle_min + i * scan->angle_increment;
    double range = scan->ranges[i];

    if (range < scan->range_max && range > scan->range_min)
        (row,col)= comvertToGrid(range,angle)

        FUNCTION convertToGrid(range, angle)
            x = range * cos(angle)
            y = range * sin(angle)
            col = floor(x / RES + WIDTH / 2)
            row = floor(y / RES + HEIGHT / 2)
            RETURN (row,col)

        markObstacle(OccupGrid. row, col)

        FUNCTION markObstacle(OccupGrid,row,col)
            if 0<=row<HEIGHT && 0<=col<WIDTH
                grid[row][col] = MAX_COST
        
    inflateObstacles(OccupGrid, INF_RADIUS)

    FUNCTION inflateObstacles(OccupGrid, INF_RADIUS)




            if 0<=grid_row<width && 0<=grid_col<height:
                OccupancyGrid[row][col] = 100

            
inflateObstacles(OccupancyGrid, infl_radius):
for row, col in OccupancyGrid
    if Occupancygrid[row][col] == 100:
        starters.append([row,col])
        std::array<stf::array<int, 10>, 3> window{};
        h = window.height
        w = window.width
        for [row][col] in window:
            cost = 100 * (1- root((row-w/2)^2 + (col - h/2)^2)))
            if 0<= (starter[row][col] + window_row - window_width/2)) < width && smae for column:
                && if cost > OccupancyGrid[row][col]:
                    OccupancyGrid[row][col] = cost
intialise Occupancygrid()
            


*/
