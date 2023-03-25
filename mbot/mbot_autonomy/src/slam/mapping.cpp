#include <slam/mapping.hpp>
#include <utils/grid_utils.hpp>
#include <numeric>
#include <chrono> 
using namespace std::chrono; 

Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
, initialized_(false)
{
}


void Mapping::updateMap(const mbot_lcm_msgs::lidar_t& scan,
                        const mbot_lcm_msgs::pose_xyt_t& pose,
                        OccupancyGrid& map)
{
    //////////////// TODO: Implement your occupancy grid algorithm here ///////////////////////
    MovingLaserScan movingScan(scan, previousPose_, pose);

    for(auto& ray : movingScan) {
        // if ray range is greater than max laser distance, I think it's a bad reading
        if (ray.range <= kMaxLaserDistance_) {
            scoreEndpoint(ray, map);
        }
    }
    for(auto& ray : movingScan) {
        if (ray.range <= kMaxLaserDistance_) {
            scoreRay(ray, map);
        }
    }

    previousPose_ = pose;
}

void Mapping::scoreEndpoint(const adjusted_ray_t& ray, OccupancyGrid& map)
{
//////////////// TODO: Implement your endpoint score ///////////////////////
    Point<float> f_end = global_position_to_grid_position(
        Point<float>(
            ray.origin.x + ray.range * std::cos(ray.theta),
            ray.origin.y + ray.range * std::sin(ray.theta)
            ), 
        map
        );
    // Cell
    Point<int> end_cell;
    end_cell.x = static_cast<int>(f_end.x);
    end_cell.y = static_cast<int>(f_end.y);
    // added new map.addLogOdds function (in occupancy_grid.cpp) that checks upper/lowerbounds of type
    // Add (positive) kMissOdds_ because there's no easy way to stop scoreRay short of also scoring the end of the laser.
    map.addLogOdds(end_cell.x, end_cell.y, kHitOdds_ + kMissOdds_);

    return;
}

void Mapping::scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map)
{
//////////////// TODO: Implement your ray score ///////////////////////
    std::vector<Point<int>> pointsTouched = Mapping::bresenham(ray, map);
    for (Point<int> point : pointsTouched) {
        map.addLogOdds(point.x, point.y, -kMissOdds_);
    }
    return;
}

/*
Takes the ray and map, and returns a vector of map cells to check
*/
std::vector<Point<int>> Mapping::bresenham(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    // Get global positions 
    Point<float> f_end = global_position_to_grid_position(
        Point<float>(
            ray.origin.x + ray.range * std::cos(ray.theta),
            ray.origin.y + ray.range * std::sin(ray.theta)
            ), 
        map
        );

    // Cells
    Point<int> start_cell = global_position_to_grid_cell(ray.origin, map);
    Point<int> end_cell;
    end_cell.x = static_cast<int>(f_end.x);
    end_cell.y = static_cast<int>(f_end.y);
    std::vector<Point<int>> cells_touched;
    //////////////// TODO: Implement Bresenham's Algorithm ////////////////
    int x0 = start_cell.x, y0 = start_cell.y, x1 = end_cell.x, y1 = end_cell.y;
    
    // from https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
    if (abs(y1 - y0) < abs(x1 - x0)) { // line low case
        if (x0 > x1) {
            plotLineLow(x1, y1, x0, y0, cells_touched);
        }
        else {
            plotLineLow(x0, y0, x1, y1, cells_touched);
        }
    }
    else { // line high case
        if (y0 > y1) {
            plotLineHigh(x1, y1, x0, y0, cells_touched);
        }
        else {
            plotLineHigh(x0, y0, x1, y1, cells_touched);
        }
    }
    return cells_touched;
}

std::vector<Point<int>> Mapping::divideAndStepAlongRay(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    auto end_cell = global_position_to_grid_cell(Point<double>(
        ray.origin.x + ray.range * std::cos(ray.theta),
        ray.origin.y + ray.range * std::sin(ray.theta)
        ), map);
    //////////////// TODO: Implement divide and step ////////////////
    std::vector<Point<int>> cells_touched;
    return cells_touched;
}





void Mapping::plotLineLow(int x0, int y0, int x1, int y1, std::vector<Point<int>>& cells_touched) {
    int dx = x1 - x0;
    int dy = y1 - y0;
    int yi = 1;
    if (dy < 0) {
        yi = -1;
        dy = -dy;
    }
    int D = (2*dy) - dx;
    int y = y0;

    for (int x=x0; x<=x1; x++) {
        cells_touched.push_back(Point<int>(x,y));
        if (D > 0) {
            y += yi;
            D += 2*(dy - dx);
        }
        else {
            D += 2*dy;
        }
    }
    return;
}

void Mapping::plotLineHigh(int x0, int y0, int x1, int y1, std::vector<Point<int>>& cells_touched) {
    int dx = x1 - x0;
    int dy = y1 - y0;
    int xi = 1;
    if (dx < 0) {
        xi = -1;
        dx = -dx;
    }
    int D = (2*dx) - dy;
    int x = x0;

    for (int y=y0; y<=y1; y++) {
        cells_touched.push_back(Point<int>(x,y));
        if (D > 0) {
            x += xi;
            D += 2*(dx - dy);
        }
        else {
            D += 2*dx;
        }
    }
    return;
}