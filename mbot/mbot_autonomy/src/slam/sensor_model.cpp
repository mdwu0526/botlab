#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <slam/occupancy_grid.hpp>
#include <utils/grid_utils.hpp>
#include <common_utils/geometric/point.hpp>
SensorModel::SensorModel(void)
:   ray_stride_(1)
{
}

double SensorModel::likelihood(const mbot_lcm_msgs::particle_t& sample, 
                               const mbot_lcm_msgs::lidar_t& scan, 
                               const OccupancyGrid& map)
{
    double likelihood = 0.0;
    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose, ray_stride_);
    // TODO
    for(const auto &ray : movingScan){
        likelihood += scoreRay(ray,map);
    }
    return likelihood;
}

double SensorModel::scoreRay(const adjusted_ray_t& ray, const OccupancyGrid& map){
    double score = 0.000001;
    double cell_length = map.metersPerCell();
    double cell_diagonal = sqrt(pow(cell_length,2)+pow(cell_length,2));

    // Finds the endpoint of the ray
    Point<float> f_end = global_position_to_grid_position(
        Point<float>(
            ray.origin.x + ray.range * std::cos(ray.theta),
            ray.origin.y + ray.range * std::sin(ray.theta)
            ), 
        map
        );
    Point<float> f_end_before = global_position_to_grid_position(
        Point<float>(
            ray.origin.x + (ray.range - cell_diagonal) * std::cos(ray.theta),
            ray.origin.y + (ray.range - cell_diagonal) * std::sin(ray.theta)
            ), 
        map
        );
    Point<float> f_end_after = global_position_to_grid_position(
        Point<float>(
            ray.origin.x + (ray.range + cell_diagonal) * std::cos(ray.theta),
            ray.origin.y + (ray.range + cell_diagonal) * std::sin(ray.theta)
            ), 
        map
        );

    // Stores the endpoint into a Point
    Point<int> end_cell;
    end_cell.x = static_cast<int>(f_end.x);
    end_cell.y = static_cast<int>(f_end.y);

    Point<int> end_cell_after;
    end_cell_after.x = static_cast<int>(f_end_after.x);
    end_cell_after.y = static_cast<int>(f_end_after.y);

    Point<int> end_cell_before;
    end_cell_before.x = static_cast<int>(f_end_before.x);
    end_cell_before.y = static_cast<int>(f_end_before.y);

    // Gets the logOdds score from the endpoint and stores it into the score variable
    // Checks if logOdds is positive
    if(map.logOdds(end_cell.x,end_cell.y) > 0){
        score = static_cast<double>(map.logOdds(end_cell.x,end_cell.y));
    }
    // If not positive, check the cell in the map one cell after or before the ray and check which one has a higher logOdds.
    // Take 30% of the logOdds of the better cell and add it to the score
    else{
        if(map.logOdds(end_cell_after.x,end_cell_after.y) > 0){
            score = map.logOdds(end_cell_after.x,end_cell_after.y) * 0.30;
        }
        if(map.logOdds(end_cell_before.x,end_cell_before.y) > 0 || map.logOdds(end_cell_before.x,end_cell_before.y) > map.logOdds(end_cell_after.x,end_cell_after.y)){
            score = map.logOdds(end_cell_before.x,end_cell_before.y) * 0.30;
        }
    }

    return score;
}