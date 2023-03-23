#include <slam/action_model.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <common_utils/geometric/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>


ActionModel::ActionModel(void)
: k1_(0.01f)
, k2_(0.01f)
, min_dist_(0.0025)
, min_theta_(0.02)
, initialized_(false)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
}


void ActionModel::resetPrevious(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    previousPose_ = odometry;
}


bool ActionModel::updateAction(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    bool moved = 0;
    double dx = odometry.x - previousPose_.x,
           dy = odometry.y - previousPose_.y,
           dtheta = odometry.theta - previousPose_.theta;

    double ds = sqrt(pow(dx,2) + pow(dy,2)),
           alpha = atan2(dy,dx) - previousPose_.theta;
    if (ds < min_dist_ && abs(dtheta) < min_theta_) {
        return moved;
    }
    else {
        moved = 1;
        alpha_ = alpha;
        ds_ = ds;
        dthetaMalpha_ = dtheta - alpha;
        utime_ = previousPose_.utime;

        alphaStd_ = k1_*abs(alpha_);
        dsStd_ = k2_*abs(ds_);
        dthetaMalphaStd_ = k1_*abs(dthetaMalpha_);
        
        // resetPrevious(odometry);
        return moved;
    }
}

mbot_lcm_msgs::particle_t ActionModel::applyAction(const mbot_lcm_msgs::particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    mbot_lcm_msgs::particle_t newSample = sample;
    return newSample;
}
