#include <slam/action_model.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <common_utils/geometric/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>

#include <random>


ActionModel::ActionModel(void)
: k1_(0.1f) // 0.01f
, k2_(0.1f) // 0.01f
, min_dist_(0.0025)
, min_theta_(0.02)
, initialized_(false)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    initialized_ = true;
    alpha_ = 0.0;
    ds_ = 0.0;
    dtheta_ = 0.0;
    utime_ = 0;
    previousPose_.x = 0; previousPose_.y = 0; previousPose_.theta = 0;
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
    
    // std::cout << "previousPose_.theta = " << previousPose_.theta << ", odometry.theta = " << odometry.theta << "\n";
    // std::cout << "fabs(dtheta) = " << fabs(dtheta) << ", fabs(ds) = " << fabs(ds) << "\n\n";
    if (fabs(ds) >= min_dist_ || fabs(dtheta) >= min_theta_) {
        moved = 1;
        alpha_ = alpha;
        ds_ = ds;
        dtheta_ = dtheta;
        utime_ = odometry.utime;

        // std::cout << "alpha_= " << alpha_ << ", ds_ = " << ds_ << ", dtheta_ = " << dtheta_ << "\n";
        // std::cout << "k1_ = " << k1_ << "k2_ = " << k2_ << "\n";

        previousPose_ = odometry;

        // std::normal_distribution<double> eps1_(0.0, k1_*fabs(alpha_));
        // std::normal_distribution<double> eps2_(0.0, k2_*fabs(ds_));
        // std::normal_distribution<double> eps3_(0.0, k1_*fabs(dtheta_ - alpha_));
    }
    return moved;
}

mbot_lcm_msgs::particle_t ActionModel::applyAction(const mbot_lcm_msgs::particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.

    // Sam's Notes: sample here is the parent sample...
    mbot_lcm_msgs::particle_t newSample = sample;
    // Setup random distributions for each epsilon
    std::mt19937 gen(rd_());
    std::normal_distribution<double> eps1_(0.0, k1_*fabs(alpha_));
    std::normal_distribution<double> eps2_(0.0, k2_*fabs(ds_));
    std::normal_distribution<double> eps3_(0.0, k1_*fabs(dtheta_ - alpha_));
    // std::cout << "eps1_(gen) = " << eps1_(gen) << ", " << eps1_(gen) << ", " << eps1_(gen) << ", " << eps1_(gen) << ", " << eps1_(gen) << ", " << "\n";

    // Randomized positions (Action&SensorModel.pdf, Slide 5)
    newSample.parent_pose = sample.pose;
    newSample.pose.x = sample.pose.x + (ds_ + eps2_(gen)) * cos(sample.pose.theta + alpha_ + eps1_(gen));
    newSample.pose.y = sample.pose.y + (ds_ + eps2_(gen)) * sin(sample.pose.theta + alpha_ + eps1_(gen));
    newSample.pose.theta = sample.pose.theta + dtheta_ + eps1_(gen) + eps3_(gen);

    // std::cout << "sample.pose = (" << sample.pose.x << ", " << sample.pose.y << ", " << sample.pose.theta << ")\n";
    // std::cout << "newSample.pose = (" << newSample.pose.x << ", " << newSample.pose.y << ", " << newSample.pose.theta << ")\n\n";

    return newSample;
}
