#include <utils/grid_utils.hpp>
#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/pose_xyt_t.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <cassert>

#include <utils/grid_utils.hpp>
#include <common_utils/geometric/point.hpp>


ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles),
  samplingAugmentation(0.5, 0.9, numParticles),
  distribution_quality(1),
  quality_reinvigoration_percentage(0.1)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const mbot_lcm_msgs::pose_xyt_t& pose)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    posteriorPose_ = pose;
    actionModel_.resetPrevious(posteriorPose_);
    mbot_lcm_msgs::particle_t sample; // make a sample, assigning its parent_pose and pose, push it kNumParticles_ times
    sample.parent_pose = posteriorPose_;
    sample.pose = pose;
    sample.weight = 1.0/kNumParticles_;
    while (posterior_.size() < kNumParticles_) {
        posterior_.push_back(sample);
    }
}

void ParticleFilter::initializeFilterRandomly(const OccupancyGrid& map)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    // Random stuff
    std::random_device rd;
    std::mt19937 gen(rd());
    double mean = 0.0, stddev = 1.0;
    std::normal_distribution<double> d(mean, stddev);

    // sample stuff
    mbot_lcm_msgs::particle_t sample; // make a sample, assigning its parent_pose and pose, push it kNumParticles_ times
    mbot_lcm_msgs::pose_xyt_t pose; // make a pose, set it to 0, assign it to sample.parent_pose
    pose.x = 0; pose.y = 0; pose.theta = 0;
    posteriorPose_ = pose;
    actionModel_.resetPrevious(posteriorPose_);


    sample.parent_pose = posteriorPose_;
    sample.weight = 1.0/kNumParticles_;
    Point<double> global_point; // initialize variables for checking map
    Point<int> map_point;
    int score;

    while (posterior_.size() < kNumParticles_) {
        global_point.x = d(gen); global_point.y = d(gen); // generate random x, y
        map_point = global_position_to_grid_cell(global_point, map); // get grid cell position
        score = map.logOdds(map_point.x, map_point.y); // check the score

        if (score < 0) { // if the score is less than 0, then it's unoccupied based on laser scan, so we can place robot there (if >= 0, either not in map or occupied)
            pose.x = global_point.x; pose.y = global_point.y; pose.theta = d(gen)/10; // divide by a number since it's theta
            posterior_.push_back(sample);
        }
    }
}

void ParticleFilter::resetOdometry(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    actionModel_.resetPrevious(odometry);
}


mbot_lcm_msgs::pose_xyt_t ParticleFilter::updateFilter(const mbot_lcm_msgs::pose_xyt_t& odometry,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    auto prior = resamplePosteriorDistribution(&map);
    auto proposal = computeProposalDistribution(prior);
    posterior_ = computeNormalizedPosterior(proposal, laser, map);
    // OPTIONAL TODO: Add reinvigoration step
    posteriorPose_ = estimatePosteriorPose(posterior_);
    posteriorPose_.utime = odometry.utime;

    return posteriorPose_;
}

mbot_lcm_msgs::pose_xyt_t ParticleFilter::updateFilterActionOnly(const mbot_lcm_msgs::pose_xyt_t& odometry)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);

    if(hasRobotMoved)
    {
        auto prior = resamplePosteriorDistribution();
        auto proposal = computeProposalDistribution(prior);
        posterior_ = proposal;
    }

    posteriorPose_ = odometry;

    return posteriorPose_;
}



mbot_lcm_msgs::pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


mbot_lcm_msgs::particles_t ParticleFilter::particles(void) const
{
    mbot_lcm_msgs::particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


ParticleList ParticleFilter::resamplePosteriorDistribution(const OccupancyGrid* map)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    ParticleList prior;
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> d(0.0, 1.0 / kNumParticles_);
    double r = d(gen);
    double U = 0.0;
    double c = posterior_[0].weight;
    int i = 0;

    for (int m=1; m<=kNumParticles_; m++) {
        U = r + (m-1.0) / kNumParticles_;
        while (U > c) {
            i += 1;
            c += posterior_[i].weight;
        }
        prior.push_back(posterior_[i]);
    }
    return prior;
}


ParticleList ParticleFilter::computeProposalDistribution(const ParticleList& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    ParticleList proposal;
    for (auto particle : prior) {
        proposal.push_back(actionModel_.applyAction(particle));
    }
    // actionModel_.resetPrevious(posteriorPose_); Not sure where to do this...
    return proposal;
}


ParticleList ParticleFilter::computeNormalizedPosterior(const ParticleList& proposal,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the
    ///////////       particles in the proposal distribution
    ParticleList posterior; // initilize new particle set
    double eta; // initialize the normalization factor
    
    for (auto particle : proposal) {
        // apply sensor model to compute importance weight
        double weight = sensorModel_.likelihood(particle, laser, map); 
        
        // update normalization factor
        eta += weight; 
        
        // add to new particle set
        mbot_lcm_msgs::particle_t newParticle;
        newParticle.pose = particle.pose;
        newParticle.weight = weight;
        posterior.push_back(newParticle); 
    }
    
    for (auto particle : posterior) {
        // normalize weight
        particle.weight /= eta; 
    }
    return posterior;
}


mbot_lcm_msgs::pose_xyt_t ParticleFilter::estimatePosteriorPose(const ParticleList& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    mbot_lcm_msgs::pose_xyt_t pose;
    mbot_lcm_msgs::particle_t bestParticle;
    double highestWeight = 0;
    for (auto particle : posterior) {
        if (particle.weight > highestWeight) {
            bestParticle = particle;
            highestWeight = bestParticle.weight;
        }
        pose = bestParticle.pose;
    }
    return pose;
}

mbot_lcm_msgs::pose_xyt_t ParticleFilter::computeParticlesAverage(const ParticleList& particles_to_average)
{
    //////// TODO: Implement your method for computing the average of a pose distribution
    // whether this should be weighted average?
    mbot_lcm_msgs::pose_xyt_t avg_pose;
    for (auto particle : particles_to_average) {
        avg_pose.x += particle.weight * particle.pose.x;
        avg_pose.y += particle.weight * particle.pose.y;
        avg_pose.theta += particle.weight * particle.pose.theta;
    }
    return avg_pose;
}
