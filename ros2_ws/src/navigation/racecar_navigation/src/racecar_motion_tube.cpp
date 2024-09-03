#include "racecar_navigation/racecar_motion_tube.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include<iostream>
#include <stdbool.h>

BarnMotionTube::BarnMotionTube()
{
    // number of curvatures (n), initial curvature (r0), minimum curvature (rmin)
    params_.r0 = 20;
    params_.rmin = 0.03;
    params_.number_of_curvatures = 50; 
    // Maximum angular rotation (wmax)
    params_.max_angular_rate = M_PI/2;
    // vector of (velocity, time horizon)
    // 0.1
    params_.vec_mp_params.push_back({.forward_velocity=0.1,.time_horizon=0.1/0.1});
    params_.vec_mp_params.push_back({.forward_velocity=0.25,.time_horizon=.1/.25});
    // .25 m
    // params_.vec_mp_params.push_back({.forward_velocity=0.25,.time_horizon=2});
    params_.vec_mp_params.push_back({.forward_velocity=0.25,.time_horizon=0.25/0.25});
    params_.vec_mp_params.push_back({.forward_velocity=0.5,.time_horizon=.25/.5});
    // .5 m
    // params_.vec_mp_params.push_back({.forward_velocity=0.5,.time_horizon=2});
    params_.vec_mp_params.push_back({.forward_velocity=0.5,.time_horizon=.5/.5});
    params_.vec_mp_params.push_back({.forward_velocity=0.75,.time_horizon=.75/.5});
    // .75
    // params_.vec_mp_params.push_back({.forward_velocity=.75,.time_horizon=2});
    params_.vec_mp_params.push_back({.forward_velocity=.75,.time_horizon=.75/.75});
    params_.vec_mp_params.push_back({.forward_velocity=1.0,.time_horizon=.75/1.0});
    // 1. m
    // params_.vec_mp_params.push_back({.forward_velocity=1.0,.time_horizon=2});
    params_.vec_mp_params.push_back({.forward_velocity=1.0,.time_horizon=1./1.0});
    params_.vec_mp_params.push_back({.forward_velocity=1.25,.time_horizon=1./1.25});
    // 1.25 m
    // params_.vec_mp_params.push_back({.forward_velocity=1.25,.time_horizon=2});
    params_.vec_mp_params.push_back({.forward_velocity=1.25,.time_horizon=1.25/1.25});
    params_.vec_mp_params.push_back({.forward_velocity=1.5,.time_horizon=1.25/1.5});
    // 1.5 m
    params_.vec_mp_params.push_back({.forward_velocity=1.5,.time_horizon=1.5/1.5});
    params_.vec_mp_params.push_back({.forward_velocity=1.75,.time_horizon=1.5/1.75});
    // params_.vec_mp_params.push_back({.forward_velocity=-0.5,.time_horizon=4.0/4});

    // Max number of samples
    params_.max_number_of_samples = 500;
    // Sampling parameters
    params_.sampling_interval = 0.03;
    // Footprint
    double x_inflation = 0.10;
    double y_inflation = 0.05;
    params_.footprint[FRONT_LEFT].x = 0.21 + x_inflation;
    params_.footprint[FRONT_LEFT].y = 0.165 + y_inflation;
    params_.footprint[AXLE_LEFT].x =  -0. ;
    params_.footprint[AXLE_LEFT].y = 0.165 + y_inflation;
    params_.footprint[FRONT_RIGHT].x = 0.21 + x_inflation;
    params_.footprint[FRONT_RIGHT].y = -0.165 - y_inflation;
    params_.footprint[AXLE_RIGHT].x =  -0.;
    params_.footprint[AXLE_RIGHT].y = -0.165 - y_inflation;
}

void BarnMotionTube::Configure(range_sensor_t *range_sensor)
{
    /*************************PARAMETERS***************************/
    // number of curvatures (n), initial curvature (r0), minimum curvature (rmin)
    double r0 = params_.r0;
    double rmin =params_.rmin;
    size_t number_of_curvatures = params_.number_of_curvatures; 
    // Maximum angular rotation (wmax)
    double max_angular_rate = params_.max_angular_rate;
    // vector of (velocity, time horizon)
    std::vector<motion_primitive_params_t> vec_mp_params = params_.vec_mp_params;
    // Max number of samples
    size_t max_number_of_samples =  params_.max_number_of_samples;
    // Sampling parameters
    double sampling_interval = params_.sampling_interval;

    // Compute curvatures
    std::vector<double> vec_curvature;
    compute_vector_of_curvatures(-rmin, -r0, number_of_curvatures-1, vec_curvature);
    vec_curvature.push_back(-10000.0);
    vec_curvature.push_back(10000.0);
    compute_vector_of_curvatures(r0, rmin, number_of_curvatures-1, vec_curvature);
    // Given wmax, compute for each combination of (time horizon, velocity) 
    int w = 0;
    for (auto &mp_params: vec_mp_params)
    {
        motion_tube_.push_back({});
        motion_primitive_.push_back({});
        size_t i=0;
        double min_admissable_curvature = compute_min_curvature(mp_params.forward_velocity, 
            max_angular_rate);
        // std::cout << "mp_params.forward_velocity: " << mp_params.forward_velocity << ", min_admissable_curvature: " << min_admissable_curvature << std::endl;
        for (auto &curvature: vec_curvature)
        {
            if(fabs(curvature) < min_admissable_curvature)
                continue;
            // Compute motion tube;
            motion_primitive_.back().push_back({});
            motion_primitive_t *motion_primitive =  &motion_primitive_.back().back();
            MotionPrimitiveUnicycle.create(motion_primitive);
            MotionPrimitiveUnicycle.allocate_memory(motion_primitive);
            motion_primitive->time_horizon = mp_params.time_horizon;
            ((unicycle_control_t *) motion_primitive->control)->forward_velocity = mp_params.forward_velocity;
            ((unicycle_control_t *) motion_primitive->control)->angular_rate =  mp_params.forward_velocity/curvature;

            motion_tube_.back().push_back({});
            motion_tube_t *motion_tube =  &motion_tube_.back().back();
            MotionTube.create(motion_tube);
            MotionTube.allocate_memory(motion_tube, &max_number_of_samples, 1);     
            // Sample tube   
            MotionTube.sample(motion_tube, sampling_interval, 
                params_.footprint, motion_primitive, range_sensor, &params_.pose_sensor);
            int nb_samples =  motion_tube->cartesian.left.number_of_points + 
                motion_tube->cartesian.right.number_of_points+
                motion_tube->cartesian.front.number_of_points;
        }
    }

    // Allocate memory for evaluation vectors
    size_t number_of_horizons = motion_tube_.size();
    curvature_offset_ = (size_t*) malloc(sizeof(size_t)*number_of_horizons); 
    availability_ = (bool**) malloc(sizeof(bool*)*number_of_horizons);
    final_position_ = (point2d_t**) malloc(sizeof(point2d_t*)*number_of_horizons);
    for(int i=0; i<motion_tube_.size(); i++)    // ith is the horizon
    {
        size_t number_of_curvatures_ith_horizon = motion_tube_[i].size();   
        availability_[i] = (bool*) malloc(sizeof(bool)*number_of_curvatures_ith_horizon);
        memset(availability_[i],0,sizeof(bool)*number_of_curvatures_ith_horizon);
        curvature_offset_[i] = (size_t) (2*number_of_curvatures - number_of_curvatures_ith_horizon)/2; 
        final_position_[i] = (point2d_t*) malloc(sizeof(point2d_t)*number_of_curvatures_ith_horizon);
        // std::cout << "number_of_horizons: " << number_of_horizons << 
        //     " number_of_curvatures: " << 2*number_of_curvatures << 
        //     " number_of_curvatures_ith_horizon: " << number_of_curvatures_ith_horizon <<
        //     " curvature_offset_: " <<   curvature_offset_[i]  << std::endl;
        // std::cout << availability_[i] << " " << curvature_offset_[i] << std::endl;
    }

    for(size_t i=0; i<motion_tube_.size(); i++)    // ith is the horizon
    {
        for(size_t j=0; j<motion_tube_[i].size(); j++)    // jth is the curvature
        {
            pose2d_t final_pose, initial_pose = {
                .x = params_.footprint[FRONT_LEFT].x + params_.sampling_interval/2, 
                .y=0, 
                .yaw=0};
            MotionPrimitiveUnicycle.excite(
                motion_primitive_[i][j].control, 
                params_.vec_mp_params[i].time_horizon, 
                &initial_pose, 
                &final_pose);
            final_position_[i][j].x = final_pose.x;
            final_position_[i][j].y = final_pose.y;
        }
    }

}
 
void BarnMotionTube::Compute(range_sensor_t *range_sensor, range_scan_t *range_scan)
{
    lidar_t lidar = {.range_sensor=range_sensor, .range_scan=range_scan};
    // Iterate tubes
    for(size_t i=0; i<motion_tube_.size(); i++)    // ith is the horizon
    {
        for(size_t j=0; j<motion_tube_[i].size(); j++)    // jth is the curvature
        {
            MotionTube.Monitor.availability(&motion_tube_[i][j], &lidar, &availability_[i][j]);
        }
    }
}

void BarnMotionTube::PrintAvailability()
{
    // // Iterate tubes
    for(size_t i=0; i<motion_tube_.size(); i++)    // ith is the horizon
    {
        size_t offset = curvature_offset_[i];
        for(size_t j=0; j<2*params_.number_of_curvatures; j++)    // jth is the curvature
        {
            if(j<offset || j > (2*params_.number_of_curvatures-1) - offset)
            {
                std::cout << "0 ";
            }else
            {
                std::cout << availability_[i][j-offset] << " ";
            }
        }
        std::cout << std::endl;
    }

    // for(size_t i=0; i<motion_tube_.size(); i++)    // ith is the horizon
    // {
    //     for(size_t j=0; j<motion_tube_[i].size(); j++)    // jth is the curvature
    //     {
    //         printf("%d, ", availability_[i][j]);
    //     }
    // }
    // std::cout << std::endl;
}



void print_cartesian_points(motion_tube_cartesian_t *motion_tube_cartesian)
{
    const point2d_array_t *samples[3] = {&motion_tube_cartesian->left, 
        &motion_tube_cartesian->front, &motion_tube_cartesian->right};

     for (size_t i=0; i<3; i++){
        for(size_t j=0; j<samples[i]->number_of_points; j++){
            std::cout << samples[i]->points[j].x << " " <<samples[i]->points[j].y << " "; 
        }
    }
    std::cout << std::endl;
}

void print_beams(motion_tube_sensor_space_t *motion_tube_sensor_space)
{
    const free_space_beam_t *beams = motion_tube_sensor_space->beams;

     for (size_t i=0; i<motion_tube_sensor_space->number_of_beams; i++){
        std::cout << beams[i].range_outer << " " << beams[i].angle << " "; 
    }
    std::cout << std::endl;
}


double compute_min_curvature(double forward_velocity, double max_angular_rate)
{
    return forward_velocity/max_angular_rate;
}

double compute_curvature_radix(double r0, double rmin, size_t number_of_samples)
{
    return exp(1.0/(number_of_samples-1)*log(r0/rmin));
}

void compute_vector_of_curvatures(double r0, double rmin, 
    size_t number_of_curvatures, std::vector<double> &curvatures)
{
    double radix = compute_curvature_radix(r0, rmin, number_of_curvatures);
    for(int i=0; i<number_of_curvatures; i++)
    {
        curvatures.push_back(r0/pow(radix,i));
    }
}
