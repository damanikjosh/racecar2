/*
 * SPDX-License-Identifier: BSD-3-Clause
 *
 *  Author(s): Shrijit Singh <shrijitsingh99@gmail.com>
 *  Contributor: Pham Cong Trang <phamcongtranghd@gmail.com>
 *  Contributor: Mitchell Sayer <mitchell4408@gmail.com>
 */

#include <algorithm>
#include <string>
#include <memory>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "fsmt_controller/fsmt_controller.hpp"
#include "nav2_util/geometry_utils.hpp"


using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;

namespace fsmt_controller
{

double compute_points_distance(point2d_t *p1, point2d_t *p2)
{
  return sqrt( pow(p1->x-p2->x, 2) + pow(p1->y-p2->y, 2) );
}

/**
 * Find element in iterator with the minimum calculated value
 */
template<typename Iter, typename Getter>
Iter min_by(Iter begin, Iter end, Getter getCompareVal)
{
  if (begin == end) {
    return end;
  }
  auto lowest = getCompareVal(*begin);
  Iter lowest_it = begin;
  for (Iter it = ++begin; it != end; ++it) {
    auto comp = getCompareVal(*it);
    if (comp < lowest) {
      lowest = comp;
      lowest_it = it;
    }
  }
  return lowest_it;
}

void FSMTController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;

  auto node = node_.lock();

  costmap_ros_ = costmap_ros;
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_forward_vel", rclcpp::ParameterValue(5.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(10.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.4));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".scan_topic", rclcpp::ParameterValue(std::string("scan")));

  node->get_parameter(plugin_name_ + ".max_forward_vel", max_forward_vel_);
  node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
  double transform_tolerance;
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".scan_topic", scan_topic_);

  // Create scan sub with Best Effort QoS
  scan_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
    std::bind(&FSMTController::scanCallback, this, std::placeholders::_1));
  global_pub_ = node->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
}

void FSMTController::cleanup()
{
  RCLCPP_INFO(
    logger_,
    "Cleaning up controller: %s of type fsmt_controller::FSMTController",
    plugin_name_.c_str());
  global_pub_.reset();
}

void FSMTController::activate()
{
  RCLCPP_INFO(
    logger_,
    "Activating controller: %s of type fsmt_controller::FSMTController\"  %s",
    plugin_name_.c_str(),plugin_name_.c_str());
  global_pub_->on_activate();
}

void FSMTController::deactivate()
{
  RCLCPP_INFO(
    logger_,
    "Dectivating controller: %s of type fsmt_controller::FSMTController\"  %s",
    plugin_name_.c_str(),plugin_name_.c_str());
  global_pub_->on_deactivate();
}

void FSMTController::setSpeedLimit(const double& speed_limit, const bool& percentage)
{
  (void) speed_limit;
  (void) percentage;
}

geometry_msgs::msg::TwistStamped FSMTController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  (void)velocity;
  (void)goal_checker;

  auto transformed_plan = transformGlobalPlan(pose);

  // RCLCPP_INFO(
  //   logger_,
  //   "Received global plan with %d poses", (int) transformed_plan.poses.size());

  // Find the transformed_plan that each horizon is pointing to
  std::vector<point2d_t> vec_goal;
  for (auto &mp_params: fsmt_.params_.vec_mp_params) {
    // Find the longest plan that is within the horizon
    for (size_t i = 0; i < transformed_plan.poses.size(); i++) {
      auto transformed_pose = transformed_plan.poses[i];
      if (fabs(mp_params.forward_velocity * mp_params.time_horizon) < std::pow(transformed_pose.pose.position.x, 2) + std::pow(transformed_pose.pose.position.y, 2)) {
        vec_goal.push_back({transformed_pose.pose.position.x, transformed_pose.pose.position.y});
        break;
      }
    }

    // If the plan is within the horizon, set the goal to the last pose
    vec_goal.push_back({
        transformed_plan.poses.back().pose.position.x,
        transformed_plan.poses.back().pose.position.y
      });
  }

  // RCLCPP_INFO(
  //   logger_,
  //   "Computed goals for each horizon");

  // Compute cost for each maneuver
  int best_i_index=-1, best_j_index;
  double best_cost=10;
  double total_cost = 0;
  int number_of_voters = 0;
  int number_at_lowest_level = 0;

  std::vector<double> best_forward_vel_;
  std::vector<double> best_angular_vel_;
  std::vector<double> best_cost_;

  // reverse direction
  for(size_t i=0; i<fsmt_.motion_tube_.size(); i++)
  {
      // double time_horizon = fsmt_.params_.vec_mp_params[i].time_horizon;
      size_t nb_tubes_ith_horizon = fsmt_.motion_tube_[i].size();
      for(size_t j=0; j<nb_tubes_ith_horizon; j++)
      {
          cost_[i][j] = 100;
          // RCLCPP_INFO(
          //   logger_,
          //   "Initialized cost for horizon %d", (int) i);
          if(fsmt_.availability_[i][j]){
            cost_[i][j] = compute_points_distance(&fsmt_.final_position_[i][j],
                &vec_goal[i]);
            // RCLCPP_INFO(
            //   logger_,
            //   "Horizon is available, computed cost for horizon %d", (int) i);
            number_at_lowest_level++;
          }

          
      }

      // RCLCPP_INFO(
      //   logger_,
      //   "Computed cost for horizon %d", (int) i);

      std::vector<double>::iterator best_cost_ith_horizon = std::min_element(cost_[i].begin(), cost_[i].end());
      int index = std::distance(cost_[i].begin(), best_cost_ith_horizon);
      if(*best_cost_ith_horizon < 0.5  )
      {
          if(*best_cost_ith_horizon < best_cost)
              best_cost = *best_cost_ith_horizon;
          best_i_index = i;
          best_j_index = index;
          unicycle_control_t *local_control = (unicycle_control_t*) 
              fsmt_.motion_primitive_[best_i_index][best_j_index].control; 
          best_forward_vel_.push_back(local_control->forward_velocity);
          best_angular_vel_.push_back(local_control->angular_rate);
          best_cost_.push_back( std::max(0.5-*best_cost_ith_horizon,0.01) );    
          total_cost += best_cost_[number_of_voters];
          // std::cout << "i: " << i << " j: " << index << " v: " << local_control->forward_velocity <<
          //     " w: " << local_control->angular_rate*180/M_PI << " r: " << local_control->forward_velocity/local_control->angular_rate <<
          //     " cost: " << *best_cost_ith_horizon << " retified cost: " << best_cost_[number_of_voters] <<std::endl;
          number_of_voters++;

      }
  }



  // RCLCPP_INFO(
  //   logger_,
  //   "Number of voters: %d, total cost: %f, best cost: %f",
  //   number_of_voters, total_cost, best_cost);


  // // Create and publish a TwistStamped message with the desired velocity
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();

  if (number_of_voters > 0) {
    double total_weight = 0;
    double v_weight_sum = 0;
    double w_weight_sum = 0;

    for (int i = 0; i < number_of_voters; i++) {
      double weight = best_cost_[i] / total_cost;
      total_weight += weight;
      v_weight_sum += weight * best_forward_vel_[i];
      w_weight_sum += weight * best_angular_vel_[i];
    }

    cmd_vel.twist.linear.x = v_weight_sum / total_weight;
    cmd_vel.twist.angular.z = w_weight_sum / total_weight;
  } else {
    // If no maneuver is available, stop the robot
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.angular.z = 0.0;
  }

  return cmd_vel;
}

void FSMTController::setPlan(const nav_msgs::msg::Path & path)
{
  global_pub_->publish(path);
  global_plan_ = path;
}

nav_msgs::msg::Path
FSMTController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  // Original mplementation taken fron nav2_dwb_controller

  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  // let's get the pose of the robot in the frame of the plan
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!transformPose(
      tf_, global_plan_.header.frame_id, pose,
      robot_pose, transform_tolerance_))
  {
    throw nav2_core::PlannerException("Unable to transform robot pose into global plan's frame");
  }

  // We'll discard points on the plan that are outside the local costmap
  nav2_costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  double dist_threshold = std::max(costmap->getSizeInCellsX(), costmap->getSizeInCellsY()) *
    costmap->getResolution() / 2.0;

  // First find the closest pose on the path to the robot
  auto transformation_begin =
    min_by(
    global_plan_.poses.begin(), global_plan_.poses.end(),
    [&robot_pose](const geometry_msgs::msg::PoseStamped & ps) {
      return euclidean_distance(robot_pose, ps);
    });

  // From the closest point, look for the first point that's further then dist_threshold from the
  // robot. These points are definitely outside of the costmap so we won't transform them.
  auto transformation_end = std::find_if(
    transformation_begin, end(global_plan_.poses),
    [&](const auto & global_plan_pose) {
      return euclidean_distance(robot_pose, global_plan_pose) > dist_threshold;
    });

  // Helper function for the transform below. Transforms a PoseStamped from global frame to local
  auto transformGlobalPoseToLocal = [&](const auto & global_plan_pose) {
      // We took a copy of the pose, let's lookup the transform at the current time
      geometry_msgs::msg::PoseStamped stamped_pose, transformed_pose;
      stamped_pose.header.frame_id = global_plan_.header.frame_id;
      stamped_pose.header.stamp = pose.header.stamp;
      stamped_pose.pose = global_plan_pose.pose;
      transformPose(
        tf_, costmap_ros_->getBaseFrameID(),
        stamped_pose, transformed_pose, transform_tolerance_);
      return transformed_pose;
    };

  // Transform the near part of the global plan into the robot's frame of reference.
  nav_msgs::msg::Path transformed_plan;
  std::transform(
    transformation_begin, transformation_end,
    std::back_inserter(transformed_plan.poses),
    transformGlobalPoseToLocal);
  transformed_plan.header.frame_id = costmap_ros_->getBaseFrameID();
  transformed_plan.header.stamp = pose.header.stamp;

  // Remove the portion of the global plan that we've already passed so we don't
  // process it on the next iteration (this is called path pruning)
  global_plan_.poses.erase(begin(global_plan_.poses), transformation_begin);
  global_pub_->publish(transformed_plan);

  if (transformed_plan.poses.empty()) {
    throw nav2_core::PlannerException("Resulting plan has 0 poses in it.");
  }

  return transformed_plan;
}

bool FSMTController::transformPose(
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::string frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose,
  const rclcpp::Duration & transform_tolerance
) const
{
  // Implementation taken as is fron nav_2d_utils in nav2_dwb_controller

  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf->transform(in_pose, out_pose, frame);
    return true;
  } catch (tf2::ExtrapolationException & ex) {
    auto transform = tf->lookupTransform(
      frame,
      in_pose.header.frame_id,
      tf2::TimePointZero
    );
    if (
      (rclcpp::Time(in_pose.header.stamp) - rclcpp::Time(transform.header.stamp)) >
      transform_tolerance)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("tf_help"),
        "Transform data too old when converting from %s to %s",
        in_pose.header.frame_id.c_str(),
        frame.c_str()
      );
      RCLCPP_ERROR(
        rclcpp::get_logger("tf_help"),
        "Data time: %ds %uns, Transform time: %ds %uns",
        in_pose.header.stamp.sec,
        in_pose.header.stamp.nanosec,
        transform.header.stamp.sec,
        transform.header.stamp.nanosec
      );
      return false;
    } else {
      tf2::doTransform(in_pose, out_pose, transform);
      return true;
    }
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("tf_help"),
      "Exception in transformPose: %s",
      ex.what()
    );
    return false;
  }
  return false;
}

void FSMTController::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  range_sensor_t range_sensor;
  //  = {
  //   .angular_resolution = msg->angle_increment,
  //   .nb_measurements = msg->ranges.size(),
  //   .max_angle = msg->angle_max,
  //   .min_angle = msg->angle_min,
  //   .max_distance = msg->range_max,
  //   .min_distance = msg->range_min,
  //   .semantic_id = 0
  // };
  range_sensor.angular_resolution = msg->angle_increment;
  range_sensor.nb_measurements = msg->ranges.size();
  range_sensor.max_angle = msg->angle_max;
  range_sensor.min_angle = msg->angle_min;
  range_sensor.max_distance = msg->range_max;
  range_sensor.min_distance = msg->range_min;

  if (!fsmt_configured_) {
    RCLCPP_INFO(
      logger_,
      "Configuring FSMT with range sensor parameters");
    fsmt_.Configure(&range_sensor);

    cost_.resize(fsmt_.motion_tube_.size());
    for (size_t i = 0; i < fsmt_.motion_tube_.size(); i++) {
      cost_[i].resize(fsmt_.motion_tube_[i].size());
    }

    fsmt_configured_ = true;
  }

  range_scan_t range_scan;
  range_scan.nb_measurements = (int) msg->ranges.size();
  range_scan.measurements = (double*) malloc(range_scan.nb_measurements*sizeof(double));
  range_scan.angles = (double*) malloc(range_scan.nb_measurements*sizeof(double));

  for (int i = 0; i < range_scan.nb_measurements; i++) {
    range_scan.measurements[i] = (double) msg->ranges[i];
    range_scan.angles[i] = msg->angle_min + i * range_sensor.angular_resolution;
  }

  fsmt_.Compute(&range_sensor, &range_scan);
}

}  // namespace fsmt_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(fsmt_controller::FSMTController, nav2_core::Controller)