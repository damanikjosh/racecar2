/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Shivang Patel
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Shivang Patel
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_nav2planner_plugin.html
 *********************************************************************/
#include <fstream>  // Add this line to include the fstream header
#include <sstream>

#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"

#include "racecar_planner/track_planner.hpp"

namespace racecar_planner
{

void TrackPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  // costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(
      0.1));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".waypoints_file_path", rclcpp::ParameterValue(
      std::string("")));
  node_->get_parameter(name_ + ".waypoints_file_path", waypoints_file_path_);

  if (waypoints_file_path_.empty()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Waypoints file path is empty, please provide the waypoints file path");
  }

  // Read the waypoints from the csv file
  std::ifstream waypoints_file(waypoints_file_path_);
  if (!waypoints_file.is_open()) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Unable to open the waypoints file, please provide the correct file path");
  }

  // Skip the first line as it contains the header
  std::string line;
  std::getline(waypoints_file, line);

  // Save the remaining lines as waypoints vector
  while (std::getline(waypoints_file, line)) {
    std::istringstream iss(line);
    std::string x, y;
    std::getline(iss, x, ',');
    std::getline(iss, y, ',');
    geometry_msgs::msg::Point point;
    point.x = std::atof(x.c_str());
    point.y = std::atof(y.c_str());
    waypoints_.push_back(point);
  }

  waypoints_file.close();
}

void TrackPlanner::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
    name_.c_str());
}

void TrackPlanner::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
    name_.c_str());
}

void TrackPlanner::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
}

int TrackPlanner::getClosestWaypointIndex(
  const geometry_msgs::msg::PoseStamped & pose)
{
  double min_distance = std::numeric_limits<double>::max();
  int closest_index = -1;
  for (int i = 0; i < waypoints_.size(); ++i) {
    const auto & waypoint = waypoints_[i];
    double distance = std::hypot(waypoint.x - pose.pose.position.x, waypoint.y - pose.pose.position.y);
    if (distance < min_distance) {
      min_distance = distance;
      closest_index = i;
    }
  }
  return closest_index;
}

nav_msgs::msg::Path TrackPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;

  // Checking if the goal and start state is in the global frame
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only except start position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_INFO(
      node_->get_logger(), "Planner will only except goal position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;

  // Get the index of closest waypoint to the start pose
  int closest_start_index = getClosestWaypointIndex(start);

  // Push the waypoints from the closest start index to 10 waypoints ahead, return to the first waypoint if end is reached
  for (int i = closest_start_index; i < closest_start_index + 200; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = waypoints_[i % waypoints_.size()].x;
    pose.pose.position.y = waypoints_[i % waypoints_.size()].y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    pose.header.stamp = node_->now();
    pose.header.frame_id = global_frame_;
    global_path.poses.push_back(pose);
  }

  return global_path;
}

}  // namespace racecar_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(racecar_planner::TrackPlanner, nav2_core::GlobalPlanner)
