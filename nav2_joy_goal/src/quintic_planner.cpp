// Copyright (c) 2026, nav2_joy authors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_joy/quintic_planner.hpp"

#include <cmath>
#include <utility>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav2_joy
{

void QuinticPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  num_points_ = node_->declare_parameter<int>(name_ + ".num_points", 50);
  node_->declare_parameter<std::string>(name_ + ".mode", "quintic");

  // Embedded "normal" planner. Use a sub-namespace so its parameters
  // (tolerance / use_astar / allow_unknown) don't collide with ours.
  navfn_planner_ = std::make_unique<nav2_navfn_planner::NavfnPlanner>();
  navfn_planner_->configure(parent, name_ + ".navfn", tf_, costmap_ros);

  RCLCPP_INFO(
    node_->get_logger(),
    "QuinticPlanner configured: num_points=%d, frame=%s, mode=quintic|navfn",
    num_points_, global_frame_.c_str());
}

void QuinticPlanner::cleanup()
{
  if (navfn_planner_) {
    navfn_planner_->cleanup();
    navfn_planner_.reset();
  }
  RCLCPP_INFO(node_->get_logger(), "Cleaning up QuinticPlanner");
}

void QuinticPlanner::activate()
{
  if (navfn_planner_) {navfn_planner_->activate();}
  RCLCPP_INFO(node_->get_logger(), "Activating QuinticPlanner");
}

void QuinticPlanner::deactivate()
{
  if (navfn_planner_) {navfn_planner_->deactivate();}
  RCLCPP_INFO(node_->get_logger(), "Deactivating QuinticPlanner");
}

double QuinticPlanner::smoothstepQuintic(double tau)
{
  // Hermite quintic: 10τ³ - 15τ⁴ + 6τ⁵
  const double t2 = tau * tau;
  const double t3 = t2 * tau;
  const double t4 = t3 * tau;
  const double t5 = t4 * tau;
  return 10.0 * t3 - 15.0 * t4 + 6.0 * t5;
}

double QuinticPlanner::shortestAngularDistance(double from, double to)
{
  double diff = to - from;
  while (diff > M_PI) diff -= 2.0 * M_PI;
  while (diff < -M_PI) diff += 2.0 * M_PI;
  return diff;
}

geometry_msgs::msg::Quaternion QuinticPlanner::yawToQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

nav_msgs::msg::Path QuinticPlanner::computeQuinticPath(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = node_->now();
  path.poses.reserve(num_points_ + 1);

  const double x_s = start.pose.position.x;
  const double y_s = start.pose.position.y;
  const double yaw_s = tf2::getYaw(start.pose.orientation);

  const double x_g = goal.pose.position.x;
  const double y_g = goal.pose.position.y;
  const double yaw_g = tf2::getYaw(goal.pose.orientation);

  const double d_yaw = shortestAngularDistance(yaw_s, yaw_g);

  for (int i = 0; i <= num_points_; ++i) {
    const double tau = static_cast<double>(i) / static_cast<double>(num_points_);
    const double s = smoothstepQuintic(tau);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = global_frame_;
    pose.header.stamp = path.header.stamp;

    pose.pose.position.x = x_s + s * (x_g - x_s);
    pose.pose.position.y = y_s + s * (y_g - y_s);
    pose.pose.position.z = 0.0;

    pose.pose.orientation = yawToQuaternion(yaw_s + s * d_yaw);
    path.poses.push_back(pose);
  }
  return path;
}

nav_msgs::msg::Path QuinticPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  const std::string mode =
    node_->get_parameter_or(name_ + ".mode", std::string("quintic"));

  if (mode == "navfn" && navfn_planner_) {
    RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "QuinticPlanner: mode=navfn -> delegating to NavfnPlanner");
    return navfn_planner_->createPlan(start, goal);
  }

  return computeQuinticPath(start, goal);
}

}  // namespace nav2_joy

PLUGINLIB_EXPORT_CLASS(nav2_joy::QuinticPlanner, nav2_core::GlobalPlanner);
