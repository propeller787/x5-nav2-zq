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

#ifndef NAV2_JOY__QUINTIC_PLANNER_HPP_
#define NAV2_JOY__QUINTIC_PLANNER_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_navfn_planner/navfn_planner.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

namespace nav2_joy
{

/**
 * @class QuinticPlanner
 * @brief A hybrid "fake" global planner. Mode is a runtime parameter:
 *        "quintic" — smooth C2 quintic (5th-order Hermite) curve from
 *                    current pose to goal (ignores costmap).
 *        "navfn"   — delegates to nav2_navfn_planner::NavfnPlanner,
 *                    A* search on the costmap (the "normal" global).
 *
 *        The mode is read on every createPlan() call from the
 *        parameter `<planner_name>.mode` so it can be flipped at
 *        runtime via the standard set_parameters service, e.g.:
 *            ros2 param set /planner_server GridBased.mode navfn
 */
class QuinticPlanner : public nav2_core::GlobalPlanner
{
public:
  QuinticPlanner() = default;
  ~QuinticPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  // C2 Hermite quintic smoothstep: 10τ³ - 15τ⁴ + 6τ⁵
  static double smoothstepQuintic(double tau);

  // Wrap angle to [-π, π]
  static double shortestAngularDistance(double from, double to);

  static geometry_msgs::msg::Quaternion yawToQuaternion(double yaw);

  nav_msgs::msg::Path computeQuinticPath(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string name_;
  std::string global_frame_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};

  // Embedded "normal" planner
  std::unique_ptr<nav2_navfn_planner::NavfnPlanner> navfn_planner_;

  int num_points_{50};
};

}  // namespace nav2_joy

#endif  // NAV2_JOY__QUINTIC_PLANNER_HPP_
