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

#ifndef NAV2_JOY__JOY_TO_TWIST_HPP_
#define NAV2_JOY__JOY_TO_TWIST_HPP_

#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace nav2_joy
{

/**
 * @class JoyToTwist
 * @brief Subscribes sensor_msgs/Joy and publishes geometry_msgs/Twist.
 *
 * The mapping is fully configurable via parameters so that the same node
 * works with both a real joystick and the keyboard emulator in
 * `nav2_joy_teleop`. If `enable_button` is provided, the node will only
 * publish non-zero Twist messages while the corresponding button is held.
 */
class JoyToTwist : public rclcpp::Node
{
public:
  explicit JoyToTwist(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  double scale_axis(double value, double scale) const;

  // ----------------------- parameters -----------------------
  std::string joy_topic_;
  std::string cmd_vel_topic_;
  int axis_linear_x_;
  int axis_linear_y_;
  int axis_linear_z_;
  int axis_angular_x_;
  int axis_angular_y_;
  int axis_angular_z_;
  double scale_linear_x_;
  double scale_linear_y_;
  double scale_linear_z_;
  double scale_angular_x_;
  double scale_angular_y_;
  double scale_angular_z_;
  int enable_button_;
  double deadman_timeout_;
  int turbo_button_;
  double turbo_scale_;

  // ----------------------- state ----------------------------
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::Time last_joy_time_;
  bool last_enable_state_;
};

}  // namespace nav2_joy

#endif  // NAV2_JOY__JOY_TO_TWIST_HPP_
