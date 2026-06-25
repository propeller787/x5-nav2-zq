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

#include "nav2_joy/joy_to_twist.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "rclcpp/logging.hpp"

namespace nav2_joy
{

JoyToTwist::JoyToTwist(const rclcpp::NodeOptions & options)
: rclcpp::Node("joy_to_twist", options),
  last_enable_state_(false)
{
  // Topics
  joy_topic_ = this->declare_parameter<std::string>("joy_topic", "/joy");
  cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

  // Axis indices
  axis_linear_x_ = this->declare_parameter<int>("axis_linear_x", 0);
  axis_linear_y_ = this->declare_parameter<int>("axis_linear_y", 1);
  axis_linear_z_ = this->declare_parameter<int>("axis_linear_z", 2);
  axis_angular_x_ = this->declare_parameter<int>("axis_angular_x", 3);
  axis_angular_y_ = this->declare_parameter<int>("axis_angular_y", 4);
  axis_angular_z_ = this->declare_parameter<int>("axis_angular_z", 5);

  // Scaling
  scale_linear_x_ = this->declare_parameter<double>("scale_linear_x", 0.5);
  scale_linear_y_ = this->declare_parameter<double>("scale_linear_y", 0.0);
  scale_linear_z_ = this->declare_parameter<double>("scale_linear_z", 0.0);
  scale_angular_x_ = this->declare_parameter<double>("scale_angular_x", 0.0);
  scale_angular_y_ = this->declare_parameter<double>("scale_angular_y", 0.0);
  scale_angular_z_ = this->declare_parameter<double>("scale_angular_z", 1.0);

  // Buttons
  enable_button_ = this->declare_parameter<int>("enable_button", 0);
  deadman_timeout_ = this->declare_parameter<double>("deadman_timeout", 0.5);
  turbo_button_ = this->declare_parameter<int>("turbo_button", -1);
  turbo_scale_ = this->declare_parameter<double>("turbo_scale", 1.5);

  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    joy_topic_, rclcpp::QoS(10),
    std::bind(&JoyToTwist::joy_callback, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

  last_joy_time_ = this->now() - rclcpp::Duration::from_seconds(deadman_timeout_ * 2);

  // Watchdog: if we don't hear from the joy node for `deadman_timeout`,
  // publish a zero twist so the robot stops. This is a safety net in case
  // the keyboard node is killed (e.g. SIGKILLed) without releasing the
  // last-known enable button.
  watchdog_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(deadman_timeout_ * 1000.0)),
    [this]() {
      auto now = this->now();
      if ((now - last_joy_time_).seconds() > deadman_timeout_) {
        if (last_enable_state_) {
          auto twist = geometry_msgs::msg::Twist();
          cmd_vel_pub_->publish(twist);
          last_enable_state_ = false;
        }
      }
    });

  RCLCPP_INFO(get_logger(),
    "joy_to_twist: '%s' -> '%s', enable_button=%d, deadman_timeout=%.2fs",
    joy_topic_.c_str(), cmd_vel_topic_.c_str(),
    enable_button_, deadman_timeout_);
}

double JoyToTwist::scale_axis(double value, double scale) const
{
  // Apply a small deadband (0.05) to avoid creep when the stick is "at rest".
  const double deadband = 0.05;
  if (std::abs(value) < deadband) {
    return 0.0;
  }
  // Re-scale: [-1, -deadband] -> [-1, 0] and [deadband, 1] -> [0, 1].
  double sign = (value > 0.0) ? 1.0 : -1.0;
  double norm = (std::abs(value) - deadband) / (1.0 - deadband);
  return sign * norm * scale;
}

void JoyToTwist::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  last_joy_time_ = this->now();

  // Safety check: enable button must be held. The keyboard emulator
  // (and most real joysticks) expose this on buttons[enable_button_].
  if (enable_button_ < 0 ||
      (enable_button_ < static_cast<int>(msg->buttons.size()) &&
       msg->buttons[enable_button_]))
  {
    last_enable_state_ = true;

    // Optionally scale up when a "turbo" button is held.
    double turbo = 1.0;
    if (turbo_button_ >= 0 &&
        turbo_button_ < static_cast<int>(msg->buttons.size()) &&
        msg->buttons[turbo_button_])
    {
      turbo = turbo_scale_;
    }

    auto get_axis = [&](int idx) -> double {
      if (idx < 0 || idx >= static_cast<int>(msg->axes.size())) {
        return 0.0;
      }
      return static_cast<double>(msg->axes[idx]);
    };

    auto twist = geometry_msgs::msg::Twist();
    twist.linear.x = scale_axis(get_axis(axis_linear_x_), scale_linear_x_ * turbo);
    twist.linear.y = scale_axis(get_axis(axis_linear_y_), scale_linear_y_ * turbo);
    twist.linear.z = scale_axis(get_axis(axis_linear_z_), scale_linear_z_ * turbo);
    twist.angular.x = scale_axis(get_axis(axis_angular_x_), scale_angular_x_ * turbo);
    twist.angular.y = scale_axis(get_axis(axis_angular_y_), scale_angular_y_ * turbo);
    twist.angular.z = scale_axis(get_axis(axis_angular_z_), scale_angular_z_ * turbo);
    cmd_vel_pub_->publish(twist);
  } else if (last_enable_state_) {
    // Enable button just released — publish a single zero twist to ensure
    // the robot stops promptly.
    auto twist = geometry_msgs::msg::Twist();
    cmd_vel_pub_->publish(twist);
    last_enable_state_ = false;
  }
}

}  // namespace nav2_joy
