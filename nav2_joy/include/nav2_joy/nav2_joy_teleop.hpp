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

#ifndef NAV2_JOY__NAV2_JOY_TELEOP_HPP_
#define NAV2_JOY__NAV2_JOY_TELEOP_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace nav2_joy
{

/**
 * @class Nav2JoyTeleop
 * @brief Reads keyboard input in raw terminal mode and publishes a Joy
 *        message that mimics the output of a physical joystick.
 *
 * Key mapping (default):
 *   i / k : linear.x  (forward / backward, full scale)
 *   j / l : angular.z (turn left / right, full scale)
 *   u / o : diagonal motion (forward + turn)
 *   w / x : linear.x  (medium speed)
 *   a / d : angular.z (medium turn)
 *   , / . : linear.x  (slow)
 *   s     : stop (all axes = 0)
 *   space : dead-man / enable button (button 0)
 *   q     : quit the node
 *
 * The node publishes sensor_msgs/Joy on `joy_topic` at `publish_rate` Hz.
 */
class Nav2JoyTeleop : public rclcpp::Node
{
public:
  explicit Nav2JoyTeleop(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~Nav2JoyTeleop() override;

private:
  /// Configure parameters, allocate buffers, put the terminal in raw mode.
  void init();

  /// Main keyboard thread loop: reads chars and updates axes/buttons.
  void keyboard_loop();

  /// Timer callback: builds and publishes the Joy message.
  void publish_joy();

  /// Restore the terminal to its original mode. Safe to call repeatedly.
  void restore_terminal();

  /// Ramp the actual axes towards the target values.
  void apply_axis_targets();

  // ----------------------- parameters -----------------------
  std::string joy_topic_;
  double publish_rate_;
  double axis_ramp_;
  int num_axes_;
  int num_buttons_;
  int enable_button_index_;

  // ----------------------- state ----------------------------
  std::vector<float> axes_;
  std::vector<float> axes_target_;
  std::vector<int32_t> buttons_;
  std::atomic<bool> running_{true};
  std::atomic<bool> dirty_{true};

  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::thread keyboard_thread_;

  // raw terminal state
  struct TermiosGuard;  // PIMPL to avoid leaking <termios.h> in the header
  std::unique_ptr<TermiosGuard> term_guard_;
};

}  // namespace nav2_joy

#endif  // NAV2_JOY__NAV2_JOY_TELEOP_HPP_
