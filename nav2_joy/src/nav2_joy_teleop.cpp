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

#include "nav2_joy/nav2_joy_teleop.hpp"

#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <map>
#include <string>

#include "rclcpp/logging.hpp"

namespace nav2_joy
{

// ---------------------------------------------------------------------------
// TermiosGuard: RAII wrapper for putting the terminal in raw, non-blocking mode
// ---------------------------------------------------------------------------
struct Nav2JoyTeleop::TermiosGuard
{
  struct termios original;
  bool active{false};

  ~TermiosGuard()
  {
    if (active) {
      tcsetattr(STDIN_FILENO, TCSANOW, &original);
    }
  }
};

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------
Nav2JoyTeleop::Nav2JoyTeleop(const rclcpp::NodeOptions & options)
: rclcpp::Node("nav2_joy_teleop", options)
{
  joy_topic_ = this->declare_parameter<std::string>("joy_topic", "/joy");
  publish_rate_ = this->declare_parameter<double>("publish_rate", 50.0);
  axis_ramp_ = this->declare_parameter<double>("axis_ramp", 4.0);
  num_axes_ = this->declare_parameter<int>("num_axes", 6);
  num_buttons_ = this->declare_parameter<int>("num_buttons", 4);
  enable_button_index_ = this->declare_parameter<int>("enable_button_index", 0);

  // Default key mapping table. Mirrors the canonical `teleop_twist_keyboard`
  // layout (i/j/k/l/u/o/,/. + w/x/a/d + space) so muscle memory transfers.
  // The table is informational; the actual handler logic is in keyboard_loop.
  std::vector<std::string> default_map = {
    "i:linear_x:+1.0", "k:linear_x:-1.0", "j:angular_z:+1.0", "l:angular_z:-1.0",
    "u:linear_x:+0.5", "u:angular_z:+0.5", "o:linear_x:+0.5", "o:angular_z:-0.5",
    "w:linear_x:+0.75", "x:linear_x:-0.75", "a:angular_z:+0.75", "d:angular_z:-0.75",
    ",:linear_x:+0.25", ".:linear_x:-0.25", "s:stop", "q:quit", " :enable",
  };
  (void)this->declare_parameter<std::vector<std::string>>("key_map", default_map);

  axes_.assign(num_axes_, 0.0f);
  axes_target_.assign(num_axes_, 0.0f);
  buttons_.assign(num_buttons_, 0);

  joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>(joy_topic_, 10);

  init();
}

void Nav2JoyTeleop::init()
{
  // Bail out gracefully if stdin is not a terminal. This can happen
  // when the node is run under systemd, in a docker container without
  // -it, or in any other context where there is no TTY. We still
  // start the node (it will just publish zero Joy) so the rest of
  // the stack keeps working, but the keyboard reader becomes a no-op.
  const bool stdin_is_tty = ::isatty(STDIN_FILENO) != 0;

  term_guard_ = std::make_unique<TermiosGuard>();
  if (stdin_is_tty &&
      ::tcgetattr(STDIN_FILENO, &term_guard_->original) == 0)
  {
    struct termios raw = term_guard_->original;
    raw.c_lflag &= ~(ICANON | ECHO);  // raw, no echo
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    ::tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    term_guard_->active = true;
  } else {
    RCLCPP_WARN(get_logger(),
      "stdin is not a TTY (errno=%d); keyboard input will be ignored, "
      "Joy axes will stay at zero. Run the node from a terminal or set "
      "terminal:=none in the launch file if this is intentional.",
      errno);
  }

  // -----------------------------------------------------------------
  // Print help / current mapping
  // -----------------------------------------------------------------
  std::cout <<
    "\n=========================================\n"
    "  nav2_joy_teleop - keyboard -> /joy\n"
    "=========================================\n"
    "  i / k : move forward / backward (full)\n"
    "  j / l : turn left / right        (full)\n"
    "  u / o : forward + turn diagonally\n"
    "  w / x : forward / backward      (medium)\n"
    "  a / d : turn left / right       (medium)\n"
    "  , / . : forward / backward      (slow)\n"
    "  s     : stop\n"
    "  space : dead-man / enable button\n"
    "  q     : quit the node\n"
    "-----------------------------------------\n"
    "  Publishing sensor_msgs/Joy on '" << joy_topic_ << "'\n"
    "=========================================\n"
    "Hold 'space' (or whichever enable key) while moving for the velocity\n"
    "to be forwarded. Press 'q' to quit cleanly.\n"
    << std::endl;

  publish_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_)),
    std::bind(&Nav2JoyTeleop::publish_joy, this));

  keyboard_thread_ = std::thread(&Nav2JoyTeleop::keyboard_loop, this);
}

Nav2JoyTeleop::~Nav2JoyTeleop()
{
  running_.store(false);
  if (keyboard_thread_.joinable()) {
    keyboard_thread_.join();
  }
  restore_terminal();
}

void Nav2JoyTeleop::restore_terminal()
{
  if (term_guard_ && term_guard_->active) {
    tcsetattr(STDIN_FILENO, TCSANOW, &term_guard_->original);
    term_guard_->active = false;
    std::cout << "\n[nav2_joy_teleop] terminal restored." << std::endl;
  }
}

// ---------------------------------------------------------------------------
// Keyboard thread
// ---------------------------------------------------------------------------
// Axis layout used by the downstream joy_to_twist node (default):
//   axes[0] = linear.x   (forward/back)
//   axes[1] = linear.y   (strafe; usually 0 for diff drive)
//   axes[2] = linear.z
//   axes[3] = angular.x
//   axes[4] = angular.y
//   axes[5] = angular.z  (yaw)
//   buttons[0] = enable / dead-man
//
// We keep a small set of "held" keys; the union of their contributions
// forms the current axes_target_. After `hold_timeout_ms` of silence we
// assume the user has released all keys and clear the held set, so the
// robot stops when nothing is being pressed. The timeout is long enough
// to span the OS-level keyboard autorepeat delay (see below), so
// pressing-and-holding a key produces continuous motion.
void Nav2JoyTeleop::keyboard_loop()
{
  std::map<char, std::pair<float, float>> held;  // key -> (linear, angular)
  auto apply_key = [&](char c) {
    switch (c) {
      // full speed
      case 'i': held[c] = {+1.0f,  0.0f}; break;
      case 'k': held[c] = {-1.0f,  0.0f}; break;
      case 'j': held[c] = { 0.0f, +1.0f}; break;
      case 'l': held[c] = { 0.0f, -1.0f}; break;
      // diagonals
      case 'u': held[c] = {+0.5f, +0.5f}; break;
      case 'o': held[c] = {+0.5f, -0.5f}; break;
      // medium
      case 'w': held[c] = {+0.75f,  0.0f}; break;
      case 'x': held[c] = {-0.75f,  0.0f}; break;
      case 'a': held[c] = { 0.0f, +0.75f}; break;
      case 'd': held[c] = { 0.0f, -0.75f}; break;
      // slow
      case ',': held[c] = {+0.25f,  0.0f}; break;
      case '.': held[c] = {-0.25f,  0.0f}; break;
      // stop
      case 's': held.clear(); break;
      // dead-man
      case ' ': buttons_[enable_button_index_] = 1; break;
      // quit
      case 'q': running_.store(false); break;
      // explicitly-released by re-pressing the same key with shift would be
      // ambiguous in raw mode, so we use timeout-based release instead.
      default: return;
    }
    dirty_.store(true);
  };

  auto recompute_targets = [&]() {
    float lin = 0.0f, ang = 0.0f;
    for (const auto & kv : held) {
      lin += kv.second.first;
      ang += kv.second.second;
    }
    // clamp to unit circle
    float mag = std::sqrt(lin * lin + ang * ang);
    if (mag > 1.0f) {
      lin /= mag;
      ang /= mag;
    }
    axes_target_[0] = lin;
    axes_target_[5] = ang;
  };

  // The "key hold" timeout: if no new key event arrives within this
  // many ms, we assume the user has released all keys. The value is
  // chosen to be longer than the typical OS keyboard autorepeat
  // delay (~500 ms) so that pressing and holding a key produces
  // continuous motion rather than the pulsating motion you would
  // get with a shorter timeout. After the autorepeat kicks in, the
  // steady-state interval is ~30 ms, so this timeout is safely
  // larger than that.
  const int hold_timeout_ms = 800;
  auto last_event = std::chrono::steady_clock::now();

  while (running_.load()) {
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);
    struct timeval tv{0, 20'000};  // 20 ms polling tick
    int rv = select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &tv);
    if (rv > 0 && FD_ISSET(STDIN_FILENO, &readfds)) {
      char c = 0;
      ssize_t n = ::read(STDIN_FILENO, &c, 1);
      if (n == 1) {
        if (c == 'q') {
          running_.store(false);
          break;
        }
        apply_key(c);
        last_event = std::chrono::steady_clock::now();
      }
    }
    // Auto-release all held keys after `hold_timeout_ms` of silence.
    auto now = std::chrono::steady_clock::now();
    if (!held.empty() || buttons_[enable_button_index_]) {
      auto since = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_event).count();
      if (since > hold_timeout_ms) {
        if (!held.empty()) {
          held.clear();
          dirty_.store(true);
        }
        if (buttons_[enable_button_index_]) {
          buttons_[enable_button_index_] = 0;
          dirty_.store(true);
        }
      }
    }
    recompute_targets();
  }
  // When the user types 'q', restore the terminal so the shell prompt is sane.
  restore_terminal();
}

void Nav2JoyTeleop::apply_axis_targets()
{
  const float ramp = static_cast<float>(axis_ramp_ * (1.0 / publish_rate_));
  for (size_t i = 0; i < axes_.size(); ++i) {
    if (std::abs(axes_[i] - axes_target_[i]) < ramp) {
      axes_[i] = axes_target_[i];
    } else if (axes_[i] < axes_target_[i]) {
      axes_[i] += ramp;
    } else {
      axes_[i] -= ramp;
    }
  }
}

void Nav2JoyTeleop::publish_joy()
{
  apply_axis_targets();

  // Only publish if anything actually changed since the last tick — saves
  // bandwidth and lets the downstream joy_to_twist hit its watchdog faster.
  if (!dirty_.load() &&
      std::all_of(axes_.begin(), axes_.end(),
                  [](float v) { return v == 0.0f; }) &&
      std::all_of(buttons_.begin(), buttons_.end(),
                  [](int32_t v) { return v == 0; }))
  {
    return;
  }

  auto msg = sensor_msgs::msg::Joy();
  msg.header.stamp = this->now();
  msg.header.frame_id = "joy";
  msg.axes.assign(axes_.begin(), axes_.end());
  msg.buttons.assign(buttons_.begin(), buttons_.end());
  joy_pub_->publish(msg);
  dirty_.store(false);
}

}  // namespace nav2_joy
