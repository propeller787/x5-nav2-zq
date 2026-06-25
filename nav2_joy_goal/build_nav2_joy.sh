#!/usr/bin/env bash
# build_nav2_joy.sh — end-to-end build + sanity check for nav2_joy.
#
# This script is meant to be run from the workspace root:
#   /home/howard/code/navigation2
#
# It does four things, stopping at the first failure:
#   1. colcon build only the nav2_joy package (with all warnings on)
#   2. ros2 pkg executables nav2_joy         (verifies the install)
#   3. syntax check the launch files         (python3 -m py_compile)
#   4. (optional) headless smoke test of joy_to_twist
#
# Usage:
#   cd /home/howard/code/navigation2
#   ./nav2_joy/build_nav2_joy.sh                # build only
#   ./nav2_joy/build_nav2_joy.sh --test         # build + headless smoke test
#   ./nav2_joy/build_nav2_joy.sh --clean        # wipe build/ install/ first
#
# The launch file is run separately:
#   ros2 launch nav2_joy nav2_joy_simulation_launch.py terminal:=xterm

# NOTE: deliberately do NOT use `set -u` (nounset). The ROS 2 setup.bash
# reads AMENT_TRACE_SETUP_FILES internally; with `-u` it aborts when that
# var is unset, which kills our build before colcon even starts.
set -eo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PKG_DIR="${WS_DIR}/nav2_joy"
cd "${WS_DIR}"

# ---------------------------------------------------------------------------
# 0. Find the active ROS 2 install
# ---------------------------------------------------------------------------
if [ -f /opt/ros/humble/setup.bash ]; then
  ROS_SETUP=/opt/ros/humble/setup.bash
elif [ -f /opt/ros/jazzy/setup.bash ]; then
  ROS_SETUP=/opt/ros/jazzy/setup.bash
else
  echo "ERROR: no /opt/ros/<distro>/setup.bash found." >&2
  exit 1
fi
# shellcheck disable=SC1090
source "${ROS_SETUP}"

# ---------------------------------------------------------------------------
# Pick a working `colcon` executable. On this machine ~/.local/bin/colcon
# is a pip entry point that imports colcon_core from the wrong Python,
# so `command -v colcon` will pick it and immediately fail. We probe a few
# well-known locations in order of preference and keep the first one whose
# `--help` actually works.
# ---------------------------------------------------------------------------
COLCON_BIN=""
for candidate in \
  /usr/bin/colcon \
  /usr/local/bin/colcon \
  /opt/ros/*/bin/colcon \
  "$(python3 -m site --user-base 2>/dev/null)/bin/colcon"
do
  # Skip globs that didn't expand
  [ -e "$candidate" ] || continue
  # If --help works, this is a real colcon
  if "$candidate" --help >/dev/null 2>&1; then
    COLCON_BIN="$candidate"
    break
  fi
done
# Fallback: whatever is on PATH, even if broken (so the error from colcon
# itself is surfaced).
if [ -z "${COLCON_BIN}" ]; then
  COLCON_BIN="$(command -v colcon || true)"
fi
if [ -z "${COLCON_BIN}" ]; then
  echo "ERROR: no working 'colcon' binary found." >&2
  echo "       Install with:  sudo apt install python3-colcon-common-extensions" >&2
  exit 1
fi
echo "[build] using colcon: ${COLCON_BIN}"
# Put the directory containing the working colcon first on PATH so that
# colcon's plugin entry points (also installed there) are found.
export PATH="$(dirname "${COLCON_BIN}"):${PATH}"

# ---------------------------------------------------------------------------
# If the colcon we picked is actually broken (entry point exists but the
# `colcon_core` import fails because the user only has the wrapper installed
# via pip without the library), try to install the library into the same
# user site-packages and re-probe.
# ---------------------------------------------------------------------------
if ! "${COLCON_BIN}" --help >/dev/null 2>&1; then
  echo "[build] ${COLCON_BIN} is missing colcon_core, attempting to fix..."
  if ! python3 -c "import colcon_core" 2>/dev/null; then
    python3 -m pip install --user --quiet \
        colcon-core \
        colcon-common-extensions \
        colcon-ros \
        colcon-cmake \
        colcon-package-selection \
        colcon-recursive-crawl 2>&1 | sed 's/^/[build] pip: /' || true
  fi
  # Re-probe
  COLCON_BIN=""
  for candidate in \
    /usr/bin/colcon \
    /usr/local/bin/colcon \
    /opt/ros/*/bin/colcon \
    "$(python3 -m site --user-base 2>/dev/null)/bin/colcon"
  do
    [ -e "$candidate" ] || continue
    if "$candidate" --help >/dev/null 2>&1; then
      COLCON_BIN="$candidate"
      break
    fi
  done
  if [ -z "${COLCON_BIN}" ] || ! "${COLCON_BIN}" --help >/dev/null 2>&1; then
    echo "ERROR: could not get a working colcon." >&2
    echo "       Try one of:" >&2
    echo "         pip3 install --user colcon-core colcon-common-extensions colcon-ros" >&2
    echo "         sudo apt install python3-colcon-common-extensions python3-colcon-ros" >&2
    exit 1
  fi
  echo "[build] repaired; using colcon: ${COLCON_BIN}"
  export PATH="$(dirname "${COLCON_BIN}"):${PATH}"
fi

# ---------------------------------------------------------------------------
# 1. (optional) clean
# ---------------------------------------------------------------------------
if [ "${1:-}" = "--clean" ]; then
  echo "[build] removing build/ install/ log/ entirely..."
  rm -rf build install log
fi

# ---------------------------------------------------------------------------
# 2. Build
# ---------------------------------------------------------------------------
echo "==============================================="
echo " [1/4] colcon build nav2_joy + runtime deps"
echo "==============================================="
# Build the whole workspace. We tried targeted --packages-select /
# --packages-up-to sets, but the navigation2 workspace has cross-tree
# runtime references (e.g. nav2_bringup's launch files include
# nav2_loopback_sim, but that's a runtime edge, not a build-time
# dep, so --packages-up-to nav2_bringup leaves nav2_loopback_sim
# un-built). Building the whole workspace once is the supported
# workflow and let colcon handle the dependency graph in topological
# order. First run takes 5-15 min; subsequent runs are incremental.
#
# nav2_loopback_sim's CMakeLists.txt find_package(backward_ros REQUIRED).
# On most ros-humble apt installs that is shipped as
# `ros-humble-backward-ros`; we try apt-get once, and if that fails
# the user is told the exact command to run by hand.
if ! dpkg -s ros-${ROS_DISTRO}-backward-ros >/dev/null 2>&1; then
  echo "[build] ros-${ROS_DISTRO}-backward-ros not installed; attempting apt-get..."
  if command -v sudo >/dev/null 2>&1; then
    sudo apt-get update -qq && sudo apt-get install -y -qq \
        "ros-${ROS_DISTRO}-backward-ros" || true
  else
    apt-get update -qq && apt-get install -y -qq \
        "ros-${ROS_DISTRO}-backward-ros" || true
  fi
fi

# nav2_msgs depends on geographic_msgs (third-party ROS message package)
if ! dpkg -s ros-${ROS_DISTRO}-geographic-msgs >/dev/null 2>&1; then
  echo "[build] ros-${ROS_DISTRO}-geographic-msgs not installed; attempting apt-get..."
  if command -v sudo >/dev/null 2>&1; then
    sudo apt-get install -y -qq "ros-${ROS_DISTRO}-geographic-msgs" || true
  else
    apt-get install -y -qq "ros-${ROS_DISTRO}-geographic-msgs" || true
  fi
fi

# nav2_loopback_sim might not be in apt for humble; try to install it
if ! dpkg -s ros-${ROS_DISTRO}-nav2-loopback-sim >/dev/null 2>&1; then
  echo "[build] ros-${ROS_DISTRO}-nav2-loopback-sim not installed; attempting apt-get..."
  if command -v sudo >/dev/null 2>&1; then
    sudo apt-get install -y -qq "ros-${ROS_DISTRO}-nav2-loopback-sim" || true
  else
    apt-get install -y -qq "ros-${ROS_DISTRO}-nav2-loopback-sim" || true
  fi
fi

"${COLCON_BIN}" build \
  --packages-select nav2_joy \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=OFF \
  --event-handlers console_direct+ 2>&1 | tee /tmp/nav2_joy_build.log

# Pick up the install
# shellcheck disable=SC1091
source install/setup.bash

# ---------------------------------------------------------------------------
# 3. Verify
# ---------------------------------------------------------------------------
echo
echo "==============================================="
echo " [2/4] ros2 pkg executables nav2_joy"
echo "==============================================="
ros2 pkg executables nav2_joy

# ---------------------------------------------------------------------------
# 4. Python launch file syntax check
# ---------------------------------------------------------------------------
echo
echo "==============================================="
echo " [3/4] syntax check on launch/*.py"
echo "==============================================="
python3 -m py_compile "${PKG_DIR}/launch/nav2_joy_teleop_launch.py"
python3 -m py_compile "${PKG_DIR}/launch/nav2_joy_simulation_launch.py"
echo "OK"

# ---------------------------------------------------------------------------
# 5. (optional) headless smoke test
# ---------------------------------------------------------------------------
if [ "${1:-}" = "--test" ]; then
  echo
  echo "==============================================="
  echo " [4/4] headless smoke test of joy_to_twist"
  echo "==============================================="
  "${PKG_DIR}/test_joy_to_twist_headless.sh"
fi

echo
echo "==============================================="
echo " build done. To run the simulation:"
echo "   ros2 launch nav2_joy nav2_joy_simulation_launch.py terminal:=xterm"
echo "==============================================="
