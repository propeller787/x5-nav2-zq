#!/bin/bash
# =============================================================================
# 容器入口: 启动 Xvfb + VNC + noVNC, 然后执行传入命令
# =============================================================================
set -e

# ROS 环境
source /opt/ros/humble/setup.bash
[ -f ${WORKSPACE}/install/setup.bash ] && source ${WORKSPACE}/install/setup.bash
export TURTLEBOT3_MODEL=waffle

# -----------------------------------------------------------------------------
# 远程 GUI: noVNC (浏览器) + VNC (客户端) + Xvfb (虚拟显示)
# 浏览器访问 http://localhost:6080/vnc.html 即可看到桌面
# -----------------------------------------------------------------------------
if [ -z "$(pgrep Xvfb)" ]; then
    Xvfb :1 -screen 0 1280x720x24 -ac &
    sleep 1
fi
if [ -z "$(pgrep x11vnc)" ]; then
    x11vnc -display :1 -forever -nopw -rfbport 5900 -quiet &
fi
if [ -z "$(pgrep websockify)" ]; then
    websockify --web=/usr/share/novnc 6080 localhost:5900 &
fi
export DISPLAY=:1

# -----------------------------------------------------------------------------
# 执行入口命令
# -----------------------------------------------------------------------------
echo "============================================================"
echo "Nav2 MPPI 开发容器已就绪"
echo "  - 浏览器看 GUI:  http://localhost:6080/vnc.html"
echo "  - VNC 客户端:    localhost:5900 (无密码)"
echo "  - ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"
echo "============================================================"
echo "常用命令:"
echo "  ros2 launch nav2_mppi_controller mppi_house_amcl_demo.launch.py"
echo "  ros2 launch nav2_mppi_controller mppi_house_slam_demo.launch.py"
echo "  ros2 launch nav2_mppi_controller mppi_demo.launch.py"
echo "============================================================"

if [ "$1" = "bash" ]; then
    exec bash
else
    exec "$@"
fi
