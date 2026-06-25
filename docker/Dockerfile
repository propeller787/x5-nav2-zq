# =============================================================================
# Nav2 + MPPI 仿真开发镜像
# 基于 osrf/ros:humble-desktop, 集成:
#   - TurtleBot3 (house + world) Gazebo 模型
#   - slam_toolbox (online_async)
#   - Nav2 完整栈 (controller, planner, bt_navigator, amcl, map_server, ...)
#   - 本仓库 nav2_mppi_controller 源码 (colcon 编译)
#   - noVNC + x11vnc + Xvfb: 浏览器内可看 Gazebo/RViz GUI
# =============================================================================

# 允许 build 时覆盖 base image, 用于国内网络:
#   docker build --build-arg BASE_IMAGE=docker.m.daocloud.io/library/osrf/ros:humble-desktop ...
# 注意: ARG 必须在 FROM 之前 (legacy builder 不支持 stage 间 ARG 共享)
ARG BASE_IMAGE=osrf/ros:humble-desktop
FROM ${BASE_IMAGE}

ARG WORKSPACE=/root/ws
ENV WORKSPACE=${WORKSPACE}
ENV TURTLEBOT3_MODEL=waffle
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DOMAIN_ID=0

# -----------------------------------------------------------------------------
# 1) 系统 + apt ROS 包
# -----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    # 仿真
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3-cartographer \
    # 编译依赖 (nav2 源码 cmake 需要)
    ros-humble-backward-ros \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-gmock \
    ros-humble-ament-lint \
    ros-humble-point-cloud-transport \
    ros-humble-pluginlib \
    ros-humble-cv-bridge \
    # 导航栈运行时 (apt 提供); nav2_ros_common/nav2_mppi_controller 等
    # 不在 apt, 由 colcon 从 src 编译
    ros-humble-geographic-msgs \
    ros-humble-unique-identifier-msgs \
    ros-humble-visualization-msgs \
    ros-humble-action-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    ros-humble-builtin-interfaces \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-eigen \
    ros-humble-tf2-sensor-msgs \
    ros-humble-slam-toolbox \
    ros-humble-nav2-bringup \
    ros-humble-nav2-common \
    ros-humble-nav2-msgs \
    ros-humble-nav2-util \
    ros-humble-nav2-costmap-2d \
    ros-humble-nav2-core \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-rviz-plugins \
    ros-humble-teleop-twist-keyboard \
    # 编译
    python3-colcon-common-extensions \
    python3-rosdep \
    # 测试依赖 (nav2_ros_common 等包需要 test_msgs)
    ros-humble-test-msgs \
    ros-humble-std-srvs \
    # 远程 GUI: Xvfb + VNC + noVNC (浏览器访问 localhost:6080)
    xvfb \
    x11vnc \
    novnc \
    python3-websockify \
    # 工具
    git vim curl wget \
    && rm -rf /var/lib/apt/lists/*

# rosdep 初始化
# rosdep 初始化 (force re-init 避免 "already exists" warning)
RUN rm -f /etc/ros/rosdep/sources.list.d/20-default.list \
    && rosdep init \
    && rosdep update --include-eol-distros || true

# -----------------------------------------------------------------------------
# 2) 复制源码
# -----------------------------------------------------------------------------
WORKDIR ${WORKSPACE}/src
COPY . ${WORKSPACE}/src/navigation2

# 保险: 手动删除 nav2_zhengqi 目录 (.dockerignore 应已排除, 但保险)
RUN rm -rf ${WORKSPACE}/src/navigation2/nav2_zhengqi || true

# 清理子包内的 build/install/log 残留 (本机 /home/howard/code/navigation2 下
# 残留的 nav2_zhengqi/build/wheelchair_nav/ 会污染 rosdep 扫描)
RUN find ${WORKSPACE}/src -type d \( -name build -o -name install -o -name log -o -name .colcon_buildinfo -o -name CMakeFiles \) \
    -exec rm -rf {} + 2>/dev/null || true

# -----------------------------------------------------------------------------
# 3) 预构建测试脚本 (可选, 真正构建在容器内 bind mount 代码后按需执行)
#    注意: apt 版已包含 nav2_mppi_controller, 无需从源码编译
#    只需要将本地改的 launch 文件 bind mount 进容器
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
# 4) 启动脚本 + 端口
# -----------------------------------------------------------------------------
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

EXPOSE 5900 6080

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
