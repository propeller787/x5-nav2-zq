# nav2_zhengqi 复盘与新需求

> 本文是一次失败开发的事后总结与需求重梳。范围仅限 `nav2_zhengqi/` 工作空间。
> 创建日期：2026-06-23

---

## 0. 结论先行

**这是一次失败的开发案例**。已删除所有未经验证的整合代码、未通过的参数配置、过时的设计文档；只保留被验证可独立运行的 `fake_robot.py` 与最小包结构。下面说明失败原因、新需求与经验。

---

## 1. 失败回顾

### 1.1 原始目标
- 在 ROS 2 Humble 沙盒（无 GPU、无 X server）下，端到端验证"摇杆 → MPPI → 轮椅避障"链路。
- 用 `fake_robot` 替代 Gazebo，发布 `/odom /scan /tf /clock`；用 `twist_to_goal` 把 `/cmd_vel` 积分成 Path，喂给 Nav2 的 MPPI `FollowPath` action。

### 1.2 实际做了
| 工作项 | 文件 | 结论 |
|---|---|---|
| `fake_robot` 仿真节点 | `scripts/fake_robot.py` | ✅ 单独运行正常（已保留） |
| `twist_to_goal` 速度积分→Path | `scripts/twist_to_goal.py` | ❌ 端到端未跑通 |
| `trajectory_analyzer` 误差分析 | `scripts/trajectory_analyzer.py` | ❌ `/tracking_error` 始终无数据 |
| `publish_initialpose` 自动初始位姿 | `scripts/publish_initialpose.py` | ⚠️ 是为了解决 `AMCL` 不收敛打的补丁 |
| `nav2_params.yaml` MPPI 配置 | `config/nav2_params.yaml` | ❌ 多版迭代未稳定 |
| `fake_one_click_launch.py` 一键启动 | `launch/fake_one_click_launch.py` | ❌ `bt_navigator` / `controller_server` 频繁 `lifecycle_manager_aborted` |
| `one_click_launch.py` Gazebo 版 | `launch/one_click_launch.py` | ❌ 沙盒无 GPU/无 X server，跑不起来 |
| `wheelchair_nav_start.sh` 启动包装 | 顶层 | ❌ 围绕失败整合打补丁 |
| 4 份设计/计划/复盘文档 | 顶层 `*.md` | ❌ 计划赶不上变化，最终未交付 |

### 1.3 失败原因（按权重排序）

1. **范围失控**：在沙盒里同时做"仿真替代 + MPPI 调参 + 误差分析 + 一键启动 + DDS 修复"，任何一个失败都会拖累其余项；没有"先单点验证再串链路"的纪律。
2. **缺乏退出判据**：每次"启动失败 → 改参数/改 launch 顺序 → 再试"，但没量化"什么算成功"，导致一直在和生命周期/bond 超时斗。
3. **MPPI 参数未隔离验证**：直接照搬社区 yaml，没在更简单的控制器（如 DWB）上跑通过再切 MPPI；一旦 `lifecycle_manager_aborted` 无法分辨是参数、插件、还是 TF 链路问题。
4. **沙盒环境假设过度**：依赖 `FASTRTPS_DEFAULT_PROFILES_FILE + ROS_LOCALHOST_ONLY + ROS_HOME 重定向` 三件套；任一缺失都会让 DDS 卡住，没把这层依赖写进最小可复现案例。
5. **没有真实反馈回路**：误差分析节点需要 MPPI 实际跑出轨迹才有意义，但 MPPI 没跑通，整个验证链悬空。
6. **文档先行 vs 验证滞后**：写了 4 份总计 1000+ 行的设计/计划文档，但实际跑通的只有一个 `fake_robot`。

### 1.4 沙盒内的客观限制（再确认一次）
- 无显卡、无 X server → Gazebo 不可用
- `/home` 在某些路径只读 → `gzserver`/RCL 日志需要重定向到 `/tmp`
- 多网卡 NO-CARRIER（`enp5s0` / `docker0`）→ DDS 发现卡住，需要 `lo` 回环
- 后台进程会被沙盒在 shell 退出时清理 → `nohup`/`setsid` 都救不回来；只能在"同一会话、同一长命令"内完成启动+验证

---

## 2. 新需求（重梳版）

把"想做什么"切小到能在一个沙盒会话里端到端验证的程度。

### 2.1 范围（仅保留能被验证的）
| 目标 | 成功判据 | 不在范围 |
|---|---|---|
| `fake_robot` 持续发布 `/odom /scan /tf /clock`，响应 `/cmd_vel` | 3 秒内 4 个话题均 `hz` 工具能采到频率 | 真实物理仿真 |
| （可选）单点 MPPI 配置 smoke test | `ros2 component types` 能列出 `nav2_mppi_controller::MPPIController`；单个 `controller_server` 能 ACTIVE | 端到端导航、误差量化 |

### 2.2 非目标（明确不做）
- ❌ 端到端避障、跟踪误差、可视化对比——失败案例已经证明在沙盒里走不通，不应再作为前置目标。
- ❌ 多份设计文档、阶段化开发计划——按"先跑通再写文档"原则，文档从简。
- ❌ Gazebo 版本 launch——沙盒跑不了，删。
- ❌ 一键启动脚本的二次包装——先把命令跑通，再考虑封装。

### 2.3 最小技术栈
- ROS 2 Humble
- `nav2_bringup`（仅做 smoke test 时用，平时不引入）
- `tf2_ros`、`geometry_msgs`、`nav_msgs`、`sensor_msgs`、`rosgraph_msgs`、`python3-numpy`
- 不再依赖：`nav2_msgs`、`tf2_geometry_msgs`、`tf_transformations`、`transforms3d`（已从 `package.xml` 移除）

---

## 3. 下一步（如果还要继续）

按"单点验证 → 串链路"的纪律，每次只新增一个变量：

1. **步骤 0（已完成）**：`fake_robot` 单独跑通，4 话题有数据。
2. **步骤 1（建议）**：在 `fake_robot` 之上单独跑 `nav2_map_server`（静态地图），验证 TF 链路 `map → odom → base_footprint` 能建立。
3. **步骤 2（建议）**：再叠加 `controller_server`（先用 DWB 不用 MPPI），验证 FollowPath action 能被调用、`cmd_vel` 能流回 `fake_robot`。
4. **步骤 3（建议）**：DWB 跑通后再切 MPPI，逐步加 critics。
5. **步骤 4（可选）**：再加 `trajectory_analyzer` 和误差分析。
6. **任何一步失败**：先回到上一步，不要跳着改参数。

每一步的退出判据都应该是 shell 命令可证伪的（例如 `ros2 topic hz /cmd_vel` 能输出 `rate`），不是"看 RViz 觉得对"。

---

## 4. 经验总结

### 4.1 技术层
- **沙盒 ≠ 开发机**：所有写"在沙盒里能跑"的代码前，先在 30 行最小复现里验证 DDS / 时钟 / 目录可写性，否则后期排查成本极高。
- **Nav2 lifecycle 错误要分层看**：`lifecycle_manager_aborted` 是表象，真因可能是 bond 超时、TF 没建立、插件类名错、参数错。**先看节点单独能不能 ACTIVE**，再串起来。
- **MPPI 不是调参玩具**：在还没跑通 DWB/默认控制器之前直接上 MPPI，调参面太大，无法定位是 critic 权重、batch size、还是 model_dt 的问题。
- **沙盒后台进程不可靠**：跨 shell 的 `nohup` / `setsid` 不可靠；要么用"同一长命令内启动+验证"，要么承认只能在前台跑。

### 4.2 工程层
- **写文档 ≠ 做开发**：这次写 4 份文档的时间远大于写代码的时间，且最终都被推翻。规则应该是"每写一份文档，先想清楚 30 天后是否还会读它"。
- **失败要早暴露**：第一次 `bt_navigator` 卡 activating 时就该停下来排查，而不是继续叠补丁。
- **"完成"的定义要可证伪**："全链路 OK"在沙盒里没有客观判据，应该改成"启动后 30 秒内这 5 条 `ros2` 命令输出 X"。

### 4.3 方法论
- **小步走、留退出判据、记录每一步证据**：3 条加起来能省 80% 的返工。
- **不要在失败案例上继续叠加**：当一个项目已经从"调一个参数"变成"要重写 launch + 改 4 个 yaml + 装新依赖"时，应该停下来复盘，而不是继续往前冲。

---

## 5. 当前仓库状态

```
nav2_zhengqi/
└── src/
    └── wheelchair_nav/
        ├── package.xml        # 依赖已精简
        ├── setup.py           # 仅保留 fake_robot 入口
        ├── setup.cfg
        ├── resource/
        │   └── wheelchair_nav
        └── wheelchair_nav/
            ├── __init__.py
            └── scripts/
                └── fake_robot.py   # 唯一被验证可独立运行的节点
```

无 `install/` `build/` `log/`，无残留进程。

---

## 6. 复现验证（清理后）

```bash
cd /home/howard/code/navigation2/nav2_zhengqi
source /opt/ros/humble/setup.bash
colcon build --packages-select wheelchair_nav --symlink-install
source install/setup.bash

# 同一会话内启动+验证（沙盒会杀跨 shell 的后台进程）
ros2 run wheelchair_nav fake_robot &
FAKE_PID=$!
sleep 3

# 验证：4 话题都有数据
ros2 topic hz /odom      # 应输出 rate
ros2 topic hz /scan      # 应输出 rate
ros2 topic hz /tf        # 应输出 rate
ros2 topic hz /clock     # 应输出 rate
ros2 topic hz /cmd_vel   # 默认无发布，发了才有

# 验证响应：发一个速度，看 odom 是否变化
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
sleep 1
ros2 topic echo /odom --once

kill $FAKE_PID
```

只有上面 6 条命令**全部可证伪地通过**，才说明 `fake_robot` 真的可用。任何一项失败都意味着该节点也并不可靠，需要回到本节点单独排查，而不是在外层加补丁。
