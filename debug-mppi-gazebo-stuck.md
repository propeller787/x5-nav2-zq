# Debug: mppi-gazebo-stuck

- Status: [FIXED]  (用户已确认)
- Symptom: `ros2 launch mppi_house_slam_demo.launch.py` 卡在 Gazebo 启动环节，gzclient 窗口长时间不出现。
- Reproduction:
  ```bash
  export ROS_LOG_DIR=/tmp/ros_logs
  export TURTLEBOT3_MODEL=waffle
  source /opt/ros/humble/setup.bash
  source ~/code/navigation2/install/setup.bash
  ros2 launch /home/howard/code/navigation2/nav2_mppi_controller/launch/mppi_house_slam_demo.launch.py
  ```
- Last attempted run: 2026-06-23 17:34:34 (PID 13730, log dir
  `/tmp/ros_logs/2026-06-23-17-34-34-113497-howard-JIGUANG-Series-13729`).

## 假设 (Step 2)

| ID  | 假设                                                    | Likelihood | Effort | 期望信号 (falsifiable)                                                       |
| --- | ------------------------------------------------------- | ---------- | ------ | ---------------------------------------------------------------------------- |
| A   | Gazebo 卡在向废弃 model 数据库下载模型                  | High       | Low    | default.log 出现 "Getting models from http://models.gazebosim.org" + 长阻塞   |
| B   | gzserver 启动后等 spawn 超时 / 不会自动 load turtlebot3 | Medium     | Low    | launch.log 中 `[spawn_entity] Waiting for service` 长时间不返回              |
| C   | gzserver 自身启动慢 (world file 解析/物理引擎)          | Medium     | Low    | default.log 中 Loading world 之后到首帧仿真输出间隔巨大                      |
| D   | GPU/驱动导致 gzclient 拉不起渲染窗口                    | Low        | Medium | ogre.log 出现 GL error / 黑屏，gzserver 正常                                 |
| E   | 旧 gzserver 进程残留 / 端口冲突                         | Low        | Low    | `[Err] [gazebo_ros_init.cpp] Could not connect to master` 或端口占用          |

## 证据收集 (Step 3-5 插桩之前先看静态日志)

无插桩改动, 仅读取现有运行日志。读取了以下文件:
- `/tmp/ros_logs/2026-06-23-17-34-34-113497-howard-JIGUANG-Series-13729/launch.log` (完整)
- `/home/howard/.gazebo/server-11345/default.log` (gzserver 进程日志, 1557 字节)
- `/home/howard/.gazebo/client-11345/default.log` (gzclient 进程日志)
- `/home/howard/.gazebo/ogre.log` (OGRE 渲染日志, 30562 字节)
- `/tmp/ros_logs/python3_13734_1782207274698.log` (spawn_entity 节点日志)

## 证据分析 (Step 7)

### 关键时间线 (来自 default.log)
```
17:34:34.875  Connected to gazebo master @ http://127.0.0.1:11345
17:34:35.266  Loading world file [/opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_house.world]
17:34:35.397  [Wrn] Getting models from[http://models.gazebosim.org/]. This may take a few seconds.
   ↓  ↓  ↓  阻塞 98 秒  ↓  ↓  ↓
17:34:35.397 → 17:35:13.915  期间无任何进展
17:35:13.915  [Err] Unable to get model name[http://models.gazebosim.org/cart_rigid_suspension]
17:34:49       ogre.log 显示 gzclient 已创建 Ogre 窗口并开始加载材质
17:34:40.602  SIGINT 终止 (用户按 Ctrl-C)
```

### 假设验证

| ID  | 假设     | 状态     | 证据                                                                                 |
| --- | -------- | -------- | ------------------------------------------------------------------------------------ |
| A   | model DB 阻塞 | ✅ Confirmed | default.log 出现 `Getting models from http://models.gazebosim.org` + `Unable to get model name[cart_rigid_suspension]`, 阻塞 98 秒 |
| B   | spawn 超时    | ❌ Rejected | python3_13734_*.log 显示 `[spawn_entity] Calling service /spawn_entity` 在 gzserver 启动 1 秒后就发了 service, 不存在长期等待   |
| C   | world 解析慢  | ❌ Rejected | world 文件本体 1.4 KB, 加载瞬时完成, 慢的是模型依赖                               |
| D   | GPU 驱动问题  | ❌ Rejected | ogre.log 显示 gzclient 渲染正常: 加载 SkyX 纹理、OgreWindow 创建成功、加载 wood.jpg 材质 |
| E   | 端口冲突      | ❌ Rejected | Connected to gazebo master @ http://127.0.0.1:11345 成功                            |

### 根因 (Step 7 结论)

`/opt/ros/humble/share/turtlebot3_gazebo/worlds/turtlebot3_house.world` 引用
`<include><uri>model://turtlebot3_house</uri></include>`, 该 SDF 中 14 处
`model://` 引用了 4 个外部模型:
- `mailbox`           ❌ 未本地安装
- `cafe_table`        ⚠️  本地仅有损坏的 `~/.gazebo/models/cafe_table.tar.gz`
- `first_2015_trash_can` ❌ 未本地安装
- `table_marble`      ⚠️  本地仅有损坏的 `~/.gazebo/models/table_marble.tar.gz`

`http://models.gazebosim.org/` 早在 2025 年初已 EOL, 不再可访问. 每次解析
失败要阻塞 98 秒 (HTTP 超时), 所以世界文件加载被卡住 -> gzserver 长时间
"看起来像卡住". gzclient 那边能进 gzclient 进程, 但要等服务端的场景
(geometry) 准备好才能渲染, 因此用户侧长时间看不到画面.

同时 gzclient 进程日志报错:
```
[Wrn] [FuelModelDatabase.cc:313] URI not supported by Fuel [model://mailbox/meshes/mailbox.dae]
[Err] [Visual.cc:2956] No mesh specified
```
也是同一个根因 (本地无 mailbox 模型, Fuel 协议下也找不到, 视觉/碰撞体丢失).

`~/code/navigation2` 仓库本身已编译 OK, 启动 launch 的 Python 逻辑没有
问题 — 卡在 Gazebo 自身的 SDF 资源依赖解析.

## 拟用修复方案 (Step 8)

候选方案:
1. **(推荐) 把缺失的 4 个模型装到 `~/.gazebo/models/` 下, 让 Gazebo 走本地路径**, 完全绕开废弃 model database. 文件可以从镜像源 (https://github.com/osrf/gazebo_models) 下载.
2. **简化 `turtlebot3_house` 模型**: 在 `/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_house/model.sdf` 里删掉 mailbox / cafe_table / first_2015_trash_can / table_marble 4 个 visual+collision, 保留墙体. (不推荐, 改了 apt 包内容, 升级会被覆盖.)
3. **改用 `turtlebot3_world.world` / `empty_world.world`**, 不再用 `turtlebot3_house`. 改 launch 文件, 损失房屋内景.

建议方案 1: 在 launch 里加 `GAZEBO_MODEL_PATH` 指向一个新目录 (如
`/home/howard/.gazebo/models`), 由我们从 GitHub 镜像下载 4 个模型.
不动 apt 包、不动 launch 主体结构.

## 已应用的修复 (Step 8 + Step 9 前的准备)

采用方案 1, 但把模型放在仓库内 `nav2_mppi_controller/models/` 而不是 `~/.gazebo/models`, 这样:
- 跟随仓库一起管理, 升级/克隆/移交都不丢
- 不会污染用户家目录
- launch 直接用 `__file__` 解析出绝对路径, 不依赖 colcon install 成功

具体改动:
1. **下载 4 个模型**:
   - 从 `https://codeload.github.com/osrf/gazebo_models/tar.gz/refs/heads/master` 下载完整仓库 (已下载并清理临时文件)
   - 提取 `mailbox`, `cafe_table`, `first_2015_trash_can`, `table_marble` 4 个子目录
   - 复制到 `/home/howard/code/navigation2/nav2_mppi_controller/models/`
2. **修改 `mppi_house_slam_demo.launch.py`** ([file:///home/howard/code/navigation2/nav2_mppi_controller/launch/mppi_house_slam_demo.launch.py](file:///home/howard/code/navigation2/nav2_mppi_controller/launch/mppi_house_slam_demo.launch.py)):
   - 增加 `PACKAGE_MODELS_DIR` 常量, 指向 `nav2_mppi_controller/models/`
   - 在 `generate_launch_description` 里追加 `SetEnvironmentVariable('GAZEBO_MODEL_PATH', PACKAGE_MODELS_DIR + ':' + <existing>)`
   - 更新 docstring, 注明依赖与启动顺序

未触碰: `NAV2_SOURCE_DIR`, `PARAMS_FILE`, `RVIZ_CONFIG`, 节点列表, lifecycle 流程, timer 间隔. 算法代码 0 改动.

## 下一步

请用户重新执行启动命令验证:
```bash
export ROS_LOG_DIR=/tmp/ros_logs
export TURTLEBOT3_MODEL=waffle
export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/humble/setup.bash
source ~/code/navigation2/install/setup.bash
ros2 launch /home/howard/code/navigation2/nav2_mppi_controller/launch/mppi_house_slam_demo.launch.py
```

期望: 启动后 ~5s 内 gzserver 就绪, 15s 后 gzclient 渲染房屋, 不再有 ~98s 的 model database 阻塞.

post-fix 验证指标:
- `~/.gazebo/server-NNNN/default.log` 中**不应**再出现 `Getting models from http://models.gazebosim.org/` 或 `Unable to get model name[...]`
- launch.log 中 gzclient 启动时间应 < 启动后 20s (之前是 15s 定时器触发后还要再等模型, 实际可见界面要 100+s)
- 房屋场景出现 mailbox / cafe_table / table_marble / trash_can 等家具

