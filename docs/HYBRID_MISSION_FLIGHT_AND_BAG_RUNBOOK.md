# 混合任务全流程：启动 EKF → 录 bag → 程控起飞 → B/C → 直控 D → Planner E/F → 直控 G → 经过 H → Planner I → 落点收敛判据后降落与回放

更新：2026-10-28。适用本机 ROS 1 Noetic、`~/gcs_ws` 和当前 `hybrid_mission.py`。

只想要启动 / 看 RViz / 录 bag 的命令清单，看 [混合任务最简步骤](HYBRID_MISSION_QUICKSTART.md)；本手册是完整操作与排查依据。

**这是操作文档，不代表已经执行或验证过实机飞行。** 先完成地面检查；若仍存在起飞振荡、LIO/EKF 跳变或地图错位，不进入执行步骤。飞手必须全程能够立即用 RC 接管。

本流程是：**A 地面原点 → 自动解锁、原地起飞 → 稳定悬停 3 秒 → Planner 到 B → 悬停 3 秒 → Planner 到 C → 悬停 3 秒（投放默认启用时最长等 10 秒检测）→ 纯 PX4Ctrl 到 D → 悬停 3 秒 → Planner 到 E → 悬停 3 秒 → Planner 到 F → 悬停 3 秒 → 纯 PX4Ctrl 到 G → 悬停 3 秒 → 纯 PX4Ctrl 经过 H（不悬停）→ Planner 回 I（A 正上方，航向 180°）→ 悬停 3 秒并等落点参考收敛到 3 cm 以内 → 请求降落并确认锁桨**。不需要飞手先起飞，也不需要再运行旧起飞脚本。

- B 自 2026-10-31 起**不再是经过点**：到点（含 Planner 终点零速保持）后连续稳定悬停 `mission.arrival_dwell`（3 秒），再从静止状态发 C 目标；B→C 是 139° 折返，带速度进弯会被优化器超速连拒。
- F 是 Planner→直控的**切换点**：Planner 到点后同样连续稳定悬停 3 秒，再开始直控飞 G。
- G 由普通 PX4Ctrl 直控，到点连续稳定悬停 3 秒；H 是**经过点**（进入 `pass_radius` 即发 I，不悬停），之后重新交给 Planner 飞 I。
- I 是**终点**：相对 A 的 `[0,0,0.8]`（回到 A 正上方、保持飞行高度，**不要填 z=0 的地面点**），航向 180°、与起飞时相反；连续稳定 3 秒后自动降落。
- B/C/D/E/F/G/H/I 的 `measured` 标记以本轮 YAML 为准；任何 `measured: false` 的点都会让 `--execute` 被拒绝，必须实地测量后填入坐标并改成 `measured: true`。

## 1. 终端分工和绝对不能混用的内容

| 终端 | 内容 | 保持运行到何时 |
|---|---|---|
| 0 | 创建本轮目录、检查/保存任务参数 | 准备完成即可 |
| 1 | MAVROS、雷达、FAST-LIO2、EKF 一键启动 | 落地锁桨后 |
| 2 | PX4Ctrl | 落地锁桨后 |
| 2b | px4_servo（`/servo/push`；投放默认启用时必需） | 落地锁桨后 |
| 3 | 混合任务专用 Planner/轨迹服务器 | 落地锁桨后 |
| 4 | rosbag 录制与地图收包监视 | 落地锁桨后继续录 5～10 秒，再正常关闭 |
| 5 | RViz 实时看图、只读检查 | 根据需要 |
| 6 | 混合脚本预检、确认后执行 | 到 DONE（I 降落后）；异常时由飞手接管 |
| 7 | YOLO 视觉识别（`yolo_trt_ros`；投放靠它出检测，未启动则不会投放） | 与飞行链无关，随时可停；见第 14 节 |

所有实机终端必须连接同一个、已确认的实机 `ROS_MASTER_URI`；新开终端要重新 `source`，环境变量不会自动跨终端继承。

- **只启动 `run_exp_hybrid_mission.launch`，不要再另外启动 `run_exp_single_lio.launch`。** 前者已经包含后者，并隔离了轨迹输出。
- 不运行 multipoint、`takeoff_then_diff_planner.py`、其他航点/位置发布脚本；不点击 RViz 的发目标工具。
- 实机 master 上不执行 `rosbag play`。回放有独立 master，见第 12 节。
- 不在空中编译后重启节点、重新运行任务、改控制参数或重置定位。新一轮只能在落地、锁桨后准备。

正常控制链应为：

```text
FAST-LIO2 → /Odometry → EKF → /ekf/ekf_odom
                            ↓
Diff-Planner → traj_server → /drone_0_mission/planner_cmd
                                      ↓
                         hybrid_mission.py 选择/转发
                                      ↓
                           /setpoints_cmd → PX4Ctrl
```

Planner 段（A→B、B→C、D→E、E→F、H→I）转发规划指令；C→D、F→G、G→H 三段由脚本发布 PX4Ctrl 原生的 `quadrotor_msgs/PositionCommand`，不走 MAVROS 位置控制接口。

## 2. 终端 0：地面准备、创建本轮目录

确认已落地、未解锁、桨和供电安全，停止上一轮任务/控制进程。以下命令只准备文件，不起飞。

```bash
source ~/gcs_ws/devel/setup.bash
export FLIGHT_DIR="$HOME/flight_logs/$(date +%Y%m%d_%H%M%S)_hybrid"
mkdir -p "$FLIGHT_DIR"
printf '%s\n' "$FLIGHT_DIR" > "$HOME/flight_logs/current_hybrid_session.txt"
printf '本轮目录：%s\n' "$FLIGHT_DIR"
df -h "$HOME/flight_logs"
```

**每轮只创建一次。** `current_hybrid_session.txt` 只帮助其他终端找到同一目录，不要在录制中途改成新会话。后文若读取失败、目录不存在或任何准备命令报错，先停止并修复，不继续粘贴后续命令。

首次使用或控制/规划 C++ 源码更新后，先在地面停止旧节点，再编译：

```bash
cmake --build "$HOME/gcs_ws/build" --target px4ctrl_node diff_planner_node traj_server -- -j2
```

已有正确编译产物时不必每轮重编译。编译失败不可继续；运行中的旧进程不会自动加载新二进制。

### 2.1 任务参数单独存一份

```bash
cp ~/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/config/hybrid_mission.yaml \
  "$FLIGHT_DIR/mission.yaml"
nano "$FLIGHT_DIR/mission.yaml"
```

只修改确认过的任务参数；以下是本文撰写时的例子，不是对实际场地的安全承诺：

| 点 | 相对 A 的位置，米 | `yaw_deg` | 说明 |
|---|---|---|---|
| A | `[0.0, 0.0, 0.0]`，地面原点 | 0 | 起飞点 |
| B | `[1.75, 0.0, 0.8]` | 0 | Planner 到点，悬停 3 秒后再切 C（2026-10-31 起不再是经过点） |
| C | `[0.44, 1.46, 0.8]` | 90 | Planner 到点，悬停 3 秒 |
| D | `[0.04, 1.45, 0.8]` | 90 | PX4Ctrl 直控到点，悬停 3 秒 |
| E | `[1.90, 2.77, 0.8]` | 90 | Planner 到点，悬停 3 秒 |
| F | 待实测（占位，`measured: false`） | 待定 | Planner 到点，悬停 3 秒后切直控 |
| G | 待实测（占位，`measured: false`） | 待定 | PX4Ctrl 直控到点，悬停 3 秒 |
| H | 待实测（占位，`measured: false`） | 待定 | PX4Ctrl 直控到点，悬停 3 秒 |
| I | `[0.0, 0.0, 0.8]`，回到 A 正上方 | 180 | Planner 到点，悬停 3 秒后等落点参考收敛（`landing.target_tolerance` 0.03 m / `target_speed` 0.05 m/s）再降落 |

- `coordinate_mode: takeoff_relative`：输入最终 `MISSION` 后，脚本再次检查并锁定地面 EKF 位置，作为 A。
- 仅做坐标平移：`目标 world 坐标 = A 的实际 world 坐标 + 配置中的相对坐标`；不重置 EKF/TF。
- 坐标轴平行 **world**，不是“机头前方/右方”；`yaw_deg` 也是 **world 绝对航向角**，不是相对起飞机头转角。
- 地面应静止，当前航向须满足 A 的 yaw 要求。不要为了通过检查直接填入不理解的角度。
- `takeoff.height` 默认 0.8 米，须与 PX4Ctrl 的 `auto_takeoff_land/takeoff_height` 一致。实际悬停 world z 是地面 EKF z 加 0.8，不一定恰好是 0.8。
- 核对 B/C/D/E/F/G/H/I、实际可飞区域、速度和到点容差。**I 下方必须能降落；起飞竖直通道和 C→D、F→G、G→H 不经过 Planner 避障。**
- `mission.pass_radius` 是 B 的切换半径：太小会为了“碰到 B”而减速甚至停住，太大会提前切 C、实际轨迹更圆滑地切角。它必须小于起飞悬停点到 B 的距离，否则脚本预检直接报错（B 会被立即跳过）。
- 任何尚未实测的航点都要保留 `measured: false`；脚本预览会打印警告，`--execute` 会被拒绝。这是防止用占位坐标实飞的硬保护，不要为了跑通而删掉。
- `mission.arrival_dwell`（3 s）用于 B、C、D、E、F、G 的到点悬停（只有 H 是经过点，不计时）；`mission.hover_seconds` 用于 I（最后一点）的悬停；`landing.target_tolerance`（0.03 m）/ `target_speed`（0.05 m/s）是发 LAND 前的落点判据，`target_timeout`（8 s）是收不紧时的兜底；`landing.approach_height` / `approach_speed` / `approach_tolerance` / `approach_timeout` 是**默认关闭**的可选贴地进近，填 0 即不做进近（判据与原因见说明第 10 节）。`landing.on_ground_height`（0.20 m）是落地收尾判据：PX4 迟迟不报 `ON_GROUND` 时，PX4Ctrl 会在自身落地判据成立 5 s 后交还飞手，脚本据实测离地高度收尾并提示杀桨（判据与原因见说明第 10 节）。
- I 的 z 必须填飞行高度（例如 0.8），不要填 `0`：`[0,0,0]` 是地面点，Planner 会把飞机规划到地面高度。
- Planner 虚拟上下边界是绝对 world z，不随 A 平移。不要把相对飞行高度直接当作虚拟边界。
- 2026-10-27 已从任务脚本移除坐标包围盒及虚拟上下边界预检；旧 YAML 的 `min_xyz`、`max_xyz`、`virtual_wall_margin` 会被忽略。Planner 本身的地图范围、虚拟墙和避障逻辑未修改，不代表任意目标都可达。I 若填 `[0,0,0]` 是地面点；要回到 A 上方 0.8 米悬停后降落，应填 `[0,0,0.8]`。

用本轮副本做离线预览：

```bash
python3 ~/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/scripts/hybrid_mission.py \
  --config "$FLIGHT_DIR/mission.yaml"
```

不加 `--execute` 或 `--check` 时只检查配置、显示任务，不连接 ROS、不起飞。后续预检和执行都用这同一份副本。

## 3. 终端 1：启动 MAVROS、雷达、FAST-LIO2 和 EKF

```bash
bash ~/gcs_ws/src/livox_ros_driver2/scripts/start_fastlio2_ekf.sh
```

该脚本按需启动 roscore，随后启动 MAVROS、PX4 IMU 频率保持、Livox MID360s、FAST-LIO2/RViz、EKF。**它不启动 PX4Ctrl，不解锁，不发起飞请求。** 日志位于 `~/gcs_ws/log/fastlio2_ekf_<时间>/`。

默认飞控连接为 `/dev/ttyACM0:921600`，请求 IMU 200 Hz；如本机连接不同，先核实再使用脚本的 `--fcu-url` 参数。已有这些节点时不重复启动；必要的重启仅在地面锁桨状态进行。

另开检查终端，先看连接和落地状态：

```bash
source ~/gcs_ws/devel/setup.bash
echo "ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}"
rosparam get /use_sim_time
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mavros/extended_state
```

实机 `/use_sim_time` 应为 false；参数未设置时 ROS 默认 false。若为 true，先确认是否接错回放 master，不要在未知节点图里盲改。
要求 `connected: True`、`armed: False`、`landed_state: 1`。未满足则不继续。

以下命令**逐条运行**，观察数秒后在该检查终端 `Ctrl+C`，再运行下一条：

```bash
rostopic hz /ekf/ekf_odom
rostopic hz /Odometry
rostopic hz /mavros/imu/data
rostopic hz /cloud_registered
```

检查实际接收频率和是否断更，不只看“请求了 200 Hz”；当前链路 EKF/IMU 应稳定高于 100 Hz。地面静置时位置、速度、姿态不得明显漂移或跳变。LIO 有数据不等于定位正确，手持正常也不能证明带动力起飞时正常。

**飞行过程中不要在终端 1 按 Ctrl+C：它会停止该脚本启动的定位/飞控通信链。**

## 4. 终端 2：启动 PX4Ctrl，不另行手动起飞

启动控制节点前，飞手确认 RC 开关、解锁状态和接管流程安全；不要从未知的高位开关状态盲启控制节点。

```bash
source ~/gcs_ws/devel/setup.bash
export FLIGHT_DIR="$(cat "$HOME/flight_logs/current_hybrid_session.txt")"
set -o pipefail
roslaunch px4ctrl run_ctrl_lio.launch
```

此 launch 读取 `/ekf/ekf_odom` 和 `/setpoints_cmd`，加载 `px4ctrl/config/ctrl_param_fpv.yaml`。不要同时启动另一套 PX4Ctrl；不要为了起飞盲调质量、悬停油门或控制增益。

在检查终端核对实际加载值：

```bash
rosparam get /px4ctrl/auto_takeoff_land
rostopic echo -n 1 /px4ctrl/state
```

程控起飞要求 `enable: true`、`enable_auto_arm: true`、`no_RC: false`、`takeoff_height` 与本轮配置一致；仍须飞手监护，不是无遥控器自主飞行。

## 4.1 终端 2b：启动舵机（危险品投放默认启用，必需）

`hazard_drop/enabled` 自 2026-10-31 起为 `true`，任务预检会等 `/servo/push`，**舵机节点没起就拒绝起飞**。落地锁桨后再停。

```bash
source ~/gcs_ws/devel/setup.bash
roslaunch px4_servo servo_aux_pwm.launch
```

本 launch 默认 `hold_s=0.0`：舵机推出去后**保持不收回**，直到 `/servo/retract`、下一次动作指令或飞控上锁输出。检查终端核对：

```bash
rostopic echo -n 1 -p /servo/state
rosservice call /servo/reload
```

自检行应显示 `区间 500..2500 us（中点 1500）... push 1400 us（归一化 -0.100）... rest 500 us`；`ready: true` 才能动作。上锁时舵机应停在收起位——若 `PWM_AUX_DIS6` 不是 500（本机曾读到 2500），先按 [px4_servo 说明](../src/px4_servo/README.md) 改参数，否则上锁会把舵机顶在推出位。不需要投放时用 `--no-hazard-drop` 执行任务，不必启动本节点。

## 5. 终端 3：启动混合任务专用 Planner

```bash
roslaunch diff_planner run_exp_hybrid_mission.launch 
```

这一步不应发送目标。不要点击 RViz 发目标，不要启动普通 Planner launch 或旧航点脚本。

检查终端执行：

```bash
rostopic info /drone_0_mission/planner_cmd
rostopic info /setpoints_cmd
rosparam get /drone_0_traj_server/traj_server/hold_at_end
rosparam get /drone_0_diff_planner_node/fsm/flight_type
rosparam get /drone_0_diff_planner_node/grid_map
```

应满足：

- `/drone_0_mission/planner_cmd` 发布者是 `/drone_0_traj_server`。
- **任务脚本尚未执行时，`/setpoints_cmd` 不应有轨迹服务器或其他位置发布者**；执行后唯一控制发布者应是本轮任务脚本。
- `hold_at_end=true`，`flight_type=1`。地图参数是本次运行节点值，不是凭记忆填写的旧膨胀值。
- 飞机尚未起飞、未收到目标时没有规划指令可以正常；但定位和地图必须有数据。

每轮任务最终/保护降落可能给 Planner 发急停；**新一轮在落地锁桨后重启这套隔离 Planner**，不要在空中重启。正常情况下 Planner 段（B、C、E、F、I）与直控段（C→D、F→G、G→H）之间只切换指令来源，不中途发 Planner 急停。

注意：D→E→F 与 H→I 都是 Planner 段，脚本会复用同一个 Planner 节点连续发目标，所以本轮录到的轨迹 ID 必须单调递增（Planner 已修复 `setGlobalTraj` 重置 `traj_id` 的问题）。若日志再次出现“发送某个点后未收到新轨迹 ID”，先查 Planner 是否收到 `/goal`、是否有新轨迹，不要靠关闭检查来绕过。

## 6. 终端 4：先开始录 bag，再执行任务

```bash
python3 ~/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/scripts/record_diff_planner_debug.py 
```

必须显式给 `--output`：录包器**不读取** `FLIGHT_DIR` 环境变量，省略时会自己建 `~/flight_logs/<时间戳>_debug/`，第 11 节按 `$FLIGHT_DIR/recording/` 验收就会找不到文件。

**不要事先 `mkdir "$FLIGHT_DIR/recording"`：录包器要求新目录，已有目录会拒绝覆盖。** 不用 `sudo`。之前出现 `/recording` 权限错误，是因为另一个终端没有设置 `FLIGHT_DIR`；本流程每个需要目录的终端都从同一会话文件读取。

默认持续录制到 `Ctrl+C`，LZ4 压缩、约 1 GB 分片，bag 放在 `recording/` 下；不是传入 `--output` 后立即退出的单次查询。无需 `--execute`。

先观察至少两个地图监视周期，再决定是否执行任务：

```text
[地图监视] RAW=... 帧/... 点，距上一帧 ...s; INFLATE=... 帧/... 点，距上一帧 ...s
```

- 终端仅打印“待执行命令”不代表已经录制；确认出现 rosbag 实际录制输出，目录有增长中的文件，且监视持续更新。
- RAW 和 INFLATE 都应收到消息且持续更新；`0 点`可能是有效空地图，`尚未收到`不是空地图。后者先查地图发布，不能继续飞。
- 看不到地图时，另开只读终端逐条检查 `rostopic hz /drone_0_diff_planner_node/grid_map/occupancy` 和 `rostopic hz /drone_0_diff_planner_node/grid_map/occupancy_inflate`。
- 磁盘空间、缓冲区丢弃和节点频率都要检查。地面录制时已造成 EKF/IMU 断更，就不能带着该负载起飞。

### 6.1 默认录到哪些内容

| 证据 | 主要话题/文件 |
|---|---|
| 未膨胀/膨胀障碍地图 | `/drone_0_diff_planner_node/grid_map/occupancy`、`occupancy_inflate` |
| 输入定位与点云 | `/Odometry`、`/ekf/*`、`/cloud_registered`、`/cloud_registered_body` |
| Planner 轨迹、可视化、心跳 | `/drone_0_diff_planner_node/*`、`/drone_0_planning/*`、`/drone_0_traj_server/*` |
| 混合切换前后指令/阶段 | `/drone_0_mission/*`，含 `planner_cmd`、`state`；以及 `/setpoints_cmd` |
| 目标、起飞/降落、控制状态 | `/goal`、`/px4ctrl/takeoff_land`、`/px4ctrl/state`、`/mandatory_stop_to_planner` |
| IMU、FCU、RC、电池、姿态输出 | 录包器列出的 MAVROS 话题、`/debugPx4ctrl` |
| 坐标和日志 | `/tf`、`/tf_static`、`/rosout`、`/rosout_agg` |
| 参数和连接快照 | `runtime_all.yaml`、`nodes.txt`、`topics_verbose.txt`、关键节点信息；结束时 `end_state/` |

正则覆盖的是默认 drone 0 命名空间；自定义改名不能自动保证覆盖。话题被列入不代表实际有消息，结束后还需验收。

若要保留重算 LIO 所需的原始 Livox 输入，在**启动录包命令时**额外加 `--raw-lidar`，录 `/livox/lidar`、`/livox/imu`。会增加存储/计算负载，先地面测试；不要在飞行中停录重开。默认地图录制不要求这个选项。

## 7. 终端 5：RViz 看实时地图和地面诊断

### 7.1 直接复制：打开当前 RAW 和膨胀障碍地图

**前提：** 已按第 3、5 节启动雷达/FAST-LIO2/EKF 和 `run_exp_hybrid_mission.launch`，并保持它们运行。只看地图不需要启动 PX4Ctrl 或执行起飞脚本，也不需要发航点。节点已运行时不要重复启动。

新开终端执行下面两行；**这条命令不依赖 `FLIGHT_DIR`，也不会起飞或发目标**：

```bash
source ~/gcs_ws/devel/setup.bash
roslaunch diff_planner view_obstacle_map.launch
```

该 launch 同时启动只读的 `flight_path.py`（把 `/ekf/ekf_odom` 累积成 `/ekf/flight_path`）和 RViz，所以实际飞行轨迹会画成连续线条。第一次使用需先 `cd ~/gcs_ws && catkin_make -j2` 安装脚本；只想单独启动 RViz 时用 `start_rviz:=false`，但那样不会出现轨迹线。

若要明确指定本工作区配置，也可以只启动 RViz，二者选一个（只启动 RViz 时没有轨迹线，除非另起 `flight_path.py`）：

```bash
rviz -d "$HOME/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/launch/include/live_obstacle_map.rviz"
```

这是实机实时显示，不是播放旧包。当前终端要与 EKF/Planner 连接同一个实机 master，不要沿用离线回放的 `11321` 环境。

确认 RViz 左侧 `Global Options → Fixed Frame = world`：

| 显示名 | 话题 | 颜色/含义 |
|---|---|---|
| `Occupancy Raw` | `/drone_0_diff_planner_node/grid_map/occupancy` | 橙色，未膨胀占据障碍 |
| `Occupancy Inflated` | `/drone_0_diff_planner_node/grid_map/occupancy_inflate` | 半透明蓝色，膨胀占据障碍 |
| `Flight Path (ekf_odom)` | `/ekf/flight_path` | 黄色连续线条，实际飞行轨迹（不用里程计箭头） |
| `Registered Cloud Optional TF Required` | `/cloud_registered` | 灰色注册点云，默认关闭，需要真实 TF 对齐 |

观看方法：

- 打开/关闭 `Occupancy Raw`、`Occupancy Inflated`，交替看原始与膨胀地图；从俯视和侧视检查实际障碍、地面及可用通道。
- 看轨迹线末端（当前 EKF 位置）与地图是否一致，确认 world X/Y 方向对应真实场地，而不是按机头猜方向。
- 检查起飞竖直通道、A→B、B→C、D→E、E→F、H→I 规划区域、C→D、F→G、G→H 直控通道、I 下方降落区。膨胀地图不应把起飞悬停点或目标包在障碍里。
- 灰色注册点云默认关闭。`/cloud_registered` 的 `camera_init` 与 `world` 若缺真实 TF，不要用随意的单位变换或改 frame_id 强行对齐；见[坐标链路核查](FRAME_TRANSFORM_AUDIT_20261020.md)。
- Points 显示尺寸不等于栅格分辨率。分辨率和膨胀以运行参数及实际地图为准，不能只按画面方块大小判断。

地图在视野外时，将 `Views → Current → Focal Point` 调到当前 EKF 的实际 world XYZ，再调整视角/距离。不要改地图坐标或发布虚假的 TF 来把障碍拉回屏幕。

若手动配置已有 RViz，设置 `Fixed Frame=world`，点击 `Add → By topic`，将上表两个地图话题添加为 `PointCloud2`，设 `Style=Points`、`Decay Time=0`。点的像素大小不是米制体素尺寸。

### 7.2 没地图或地图不更新：只读排查命令

新开同实机 master 的终端，先检查发布者和地图 frame：

```bash
source ~/gcs_ws/devel/setup.bash
echo "ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}"
rosnode list
rostopic info /drone_0_diff_planner_node/grid_map/occupancy
rostopic info /drone_0_diff_planner_node/grid_map/occupancy_inflate
rostopic echo -n 1 /drone_0_diff_planner_node/grid_map/occupancy/header
rostopic echo -n 1 /drone_0_diff_planner_node/grid_map/occupancy_inflate/header
```

`echo -n 1` 在没消息时会等待，可 `Ctrl+C` 退出；等待不表示空地图。当前地图 frame 默认是 `world`，应与 RViz Fixed Frame 一致。

以下频率命令**逐条运行**，每条看数秒后 `Ctrl+C`，再运行下一条：

```bash
rostopic echo /ekf/ekf_odom
```

核对当前实际加载的分辨率、膨胀和坐标参数：

```bash
rosparam get /drone_0_diff_planner_node/grid_map/resolution
rosparam get /drone_0_diff_planner_node/grid_map/obstacles_inflation
rosparam get /drone_0_diff_planner_node/grid_map/frame_id
```

| 现象 | 排查方向 |
|---|---|
| 无话题/发布者 | Planner 是否启动成功、master 是否相同、是否修改 drone ID/话题名 |
| 有发布者但无消息 | 点云和 EKF 是否持续到达，Planner 终端是否报告输入或地图错误 |
| `No transform` | 核对地图 header 与 Fixed Frame；注册点云 `camera_init` 不能随意当作地图 `world` |
| 有消息但看不见 | 是否为空地图、显示是否勾选、视角是否远离 EKF、是否存在可视化裁剪 |
| 画面一直不变 | RViz 可能保留最后一帧；用频率检查或录包器监视确认是否仍在更新 |

这些命令不发控制指令。地图正常显示不证明路径可达或可安全起飞。

**回看 bag 的障碍地图：** 使用第 12 节的独立 master、RViz 和白名单 `rosbag play` 命令，不在当前实机 master 播放历史包。实时看图用 `live_obstacle_map.rviz`，回放用 `flight_replay.rviz`。

### 7.3 保存地图检查报告

下面只检查、不发目标。**先将 `HOVER_WORLD_Z` 改为当前地面 EKF z 加本轮起飞高度**；0.8 只是地面 EKF z 约为 0 时的例子。下面 B 偏移也须跟随本轮 `mission.yaml` 修改。

```bash
source ~/gcs_ws/devel/setup.bash
export FLIGHT_DIR="$(cat "$HOME/flight_logs/current_hybrid_session.txt")"
HOVER_WORLD_Z=0.8
python3 ~/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/scripts/check_diff_planner_map.py \
  --check-z "$HOVER_WORLD_Z" \
  --goal-dx 0.46 --goal-dy 1.35 \
  --output "$FLIGHT_DIR/map_check_B.json"
```

工具读取正在运行的地图参数和消息，推算有效分辨率/膨胀范围。`--check-z` 是 **world 绝对高度**；`--goal-dx/dy` 是相对采样时当前位置的 world 偏移，不会发送 `/goal`。

它只是地面局部探测，不会完整验证 A→B、B→C、C→D、D→E、E→F、F→G、G→H、H→I 或 I 降落通道；不能拿一次报告代替整条任务检查。这里不使用检查器的旧 `--rviz` 配置，统一用上面的实时 RViz 配置。

## 8. 终端 6：先只读预检，确认没有提前手动起飞

按本机已核实的 RC 流程准备程控起飞条件，飞机仍应落地、锁桨。当前脚本检查：

- MAVROS `armed=false`，PX4 `landed_state=1`（ON_GROUND）。
- PX4Ctrl 为 `MANUAL_CTRL(1)`，不是 `AUTO_HOVER(2)`。
- RC 至少 8 通道，CH5/CH6 高于 1750，前四个通道包括油门均在 1500±125。**这是本机程序的通道解释，不可照搬到其他遥控映射。**
- EKF/LIO/IMU、FCU、RC 和地图接收及时，控制指令归属正确；任务脚本不再按坐标包围盒或虚拟墙范围阻止执行，实际地图与目标仍须人工核对。

如果在地面显示状态 2，也不能重复发起飞请求；先保持锁桨，按控制器流程恢复正确地面状态，并查明原因。

```bash

python3 ~/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/scripts/hybrid_mission.py \
--check 

```

`--check` 只订阅/查询，不注册飞行指令发布端，不起飞。查看输出的实际 A 原点和转换后的 hover/B/C/D/E/F/G/H/I **world 坐标**，与 RViz 一致后才继续；预检进程退出后再执行任务，不并行启动两份脚本。

如果预览打印“航点 F/G/H 尚未实测（measured: false）”，说明 `mission.yaml` 里还是占位坐标。此时 `--check` 仍可做地图/定位核对，但 `--execute` 会被直接拒绝，必须先把未实测航点的坐标填好并改成 `measured: true`。

当前按此前要求，**LIO `/Odometry` 的原始 `header.stamp` 不再参与过期/超前判断**；仍检查其接收断更。EKF、IMU 及规划指令等时间戳检查仍保留。这不代表上游 LIO 发散或断更可以忽略，也不证明之前的报警一定是误判。

## 9. 终端 6：程控起飞并执行混合任务

确认录包终端还在运行、地图持续更新、飞手可接管、场地无人，所有实际路径和 I 降落区已人工检查。然后运行：

```bash
python3 ~/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/scripts/hybrid_mission.py \
  --execute \
  --map-verified \
  --direct-path-verified 
```

这里继续使用第 8 节同一个终端的环境。若重新开终端，先重新 `source` 并从 `current_hybrid_session.txt` 读取 `FLIGHT_DIR`。

两个 `verified` 参数只是**人的确认**，不执行自动路径验障，不应为通过命令而盲加。阅读脚本显示的目标后，确实允许本轮自动起飞才输入：

```text
MISSION
```

确认后脚本再次检查、锁定本轮 A，发送**一次** TAKEOFF，由 PX4Ctrl 切 OFFBOARD、自动解锁并起飞；不自动重试，B/C/D/E/F/G/H/I 目标全部由脚本自动发送。预期阶段为：

```text
TAKEOFF → TAKEOFF_HOVER → WAIT_PLANNER_B → PLANNER_B
        → WAIT_PLANNER_C → PLANNER_C
        → DIRECT → DWELL
        → WAIT_PLANNER_E → PLANNER_E
        → WAIT_PLANNER_F → PLANNER_F
        → DIRECT → DWELL → DIRECT → DWELL
        → WAIT_PLANNER_I → PLANNER_I → HOLD → LANDING → DONE
```

| 阶段 | 正常行为 |
|---|---|
| TAKEOFF / TAKEOFF_HOVER | 确认起飞状态、armed+OFFBOARD、IN_AIR，到 A 上方并稳定 3 秒 |
| WAIT_PLANNER_B / PLANNER_B | 发一次 B 目标，收到有效规划指令后转发；B 到点后连续稳定悬停 3 秒，再发 C |
| WAIT_PLANNER_C | 等 C 的新轨迹。等待期间继续转发上一段 B 轨迹（“桥接”），避免在切换点急停；不能用 B 末端保持冒充 C 轨迹 |
| PLANNER_C | 转发 C 轨迹；到点后连续稳定 3 秒（`arrival_dwell`）再切直控 |
| DIRECT（C→D） | Planner 不参与 C→D 轨迹，脚本向 PX4Ctrl 连续发平滑位置指令 |
| DWELL（D） | D 直控到点后连续稳定 3 秒，再发 E 目标给 Planner |
| WAIT_PLANNER_E / PLANNER_E | Planner 接管 D→E；要求新的轨迹 ID，旧轨迹不放行；E 到点稳定 3 秒 |
| WAIT_PLANNER_F / PLANNER_F | Planner 接管 E→F；到点后连续稳定 3 秒，再切直控飞 G |
| DIRECT（F→G）/ DWELL（G） | 直控到 G，连续稳定 3 秒后开始直控 H |
| DIRECT（G→H） | 直控到 H；H 是**经过点**，距离 ≤ `pass_radius` 就立刻发 I 目标，不悬停 |
| WAIT_PLANNER_I / PLANNER_I | 等 I 的新轨迹。等待期间继续跟踪 G→H 的直控参考（不停车）；旧轨迹不放行 |
| HOLD / DESCEND | I 到点连续稳定 3 秒（`hover_seconds`），再等落点参考收敛：实测误差 ≤ `landing/target_tolerance`(0.03 m) 且速度 ≤ `target_speed`(0.05 m/s)，最多等 `target_timeout`(8 s)；仅当 `landing/approach_height>0`（默认 0，不启用）时中间多一段位置控制下降 |
| LANDING | 判据满足（或进近到位）后请求一次 LAND，并打印当时的实测误差；交给 PX4Ctrl 完成最后一段并锁桨 |
| DONE | 同时确认落地和 FCU 已锁桨，不是只发出 LAND 就算完成 |

可在另一个**实机检查终端**只读观察：

```bash
source ~/gcs_ws/devel/setup.bash
rostopic echo /drone_0_mission/state
```

不要为了监视而启动第二份执行脚本。

## 10. 异常时怎么处理

- **明显振荡、位置发散、非预期运动：飞手立即按已验证流程接管。** 不要等脚本自己“纠正”，也不要通过重启定位/任务尝试恢复空中流程。
- 起飞请求发出后，脚本退出、报错或 `Ctrl+C` **不会撤销起飞，也不保证自动降落/停桨**；PX4Ctrl 可能仍在独立执行起飞或悬停。
- Planner 无输出、指令异常、3 秒无有效运动等情况下，只有定位、RC、OFFBOARD、控制归属等仍健康，脚本才可能请求保护降落；不是所有报错都自动降落。
- 定位断更、时间异常、RC 接管、OFFBOARD 退出或控制争抢时，脚本停止继续发位置指令并要求飞手接管，不基于失效定位盲降，也不强制停桨。
- `WAIT_PLANNER_C` / `WAIT_PLANNER_E` / `WAIT_PLANNER_F` / `WAIT_PLANNER_I` 卡住时，保留日志和 bag；不要手动发 `/mandatory_stop_to_planner` 或清急停来硬推进。B→C 等待期内脚本在桥接转发 B 轨迹，若桥接指令断更，脚本按指令失效处理。
- 若只是录包器退出，飞手优先关注飞行安全；不要在空中为了补包重启控制或定位链。

故障保护是应用层检查，不代替飞手，不证明位置估计正确、路径无障碍或降落区安全。

## 11. 落地后停止录制并验收证据

先确认实际落地、已锁桨。让录包再运行 5～10 秒，随后**只在终端 4** 按一次 `Ctrl+C`，等待索引关闭、结束快照与覆盖检查完成。不要用 `kill -9`，不要直接拔电或拔存储设备。

此时控制/定位节点先保持运行，方便录包器保存 `end_state/`；录包器退出后，再按现场地面流程停止任务相关节点。

```bash
source ~/gcs_ws/devel/setup.bash
export FLIGHT_DIR="$(cat "$HOME/flight_logs/current_hybrid_session.txt")"
ls -lh "$FLIGHT_DIR/recording/"
python3 -m json.tool "$FLIGHT_DIR/recording/manifest.json"
find "$FLIGHT_DIR/recording" -maxdepth 1 -name '*.active' -print
```

验收要求：

- 有正常结束的 `.bag` 文件；可能是 `flight_0.bag` 等多个分片，以实际文件为准。存在 `.active` 时先确认是否仍在录制或异常结束，不宣称已完整保存。
- `manifest.json` 的 `coverage.missing_inputs_or_maps` 应为 `[]`；检查 `snapshot_errors`、`end_snapshot_errors` 是否有失败。
- 额外检查目标 `/goal`、阶段 `/drone_0_mission/state`、私有 Planner 指令、最终 `/setpoints_cmd`、PX4Ctrl 和 FCU 状态确实有消息。默认覆盖检查并不验证全部混合任务证据。
- 降落证据两条一起看：终端 6 在发 LAND 前打印的 `落点参考已收敛：误差 x.xxxm、速度 x.xxxm/s`（或未收敛的 `logwarn` 兜底行），以及实际落点与 I 的水平偏差。两者对不上时，问题在估计或增益；前者本身就很大时，问题是悬停没有收敛到位。

在 Bash 中查看本轮各分片的摘要：

```bash
shopt -s nullglob
BAGS=( "$FLIGHT_DIR"/recording/*.bag )
printf '分片数量：%s\n' "${#BAGS[@]}"
for BAG in "${BAGS[@]}"; do
  rosbag info "$BAG"
done
```

分片数量为 0 时先查录制结果，不继续回放。首片里可以没有后半段才出现的目标/指令，应看**所有分片合计**，不能只看第一片就认定没发目标。

保留整个 `$FLIGHT_DIR`：任务参数副本、控制/Planner/任务终端日志、地图检查报告、所有 bag 分片、运行参数和结束快照。分析起飞振荡或飞控内环问题时，还应按现场流程保存对应 PX4 ULog；ROS bag 不能替代它。

## 12. 可选：落地后用独立 master 回放 RViz

**等落地锁桨、结束采集，并停止实机飞行节点；有条件时断开飞控。** 下面只做离线回放，不启动 MAVROS、PX4Ctrl、Planner、轨迹服务器或任务脚本。

### 12.1 回放终端 A：独立 master

```bash
source ~/gcs_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11321
export ROS_HOSTNAME=127.0.0.1
unset ROS_IP
roscore -p 11321
```

端口被占用时先确认是谁在运行，不接入未知 master。

### 12.2 回放终端 B：RViz

```bash
source ~/gcs_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11321
export ROS_HOSTNAME=127.0.0.1
unset ROS_IP
rosnode list
rosparam set /use_sim_time true
rviz -d "$(rospack find diff_planner)/launch/include/flight_replay.rviz"
```

初次检查 `rosnode list` 应只有隔离 master 的 `/rosout`；若出现实机控制节点，先停止准备并检查连接。模拟时间只设置在这个隔离 master。

### 12.3 回放终端 C：读同一轮分片，白名单播放

先按第 11 节确认确实有正常 `.bag`。下面默认最近这轮；看历史试验时，把 `FLIGHT_DIR` 改成那轮目录，不要混入其他会话的分片。

```bash
source ~/gcs_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11321
export ROS_HOSTNAME=127.0.0.1
unset ROS_IP
export FLIGHT_DIR="$(cat "$HOME/flight_logs/current_hybrid_session.txt")"
rosbag play "$FLIGHT_DIR"/recording/*.bag --clock --pause --rate 0.5 \
  /goal:=/replay/goal \
  /setpoints_cmd:=/replay/setpoints_cmd \
  /drone_0_mission/planner_cmd:=/replay/planner_cmd \
  /drone_0_mission/state:=/replay/mission_state \
  /px4ctrl/state:=/replay/px4ctrl_state \
  /mavros/state:=/replay/mavros_state \
  /rosout:=/replay/rosout \
  --topics \
  /ekf/ekf_odom /Odometry /tf /tf_static /cloud_registered \
  /goal /setpoints_cmd /drone_0_mission/planner_cmd /drone_0_mission/state \
  /px4ctrl/state /mavros/state /rosout \
  /drone_0_diff_planner_node/grid_map/occupancy \
  /drone_0_diff_planner_node/grid_map/occupancy_inflate \
  /drone_0_diff_planner_node/goal_point \
  /drone_0_diff_planner_node/global_list \
  /drone_0_diff_planner_node/init_list \
  /drone_0_diff_planner_node/optimal_list \
  /drone_0_diff_planner_node/failed_list \
  /drone_0_diff_planner_node/a_star_list
```

在播放器终端按空格开始/暂停，`Ctrl+C` 结束；`--rate 0.5` 为半速。RViz 在播放前等 `/clock` 属正常现象。

保留独立 master、白名单和改名：**不播放 `/px4ctrl/takeoff_land`、飞控姿态输出等执行话题**。不要改成在实机 master 全量回放。

RViz 的 `EKF History` 是实际估计位置；`Requested Goal` 是目标；`Optimized Trajectory` 是规划可视化；RAW/Inflated 是录制时发布的地图，几者不是同一种轨迹。缺地图消息无法凭 RViz 恢复，不要重启现在的 Planner 冒充历史证据。

要看切换阶段和指令，在另一个终端同样设置上述隔离 master 环境，再分别执行：

```bash
rostopic echo /replay/mission_state
rostopic echo /replay/planner_cmd
rostopic echo /replay/setpoints_cmd
```

这些命令逐条观察或分终端运行。`PositionCommand` 不能直接当 RViz 的 Path 显示。对比“阶段 → 私有 Planner 指令 → 最终输出 → EKF 实际运动”，可判断问题在 Planner 出指令、任务转发还是执行/定位链。

回放结束后关闭隔离 RViz、播放器和 master。下一次实机操作使用新终端，重新确认 master 与 `/use_sim_time`，不要把回放环境带回飞行。

### 12.4 回放自己录制的 bag（任意目录、任意录制方式）

12.1～12.3 针对本手册 `record_diff_planner_debug.py` 录到 `$FLIGHT_DIR/recording/` 的包。下面用于其他方式录下的包，例如手动 `rosbag record`、其他调试脚本、`~/flight_logs/<时间戳>_debug/` 下的 `flight_0.bag`。**仍然只在隔离 master 上回放，绝不在实机 master 上 `rosbag play`。**

#### (1) 先确认包完整可读

```bash
ls -lh ~/flight_logs/<时间戳>_debug/
BAG=~/flight_logs/<时间戳>_debug/flight_0.bag
rosbag info "$BAG"
```

按结果分别处理：

- **`bag unindexed`**：录制进程没有正常退出（例如被 kill、终端直接关闭），`.bag.active` 缺少索引。不要在原文件上修，复制一份再重建索引：

  ```bash
  mkdir -p ~/bag_replay/<名字>
  cp --reflink=auto "$BAG" ~/bag_replay/<名字>/flight.bag.active
  cd ~/bag_replay/<名字>
  rosbag reindex flight.bag.active
  rosbag info flight.bag.active
  ```

  重建后原文件保留为 `flight.bag.orig.active`，同名文件可直接 `rosbag info/play`。确认 `messages` 不为 0、`duration` 合理后再往下做。

- **`messages: 0`、文件只有几 KB、`duration: 0:00s`**：空包，回放没有意义。回到录制终端检查当时是否真的订阅到了话题（录制时话题名写错、节点未启动都会产生空包）。

- **分片 `flight_0.bag flight_1.bag ...`**：`rosbag play` 可以一次接多个文件名，按时间顺序连续播放；只放第一片会漏掉后半段的目标/切换/降落，分析时必须所有分片一起看。

#### (2) 在隔离 master 上白名单播放

先按 12.1 起独立 master、按 12.2 起 RViz 并设置 `/use_sim_time`，然后在播放终端：

```bash
source ~/gcs_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11321
export ROS_HOSTNAME=127.0.0.1
unset ROS_IP
BAG=~/bag_replay/<名字>/flight.bag.active
rosbag play "$BAG" --clock --pause --rate 0.5 \
  --topics /ekf/ekf_odom /Odometry /tf /tf_static /cloud_registered \
           /goal /setpoints_cmd /drone_0_mission/planner_cmd /drone_0_mission/state \
           /px4ctrl/state /mavros/state /rosout \
           /drone_0_diff_planner_node/grid_map/occupancy \
           /drone_0_diff_planner_node/grid_map/occupancy_inflate \
           /drone_0_diff_planner_node/goal_point \
           /drone_0_diff_planner_node/global_list \
           /drone_0_diff_planner_node/init_list \
           /drone_0_diff_planner_node/optimal_list \
  /goal:=/replay/goal \
  /drone_0_mission/planner_cmd:=/replay/planner_cmd \
  /drone_0_mission/state:=/replay/mission_state \
  /setpoints_cmd:=/replay/setpoints_cmd \
  /px4ctrl/state:=/replay/px4ctrl_state \
  /mavros/state:=/replay/mavros_state \
  /rosout:=/replay/rosout
```

- `--topics` 只播白名单；**不要**播 `/px4ctrl/takeoff_land`、`/mavros/setpoint_raw/*`、`/setpoints_cmd` 的原始名等任何执行/飞控输出话题。
- 控制类话题统一改名到 `/replay/...`，防止被误认为当前指令；`--clock` 必须加，否则 RViz/tf 时间对不上。
- `--pause` 先暂停，播放器终端按空格开始/继续，`Ctrl+C` 结束；`--rate 0.5` 半速，`--rate 1` 原速。
- `-s <秒>` 从包内指定时刻开始（相对包起点），`-u <秒>` 只播一段，便于反复看同一次异常。

#### (3) 只截取一段异常时间窗

整包太大或只看某次切换时，先过滤成小包再回放：

```bash
rosbag filter "$BAG" /tmp/window.bag \
  "t.to_sec() >= 1793110983 and t.to_sec() <= 1793110990"
```

时间戳从 `rosbag info` 的 start/end 或 `rosbag play --pause` 的进度里取。过滤出的包按第 (2) 步播放（同样白名单、改名）。

#### (4) 不开 RViz，直接核对数据

另开终端，设置同样的隔离 master 环境后：

```bash
rostopic echo -n 1 /replay/planner_cmd
rostopic echo /replay/mission_state
rostopic hz /replay/planner_cmd /replay/setpoints_cmd
```

也可以不起 master，用 Python 只读解析：

```bash
python3 - <<'PY'
import rosbag
bag = rosbag.Bag('/home/nvidia/flight_logs/<时间戳>_debug/flight_0.bag')
for topic, msg, t in bag.read_messages(topics=['/goal', '/drone_0_mission/planner_cmd']):
    print(topic, t.to_sec(), msg)
bag.close()
PY
```

需要 `source ~/gcs_ws/devel/setup.bash` 后才能 `import rosbag` 和 `import` 话题消息类型。

#### (5) 常见报错

| 现象 | 原因/处理 |
|---|---|
| `ERROR bag unindexed` | 包未正常关闭，按 (1) 复制后 `rosbag reindex` |
| `ROSBagException: Bag contains no message` | 空包或只录到连接没录到数据，检查录制命令和话题 |
| `No messages ... to play` | `--topics` 名字与包内不完全一致；`rosbag info` 看实际话题名，注意前导 `/` 和命名空间 |
| RViz 不动或不显示 | 隔离 master 未 `rosparam set /use_sim_time true`、播放器没按空格、漏了 `--clock` |
| 时间跳变/队列满 | 降 `--rate`、减少 `--topics`、关闭其他占用 CPU 的节点；Jetson 上尤其明显 |

回放结束后关闭隔离 RViz、播放器和 master。下一次实机操作重新开终端确认 `ROS_MASTER_URI` 指向实机，不要把隔离回放环境带回飞行。

## 13. 出发前最后核对

- [ ] 已落地未解锁，飞手接管流程已确认；无起飞振荡/定位发散等未解决问题。
- [ ] 只有一套 EKF、PX4Ctrl 和混合专用 Planner；没有旧目标/控制发布脚本。
- [ ] 本轮 `mission.yaml` 已预览；A/world/yaw、B/C/D/E/F/G/H/I、实际可飞区域和 I 降落区已核实。
- [ ] F/G/H 已实测并改为 `measured: true`（或明确知道本轮不飞）；`pass_radius` 小于起飞悬停点到 B 的距离。
- [ ] RAW/膨胀地图持续更新，坐标对齐；起飞通道、Planner 段（A→B、B→C、D→E、E→F、H→I）与直控段（C→D、F→G、G→H）均人工确认。
- [ ] 录包已实际开始且地图监视正常，磁盘/负载合格；不是仅打印了命令。
- [ ] 危险品投放：`/servo/push` 服务可用（`px4_servo` 已启动、`ready: true`），YOLO 检测话题 `/yolo/detections` 已有数据，投放点 C/D、`height`、`wait_timeout` 已确认；本轮不投放则用 `--no-hazard-drop`。
- [ ] 舵机推出后不自动收回（`hold_s=0`）已确认；投放成功后需 `/servo/retract` 或上锁才收回，`PWM_AUX_DIS6` 为 500。
- [ ] `--check` 已通过；飞机没有被提前手动起飞，RC/控制状态满足程序要求。
- [ ] 确认知道脚本退出不等于飞机停止，异常时优先飞手接管。

## 14. YOLO 视觉识别（投放默认启用，需要它出检测）

`yolo_trt_ros`（USB 相机 + TensorRT 检测，队友新增）只订阅图像、发布检测结果，**不订阅也不发布任何飞行指令话题**，与第 1～13 节的飞行链相互独立；但自 2026-10-31 起 `hazard_drop/enabled: true`，投放判定要靠它的 `/yolo/detections`。没启动时任务照飞，只是 C/D 悬停等满 `wait_timeout`(10 s) 后不投继续。启动命令、话题表、检测 JSON 字段和覆盖参数见 [最简启动文档第 7 步](HYBRID_MISSION_QUICKSTART.md#第-7-步终端-7可选yolo-视觉识别)。

本手册只补充三条与本手册其余内容相关的约定：

- 它占 GPU 与 3 个推理线程，**不在空中第一次启动**；先在地面跑起来，确认 `/ekf/ekf_odom`、`/Odometry` 频率和 Planner 周期没有下降再决定要不要带着飞。
- 默认 `show_window:=true`：启动会在桌面弹出 `YOLO TensorRT` 预览窗口（10 FPS）。飞行中不需要看图就用 `show_window:=false` 启动，少占一点 GPU。
- 第 6 节的录包器话题表固定、不含 YOLO 话题；要留检测结果就另开独立 `rosbag record`（写在 `$FLIGHT_DIR/yolo.bag`），不要改飞行包的录制命令。

参数和状态机细节见 [混合任务说明](HYBRID_PLANNER_PX4CTRL_MISSION.md)；地图细节见 [地图检查](OBSTACLE_MAP_CHECK.md)；历史故障证据见 [文档总入口](README.md)。
