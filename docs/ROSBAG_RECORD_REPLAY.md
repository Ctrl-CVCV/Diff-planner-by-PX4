# 从 EKF 定点启动到录包、目标交接与 RViz 离线回放

> 本文保留旧 `takeoff_then_diff_planner.py` 单段任务流程与历史故障说明。当前 `hybrid_mission.py` 的 EKF → 录包 → 自动起飞 → Planner B → 直控 C → Planner D → 降落，使用 [混合任务起飞与录包全流程](HYBRID_MISSION_FLIGHT_AND_BAG_RUNBOOK.md)，不要混用本文普通 Planner launch。

更新：2026-10-21。适用 ROS 1 Noetic、当前 `~/gcs_ws`。

本文的命令是供操作者按条件执行的步骤，不是已经执行过的飞行。
**当前应先解决地图把起点/目标判为占据的问题，不要为补录证据盲目再飞。**
本次包的结论见 [2026-10-20 航点后仍悬停的分析](FLIGHT_ANALYSIS_20261020_2145.md)。

## 0. 本次代码更新与飞行保护

先读 [文档总入口](README.md)、[坐标链路核查](FRAME_TRANSFORM_AUDIT_20261020.md)。
当前无法用缺地图的历史包确定占据来源；完成地面检查之前不要盲目再飞。
本次更新涉及 Python、PX4Ctrl 的 LAND 状态入口及两种地图消息时间戳。源码变更后需编译：

```bash
source ~/gcs_ws/devel/setup.bash
cmake --build "$HOME/gcs_ws/build" --target px4ctrl_node diff_planner_node -- -j2
```

编译本身不启动节点。**仅在确认落地、锁桨、停止旧控制节点后**重启更新的节点；空中不得重启。
旧进程不会因为磁盘上的二进制更新而自动变成新版。只更新 Python 而仍运行旧控制器，LAND 可能被拒绝。

默认执行命令保持不变：脚本发目标后继续观察3秒，最大水平位移不足5cm时，
在 fresh EKF/IMU/RC、armed+OFFBOARD、AUTO_HOVER/CMD_CTRL 条件下只发一次 LAND。
新版 PX4Ctrl 接受该 LAND 后进入 AUTO_LAND，忽略规划指令；脚本最多等待2秒确认状态5。
遥控接管、定位过期或模式改变时不抢控制、不盲发降落；没有状态5确认就不能宣称已开始降落。
正下方必须可安全降落；执行前仍需飞手负责。3秒初始检查不是全程看门狗，
脚本退出/被终止不会提供持续保护，脚本自身不会直接停桨，PX4Ctrl 在正常落地流程中可自动停桨。
细则见 [顺序脚本说明第13节](TAKEOFF_THEN_DIFF_PLANNER.md)。

## 1. 必须区分两种环境

| 环境 | 运行的内容 | 禁止事项 |
|---|---|---|
| 实机采集 | MAVROS、雷达、FAST-LIO2、EKF、规划器、PX4Ctrl、录包器 | 不回放任何历史 bag；地图/定位异常时不起飞 |
| 离线回放 | 独立的 `11321` master、RViz、rosbag play | 不启动 MAVROS、PX4Ctrl、起飞脚本、规划器或轨迹服务器 |

`rosbag play` 会重新发布消息，不是只打开一个视频。包含 `/goal`、起飞请求或控制指令的
bag 不能直接回放到连接实机的 master。下面同时使用独立 master、话题白名单和控制话题重映射。
必须等飞机安全落地、锁桨后才切换到离线环境；有条件时再断开飞控连接。

## 2. 实机采集：终端 1 启动感知和 EKF

开始新一轮前，确认飞机落地、未解锁，没有遗留的控制器/起飞脚本。
已有节点运行时不要重复启动；停止、重启控制链只能在地面、锁桨且确认安全后进行。

```bash
bash ~/gcs_ws/src/livox_ros_driver2/scripts/start_fastlio2_ekf.sh


这个已有脚本启动 MAVROS、IMU 频率保持、Livox、FAST-LIO2/RViz、EKF；
**它本身不启动 PX4Ctrl，也不发起飞请求。** 等待定位初始化完成，静置检查定位是否稳定。
`current_session.txt` 只是方便后续终端读取同一目录；一轮采集完成前不要再次创建新会话。

在另一终端进行只读检查，每条 `rostopic hz` 观察后按 `Ctrl+C` 返回：

```bash
source ~/gcs_ws/devel/setup.bash
rostopic echo -n 1 /mavros/state
rostopic echo -n 1 /mavros/extended_state
rostopic hz /ekf/ekf_odom
rostopic hz /mavros/imu/data
```

起飞前必须确认连接正常、`armed: False`、`landed_state: 1`，并且 EKF、IMU 实测频率稳定高于
100 Hz。不要只看请求的 200 Hz。定位跳变、明显漂移、方向错误或电池告警均需先处理。

## 3. 终端 2 启动规划器，不发送目标

```bash

roslaunch diff_planner run_exp_single_lio.launch 
```

不要启动 multipoint，不要点击 RViz 的发布目标工具。
规划器有些状态机、优化器输出使用 `cout` 而不是 ROS 日志，所以同时保留终端日志。
现在改 launch 不等于已经运行的地图参数改变；参数只在相应初始化时被读取。

### 3.1 地面实时 RViz 查看当前障碍地图

这一步不播放 rosbag，也不需要 PX4Ctrl 或起飞脚本；保持未解锁，新开终端连接上述实机 master：

```bash
source ~/gcs_ws/devel/setup.bash
rviz -d "$(rospack find diff_planner)/launch/include/live_obstacle_map.rviz"
```

配置默认 `Fixed Frame=world`：橙色为未膨胀占据图，半透明蓝色为膨胀图，黄色箭头为 EKF 位置。
左侧分别切换 `Occupancy Raw`、`Occupancy Inflated`，从侧视与俯视检查障碍和通道。
地图显示使用 Points，不用固定大小的方块冒充实际分辨率；没有发布航点的工具。
灰色 `/cloud_registered` 默认关闭，缺少 `camera_init → world` 的真实 TF 时不要强行叠加。
完整话题、手动 Add 步骤、参数/频率核对和空图/TF 排错见[地图检查手册第 3.1 节](OBSTACLE_MAP_CHECK.md)。

**不要沿用第 8～10 节离线回放的 `11321` master 或模拟时间环境来看实机图；
不要在实机 master 上执行 `rosbag play`。** 只做地图检查可在此停止，不必继续启动控制器或起飞。

## 4. 终端 3：先准备 RC，再启动 PX4Ctrl

**仅在落地且未解锁时操作。** PX4Ctrl 尚未启动前，先保持你们已经验证的自动起飞组合：

- 本地源码解释的 CH5、CH6 均高于 1750；它们不等同于 QGC 模式名称。
- CH1～CH4 全部居中，包括油门；脚本检查范围为 1500±125。
- 保留已验证的飞手接管能力，不绕过飞控的解锁检查。

然后启动控制器，保持开关不动：

```bash
source ~/gcs_ws/devel/setup.bash
export FLIGHT_DIR="$(cat "$HOME/flight_logs/current_session.txt")"
set -o pipefail
roslaunch px4ctrl run_ctrl_lio.launch \
  2>&1 | tee "$FLIGHT_DIR/px4ctrl_terminal.log"
```

源码会把第一帧 RC 开关位置作为初始值；若先启动控制器再把 CH5 从低拨高，
可能在地面就进入 `AUTO_HOVER(2)`。状态 2 本身不证明已经离地，但起飞脚本仍要求从
`MANUAL_CTRL(1)` 开始，不能简单放开为允许状态 2 再起飞。
若已出现该情况，只有确认落地、`armed=false` 后才能按本节顺序重启控制器；不要在空中操作。


### 5.1 新终端没有 FLIGHT_DIR 时如何单独录包

Shell 变量不会自动出现在另一个新终端。若 `FLIGHT_DIR` 未设置或为空，
`--output "$FLIGHT_DIR/recording"` 会展开成 `/recording`，不是家目录下的 recording。
普通用户无法创建根目录中的该目录；这时录包器尚未启动，也没有记录这次飞行。
终端打印的 `rosbag record ...` 是准备执行的命令，不是录包成功证明，不要用 `sudo` 处理这个问题。

**不需要关联已有会话时，最简单是省略 `--output`，由脚本创建家目录下的新目录：**

```bash
source ~/gcs_ws/devel/setup.bash
python3 ~/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/scripts/record_diff_planner_debug.py
```

默认路径为 `~/flight_logs/YYYYMMDD_HHMMSS_debug/`；终端会显示实际目录。
若要与全流程的控制器终端日志放在同一会话，请使用本节前面读取 `current_session.txt` 并校验非空的命令。
确认终端没有退出报错、出现实际录包输出和持续的地图监视；RAW/INFLATE 缺失时先排查，不要直接起飞。

### 5.2 明显振荡时先停止自动试飞，不靠改地图参数解决

若仍在空中，由飞手按已验证的接管流程安全落地；地面锁桨后排查。
仅凭肉眼振荡无法判定是规划、外环控制、飞控内环、定位时延还是机体/电机/桨叶问题。
发生振荡的这一次若录包器在创建目录时报错，不能把历史包当作这一次的证据。

本机 `run_ctrl_lio.launch` 加载 `px4ctrl/config/ctrl_param_fpv.yaml`。
截至本次源码核对，文件为 `mass=2.35`、`hover_percentage=0.340`、
`accurate_thrust_model=false`、`pose_solver=0`；这不是已经运行节点的参数证明。
代码中初始 `thr2acc=gra/hover_percentage`，近似模型输出为 `des_acc_norm/thr2acc`。
因此把悬停油门从 0.45 改成 0.34，会改变同一加速度目标对应的初始归一化推力；
不能把任意模式下观察到的遥控器油门杆位置直接当成这个参数。
推力模型自适应只在 armed 且 AUTO_HOVER/CMD_CTRL 时执行，并非从 AUTO_TAKEOFF 一开始就执行。
质量也参与当前控制算法，不应在没有数据时断言与质量无关或盲目调回错误质量。

在地面先确认实际加载参数、静止 EKF/IMU 是否稳定、机体/桨叶和传感器固定是否正常。
后续按现场安全流程取得有效记录时，至少比较 `/px4ctrl/state`、`/debugPx4ctrl`、
`/ekf/ekf_odom`、`/Odometry`、`/mavros/imu/data`、`/mavros/setpoint_raw/attitude`、
`/mavros/setpoint_raw/target_attitude` 与 `/setpoints_cmd` 的时间先后；录包器默认已覆盖这些话题。
若振荡发生在目标发布前且控制状态尚未进入 CMD_CTRL，应先检查起飞/悬停控制和定位链，
而不是先归因于本次规划目标或地图膨胀。最终区分机械振动、内环跟踪和输出饱和通常还需配套飞控 ULog。
`/debugPx4ctrl.des_thr` 在当前源码中记录的是加速度投影，不是 0～1 的最终推力；
最终归一化推力请查看发送给 MAVROS 的 AttitudeTarget 消息的 `thrust` 字段。

### 5.3 录包覆盖与验收

默认覆盖点云 world/body、LIO/EKF、两路 MAVROS IMU、TF、FCU/RC/电池/控制指令、时间同步，
并用正则订阅整个 `drone_0_diff_planner_node`、`drone_0_planning`、`drone_0_traj_server` 与 `ekf` 命名空间。
这样无需手动逐个补 Marker/调试话题，晚于录包器启动的规划话题也可以被发现。
其他 drone ID 必须显式传入 `--drone-id`。自定义重映射到其他命名空间的输入不在默认覆盖保证内。

**每 5 秒看地图监视：RAW 和 INFLATE 都应收到消息，而且更新不能停住。**
`0 点` 可以是有效空地图；`尚未收到` 则不是空地图，而是没有消息。先解决没有地图消息，不能继续飞行。
录包终端保持运行，确认没有磁盘、缓冲区丢弃或订阅异常，不要同时再开另一份旧录包命令。
规划轨迹和 `/setpoints_cmd` 在发目标前没有消息可以是正常现象；地图/定位此时应已有数据。
录包器订阅 Marker/地图话题，也有助于保存仅在存在订阅者时才发布的可视化。

开始时保存全参数 `runtime_all.yaml`、节点列表、详细话题列表和关键节点连接；结束时再保存 `end_state/`。
参数服务器会保留已退出节点的参数：看到某命名空间存在不代表对应节点仍在运行，须结合节点列表。
参数 YAML 是采样时的快照，不是飞行全程的参数变化记录；飞行期间不要改参数或重启感知。

注册点云和地图会增加磁盘及 CPU 负载。先在地面确认录制期间 EKF/IMU/控制频率与消息新鲜度
没有变差，磁盘空间充足；不要为了排查飞行问题反而造成链路超时。
这份清单用于保留规划输入/输出和地图可视化，**不等于原始传感器完整备份**。
若需重新运行 FAST-LIO2，在这条命令末尾加 `--raw-lidar`，同时录 `/livox/lidar` 和 `/livox/imu`；
实际使用的 `/mavros/imu/data_raw` 已在默认清单内。先做短时地面负载检查，保存源码/配置版本及对应消息包，
并单独设计隔离的离线重算流程；本文不重算定位或地图，也不默认录 `/mavlink/from` 等全部高带宽原始流。

## 6. 终端 5：地图检查、只读预检，再决定是否执行

```bash
source ~/gcs_ws/devel/setup.bash
export FLIGHT_DIR="$(cat "$HOME/flight_logs/current_session.txt")"
set -o pipefail
python3 ~/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/scripts/check_diff_planner_map.py \
  --check-z 0.85 --goal-dx 0.5 \
  --output "$FLIGHT_DIR/map_check.json" --rviz
python3 ~/gcs_ws/src/livox_ros_driver2/scripts/takeoff_then_diff_planner.py \
  2>&1 | tee "$FLIGHT_DIR/preflight_terminal.log"
```

这里 `--check-z 0.85` **只是绝对 world/EKF 高度的例子**，要换成实际预计悬停 z：
地面 EKF z 加运行时相对起飞高度。本次样本实际悬停约 0.857m。
`--goal-dx` 在地图检查中只作假设探测，不会发送目标；诊断工具也不是完整碰撞检查器。
地图消息存在、脚本预检通过，不能证明起点到目标真正可达。
在线检查器的 `--rviz` 打开的是旧 `map_check.rviz`，其中 Boxes 的显示尺寸是固定值，
并不会随运行参数自动更新；应核对显示尺寸，或改用本文不依赖固定米数方块的 Points 配置。

只有地图确实正确、起点/目标与安全通道明确可用、飞手与场地条件满足时，才运行以下执行命令。
**目前已有“起点在障碍中”的证据，问题未解决之前不要执行：**

```bash
python3 ~/gcs_ws/src/livox_ros_driver2/scripts/takeoff_then_diff_planner.py \
  --execute --map-verified --goal-dx 4.0 --goal-dy 0.0 \
  2>&1 | tee "$FLIGHT_DIR/takeoff_terminal.log"
```

如只验证起飞与悬停，不给目标，改用下面命令，**二选一，不要连着执行两次**：

```bash
python3 ~/gcs_ws/src/livox_ros_driver2/scripts/takeoff_then_diff_planner.py \
  --execute --map-verified --takeoff-only
```

`--takeoff-only` 不启用航点无位移保护，后续由飞手按已验证流程降落。
需要人工输入 `TAKEOFF`；起飞请求可能间接触发 PX4Ctrl 自动解锁。
`--goal-dx 0.5` 是 world/EKF 的 +X，不是自动沿机头前方。
除明确满足3秒无位移保护并确认 AUTO_LAND 的分支外，脚本退出不会自动补发降落或撤回目标；录包器退出更不会改变飞行。

**“收到新指令 + CMD_CTRL(3)”只说明链路接上，不能证明生成了向目标运动的轨迹。**
本次规划失败后生成的原地应急停留轨迹也满足该条件。
新版会在交接后继续监护到发目标后3秒：位移不足阈值就请求 LAND，即便收到的是原地应急轨迹。
仍必须观察指令位置/速度和规划日志，不能把交接或5cm位移通过当作到点或规划成功。

## 7. 落地后结束录包并验收

由飞手按已验证流程接管、降落，确认锁桨。之后在录包终端按 `Ctrl+C`，等待保存完成。
不要在正在写入的 `.bag.active` 上执行重建索引/修复或覆盖写入。

```bash
source ~/gcs_ws/devel/setup.bash
export FLIGHT_DIR="$(cat "$HOME/flight_logs/current_session.txt")"
for bag in "$FLIGHT_DIR"/recording/*.bag; do rosbag info "$bag"; done \
  | tee "$FLIGHT_DIR/bag_info.txt"
python3 ~/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/scripts/analyze_diff_planner_frames.py \
  "$FLIGHT_DIR"/recording/*.bag \
  --params "$FLIGHT_DIR/recording/runtime_all.yaml" \
  --output "$FLIGHT_DIR/frame_audit.json"
```

重点核对：

- `/goal`、`/setpoints_cmd`、`/px4ctrl/state`：是否给目标、给了怎样的指令、控制器是否接管。
- `/drone_0_planning/trajectory`、心跳：是正常运动轨迹、应急停留，还是链路中断。
- `/cloud_registered`、原始/膨胀占据图：必须有消息才可能回看障碍物。
- `/rosout` 与终端日志：是否有起点碰撞、目标碰撞、规划失败、应急停留、心跳丢失。
- `recording/runtime_all.yaml` 与结束快照：不能仅靠后来修改过的 launch 还原飞行时参数。
- `recording/manifest.json`：记录所有话题计数；缺地图/LIO/EKF/IMU 时脚本以状态码 2 结束，不能忽略。

坐标分析脚本按 header 时间配对 LIO/EKF，报告位置/姿态差、world/body 点云变换抽样误差、
FCU/EKF 姿态对齐变化、实际 frame_id、TF 边与零时间戳。它不会启动 master、播放 bag 或发布消息。
返回成功只表示报告生成成功，不表示坐标链路、地图或飞行安全检查通过。
只传同一会话的不重叠分片，不要混合不同重启或重复的 bag；报告输出必须为新文件。
详细核查结论与已知限制见 [FRAME_TRANSFORM_AUDIT_20261020.md](FRAME_TRANSFORM_AUDIT_20261020.md)。

没有发过目标时目标/轨迹计数为零不一定是录制错误。
但地图话题缺失或一直零条，不能在回放时凭空恢复当时的地图。

## 8. 离线回放：终端 A 创建隔离 master

下面每个回放终端都需要设置环境，**只在一个终端 export 不会影响其他终端**。
实机节点不要连接到这里，不要在回放环境执行前面的启动/起飞命令。

```bash
source ~/gcs_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11321
export ROS_HOSTNAME=127.0.0.1
unset ROS_IP
roscore -p 11321
```

终端 A 保持运行。如果端口已被占用，先确认占用者，不要把回放接入未知节点图。

## 9. 终端 B：先开启 RViz，再播放

```bash
source ~/gcs_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11321
export ROS_HOSTNAME=127.0.0.1
unset ROS_IP
rosnode list
rosparam set /use_sim_time true
rviz -d "$(rospack find diff_planner)/launch/include/flight_replay.rviz"
```

启动 RViz 前，`rosnode list` 应只有隔离 master 的 `/rosout`；如果出现 MAVROS、PX4Ctrl、
规划器、轨迹服务器或起飞脚本，先停止回放准备并检查环境，不要继续。
`/use_sim_time` 只设置在这个隔离 master，不要修改实机 master。

这个专用 RViz 配置默认 `Fixed Frame=world`，只带移动视角、选择、测量等工具，
不带发布目标或初始位姿的工具。

## 10. 终端 C：白名单播放与控制消息改名

下面先使用本次实际存在的 bag。其他录包请修改 `BAG`，不要仅凭目录名猜录制时间：

新录包器通常生成 `recording/flight_0.bag` 等分片；将下面的输入文件换成实际文件。
需要连续播放同一会话的多个分片时，用 `"$FLIGHT_DIR"/recording/*.bag` 替代 `"$BAG"`，
仍保留后面的隔离 master、白名单与重映射；不要把通配符一起放进引号，也不要混入旧会话。

```bash
source ~/gcs_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11321
export ROS_HOSTNAME=127.0.0.1
unset ROS_IP
export BAG="$HOME/flight_logs/20261020_211622/takeoff.bag"
rosbag info "$BAG"
rosbag play "$BAG" --clock --pause --rate 0.5 \
  /goal:=/replay/goal \
  /setpoints_cmd:=/replay/setpoints_cmd \
  /px4ctrl/state:=/replay/px4ctrl_state \
  /mavros/state:=/replay/mavros_state \
  /rosout:=/replay/rosout \
  --topics \
  /ekf/ekf_odom /Odometry /goal /setpoints_cmd \
  /px4ctrl/state /mavros/state /rosout /tf /tf_static \
  /cloud_registered \
  /drone_0_diff_planner_node/grid_map/occupancy \
  /drone_0_diff_planner_node/grid_map/occupancy_inflate \
  /drone_0_diff_planner_node/goal_point \
  /drone_0_diff_planner_node/global_list \
  /drone_0_diff_planner_node/init_list \
  /drone_0_diff_planner_node/optimal_list \
  /drone_0_diff_planner_node/failed_list \
  /drone_0_diff_planner_node/a_star_list
```

- 在终端 C 按空格开始/暂停；`Ctrl+C` 结束播放器。`--rate 0.5` 表示半速。
- `/clock` 驱动隔离环境的模拟时间；开始播放前或暂停时 RViz 等待时钟是正常现象。
- 原始 `/goal` 被改为 `/replay/goal`；控制指令和状态也只发到 `/replay/...`。
- 白名单不包括 `/px4ctrl/takeoff_land`、`/mavros/setpoint_raw/attitude` 等执行话题。
- 不要删掉隔离、重映射和白名单，改成在实机 master 全量回放。

**本次 bag 的快速定位：** 在同一条 `rosbag play` 命令里，`--topics` 前再加
`--start 570 --duration 60`，可跳到本次起飞前约 6 秒并覆盖目标、悬停与降落。
`--start` 是相对 bag 开头的秒数，不是 UNIX 时间，也不是相对目录时间；其他 bag 不可照搬 570。
这次目标出现在 bag 开始后约 588.417 秒。建议 RViz 先就绪、播放器初始暂停，避免漏掉仅一条的目标。

回放结束或重播后，可在 RViz 点击 Reset 清理旧显示；若时间跳回导致 TF 缓存异常，重新打开该
离线 RViz 再从所需时间之前播放，不要靠启动实机节点补数据。

## 11. RViz 看什么，缺什么不能补什么

| RViz 显示 | 话题 | 用途 |
|---|---|---|
| EKF History | `/ekf/ekf_odom` | 实际估计位置的历史箭头，不是规划指令 |
| Requested Goal | `/replay/goal` | 实际发送的目标；箭头朝向是目标姿态，目标位置是箭头根部 |
| Registered Cloud | `/cloud_registered` | 注册点云；需要与 world 的真实 TF 关系 |
| Occupancy Raw / Inflated | 对应 `grid_map/occupancy*` | 当时发布的占据地图可视化 |
| Global Reference | `global_list` | 全局参考线，不保证避障成功 |
| Optimized Trajectory | `optimal_list` | 发布过的优化可视化，不等同于飞机实测轨迹 |
| Failed Candidates / A Star | `failed_list` / `a_star_list` | 可选打开，查看失败候选与搜索结果 |

地图显示用 Points 而不是固定米数的 Boxes，避免 RViz 方块尺寸被误认为实际栅格分辨率。
如果想改成 Boxes，必须按本轮运行参数及代码推算的有效分辨率设置，不能沿用以前的 0.125。
本地地图可视化有前向与高度裁剪，空白区域不自动等于完整地图中的自由空间。

**本次现有 bag 没有注册点云、原始/膨胀地图、规划多项式、心跳、Marker 和 TF。**
因此可以回看 EKF 位置和目标，但不能展示当时障碍地图/规划曲线，也不能重建哪个点造成了占据。
RViz 对缺失显示报 No messages 时，把对应显示取消勾选即可；不要把它当成“当时没有障碍”。
不要启动当前版本规划器来“补”历史地图，所得结果不再是原始飞行证据。

FAST-LIO2 的注册点云源码使用 `camera_init`，EKF/规划地图使用 `world`。
如果后续完整 bag 中仍没有两者的真实 TF 关系，RViz 可能提示 `No transform`。
先核对采集时的坐标定义和 TF；也可以单独以 `camera_init` 为 Fixed Frame 检查点云，
但这不是证明它与 EKF 地图已经对齐。**不要为消除显示错误随意发布单位变换或改 frame_id。**

## 12. 回放时同时检查指令和状态

终端 D 同样连接隔离 master：

```bash
source ~/gcs_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11321
export ROS_HOSTNAME=127.0.0.1
unset ROS_IP
rostopic echo /replay/setpoints_cmd
```

在指令出现前开始观察；看完按 `Ctrl+C`，再按需分别运行：

```bash
rostopic echo /replay/px4ctrl_state
rostopic echo /replay/goal
rostopic echo /replay/rosout
```

`PositionCommand` 不是 RViz 内置的 Path 类型，不能直接 Add Path 后选它就看到指令线。
要区分三件事：目标位置、指令中的位置/速度、EKF 实际位置。
本次 200 条指令的位置完全不变，速度/加速度均为零；即使进入状态 3，也是在执行原地停留。
日志中的 “The drone is in obstacle” 指规划器的占据判断，不是物理撞机传感器的证明。

## 13. 结束回放

先在播放器终端 `Ctrl+C`，关闭离线 RViz，再停止独立的 `roscore -p 11321`。
关闭这些回放终端最不容易把环境混用。以后做实机检查时使用新的终端和正确 master，
先核对 `ROS_MASTER_URI`、`rosnode list` 和 `/use_sim_time`，不要把回放环境当成实机链路。

## 14. 本教程验证范围

2026-10-21：PX4Ctrl/规划器目标编译通过；起飞脚本49项、录包/坐标诊断17项离线测试通过。
当前5份主要操作/审计手册的Shell代码通过 `bash -n`，迁移入口与本地Markdown链接检查通过。
历史部署资料保留有占位命令（例如 `<your-fcu-url>`），不是可直接复制执行的现行清单。

命令选项按本机 ROS Noetic 的 `rosbag_main.py` 核对，RViz 配置做 YAML/话题/工具静态检查，
Shell 片段做语法检查。当前 bag 通过 `rosbag` Python API 只读分析。
已进行静止、未解锁条件下的只读短录包；录包器5秒烟测在规划器未运行时正确报告缺少两种地图并以状态码2结束。
合成 bag 验证了两种地图的收集验收与零点空图处理。没有启动实机飞行节点、没有回放到实机、没有发送起飞/航点/降落，也没有进行 RViz GUI 或本次新保护实飞验证。
