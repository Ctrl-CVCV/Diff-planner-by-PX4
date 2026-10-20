# PX4Ctrl + Diff-Planner：悬停未交接、障碍地图检查与禁止上方绕障

更新日期：2026-10-20。本说明对应本工作区的 ROS1 代码。

## 1. 已有录包能说明什么

录包和参数快照：

```text
/home/nvidia/flight_logs/20261019_224454/takeoff.bag
/home/nvidia/flight_logs/20261019_224454/px4ctrl_runtime.yaml
```

- 录包约 105 秒，开始时已在自动起飞；约第 6.29 秒进入 `AUTO_HOVER(2)`，之后没有 `CMD_CTRL(3)`。
- 第 6.3～20 秒，EKF 高度约 `0.818～0.857 m`，速度模长中位数约 `0.035 m/s`，约 99.2% 的里程计样本速度不超过 `0.12 m/s`。不能仅因之前有振荡就认定本次必然无法通过稳定悬停检查。
- 以录包首帧位置近似起飞原点，第 6.43～27.29 秒连续满足当前脚本的位置/高度/速度阈值；这不是完整脚本执行复现，因为缺少其真实起飞前原点、时钟及全部检查上下文。
- RC 在约第 27.30 秒前满足当前脚本的居中与 CH5/CH6 条件，之后 CH6 从高位切到低位；约第 41.05 秒飞控上锁。不要把后半包的地面状态混入悬停推力统计。
- 没有 `/goal`、`/setpoints_cmd` 的消息记录，没有障碍地图，也没有起飞顺序脚本的完整终端输出。因此不能确定是未发目标、脚本提前退出、规划失败还是地图堵塞。
- 参数快照是 `mass=2.35`、`hover_percentage=0.34`、起飞高度 `0.8`。它不是规划器参数快照，也不能用于还原当时的膨胀地图。

**当前结论：没有进入轨迹控制是已知事实，地图膨胀只是待验证原因，不是已经确定的根因。**

## 2. 先检查命令与交接过程

多行 shell 命令必须在续行处加反斜杠，反斜杠之后不能再有空格：

```bash
python3 ~/gcs_ws/src/livox_ros_driver2/scripts/takeoff_then_diff_planner.py \
  --execute \
  --map-verified \
  --goal-dx 0.5 \
  --goal-dy 0.0
```

上面会真实请求起飞，仅在安全条件满足、地图经人工确认后使用；不是地图检查命令。
如果每行没有反斜杠而直接分别执行，第一行只是默认只读检查，后面的 `--execute` 等会成为独立命令，不会传给 Python。
如果飞机确实起飞了，还需核对是否有其他起飞入口，不能仅凭粘贴格式推断原因。

顺序脚本必须依次出现：

1. `已发送一次起飞请求`。
2. `已确认AUTO_HOVER，连续稳定...秒`。
3. `已发送一次/goal=...`。
4. `交接确认：收到新的/setpoints_cmd，PX4Ctrl已进入CMD_CTRL(3)`。

缺少第 2 步：检查脚本报错、RC、时间戳、位置/高度/速度稳定条件。
缺少第 3 步：目标还未发送，不应先归咎于规划器不执行目标。
有第 3 步但没有第 4 步：继续查规划器日志、地图、轨迹和心跳。
即使有第 4 步，也只是交接确认，不是到点或自动降落确认。

目标的 `dx/dy` 使用 **world/EKF 坐标轴**，不是机头前方。
`--map-verified` 是人工确认声明，不是自动碰撞检测。
不要放宽稳定阈值、重复发目标或关闭碰撞检查来强行通过交接。

## 3. 地面快速检查地图

新增的只读脚本：

```text
diff_planner/plan_manage/scripts/check_diff_planner_map.py
```

先启动已验证的感知/EKF 流程；飞机保持未解锁，地图检查不要求启动 PX4Ctrl。
再在单独终端启动规划器及轨迹服务器，不启动 multipoint，也不要给 RViz 目标：

```bash
source ~/gcs_ws/devel/setup.bash
roslaunch diff_planner run_exp_single_lio.launch
```

新开终端快速检查：

```bash
source ~/gcs_ws/devel/setup.bash
python3 ~/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/scripts/check_diff_planner_map.py \
  --check-z 0.8 \
  --goal-dx 0.5 \
  --goal-dy 0.0 \
  --output /tmp/diff_planner_map_check.json
```

**`--check-z` 是 world/EKF 中的绝对 z，不是离地高度。**
例如地面静止时 EKF z 为 `0.03`，相对起飞 `0.8 m` 的检查高度应接近 `0.83`，而非机械填写 `0.8`。
默认观察 5 秒，可用 `--seconds 15` 延长；直接使用 Python 路径运行，无需重新编译。

脚本只订阅消息、读取参数和 ROS 图，不发送起飞/目标/控制指令，不调用解锁/模式服务、不改参数。
它会检查：

- `/cloud_registered`、`/ekf/ekf_odom`、原始占据地图、膨胀地图是否收到消息及接收新鲜度。
- 地图参数、推算的有效分辨率与 XYZ 轴向膨胀距离、虚拟高度边界。
- 地图有限点数量、XYZ 范围、检查高度附近的点数量。
- 假设起点/目标附近与已发布膨胀栅格中心的距离，作为疑似占据线索。
- 规划器是否订阅 `/goal`、指令发布者是否只有预期轨迹服务器，以及轨迹、心跳、控制状态。
- 采样窗口中如有 `/goal` 或控制状态变化，会即时打印事件。

返回码：`0` 表示没有检测到 `FAIL`；`1` 表示检查项有 `FAIL`；`2` 表示依赖、连接、参数或执行异常。
`WARN` 必须结合现场核实，返回 `0` **不表示航线安全，也不是实飞许可**。
没有收到 `/goal` 只能说明本观察窗口没有收到；目标不是 latched 消息，不能据此判断之前是否发过。
地图采样不是同步快照，脚本的假设目标也不是起飞脚本保存的真实悬停目标。

2026-10-20 在当前连接的 ROS master 上已运行只读检查，结果保存在 `/tmp/diff_planner_map_check.json`：
规划器参数缺失，采样窗口内没有 EKF、配准点云、地图或轨迹心跳，规划器也没有订阅 `/goal`。
这只描述本次检查时所连接的 master，不代表之前飞行时没有启动规划器。
需要按上述流程启动感知与规划器后重新检查，不能用这个结果认定旧包中的根因。

### 3.1 同时打开专用 RViz

在检查命令末尾增加 `--rviz`：

```bash
source ~/gcs_ws/devel/setup.bash
python3 ~/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/scripts/check_diff_planner_map.py \
  --check-z 0.8 --goal-dx 0.5 --rviz
```

只想开图，不采样也可以：

```bash
rosrun rviz rviz -d ~/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/launch/include/map_check.rviz
```

专用配置固定坐标系为 `world`，只保留观察工具，没有 `2D Nav Goal` 或初始位姿发布工具：

| 显示 | 话题 | 观察内容 |
|---|---|---|
| 灰色点云 | `/cloud_registered` | 原始配准点云，先确认障碍物位置正确 |
| 橙色栅格 | `/drone_0_diff_planner_node/grid_map/occupancy` | 未膨胀的占据地图 |
| 半透明蓝色栅格 | `/drone_0_diff_planner_node/grid_map/occupancy_inflate` | 膨胀后的障碍体积 |
| EKF 箭头 | `/ekf/ekf_odom` | 机体位置与方向 |
| 全局参考/优化轨迹 | 规划节点的 `global_list` / `optimal_list` | 参考路径与优化结果；参考路径不等于已通过碰撞检查的执行轨迹 |

地图实际分辨率改变时，RViz 的 `Size (m)` 也应对应修改，避免显示方块尺寸误导判断。
如果 RViz 报 frame/TF 错误，不要随意发布单位变换让画面“看起来正常”；先验证点云和 EKF 的数值坐标确实一致。

### 3.2 地图验收顺序

1. 在无人机未解锁、无旋转桨叶风险的条件下，观察静止墙面/箱体。
2. 先关闭蓝色膨胀图，确认橙色占据图与灰色点云对应。
3. 打开蓝色图，观察从侧面看地面向上膨胀了多高；俯视检查通道是否被两侧膨胀封死。
4. 在实际计划飞行高度检查起点、目标以及整段可行走廊，不只看地面或某一个目标点。
5. 在安全搬动机体进行地面检查时，确认墙面固定、不跟随机体移动；检查雷达安装外参与坐标。

**本机地图可视化不是完整碰撞查询：**

- 未膨胀地图只发布部分前向区域，膨胀地图也有前向裁剪；后方空白不能解释为没有障碍。
- 发布范围被 `virtual_ground/virtual_ceil` 裁剪；下方原始地面点可能不可见，但其膨胀体仍可能影响飞行层。
- 当前发布代码没有给可视化地图设置非零时间戳，因此脚本主要显示墙钟接收新鲜度；收到新消息也不保证输入坐标正确。
- `grid_map/visualization_truncate_height` 虽出现在 launch 中，但当前 `GridMap` 没有读取它。不要用它控制飞行高度。
- 空点云只说明当前发布区域没有显示占据点，也可能是输入/坐标问题，不构成自由空间证明。

## 4. 0.8m 膨胀与 0.8m 悬停是否冲突

有可能，但不是数值相等就必然冲突。

本机 `changeInfBuf()` 沿 X/Y/Z 三个轴膨胀成立方体，并不是只在 XY 平面扩张，也不是无量纲“系数”。
如果地面被当作占据栅格融合，它会向上膨胀；地面高程、机体/EKF 原点偏移、栅格取整都会影响 `0.8m` 飞行层是否被占据。
反之，若起点与目标周围并未融合这些地面障碍，就不能仅凭高度等于膨胀距离断言被堵。

当前源码：

```text
layers = ceil((inflation_size - 1e-5) / configured_resolution)
如果 layers > 4：
    layers = 4
    effective_resolution = inflation_size / 4
axis_inflation = layers * effective_resolution
```

因此，在配置分辨率为 `0.125m` 时：

| 膨胀距离 | 层数 | 推算有效分辨率 | XYZ轴向膨胀 |
|---|---:|---:|---:|
| `0.8m` | 4 | `0.2m`（代码自动增大） | `0.8m` |
| `0.5m` | 4 | `0.125m` | `0.5m` |

参数服务器可能仍显示配置值 `0.125`，不代表代码内部没有自动增大分辨率。
膨胀变化也可能改变地图分辨率，不应只按“半径减少 0.3m”理解。
`optimization/obstacle_clearance` 和 `obstacle_clearance_soft` 还会影响路径，不能只检查膨胀距离。

你当前文件已经是 `inflation_size=0.5`、`map_resolution=0.125`，本次不再改动。
但 `0.5` 是否安全仍需根据桨盘最大包络、定位/跟踪误差及余量确认；不能为了通过通道继续盲目减小。
修改 launch 后，必须在地面、未解锁且确认安全时重启相应规划器流程才会重新初始化地图。
单纯改文件或 `rosparam set` 不会让已初始化的地图自动更新这些参数；不要在空中重启控制链。

## 5. 如何防止从障碍物上方越过

### 5.1 现有参数能做到：限制规划高度上界

本机已有 `enable_virtual_wall`、`virtual_ceil`、`virtual_ground`。
`GridMap::getInflateOccupancy()` 对 `z >= virtual_ceil` 或 `z <= virtual_ground` 返回 `-1`，搜索/碰撞检查会把这些区域视为不可用。

文件：`diff_planner/plan_manage/launch/exp/run_exp_single_lio.launch`。

**仅举例，不是已替你设置的推荐实飞参数：** 假设实际悬停 world z 约 `0.8`，已验证的目标障碍在膨胀后连续占据到 world z `1.2` 附近，可以评估上界 `1.1`：

```xml
<arg name="enable_virtual_wall" value="true"/>
<arg name="virtual_ground" value="0.1"/>
<arg name="virtual_ceil" value="1.1"/>
```

前提：

- 起点、目标、跟踪误差和上冲余量必须都在可用高度带内，所有 z 均使用同一 world/EKF 原点。
- 起飞脚本要求 `ground + 0.2 < target_z < ceiling - 0.2`，是严格不等式。例如目标 `0.8`、上界 `1.0` 会被拒绝。
- 障碍在对应 XY 位置的膨胀占据区必须确实延伸至天花板，并留栅格与误差余量，才能封住其上方通路；不能只凭障碍名义高度相加认定成立。
- 地面膨胀不能同时封死下方飞行带。

`virtual_ceil=2.0` 允许在 2m 以下搜索三维绕行，并不要求飞机始终在 0.8m。
即使目标 z 相同，中途轨迹也可能升降。
降低上界只能封住满足上述条件的障碍上方通路，**不能保证不越过任意低矮障碍**。

### 5.2 如果要求“所有指定障碍只许侧绕”

需要地图/规划层的明确约束，不能只靠目标高度：

1. 将选定的非地面障碍投影到 XY 平面，再沿 Z 挤出成覆盖整个允许高度带的柱状禁飞区；避免把整片地面也投影成障碍。
2. 或实现固定/受限高度的二维、2.5D 搜索，并同步约束轨迹优化与最终轨迹碰撞检查。
3. 如果还禁止从障碍下方绕行，也必须同时处理下方通道。

本次未修改三维地图/搜索算法，也未静默改变你的高度上界。
不要把 `/setpoints_cmd.position.z` 强行改成常数：这会使位置、速度、加速度和原碰撞检查失配。
`virtual_ceil` 是规划器约束，不是飞控硬限高，不能保证起飞段、手动飞行、跟踪超调或估计错误时实体机体不越界。
需要独立的飞控围栏/安全监护措施，且必须另行校核其参考高度与动作，不应把规划参数当作硬安全保证。

## 6. 下次诊断如何保留证据

先做地面地图检查。确有必要且具备安全条件再飞行时，应在请求起飞之前开始录包，并保留脚本终端输出。

```bash
source ~/gcs_ws/devel/setup.bash
LOGDIR="$HOME/flight_logs/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOGDIR"
rosparam dump "$LOGDIR/px4ctrl_runtime.yaml" /px4ctrl
rosparam dump "$LOGDIR/planner_runtime.yaml" /drone_0_diff_planner_node
rosbag record -O "$LOGDIR/flight.bag" \
  /goal /px4ctrl/takeoff_land /px4ctrl/state \
  /setpoints_cmd /debugPx4ctrl /ekf/ekf_odom \
  /mavros/state /mavros/extended_state /mavros/rc/in \
  /mavros/imu/data /mavros/setpoint_raw/attitude /mavros/battery \
  /drone_0_planning/trajectory /drone_0_traj_server/heartbeat \
  /cloud_registered \
  /drone_0_diff_planner_node/grid_map/occupancy \
  /drone_0_diff_planner_node/grid_map/occupancy_inflate /rosout
```

点云会明显增加磁盘与系统负载，应先在地面确认存储空间及控制/里程计频率不受影响。
只在录包终端按 `Ctrl+C` 结束并等待保存；停止录包不会停止飞机。
分析时优先直接读取 bag，不要把包含起飞/目标/控制消息的 bag 回放到连接实机的 ROS master。

## 7. 对应代码位置

- 起飞与交接：`~/gcs_ws/src/livox_ros_driver2/scripts/takeoff_then_diff_planner.py` 的 `wait_hover()`、`handoff()`。
- 配置入口：`diff_planner/plan_manage/launch/exp/run_exp_single_lio.launch`。
- 参数转发：`diff_planner/plan_manage/launch/include/advanced_param_exp.xml`。
- 膨胀层数与分辨率调整：`diff_planner/plan_env/src/grid_map.cpp` 的 `initMap()`。
- 三轴膨胀与高度边界：`diff_planner/plan_env/include/plan_env/grid_map.h` 的 `changeInfBuf()`、`getInflateOccupancy()`。
- 可视化裁剪：`diff_planner/plan_env/src/grid_map.cpp` 的 `publishMap()`、`publishMapInflate()`。
- 接收目标：`diff_planner/plan_manage/src/diff_replan_fsm.cpp` 的 `waypointCallback()`。
- 心跳与指令输出：`diff_planner/plan_manage/src/traj_server.cpp` 的 `cmdCallback()`。
