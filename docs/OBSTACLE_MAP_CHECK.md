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

### 3.1 实时查看当前障碍地图（不是 rosbag 回放）

只看地图时，在地面未解锁状态启动前述感知/EKF 和规划器即可，**不需要启动 PX4Ctrl、起飞脚本或发送航点**。
本节显示当前节点发布的地图；历史包请按[混合任务全流程手册](HYBRID_MISSION_FLIGHT_AND_BAG_RUNBOOK.md)第 12 节隔离回放。

#### 第一步：确认连接实机 master

新开终端，避免沿用回放终端的 `11321` 环境：

```bash
source ~/gcs_ws/devel/setup.bash
printf 'ROS_MASTER_URI=%s\n' "${ROS_MASTER_URI:-http://localhost:11311}"
rosnode list
rosparam get /use_sim_time
```

实机应连接实际运行感知和规划器的 master。若实机使用本机默认 master、而当前终端残留回放地址，
先把本终端 `ROS_MASTER_URI` 改回 `http://127.0.0.1:11311`；跨机系统使用实际主机地址，
不要随意修改 `ROS_IP/ROS_HOSTNAME`。只剩 `/rosout` 不能说明感知/规划器已经启动。
`/use_sim_time` 未设置或为 `false` 才是本流程正常的实机时间环境；若为 `true`，先排查误连回放 master。
**不要在飞行过程中为了看图修改时间参数，也不要在实机 master 执行 `rosbag play`。**

#### 第二步：一条命令打开实时地图

```bash
source ~/gcs_ws/devel/setup.bash
roslaunch diff_planner view_obstacle_map.launch
```

该 launch 同时启动只读的 `flight_path.py`（把 `/ekf/ekf_odom` 累积成 `/ekf/flight_path` 实测轨迹线，把 `/setpoints_cmd` 累积成 `/commanded_path` 指令轨迹线）和 RViz；第一次使用先 `cd ~/gcs_ws && catkin_make -j2` 安装脚本。只想开 RViz 就改用 `rviz -d "$(rospack find diff_planner)/launch/include/live_obstacle_map.rviz"`，此时轨迹线没有数据。两条线都从 `flight_path` 节点启动时开始累积、一直画到降落，不是只显示当前一段。此配置 `Fixed Frame=world`，只有观察/测量工具，没有 `2D Nav Goal` 或初始位姿发布工具，不会发送起飞、航点或控制消息。

| 显示名称 / 颜色 | 话题 | 含义 |
|---|---|---|
| Occupancy Raw / 橙色 | `/drone_0_diff_planner_node/grid_map/occupancy` | 未膨胀的占据栅格中心，不是雷达原始点云 |
| Occupancy Inflated / 半透明蓝色 | `/drone_0_diff_planner_node/grid_map/occupancy_inflate` | 膨胀后的占据栅格中心 |
| Flight Path (ekf_odom) / 黄色线条 | `/ekf/flight_path` | 实际飞行轨迹（EKF 里程计累积）；不再用箭头表示里程计 |
| Commanded Path (setpoints_cmd) / 绿色线条 | `/commanded_path` | 指令轨迹：Planner 段与直控段的位置指令累积，从节点启动到降落 |
| Registered Cloud Optional TF Required / 灰色，默认关闭 | `/cloud_registered` | 配准点云；需要真实有效的坐标变换才能叠加 |

先取消勾选蓝色图查看橙色障碍边界，再打开蓝色图观察膨胀范围；从俯视和侧视分别检查通道、地面与飞行高度。
地图使用 `Style=Points`、像素尺寸，`Decay Time=0` 避免主动累积旧地图；屏幕点大小不是实际体素尺寸，
接收停止时 RViz 仍可能保留最后一帧，因此还要检查消息频率。若地图在视野外，围绕 EKF 位置调整视角，
或修改 `Views → Current → Focal Point` 到 EKF 的实际 XYZ。

若手动配置已有 RViz：设置 `Global Options → Fixed Frame=world`，
两次 `Add → By display type → PointCloud2`，分别填入上述 RAW / INFLATE 话题，
`Color Transformer=FlatColor`、`Style=Points`，并设置不同颜色。不要添加 `OccupancyGrid/Map` 显示，
因为这里发布的是 `sensor_msgs/PointCloud2`，不是二维 `nav_msgs/OccupancyGrid`。

#### 第三步：确认消息与实际运行参数

RViz 保持打开，在另一个连接同一实机 master 的终端执行：

```bash
source ~/gcs_ws/devel/setup.bash
rostopic info /drone_0_diff_planner_node/grid_map/occupancy
rostopic info /drone_0_diff_planner_node/grid_map/occupancy_inflate
rostopic echo -n 1 /drone_0_diff_planner_node/grid_map/occupancy/header
rostopic echo -n 1 /drone_0_diff_planner_node/grid_map/occupancy_inflate/header
rosparam get /drone_0_diff_planner_node/grid_map/resolution
rosparam get /drone_0_diff_planner_node/grid_map/obstacles_inflation
rosparam get /drone_0_diff_planner_node/grid_map/virtual_ground
rosparam get /drone_0_diff_planner_node/grid_map/virtual_ceil
```

再分别观察两路频率，每条命令运行几秒后 `Ctrl+C`，再执行下一条：

```bash
rostopic hz /drone_0_diff_planner_node/grid_map/occupancy
rostopic hz /drone_0_diff_planner_node/grid_map/occupancy_inflate
```

以上 `echo/hz` 会等待消息，无输出时可 `Ctrl+C` 退出。地图发布函数在没有订阅者时会跳过发布，
因此先开 RViz 或录包器再检查。参数服务器可能残留旧值，必须结合节点/发布者和消息一起判断；
改 launch 不会自动改变已运行节点。点云显示自动使用实际发布的栅格中心，配置没有硬编码膨胀距离或米制方块大小。

#### 看不到地图时按这个顺序检查

1. `rostopic info` 没有发布者：检查 master 地址、规划器是否启动及节点实际命名；有话题名不等于有数据。
2. 有发布者但持续收不到消息：检查规划器终端、点云/里程计输入和地图初始化；不要只靠残留参数判断正常。
3. 有消息但画面空：检查坐标/视角/高度范围，再查看一次点云的 `width`、`height`。
   `rostopic echo -n 1 /drone_0_diff_planner_node/grid_map/occupancy/width` 可检查点数宽度；
   `width=0` 是收到空图，不等于没订阅成功，也不证明空间安全。INFLATE 同理检查。
4. 报 `No transform`：查看对应消息的 `header.frame_id`。当前地图预期为 `world`，
   `/cloud_registered` 通常为 `camera_init`；两者缺少真实 TF 时，关闭灰色 Registered Cloud，先看 world 地图和 EKF。
   要单独看配准点云，可另开一个 RViz，设 `Fixed Frame=camera_init`、只显示 `/cloud_registered`；
   这不代表它已与 world 地图正确叠加。不要发布虚假的单位 TF 或仅重命名 frame 掩盖问题。
5. 后方或某些高度没有点：当前可视化有前向和虚拟高度裁剪，并不是完整内部碰撞地图。
   看图正常、没有自体点或某处空白，都不能代替坐标链验证及完整路径碰撞检查。

### 3.1.1 同时运行只读地图检查器

在检查命令末尾增加 `--rviz`：

```bash
source ~/gcs_ws/devel/setup.bash
python3 ~/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/scripts/check_diff_planner_map.py \
  --check-z 0.8 --goal-dx 0.5 --rviz
```

检查器的 `--rviz` 仍打开旧 `map_check.rviz`；只想实时看图时优先使用上一节的新配置。
旧配置也可以单独打开：

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

旧配置使用 Boxes，地图实际分辨率改变时，RViz 的 `Size (m)` 也应对应修改，避免显示方块尺寸误导判断。
新 `live_obstacle_map.rviz` 使用 Points，没有这个米制方块尺寸问题。
如果 RViz 报 frame/TF 错误，不要随意发布单位变换让画面“看起来正常”；先验证点云和 EKF 的数值坐标确实一致。

### 3.2 地图验收顺序

1. 在无人机未解锁、无旋转桨叶风险的条件下，观察静止墙面/箱体。
2. 先关闭蓝色膨胀图，确认橙色占据图与现场静态障碍对应；只有 TF 验证通过后才叠加灰色配准点云比较。
3. 打开蓝色图，观察从侧面看地面向上膨胀了多高；俯视检查通道是否被两侧膨胀封死。
4. 在实际计划飞行高度检查起点、目标以及整段可行走廊，不只看地面或某一个目标点。
5. 在安全搬动机体进行地面检查时，确认墙面固定、不跟随机体移动；检查雷达安装外参与坐标。

**本机地图可视化不是完整碰撞查询：**

- 未膨胀地图只发布部分前向区域，膨胀地图也有前向裁剪；后方空白不能解释为没有障碍。
- 发布范围被 `virtual_ground/virtual_ceil` 裁剪；下方原始地面点可能不可见，但其膨胀体仍可能影响飞行层。
- 本次源码已补地图发布时刻的时间戳，重编译后的节点需在地面安全重启才生效；它不是原始雷达采样时刻，也不证明输入地图及时更新或坐标正确。
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

从 EKF 启动、RC/控制器准备、录包、任务执行到隔离 master 下 RViz 回放的完整步骤，见
[混合任务全流程手册](HYBRID_MISSION_FLIGHT_AND_BAG_RUNBOOK.md)。
2026-10-20 21:45 这次已发航点但仍悬停的证据与结论，见
[FLIGHT_ANALYSIS_20261020_2145.md](FLIGHT_ANALYSIS_20261020_2145.md)：
规划器连续碰撞判定失败后输出约 2 秒的原地指令；现有包缺失地图，不能断言具体占据来源。

先做地面地图检查。确有必要且具备安全条件再飞行时，应在请求起飞之前开始录包，并保留脚本终端输出。

```bash
source ~/gcs_ws/devel/setup.bash
python3 ~/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/scripts/record_diff_planner_debug.py \
  --raw-lidar
```

点云会明显增加磁盘与系统负载，应先在地面确认存储空间及控制/里程计频率不受影响。
录包器默认创建新目录，完整覆盖两种地图、world/body点云、LIO/EKF、TF、规划与控制状态；每5秒报告地图是否收到，结束后写入计数验收。
缺少地图时应先地面排查，不是“空旷所以没有地图消息”。只在录包终端按 `Ctrl+C` 结束并等待保存；停止录包不会停止飞机。
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
