# 比赛分段任务：程控起飞 → Planner 经过 B 到 C → 直控 D → Planner E/F → 直控 G/H → Planner I → 悬停后降落

更新：2026-10-28。适用当前 ROS 1 / PX4Ctrl / Diff-Planner 接线。

**当前实际任务流程（以此为准；本文后面描述“B→C 直控 / C→D Planner”的段落属于历史设计，已被取代）：**

```text
A 地面原点 → 自动起飞 → A 上方悬停 3 s
→ WAIT_PLANNER_B → PLANNER_B（经过点：距 B ≤ pass_radius 立即切 C，不悬停）
→ WAIT_PLANNER_C（桥接转发上一段 B 轨迹，不停车）→ PLANNER_C → C 到点稳定 3 s
→ DIRECT → DWELL（D 稳定 3 s）
→ WAIT_PLANNER_E → PLANNER_E → E 稳定 3 s
→ WAIT_PLANNER_F → PLANNER_F → F 稳定 3 s（之后切直控）
→ DIRECT → DWELL（G 稳定 3 s）→ DIRECT → DWELL（H 稳定 3 s）
→ WAIT_PLANNER_I → PLANNER_I → HOLD（I 稳定 3 s）→ LANDING → DONE
```

- `run()` 里的航段表：`B/planner 经过点`、`C/planner`、`D/direct`、`E/planner`、`F/planner`、`G/direct`、`H/direct`、`I/planner`。Planner 与直控可以多次来回切换，每次切回 Planner 都要求更新的 `trajectory_id` 和更新的 `header.stamp`。
- B 与普通 Planner 航点的区别只有“经过点”一条：不做 3 秒到点计时，实测距离进入 `mission/pass_radius` 就发下一个目标；切换等待期间继续转发上一段 Planner 轨迹，避免在 B 附近急停。
- F 是 Planner→直控切换点：到点条件（位置/航向/低速 + Planner 终点零速保持）满足后，再连续稳定 `mission/arrival_dwell` 才切换直控。
- C、D、E、G、H 各自到点悬停 `mission/arrival_dwell`；I 是终点，到点悬停 `mission/hover_seconds` 后请求降落。
- I 的相对坐标是 `[0,0,0.8]`（A 正上方、飞行高度），航向与起飞相反（180°）。不要写 `[0,0,0]`，那是地面点。
- 未实测的航点在 YAML 里写 `measured: false`：预览会警告，`--execute` 会被直接拒绝。
- 操作步骤、检查单和 bag 回放方法见 [混合任务飞行与回放运行手册](HYBRID_MISSION_FLIGHT_AND_BAG_RUNBOOK.md)。

2026-10-27 调整：任务脚本的坐标包围盒、实测/指令坐标越界拦截和虚拟墙范围预检已移除，旧 YAML 的 `min_xyz`、`max_xyz`、`virtual_wall_margin` 会被忽略。下文历史边界描述以此更新为准；Planner 原生地图/虚拟墙及避障没有修改。数值有效性、定位/RC/指令健康检查保留。源码修改不代表已验证实机飞行。
本功能已经过离线单元测试，未进行实机飞行验证。
此前带动力起飞的雷达重置、LIO 断更问题没有被这份任务脚本修复；问题未排除时只做离线检查。

## 1. 文件与任务定义

- 任务脚本：`~/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/scripts/hybrid_mission.py`
- 独立调参：`~/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/config/hybrid_mission.yaml`
- 隔离 Planner 启动文件：`roslaunch diff_planner run_exp_hybrid_mission.launch`
- 默认任务：地面 A=(0,0,0) 未解锁 → PX4Ctrl 自动切 OFFBOARD、解锁并原地垂直起飞 → A 上方相对高度 0.8 m 连续稳定悬停 3 秒 → Planner 到 B（B 为**经过点**：进入 `pass_radius` 即切 C，不悬停）→ Planner 到 C，连续稳定悬停 3 秒 → 关闭 Planner 指令通路 → 位置直控到 D，连续稳定悬停 3 秒 → 重新放行 Planner 到 E，连续稳定悬停 3 秒 → Planner 到 F，连续稳定悬停 3 秒 → 位置直控到 G，连续稳定悬停 3 秒 → 位置直控到 H，连续稳定悬停 3 秒 → 重新放行 Planner 回 I（A 正上方、航向 180°），连续稳定悬停 3 秒 → 请求 PX4Ctrl 降落 → 等待落地并锁桨。

**本版从地面自动起飞，不需要飞手手动起飞。**脚本向 `/px4ctrl/takeoff_land` 只发送一次 TAKEOFF，由现有 PX4Ctrl 完成模式切换、解锁和起飞，不直接调用 MAVROS 解锁服务。
飞手仍需检查场地和遥控器、确认执行并随时准备接管；起飞条件不满足时脚本拒绝执行，不能靠关闭 RC 检查强行运行。
不要在空中为了让检查通过而停止/重启 PX4Ctrl、定位或 Planner。

## 2. 为什么必须用独立 launch

```text
Diff-Planner → traj_server → /drone_0_mission/planner_cmd
                                      ↓
                              hybrid_mission.py
                          Planner 转发 / 位置直控二选一
                                      ↓
                               /setpoints_cmd
                                      ↓
                                   PX4Ctrl
```

正常情况下 `/setpoints_cmd` 只有一个 `/hybrid_mission_随机后缀` 发布者。
不允许原 traj_server 和比赛脚本同时发布该话题，否则不是可靠切换，而是两路指令相互覆盖。
新 launch 复用当前 `run_exp_single_lio.launch`，不另复制地图参数，仅把轨迹服务器输出改为任务私有输入。
原 launch 默认输出仍为 `/setpoints_cmd`，普通单段飞行方式不变。

轨迹服务器新增可选 `traj_server/hold_at_end`：混合任务开启，原 launch 默认关闭。
原实现到轨迹时间末端后停止发布，无法可靠完成 B 点低速/航向的连续确认；
开启后在 Planner 心跳仍健康时持续发布轨迹终点的零速度保持指令，供任务转发并完成航向调整。
不会靠放宽“指令断更”保护来伪装轨迹仍有效。Planner 心跳丢失仍会停止持续输出。

B 是经过点：不要求到点停下，只看实测位置到 B 的距离是否 ≤ `mission/pass_radius`，满足就发送一次 C 点 `/goal`。等待 C 新轨迹期间继续转发上一段 B 的 Planner 指令（“桥接”），避免在 B 附近急停；该指令必须在 `safety/planner_command_timeout` 内保持新鲜，否则按保护降落处理。

到 C 后，必须同时满足实测到点、航向/低速，以及 Planner 参考到达 C 的零速度、零加速度、零 jerk 终点保持，并连续确认 `mission/arrival_dwell`，才进入 DIRECT。此时不再转发 Planner 指令，**不发送 `/mandatory_stop_to_planner`**；Planner 和地图继续运行。D 点位置/航向/低速连续达标后，发送一次 E 点 `/goal`。等待阶段持续保持 D 附近的实测稳定位置；只有目标发送后接收到消息、原始指令时间戳不早于发送时刻，且 `trajectory_id` 大于切换前记录的 ID，才重新放行 Planner。随后仍检查指令新鲜度、坐标系、速度、跟踪误差和安全边界。

同样的规则适用于多次切换：D→E、E→F、H→I 都要重新发 `/goal` 并等待更新的 `trajectory_id`；F→G、G→H 是直控段，不读取 Planner 数据。F 在 Planner 段到达条件满足后，和 C、E 一样按 `mission/arrival_dwell` 连续稳定悬停，再由脚本切到直控。

H→I 重新回到 Planner 后，I 到点连续稳定 `mission/hover_seconds`，再请求一次降落。`trajectory_id` 必须单调递增（已修复 `TrajContainer::setGlobalTraj()` 重置 `traj_id` 导致每次新目标都从 1 开始的问题）；若日志仍报“未收到新轨迹 ID”，先查 Planner 是否真的收到 `/goal` 并产生新轨迹，不要放宽检查。

原先在 B 发送 mandatory stop 的实现仅适用于“切出后直接降落”，不适用于切回 Planner。本仓库 `mandatoryStopCallback()` 会进入 `EMERGENCY_STOP` 并设置 `enable_fail_safe_ = false`；再次发送 `/goal` 不会清除这个锁定。现在只在最终降落或允许的保护降落时发送 mandatory stop；已经锁定时拒绝强行恢复。**每轮任务后应落地锁桨，再重启隔离 Planner**，不要在空中重启来解除急停。

### 2.1 是否使用原生接口

使用原生接口，但比赛任务状态机和独占指令切换是本脚本新增的，不是声称 Diff-Planner 自带完整的 A/B/C/D/E/F/G/H/I 任务模式：

- 规划目标：原生 `/goal`，类型 `geometry_msgs/PoseStamped`，对应 `diff_planner/plan_manage/src/diff_replan_fsm.cpp` 的 `waypointCallback()`。
- 位置直控：原生 PX4Ctrl `~cmd`，类型 `quadrotor_msgs/PositionCommand`，本机 `realflight_modules/px4ctrl/launch/run_ctrl_lio.launch` 将它映射到 `/setpoints_cmd`；`input.cpp` 的 `Command_Data_t::feed()` 直接读取位置、速度、加速度、jerk 和 yaw，不要求来自 Planner。
- 仓库原有 `Utils/manual_take_over` 也是通过停用 Planner 指令后自行发布 `PositionCommand` 接管，但它是摇杆接管示例，不是可恢复的 Planner/直控航点切换服务。
- 当前检查到的 `/goal` 回调没有“不规划直接直控”的开关；`/planning/yaw` 只设置航向，不是位置绕过接口。独立启动文件中的 remap 只隔离输出，避免原轨迹服务器和脚本同时向 PX4Ctrl 发布互相覆盖的指令。

因此 C→D 确实不做 Planner 路径规划，也没有改为 MAVROS 位置控制；仍由原 PX4Ctrl 控制器跟踪脚本生成的平滑 `PositionCommand`。A→B、B→C、D→E 三段则由 Diff-Planner 规划。新轨迹 ID 用于防止误放行切换前持续发布的旧保持指令，不等于原生协议中有带目标请求 ID 的应答服务；运行中不要注入其他 `/goal` 发布者。

## 3. 独立参数文件

只调整 `config/hybrid_mission.yaml`，不用改 Python 代码。

```yaml
frame_id: world
coordinate_mode: takeoff_relative
points:
  A: {xyz: [0.0, 0.0, 0.0], yaw_deg: 0.0}
  B: {xyz: [0.5, 0.0, 0.8], yaw_deg: 0.0}
  C: {xyz: [0.5, 0.5, 0.8], yaw_deg: 90.0}
  D: {xyz: [0.0, 0.0, 0.8], yaw_deg: 90.0}
  E: {xyz: [0.0, 0.0, 0.8], yaw_deg: 90.0, measured: false}
  F: {xyz: [0.0, 0.0, 0.8], yaw_deg: 90.0, measured: false}
  G: {xyz: [0.0, 0.0, 0.8], yaw_deg: 90.0, measured: false}
  H: {xyz: [0.0, 0.0, 0.8], yaw_deg: 90.0, measured: false}
  I: {xyz: [0.0, 0.0, 0.8], yaw_deg: 180.0}
mission:
  pass_radius: 0.30
takeoff:
  height: 0.8
```

- 默认 `takeoff_relative`：**A 就是这一轮地面起飞原点，固定填写 `(0,0,0)`**。脚本在输入 `MISSION` 后的最后一次地面检查中，记录 `/ekf/ekf_odom` 的实际起点 `(x0,y0,z0)`，本轮任务中不再更新这个原点。
- 起飞悬停点为相对 A 的 `(0,0,takeoff.height)`；不是把飞机起飞后的当前位置重新设为零。A.z=0 不会被当成空中位置目标发给 PX4Ctrl。
- B/C/D/E/F/G/H/I 的 `xyz` 单位米，都是**相对同一个地面 A 原点**，不是相对上一点。示例 B 在 A 的 +X 方向 0.5 m、高度 0.8 m；C 在 A 的 +X 0.5 m、+Y 0.5 m；I 回到 A 正上方 0.8 m、航向 180°。以上都是示例，实际以当前 YAML 为准；必须按实际场地修改并核对 I 下方降落区，不能把示例视为已验证的实飞航线。
- 相对坐标仅做平移：`p_world = (x0,y0,z0) + p_relative`。轴方向仍平行 world：+X 不一定是机头前方，+Y 不一定是机体左方；不自动跟随起飞机头旋转。
- `yaw_deg` 用度，绕 +Z 轴按右手方向：0° 朝 world +X，90° 朝 world +Y；不是相对当前机头旋转。
- 起飞前机头朝向须匹配 A 的 yaw，地面速度不超过 0.1 m/s；不会在地面或起飞途中主动横移、转向寻找起点。
- 例如地面实际 world 起点为 `(10,-3,0.034)`，仍填写 A=`(0,0,0)`、B=`(0.5,0,0.8)`；脚本实际给 Planner 的 B 是 world `(10.5,-3,0.834)`，不需要每轮手工修改坐标偏移。
- **不会清零 EKF、重置 MAVROS 或改 TF。** `/goal`、`/setpoints_cmd` 和 Planner 障碍地图始终使用原来的 world 坐标系，只有任务参数相对起点填写。
- 兼容旧参数：显式 `coordinate_mode: world`，或旧文件没有此字段时，仍按原先绝对 world 坐标执行，A 表示空中起飞悬停点而非地面原点；两种模式不要混写。
- Planner 的 `/goal` 回调只读取位置，不保证执行消息中的四元数。任务脚本因此在转发 Planner 轨迹时单独限速控制到各航点的航向，并用实测航向确认到点。
- C→D 使用五次时间插值，发送位置、速度、加速度、jerk、yaw 和 yaw_dot；不是向 D 瞬间跳变一个大位置阶跃。
- C→D、F→G、G→H 不使用 Planner 做避障，必须人工确认整条直控通道安全；A→B、B→C、D→E、E→F、H→I 都要检查地图，最终降落区在 I 下方。C、D、E、G、H 各做 `arrival_dwell` 到点稳定确认，F 只做到点确认即切直控，I 的 3 秒最终悬停由 `hover_seconds` 控制。
- 自定义旧 YAML 必须补充 `points/D`、`points/E` 的 `xyz` 和 `yaw_deg`；E 必须在任务边界和地图虚拟上下边界内。E 未实测时保留 `measured: false`，脚本拒绝执行；不静默猜测目的地。

主要调参项：

| 参数 | 默认 | 含义 |
|---|---:|---|
| `takeoff/height` | 0.80 m | 相对地面起飞高度；必须与 PX4Ctrl 运行参数 `auto_takeoff_land/takeoff_height` 一致，误差不得超过 0.01 m |
| `takeoff/timeout` | 60 s | 起飞及稳定悬停总等待上限 |
| `takeoff/ack_timeout` | 5 s | 等待进入 AUTO_TAKEOFF(4) 的上限；失败不重发起飞 |
| `takeoff/fcu_confirm_timeout` | 2 s | 进入起飞状态后等待 MAVROS 确认 armed+OFFBOARD 的宽限，不在请求刚发出时误判失败 |
| `takeoff/hover_seconds` | 3 s | 到 A 上方起飞悬停点后连续稳定确认，至少 3 秒 |
| `takeoff/corridor_radius` | 0.20 m | 起飞全过程允许偏离地面起点的最大水平距离 |
| `takeoff/max_sink` | 0.10 m | 起飞期间允许低于地面起始 z 的最大偏差；超限要求人工接管 |
| `direct/max_speed` | 0.20 m/s | C→D 参考轨迹最大速度 |
| `direct/max_acceleration` | 0.30 m/s² | C→D 参考轨迹最大加速度 |
| `direct/max_jerk` | 1.0 m/s³ | C→D 参考轨迹 jerk 上限 |
| `direct/max_yaw_rate_deg` | 25 °/s | Planner 段航向限速、直控段最大航向速率 |
| `mission/arrival_dwell` | 3 s | C、D 位置、航向、速度连续满足到点条件的时长 |
| `mission/hover_seconds` | 3 s | E 点连续稳定悬停后降落的时长，不能小于 3 秒 |
| `mission/pass_radius` | 0.30 m | B 作为经过点的切换半径；必须小于起飞悬停点到 B 的距离 |
| `mission/planner_first_command_timeout` | 2 s | 发每个 Planner 目标后等待首条新轨迹指令的上限 |
| `mission/planner_timeout` | 45 s | 单个 Planner 段（B/C/E）总超时 |
| `mission/direct_timeout` | 30 s | C→D 轨迹及到点确认总超时 |
| `safety/position_tolerance` | 0.10 m | 三维到点位置误差 |
| `safety/yaw_tolerance_deg` | 10° | 航向到点误差 |
| `safety/arrival_speed` | 0.20 m/s | 到点时的实测速度上限 |
| `safety/max_measured_speed` | 0.65 m/s | 实测速度上限；标准速度 0.30 m/s 的实测允许值 |
| `safety/max_command_speed` | 0.45 m/s | 本脚本转发的位置指令速度上限 |
| `safety/fcu_timeout` | 3 s | MAVROS 飞控状态及落地状态的本机接收超时；给低频更新留有限宽限，不代表允许离开 OFFBOARD 或断连 |
| `safety/motion_timeout` | 3 s | 未接近目标时无有效位移超时 |
| `safety/max_lio_age` 等估计器检查 | 见 8.1 | 激光里程计延迟、EKF/激光偏差及起飞前估计器检查 |

















































| `safety/motion_distance` | 0.04 m | 认定发生有效位移的三维距离 |
| `safety/max_tracking_error` | 0.60 m | 指令位置与实际位置的最大距离 |
| `safety/min_xyz`, `max_xyz` | 见 YAML | 历史上相对 A 的飞行包围盒；2026-10-27 起任务脚本已不再使用，旧值会被忽略 |

到点条件同时要求位置、航向和速度达标。C、D、E、G、H、I 点若漂出到点条件，稳定悬停计时重新开始。每个 Planner 航段（B、C、E、F、I）分别重新计算首条新轨迹等待时限、总超时和 3 秒无位移看门狗；等待 D→E、E→F、H→I 的新轨迹时，切换前残留的 Planner 指令即使持续发布，也不能当作新目标的结果。
轨迹限速只约束参考指令，不代表实机一定跟得上。参数过慢/旋转过久也可能触发 3 秒无位移保护，应先离线评估，不要直接放宽保护来掩盖控制问题。
Planner 调速和地图膨胀仍使用原 Planner 的 launch/XML；任务 YAML 不会替代它们。
起飞高度实际由 PX4Ctrl 执行；脚本只核对 `takeoff/height`，不会偷偷修改控制器参数。修改高度后须在地面同步核对 PX4Ctrl 配置、重启控制器并检查运行参数，以及 B/C/D/E/F/G/H/I 的相对高度。
Planner 的 `virtual_ground`、`virtual_ceil` 仍是绝对 world 高度，**不会随 A 原点平移**；任务脚本本身不检查这两个边界，必须在 RViz/参数中人工核对 B/C/D/E/F/I 的可达性，不要直接放宽 Planner 边界来“过检查”。
改变 `topics/planner_cmd` 时，也必须同步修改隔离 launch 的输出话题，检查会拒绝不匹配的链路。

## 4. 推荐执行顺序

从 EKF 启动、PX4Ctrl、混合专用 Planner，到先录包、预检、程控起飞、落地验包和隔离 RViz 回放的完整可执行清单，统一见 [混合任务起飞与录包全流程](HYBRID_MISSION_FLIGHT_AND_BAG_RUNBOOK.md)。本节作为简要参考；不要照旧单段任务的 Planner 启动命令混用。

### 4.1 离线预览，无需 ROS

```bash
python3 ~/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/scripts/hybrid_mission.py
```

默认仅检查 YAML 和显示 A/B/C/D/E/F/G/H/I，不连接 ROS，不起飞、不发目标、不发位置指令。航点带 `measured: false` 时打印警告；`--execute` 会被拒绝。
可用 `--config /绝对路径/自己的参数.yaml` 选择参数副本。

### 4.2 地面锁桨时准备运行环境

首次使用或更换源码后，需要在地面编译新增的轨迹末端保持功能；已有配置好的工作区可运行：

```bash
source ~/gcs_ws/devel/setup.bash
cmake --build "$HOME/gcs_ws/build" --target traj_server -- -j2
```

这是编译，不会启动节点。编译成功后再按地面启动顺序启动新节点；不要在空中重启正在运行的轨迹服务器。

感知/定位和 PX4Ctrl 的启动条件参阅 [全流程手册](ROSBAG_RECORD_REPLAY.md)。
本次把其中“启动规划器”的命令替换为以下命令；**不能同时运行两套 Planner launch**：

```bash
source ~/gcs_ws/devel/setup.bash
roslaunch diff_planner run_exp_hybrid_mission.launch
```

其他终端按已经验证的流程启动感知和控制器，常用入口分别是：

```bash
source ~/gcs_ws/devel/setup.bash
bash ~/gcs_ws/src/livox_ros_driver2/scripts/start_fastlio2_ekf.sh
```

```bash
source ~/gcs_ws/devel/setup.bash
roslaunch px4ctrl run_ctrl_lio.launch
```

启动前确认 RC 开关和解锁流程安全，不因本文命令跳过原手册的启动顺序和飞手检查。
不要同时运行 multipoint、旧目标脚本或点击 RViz 的发布目标工具。
旧的 `takeoff_then_diff_planner.py`（包括 `--takeoff-only`）要求轨迹服务器直接发布 `/setpoints_cmd`，**与本次隔离架构不兼容，不要混用或在空中重启节点切换架构**。

### 4.3 起飞前开始录包、看地图

```bash
source ~/gcs_ws/devel/setup.bash
python3 ~/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/scripts/record_diff_planner_debug.py \
  --raw-lidar
```

此命令默认在 `~/flight_logs` 下创建新目录，不依赖其他终端的 `FLIGHT_DIR`。
录包新增覆盖 `/drone_0_mission/*`，包括原始 Planner 指令和任务阶段；已有 `/setpoints_cmd`、目标、地图、IMU 和降落请求仍录制。
`--raw-lidar` 会增加存储量；确认磁盘有空间，并确认录包没有因 1 GB 分片/异常提前结束。

实时看图（不是播放旧包）：

```bash
source ~/gcs_ws/devel/setup.bash
rviz -d "$(rospack find diff_planner)/launch/include/live_obstacle_map.rviz"
```

检查 A→B、B→C、D→E、E→F、H→I 地图、膨胀范围、坐标方向、C→D、F→G、G→H 直控通道和 I 下方降落区。地图消息新鲜不等于证明无障碍。

以上 RViz 命令不依赖 `FLIGHT_DIR`，不发目标或起飞。`Fixed Frame=world`，橙色 `Occupancy Raw` 是未膨胀图，半透明蓝色 `Occupancy Inflated` 是膨胀图。完整的话题表、显示设置、无地图排查命令和隔离 bag 回放入口，见 [全流程手册第 7 节](HYBRID_MISSION_FLIGHT_AND_BAG_RUNBOOK.md#7-终端-5rviz-看实时地图和地面诊断)。

### 4.4 地面未解锁时，只读检查程控起飞条件

将飞机放在本轮地面起飞位置，参数中的 A 保持 `(0,0,0)`，机头朝向 A 的 world yaw；不要手动起飞或提前解锁。实际 EKF 起点的 x/y/z 不需要恰好为零。
要求 PX4Ctrl 为 `MANUAL_CTRL(1)`、MAVROS `armed=false`、`landed_state=1`（ON_GROUND）。如果地面已经是 AUTO_HOVER(2)，也会拒绝起飞，应在锁桨状态按控制器流程恢复 MANUAL，而不是反复发 TAKEOFF 或在空中重启节点。
CH5/CH6 高于 1750、四个摇杆居中，包括油门；这来自本机 PX4Ctrl 解释，不按其他机型照搬。
还要求 PX4Ctrl 的 `auto_takeoff_land/enable=true`、`enable_auto_arm=true`、`no_RC=false`；这些必须由飞手在地面核实，不为通过检查盲改参数。

```bash
source ~/gcs_ws/devel/setup.bash
python3 ~/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/scripts/hybrid_mission.py --check
```

检查节点/话题归属、PX4Ctrl 地面状态、预计起飞悬停点、航向、起飞高度、地图输入、EKF/LIO/IMU 接收是否及时，以及 EKF/IMU 时间戳。LIO `/Odometry` 不再检查原始 `header.stamp`，但仍检查消息未收到或断更。日志显示实际 world 起点和转换后的悬停点、B/C/D/E/F/G/H/I，便于在 RViz 中核对。
只订阅和读取状态，不注册飞行指令发布端，不发布目标、控制位置或降落请求。检查进程退出后再运行执行命令，不要并行启动任务。

### 4.5 执行分段任务

```bash
source ~/gcs_ws/devel/setup.bash
python3 ~/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/scripts/hybrid_mission.py \
  --execute \
  --map-verified \
  --direct-path-verified
```

人工确认后输入 `MISSION`。确认后再次完整检查地面起飞条件，发送一次 TAKEOFF，再等待起飞和稳定悬停，不需要另运行旧的起飞脚本。
只有观察到 AUTO_TAKEOFF(4)、随后 AUTO_HOVER(2)，确认 armed+OFFBOARD、PX4 报告 IN_AIR(2)，且 A 上方起飞悬停点的位置/航向/低速连续满足 3 秒，才发送一次 B 点目标。起飞全过程不向 `/setpoints_cmd` 发布位置指令，不提前给 Planner 目标；起飞请求和 B 点目标都不自动重发。
日志阶段依次为 `TAKEOFF → TAKEOFF_HOVER → WAIT_PLANNER_B → PLANNER_B → WAIT_PLANNER_C → PLANNER_C → DIRECT → DWELL → WAIT_PLANNER_E → PLANNER_E → WAIT_PLANNER_F → PLANNER_F → DIRECT → DWELL → DIRECT → DWELL → WAIT_PLANNER_I → PLANNER_I → HOLD → LANDING → DONE`。每个目标只发送一次，不自动重发；等待新轨迹超时且健康检查仍通过时走原有保护降落，不强行放行旧指令或清除急停。B 的 `PLANNER_B` 可能非常短——经过点不要求悬停；Planner 航点的 3 秒悬停仍在 `PLANNER_x` 阶段内计时，不会单独出现 `DWELL`，`DWELL` 只用于直控到点后的悬停。
`DONE` 必须同时看到 FCU 已锁桨、PX4 报告落地；单纯发送 LAND 不算完成。

## 5. 故障保护及边界

- 起飞未确认、起飞/稳定悬停超时、起飞偏离垂直通道：不发送 B 点目标，不重发 TAKEOFF，要求飞手接管。PX4Ctrl 起飞状态不接受这里的保护 LAND，因此不能承诺起飞异常时脚本会自动降落。
- 允许 FCU 状态更新有短暂确认宽限；未确认 armed+OFFBOARD 时若机体已明显移动，或确认后退出该状态，立即停止任务推进，不继续等到超时。
- Planner 不出新指令、轨迹指令断更/非法、3 秒无有效位移、到点超时等：先停止任务流，只有在定位、RC、OFFBOARD、控制归属仍健康时才请求一次降落。
- EKF 或上游 `/Odometry` 断更、EKF/IMU 时间戳异常、定位/速度明显异常、RC 接管、OFFBOARD 退出、争抢控制：停止继续发位置指令并要求飞手接管，不强切 OFFBOARD，不基于失效定位盲目自动降落。LIO 原始时间戳自 2026-10-29 起重新参与中止判定，但必须持续超限才算（见第 8 节）。
- 发出 LAND 后停止位置指令，不与 PX4Ctrl 自身下降控制竞争；要求在超时内确认 `AUTO_LAND(5)`，继续等待落地锁桨。
- 本机 PX4Ctrl 源码支持在 CMD_CTRL 接受 LAND，但应确认运行二进制与源码一致；若 LAND 没有被确认，不会无限重发。
- `Ctrl+C` 停止任务进程，不保证自动降落；RC 接管、控制器失效保护仍须由飞手监护。任务脚本不调用强制停桨/解锁服务。
- 这些是应用层检查，不是实时飞行控制器或经验证的飞行安全系统，不能证明定位正确、路径无障碍或降落区安全。

## 6. 如何检查这一轮是否真的切换成功

实机连接时仅使用只读命令：

```bash
rostopic info /setpoints_cmd
rostopic echo /drone_0_mission/state
rostopic echo /px4ctrl/state
```

预期 `/setpoints_cmd` 发布者仅一个 `/hybrid_mission_随机后缀`；`DIRECT` 后即使 Planner 还有迟到的轨迹消息，也不会再读取转发。
落地锁桨后按 [全流程手册的隔离回放步骤](ROSBAG_RECORD_REPLAY.md) 回看，严禁把含控制消息的包播放到实机 master。
分析时一起查看原 Planner `/drone_0_mission/planner_cmd`、实际输出 `/setpoints_cmd`、`/ekf/ekf_odom`、任务阶段、`/mandatory_stop_to_planner`、`/px4ctrl/takeoff_land` 和地图。
任务阶段消息还包含 `origin_world`（地面起飞点）、`configured_points`（原始参数）、`points`（实际 world 坐标）和 `takeoff_hover`（实际 world 悬停目标），可以核对相对目标到 world 的平移是否正确。
回放比较控制指令应在隔离 master 中重映射为调试话题，不把它们接到任何飞行控制器。

## 7. 已起飞悬停，却报 fcu 断更、没有发送 B

2026-10-24 的一次终端日志显示：发送 TAKEOFF 后约 7.8 秒，任务以 `fcu 未收到或断更` 退出，且没有出现 `WAIT_PLANNER` 或 `PLANNER`。这表示脚本在等待起飞/稳定悬停期间，未通过 `/mavros/state` 接收新鲜度检查，尚未发送 B 点目标；不能据此判定 Planner 规划失败或坐标转换错误。PX4Ctrl 独立执行起飞，因此脚本退出与飞机仍正常起飞悬停可以同时发生。

这次没有录包，原错误也未打印实际接收间隔，无法确认是低频状态更新触碰阈值、真实 MAVROS/链路中断，还是本机回调调度延迟。旧配置 `safety/fcu_timeout: 1.0` 对接收间隔接近一秒且存在抖动的状态源过于紧；离线测试可复现：每 1.08 秒接收一次状态时，旧阈值中止任务，新阈值可继续起飞悬停确认。这是误判风险的复现，不是对本次实飞原因的证明。

现已将该阈值改为 3.0 秒，并将未收到和断更分开报告。断更日志包含话题、任务阶段、距上次本机接收的秒数、阈值以及最近一次 FCU 的 connected/armed/mode。它记录的是消息接收间隔，不是 EKF 数据年龄。EKF/LIO/IMU 仍使用严格的接收超时检查；EKF/IMU 还检查原始时间戳，LIO 原始时间戳检查已按下节说明取消。收到明确断连、退出 OFFBOARD、锁桨或 RC 接管状态仍按原逻辑停止，不等待这三秒。自定义 YAML 不会自动迁移，请核对自己的 `safety/fcu_timeout`。

先在地面锁桨、只启动定位/控制/隔离 Planner 环境，不执行任务；分别在终端观察至少 20～30 秒：

```bash
source ~/gcs_ws/devel/setup.bash
rostopic hz /mavros/state --window=30
```

```bash
source ~/gcs_ws/devel/setup.bash
rostopic echo /mavros/state
```

```bash
source ~/gcs_ws/devel/setup.bash
python3 ~/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/scripts/hybrid_mission.py --check
```

`rostopic hz` 用于看正常频率与最大间隔；在其等待新消息、怀疑断流时，同时检查 echo 和 MAVROS 终端日志，不只依赖最后一次频率输出。检查 MAVROS 是否重启、connected 是否跳变、是否发生串口/UDP 中断，以及本机是否严重卡顿。`conn/heartbeat_rate` 配置的是 MAVROS 发出的心跳，不能直接当作 `/mavros/state` 的实际接收频率。

若仍出现三秒断更，先排查真实链路/进程/负载问题，不要继续扩大阈值来掩盖故障。只读检查通过不能证明带动力飞行时链路也稳定，后续受控试验仍需飞手监护并建议完整录包。已起飞时不要重新启动此任务补发起飞或强行跳过状态检查；先由飞手接管、落地锁桨，再重新检查。

## 8. LIO 原始时间戳检查调整（2026-10-26）

按使用者要求，混合任务脚本不再比较 LIO `/Odometry` 的 `header.stamp` 与 ROS 当前时间。因此即使该字段为零、过旧或超前，只要消息持续及时接收，也不会仅因此触发 `lio 时间戳过期或超前` 中止。此调整覆盖地面检查、起飞、航段及降落阶段；不是对上游 LIO 时间同步或定位发散问题的修复，也不能据此证明此前一定是误判。

LIO 必须仍收到消息，并通过基于本机 `time.monotonic()` 接收时间的 `safety/telemetry_timeout` 检查，当前配置为 0.30 秒。EKF、IMU、控制器及规划指令原有时间戳检查不变，RC、飞控模式、地图和位置边界保护也不变。`max_header_age` 和 `max_future_stamp` 仍用于其他原有检查，不能删除。

这是 Python 脚本修改，无需重新编译。正在运行的旧进程不会自动更新；必须先由飞手接管、落地锁桨，再重新运行 `--check` 并按正常确认流程启动任务，不能在空中重启任务来补发航点。

### 8.1 重新启用 LIO 延迟检查（2026-10-29，覆盖上一节）

2026-10-29 的 rosbag 显示：起飞前 `laserMapping` 已落后，全程 `/Odometry` 的 `header.stamp` 比 ROS 当前时间晚 0.9～1.2 秒（正常 0.05～0.15 秒，10-28 成功飞行单次采样最大 0.293 秒）。自定义 EKF 随后打印 `delaytime 1069.21, buf_size 100` 与 `odom time out of buffer time range`，回退到缓冲区最旧状态、用约 1 秒前的激光做校正；`pos_diff > 0.3` 只打日志不返回。B 点通过时估计仍准（偏差 1cm），加速后才崩，最终在 B→C 冲出地图边界。上一节的"不参与判定"使脚本在这一故障模式下没有任何反应。

因此重新启用该检查，但与 2026-10-26 取消它的原因（单次抖动误判）区分开：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `safety/max_lio_age` | 0.30 s | `/Odometry` `header.stamp` 落后本机时间的上限；`0` 关闭 |
| `safety/lio_grace` | 1.0 s | 延迟必须**持续**超限这么久才中止，单次尖峰不算 |
| `safety/max_ekf_lio_divergence` | 0.35 m | EKF 与 LIO 水平位置偏差上限；`0` 关闭 |
| `safety/divergence_grace` | 0.5 s | 偏差持续超限多久才中止 |
| `safety/preflight_estimator_seconds` | 2.0 s | 起飞前连续采样这么久，任一项不达标就拒绝起飞 |

按离线数据，10-28 成功飞行的最大单次延迟 0.293 秒低于 0.30 秒阈值，且即便短暂越过也不满足 1 秒宽限；10-29 的 0.9～1.2 秒持续延迟会在约 1 秒内中止。**这是离线时间序列回算，不是实飞验证**；阈值针对本机建图负载设定，换机器或换场景需重新确认正常延迟量级。要恢复 2026-10-26 的行为，把 `max_lio_age` 和 `max_ekf_lio_divergence` 同时设为 `0`。

## 9. 速度链与 PID 调节位置（2026-10-29）

标准任务速度定为 0.30 m/s。这条链上**没有一处**是硬钳位到 0.30 的：每层只设上限或惩罚，真正的速度天花板由 PX4Ctrl 的位置环钳位决定。

| 层 | 位置 | 参数 | 现值 | 性质 |
| --- | --- | --- | --- | --- |
| Planner 优化 | `launch/exp/run_exp_single_lio.launch` | `max_vel` | 0.30 | 软惩罚项，不是硬约束；实测指令 0.37～0.56 m/s |
| Planner 验收 | `launch/include/advanced_param_exp.xml` | `optimization/vel_tolerance` | 0.10 | 验收上限 = `max_vel + vel_tolerance` = 0.40 |
| traj_server | `traj_server.cpp` | 无 | — | 原样转发，不缩放 |
| PX4Ctrl 指令 | `controller.cpp` `cmd_ctrl` | 无 | — | 位置指令不做速度钳位 |
| PX4Ctrl 位置环 | `controller.cpp` `computePIDErrorAcc` | `max_auto_vel` | 0.50 | **真正的速度天花板**，逐轴；2026-10-29 前是硬编码 1.0 |
| PX4Ctrl 手动 | `ctrl_param_fpv.yaml` | `max_manual_vel` | 0.30 | 仅 RC 增稳/悬停模式，不影响自动飞行 |
| 姿态限制 | `ctrl_param_fpv.yaml` | `max_angle` | 20° | 约 3.57 m/s² 最大水平加速度 |
| 任务脚本 | `config/hybrid_mission.yaml` | `max_command_speed` / `max_measured_speed` | 0.45 / 0.65 | 只检查本任务转发指令与实测，不限制 PX4Ctrl |

`vel_tolerance` 原为 1.0，等于把验收上限放到 1.35 m/s，1.57 m/s 的指令也能通过验收；现已改为 0.10。`max_auto_vel` 的钳位是 2026-10-29 事故的直接上限来源：位置误差 0.125 m 就会让单轴速度指令饱和到 1.0 m/s，斜向可达 1.41 m/s，与实测 1.57～1.62 m/s 吻合（估计器发散时误差远大于 0.125 m）。

**PID 在哪调**：`realflight_modules/px4ctrl/config/ctrl_param_fpv.yaml` 的 `gain:` 段，由 `run_ctrl_lio.launch` 用 `<rosparam command="load">` 载入，`PX4CtrlParam.cpp` 的 `config_from_ros_handle()` 读取。

- `Kp0/Kp1/Kp2`（默认 4.0）：位置误差 → 速度指令的增益。**它决定"修正多猛"，不决定速度上限**；位置误差大时靠 `max_auto_vel` 钳位兜底，调小 Kp 只是让误差积累更久，不能替代钳位。
- `Kv0/Kv1/Kv2`（默认 3.5）：速度误差 → 加速度指令的增益。
- `KAngR/KAngP/KAngY`（默认 20/20/4.0）：仅在角速度控制模式下使用，位置环不经过。
- `Kvi*/Kvd*`：当前代码未使用，保持 0。

改增益是改变飞行行为的调参，必须地面确认后由飞手监护小幅试飞，不能只看日志。这些改动（Planner 0.30、`vel_tolerance` 0.10、`max_auto_vel` 0.50）**尚未实飞验证**。
