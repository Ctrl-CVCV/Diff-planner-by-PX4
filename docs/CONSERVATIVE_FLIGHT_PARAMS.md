# 保守飞行参数总表

更新：2026-10-31
适用范围：`~/gcs_ws` 工作区，实机（非仿真）单机混合任务。
基准：原值取自 git 已提交版本；"新建"表示该参数随文件一起新增。

> 本文只记录**为了保守飞行而改动的数值参数**。行为类改动（状态机、故障处理逻辑）集中列在第 7 节，
> 不属于参数但同样影响安全性。参数服务器可能保留已退出节点的值——飞行前以运行节点
> `/rosout` 启动日志和录包的 `runtime_all.yaml` 为准，不要只看文件。

---

## 0. 参数分三层，先分清各自管什么

| 层 | 文件 | 决定什么 |
|---|---|---|
| 规划器 | `plan_manage/launch/exp/run_exp_single_lio.launch`、`plan_manage/launch/include/advanced_param_exp.xml` | 生成什么样的轨迹：速度/加速度/jerk 目标、避障余量、地图精度 |
| 控制器 | `realflight_modules/px4ctrl/config/ctrl_param_fpv.yaml` | 跟踪时的硬上限：倾角、速度误差、起飞/降落速度 |
| 任务脚本 | `diff_planner/plan_manage/config/hybrid_mission.yaml` | 飞不飞、停不停：到点判据、超时、中止条件 |

规划器里的 `max_vel` / `max_acc` / `max_jer` 是**软罚目标**，不是保证值；真正会拒绝轨迹的是
`optimization/vel_tolerance`、`acc_tolerance` 形成的验收线。控制器和任务脚本里的才是硬判据。

---

## 1. 规划器：动力学软约束（生成层）

文件：`src/Diff-Planner/src/diff_planner/plan_manage/launch/exp/run_exp_single_lio.launch`

| 参数 | 当前值 | 原值 | 行 | 含义 | 保守性作用 |
|---|---|---|---|---|---|
| `max_vel` | 0.24 m/s | 0.5 | 29 | 速度罚的目标点（`feasibilityGradCostV` 三次方软罚从超过它开始生效） | 值小 → 时间分配更宽 → 轨迹更缓。**实测峰值仍会比它高约 0.2 m/s**，不能当上限用 |
| `max_acc` | 0.6 m/s² | 3.0 | 30 | 加速度软罚目标 | 限制优化器在拐角处可用的加速度，转弯更软 |
| `max_jer` | 5.0 m/s³ | 20.0 | 31 | jerk 软罚目标 | 降低冲击变化率，对挂载载荷有利 |

> 这三个值通过 `<arg>` 同时喂给 `manager/*` 和 `optimization/*`，见
> `advanced_param_exp.xml:134-136, 156, 163, 165`。

## 2. 规划器：代价函数、验收容差与避障余量

文件：`src/Diff-Planner/src/diff_planner/plan_manage/launch/include/advanced_param_exp.xml`

| 参数 | 当前值 | 原值 | 行 | 含义 | 保守性作用 |
|---|---|---|---|---|---|
| `optimization/weight_obstacle` | 12000 | 10000 | 144 | 硬障碍代价权重（轨迹进入障碍区的罚） | 调高，更不愿意贴近障碍 |
| `optimization/weight_obstacle_soft` | 6000 | 5000 | 145 | 软避障代价权重（两段轨迹之间的期望间隙） | 调高，更愿意绕远保持间隙 |
| `optimization/weight_time` | 5.0 | 10.0 | 149 | 时间代价权重：`cost = Σ(1/Tᵢ) × weight_time` | **调低 = 允许更长的段时长 = 峰值速度更低**。2026-10-31 改，尚无实测峰值 |
| `optimization/obstacle_clearance` | 0.15 m | 0.1 | 153 | 硬避障要求的最小间隙 | 调高，轨迹与障碍保持更远 |
| `optimization/obstacle_clearance_soft` | 0.4 m | 0.5 | 154 | 软避障期望间隙（软罚开始起作用的距离） | 与硬间隙 0.15 一起定义"期望安全距离带"；单独看它调低**不是**更保守，实际收紧来自硬间隙从 0.1 提到 0.15 |
| `optimization/vel_tolerance` | 0.30 | 1.0 | 157 | **验收容差**：采样速度 > `max_vel + 此值` 即判不可行、拒绝该轨迹 | 从 1.0 收紧到 0.30，验收线 = 0.24+0.30 = **0.54 m/s** |
| `optimization/acc_tolerance` | 0.2 | 1.0 | 164 | 同上，针对加速度：> `max_acc + 此值` 判不可行 | 验收线 = 0.6+0.2 = **0.8 m/s²** |
| `optimization/weight_feasibility` | 10000 | 10000 | 147 | 可行性罚权重（未改，列此说明） | 与 `weight_time` 共同决定实际均衡点 |

> 注意：`vel_tolerance` 是验收线，不是速度上限。实测在 `weight_time=10` 时优化器输出峰值
> 0.45–0.49，几乎不随 `max_vel` 变；`weight_time` 改为 5.0 后峰值预计下降，但**没有实测数据**，
> 在拿到新峰值前**不要**下调这个容差。

## 3. 地图与虚拟围栏

文件：`src/Diff-Planner/src/diff_planner/plan_manage/launch/exp/run_exp_single_lio.launch`

| 参数 | 当前值 | 原值 | 行 | 含义 | 保守性作用 |
|---|---|---|---|---|---|
| `virtual_ceil` | 1.5 m | 2.0 | 39 | 虚拟天花板高度，地图在此以上视为障碍 | 压低下限，防止飞高越过感知范围 |
| `map_resolution` | 0.075 m | 0.2 | 41 | 占据栅格分辨率 | 变细，障碍边界更精确，但更耗 CPU |
| `inflation_size` | 0.3 m | 0.4 | 42 | 障碍膨胀半径 | 配合更细的栅格调小；实际安全间距由 `obstacle_clearance` 保证 |
| `virtual_ground` | 0.1 m | 0.1 | 40 | 虚拟地面高度（未改，列此说明） | 低于此高度视为障碍 |
| `enable_virtual_wall` | true | true | 38 | 启用虚拟墙（未改） | 限制在局部地图内 |
| `local_map_size_x/y/z` | 5.5 / 5.5 / 2.0 m | 同 | 35-37 | 局部地图尺寸（未改） | 与 `max_ray_length` 匹配 |

## 4. PX4Ctrl：跟踪硬上限

文件：`src/realflight_modules/px4ctrl/config/ctrl_param_fpv.yaml`

| 参数 | 当前值 | 原值 | 行 | 含义 | 保守性作用 |
|---|---|---|---|---|---|
| `max_angle` | 20° | 80° | 11 | 姿态倾角上限；超过时按 `g/cos(20°)` 限制总加速度方向 | 侧向加速度上限约 `g·tan20° ≈ 3.6 m/s²`；大幅抑制急转和猛加速 |
| `max_auto_vel` | 0.50 m/s | 无（硬编码 1.0） | 10 | 自动飞行位置环**每轴速度误差**钳位：`des.v + Kp·位置误差 − odom.v` 被夹到 ±0.50 | 防止定位误差经 Kp 放大成大速度指令（2026-10-29 B→C 跑飞后新增） |
| `max_manual_vel` | 0.35 m/s | 1.0 | 9 | 手动（RC）模式速度上限 | 降低人工误操作的速度 |
| `takeoff_land_speed` | 0.15 m/s | 0.2 | 25 | 自动起飞/降落升降速度 | 起降更慢更稳 |
| `mass` | 2.35 kg | 1.0 | 5 | 机体质量 | **不是保守参数**，是实际配重标定值；填错直接影响推力估计 |
| `hover_percentage` | 0.430 | 0.45 | 36 | 悬停油门百分比（近似推力模型初值） | 同上，属于标定值 |

> `max_auto_vel` 的实现见 `src/realflight_modules/px4ctrl/src/controller.cpp:696-720`
> （`computePIDErrorAcc`）。参数**缺失**时 `read_essential_param` 会 `ROS_BREAK` 让节点退出；
> 但填成 **0 或负数**时只打一条 `ROS_ERROR` 就继续运行，而 `v_limit≤0` 会让钳位失效
> （0 时位置修正被完全夹掉，负数时速度误差恒为正），节点并不会真的"拒绝自动飞行"。
> 所以加载 yaml 后必须 `rosparam get /px4ctrl/max_auto_vel` 确认值，不能只信启动日志。

## 5. 任务脚本：`hybrid_mission.yaml`（新建文件，全部为保守飞行设计）

文件：`src/Diff-Planner/src/diff_planner/plan_manage/config/hybrid_mission.yaml`

### 5.1 起飞（第 21-28 行）

| 参数 | 值 | 行 | 含义 |
|---|---|---|---|
| `height` | 0.80 m | 22 | 起飞目标高度（相对起飞原点） |
| `timeout` | 60.0 s | 23 | 起飞总超时 |
| `ack_timeout` | 5.0 s | 24 | 发 takeoff 指令后等 `takeoff_land` 话题确认的时间 |
| `fcu_confirm_timeout` | 2.0 s | 25 | 等飞控进入 OFFBOARD/解锁状态的时间 |
| `hover_seconds` | 3.0 s | 26 | 到高后稳定悬停多久才继续 |
| `corridor_radius` | 0.20 m | 27 | 起飞过程中允许的水平偏移半径，超出即中止 |
| `max_sink` | 0.10 m | 28 | 起飞过程中允许的最大下沉量，超出即中止 |

### 5.2 直控段（第 30-35 行，C→D、F→G、G→H）

| 参数 | 值 | 行 | 含义 |
|---|---|---|---|
| `max_speed` | 0.20 m/s | 31 | 直控五次曲线速度上限（峰值） |
| `max_acceleration` | 0.30 m/s² | 32 | 直控加速度上限 |
| `max_jerk` | 1.0 m/s³ | 33 | 直控 jerk 上限 |
| `min_duration` | 3.0 s | 34 | 每段直控的最短时长（距离越短降速越多） |
| `max_yaw_rate_deg` | 25 °/s | 35 | 直控段偏航角速度上限 |

### 5.3 降落（第 40-50 行）

| 参数 | 值 | 行 | 含义 |
|---|---|---|---|
| `approach_height` | 0.0 m（关闭） | 43 | 可选进近：>0 时先降到此高度再发 LAND |
| `approach_speed` | 0.15 m/s | 44 | 进近下降速度 |
| `approach_tolerance` | 0.02 m | 45 | 进近到位判据 |
| `approach_timeout` | 20.0 s | 46 | 进近超时 |
| `target_tolerance` | 0.03 m | 47 | 发 LAND 前对落点的位置收紧判据（比 `position_tolerance` 更严） |
| `target_speed` | 0.05 m/s | 48 | 发 LAND 前的速度判据 |
| `target_timeout` | 8.0 s | 50 | 收不紧时按当前位姿降落的兜底时间 |
| `on_ground_height` | 0.20 m | 55 | 实测离地高度 ≤ 该值即视为已落地：PX4Ctrl 交还飞手或降落超时时据此收尾并提示杀桨（2026-10-31 新增） |

> PX4Ctrl 收到 LAND 时会把**当时实测位姿**冻结成降落参考，所以发 LAND 前的位置/速度判据才是决定落点的关键。

### 5.4 任务时序（第 80-97 行）

| 参数 | 值 | 行 | 含义 |
|---|---|---|---|
| `publish_hz` | 50 Hz | 81 | 位置指令发布频率 |
| `startup_timeout` | 10.0 s | 82 | 起步前等待各话题就绪的时间 |
| `planner_first_command_timeout` | 2.0 s | 83 | 发目标后等 Planner 第一条有效指令的时间 |
| `planner_timeout` | 45.0 s | 84 | 规划段总超时 |
| `direct_timeout` | 30.0 s | 85 | 直控段总超时 |
| `arrival_dwell` | 3.0 s | 93 | B、C、D、E、F、G 到点后连续稳定悬停多久再发下一段（2026-10-31 按要求改为 3 s）；**B 点靠它从静止规划 B→C**，避免 139° 折返带速度进弯 |
| `hover_seconds` | 3.0 s | 89 | 通用悬停确认时长 |
| `pass_radius` | 0.30 m | 90 | `pass_through` 航点的切换半径（仅 H 使用） |
| `landing_ack_timeout` | 2.0 s | 91 | 发 LAND 后等飞控确认 |
| `landing_timeout` | 45.0 s | 100 | 发 LAND 后等落地锁桨的总超时（2026-10-31 由 25 s 放宽：PX4Ctrl 自身落地判据约 23 s + 等 PX4 确认 5 s） |
| `stalled_command_timeout` | 2.0 s | 97 | Planner 参考位置在该时长内前进不足 `motion_distance` 即判"参考停滞"并中止 |

### 5.5 安全判据（第 99-130 行，最重要的中止条件）

| 参数 | 值 | 行 | 含义 |
|---|---|---|---|
| `position_tolerance` | 0.10 m | 100 | 到点位置容差（也是多项保护的判据基准） |
| `yaw_tolerance_deg` | 10° | 101 | 到点偏航容差 |
| `arrival_speed` | 0.20 m/s | 102 | 判定"已到点"的速度上限；高于它即使位置到了也不算到 |
| `telemetry_timeout` | 0.30 s | 103 | 高频遥测（位姿/控制状态）断更判据 |
| `lio_timeout` | 1.0 s | 106 | `/Odometry` 断更判据（10 Hz 话题，**不能套用 0.30 s**，2026-10-29 曾因此误判中止） |
| `fcu_timeout` | 3.0 s | 107 | 飞控状态断更判据 |
| `rc_timeout` | 0.50 s | 108 | 遥控输入断更判据 |
| `map_timeout` | 2.0 s | 109 | 障碍地图断更判据 |
| `planner_command_timeout` | 0.25 s | 110 | Planner 指令断更判据；也是"任务循环不能有阻塞调用"的根源（舵机调用因此走后台线程） |
| `max_header_age` | 0.30 s | 111 | 消息 header 时间戳允许的最大滞后 |
| `max_future_stamp` | 0.05 s | 112 | 允许的时间戳超前量 |
| `motion_timeout` | 3.0 s | 113 | 运动看门狗：锚点位移不足 `motion_distance` 的持续时间上限 |
| `motion_distance` | 0.04 m | 114 | 运动看门狗位移阈值 |
| `max_tracking_error` | 0.60 m | 115 | 实测与参考位置的最大允许偏差，超出即中止 |
| `max_measured_speed` | 0.65 m/s | 119 | 实测速度上限，超出判定位异常（PX4Ctrl 反馈钳位 0.50，留了余量） |
| `max_command_speed` | 0.60 m/s | 120 | 下发指令速度上限；必须高于 Planner 验收线 0.54，否则合格轨迹被任务脚本拦下 |
| `max_command_acceleration` | 2.0 m/s² | 121 | 下发指令加速度上限 |
| `max_command_jerk` | 10.0 m/s³ | 122 | 下发指令 jerk 上限 |
| `max_ekf_lio_divergence` | 0.35 m | 126 | EKF 与 LIO 位置允许的最大偏差；填 0 关闭 |
| `divergence_grace` | 0.5 s | 127 | 发散判据的持续时间/宽限 |
| `preflight_estimator_seconds` | 2.0 s | 128 | 起飞前观察估计器稳定性的时长 |
| `max_distance_from_origin` | 0.0 m（关闭） | 130 | 距起飞原点的水平围栏半径；启用前确认所有航点都在圈内 |

### 5.6 危险品投放（第 63-83 行，2026-10-31 起默认开启）

| 参数 | 值 | 行 | 含义 |
|---|---|---|---|
| `enabled` | true | 64 | 总开关；也可用 `--hazard-drop` / `--no-hazard-drop` 覆盖 |
| `points` | [C, D] | 65 | 允许投放的航点 |
| `label` | 危险品 | 66 | 标签子串匹配（YOLO 里叫"危险品标识"） |
| `min_confidence` | 0.70 | 67 | 单帧最低置信度 |
| `vote_frames` | 15 | 72 | 投票窗口帧数 |
| `vote_ratio` | 0.60 | 73 | 窗口内命中比例阈值（15 帧至少 9 帧） |
| `hover_seconds` | 3.0 s | 74 | 到点后稳定悬停多久才允许下降 |
| `wait_timeout` | 10.0 s | 75 | 最多等检测多久；超时只告警并继续任务（C/D 的悬停因此可长于 3 s） |
| `height` | 0.15 m | 76 | 下降到离地高度 |
| `speed` | 0.15 m/s | 77 | 下降/爬升速度上限 |
| `tolerance` | 0.02 m | 78 | 高度到位判据 |
| `timeout` | 20.0 s | 79 | 单次下降/爬升超时；爬升超时按保护降落处理 |
| `push_wait` | 2.5 s | 80 | 推舵机后保持低空的时间，让载荷脱离；舵机 `hold_s=0`，推出后不自动收回 |
| `max_age` | 0.5 s | 81 | 检测消息过期判据 |
| `once` | true | 82 | 全程最多投放一次 |

> 启用时预检要求 `/servo/push` 服务可发现，否则**拒绝起飞**。

## 6. traj_server

文件：`src/Diff-Planner/src/diff_planner/plan_manage/launch/exp/run_exp_single_lio.launch`

| 参数 | 值 | 行 | 含义 |
|---|---|---|---|
| `traj_server/hold_at_end` | false / true | 3, 75 | 轨迹结束后是否继续以零速度发布终点。`run_exp_hybrid_mission.launch` 传 `true`，让 PX4Ctrl 保持 CMD_CTRL，而不是因指令断更切到 AUTO_HOVER 冻结当前位置 |

## 7. 非参数的保守性行为改动（不涉及数值，但影响安全）

| 文件 | 改动 |
|---|---|
| `realflight_modules/px4ctrl/src/PX4CtrlFSM.cpp` | 起飞前 OFFBOARD/解锁失败则**中止起飞**并退出 OFFBOARD，不再进入 AUTO_TAKEOFF；`CMD_CTRL`/`AUTO_HOVER` 收到 LAND 时直接转 AUTO_LAND（原先在 CMD_CTRL 下拒绝）；推力模型只在已解锁时估计 |
| `realflight_modules/px4ctrl/src/input.cpp` | 自动起飞要求 RC 四通道（含油门）都在中位死区内，且通道数 ≥4 |
| `realflight_modules/px4ctrl/src/controller.cpp` | 速度误差钳位由硬编码 1.0 改为参数 `max_auto_vel` |
| `diff_planner/plan_manage/src/traj_server.cpp` | 轨迹结束后先发布末点再 `return`，不再继续走进后续分支；新增 `hold_at_end` |
| `diff_planner/traj_utils/include/traj_utils/plan_container.hpp` | 新目标不再重置 `traj_id=0`，改为单调递增，避免下游把新轨迹当成旧指令 |
| `diff_planner/plan_manage/scripts/hybrid_mission.py` | 大角度折返点先驻留再从静止规划（B）；舵机调用走后台线程，避免 0.25 s 循环停顿保护中止；规划中等待下一段时继续转发上一条指令（bridge） |

## 8. 未验证项与调参注意事项

1. **`weight_time=5.0` 没有实飞数据**。它是本轮唯一没有实测支撑的改动；第一次飞行后应从
   `/drone_0_planning/data_display` 或 bag 里量峰值速度，再决定要不要继续压。
2. **`vel_tolerance` 不要先动**。它只是验收线；调大只是让不合格轨迹放行，飞行峰值不会变。
3. **控制层与任务层的速度关系**：Planner 验收线 0.54 < 任务指令上限 0.60 < 实测上限 0.65；
   PX4Ctrl 的 `max_auto_vel=0.50` 限制的是速度**误差**，不是前馈速度本身，三者不冲突但也不要单独改其中一个。
4. **时长会变长**。`weight_time` 调低后每段更慢，`planner_timeout`(45 s)、`motion_timeout`(3 s)、
   `stalled_command_timeout`(2 s) 的余量需要在下一次飞行时确认。
5. **`mass` 与 `hover_percentage` 是标定值，不是安全余量**。换载荷必须重新标定，不能当调参手段。
6. **`max_angle=20°` 是强限制**。它把侧向加速度限制在约 3.6 m/s²，遇到需要更快横向修正的扰动时
   可能表现为"跟不上"，此时先查估计器而不是放宽倾角。

## 9. 飞行前核对当前生效值

```bash
# 规划器（注意：以运行节点的启动日志为准）
rosparam get /drone_0_diff_planner_node/optimization/weight_time
rosparam get /drone_0_diff_planner_node/optimization/vel_tolerance
rosparam get /drone_0_diff_planner_node/manager/max_vel

# 控制器
rosparam get /px4ctrl/max_angle
rosparam get /px4ctrl/max_auto_vel

# 任务脚本（每次 --execute 读取的是命令行指定的 yaml）
python3 ~/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/scripts/hybrid_mission.py \
  --config ~/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/config/hybrid_mission.yaml --check
```
