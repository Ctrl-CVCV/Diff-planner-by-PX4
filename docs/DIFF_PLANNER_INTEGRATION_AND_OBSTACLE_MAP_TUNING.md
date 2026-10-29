# Diff-Planner 接入、障碍地图测试与膨胀参数调试报告

> 历史部署/调参参考，保留当时配置，不代表当前参数。当前实机操作顺序、录包和回放以 [全流程手册](ROSBAG_RECORD_REPLAY.md) 为准；不要同时运行旧启动/录包命令。

- **日期：** 2026-10-19
- **平台：** PX4 + MAVROS + FAST-LIO2 + EKF + PX4Ctrl + Diff-Planner
- **机架：** 四旋翼，标称轴距 450 mm，8 英寸桨
- **当前进度：** 已完成 FAST-LIO2、EKF、PX4Ctrl 部署，并实现定点起飞与悬停
- **本文目标：** 在不破坏现有稳定悬停参数的前提下，接入 Diff-Planner，验证障碍地图，逐步调试膨胀半径并完成低速避障测试

> **安全原则：** 地图、坐标系和轨迹必须先在“拆桨/不解锁”状态验证。首次实飞应在空旷场地、低高度、低速度下进行，并安排一人专门控制模式切换与紧急停机。

---

## 1. 飞机尺寸与碰撞半径计算

### 1.1 8 英寸桨尺寸

8 英寸桨的直径为：

```text
8 × 25.4 = 203.2 mm
```

桨叶旋转半径为：

```text
203.2 / 2 = 101.6 mm
```

### 1.2 按“450 mm 为对角电机轴距”计算

多数四旋翼所说的 450 mm 轴距，是指两个对角电机中心之间的距离。按这种定义：

```text
机体中心到电机中心 = 450 / 2 = 225 mm
机体中心到最外侧桨尖 = 225 + 101.6 = 326.6 mm
最大桨尖到桨尖尺寸 = 2 × 326.6 = 653.2 mm
```

因此，这架飞机不能按照 450 mm 的机架尺寸设置碰撞范围。考虑旋转桨盘后，其理论最大对角占用尺寸约为：

```text
0.653 m
```

也就是用户估计的“600 多毫米”。飞机的理论物理碰撞半径约为：

```text
r_physical ≈ 0.327 m
```

### 1.3 必须确认轴距定义

如果厂家所说的 450 mm 是**相邻电机之间的距离**，而不是对角轴距，那么尺寸会变为：

```text
中心到电机 = 450 / √2 ≈ 318.2 mm
中心到外侧桨尖 = 318.2 + 101.6 ≈ 419.8 mm
最大桨尖到桨尖尺寸 ≈ 839.6 mm
```

这时物理碰撞半径将接近 `0.420 m`，不能继续按 `0.327 m` 计算。

### 1.4 实机测量方法

最终参数不要只依赖标称值。应在断电、拆电池状态下进行测量：

1. 将一对对角桨叶转到沿机臂向外的位置；
2. 测量两个最外侧桨尖之间的最大距离；
3. 将测量结果除以 2，得到实际物理半径；
4. 如果有桨保护圈、天线、支架或突出线缆，还要把这些部分计入；
5. 再增加定位误差、控制跟踪误差和额外安全余量。

安全半径可按下式估算：

```text
r_safe = r_physical + e_localization + e_tracking + margin
```

以对角轴距 450 mm 为例，若采用以下保守估计：

```text
物理半径             0.327 m
FAST-LIO2/EKF误差     0.080 m
轨迹跟踪误差          0.100 m
额外安全余量          0.100 m
--------------------------------
建议安全半径          0.607 m
```

因此：

- `0.4 m` 对这架飞机明显偏小；
- `0.6 m` 可作为完成误差测试后的实用值；
- `0.8 m` 建议作为首次障碍测试的保守值。

---

## 2. 系统数据链路

建议系统保持以下数据流：

```text
Livox MID-360
    ↓
livox_ros_driver2
    ↓ 点云/IMU
FAST-LIO2
    ├── /cloud_registered
    └── FAST-LIO2里程计
             ↓
            EKF
             └── /ekf/ekf_odom

/cloud_registered + /ekf/ekf_odom
             ↓
        Diff-Planner
             ├── 障碍栅格地图
             ├── 膨胀障碍地图
             └── /setpoints_cmd
                      ↓
                   PX4Ctrl
                      ↓
        /mavros/setpoint_raw/attitude
                      ↓
                     PX4
```

### 2.1 关键话题

| 用途 | 话题 |
|---|---|
| EKF 融合里程计 | `/ekf/ekf_odom` |
| FAST-LIO2 配准点云 | `/cloud_registered` |
| 规划目标点 | `/goal` |
| PX4Ctrl 轨迹指令 | `/setpoints_cmd` |
| 原始占据地图 | `/drone_0_diff_planner_node/grid_map/occupancy` |
| 膨胀占据地图 | `/drone_0_diff_planner_node/grid_map/occupancy_inflate` |

### 2.2 重要坐标系要求

当前 Diff-Planner 的地图代码直接读取点云的 XYZ 数值，**不会自动通过 TF 将点云转换到里程计坐标系**。因此必须确保：

```text
/cloud_registered 和 /ekf/ekf_odom 位于同一个数值坐标系
```

必须重点观察：

- 无人机静止时，墙面在地图中应保持静止；
- 无人机移动时，墙面不能跟着无人机一起移动；
- 地图中的障碍方向应与 EKF 位置、RViz 中的飞行方向一致；
- 规划器使用的 `world` 坐标必须和 EKF、FAST-LIO2 对齐。

如果障碍物随着飞机一起移动，首先检查 frame、外参和坐标变换，**不要通过增大膨胀半径掩盖坐标系错误**。

---

## 3. 需要修改的文件

### 3.1 主规划启动文件

```text
/home/nvidia/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/launch/exp/run_exp_single_lio.launch
```

该文件主要调整：

- 里程计话题；
- 点云话题；
- 最大速度、加速度和加加速度；
- 规划范围；
- 地图分辨率；
- 障碍膨胀半径；
- 虚拟地面和虚拟天花板。

### 3.2 高级地图和优化器参数

```text
/home/nvidia/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/launch/include/advanced_param_exp.xml
```

该文件主要调整：

- 地图概率参数；
- 障碍膨胀参数；
- 地图可视化高度；
- 优化器软避障距离；
- 地图衰减和射线清除参数。

### 3.3 多目标点配置

启动文件：

```text
/home/nvidia/gcs_ws/src/Diff-Planner/src/user_command/multipoint/launch/multipointplan_exp_lio.launch
```

目标点文件：

```text
/home/nvidia/gcs_ws/src/Diff-Planner/src/user_command/multipoint/config/points.yaml
```

现有约 10 m 的目标点不适合作为首次实飞测试。建议先改成 1 m、2 m 级别的短距离目标。

### 3.4 PX4Ctrl 文件

```text
/home/nvidia/gcs_ws/src/Diff-Planner/src/realflight_modules/px4ctrl/launch/run_ctrl_lio.launch
/home/nvidia/gcs_ws/src/Diff-Planner/src/realflight_modules/px4ctrl/config/ctrl_param_fpv.yaml
```

由于当前飞机已经能够稳定定点起飞和悬停，第一次接入 Diff-Planner 时，不建议同时修改 PX4Ctrl 的姿态增益、质量、悬停油门、`rc_reverse` 等参数。否则一旦发生振荡，将无法区分是控制器问题还是规划轨迹问题。

---

## 4. 第一阶段推荐参数

在主规划启动文件中，建议先使用以下低动态参数：

```xml
<arg name="max_vel"             value="0.3" />
<arg name="max_acc"             value="0.6" />
<arg name="max_jer"             value="2.0" />
<arg name="planning_horizon"    value="3.0" />

<arg name="local_map_size_x"    value="5.5" />
<arg name="local_map_size_y"    value="5.5" />
<arg name="local_map_size_z"    value="2.0" />

<arg name="virtual_ceil_height" value="2.0" />
<arg name="virtual_ground_height" value="0.1" />

<arg name="resolution"          value="0.2" />
<arg name="inflation_size"      value="0.8" />
```

### 4.1 参数解释

| 参数 | 初始建议 | 说明 |
|---|---:|---|
| `max_vel` | `0.3 m/s` | 首次实飞降低速度，减小刹车距离和跟踪误差 |
| `max_acc` | `0.6 m/s²` | 防止轨迹加速度过大引起姿态猛变 |
| `max_jer` | `2.0 m/s³` | 降低指令突变和机体抖动 |
| `planning_horizon` | `3.0 m` | 首次近距离测试无需过长规划范围 |
| `resolution` | `0.2 m` | 当前代码和算力条件下的初始值 |
| `inflation_size` | `0.8 m` | 首次障碍飞行使用的保守碰撞半径 |

> 不要在第一次测试中同时提高速度、减小膨胀半径并缩小软避障距离。每次只改变一个主要变量，并保存 rosbag。

---

## 5. 膨胀参数的实际含义

### 5.1 当前实现不是精确球形膨胀

当前 `grid_map.cpp` 的膨胀方式是在 X、Y、Z 三个方向扩展栅格，形成近似立方体的三维膨胀区域，而不是严格的球形或圆柱形飞行器模型。

膨胀栅格数近似为：

```text
inf_grid = ceil(inflation_size / resolution)
```

当前分辨率为 `0.2 m` 时，参数会量化为以下有效膨胀距离：

| `inflation_size` 设置值 | 实际栅格层数 | 轴向有效膨胀距离 |
|---:|---:|---:|
| `0.2` | 1 | `0.2 m` |
| `0.3` 或 `0.4` | 2 | `0.4 m` |
| `0.5` 或 `0.6` | 3 | `0.6 m` |
| `0.7` 或 `0.8` | 4 | `0.8 m` |

代码将膨胀层数限制为最多 4 层。因此在 `0.2 m` 分辨率下，不要期望通过设置远大于 `0.8 m` 的值继续线性增加栅格层数。

### 5.2 本机不建议使用 0.4 m 的原因

如果飞机物理半径为 `0.327 m`：

```text
0.4 - 0.327 = 0.073 m
```

只有约 7.3 cm 留给以下全部误差：

- FAST-LIO2 漂移；
- EKF 融合误差；
- 控制跟踪误差；
- 点云噪声；
- 障碍表面稀疏；
- 风和气流扰动；
- 地图离散化误差。

因此 `0.4 m` 不适合真实障碍避障飞行。

### 5.3 推荐调试顺序

```text
首次地图与障碍飞行：0.8 m
        ↓
确认地图坐标正确、无假障碍、跟踪稳定
        ↓
测量 EKF 最大误差和轨迹最大跟踪误差
        ↓
若误差满足要求，再尝试 0.6 m
```

建议：

- `0.8 m`：首次开放场地障碍测试；
- `0.6 m`：完成误差测量后的常规候选值；
- `<0.6 m`：不建议用于当前约 653 mm 桨尖尺寸的实机障碍飞行。

### 5.4 通道宽度估算

规划器把飞机近似为膨胀地图中的一个点，因此障碍通道的理论最小宽度约为：

```text
W_min ≈ 2 × inflation_size
```

实际还要考虑优化器软避障距离和轨迹跟踪误差：

| 膨胀参数 | 理论几何通道 | 建议首次实际通道 |
|---:|---:|---:|
| `0.8 m` | 大于 `1.6 m` | 约 `1.8～2.2 m` 或更宽 |
| `0.6 m` | 大于 `1.2 m` | 约 `1.4～1.8 m` 或更宽 |

不要直接在窄门、走廊或两堵硬墙之间进行首次调参。

---

## 6. 优化器软避障参数

高级参数文件中可见类似参数：

```xml
<param name="optimization/obstacle_clearance" value="0.1" />
<param name="optimization/obstacle_clearance_soft" value="0.5" />
```

可作如下理解：

- 地图 `inflation_size`：承担主要几何安全边界；
- `obstacle_clearance_soft`：鼓励轨迹进一步远离障碍；
- 软距离不等同于硬碰撞半径，不能用它代替正确的机体膨胀。

建议初始保持当前软避障值。若后续发现规划器在宽阔可行通道中仍过分保守，可在保持 `inflation_size=0.6～0.8 m` 的情况下，将软避障距离逐步从 `0.5 m` 调到 `0.4 m`、`0.3 m`。不要为了通过窄通道直接把机体膨胀缩小到危险范围。

---

## 7. 启动流程

### 7.1 启动已有感知与 EKF 一键脚本

```bash
cd ~/gcs_ws
source devel/setup.bash
bash ~/gcs_ws/src/livox_ros_driver2/scripts/start_fastlio2_ekf.sh
```

该脚本应完成：

- MAVROS；
- Livox 驱动；
- FAST-LIO2；
- FAST-LIO2 RViz；
- EKF。

确认没有重复启动 MAVROS、Livox、FAST-LIO2 或 EKF。

### 7.2 启动 Diff-Planner

新终端执行：

```bash
source ~/gcs_ws/devel/setup.bash
roslaunch diff_planner run_exp_single_lio.launch
```

### 7.3 启动 PX4Ctrl

新终端执行：

```bash
source ~/gcs_ws/devel/setup.bash
roslaunch px4ctrl run_ctrl_lio.launch
```

### 7.4 可选：启动多目标点节点

```bash
source ~/gcs_ws/devel/setup.bash
roslaunch multipoint multipointplan_exp_lio.launch
```

### 7.5 RViz

可单独启动 Diff-Planner 的 RViz：

```bash
source ~/gcs_ws/devel/setup.bash
roslaunch diff_planner exp_rviz.launch
```

如果 Jetson 算力或显存紧张，更推荐直接在现有 FAST-LIO2 RViz 中添加下列显示项，而不要同时运行两个 RViz：

```text
PointCloud2: /cloud_registered
PointCloud2: /drone_0_diff_planner_node/grid_map/occupancy
PointCloud2: /drone_0_diff_planner_node/grid_map/occupancy_inflate
Odometry:   /ekf/ekf_odom
```

---

## 8. 拆桨地图测试

### 8.1 测试前检查

保持飞机不解锁，并建议拆下螺旋桨。执行：

```bash
rostopic hz /cloud_registered
rostopic hz /ekf/ekf_odom
rostopic echo -n 1 /ekf/ekf_odom
rostopic info /drone_0_diff_planner_node/grid_map/occupancy
rostopic info /drone_0_diff_planner_node/grid_map/occupancy_inflate
```

检查规划节点是否存在：

```bash
rosnode list | grep -E 'diff|planner|grid'
```

### 8.2 静态障碍测试

使用纸箱、泡沫板或软质障碍物进行测试：

1. 把障碍物放在飞机前方约 2～3 m；
2. 观察 `/cloud_registered` 是否出现障碍；
3. 观察原始占据地图是否出现对应障碍；
4. 观察膨胀地图是否比原始障碍向外扩大；
5. 移走障碍，观察射线清除是否能够清除旧障碍；
6. 缓慢搬动飞机，确认障碍在世界坐标中保持固定。

### 8.3 注意可视化截断

当前地图可视化代码可能只发布飞机朝向前方的一部分局部地图。障碍物移动到飞机后方后在 RViz 中消失，不一定说明内部地图已经被完全清除。应结合飞机朝向、局部地图范围和射线清除结果共同判断。

### 8.4 地图验收标准

进入实飞测试前至少满足：

- [ ] 静止状态下墙面不漂移；
- [ ] 移动飞机时障碍不跟随机体移动；
- [ ] 障碍方向与真实方向一致；
- [ ] 原始地图和膨胀地图都能正常发布；
- [ ] 膨胀范围与设置值基本一致；
- [ ] 移走障碍后没有大面积永久残影；
- [ ] 地面没有被误判成前方整面障碍墙；
- [ ] 天花板和地面高度限制符合测试场地；
- [ ] 点云、里程计时间戳连续，无明显倒退或跳变。

---

## 9. 规划器地面测试

先不要解锁飞机，只测试规划器是否能生成合理轨迹。

### 9.1 发送 1 m 短距离目标

```bash
rostopic pub -1 /goal geometry_msgs/PoseStamped "header:
  frame_id: 'world'
pose:
  position: {x: 1.0, y: 0.0, z: 0.8}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
```

观察：

- 规划是否成功；
- 轨迹是否从当前位置开始；
- 轨迹是否在地图自由空间内；
- 有障碍时轨迹是否绕开膨胀区域；
- 目标点高度是否合理；
- 是否出现轨迹突然跳变或穿障碍。

### 9.2 推荐的首次目标点

将 `points.yaml` 中的大范围目标暂时改为：

```yaml
# 示例，具体键名保持原文件格式
point0: [1.0, 0.0, 0.8]
point1: [2.0, 0.0, 0.8]
point2: [0.0, 0.0, 0.8]
```

先做直线短距离测试，再加入绕障目标。不要一开始就发送 10 m 远点。

---

## 10. 分阶段实飞测试

### 阶段 A：无障碍短距离飞行

场地要求：

- 空旷；
- 无人员靠近；
- 起飞高度约 `0.8～1.0 m`；
- 目标距离 `1 m`；
- 最大速度 `0.3 m/s`。

验证：

- PX4Ctrl 能稳定接收 `/setpoints_cmd`；
- 飞机不会突然倾斜或快速加速；
- 到达目标后不会持续来回振荡；
- EKF 位置和实际运动方向一致；
- 遥控器模式切换和紧急停机有效。

### 阶段 B：单个软障碍物

1. 在航线侧前方放置大型纸箱或泡沫板；
2. 保证即使规划失败，直线航迹也不会撞向人员或硬墙；
3. 使用 `inflation_size=0.8 m`；
4. 先在 RViz 中确认绕障轨迹；
5. 再允许飞机低速执行；
6. 记录实际最小障碍距离和最大跟踪误差。

### 阶段 C：降到 0.6 m 膨胀

只有满足以下条件后才能尝试：

- FAST-LIO2/EKF 地图长期稳定；
- 定位最大误差有实测记录；
- 轨迹跟踪最大误差有实测记录；
- 不存在明显控制振荡；
- 障碍点云连续且无大面积孔洞；
- `r_physical + e_localization + e_tracking + margin ≤ 0.6 m`。

每次只降低一个档位，并重新完成地面规划、空旷飞行和软障碍测试。

---

## 11. 建议记录 rosbag

每次参数测试都应记录数据：

```bash
rosbag record -O ~/diff_planner_obstacle_test.bag \
  /ekf/ekf_odom \
  /cloud_registered \
  /drone_0_diff_planner_node/grid_map/occupancy \
  /drone_0_diff_planner_node/grid_map/occupancy_inflate \
  /drone_0_planning/trajectory \
  /setpoints_cmd \
  /mavros/state \
  /mavros/imu/data
```

如果某个话题名称不存在，先通过下面命令确认实际名称：

```bash
rostopic list | grep -E 'trajectory|setpoint|occupancy|cloud_registered|ekf_odom'
```

每个 rosbag 建议配套保存参数记录，例如：

```text
测试编号：T03
inflation_size：0.8
resolution：0.2
max_vel：0.3
max_acc：0.6
max_jer：2.0
障碍类型：泡沫板
障碍宽度：1.0 m
目标距离：2.0 m
结果：成功/失败
最大跟踪误差：___ m
最小障碍距离：___ m
异常现象：___
```

---

## 12. 常见问题诊断表

| 现象 | 优先检查 | 不应首先做的事 |
|---|---|---|
| 障碍跟着飞机移动 | 点云与 EKF 坐标系、FAST-LIO2 外参 | 盲目增大膨胀半径 |
| 地图没有障碍 | 点云话题、订阅关系、点云 frame、局部地图范围 | 直接实飞验证 |
| 障碍移走后长期残留 | 射线清除、点云遮挡、时间戳、地图衰减 | 立即减小膨胀 |
| 地面形成整片障碍 | 雷达外参、地面高度、机体姿态、点云裁剪 | 提高飞行速度冲过去 |
| 规划器找不到路径 | 膨胀过大、目标在障碍内、通道过窄、地图范围不足 | 直接将膨胀降到 0.2 m |
| 轨迹可行但飞机振荡 | `max_acc`、`max_jer`、控制跟踪、估计延迟 | 同时重调全部 PX4Ctrl 增益 |
| 到障碍附近才急转 | 规划范围过短、速度过高、点云探测距离不足 | 只提高姿态增益 |
| RViz 后方障碍消失 | 前向可视化截断和局部地图显示范围 | 认定内部地图一定已清除 |
| 出现 position diff 过大 | EKF跳变、指令过激、时延、规划轨迹不连续 | 放宽报警后继续高速飞行 |

---

## 13. 参数调整决策

### 13.1 路径过分保守

按以下顺序处理：

1. 确认实际通道宽度足够；
2. 确认障碍地图没有残影和假障碍；
3. 保持几何膨胀不变；
4. 将软避障距离从 `0.5` 逐步调整到 `0.4`、`0.3`；
5. 最后才评估是否将膨胀从 `0.8` 降到 `0.6`。

### 13.2 飞机转弯或刹车时振荡

优先降低：

```text
max_vel: 0.3 → 0.2 m/s
max_acc: 0.6 → 0.4 m/s²
max_jer: 2.0 → 1.0～1.5 m/s³
```

先看轨迹是否过激，再判断是否需要调整控制器。不要因为接入规划器后振荡，就立刻修改已经验证稳定的悬停油门和 `rc_reverse`。

### 13.3 地图噪声造成假障碍

优先检查：

- 雷达是否牢固；
- 外参是否正确；
- FAST-LIO2 是否稳定；
- 点云时间戳是否连续；
- 地图命中/未命中概率是否合理；
- 地面和机体自身点云是否被误加入地图。

概率参数应最后小幅调整，不能使用过强滤波掩盖外参或振动问题。

---

## 14. 编译与重启说明

只修改 `.launch`、`.xml` 和 `.yaml` 参数时，通常不需要重新编译，但必须重启对应 ROS 节点才能生效。

如果修改了 C++ 源码，则执行：

```bash
cd ~/gcs_ws
catkin_make
source devel/setup.bash
```

随后重新启动 Diff-Planner。

---

## 15. 最终起飞前检查单

### 硬件

- [ ] 桨叶无裂纹，安装方向正确；
- [ ] 机架、雷达、飞控和电池固定可靠；
- [ ] 实际最大桨尖尺寸已测量；
- [ ] 电池电量充足；
- [ ] 遥控器模式切换和紧急停机已测试；
- [ ] 测试区无人、无硬质近距离障碍。

### 定位与地图

- [ ] FAST-LIO2 正常；
- [ ] EKF 正常，无位置跳变；
- [ ] `/cloud_registered` 与 `/ekf/ekf_odom` 坐标一致；
- [ ] 原始障碍地图正常；
- [ ] 膨胀障碍地图正常；
- [ ] 地图无明显漂移、拖影和大面积假障碍。

### 规划与控制

- [ ] 首次速度不超过 `0.3 m/s`；
- [ ] 首次加速度不超过 `0.6 m/s²`；
- [ ] 首次 jerk 不超过 `2.0 m/s³`；
- [ ] 首次膨胀参数为 `0.8 m`；
- [ ] 已完成拆桨规划测试；
- [ ] 已完成 1 m 无障碍短距离测试；
- [ ] `/setpoints_cmd` 连续、无突跳；
- [ ] PX4Ctrl 没有频率、估计或油门异常报警；
- [ ] 已启动 rosbag 记录。

---

## 16. 建议结论

根据 450 mm 对角轴距与 8 英寸桨估算，这架飞机的最大桨盘对角尺寸约为 `653 mm`，物理碰撞半径约为 `0.327 m`。因此，Diff-Planner 不能再按小型穿越机或裸机架设置碰撞范围。

推荐采用以下实施路线：

```text
拆桨验证地图和坐标系
        ↓
0.8 m 膨胀，验证规划轨迹
        ↓
0.3 m/s 空旷短距离实飞
        ↓
0.8 m 膨胀，软障碍低速绕行
        ↓
统计定位误差和跟踪误差
        ↓
满足安全公式后再评估 0.6 m 膨胀
```

首次接入期间，应保持已经验证稳定的 PX4Ctrl 悬停参数不变，只调整 Diff-Planner 的速度、加速度、jerk、规划范围和地图安全距离。这样发生问题时，才能明确区分是地图、规划、估计还是控制环节造成的。

---

## 17. RViz 地图判读补充：不要把地面膨胀误认为空闲区域

> **修正此前初始膨胀建议的适用条件：** `0.8 m` 不只是水平安全半径，当前代码会在 XYZ 三个方向使用同样的膨胀层数。如果输入点云包含地面，地面也会向上膨胀。因此 `0.8 m` 膨胀与 `0.8 m` 悬停高度不能在未经地图验证时直接视为可行组合；地面体素离散化还会改变边界。必须先确认目标高度的水平截面没有被地面膨胀占满。不能简单删除全部地面点或缩小水平碰撞半径来获得可飞区域。若垂直空间受限，应进一步评估独立的 XY/Z 膨胀模型、机体上下尺寸和误差余量。

原始 RViz 配置中 `map inflate` 使用 `AxisColor`、`Axis=Z`，颜色只用于区分高度，不代表空闲或占据概率。发布代码只输出膨胀计数非零的体素，并对高度和前向区域进行裁剪；看不到方块不等于确认自由。

精简的被动地图检查配置：

```bash
source ~/gcs_ws/devel/setup.bash
rviz -d ~/gcs_ws/src/livox_ros_driver2/config/rviz/diff_planner_map_debug.rviz
```

此配置无机器人网格模型、无仿真地图、无多机显示，也没有发布飞行目标的工具。默认显示原始点云与未膨胀占据地图，膨胀图层需手动勾选，以便逐层比较。它不会修改规划器或飞控参数。

参数名称以实际启动文件为准：分辨率传入参数为 `map_resolution`，上下边界为 `virtual_ceil` 与 `virtual_ground`；它们分别映射到节点参数 `grid_map/resolution`、`grid_map/virtual_ceil` 与 `grid_map/virtual_ground`。第4节的概念示例若直接用于此版本 launch，应使用这些实际名称。
