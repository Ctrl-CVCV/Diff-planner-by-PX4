# 2026-10-20：坐标链路核查与补全录包

## 1. 结论和证据边界

### 2026-10-21 21:08 新飞行包补充

已只读检查后来新增的 `20261021_210844_debug/flight_0.bag`：
约第14.34秒起LIO/注册点云中断，EKF仍继续至包末，z升至10.464米。
此前静止检查正常不能证明此次飞行全程正常；详见[新包漂移分析与证据](FLIGHT_ANALYSIS_20261021_210844.md)。
以下“本轮复核状态”记录的是更早一次检查，不代表这个新包缺失，也不是当前ROS图状态。

### 2026-10-21 本轮复核状态

- 只读查询当前ROS图时仅有 `/rosout`；MAVROS状态与落地状态话题均无发布并查询超时，因此本轮不能采集新的实时坐标链或验证降落动作。
- 下文2026-10-20的15秒静止短包及其 `/tmp` 快照本轮已找不到；表格保留的是上一轮审计统计，不是本轮重新采样或重新计算的结果。
- 已直接重新读取仍在的 `~/flight_logs/20261020_211622/takeoff.bag`：缺 `/Odometry`、注册点云、RAW/膨胀地图、TF，LIO/EKF配对数为0，不能从它重建完整坐标证据。
- 本轮只读报告长期保存在 [flight_frame_audit_20261021.json](evidence/flight_frame_audit_20261021.json)，不是仅保存在临时目录。

### 2026-10-20 历史静止核查

本次不仅查看源码，还在确认 `/mavros/state` 为未解锁、`extended_state` 为 ON_GROUND 后，
只读录取了 15 秒运行数据。没有启动规划器/控制器，没有发航点、切模式或解锁。
该次采样时运行的是 Livox、FAST-LIO、EKF、MAVROS 等感知链；当时规划器、PX4Ctrl 不在节点列表中，不能把它当成后来节点状态。

- 短包时间：2026-10-20 22:41:04.632～22:41:19.630，北京时间 UTC+8。
- 短包：`/tmp/diff_frames_ground_20261020.bag`，约 27 MB。
- 当时运行参数：`/tmp/diff_frames_runtime_20261020.yaml`。
- 完整分析：`/tmp/diff_frames_ground_analysis_final_20261020.json`。
- `/tmp` 是临时目录，需长期保留时应另行备份；本报告已保留核心数值。

**该次静止短包没有显示 LIO/EKF 之间存在几十厘米级的数值错位。**
但确实存在 frame_id/TF 接口不一致、LiDAR/IMU 时间轴衔接隐患，以及建图射线起点未计入雷达平移外参的问题。
这些不是“已经证实上次规划失败的根因”；旧飞行包没有地图、LIO 和 TF，不能倒推当时情况。
当前参数服务器中的 planner 参数也不能当成旧飞行时参数：节点退出后参数仍可能残留。

## 2. 实际坐标链路

```text
/livox/lidar                          /mavros/imu/data_raw
     │ LiDAR 坐标 L                         │ PX4 IMU 机体系 I
     └──────────── FAST-LIO ─────────────────┘
          p_I = R_IL * p_L + t_IL
          p_C = R_CI * p_I + p_CI
                   │
       ┌───────────┼────────────────────┐
       │           │                    │
/cloud_registered  /cloud_registered_body  /Odometry
 frame=camera_init  frame=body             camera_init -> body
 点坐标已在 C 系    点坐标已在 I 系         IMU 的 LIO 位姿
       │                                  │
       │                                  └── EKF + /mavros/imu/data
       │                                         │
       │                                   /ekf/ekf_odom
       │                                    frame=world
       └────────── GridMap / Diff-Planner ────────┤
                    不自动应用 TF                 │
                         │                 PX4Ctrl 反馈定位
                轨迹 -> /setpoints_cmd ──────────┘
                         frame=world
```

这里 C 表示 FAST-LIO 的 `camera_init` 世界系，I 表示实际使用的 PX4 IMU 机体系；
`body` 是源码给 I 系点云与位姿使用的名字，不应再额外把 LiDAR 外参施加一次。
`camera_init` 不是“相机坐标中的雷达点”，名称不能代替数值语义。

## 3. 逐段核查

### 3.1 LiDAR 到 IMU，再到 LIO 世界系

运行配置中的 `mapping/extrinsic_R`、`extrinsic_T` 用于 LiDAR 到 IMU：

```text
t_IL = [0.006849, -0.034990, 0.069355] m
R_IL = [[ 0.858277, -0.017062,  0.512903],
        [ 0.026414,  0.999591, -0.010949],
        [-0.512507,  0.022946,  0.858377]]
```

`det(R)=1.0000003867`，`max|RᵀR-I|≈7.12e-7`：矩阵本身接近有效旋转，
但这只验证代数性质，不证明物理安装标定准确。

对应源码：`FAST_LIO/src/laserMapping.cpp` 的 `RGBpointBodyLidarToIMU()`、
`RGBpointBodyToWorld()`、`publish_frame_world()`、`publish_frame_body()`、`publish_odometry()`。
`/cloud_registered` 已变换到世界系，**不能再拿整帧点云乘一次当前机体位姿**。

短包使用相同 header 时间的 body/world 点云与 `/Odometry`，每帧抽样最多 32 点：

| 验证项 | 结果 |
|---|---:|
| 同时刻点云/位姿 | 149 帧 |
| 抽样点 | 4,768 |
| `p_world - (R_odom*p_body+p_odom)` 误差 p95 | `2.75e-7 m` |
| 最大误差 | `9.05e-7 m` |

这支持当前发布链中的 body/world 数学变换一致，没有观察到整帧重复变换。
这个抽样比较要求 `dense_publish_en=true`、两路点云点顺序一致；不用于验证物理外参或全部点的质量。

### 3.2 LIO 到 EKF：数值大体一致，但名称不一致

实际编译目标由 `realflight_modules/faster-lio/ekf_pose/CMakeLists.txt` 指向
`ekf_node_vio_timesync_with_acc_pub.cpp`。
`get_pose_from_VIOodom()` 使用：

```text
R_WI = R_WR * inverse(Rr_i)
p_WI = p_WR - R_WI * tr_i
```

当前 `Rr_i` 为单位四元数，`tr_i=0`，没有额外坐标轴翻转或杆臂变化；
`rotation_imu=I`，`imu_trans_x/y/z=0`，`offset_px/py/pz=0`。
初始化沿用 LIO 位姿，随后 IMU 传播和 LIO 校正，发布位置还经过低通滤波。

问题在 `system_pub()`：**输出 header 写死为 `world`，但没有执行 camera_init 到 world 的空间变换**；
`child_frame_id` 也没有设置。短包中 LIO 为 `camera_init/body`，EKF 为 `world/空字符串`。

按 header 最近邻、最大配对时间差 30 ms 的 149 组样本：

| 指标 | 中位数 | p95 | 最大值 |
|---|---:|---:|---:|
| LIO/EKF 数值位置差 | 0.00860 m | 0.01574 m | 0.01968 m |
| LIO/EKF 姿态差 | 0.553° | 0.655° | 0.703° |
| 配对时间差 | 1.51 ms | 4.23 ms | 29.94 ms |

这不是外参估计结果，没有应用 TF，也不是运动中的跟踪精度或飞行安全认证。
它支持“此刻两路位置数值基本对齐”，不能代表上次飞行、快速运动或后续重启。

### 3.3 TF：当前记录中 world 与 camera_init 不连通

短包只有 `camera_init -> body` 的动态边，以及 MAVROS 的
`map -> map_ned`、`odom -> odom_ned`、`base_link -> base_link_frd` 静态边。
**没有连接 `world` 与 `camera_init` 的链。**

这会影响 RViz 在同一 Fixed Frame 下叠加注册点云与 EKF/地图，
但 GridMap 直接用 XYZ，不调用 TF，所以“缺这条 TF”本身不是内部碰撞失败的直接证明。
不能只加一个任意 identity TF 让显示不报错：应先确认坐标原点/方向、非零 offset、重启生命周期和发布责任。
也不能把带滤波误差的两路定位差，直接当成两个世界系之间的动态 TF。

### 3.4 点云到 GridMap：没有坐标系检查/变换，射线起点也存在近似

`grid_map.cpp` 的 `cloudCallback()` 直接采用注册点云 XYZ 加 `init_x/y/z`，不查 header/TF。
`odomCallback()` 用 EKF 位姿构造地图中心与射线起点。
当前 `init_x/y/z=0`，默认 `cam2body` 平移也为 0，故射线起点等于 EKF 的 IMU 位置。

而真实 LiDAR 原点应为 `p_IMU + R_IMU*t_IL`；当前杆臂长度约 **0.078 m**。
这意味着即便 world/body 点云变换正确，射线自由空间更新仍用了近似起点，误差量级接近当前 0.0875 m 的一个体素。
它可能影响近场清障，但不能据此认定已经造成整个 0.5 m 路径被占据。
不要简单把 `imu_trans` 改成雷达外参：那还会改变 PX4Ctrl 使用的控制反馈点，必须分别建模。

地图发布另有方向裁剪：RAW 点积阈值 `>0.5`，膨胀图 `>0`，并裁掉虚拟高度边界外点。
核查时旧代码的两个发布消息没有显式填写时间戳。2026-10-21 已补充 `header.stamp=ros::Time::now()`：
这是地图**发布时刻**，不是原始点云采样/占据更新时刻，历史包不会被补齐，必须重启新编译的规划器才生效。
新录包器能保留这些地图，但**不是完整内部占据快照**；不能仅凭 RViz 某处没有点就证明优化器该处查询为空闲。

### 3.5 时间轴：发现需要专门验证的衔接风险

运行参数 `common/time_offset_lidar_to_imu=-0.068406 s`，`time_sync_en=false`。
FAST-LIO 的 `imu_cbk()` 实际执行：

```text
t_imu_for_lio = t_mavros_imu - time_offset_lidar_to_imu
              = t_mavros_imu + 0.068406 s
```

随后 LIO 的点云和里程计用 `lidar_end_time` 发 header；EKF 接收原始 MAVROS IMU 时间戳，
并直接根据 LIO header 回溯自己的 IMU 状态缓冲，没有对应的时间偏移转换。
**若这项标定确实表示传感器时间轴偏差，则两级融合存在约 68.4 ms 的时间语义不一致风险。**
静止短包不能检验运动时相位误差，也不能验证此时间标定准确性，因此本次没有改符号或补偿值。
在 0.3 m/s 下，68.4 ms 对应约 0.0205 m 位移，仅这个量级也不足以证明整段被占据。
应在隔离离线重算/受控地面运动数据中核对一次补偿应该放在哪里，避免重复补偿。

### 3.6 目标到控制器：世界偏移，不是机体前方；FCU 航向可以不同

起飞脚本生成 `(odom.x+dx, odom.y+dy, odom.z)`，header 写 `world`。
规划器读取目标和 EKF 数值，不自动变换；轨迹服务器输出 `world` 的 `/setpoints_cmd`；
PX4Ctrl 读取同一路 EKF 和轨迹指令，未在位置环中另做 ENU/NED 翻转。
因此 `--goal-dx 0.5` 表示定位世界系 +X，不一定是机头前方。
旧飞行目标消息的航向约 1.69°；该次方向差异不能解释完全静止，应急停留指令仍是直接原因。

PX4Ctrl 的 `controller.cpp` 使用 `u.q = imu.q * odom.q.inverse() * desired_attitude`，
对齐 FCU 与定位世界姿态。短包中 FCU/EKF 的绝对角差中位数约 82.08°，
但该相对旋转相对首帧的变化 p95 约 0.206°，最大 0.361°。
不同世界航向不直接表示姿态错 82°；也不能据此再手工给输入旋转一次 NED/ENU。
物理机体系是否完全一致仍应结合安装和校准核实。

## 4. 现在如何收集足够的证据

下一步应采集**地面缓慢运动**而非盲目重飞：拆桨、保持未解锁且不运行控制器/起飞脚本，
由人员安全托持缓慢平移约0.5m并做小角度转向，对照固定障碍物是否随位置/朝向漂移。
同时保留原始Livox、两路IMU、LIO/EKF和两种地图，分别验证动态数值对齐、时间衔接和近场射线清障。
静止数据不足以验证68.4ms时间轴风险或物理外参；不能仅为让RViz叠图就添加未经确认的identity TF。

完整启动与隔离回放流程见 [ROSBAG_RECORD_REPLAY.md](ROSBAG_RECORD_REPLAY.md)。
先只启动已验证的感知链和规划器，不启动起飞脚本，做地面检查：

```bash
source ~/gcs_ws/devel/setup.bash
python3 ~/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/scripts/record_diff_planner_debug.py \
  --duration 60 --raw-lidar
```

控制台会打印新目录和 RAW/INFLATE 收包状态。原始点云录制增加负载，先确认磁盘空间、CPU、
EKF/IMU 频率和新鲜度正常，不要把采集负载带入未验证的飞行链。
默认不加 `--duration` 时录到 Ctrl+C；在落地、未解锁后结束飞行采集，等待索引正常关闭。

分析时将目录替换成实际输出，不混入不同会话：

```bash
source ~/gcs_ws/devel/setup.bash
DIR="$HOME/flight_logs/实际输出目录"
python3 ~/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/scripts/analyze_diff_planner_frames.py \
  "$DIR"/*.bag --params "$DIR/runtime_all.yaml" --output "$DIR/frame_audit.json"
```

新录包器保存起止参数、节点信息，默认录 world/body 点云、两路 IMU、TF、LIO/EKF、控制/电池/时间同步，
并覆盖整个规划命名空间，结束时检查地图是否真正有消息。
地图消息为 0 点与没有地图消息是不同情况：前者可表示空图，后者无法回看。
本次 5 秒实测中规划器未运行，脚本如实报告两张地图缺失并返回 2；没有伪造地图或启动规划器来凑数据。

后续若要精确解释某次碰撞，还需在内部查询处保留真实采样坐标、占据返回值和虚拟边界状态；
当前录包/坐标脚本没有修改优化器、地图内部查询或飞行参数，也没有把发布地图当成内部查询证明。
