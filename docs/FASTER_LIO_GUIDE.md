# Faster-LIO 当前版本：源码、启动、看图与录包

更新：2026-11-20。本页是当前 Faster-LIO 操作入口，替代此前性能、精细参数、时间补偿和高度排查等阶段性调试文档。比赛任务继续使用[混合任务全流程](HYBRID_MISSION_FLIGHT_AND_BAG_RUNBOOK.md)，标定方法保留在[外参标定](LIDAR_IMU_EXTRINSIC_CALIBRATION.md)。

## 1. 代码和配置只从 gcs_ws 使用

| 用途 | 当前路径 |
|---|---|
| Faster-LIO 源码包 | `~/gcs_ws/src/Diff-Planner/src/realflight_modules/faster-lio/faster-lio` |
| 实机配置（唯一推荐入口） | 上述包的 `config/mid360_px4.yaml` |
| 程序 | `~/gcs_ws/devel/lib/faster_lio/run_mapping_online` |
| 动态库 | `~/gcs_ws/devel/lib/libfaster_lio.so` |
| EKF 源码 | `~/gcs_ws/src/Diff-Planner/src/realflight_modules/faster-lio/ekf_pose` |
| 一键定位链 | `~/gcs_ws/src/livox_ros_driver2/scripts/start_fastlio2_ekf.sh` |
| EKF/视觉桥入口 | `roslaunch diff_planner faster_lio_ekf_bridge.launch` |

这次整理把通过地面测试的外部工作区源码和实机配置收进 `gcs_ws`，补齐一键启动所需的 launch 参数，并统一默认输出话题。**没有增加 IVox 上限或更改滤波、外参、时间补偿算法。** 配置示例 `mid360.yaml` 不作为当前飞行配置；不要与 `mid360_px4.yaml` 混用。

`~/fasterlio_ws` 暂时保留为旧副本，避免破坏仍在运行的进程；不再作为推荐启动路径。这次整理不会停止旧节点或强切定位。**迁移必须落地锁桨后，先正常停止旧定位、EKF 和桥，再在新终端启动整条新链**。只 source 新工作区不会改变正在运行的程序；一键脚本可能复用已有 `/Odometry`，因此不能用它直接覆盖旧链。

## 2. 保留的当前参数

| 参数 | 值 |
|---|---|
| `filter_size_surf` / `filter_size_map` | `0.20` / `0.40` m（2026-11-21 因 CPU 余量回退，见第 8 节） |
| `point_filter_num` / `max_iteration` | `3` / `3` |
| IVox 每格点数上限 | **没有此限制** |
| `ivox_grid_resolution` / `ivox_nearby_type` | `0.5` / `18` |
| `mapping/extrinsic_est_en` | `false` |
| `common/imu_topic` | `/mavros/imu/data_raw` |
| `common/time_sync_en` | `false` |
| `common/time_offset_lidar_to_imu` | `-0.07625571` s |
| `publish/dense_publish_en` | `false` |
| PCD / path 文件保存 | 关闭（ROS `/path` 仍发布） |

保留 2026-11-13 标定值，方向为 **LiDAR → IMU**：

```yaml
extrinsic_T: [0.032600, 0.027755, 0.086446]
extrinsic_R: [0.856373, -0.003504, 0.516346,
              0.007575, 0.999955, -0.005777,
             -0.516302, 0.008859, 0.856361]
```

当前版本保留实际运行副本中的静态时间补偿：IMU 时间戳先减去该参数。**不是不带时间补偿的纯官方版本，也不是 0.000 s 的旧试验档。** 不要因清理文档再次套用旧参数。

## 3. 地面编译与来源检查

编译会占用 CPU；只在地面锁桨、非飞行状态执行。

```bash
source /opt/ros/noetic/setup.bash
cd ~/gcs_ws
catkin_make --pkg faster_lio -DCMAKE_BUILD_TYPE=Release -j1 -l2
source ~/gcs_ws/devel/setup.bash
rospack find faster_lio
ldd ~/gcs_ws/devel/lib/faster_lio/run_mapping_online | grep libfaster_lio
bash ~/gcs_ws/src/livox_ros_driver2/scripts/start_fastlio2_ekf.sh --check-only
```

包路径、程序、动态库都应来自 `gcs_ws`。一键脚本也会检查这些来源，检测到外部包或动态库就拒绝继续。每个新终端都只加载 `~/gcs_ws/devel/setup.bash`；不要随后再 source `~/fasterlio_ws/devel/setup.bash`。源码或 launch 改了但没重编译/重启，不能认为运行状态已更新。

## 4. 启动定位链（不发起飞命令）

### 推荐：一键启动

地面先观察 ROS 估计、暂不送入 PX4：

```bash
bash ~/gcs_ws/src/livox_ros_driver2/scripts/start_fastlio2_ekf.sh --no-vision-bridge
```

它启动 roscore、MAVROS、IMU 频率设置、Livox、Faster-LIO、外部 EKF；默认不启动 RViz，不启动 PX4Ctrl、不解锁、不起飞。`--no-vision-bridge` 只是不新启动桥，不会停止其他终端已有的桥；仅观察前要在地面确认旧桥已停止。检查完后在地面停止，再按任务手册启动正式整链：

```bash
bash ~/gcs_ws/src/livox_ros_driver2/scripts/start_fastlio2_ekf.sh
```

默认还启动 `/ekf/ekf_odom → /mavros/vision_pose/pose` 桥，**可能影响 PX4 的估计，不是只读观察**。PX4 是否采用取决于飞控参数。文件名保留 `fastlio2` 以兼容旧命令，但默认实现已经是 Faster-LIO；切回 FAST-LIO2 使用 `--lio fastlio`，也必须先在地面停止另一套定位。飞行中不要重启、Ctrl+C 或编译定位链。

### 单独启动 Faster-LIO（与一键方式二选一）

仅在 MAVROS/IMU、Livox 驱动已启动，且没有其他 LIO 发布者时：

```bash
source ~/gcs_ws/devel/setup.bash
roslaunch faster_lio mapping_mid360.launch \
  config_file:=$(rospack find faster_lio)/config/mid360_px4.yaml \
  start_lidar:=false \
  mapping_node_name:=laserMapping \
  rviz:=true
```

`start_lidar:=true` 只用于没有驱动的情况，防止重复驱动。ROS 参数赋值是 `rviz:=true`，不是 `rviz=true`。

统一默认输出 `/Odometry`、`/cloud_registered`、`/cloud_registered_body`、`/path`；launch 内将源码原生 `/laserMapping/...` 话题重映射到这些名称。自定义输出可以传 `odom_topic`、`cloud_topic`、`body_cloud_topic`、`path_topic`，对应 EKF/RViz/录包也要跟着改。

需要单独追加外部 EKF 时（现有 EKF 未运行）：

```bash
roslaunch diff_planner faster_lio_ekf_bridge.launch
```

默认订阅 `/Odometry` 和 `/mavros/imu/data`，输出 `/ekf/ekf_odom`，不发布视觉位姿。不要与一键脚本重复启动 EKF。已有 EKF、只想预览桥输出时可使用：

```bash
roslaunch diff_planner faster_lio_ekf_bridge.launch \
  start_ekf:=false publish_vision:=true vision_topic:=/debug/vision_pose
```

桥复制位姿和原时间戳，不加高度、不翻转轴、不补做 TF 变换。Faster-LIO 使用 `camera_init`，外部 EKF 使用 `world`；仅改 frame 名称不构成真实坐标变换，比较两者须核对初始化原点与方向。

## 5. RViz 看点云地图和里程计

定位已经运行时，不要再启动一套 LIO。另开终端只启动显示：

```bash
source ~/gcs_ws/devel/setup.bash
rviz -d "$(rospack find diff_planner)/launch/include/lio_cloud_odom.rviz"
```

- Fixed Frame 为 `camera_init`，PointCloud2 选择 `/cloud_registered`，Odometry 选择 `/Odometry`，Path 选择 `/path`。
- 这是 **LIO 点云与轨迹**，不是 Planner 占据地图；障碍地图 RAW/膨胀图见[地图检查](OBSTACLE_MAP_CHECK.md)。
- `/cloud_registered` 是逐帧配准点云，显示累积时间决定可见地图，不代表已启用完整 PCD 保存。不要用过长累积掩盖运动残影或增加负载。
- 没有显示时检查话题有无数据、消息 frame_id、Fixed Frame 和 ROS master；不要盲目增加静态 TF 或将 `world` 与 `camera_init` 宣称等价。

基础检查：

```bash
rostopic hz /livox/lidar
rostopic hz /Odometry
rostopic delay /Odometry
rostopic hz /ekf/ekf_odom
rostopic echo -n 1 /Odometry/header
rostopic info /Odometry
```

LiDAR 和 LIO 目标约 10 Hz；EKF 是另一层输出频率。**频率正常不等于没有积压**，必须同时看时间戳年龄是否随时间增长、是否唯一发布者。改变参数不会自动清空旧进程的 FIFO 队列；不能在空中通过重启解决。原点非零与同高地面之间持续漂移是不同问题，验收还要手持移动、静止回环和带完整负载测试。

## 6. 录制可排查初始化和漂移的包

需要初始化证据时，先运行 ROS master、MAVROS 和 Livox，让原始 IMU/雷达已有数据，再开始录制，然后启动单独 LIO 与 EKF。正常一键链之后才录制无法补回初始化之前的数据。

```bash
source ~/gcs_ws/devel/setup.bash
TEST_DIR="$HOME/gcs_ws/log/lio_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$TEST_DIR"
rosparam dump "$TEST_DIR/runtime_before.yaml"
rosbag record --lz4 --split --size=1024 -b 256 -O "$TEST_DIR/lio.bag" \
  /livox/lidar /mavros/imu/data_raw /mavros/imu/data \
  /Odometry /cloud_registered /cloud_registered_body /path \
  /ekf/ekf_odom /mavros/local_position/odom /mavros/local_position/pose \
  /mavros/state /mavros/extended_state /mavros/vision_pose/pose \
  /mavros/timesync_status /mavros/time_reference /tf /tf_static /rosout /rosout_agg
```

录制终端占用期间，在另一个终端使用打印/保存的同一目录，LIO/EKF 启动后再次 `rosparam dump <同一目录>/runtime_running.yaml`。上面命令不包含 Planner 障碍地图；任务飞行请用[任务全流程](HYBRID_MISSION_FLIGHT_AND_BAG_RUNBOOK.md)的 `record_diff_planner_debug.py`，需要原始雷达加 `--raw-lidar`，地图仍由该录包器涵盖。不要为了性能验收同时开多套重型录制和过长 RViz 点云累积。

### 隔离回放（禁止回送实机控制链）

每个回放终端均先执行：

```bash
source ~/gcs_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11321
```

终端 1：`roscore -p 11321`。

终端 2：

```bash
rosparam set /use_sim_time true
rviz -d "$(rospack find diff_planner)/launch/include/lio_cloud_odom.rviz"
```

终端 3：将实际文件路径填入，按空格开始：

```bash
rosbag play --clock --pause /实际目录/lio_*.bag
```

**回放 master 不启动 MAVROS、PX4Ctrl、视觉桥或任务脚本，也不要连接实机 master。** 此方法显示录包已有结果，不重新计算 LIO；旧包的 native 话题须在 RViz 选择原话题。包里没有录到的点云/地图不能靠 RViz 恢复。

## 7. 历史数据和本次整理边界

选定检查点 `1dbfede` 的 90 秒地面记录：输入 10.000 Hz、输出 10.001 Hz；消息年龄中位数 22.9 ms、P95 33.5 ms；LIO CPU 86.4%（单核 100% 口径），系统约 47.3%。这是当时负载下的测量，不代表飞行精度、长时间运动或 YOLO/Planner/Sunshine 全开后保证 10 Hz。本次文件迁移不将这些旧数据冒充新验收。

- 精确历史源码差异与测量元数据保留在 [检查点](checkpoints/faster_lio_20261117/README.md)；不要再次向当前源码套用归档补丁。
- 删除的是 10 份过时 Faster-LIO/高度阶段性调试说明；标定、飞行、地图、相机、舵机和 Git 手册保留。
- 整理前备份：`~/gcs_ws/log/maintenance/faster_lio_consolidate_20261117_221001/`，包含文档/入口备份和外部运行源码备份；历史说明也可从 Git 检查点 `1dbfede` 查看。
- 编译产物留在 `gcs_ws/build`、`gcs_ws/devel`；bag、日志不入 Git。旧外部工作区保留至地面切换与验证完成，不会自动删除。

## 8. MAXN 下重新测试 0.15 / 0.30（2026-11-20）

读取 `nvpmodel -q` 确认为 MAXN。CPU 仍为 `schedutil` 动态调频，读取到的 CPU 频率上限为 1984 MHz；本次没有执行锁频或修改系统功耗设置。以下两档均在 MAXN 下测量，不能据此推算相对旧功耗模式的提升比例。

使用 `gcs_ws` 中的程序，雷达输入约 10 Hz、MAVROS 原始 IMU 约 170 Hz，每档初始化后预热 20 秒。里程计发布到隔离的 `/maxn_lio_test/odometry`，没有启动外部 EKF、视觉位姿桥、PX4Ctrl、Planner、YOLO 或 RViz，也没有解锁、发送目标或飞行。保留了桌面/远程连接等已有后台负载；测试期间还出现了新的控制台节点，两档后台负载并非严格一致。

| 参数 surf / map（m） | 有效统计时长 | 输出频率 | 消息年龄中位数 / P95 | LIO 平均 CPU | 全机平均 CPU |
|---|---|---|---|---|---|
| 0.20 / 0.40 | 60 s | 10.0001 Hz | 25.62 / 29.34 ms | 105.7% | 33.4% |
| 0.15 / 0.30 | 120 s | 9.9994 Hz | 29.91 / 37.32 ms | 141.6% | 37.6% |

LIO CPU 使用“单核满载为 100%”口径，141.6% 约等于 1.42 个核的累计占用；全机 CPU 是所有核的归一化利用率。消息年龄为接收时 ROS 时间减 `header.stamp`，不是完整 EKF/控制链延迟，也不是直接测得的内部队列等待时间。

`0.15 / 0.30` 的 120 秒中收到 1200 帧，每个 10 秒窗口都是 100 帧，没有非递增时间戳；最长接收间隔为 125.40 ms。首末 10 秒的消息年龄中位数从 23.89 ms 升到 35.11 ms，**频率达标，但延迟存在约 11 ms 的上升，不能宣称长时间完全无积压**。后续 5 分钟加测在启动前检测到新出现的 `/gcs_console_*` 节点，因此主动拒绝启动，未干扰该控制台。

2026-11-21 更新：11-20 事故后已把默认值回退为 `0.20 / 0.40`，给 CPU 留出余量；`0.15 / 0.30` 只在整机空载时测过 10 Hz，与 YOLO/录包同时运行时没有余量。

（以下为 11-20 当时的测试记录）当前 YAML 当时为 `0.15 / 0.30` 试验档；`point_filter_num=3`、`max_iteration=3`、无限制 IVox、标定外参和时间补偿均未更改。YAML 不需要重新编译，但只对下一次启动生效，不会热更新已有进程。要回到上一个档位，将这两个值恢复为 `0.20 / 0.40`。

**本次只证明当前地面短时负载下能达到 10 Hz，不代表运动精度或 Planner + YOLO + 远程连接全开时已验收。** 下一步应在地面锁桨状态做手持运动和完整负载长时验证，同时观察消息年龄，而不只看 `rostopic hz`。

原始记录：`~/gcs_ws/log/performance/maxn_lio_20261120_163733/`，包含两档配置、`*_result.json`、逐秒 CPU/频率/RSS 样本、节点日志、`tegrastats.log`、测试脚本，以及 `aborted_long_test/` 中的加测中止原因和功耗模式查询记录。它们是日志，不入 Git。
