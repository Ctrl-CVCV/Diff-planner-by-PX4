# Faster-LIO 官方基线与标定外参

> 2026-11-17：用户观察到 64 点限额后坐标误差增大，现已撤销整套每格限点实现及其参数读取，回到检查点 `3d68512632b7c94e15214e74504b224af0f53e01` 的算法行为，不是仅把参数设成 0。当前降采样为 `filter_size_surf: 0.20`、`filter_size_map: 0.40`；外参、在线外参开关、桥接和线程代码不变。运行性能须实测，不预先保证 10 Hz。

官方定位稳定后，追加外部 EKF、可选 MAVROS 视觉桥和三路 Z 数值对比的方法见 [EKF 桥接地面测试](FASTER_LIO_EKF_BRIDGE_TEST.md)。新入口不改官方算法，也不依赖旧一键脚本。

## 恢复范围

本次从现有 `origin` 拉取并核实 `DifferentialRobotics/faster-lio` 的官方 `main`，版本为 `c8d635c4689f5ebd89a9532411254c93eaa35adc`。这是设备配套版，包含其原有重力对齐和 MID360 适配，不是切换到 `gaoxiang12/faster-lio` 作者原版。

仅恢复 `realflight_modules/faster-lio/faster-lio`。算法源码、头文件、IVox、入口程序和构建文件与该官方提交一致；撤销本地时间补偿、每格点数限制、线程限制、性能日志和源码默认关闭外参估计等补丁。不改变 EKF、雷达驱动、PX4Ctrl、Planner、任务脚本和视觉桥；不切换分支、改写 HEAD、提交或推送。

唯一配置差异在 `faster-lio/config/mid360.yaml`：IMU 使用标定对应的 `/mavros/imu/data_raw`，并保留此前标定：

```yaml
extrinsic_T: [0.032600, 0.027755, 0.086446]
extrinsic_R: [0.856373, -0.003504, 0.516346,
              0.007575, 0.999955, -0.005777,
              -0.516302, 0.008859, 0.856361]
imu_body_T: [0.0, 0.0, 0.0]
imu_body_R: [1, 0, 0, 0, 1, 0, 0, 0, 1]
```

检查点原本使用官方 MID360 滤波尺寸 `0.5/0.5`；当前地面对照使用 `0.20/0.40`，`point_filter_num: 3` 不变。此配置原本就有 `mapping/extrinsic_est_en: false`；源码缺省恢复官方的 `true`，因此必须加载本配置，不能以源码缺省已关闭为前提启动。

## 备份与编译

恢复前的完整源码、补丁、配置、状态和二进制备份在 `~/gcs_ws/log/faster_lio_before_official_restore_gbJ6Pmz3/`。`source_before.tar.gz` 是完整源码备份，`calibration_before.yaml` 保存原配置，`build_official.log` 是本次编译日志。恢复备份会覆盖文件，应先保存后续新改动。

按官方 README 的 `catkin_make` 流程重新配置并构建，仅构建本包、限制编译内存占用：

```bash
source /opt/ros/noetic/setup.bash
source ~/gcs_ws/devel/setup.bash
catkin_make -C ~/gcs_ws --pkg faster_lio --force-cmake \
  -DCMAKE_BUILD_TYPE=Release -j1 -l2
```

编译不是飞行验证，也不证明 Z 漂移已修复。

## 启动接口变化

自定义的 `mid360_px4.yaml` 和官方 launch 中原先加入的 `config_file`、`start_lidar`、`mapping_node_name`、`odom_topic` 等参数已撤销。旧 `start_fastlio2_ekf.sh` 的 Faster-LIO 分支仍传这些参数，本次没有修改该脚本，暂不能照旧使用。

官方 `mapping_mid360.launch` 会自行启动驱动，并要求环境变量 `BD_LIST`，不能与现有雷达驱动重复启动。为单独核验官方定位，下面直接运行官方可执行程序，用 ROS 参数和 remap 保持原点云/里程计显示接口，不改官方算法。

## 地面定位测试

保持未解锁，不运行任务脚本、PX4Ctrl 自动起飞、Planner 或视觉位姿桥。保留 MAVROS 提供飞控 IMU，停止旧 FAST-LIO2/Faster-LIO 定位实例，避免同名里程计或 TF 多源发布。不要在飞行时停止节点。

每个终端先 source 上面的 ROS 和工作区环境。ROS master 未运行时另开 `roscore`；雷达驱动未运行时才启动 `roslaunch livox_ros_driver2 msg_MID360.launch`。保留现有且端口已核对的 MAVROS，不另开第二个 MAVROS。确认 `/livox/lidar` 和 `/mavros/imu/data_raw` 均有数据。

需要观察初始化时，在另一个终端先录包，再启动定位：

```bash
TEST_DIR="$HOME/gcs_ws/log/faster_lio_official_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$TEST_DIR"
rosbag record --lz4 --repeat-latched -O "$TEST_DIR/height.bag" \
  /livox/lidar /livox/imu /mavros/imu/data_raw /mavros/imu/data \
  /mavros/state /Odometry /cloud_registered /cloud_registered_body \
  /path /tf /tf_static /rosout /rosout_agg
```

定位终端加载标定配置，核对在线估计关闭和外参正确，再启动。额外 ROS 参数仅统一显示坐标名并关闭测试文件保存，不添加时间/高度补偿：

```bash
rosparam load "$(rospack find faster_lio)/config/mid360.yaml"
rosparam set /publish/tf_world_frame camera_init
rosparam set /pcd_save/pcd_save_en false
rosparam set /path_save_en false
rosparam get /mapping/extrinsic_est_en
rosparam get /mapping/extrinsic_T
rosparam get /mapping/extrinsic_R
rosrun faster_lio run_mapping_online __name:=laserMapping_official \
  /laserMapping/odometry:=/Odometry \
  /laserMapping/cloud_registered:=/cloud_registered \
  /laserMapping/cloud_registered_body:=/cloud_registered_body \
  /laserMapping/path:=/path
```

查看点云和里程计，使用现有工作区显示配置：

```bash
rviz -d "$(rospack find diff_planner)/launch/include/lio_cloud_odom.rviz"
```

## 此前无每格限额、0.15/0.30 地面对照

本节保留上一轮参数；当前参数与测试结果见末节的 `0.20/0.40` 测试。

已删除 `ivox_max_points_per_grid` 配置、参数读取和限点分支，恢复原来的 IVox 插入行为。它仍有官方原有的网格容量管理，不代表整个地图无限占用内存。此处只修改 Faster-LIO，不修改 FAST-LIO2。

```yaml
filter_size_surf: 0.15
filter_size_map: 0.30
point_filter_num: 3
```

源码回退需要重新编译；随后必须在未解锁、已落地状态停止旧定位实例，重新加载本配置并重启 Faster-LIO。只修改 ROS 参数不会更新已经初始化的定位对象。参数服务器若残留 `/ivox_max_points_per_grid`，新源码不会读取它，不能据此判断二进制是否已更新。

使用上文启动命令及话题重映射时，在两个终端分别观察：

```bash
rostopic hz -w 300 /Odometry
rostopic delay -w 300 /Odometry
```

若直接使用官方默认输出，两条命令的话题都改为 `/laserMapping/odometry`。输入雷达也应核实频率，例如 `rostopic hz /livox/lidar`。应在编译结束后测试，避免把编译竞争算进定位性能；至少观察一分钟，并继续手持沿相同路线往返、在同高度静置。不要在飞行中做本测试。

频率接近 10 Hz 还不够：同时确认消息时间戳持续前进、延迟没有持续增长、没有旧数据成批补发，并比较运动重影和同高度静止 Z 差。静置跑满不等于运动或 PX4Ctrl、Planner、YOLO、远程桌面全开时也能跑满，应注明实际负载。没有实测前不保证本参数能持续 10 Hz。

RViz 的 Fixed Frame 为 `camera_init`，点云 `/cloud_registered`，里程计 `/Odometry`，轨迹 `/path`。每次对照重新初始化地图，不用上一次 RViz 累积的历史点云判断新算法效果。

## 本次实测与双工作区注意事项

2026-11-17 检查发现，当前 `/laserMapping_faster` 实际运行的是 `~/fasterlio_ws/devel/lib/faster_lio/run_mapping_online`，加载的库也来自 `~/fasterlio_ws`，并非本次重新编译的 `~/gcs_ws`。该副本源码本身没有每格限点实现，当前参数已经是 `0.15/0.30`、`point_filter_num: 3`、在线外参估计关闭；另外仍有旧的静态时间补偿 `-0.07625571` 秒。本次没有修改、停止或替换该副本，不能将它的输出冒称为新编译回退版的实测结果，也不能直接把此前观察归因于另一个工作区的改动。

编译结束后，对这个现有节点进行 60 秒只读测量。订阅使用 `AnyMsg`，仅读取消息头，不在 Python 中反序列化点云：

| 指标 | 现有 fasterlio_ws 节点实测 |
| --- | --- |
| 雷达消息实际到达频率 | 10.000 Hz |
| 里程计实际到达频率 | 5.320 Hz |
| 雷达时间戳延迟中位数 | 0.102 秒 |
| 里程计首/末 10 秒延迟中位数 | 408.881 / 430.927 秒 |
| 定位进程 CPU，单核满载记为 100% | 438.8% |
| 全机 CPU 忙碌比例 | 96.9% |

这说明现有节点在当时负载下没有跑满 10 Hz，且正在处理严重积压的旧数据。虽然相邻里程计消息的时间戳间隔仍接近 0.1 秒，但 60 秒实际只输出 318 条，不能据此称为实时 10 Hz。当前结果不能用于飞行，也未完成运动精度或整套负载验收。

低开销测量原始结果在 `/tmp/faster_lio_uncapped_015_030_live_low_overhead.json`；回退版构建日志在 `/tmp/faster_lio_uncapped_015_030_build.log`。这些临时文件不作为永久备份。

下一次地面复测应先确认使用哪一个工作区，停止旧定位实例后重新初始化，而不是让旧队列继续积压或同时启动两个定位发布源。上述 0.15/0.30 轮次没有替换节点；`gcs_ws` 回退版已编译通过，但该轮未测试其新二进制。后续重启测试见下一节。切换前确认已落地且未解锁，并考虑定位原点重置对现有 EKF 的影响，之后重新核对整条链路。

## 当前 0.20/0.40 地面测试

2026-11-17 按用户要求，将 `gcs_ws` 的 `mid360.yaml` 与实际运行工作区 `~/fasterlio_ws/src/faster-lio/config/mid360_px4.yaml` 都改为：

```yaml
filter_size_surf: 0.20
filter_size_map: 0.40
```

只改变这两个参数，保留 `point_filter_num: 3`、标定外参、在线外参估计关闭和无限每格点数的实现。本轮仍使用 `fasterlio_ws` 的原有可执行程序和动态库，未切换成 `gcs_ws` 二进制，且保留该副本原有的 `-0.07625571` 秒时间补偿；本结果不能作为无时间补偿版本的验收。

重启前已重新确认 FCU 连接、未解锁、落地且没有飞行任务节点。仅正常关闭旧 `/laserMapping_faster`，等待进程退出，再设置两个 ROS 参数并重启同一二进制，清空旧队列；没有切换飞控模式、解锁、发送航点或重启 MAVROS、EKF。定位原点已重新初始化，飞行前必须重新初始化并核对整套 EKF 链路，不可直接沿用旧原点状态起飞。

重启后用不反序列化点云的只读监测连续观察 90.0 秒：

| 指标 | 本轮实测 |
| --- | --- |
| 雷达实际到达频率 | 10.000 Hz |
| 里程计实际到达频率 | 10.001 Hz |
| 里程计消息数 | 900 |
| 里程计时间戳延迟中位数 | 22.9 ms |
| 里程计时间戳延迟 P95 | 33.5 ms |
| 首/末 10 秒延迟中位数 | 24.5 / 23.7 ms |
| 定位进程 CPU，单核满载记为 100% | 86.4% |
| 全机 CPU 忙碌比例 | 47.3% |

本轮现场负载下跑满 10 Hz，未观察到时间戳延迟持续增长。延迟是 ROS 接收时刻减里程计消息头时间，不等于单帧计算耗时。尚未验证运动精度、长时间地图增长和 PX4Ctrl、Planner、YOLO 全开负载。由于同时重启清空了旧队列，本轮与上一轮不是严格单变量 A/B，不能断言全部改善只来自这两个参数，也不能据此排除原点、时间补偿或地图增长因素。

参数改动无需重新编译。`fasterlio_ws` 原配置备份在 `/tmp/mid360_px4_before_020_040.yaml`；原始测量结果 `/tmp/faster_lio_uncapped_020_040_live.json`，当前节点日志 `/tmp/faster_lio_020_040_runtime.log`。这些临时文件不是永久备份。

测试后的定位节点保持运行，由本次测试进程启动，不再是原 roslaunch 的子进程。需要停止时，仅在落地未解锁后执行 `rosnode kill /laserMapping_faster`；重新启动前避免重复定位发布源。

## Git 检查点归档

用户当前选定的最佳地面测试版本已归档到 [2026-11-17 检查点](checkpoints/faster_lio_20261117/README.md)。其中保存实际运行 `fasterlio_ws` 相对于本仓库官方恢复版的源码/配置补丁、文件校验值和 90 秒原始测量结果，避免只保存另一个工作区而无法复现本次配置。
