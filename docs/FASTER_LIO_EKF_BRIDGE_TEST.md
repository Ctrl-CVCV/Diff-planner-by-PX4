# 官方 Faster-LIO → 外部 EKF → MAVROS 地面对比

更新：2026-11-17。只做未解锁的地面定位测试，不启动 PX4Ctrl、Planner 或任务脚本。

## 1. 三种估计值不是一回事

```text
雷达 + 飞控原始 IMU → 官方 Faster-LIO 内部 IEKF → /laserMapping/odometry
                                                      ↓
                                    外部 EKF + /mavros/imu/data
                                                      ↓
                                              /ekf/ekf_odom
                                                      ↓
                                           ekf_vision_bridge.py
                                                      ↓
                                          /mavros/vision_pose/pose
                                                      ↓
                                MAVROS → PX4 估计器（是否融合由飞控决定）
                                                      ↓
                                         /mavros/local_position/pose
```

- 看 Faster-LIO 自己的 IEKF：看 `/laserMapping/odometry`。若启动时 remap 到 `/Odometry`，改看后者。
- 看工作区外部 EKF 的结果：看 `/ekf/ekf_odom`，它不是 Faster-LIO 内部 IEKF。
- 看飞控最终定位：看 `/mavros/local_position/pose`；不能仅凭收到 ROS 视觉话题就认定飞控已融合。

本次只增加独立 launch 并参数化既有外部 EKF 接口，不修改官方 Faster-LIO 算法、滤波参数或标定外参。外部 EKF 沿用现有配置：`Rr_i` 为单位四元数，`tr_i`、`imu_trans_*`、`offset_p*` 均为零，不再次施加雷达安装倾角。桥接原样复制位置、姿态和测量时间戳，不交换坐标轴、不加 Z 高度、不加时间补偿。

注意：既有外部 EKF 输出的 `frame_id` 固定为 `world`。若上游叫 `camera_init`，这不代表自动完成了 TF 转换；数值是否共用同一原点和轴向需要实测核对。不要为了在 RViz 中重合而随意添加静态 TF。飞控局部坐标原点也可能不同，对比时同时看各自相对初始值的变化量。

## 2. 先检查当前运行状态

每个终端先执行：

```bash
source /opt/ros/noetic/setup.bash
source ~/gcs_ws/devel/setup.bash
```

保留当前已经验证稳定的 Faster-LIO、雷达驱动和 MAVROS，不重复启动它们。检查：

```bash
rostopic echo -n 1 /mavros/state
rostopic hz /laserMapping/odometry
rostopic hz /mavros/imu/data
rostopic info /ekf/ekf_odom
rostopic info /mavros/vision_pose/pose
```

`hz` 观察后按 Ctrl+C。必须确认 `armed: False`，并且 LIO 与 IMU 持续有数据；无飞控状态或话题不更新时先恢复上游，不进行带飞控视觉输入的测试。没有 EKF/视觉话题时，`rostopic info` 报未知话题是正常的。

如果 `/ekf/ekf_odom` 已有发布者，不再启动第二个 EKF；后面使用 `start_ekf:=false`。如果视觉话题已有发布者，不再启动第二个视觉桥。本 launch 不会自动检测或消除重复节点。

## 3. 先只启动外部 EKF

当前官方默认里程计接口：

```bash
roslaunch diff_planner faster_lio_ekf_bridge.launch
```

如果 Faster-LIO 已按官方基线文档 remap 为 `/Odometry`，改用：

```bash
roslaunch diff_planner faster_lio_ekf_bridge.launch lio_odom_topic:=/Odometry
```

两条命令二选一。默认 `publish_vision:=false`，此时不会向飞控发送视觉位姿。这个 launch 不启动 LIO、驱动、MAVROS、PX4Ctrl 或 Planner，不解锁、不切模式、不修改 PX4 参数。

已有 EKF 源码包含前 1000 次输出调用不发布的启动逻辑，因此启动后可能需数秒才能看到 `/ekf/ekf_odom`；不能据此立刻判断故障。持续无输出时核对两个输入和日志，不重复开同名节点。

## 4. 不重启 EKF，追加视觉桥

保留第 3 节终端运行。在另一个已 source 的终端执行以下两种方式之一。

先只看桥接结果、不送给 MAVROS：

```bash
roslaunch diff_planner faster_lio_ekf_bridge.launch \
  start_ekf:=false publish_vision:=true vision_topic:=/debug/vision_pose
```

确认未解锁、数据正确且 MAVROS 视觉话题没有其他发布者后，可停止上述预览桥，再改用真实视觉输入：

```bash
roslaunch diff_planner faster_lio_ekf_bridge.launch \
  start_ekf:=false publish_vision:=true
```

桥接默认最多 50 Hz，丢弃超过 0.5 秒或无效的 EKF 位姿。这里只检查 EKF 输出时间戳，不能证明上游雷达里程计没有断更：外部 EKF 仍可能仅靠 IMU 预测，所以要同时观察 LIO 频率和延迟。

视觉输入可能改变 PX4 估计器的状态，不是对飞控完全无副作用。不要在飞行中切换测试，不为看 ROS 数值而修改飞控融合参数；确认是否融合需要结合对应固件的估计器配置与状态。不要连接真飞控回放旧包。

## 5. 看 XYZ、Z 曲线和时间延迟

不同终端分别查看：

```bash
rostopic echo /laserMapping/odometry/pose/pose/position
rostopic echo /ekf/ekf_odom/pose/pose/position
rostopic echo /mavros/local_position/pose/pose/position
```

连续画 Z 曲线（需要已安装 `rqt_plot`）：

```bash
rosrun rqt_plot rqt_plot \
  /laserMapping/odometry/pose/pose/position/z \
  /ekf/ekf_odom/pose/pose/position/z \
  /mavros/local_position/pose/pose/position/z
```

如果上游是 `/Odometry`，替换命令中的 `/laserMapping/odometry`。同高度移动前后各静置数秒，对比静止段的相对 Z 变化。若 LIO 稳定但外部 EKF 漂移，优先查外部 EKF/IMU 链路；两者稳定而飞控不同，进一步查视觉输入和飞控估计器，不能直接归因于桥接。

还应分终端观察 `rostopic hz /ekf/ekf_odom`、`rostopic delay /laserMapping/odometry` 和 `rostopic delay /ekf/ekf_odom`。不同原点造成的固定高度差与随运动持续增大的漂移应分开判断。

## 6. 录包保留完整对比链路

在地面移动测试前运行，测试完成后 Ctrl+C 等待索引写完：

```bash
TEST_DIR="$HOME/gcs_ws/log/faster_lio_ekf_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$TEST_DIR"
rosparam dump "$TEST_DIR/runtime_all.yaml"
rosbag record --lz4 --repeat-latched -O "$TEST_DIR/compare.bag" \
  /livox/lidar /livox/imu /mavros/imu/data_raw /mavros/imu/data \
  /laserMapping/odometry /Odometry /ekf/ekf_odom /ekf/cam_ekf_odom \
  /mavros/vision_pose/pose /debug/vision_pose \
  /mavros/local_position/pose /mavros/local_position/odom \
  /mavros/state /mavros/estimator_status \
  /laserMapping/cloud_registered /cloud_registered \
  /laserMapping/path /path /tf /tf_static /rosout /rosout_agg
```

同时列出官方和 remap 后的话题是为了兼容两种启动方式，未发布的话题不会凭空生成消息。需要初始化数据时，在启动 LIO/EKF 前开始录制；不要为了补录初始化在带飞控视觉输入的运行中直接重启 LIO。

离线回看应断开真飞控，在独立 ROS master 上仅运行 bag、RViz 或曲线工具，不运行本桥向真实 MAVROS 发旧位姿。点云与里程计 RViz 的基线启动方式见 [官方基线说明](FASTER_LIO_OFFICIAL_BASELINE.md)。
