# faster-lio 时间偏移支持与外参迁移记录

> 日期：2026-11-13  
> 仓库：`src/Diff-Planner/src/realflight_modules/faster-lio`（重力对齐版 faster-lio）  
> 隔离编译空间：`~/fasterlio_ws`（不参与竞赛工作空间的 `catkin_make`）  
> 结论先行：**代码、配置和编译已完成；没有任何飞行或手持实测验证。**

## 1. 背景与目标

当前飞行定位用的是 `FAST_LIO/config/mid360_px4.yaml`，它依赖 LI-Init 的
`time_offset_lidar_to_imu: -0.068406` 才能把 LiDAR 时钟和飞控 IMU 时钟对齐。

重力对齐版 faster-lio 原本只提供粗粒度自同步 `time_sync_en`
（|lidar−imu| > 1 s 时才触发，并直接把 `timediff` 设成 `lidar + 0.1 − imu`），
没有 FAST-LIO 那种静态标定偏移，所以把标定结果抄过去也无法生效。

本次工作：

1. 给 faster-lio 增加与 FAST-LIO 同约定、同方向的静态时间偏移参数；
2. 把 LI-Init 的外参和时间偏移抄进一份新的飞行配置；
3. 在隔离空间编译并做启动冒烟验证；
4. 记录接入前尚未解决的问题（节点名、话题名、驱动冲突）。

## 2. 代码修改

修改位置：`faster-lio/include/laser_mapping.h`、
`faster-lio/src/laser_mapping.cc`。

### 2.1 成员变量

```cpp
// laser_mapping.h
bool time_sync_en_ = false;
// Static LiDAR-to-IMU time offset from offline calibration (e.g. LiDAR_IMU_Init),
// same convention as FAST-LIO: imu_stamp_corrected = imu_stamp - time_offset.
double time_offset_lidar_to_imu_ = 0.0;
```

### 2.2 参数读取（两条路径都改）

在线启动（ROS 参数服务器 / `rosparam`）：

```cpp
nh.param<bool>("common/time_sync_en", time_sync_en_, false);
nh.param<double>("common/time_offset_lidar_to_imu", time_offset_lidar_to_imu_, 0.0);
```

离线 `run_mapping_offline -c xxx.yaml`：

```cpp
time_sync_en_ = yaml["common"]["time_sync_en"].as<bool>();
time_offset_lidar_to_imu_ = yaml["common"]["time_offset_lidar_to_imu"] ?
                            yaml["common"]["time_offset_lidar_to_imu"].as<double>() : 0.0;
```

旧配置不含该键时取 `0.0`，行为与修改前完全一致。

### 2.3 IMU 时间戳应用

```cpp
void LaserMapping::IMUCallBack(const sensor_msgs::Imu::ConstPtr &msg_in) {
    publish_count_++;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    // Static calibration offset (FAST-LIO convention): shift the IMU clock onto the LiDAR clock.
    double stamp = msg_in->header.stamp.toSec() - time_offset_lidar_to_imu_;
    if (abs(timediff_lidar_wrt_imu_) > 0.1 && time_sync_en_) {
        stamp += timediff_lidar_wrt_imu_;
    }
    msg->header.stamp = ros::Time().fromSec(stamp);
    ...
}
```

方向说明：`FAST_LIO/src/laserMapping.cpp` 用的是
`msg->header.stamp = msg_in->header.stamp.toSec() - time_diff_lidar_to_imu`，
本补丁逐字保持同一约定，因此 LI-Init 结果里的
`Time Lag IMU to LiDAR = -0.068406` 可以原样填入，不需要取反。
`time_sync_en` 的粗同步仍然保留，两者同时开启时先做标定偏移、再叠加粗同步。

### 2.4 日志

两处参数加载完成后都会打印一行，便于起飞前核对：

```text
time_offset_lidar_to_imu: -0.068406 s
```

### 2.5 启动文件

`faster-lio/launch/mapping_mid360.launch` 增加三个 arg：

| arg | 默认 | 作用 |
| --- | --- | --- |
| `config_file` | `config/mid360.yaml` | 可切到 `config/mid360_px4.yaml` |
| `start_lidar` | `true` | `false` 时不再启动 `livox_lidar_publisher2`（飞行栈里驱动已在跑） |
| `mapping_node_name` | `laserMapping` | 避免与正在运行的 FAST-LIO 的 `/laserMapping` 冲突 |

## 3. 新配置：`faster-lio/config/mid360_px4.yaml`

与 `FAST_LIO/config/mid360_px4.yaml` 保持同一套标定数值（2026-11-13 重装后重新标定，见第 7 节）：

```yaml
common:
    lid_topic: "/livox/lidar"
    imu_topic: "/mavros/imu/data_raw"
    time_sync_en: false
    time_offset_lidar_to_imu: -0.07625571

mapping:
    extrinsic_est_en: false
    extrinsic_T: [0.032600, 0.027755, 0.086446]
    extrinsic_R: [ 0.856373, -0.003504,  0.516346,
                   0.007575,  0.999955, -0.005777,
                  -0.516302,  0.008859,  0.856361]
    imu_body_T: [0.0, 0.0, 0.0]
    imu_body_R: [1, 0, 0, 0, 1, 0, 0, 0, 1]
```

要点：

- 标定时间：2026-11-13（重装后重新标定，`Final Result`），替换了原先
  2026-10-17 18:14:23 的 `Refinement result`
  （两次记录见 `docs/LIDAR_IMU_EXTRINSIC_CALIBRATION.md` 第 14、15 节）；
- 这组数值同时写入了 FAST-LIO 和 faster-lio 两份配置，两处必须保持一致；
- `extrinsic_est_en: false`：既然已有离线外参，就不让在线估计把它拉走；
- `imu_body_R/T` 为单位阵：faster-lio 会额外发布 `/laserMapping/odometry_body`，
  本机 PX4/Mavros IMU 坐标系即机体系；
- `blind` 取 `0.5`，与 FAST-LIO 飞行配置一致；
- `pcd_save_en: false`、`path_save_en: false`：避免退出时写 PCD/轨迹污染工作目录。

## 4. 编译与验证

### 4.1 编译

在隔离空间编译，避免影响 `~/gcs_ws` 的竞赛构建：

```bash
cd ~/fasterlio_ws
source /opt/ros/noetic/setup.bash
catkin_make -j2 -DCMAKE_BUILD_TYPE=Release
```

结果：`libfaster_lio.so`、`run_mapping_online`、`run_mapping_offline` 均编译通过。

### 4.2 启动冒烟验证（隔离 master，不接硬件）

在 `127.0.0.1:11412` 起独立 roscore，加载 `mid360_px4.yaml` 后运行节点：

```text
I1113 18:02:50.524079 laser_mapping.cc:114] imu_body_R:
I1113 18:02:50.524791 laser_mapping.cc:115] imu_body_T: 0 0 0
I1113 18:02:50.524830 laser_mapping.cc:116] time_offset_lidar_to_imu: -0.068406 s
I1113 18:02:50.524853 laser_mapping.cc:121] Using AVIA Lidar
```

无 `bad conversion`、无 `Invalid imu_body_*` 报错，说明参数被正确加载。

### 4.3 启动文件参数检查

```bash
roslaunch faster_lio mapping_mid360.launch --dump-params \
  config_file:=$(rospack find faster_lio)/config/mid360_px4.yaml \
  start_lidar:=false mapping_node_name:=laserMapping_faster
```

输出包含 `/common/time_offset_lidar_to_imu: -0.068406`、
`/common/imu_topic: /mavros/imu/data_raw`；
不带参数时仍加载 `mid360.yaml`（`/mavros/imu/data`、无时间偏移），默认行为未变。

### 4.4 尚未验证的部分

- 没有接入真实 LiDAR/IMU 数据；
- 没有比较 faster-lio 与 FAST-LIO 在同一段数据上的轨迹差异；
- **没有飞行**，也没有手持走动实测；
- 时间偏移方向只做了代码约定层面的核对，没有用实际数据验证收敛效果。

## 5. 接入飞行栈前必须处理

1. **节点名冲突**：faster-lio 节点默认也叫 `laserMapping`，与 Fast-LIO 相同。
   切换时必须停掉 Fast-LIO，或用 `mapping_node_name:=` 改名。
2. **话题名不同**：faster-lio 输出
   `/laserMapping/odometry`（IMU 系）、`/laserMapping/odometry_body`、
   `/laserMapping/cloud_registered`，不是 Fast-LIO 的 `/Odometry`、`/cloud_registered`。
   EKF 配置（`ekf/ekf_fastlio2.launch`）订阅的话题需要同时确认；
   两者输出的都是 IMU/body 位姿，因此 EKF 内的 `Rr_i`、`tr_i` 仍应保持单位/零，
   **不能**再重复应用一次 LiDAR→IMU 外参。
3. **驱动冲突**：launch 默认会启动 `livox_ros_driver2` 的 `livox_lidar_publisher2`，
   与飞行栈里 `livox_ros_driver` 的驱动同名节点冲突。接飞行时用
   `start_lidar:=false`，并确认已有驱动发布的是 `/livox/lidar`。
   （`livox_ros_driver2` 的 `CustomMsg` 与 `livox_ros_driver` 的 `CustomMsg`
   字段和 md5 相同，实测可以互通。）
4. **任务脚本话题**：`hybrid_mission.yaml` 的 `topics.lio`、`topics.cloud`
   指向 `/Odometry`、`/cloud_registered`，切换时要同步确认或做话题重映射。
5. **未测飞行**：在完成接入和静止/手持验证之前，不要用 faster-lio 执行带桨飞行。

## 6. 后续步骤（计划内）

用户计划**重新标定外参**。重新标定后需要同步两处，数值必须一致：

1. `~/gcs_ws/src/FAST_LIO/config/mid360_px4.yaml`（当前飞行用的 FAST-LIO）；
2. `~/gcs_ws/src/Diff-Planner/src/realflight_modules/faster-lio/faster-lio/config/mid360_px4.yaml`。

流程仍按 `docs/LIDAR_IMU_EXTRINSIC_CALIBRATION.md`：先备份，采用
`Refinement result`，检查 `det(R)`、正交性、重力模长、重复性，
时间偏移取同一份结果的 `Time Lag IMU to LiDAR`，**不要取反**。

写入后建议的验证顺序（全部无桨）：

1. `rosparam get /common/time_offset_lidar_to_imu` 核对；
2. 静止观察点云地图是否分层/漂移；
3. 手持原地旋转，检查墙面重影；
4. 前后左右上下平移，检查里程计方向；
5. 与 FAST-LIO 同段数据对比轨迹，再考虑接入 EKF 和任务栈。

## 7. 2026-11-13 重装后重新标定

### 7.1 第一次结果（未通过，已作废）

飞机重装后重新跑了一次 LiDAR_IMU_Init，`Final Result`：

```text
Rotation LiDAR to IMU    =  33.629932   0.685810 -146.925518 deg
Translation LiDAR to IMU =   1.365180   0.919982  -0.100021 m
Time Lag IMU to LiDAR    =  -0.23176084 s
Bias of Gyroscope        =   0.022484  -0.046725   0.160431 rad/s
Bias of Accelerometer    =  -0.168845  -0.010041  -0.120353 m/s^2
Gravity in World Frame   =   2.860045  -5.882787  -6.978986 m/s^2
```

换算与检查（`R = Rz(yaw)·Ry(pitch)·Rx(roll)`，与 2026-10-17 结果同一约定，已核对复现）：

```text
R_LI = [-0.837902  0.448836 -0.310590
        -0.545690 -0.701331  0.458647
        -0.011969  0.553787  0.832572]
det(R) = 1.000000, ||RᵀR − I|| ≈ 2.3e-16
|T_LI| = 1.649 m
gravity 与 −Z 夹角 = 43.1°, |g| = 9.565 m/s²
陀螺 Z 零偏 = 0.160 rad/s = 9.2 deg/s
与 2026-10-17 在用的旋转差 = 159.5°
时间偏移变化 = 163.4 ms
```

**结论：未通过验收，禁止用于飞行。** 依据：

- 平移模长 `1.649 m`，在这套机架的 LiDAR/飞控实际安装距离上不可能（在用值是 `0.078 m`）；
- 重力方向偏离 `-Z` 约 `43°`，说明初始化时整机不水平或初始化阶段在动；
- 陀螺 Z 零偏 `9.2 deg/s` 远超正常量级；
- 时间偏移从 `-68 ms` 跳到 `-232 ms`，不是毫秒级变化；
- 与在用外参旋转差近 `160°`。

最可能的原因：初始化静止段不足/机身不水平、激励过程中 LiDAR 或飞控没有刚性固定、
或在线 refinement 发散。

处理方式：该结果当时只写入过一份台架配置
（`faster-lio/config/mid360_px4_recal_20261113.yaml`），未进入任何飞行配置；
第二次标定通过后该文件已删除，结果只保留在本节记录里。

### 7.2 第二次结果（通过，已采用）

随后按"先校准 PX4 飞控传感器、再重新标定"的流程重跑 LiDAR_IMU_Init，`Final Result`：

```text
Rotation LiDAR to IMU    =  0.592708  31.084524  0.506819 deg
Translation LiDAR to IMU =  0.032600  0.027755  0.086446 m
Time Lag IMU to LiDAR    =  -0.07625571 s
Bias of Gyroscope        =  0.007520  0.003108 -0.002814 rad/s
Bias of Accelerometer    = -0.014924  0.027314 -0.001732 m/s^2
Gravity in World Frame   =  4.754746 -0.016617 -8.584941 m/s^2
```

换算与检查：

```text
R_LI = [ 0.856373 -0.003504  0.516346
         0.007575  0.999955 -0.005777
        -0.516302  0.008859  0.856361]
det(R) = 1.000000, ||RᵀR − I|| ≈ 3.4e-16
|T_LI| = 0.0965 m（2026-10-17 为 0.0780 m，差 7.0 cm）
与 2026-10-17 在用旋转差 = 1.14°
时间偏移 = -76.26 ms（比旧值差 7.8 ms）
陀螺零偏模长 = 0.49 deg/s
加计零偏模长 = 0.033 m/s^2
```

重力方向的解释与自洽性核对：LI-Init 的"世界系"是**初始 LiDAR 系**
（`LI_init.cpp:470`，`Grav_L0 = Rot_Grav_wrt_Init_Lidar × STD_GRAV`），
所以 `(4.755, -0.017, -8.585)` 本身带约 29° 倾斜是正常的——雷达按约 31° 俯仰安装。
用新外参把该重力旋转到 IMU 系得到 `(-0.361, 0.069, -9.807)`，
与竖直方向只差 **2.1°**，与"雷达倾斜 31°、IMU 基本水平"的几何关系一致。
（第一次失败结果的同样换算无法自洽。）

**结论：通过验收，已采用。**

- 写入 `faster-lio/config/mid360_px4.yaml` 与
  `FAST_LIO/config/mid360_px4.yaml` 两份配置，`time_offset_lidar_to_imu`
  同步改为 `-0.07625571`；
- `~/fasterlio_ws` 已同步，`roslaunch --dump-params` 确认新值生效；
- 尚未做静止/手持实测，也未飞过；飞行前按第 9 节做点云与里程计检查；
- 平移比旧值差 7.0 cm，属于重装后的预期变化，但仍建议用尺量核对安装距离；
- 目前只有一次通过的结果，条件允许时再标 1–2 次确认重复性。

## 8. 回滚

- 代码：删除 `time_offset_lidar_to_imu_` 相关三处改动即可回到上游行为
  （默认 `0.0`，等价于未启用）。
- 配置：不使用 `config/mid360_px4.yaml`，继续用 `config/mid360.yaml`。
- 启动文件：`config_file`、`start_lidar`、`mapping_node_name` 都有默认值，
  旧命令行不需要修改。
