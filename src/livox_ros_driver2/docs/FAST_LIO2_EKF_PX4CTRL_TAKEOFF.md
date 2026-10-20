# 配套标定笔记

LiDAR–IMU 外参与时间偏移重新标定流程见：[`LIDAR_IMU_EXTRINSIC_CALIBRATION.md`](LIDAR_IMU_EXTRINSIC_CALIBRATION.md)。

---

# FAST-LIO2 → EKF → PX4Ctrl 定点起飞全流程（ROS 1 Noetic）

> 适用工作空间：`~/gcs_ws`  
> 飞控：PX4 / Pixhawk 6C Mini  
> 激光雷达：Livox MID-360s  
> 定位：FAST-LIO2 + Diff-Planner 中的 EKF  
> 控制：Diff-Planner 中的 `px4ctrl`

## 0. 先说清楚：200 Hz 是“请求值”，不是保证值

`MAV_CMD_SET_MESSAGE_INTERVAL` 的第二个参数是消息间隔，单位为微秒：

- 50 Hz：`20000 us`
- 100 Hz：`10000 us`
- 200 Hz：`5000 us`
- 500 Hz：`2000 us`

重启 PX4、重启 MAVROS、USB 断开重连后，之前用 `mavcmd` 发出的请求可能失效，话题就会回到约 50 Hz。因此本仓库增加了自动保持节点：

```bash
roslaunch livox_ros_driver2 set_px4_imu_rate.launch rate_hz:=200
```

它会在 MAVROS 每次连接后重新请求，并每 30 秒刷新一次。但是，PX4 调度、MAVLink 带宽、串口/USB、MAVROS 和 Jetson 负载都可能使实测值低于 500 Hz。你以前请求 200 Hz 后实测约 170 Hz，说明链路已有实际上限。**px4ctrl 的硬性检查是 IMU 和融合里程计都稳定高于 100 Hz，而不是必须达到 500 Hz。**

如果请求 500 Hz 后出现丢包、连接不稳、其他 MAVLink 消息延迟，请改为：

```bash
roslaunch livox_ros_driver2 set_px4_imu_rate.launch rate_hz:=200
```

---

## 1. 飞行前安全规则

第一次联调必须遵守：

1. 参数检查、遥控器检查、方向检查时先拆桨。
2. 真正起飞时使用空旷区域，人员离机至少数米，并准备随时切回手动模式或执行紧急停桨。
3. 不允许带着 px4ctrl 的频率警告起飞。
4. 不允许在 `/ekf/ekf_odom` 漂移、跳变、方向错误时起飞。
5. 不允许低电量起飞。你之前日志显示：

   ```text
   Voltage=22.581, percentage=0.120
   ```

   即使电压显示 22.581 V，也必须先确认 6S 电池已充好、PX4 电池参数正确、QGC 电量显示可信。`12%` 状态下禁止起飞。
6. 当前配置为 `enable_auto_arm: true`。发送起飞命令后 px4ctrl 会先切 OFFBOARD，再自动解锁；**不要提前手动解锁。**

---

## 2. 当前已经确认的关键配置

### 2.1 FAST-LIO2

配置文件：

```text
~/gcs_ws/src/FAST_LIO/config/mid360_px4.yaml
```

当前关键话题：

```yaml
common:
  lid_topic: "/livox/lidar"
  imu_topic: "/mavros/imu/data_raw"
```

FAST-LIO2 原始里程计输出：

```text
/Odometry
```

### 2.2 EKF

启动文件：

```text
~/gcs_ws/src/Diff-Planner/src/realflight_modules/faster-lio/ekf_pose/launch/ekf_fastlio2.launch
```

输入与输出：

```text
/mavros/imu/data + /Odometry  →  /ekf/ekf_odom
```

FAST-LIO 已经在内部使用 LiDAR→IMU 外参，因此 EKF 中的 `Rr_i` 和 `tr_i` 保持单位旋转与零平移是合理的；不要把 LiDAR→IMU 外参重复应用一次。

### 2.3 PX4Ctrl

参数文件：

```text
~/gcs_ws/src/Diff-Planner/src/realflight_modules/px4ctrl/config/ctrl_param_fpv.yaml
```

当前关键配置：

```yaml
ctrl_freq_max: 200.0

auto_takeoff_land:
  enable: true
  enable_auto_arm: true
  no_RC: false
  takeoff_height: 0.5
  takeoff_land_speed: 0.15
```

`ctrl_freq_max: 200.0` 是 px4ctrl 主循环频率上限，不是设置飞控 IMU 输出频率的参数。先保持 200，不要为了追求“500”把控制循环也盲目改成 500。

---

## 3. 建议的终端启动顺序

每个终端都先执行：

```bash
source /opt/ros/noetic/setup.bash
source ~/gcs_ws/devel/setup.bash
```

roslaunch livox_ros_driver2 set_px4_imu_rate.launch rate_hz:=200
### 终端 1：启动 MAVROS

你此前使用过的连接命令是：

```bash
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600
```

连接后检查：

```bash
rostopic echo -n 1 /mavros/state
```

应看到：

```text
connected: True
```

此时先保持：

```text
armed: False
```

### 终端 2：自动请求并保持 PX4 IMU 消息频率

```bash
source ~/gcs_ws/devel/setup.bash
roslaunch livox_ros_driver2 set_px4_imu_rate.launch rate_hz:=200
```

该节点请求两种 MAVLink 消息：

- `31`：`ATTITUDE_QUATERNION`
- `105`：`HIGHRES_IMU`

如果只想临时执行一次，不使用自动保持节点，可运行：

```bash
rosrun mavros mavcmd long 511 31 2000 0 0 0 0 0
rosrun mavros mavcmd long 511 105 2000 0 0 0 0 0
```

但这两个一次性命令在重启或重连后可能失效。

另开终端测量，窗口不要太小：

```bash
rostopic hz -w 1000 /mavros/imu/data
rostopic hz -w 1000 /mavros/imu/data_raw
```

判定标准：

- 只有约 50 Hz：不要启动 px4ctrl 起飞；先查看终端 2 是否收到成功响应。
- 稳定高于 100 Hz：满足 px4ctrl 的最低频率检查。
- 约 160～200 Hz：可以接受，不必执着于显示 500 Hz。
- 频率剧烈抖动、MAVROS 掉线或其他消息明显延迟：把请求值降到 200 Hz。

可确认节点仍在运行：

```bash
rosnode list | grep px4_imu_rate_keeper
```

### 终端 3：启动 PTP（如果没有配置成系统服务）

```bash
sudo ptp4l -i eth0 -S -m -f \
  ~/gcs_ws/src/livox_ros_driver2/config/time_sync/ptp4l-mid360s.conf
```

保持该终端运行。若 PTP 已由 systemd 管理，则不要重复启动第二个 `ptp4l` 实例。

### 终端 4：启动 Livox MID-360

```bash
source ~/gcs_ws/devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360s.launch
```

检查：

```bash
rostopic hz /livox/lidar
rostopic echo -n 1 /livox/lidar/header
```

并可运行时间同步检查：

```bash
rosrun livox_ros_driver2 check_px4_livox_time_sync.py _duration:=30
```

### 终端 5：启动 FAST-LIO2

为减轻 Jetson 负载，第一次飞行建议先不启动 RViz：

```bash
source ~/gcs_ws/devel/setup.bash
roslaunch fast_lio mapping_mid360_px4.launch rviz:=false
```

检查 FAST-LIO2：

```bash
rostopic hz /Odometry
rostopic echo -n 1 /Odometry
rostopic hz /cloud_registered
```

注意：FAST-LIO2 的 `/Odometry` 可能跟随 LiDAR 帧率，约为 10 Hz。这并不等于最终送给 px4ctrl 的融合里程计频率。EKF 会使用高频 IMU 传播并输出更高频的 `/ekf/ekf_odom`。

静止放置无人机时，重点检查：

- 位置不能持续快速漂移；
- 速度应接近 0；
- 姿态不能跳变；
- 手动把机头朝前、抬高、向右移动时，坐标和姿态变化必须符合系统定义；
- 点云不能随着轻微晃动发生明显分层或撕裂。

### 终端 6：启动 EKF

```bash
source ~/gcs_ws/devel/setup.bash
roslaunch ekf ekf_fastlio2.launch
```

EKF 启动后先等待约 10 秒，再检查：

```bash
rostopic hz -w 1000 /ekf/ekf_odom
rostopic echo -n 1 /ekf/ekf_odom
```

再检查静止速度：

```bash
rostopic echo /ekf/ekf_odom/twist/twist/linear
```

起飞前必须满足：

- `/ekf/ekf_odom` 稳定高于 100 Hz；
- 静止时总速度最好小于 `0.1 m/s`；
- 位置没有持续发散；
- z 轴方向正确：向上移动时 z 增大；
- 时间戳持续前进，没有回跳。

一条命令同时检查两条关键频率：

```bash
rostopic hz /mavros/imu/data /ekf/ekf_odom
```

### 终端 7：检查遥控器、飞行模式和电池

查看 PX4 当前状态：

```bash
rostopic echo /mavros/state
```

查看着陆状态：

```bash
rostopic echo /mavros/extended_state
```

查看遥控器原始通道：

```bash
rostopic echo /mavros/rc/in/channels
```

当前 px4ctrl 代码固定使用：

- CH1～CH4：横滚、俯仰、油门、偏航；
- CH5：Hover 模式开关；
- CH6：Command 模式开关；
- CH8：飞控重启开关，飞行中禁止误触。

自动起飞命令被接受时，若已连接遥控器，必须满足：

- CH5 大于约 `1750`；
- CH6 大于约 `1750`；
- CH1～CH4 均位于中位死区，约 `1375～1625`；
- 遥控器持续有数据，不能时有时无。

可以用下面命令只显示关键通道：

```bash
rostopic echo /mavros/rc/in/channels | grep -E '^-|channels' -A 18
```

`rc_reverse` **不会影响起飞命令是否被接受**；它只影响进入悬停后，拨动摇杆时目标位置改变的正负方向。拆桨测试时，应逐轴确认摇杆方向，再决定 `roll/pitch/yaw/throttle` 是否需要 `true`。

检查电池：

```bash
rostopic echo /mavros/battery
```

若 `percentage` 很低、为负值、跳动异常，先修正 PX4/QGC 电池参数，不要直接起飞。

### 终端 8：确认规划器尚未发送轨迹

定点自动起飞之前，不应有持续的 `/setpoints_cmd` 控制消息：

```bash
rostopic info /setpoints_cmd
rostopic hz /setpoints_cmd
```

若正在持续发布轨迹，先停止相应 Diff-Planner/命令节点。px4ctrl 的自动起飞代码会拒绝“起飞前已经收到控制命令”的情况。

### 终端 9：启动 PX4Ctrl

```bash
source ~/gcs_ws/devel/setup.bash
roslaunch px4ctrl run_ctrl_lio.launch
```

启动后检查：

```bash
rostopic echo /px4ctrl/state
rostopic hz /mavros/setpoint_raw/attitude
rostopic echo -n 1 /mavros/state
```

px4ctrl 状态值：

| 数值 | 状态 |
|---:|---|
| 1 | `MANUAL_CTRL` |
| 2 | `AUTO_HOVER` |
| 3 | `CMD_CTRL` |
| 4 | `AUTO_TAKEOFF` |
| 5 | `AUTO_LAND` |

发送起飞命令前应看到：

- `/px4ctrl/state` 为 `1`；
- `/mavros/state` 为 `connected: True`、`armed: False`；
- `/mavros/setpoint_raw/attitude` 稳定发布；
- px4ctrl 不再持续报告 `IMU frequency seems lower than 100Hz`；
- px4ctrl 不再持续报告 `ODOM frequency seems lower than 100Hz`；
- 电池状态正常；
- 无人机静止、水平放置，周围人员撤离。

如果仍有频率警告，依次执行：

```bash
rostopic hz -w 1000 /mavros/imu/data
rostopic hz -w 1000 /mavros/imu/data_raw
rostopic hz -w 1000 /Odometry
rostopic hz -w 1000 /ekf/ekf_odom
```

px4ctrl 实际检查的是：

```text
/mavros/imu/data
/ekf/ekf_odom
```

不要因为 `/Odometry` 只有约 10 Hz 就误判；真正必须超过 100 Hz 的是 EKF 输出 `/ekf/ekf_odom`。

---

## 4. 执行定点自动起飞

### 4.1 起飞前最后口令

逐项确认：

- [ ] 桨叶安装方向正确、螺母锁紧；
- [ ] 电池电量充足且固定牢靠；
- [ ] MAVROS 已连接；
- [ ] IMU `/mavros/imu/data` 稳定高于 100 Hz；
- [ ] EKF `/ekf/ekf_odom` 稳定高于 100 Hz；
- [ ] 里程计静止速度小于 0.1 m/s；
- [ ] 点云与坐标轴方向正确；
- [ ] CH5、CH6 均高位，四个摇杆居中；
- [ ] `/setpoints_cmd` 没有在起飞前持续发送；
- [ ] px4ctrl 没有频率、里程计、电池相关警告；
- [ ] 遥控器可随时切回手动/执行紧急处置；
- [ ] 人员已远离旋翼范围。

### 4.2 不要先手动解锁

当前参数：

```yaml
enable_auto_arm: true
```

因此正确顺序是：

1. 保持飞控未解锁；
2. 启动并检查全部节点；
3. 遥控器 CH5、CH6 高位，摇杆居中；
4. 发送一次起飞命令；
5. px4ctrl 自动切换 OFFBOARD，然后自动解锁并起飞。

### 4.3 发送起飞命令

```bash
rostopic pub -1 /px4ctrl/takeoff_land \
  quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1"
```

预期现象：

1. px4ctrl 从状态 `1` 进入 `4`；
2. `/mavros/state` 变为 `mode: "OFFBOARD"`；
3. `/mavros/state` 变为 `armed: True`；
4. 电机先进入准备阶段；
5. 飞机保持起飞瞬间的 x、y 和 yaw，并按约 `0.15 m/s` 上升；
6. 高度达到起点上方约 `0.5 m` 后，状态从 `4` 进入 `2`，保持定点悬停。

监视命令：

```bash
rostopic echo /px4ctrl/state
rostopic echo /mavros/state
rostopic echo /ekf/ekf_odom/pose/pose/position
```

---

## 5. 降落

在 `AUTO_HOVER` 状态下发送：

```bash
rostopic pub -1 /px4ctrl/takeoff_land \
  quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 2"
```

预期状态：

```text
AUTO_HOVER(2) → AUTO_LAND(5) → MANUAL_CTRL(1)
```

着陆检测完成后，px4ctrl 会尝试自动上锁并退出 OFFBOARD。必须观察：

```bash
rostopic echo /mavros/state
rostopic echo /mavros/extended_state
```

确认：

```text
armed: False
LANDED_STATE_ON_GROUND
```

然后才能接近无人机、停止节点或拔电池。

---

## 6. 为什么频率会再次掉回 50 Hz

最常见原因按优先级排列：

1. **PX4 或 MAVROS 重启/重连。** 一次性的 `mavcmd long 511 ...` 请求不等于写入永久参数。
2. **QGC 或其他 MAVLink 客户端重新设置了消息流。** 后发出的请求可能覆盖先前间隔。
3. **PX4/MAVLink 链路带宽不足。** 请求 500 Hz 不代表链路能够稳定送出 500 Hz。
4. **Jetson 负载过高。** RViz、点云显示、录包、FAST-LIO2、EKF 和规划器同时运行会造成回调延迟。
5. **串口/USB 连接或设备重枚举。** MAVROS 重连后需要重新下发消息间隔。

本仓库新增的自动保持节点解决第 1、2 类问题：

```text
scripts/set_px4_mavlink_imu_rate.py
launch_ROS1/set_px4_imu_rate.launch
```

它不能突破硬件和链路上限。若请求 500 Hz 后仍只有约 170 Hz，但稳定、无丢包、两条控制输入均高于 100 Hz，应以稳定性为先。

---

## 7. 建议的最小化启动清单

以下命令分别放在不同终端：

```bash
# T1 MAVROS
source ~/gcs_ws/devel/setup.bash
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600
```

```bash
# T2 IMU 频率自动保持
source ~/gcs_ws/devel/setup.bash
roslaunch livox_ros_driver2 set_px4_imu_rate.launch rate_hz:=200
```

```bash
# T3 PTP（没有 systemd 服务时）
sudo ptp4l -i eth0 -S -m -f \
  ~/gcs_ws/src/livox_ros_driver2/config/time_sync/ptp4l-mid360s.conf
```

```bash
# T4 Livox
source ~/gcs_ws/devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360s.launch
```

```bash
# T5 FAST-LIO2
source ~/gcs_ws/devel/setup.bash
roslaunch fast_lio mapping_mid360_px4.launch rviz:=false
```

```bash
# T6 EKF
source ~/gcs_ws/devel/setup.bash
roslaunch ekf ekf_fastlio2.launch
```

```bash
# T7 关键频率检查
rostopic hz -w 1000 /mavros/imu/data
rostopic hz -w 1000 /ekf/ekf_odom
```

```bash
# T8 PX4Ctrl
source ~/gcs_ws/devel/setup.bash
roslaunch px4ctrl run_ctrl_lio.launch
```

```bash
# T9 起飞（所有检查通过后，仅发送一次）
rostopic pub -1 /px4ctrl/takeoff_land \
  quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1"
```

```bash
# T9 降落
rostopic pub -1 /px4ctrl/takeoff_land \
  quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 2"
```

---

## 8. 任何一项出现时都禁止起飞

- `/mavros/imu/data` 低于 100 Hz；
- `/ekf/ekf_odom` 低于 100 Hz；
- px4ctrl 持续发出 IMU/ODOM 频率警告；
- EKF 位置跳变、速度大于 0.1 m/s 或 z 轴方向错误；
- FAST-LIO 点云明显撕裂、重影、漂移；
- `/mavros/state` 未连接；
- 遥控器通道映射不清楚或无法可靠切回手动；
- 电池剩余量低、数据异常或电压参数未校准；
- 起飞前 `/setpoints_cmd` 已持续发布；
- px4ctrl 报 `Reject AUTO_TAKEOFF`；
- 现场没有安全员或没有足够空旷区域。
