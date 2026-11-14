# faster-lio 测试文档（台架 / 手持，不飞行）

> 适用版本：`~/fasterlio_ws`（faster-lio + livox_ros_driver2 + ekf_pose）
> 配置文件：`faster-lio/config/mid360_px4.yaml`（2026-11-13 标定值）
> 前置文档：`docs/FASTER_LIO_TIME_OFFSET.md`、`docs/LIDAR_IMU_EXTRINSIC_CALIBRATION.md`

## 1. 目的与范围

**目标**：在不飞行的前提下，验证 faster-lio 在 MID-360 + Pixhawk IMU 上能否正常初始化、
输出稳定的 10 Hz 里程计，并与现有 FAST-LIO 的表现做对比。

**范围**：静止测试、手持六自由度运动、同一段数据与 FAST-LIO 对比。

**明确不做**：不解锁、不启动电机、不启动 px4ctrl、不发任何控制指令。
本测试通过之前，faster-lio 不能用于飞行。

## 2. 前提条件

### 2.1 硬件

- 整机（或至少 LiDAR + 飞控）刚性固定在同一支架上；
- **拆桨**；
- MID-360 网络正常（本机默认 `192.168.1.5` 主机 / `192.168.1.124` 雷达）；
- 飞控通过 USB 或串口连接，MAVROS 能连上。

### 2.2 软件

```bash
# 编译空间已存在且编译过
ls ~/fasterlio_ws/devel/lib/faster_lio/run_mapping_online

# 配置文件存在
ls ~/fasterlio_ws/src/faster-lio/config/mid360_px4.yaml
```

确认配置里是 2026-11-13 的标定值：

```yaml
common:
    imu_topic: "/mavros/imu/data_raw"
    time_sync_en: false
    time_offset_lidar_to_imu: -0.07625571
mapping:
    extrinsic_est_en: false
    extrinsic_T: [0.032600, 0.027755, 0.086446]
    extrinsic_R: [ 0.856373, -0.003504,  0.516346,
                   0.007575,  0.999955, -0.005777,
                  -0.516302,  0.008859,  0.856361]
```

如果重新标定过，先同步 `~/fasterlio_ws/src/faster-lio/config/` 再测。

### 2.3 节点冲突检查

开始前确认没有正在运行的飞行相关节点：

```bash
source ~/gcs_ws/devel/setup.bash
rosnode list | grep -E 'laserMapping|px4ctrl|ekf|flight_path'
```

faster-lio 的节点默认也叫 `laserMapping`，与 FAST-LIO 相同。要么先停掉 FAST-LIO，
要么按第 3 节用 `mapping_node_name:=laserMapping_faster` 改名。

## 3. 启动顺序

### 终端 1：MAVROS

```bash
source /opt/ros/noetic/setup.bash
source ~/gcs_ws/devel/setup.bash
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600
```

检查：

```bash
rostopic echo -n 1 /mavros/state        # connected: True, armed: False
```

### 终端 2：提高 PX4 IMU 频率（每次连接飞控做一次）

```bash
source ~/gcs_ws/devel/setup.bash
roslaunch livox_ros_driver2 set_px4_imu_rate.launch rate_hz:=200
```

检查（应高于 100 Hz，本机约 160–200 Hz）：

```bash
rostopic hz -w 1000 /mavros/imu/data_raw
```

### 终端 3：Livox 驱动

如果驱动已经在跑（例如用 `start_fastlio2_ekf.sh` 启动过，或单独启动过），**跳过本步**，
并在第 4 步使用 `start_lidar:=false`。

```bash
source ~/gcs_ws/devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360s.launch
```

检查（约 10 Hz）：

```bash
rostopic hz /livox/lidar
rostopic info /livox/lidar              # 发布者类型 livox_ros_driver2/CustomMsg
```

> 驱动发布的是 `livox_ros_driver2/CustomMsg`，faster-lio 订阅的是
> `livox_ros_driver/CustomMsg`。两者字段和 md5 相同，已验证可以互通，不需要转换。

### 终端 4：faster-lio

```bash
source /opt/ros/noetic/setup.bash
source ~/fasterlio_ws/devel/setup.bash
roslaunch faster_lio mapping_mid360.launch \
  config_file:=$(rospack find faster_lio)/config/mid360_px4.yaml \
  start_lidar:=false \
  mapping_node_name:=laserMapping_faster
```

参数说明：

| 参数 | 值 | 原因 |
| --- | --- | --- |
| `config_file` | `config/mid360_px4.yaml` | launch 默认是 bench 用的 `mid360.yaml`，必须显式指定 |
| `start_lidar` | `false` | 驱动已在终端 3 启动，避免同名节点冲突 |
| `mapping_node_name` | `laserMapping_faster` | 避免与 FAST-LIO 的 `/laserMapping` 冲突 |

启动日志里必须能看到这一行，且数值与配置一致：

```text
time_offset_lidar_to_imu: -0.0762557 s
imu_body_T: 0 0 0
Using AVIA Lidar
```

如果出现 `bad conversion` 或 `Invalid imu_body_T or imu_body_R size`，说明 YAML 有问题，
不要继续。

### 终端 5（可选）：RViz

launch 里的 `rviz` 参数默认是 `false`，不传 `rviz:=true` 不会有任何窗口。两种方式任选：

```bash
# 方式 A：启动 faster-lio 时带上（第 3 节终端 4 的命令末尾加 rviz:=true）
roslaunch faster_lio mapping_mid360.launch \
  config_file:=$(rospack find faster_lio)/config/mid360_px4.yaml \
  start_lidar:=false mapping_node_name:=laserMapping_faster rviz:=true

# 方式 B：单独启动
source ~/fasterlio_ws/devel/setup.bash
rosrun rviz rviz -d ~/fasterlio_ws/src/faster-lio/config/rviz/faster_lio.rviz
```

手动添加：

| 显示项 | 话题 | 说明 |
| --- | --- | --- |
| PointCloud2 | `/laserMapping/cloud_registered` | 世界系点云 |
| Odometry | `/laserMapping/odometry` | IMU 系里程计 |
| Odometry | `/laserMapping/odometry_body` | body 系里程计 |
| Path | `/laserMapping/path` | 轨迹 |

Fixed Frame 设为 `camera_init`（配置里的 `tf_world_frame`）。

## 4. 启动后必须核对的项

```bash
rosnode list | grep -E 'laserMapping_faster|livox'
rostopic hz /laserMapping/odometry
rostopic echo -n 1 /laserMapping/odometry/header
rostopic echo -n 1 /laserMapping/odometry/pose/pose/position
```

| 检查项 | 期望 |
| --- | --- |
| `/laserMapping/odometry` 频率 | ≈ 10 Hz（与点云同频） |
| 帧名 | `camera_init`（world）/ `body` |
| 静止位置 | 不随时间单调漂移 |
| 日志 | 无 `imu loop back`、无 `bad conversion` |
| CPU | 单帧耗时 < 100 ms（`top` 看 laserMapping 进程） |

如果里程计频率明显低于 10 Hz，先加大 `ivox_grid_resolution`（例如 0.5 → 0.7）
或 `point_filter_num`（3 → 4），再测。

## 5. 静止测试（5–10 分钟）

1. 上电后整机**完全静止**，先等 10 秒让 faster-lio 完成重力对齐和初始化；
2. 保持静止，录 5 分钟数据：

```bash
rosbag record -O ~/fasterlio_static_$(date +%Y%m%d_%H%M%S).bag \
  /livox/lidar /mavros/imu/data_raw /laserMapping/odometry
```

3. 观察 RViz：点云不应分层、撕裂，不应缓慢旋转。

静态漂移统计（不需要录包，直接采样 5 分钟）：

```bash
# 采样 5 分钟，Ctrl-C 结束；tail 去掉 CSV 表头
rostopic echo -p /laserMapping/odometry/pose/pose/position | tail -n +2 > /tmp/static_pos.csv

python3 - <<'EOF'
import numpy as np
d = np.loadtxt('/tmp/static_pos.csv', delimiter=',')
xyz = d[:, 0:3]
print("samples:", len(xyz))
print("max |dpos| (m):", np.max(np.linalg.norm(xyz - xyz[0], axis=1)))
print("xyz std (m):", xyz.std(axis=0))
EOF
```

## 6. 手持运动测试

整机拿起，按下面顺序做，每个动作 20–30 秒：

1. **原地旋转**：绕竖直轴左右各转 180–360°；
2. **前后平移**：平滑前进 1–2 m 再退回；
3. **左右平移**：同上；
4. **上下抬升**：0.5 m 左右往复；
5. **回到出发点**停住。

每个动作都要核对：

| 观察项 | 正常表现 | 异常表现 |
| --- | --- | --- |
| 点云地图 | 墙面/地面结构清晰，无明显重影 | 重影、分层、地图"糊" |
| 里程计方向 | 与实际运动方向一致 | 方向反、轴互换 |
| 原地旋转 | 位置基本不变，yaw 同步变化 | 旋转时位置大幅漂移 |
| 回到起点 | 位置误差小 | 单向漂移越来越大 |

如果方向反了或轴互换，先检查外参旋转矩阵（不要靠改代码"凑"）。

## 7. 与 FAST-LIO 对比（推荐）

### 7.1 录一段共同数据

用 FAST-LIO 的正常启动方式（`start_fastlio2_ekf.sh` 或单独 launch），录一段包含
LiDAR + IMU + FAST-LIO 里程计的包：

```bash
rosbag record -O ~/compare_ref.bag /livox/lidar /mavros/imu/data_raw /Odometry
# 手持做第 6 节的五个动作，约 2–3 分钟，Ctrl-C
```

### 7.2 用 faster-lio 离线回放同一段数据

faster-lio 自带离线运行器，不走 ROS，速度更快：

```bash
source ~/fasterlio_ws/devel/setup.bash
cd ~/fasterlio_ws
rosrun faster_lio run_mapping_offline \
  --config_file=src/faster-lio/config/mid360_px4.yaml \
  --bag_file=$HOME/compare_ref.bag \
  --traj_log_file=/tmp/fasterlio_traj.txt
```

注意：

- 离线模式读的也是 `mid360_px4.yaml` 里的 `time_offset_lidar_to_imu`（YAML 路径已支持）；
- 包里如果只有 `/Odometry` 而没有 LiDAR/IMU，离线运行器没数据可用；
- 轨迹保存到 `--traj_log_file`，表头为 `#timestamp x y z q_x q_y q_z q_w`。

### 7.3 在线对比（更直观）

也可以让 faster-lio 跑在线，同时 `rosbag play` 回放。**注意：回放前先停掉 Livox 驱动**，
否则真实点云和回放点云会同时进同一个话题：

```bash
# 终端 A：faster-lio（第 3 节终端 4 的命令）
roslaunch faster_lio mapping_mid360.launch \
  config_file:=$(rospack find faster_lio)/config/mid360_px4.yaml \
  start_lidar:=false mapping_node_name:=laserMapping_faster

# 终端 B：回放（--clock 可选，faster-lio 用消息自带时间戳）
rosbag play ~/compare_ref.bag
```

然后比较 `/Odometry`（FAST-LIO）与 `/laserMapping/odometry`（faster-lio）：
两者都是 IMU 系位姿，可以做位置差统计。

**注意**：faster-lio 的世界系是它自己的初始化时刻，FAST-LIO 也是。
两次运行的起点和朝向不一定相同，直接相减前要先对齐第一帧（平移 + yaw）。

## 8. 建议的通过标准

以下阈值是**初始建议值**，先用 FAST-LIO 的实测数据当基线，再按需要调整：

| 项目 | 建议标准 |
| --- | --- |
| 静止 5 min 最大位置漂移 | < 0.10 m |
| 静止 xyz 标准差 | < 0.02 m |
| 原地旋转 360° 后 yaw 回差 | < 2° |
| 手持回到起点位置误差 | < 0.10 m |
| 与 FAST-LIO 同段数据位置差 | 均值 < 0.05 m，最大 < 0.15 m |
| 里程计频率 | ≈ 10 Hz，单帧耗时 < 100 ms |
| 点云地图 | 无明显重影/分层 |

达不到时**不要靠放宽阈值解决**，先查时间同步、外参和 IMU 数据质量。

## 9. 常见问题

| 现象 | 可能原因 | 处理 |
| --- | --- | --- |
| `RLException: environment variable 'BD_LIST' is not set` | 上游 launch 用 `$(env BD_LIST)`，即使 `start_lidar:=false` 也会求值 | 已改为 `$(optenv BD_LIST 100000000000000)`，不再要求环境变量；旧版本可临时 `export BD_LIST=100000000000000` |
| 节点启动即退出 / `bad conversion` | YAML 键缺失或顺序问题 | 对比 `config/mid360.yaml` 检查 |
| 没有 `/laserMapping/odometry` | 没收到点云或 IMU | `rostopic hz /livox/lidar`、`/mavros/imu/data_raw` |
| 日志反复 `imu loop back, clear buffer` | IMU 时间戳回退，常因偏移方向错或时钟跳变 | 核对 `time_offset_lidar_to_imu` 符号 |
| 里程计静止也在漂 | 外参错误、IMU 零偏大、时间不同步 | 重新标定 / 先做静态 IMU 检查 |
| 点云重影、地图撕裂 | 时间偏移不准 | 复核 LI-Init 的时间偏移，必要时重标 |
| 频率掉到 5 Hz 以下 | Jetson CPU 负载 / ivox 分辨率过细 | 加大 `ivox_grid_resolution`、`point_filter_num` |
| launch 报节点重名 | FAST-LIO 还在跑 | 停掉 FAST-LIO，或用 `mapping_node_name` |
| 与驱动节点冲突 | launch 又启动了一个 `livox_lidar_publisher2` | 用 `start_lidar:=false` |
| 订阅不到点云 | 驱动发布的是 `livox_ros_driver2/CustomMsg` | 已验证兼容；若仍失败，用 `rostopic info` 核对 |

## 10. 测完之后

- 记录结果：日期、配置值、静态漂移、手持表现、与 FAST-LIO 的差异；
- 结果满足第 8 节后才考虑接入 EKF 和任务栈；
- 接入时注意话题差异：faster-lio 是 `/laserMapping/odometry`、
  `/laserMapping/cloud_registered`，而 `hybrid_mission.yaml` 用的是 `/Odometry`、
  `/cloud_registered`，需要重映射或改参数；
- 该配置**尚未飞行验证**，飞行前仍需完整的地面检查。

## 11. 回放与清理

```bash
# 退出所有节点：各自终端 Ctrl-C
# 检查残留
rosnode list | grep -E 'laserMapping|livox'
# 清理本次测试的包（确认不需要后再删）
ls -lh ~/fasterlio_static_*.bag ~/compare_ref.bag
```
