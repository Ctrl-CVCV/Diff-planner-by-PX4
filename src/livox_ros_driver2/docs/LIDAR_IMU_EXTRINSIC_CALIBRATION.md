# MID-360 + Pixhawk IMU 外参与时间偏移标定笔记

> 最后整理：2026-10-17  
> 工作空间：`~/gcs_ws`  
> 标定工具：`LiDAR_IMU_Init`  
> LiDAR：Livox MID-360  
> IMU：Pixhawk，经 MAVROS 发布 `/mavros/imu/data_raw`

## 1. 标定结果用在哪里

本套系统的数据链路是：

```text
/livox/lidar + /mavros/imu/data_raw
        ↓
FAST-LIO2（应用 LiDAR→IMU 外参和时间偏移）
        ↓
/Odometry（已经表示 IMU/body 位姿）
        ↓
EKF
        ↓
/ekf/ekf_odom
```

LiDAR→IMU 标定结果只写入：

```text
~/gcs_ws/src/FAST_LIO/config/mid360_px4.yaml
```

当前 EKF 输入的 `/Odometry` 已经由 FAST-LIO2 转换到 IMU/body frame，因此 EKF 中：

```yaml
Rr_i: [1, 0, 0, 0]
tr_i: [0.0, 0.0, 0.0]
```

继续保持单位旋转和零平移。不要把 LiDAR→IMU 外参在 EKF 中重复应用一次。

---

## 2. 安全要求

1. **拆下全部桨叶。** 标定全程不需要解锁，更不能起飞。
2. 关闭 `px4ctrl`、Diff-Planner、EKF 和正常 FAST-LIO2。
3. 标定程序与 FAST-LIO2 的 ROS 节点都叫 `/laserMapping`，两者不能同时运行。
4. LiDAR、飞控和机架必须刚性固定；标定过程中不能让 LiDAR 相对飞控发生移动。
5. 搬动的是整架无人机/整套传感器，不是单独晃动雷达。
6. 选择点、线、平面结构丰富的环境，避免空旷、玻璃过多或大量动态人员的场地。

检查是否还有冲突节点：

```bash
source ~/gcs_ws/devel/setup.bash
rosnode list | grep -E 'laserMapping|px4ctrl|ekf|multipoint'
```

开始标定前不应存在 `/laserMapping`、`/px4ctrl`、`/ekf` 等飞行相关节点。

---

## 3. 标定前配置

LiDAR_IMU_Init 配置文件：

```text
~/gcs_ws/src/LiDAR_IMU_Init/config/mid360s_px4.yaml
```

当前关键配置为：

```yaml
common:
    lid_topic:  "/livox/lidar"
    imu_topic:  "/mavros/imu/data_raw"

initialization:
    cut_frame_num: 5
    orig_odom_freq: 10
    mean_acc_norm: 9.805
    online_refine_time: 20
    data_accum_length: 500
```

含义：

- `lid_topic`：MID-360 点云；
- `imu_topic`：飞控原始 IMU；
- `mean_acc_norm`：加速度单位为 `m/s²`，静止模长约为 `9.8`；
- `data_accum_length`：激励数据积累要求，越大通常需要越充分的三轴运动；
- `online_refine_time`：初始化完成后继续在线优化的时间，本机为 20 秒。

先检查静止 IMU 模长和频率：

```bash
rostopic hz -w 1000 /mavros/imu/data_raw
rostopic echo -n 1 /mavros/imu/data_raw
```

角速度单位必须是 `rad/s`，线加速度单位必须是 `m/s²`。

---

## 4. 启动顺序（本机跳过 PTP）

标定时不要运行一键 FAST-LIO2+EKF 脚本，因为该脚本会启动正常 FAST-LIO2，与 LiDAR_IMU_Init 的 `/laserMapping` 冲突。

### 终端 1：MAVROS

```bash
source /opt/ros/noetic/setup.bash
source ~/gcs_ws/devel/setup.bash
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600
```

检查：

```bash
rostopic echo -n 1 /mavros/state
```

应看到：

```text
connected: True
armed: False
```

### 终端 2：保持 PX4 IMU 消息频率

```bash
source ~/gcs_ws/devel/setup.bash
roslaunch livox_ros_driver2 set_px4_imu_rate.launch rate_hz:=200
```

检查：

```bash
rostopic hz -w 1000 /mavros/imu/data_raw
```

建议稳定高于 100 Hz；本机约 160～200 Hz 可以接受。

### 终端 3：Livox MID-360

```bash
source ~/gcs_ws/devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360s.launch
```

检查：

```bash
rostopic hz /livox/lidar
rostopic echo -n 1 /livox/lidar/header
```

MID-360 点云通常约为 10 Hz。

### 终端 4：启动 LiDAR_IMU_Init 和 RViz

```bash
source ~/gcs_ws/devel/setup.bash
roslaunch lidar_imu_init livox_mid360s_px4.launch rviz:=true
```

---

## 5. 看到初始化界面后怎么操作

### 阶段 A：先保持静止

启动标定程序后，把整架无人机稳定放在地面或牢固支架上，**至少静止 5～10 秒**：

- 不要碰机架；
- 不要晃动桌面；
- 不要启动电机；
- 等待 IMU 初始化和初始点云地图积累。

如果看到类似：

```text
IMU Initialization Done
```

说明静止初始化已经完成，可以准备进行运动激励。

### 阶段 B：开始三轴旋转和移动

拿稳整架无人机，进行平滑、幅度充分的六自由度运动。

推荐顺序：

1. **Roll 激励**：机体左右倾斜，约 `±30°～60°`，反复 5～10 次；
2. **Pitch 激励**：机头上仰、下俯，约 `±30°～60°`，反复 5～10 次；
3. **Yaw 激励**：水平左右转动，约 `±45°～90°`，反复 5～10 次；
4. 前后平移并平滑反向；
5. 左右平移并平滑反向；
6. 上下抬升并平滑反向；
7. 最后进行三轴组合运动。

动作要求：

- 平滑但不要过慢，要产生可观测的角速度和线加速度；
- 不要猛烈撞击、急停或敲打机架；
- 不要只绕单一轴旋转；
- 不要只做匀速平移，因为匀速段线加速度信息不足；
- 始终保持 LiDAR 周围有足够静态几何结构。

终端会显示类似：

```text
[Initialization] Movement detected, data accumulation starts.
[Initialization] Rotation around Lidar X Axis: ...
[Initialization] Rotation around Lidar Y Axis: ...
[Initialization] Rotation around Lidar Z Axis: ...
```

哪一轴进度低，就重点补充绕该轴的往复旋转。不要把坐标轴名称理解成“只朝某个方向平移”；这里主要要求对应轴的旋转激励。

### 阶段 C：等待初始化和 refinement 完成

当三个旋转方向的进度都达到要求后，会出现：

```text
[Initialization] Data accumulation finished, Lidar IMU initialization begins.
```

此后不要立刻停止。配置中还有约 20 秒在线 refinement：

- 继续做平滑的三轴旋转和少量往复平移；
- 不要提前按 `Ctrl+C`；
- 等待终端明确显示最终结果已写入文件。

完成标志：

```text
Initialization and refinement result is written to
/home/nvidia/gcs_ws/src/LiDAR_IMU_Init/result/Initialization_result.txt
```

看到这句之后再停止标定程序。

---

## 6. 保存本次结果

结果文件每次标定可能被覆盖，因此立即备份：

```bash
cd ~/gcs_ws/src/LiDAR_IMU_Init
cp result/Initialization_result.txt \
   "result/Initialization_result_$(date +%Y%m%d_%H%M%S).txt"
```

查看：

```bash
cat ~/gcs_ws/src/LiDAR_IMU_Init/result/Initialization_result.txt
```

结果中同时有：

- `Initialization result`：批量初始化结果；
- `Refinement result`：在线优化后的结果。

**写入 FAST-LIO2 时优先采用 `Refinement result`。**

---

## 7. 如何把结果写入 FAST-LIO2

结果定义为：

```text
p_I = R_LI × p_L + T_LI
```

即“LiDAR 坐标中的点转换到 IMU 坐标”。FAST-LIO2 当前配置使用的也是同一方向，所以：

- 不要对旋转矩阵转置；
- 不要求逆；
- 不要把平移取负；
- 旋转矩阵按照结果文件的三行顺序展开为 9 个数。

先备份 FAST-LIO2 配置：

```bash
cp ~/gcs_ws/src/FAST_LIO/config/mid360_px4.yaml \
   ~/gcs_ws/src/FAST_LIO/config/mid360_px4.yaml.bak_$(date +%Y%m%d_%H%M%S)
```

编辑：

```bash
nano ~/gcs_ws/src/FAST_LIO/config/mid360_px4.yaml
```

填写模板：

```yaml
common:
    lid_topic:  "/livox/lidar"
    imu_topic:  "/mavros/imu/data_raw"
    time_sync_en: false
    time_offset_lidar_to_imu: <Time Lag IMU to LiDAR>

mapping:
    extrinsic_est_en: false
    extrinsic_T: [Tx, Ty, Tz]
    extrinsic_R: [R11, R12, R13,
                  R21, R22, R23,
                  R31, R32, R33]
```

因为使用的是已标定外参，所以：

```yaml
extrinsic_est_en: false
```

时间偏移也必须根据当前 FAST-LIO2 配置的定义填写。此工作空间现有配置已经按 LI-Init 的 `Time Lag IMU to LiDAR` 数值直接写入 `time_offset_lidar_to_imu`。

本机当前没有由启动脚本运行 PTP，因此如果 LiDAR 或飞控重启后时间偏移不稳定，应重新标定或至少重新验证时间偏移，不能只依赖旧值。

---

## 8. 本机当前标定结果

当前结果文件：

```text
~/gcs_ws/src/LiDAR_IMU_Init/result/Initialization_result.txt
```

当前采用的是 `Refinement result`：

```text
Rotation LiDAR to IMU (degree)   = 1.531353 30.833219 1.762915
Translation LiDAR to IMU (meter) = 0.006849 -0.034990 0.069355
Time Lag IMU to LiDAR (second)   = -0.068406
```

齐次变换矩阵：

```text
 0.858277 -0.017062  0.512903  0.006849
 0.026414  0.999591 -0.010949 -0.034990
-0.512507  0.022946  0.858377  0.069355
 0.000000  0.000000  0.000000  1.000000
```

对应 FAST-LIO2 配置：

```yaml
common:
    time_sync_en: false
    time_offset_lidar_to_imu: -0.068406

mapping:
    extrinsic_est_en: false
    extrinsic_T: [0.006849, -0.034990, 0.069355]
    extrinsic_R: [ 0.858277, -0.017062,  0.512903,
                   0.026414,  0.999591, -0.010949,
                  -0.512507,  0.022946,  0.858377]
```

这些数值已经写入：

```text
~/gcs_ws/src/FAST_LIO/config/mid360_px4.yaml
```

对当前结果的数学检查：

```text
det(R) ≈ 1.000000
||RᵀR - I|| ≈ 1.74e-6
||gravity|| ≈ 9.8073 m/s²
```

矩阵正交性和重力模长正常。不过平移和约 30.8° 的安装角仍必须与实际机械安装方向、尺量距离相符。

---

## 9. 标定结果验收

建议完整标定至少做 2～3 次，每次单独备份结果，然后比较：

- 三次旋转结果是否接近；
- 平移结果是否与尺量安装距离同量级；
- 时间偏移是否稳定；
- 重力模长是否接近 `9.81 m/s²`；
- 是否出现 `NaN`、极大平移或明显不合理角度。

实用参考而非硬性标准：

- 多次旋转差异最好小于约 `1°～2°`；
- 多次平移差异最好小于约 `2～3 cm`；
- 时间偏移最好只在数毫秒量级内变化。

如果重复结果差别很大，优先排查：

1. 三轴激励不足；
2. LiDAR 和飞控固定不牢；
3. IMU 频率不稳定；
4. 时间戳来源或同步方式变化；
5. 环境几何退化或存在大量动态物体；
6. 标定时动作过猛，机架产生振动或形变。

---

## 10. 写入后验证 FAST-LIO2

停止 LiDAR_IMU_Init，确保 `/laserMapping` 已退出，然后启动正常系统：

```bash
cd ~/gcs_ws/src/livox_ros_driver2
./scripts/start_fastlio2_ekf.sh
```

检查加载值：

```bash
rosparam get /mapping/extrinsic_est_en
rosparam get /mapping/extrinsic_T
rosparam get /mapping/extrinsic_R
rosparam get /common/time_offset_lidar_to_imu
```

检查输出：

```bash
rostopic hz /Odometry
rostopic hz /ekf/ekf_odom
rostopic echo -n 1 /Odometry
rostopic echo -n 1 /ekf/ekf_odom
```

静止和手持运动验收：

1. 静止时点云地图不应持续分层、撕裂或旋转；
2. 静止时速度应接近 0，位置不能快速漂移；
3. 原地旋转时墙面不应出现明显重影；
4. 前后、左右、上下移动时里程计方向必须正确；
5. FAST-LIO2 正常后再启动 EKF；
6. EKF 正常后仍不要立即起飞，先完成无桨状态机和控制输出检查。

---

## 11. 最容易犯的错误

- 标定时 FAST-LIO2 还在运行，两个 `/laserMapping` 冲突；
- 启动后立刻晃动，没有先静止 5～10 秒；
- 只做水平 yaw，X/Y 旋转激励不足；
- 只做匀速平移，没有线加速度激励；
- 看到 Initialization result 就停止，没有等待 Refinement result；
- 把外参矩阵转置或求逆后再写入；
- 同一套 LiDAR→IMU 外参同时写进 FAST-LIO2 和 EKF，导致重复转换；
- LiDAR/飞控重新安装后仍使用旧外参；
- 不使用 PTP，却没有验证每次上电后的时间偏移是否稳定。

---

## 12. 2026-10-17 重新标定记录（未写入飞行配置）

结果文件更新时间：2026-10-17 17:32:51（本机时间）。

本次 `Initialization result`：

```text
Rotation LiDAR to IMU (degree)     =  0.866703 29.707753  1.286948
Translation LiDAR to IMU (meter)   =  0.005665 -0.059560  0.133243
Time Lag IMU to LiDAR (second)     = -0.113680
Bias of Gyroscope (rad/s)          = -0.009214 -0.021156 -0.002075
Bias of Accelerometer (m/s^2)      = -0.003577 -0.010233  0.013509
```

本次 `Refinement result`：

```text
Rotation LiDAR to IMU (degree)     = -9.673407 68.088889 -8.467038
Translation LiDAR to IMU (meter)   =  0.041339 -0.044877  0.262567
Time Lag IMU to LiDAR (second)     = -0.113680
Bias of Gyroscope (rad/s)          = -0.014740 -0.209030 0.041726
Bias of Accelerometer (m/s^2)      =  0.504778 0.040739 0.062357
```

`Refinement result` 的 LiDAR→IMU 齐次变换为：

```text
 0.369181 -0.009042  0.929313  0.041339
-0.054953  0.997991  0.031541 -0.044877
-0.927731 -0.062713  0.367943  0.262567
 0.000000  0.000000  0.000000  1.000000
```

验收结论：**本次 refinement 未通过合理性检查，因此暂未覆盖
`FAST_LIO/config/mid360_px4.yaml`。** 主要依据：

- refinement 相对本次 initialization 的旋转差约 `38.90°`；
- refinement 相对当前在用结果的旋转差约 `37.67°`；
- 本次 initialization 相对当前在用结果仅约 `1.68°`；
- refinement 的陀螺仪 Y 轴零偏为 `-0.209030 rad/s`，明显可疑；
- refinement 的平移模长约 `0.270 m`，必须先与实际尺量安装距离核对；
- 新旧时间偏移相差约 `38.864 ms`，也需要重新验证时间同步是否稳定。

注意：旋转矩阵本身正交、行列式接近 1，只能说明矩阵格式正确，不能证明标定物理结果正确。重新标定并通过重复性、机械尺寸和静态点云检查后，再把最终 refinement 写入飞行配置。

---

## 13. 2026-10-17 第二次重新标定记录（未写入飞行配置）

结果文件更新时间：2026-10-17 17:48:58（本机时间）。

本次 `Initialization result`：

```text
Rotation LiDAR to IMU (degree)     =  1.620168 32.352554  1.752315
Translation LiDAR to IMU (meter)   =  0.035006 -0.138012  0.092795
Time Lag IMU to LiDAR (second)     = -0.104359
Bias of Gyroscope (rad/s)          = -0.001696 -0.021816 -0.002458
Bias of Accelerometer (m/s^2)      = -0.003243  0.009619  0.014034
```

本次 `Refinement result`：

```text
Rotation LiDAR to IMU (degree)     = -28.911499 61.164626 -17.294866
Translation LiDAR to IMU (meter)   =  0.030166 -0.099486  0.219038
Time Lag IMU to LiDAR (second)     = -0.104359
Bias of Gyroscope (rad/s)          =  0.089341 -0.185847 0.020136
Bias of Accelerometer (m/s^2)      =  0.458246 0.230870 0.144746
```

`Refinement result` 的 LiDAR→IMU 齐次变换：

```text
 0.460558 -0.144099  0.875855  0.030166
-0.143391  0.961696  0.233623 -0.099486
-0.875971 -0.233187  0.422254  0.219038
 0.000000  0.000000  0.000000  1.000000
```

验收结论：**本次 refinement 仍然未通过，未覆盖 FAST-LIO2 当前配置。**

- initialization 与当前在用旋转仅相差约 `1.82°`；
- refinement 与 initialization 相差约 `35.18°`；
- refinement 与当前在用旋转相差约 `36.41°`；
- refinement 陀螺仪零偏达到 `[0.089341, -0.185847, 0.020136] rad/s`，明显异常；
- refinement 平移模长约 `0.242 m`；
- 连续两次都是 initialization 接近约 30°～32°，进入 refinement 后跳到约 61°～68°，说明问题集中发生在在线 refinement 阶段，而不是 YAML 矩阵格式。

当前继续保留上一次已验证并正在使用的 FAST-LIO2 参数，禁止直接使用本次 refinement 进行起飞测试。


---

## 14. 2026-10-17 第三次重新标定记录（已采用）

结果文件更新时间：2026-10-17 18:14:23（本机时间）。本次 initialization 与 refinement 收敛一致，已写入 FAST-LIO2。

```text
Initialization rotation =  1.039987 30.483751 1.495813 deg
Refinement rotation     =  1.531353 30.833219 1.762915 deg
Initialization T        =  0.002297 -0.038677 0.066932 m
Refinement T            =  0.006849 -0.034990 0.069355 m
Time lag                = -0.068406 s
```

验收数据：

- initialization 与 refinement 旋转差约 `0.55°`；
- 二者平移差约 `6.34 mm`；
- refinement 与原在用旋转差约 `1.23°`；
- refinement 旋转矩阵 `det(R) ≈ 1.000000`；
- 正交误差约 `1.74e-6`；
- refinement 重力模长约 `9.8073 m/s²`；
- gyro bias 和 accelerometer bias 均处于合理量级。

写入文件：

```text
/home/nvidia/gcs_ws/src/FAST_LIO/config/mid360_px4.yaml
```

写入前备份：

```text
/home/nvidia/gcs_ws/src/FAST_LIO/config/mid360_px4.yaml.bak_20261017_181423
```
