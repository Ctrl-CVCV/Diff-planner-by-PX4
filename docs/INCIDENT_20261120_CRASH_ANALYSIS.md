# 2026-11-20 冲顶/坠落事故分析

> 分析时间：2026-11-20 21:30
> 数据：恢复后的机载 bag（仅地面段）+ 飞控 ULog（事故航段）+ ROS 日志
> 证据文件已保全在 `~/flight_logs/incident_20261120/`

## 1. 数据来源

| 数据 | 路径 | 内容 |
| --- | --- | --- |
| 原始 bag（断电未关闭） | `~/flight_logs/console_20261120_205702/flight_0.bag.active` | 527 MB |
| 修复后的 bag | `~/flight_logs/incident_20261120/ground_segment_reindexed.bag` | 20:57:36–20:58:56，128,517 条消息 |
| 飞控 ULog（事故航段） | `~/flight_logs/incident_20261120/crash_ulog_id53.ulg` | 武装后 9.1 秒，至断电 |
| ROS 日志 | `~/.ros/log/3867a3a0-.../rosout.log` | 到 20:58:51 被断电截断 |

说明：

- bag 里**没有飞行段**：`rosbag record -b 256`（256 MB 内存缓冲），断电时最后几十秒未落盘；任务指令话题（`/setpoints_cmd`、`/goal`、任务状态）在录制开始时尚无发布者，rosbag 没有订阅，因此缺失。
- ULog 通过 MAVROS 从飞控拉取（MAVLink FTP/日志传输，id=53），内容完整到断电瞬间。

## 2. 结论摘要

**事故是两条独立的严重问题叠加的结果，不是单一操作失误：**

1. **定位/估计器失效**：faster-lio 输出滞后 10–15 秒，EKF 持续报
   `odom time out of buffer ... 13.6 s, ekf may be wrong`。EKF 输出的高度在爬升过程中甚至变成
   **负值**（-0.59 m），而气压计显示实际高度约 **2.9 m**。
2. **PX4 被喂了错误的外部视觉**：`EKF2_EV_CTRL=11`、`EKF2_HGT_REF=3`（视觉高度）已启用，
   ULog 中 `estimator_aid_src_ev_pos/hgt/yaw` 全部 `fused=1`，PX4 把这份错误的 EKF 位姿当作
   真值融合；PX4Ctrl 也用同一份 EKF 做控制。结果：**飞机实际爬升到约 2.9 m（顶到天花板），
   而飞控和控制器都以为还在 0.7 m 以下，持续给爬升推力。**
3. **电池紧急触发飞控自毁（flight termination）**：电池在 25–38 A 负载下电压快速跌落，
   SOC 在约 5 秒内从 43% 掉到 0（`BAT1_CAPACITY=-1`，容量未配置，SOC 是电压估算）；
   3.5 秒触发低电告警，4.9 秒进入紧急；8.26 秒 PX4 发出 `DO_FLIGHTTERMINATION`（cmd 208），
   **在约 2.9 m 高度直接停桨**，飞机自由落体坠地，撞击时断电，日志终止。

**直接导致坠地的是第 3 条（空中停桨）；让它爬到 2.9 m 的是第 1、2 条。**

## 3. 时间线（ULog 时间，t=0 为解锁/起飞）

| t (s) | 事件 |
| --- | --- |
| 0.00 | 已解锁，`nav_state=OFFBOARD`，`Takeoff detected`；地面 |
| 2.76 | `vehicle_land_detected.ground_contact` 消失，离地 |
| ~2.8 | PX4Ctrl 推力设定从 0.13 升到 0.50，电流 25–38 A |
| 3.54 | `Low battery`，failsafe 激活 |
| 4.25 | 电池 critical（SOC≈0.11） |
| 4.85 | 电池 emergency（SOC≈0.01） |
| 8.0 | 气压计高度约 **2.88 m**；PX4 融合高度仅 **0.72 m** |
| 8.2–8.26 | 垂直速度由 +0.40 突变为 -0.20 m/s（疑似顶到天花板） |
| 8.26 | PX4 发出 cmd 208 `DO_FLIGHTTERMINATION`，`Kill engaged` |
| 8.68 | `freefall=1` |
| 9.1 | ULog 结束（撞击断电） |

## 4. 关键证据

### 4.1 LIO 里程计滞后（恢复 bag）

接收时间 − 消息时间戳：

```text
包内 0–40 s : 8–10 s
45 s        : 7.2 s
55–80 s     : 10.6 → 15.3 s
```

同时段 EKF 日志：

```text
delaytime 14158.09, buf_size 100
odom time out of buffer time range: 13651.08ms, ekf may be wrong !!!!
```

### 4.2 PX4 融合高度 vs 气压计实际高度（ULog）

| t (s) | PX4 融合高度 | 气压计高度（相对起始） | EV 高度观测 |
| --- | --- | --- | --- |
| 0.0 | 0.01 | 0.00 | -0.01 |
| 3.5 | 0.05 | 0.74 | -0.08 |
| 5.0 | 0.27 | 1.03 | -0.22 |
| 6.5 | 0.51 | 2.12 | -0.42 |
| 8.0 | 0.72 | 2.88 | -0.59 |

EV（外部视觉）高度来自 EKF 经视觉位姿桥送入 PX4；爬升过程中它一直报告**地面附近甚至负高度**，
与气压计差约 2 m。

### 4.3 电池（ULog）

| t (s) | 电压 V | 电流 A | SOC |
| --- | --- | --- | --- |
| 0.05 | 19.6–23.3 | 10–21 | 0.43 |
| 3.05 | 21.5 | 30.9 | 0.23 |
| 4.05 | 17.7 | 24.3 | 0.11 |
| 4.85 | 22.0 | 34.9 | 0.01 |
| 8.25 | 21.2 | 30.2 | 0.00 |

`BAT1_CAPACITY=-1`（未配置容量），`BAT1_N_CELLS=6`，`V_EMPTY=3.7`、`V_CHARGED=4.2`；
初始 23.9 V ≈ 3.98 V/cell，约 50–60% 电量，重载下压到 3.0 V/cell 以下，触发低电/紧急。

### 4.4 其它参数（ULog 初始参数）

```text
EKF2_EV_CTRL     11      # 水平+垂直+偏航视觉融合
EKF2_HGT_REF     3       # 高度参考 = 视觉
EKF2_EV_DELAY    0
COM_FLTT_LOW_ACT 3       # 剩余飞行时间不足时的动作：Return
COM_FAIL_ACT_T   5.0 s   # failsafe 触发到动作的延迟
COM_LOW_BAT_ACT  0       # 普通低电只告警
BAT_LOW_THR      0.15
BAT_CRIT_THR     0.07
BAT_EMERGEN_THR  0.05
SDLOG_MODE       0       # 解锁后记录
COM_ARM_WO_GPS   1
```

姿态稳定：全程 roll/pitch 约 0.01 rad，无翻转迹象。

## 5. 与 `filter_size_surf 0.15 / filter_size_map 0.30` 的关系

- 20:56:29 时 LIO 还能跟上：`/Odometry` 9.27 Hz，时间戳延迟 0.05–0.20 s；
- 20:52:47 启动 YOLO 后，EKF 差时告警在 20:52:49 首次出现；20:57:05 开始录包后进一步恶化到 10–15 s；
- 0.15/0.30 的配置下 LIO 平均处理速率约 9.3 Hz，**对 10 Hz 输入几乎没有余量**。这个参数本身不是错误，
  但在本机同时跑 YOLO 50 FPS、录包（lz4 6.6 MB/s）、sunshine、VS Code 时容不下任何抖动，
  一旦处理低于 10 Hz，队列按 0.06–1 s/s 累积，最终 10–15 s。
- 2026-11-21 已把默认值回退为 `filter_size_surf 0.20 / filter_size_map 0.40`；
- 结论：**参数偏激进、缺乏 CPU 余量**，但真正致命的是"估计器已经明显失效后仍起飞"，
  以及 PX4 正在融合这份失效的 EKF。

## 6. 必须修改的项（按优先级）

1. **起飞前硬门禁**（2026-11-21 已在任务脚本实现，见 `check_lio_realtime`）：
   - `/Odometry` 时间戳延迟 > 0.3 s 持续 1 s → 拒绝 `--execute`；
   - 最近 30 秒出现 `odom time out of buffer` → 拒绝；
   - EKF 高度与气压计/其他独立源偏差超过 0.5 m → 告警并拒绝。
2. **PX4 外部视觉**：
   - 在门禁完备前，不要同时启用 `EKF2_HGT_REF=3`；先保持 `EKF2_EV_CTRL=9`（仅水平+偏航）、
     `EKF2_HGT_REF=0`（气压高度），或干脆关闭外部视觉；
   - 视觉位姿桥增加"输入新鲜度"检查：EKF 的 `/Odometry` 输入过期/差时告警时停止发布，
     而不是继续把 EKF 输出当作实时视觉。
3. **电池**：
   - 设置 `BAT1_CAPACITY` 为实际容量，做一次满电负载测试（记录 30 A 下电压、SOC）；
   - 重新评估低电动作：当前 `COM_FLTT_LOW_ACT=3` + 紧急阈值会在低空直接触发停桨，
     对多旋翼是高风险动作；
   - 电量不足/电池老化时不要起飞。
4. **负载与 LIO 参数**：
   - 飞行时不要同时跑 YOLO/sunshine/VS Code；
   - 可把 `filter_size_surf` 调到 0.20–0.25、`filter_size_map` 0.40，或 `point_filter_num=4`；
   - EKF `seqsize`（100 ≈0.5 s）可提高到 300 作为抖动缓冲，但不能替代实时性。
5. **录包**：GUI 的 `-b 256` 改为 16–32，断电最多丢不到一秒；并让任务脚本在启动时就发布
   `/drone_0_mission/state` 等话题，或在任务开始后再启动录包，避免指令话题缺失。

## 7. 未决问题

- 是否真的顶到天花板：气压计在 8.0–8.5 s 停在 2.88–2.90 m，垂直速度在 8.2 s 由 +0.4 变 -0.2，
  但撞击瞬间的加速度样本缺失（日志只剩 1 个样本），只能判为"高度/速度特征符合顶到天花板"。
- 电池是真实容量不足还是电流计比例偏大：需要一次地面负载测试核对。
- LIO 10 s 级滞后是纯 CPU 抢占还是 LiDAR 时间基准变化：当前数据更支持 CPU 抢占（滞后随负载变化、
  有回落），但建议下次启动时同时记录 `/livox/lidar` 原始时间戳确认。
- 飞控 SD 卡里还有更多历史 ULog，事故这条 id=53 已拉取；不要删除 SD 卡日志。
