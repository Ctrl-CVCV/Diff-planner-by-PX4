# EKF → MAVROS → PX4 视觉位姿桥

> 2026-11-17 补充：当前 Faster-LIO 源码和一键启动入口已统一到 `gcs_ws`，地面编译与独立 EKF/视觉桥使用方式见 [Faster-LIO 当前指南](FASTER_LIO_GUIDE.md)。本页涉及的 PX4 参数和历史故障结论不代表当前运行状态；观察 ROS 定位数据无需先修改飞控参数。发布视觉位姿可能影响飞控估计，不应视为完全只读操作。

> 更新：2026-11-14
> 相关：`ekf_vision_bridge.py`、`start_fastlio2_ekf.sh` 第 7 步
> **状态：代码已通过单元测试和实时数据冒烟测试；PX4 参数尚未修改，未做飞行验证。**

## 1. 为什么需要这个桥

整条定位链路：

```text
Livox MID-360 ─┐
               ├─> faster-lio (/Odometry) ─> EKF (/ekf/ekf_odom) ─> PX4Ctrl
Pixhawk IMU ───┘                              │
                                              └─> 视觉位姿桥 ─> /mavros/vision_pose/pose
                                                                     │
                                                     MAVROS（ENU→NED）
                                                                     ▼
                                                    PX4 EKF2（VISION_POSITION_ESTIMATE）
```

PX4Ctrl 直接消费 `/ekf/ekf_odom` 飞位置环；但 **PX4 自己的估计器**没有外部定位输入，
`estimator_status` 里水平位置/速度全部无效（`const_pos_mode=True`），`EKF2_EV_CTRL=0`，
所以 PX4 在 OFFBOARD 下拒绝解锁（`MAV_RESULT=1`）。这个桥的作用是把 EKF 的位姿送进 PX4，
补上最后一段。

> 调查结论：本机仓库、launch、脚本、git 历史、ROS master 日志（2026-10-28 起）里
> **没有任何节点发布过** `/mavros/vision_pose/pose`、`/mavros/mocap/pose`、`/mavros/odometry/out`。
> 所以这个桥是按 MAVROS/PX4 标准接口新写的，不是恢复旧代码。

## 2. 桥接行为

`diff_planner/plan_manage/scripts/ekf_vision_bridge.py`：

| 项目 | 值 |
| --- | --- |
| 输入 | `/ekf/ekf_odom`（`nav_msgs/Odometry`，frame `world`） |
| 输出 | `/mavros/vision_pose/pose`（`geometry_msgs/PoseStamped`） |
| 频率 | 默认最多 50 Hz（PX4 建议 30–50） |
| 坐标系 | 原样转发 ENU/FLU；MAVROS vision_pose 插件负责 ENU→NED、FLU→FRD |
| 安全过滤 | 时间戳超 `--max-age`（默认 0.5 s）丢弃；NaN/零四元数丢弃 |
| 副作用 | 只读转发，不发控制指令，不依赖 PX4 参数开关 |

参数：

```bash
python3 .../scripts/ekf_vision_bridge.py \
  --odom /ekf/ekf_odom --topic /mavros/vision_pose/pose \
  --rate 50 --max-age 0.5 --frame ''
```

## 3. 启动方式

### 3.1 随快速启动脚本（默认开启）

`start_fastlio2_ekf.sh` 第 7 步会在 EKF 就绪后自动启动桥：

```bash
bash ~/gcs_ws/src/livox_ros_driver2/scripts/start_fastlio2_ekf.sh
```

关闭：

```bash
bash ~/gcs_ws/src/livox_ros_driver2/scripts/start_fastlio2_ekf.sh --no-vision-bridge
```

### 3.2 单独启动

```bash
source ~/gcs_ws/devel/setup.bash
python3 ~/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/scripts/ekf_vision_bridge.py
```

## 4. PX4 参数（需要人工确认后设置）

这些是**飞控参数修改**，不属于工作空间代码；在 QGC 参数页搜索名称设置（或 `rosrun mavros
mavparam set <名称> <值>`），改完建议重启飞控再飞。

| 参数 | 建议值 | 含义 |
| --- | --- | --- |
| `EKF2_EV_CTRL` | **11** | 位掩码：1=水平位置，2=垂直位置，8=偏航；11 = 水平+垂直+偏航。只想先要水平位置可先用 **9**（1+8），高度继续用气压计 |
| `EKF2_HGT_REF` | **3**（Vision） | 高度参考源。枚举：0=Baro，1=GNSS，2=Range，3=Vision（当前 FCU 是 2=Range，但机上没有测距仪） |
| `EKF2_EV_DELAY` | 0（ms） | 视觉相对 IMU 的延迟；本链路时间戳经 MAVROS/PX4 timesync 校正，先保持 0，按飞行日志再调。**该参数修改后需要重启飞控** |
| `EKF2_EV_POS_X/Y/Z` | 0 | 视觉测量点相对机体中心的偏移；EKF 输出就是机体/IMU 位姿，填 0 |
| `COM_ARM_WO_GPS` | 1（已设置） | 无 GPS 允许解锁 |

说明：

- `EKF2_EV_CTRL` 的 bit2 是 3D 速度。`/mavros/vision_pose/pose` 只带位姿、不带速度，
  **不要**把这个位置 1（否则 PX4 会把 0 速度当测量融合）。
- 如果只设 `EKF2_EV_CTRL=9`、`EKF2_HGT_REF=0`，PX4 的高度仍来自气压计，与 EKF 的 z
  可能有缓慢漂移；此时 PX4 的位置估计只用于水平控制/解锁条件。
- 设置前先确认桥在稳定输出（见第 5 节）。

## 5. 验证顺序（全部在地面、拆桨）

1. **LIO 必须实时**（否则送进去的是旧位姿，比没有更危险）：

   ```bash
   rostopic hz /Odometry            # 应约 10 Hz
   rostopic delay /Odometry         # 应稳定在 0.1–0.3 s，不随时间增长
   ```

2. **桥在输出**：

   ```bash
   rostopic hz /mavros/vision_pose/pose     # 应约 50 Hz
   rostopic echo -n 1 /mavros/vision_pose/pose
   ```

3. **PX4 收到并融合**（设置 EV_CTRL 并重启飞控后）：

   ```bash
   rostopic echo -n 1 /mavros/estimator_status
   # 期望：pos_horiz_rel_status_flag / pos_horiz_abs_status_flag / velocity_horiz_status_flag 变为 True
   rostopic echo /mavros/local_position/pose     # 手抬飞机时位置应跟随
   ```

4. **对比一致性**：手抬飞机移动，比较 `/ekf/ekf_odom` 与 `/mavros/local_position/pose`
   的位置和航向，方向和量级应一致（允许小偏差）。

5. **解锁检查**：上述都正常后，用 PX4Ctrl/任务脚本尝试 arm；日志不应再出现
   `ARM rejected by PX4`。成功解锁后**不要**直接起飞，先做第 4 步的跟随检查。

## 6. 常见问题

| 现象 | 可能原因 | 处理 |
| --- | --- | --- |
| `/mavros/vision_pose/pose` 没有数据 | 桥没启动 / EKF 没有输出 / 桥把过期数据全丢了 | `rosnode list` 看 `ekf_vision_bridge`；`rostopic delay /ekf/ekf_odom` |
| 桥日志报"里程计过期" | LIO 积压（`/Odometry` 时间戳落后几百秒） | 先修 LIO 频率（调大 `filter_size_surf`、停 YOLO/VS Code），重启节点 |
| `estimator_status` 水平位置仍为 False | `EKF2_EV_CTRL` 仍为 0 / 没有重启飞控 / 桥数据未到 | 核对参数、重启 FCU、确认桥在输出 |
| PX4 高度乱跳 | `EKF2_HGT_REF=3` 但视觉高度不稳 | 先改回 `EKF2_HGT_REF=0`、`EKF2_EV_CTRL=9`（只用视觉水平） |
| 位置方向反了/轴不对 | 桥被改过 frame 或做了转换 | 桥必须原样转发 ENU；转换由 MAVROS 完成 |
| EV 数据被拒（PX4 日志/statustext） | 时间戳偏差大 | 检查 MAVROS timesync，`rostopic echo /mavros/timesync_status`；再调 `EKF2_EV_DELAY` |
| 解锁仍失败 | 还有其它 preflight 项 | 看 `/mavros/statustext/recv` 的 `Preflight Fail: ...` |

## 7. 回退

1. 启动加 `--no-vision-bridge`，或单独启动时 Ctrl-C；
2. PX4 参数改回：`EKF2_EV_CTRL=0`、`EKF2_HGT_REF=2`（改动前记录原值），重启飞控；
3. 桥是独立节点，不影响 EKF/PX4Ctrl/任务脚本，停掉即恢复原链路（PX4 无外部定位的状态）。

## 8. 限制

- 桥只经过单元测试（`tests/test_ekf_vision_bridge.py`）和实时数据冒烟（50 Hz 输出、
  无过期丢弃），**没有做过带 PX4 融合的飞行或手持验证**。
- PX4 参数一旦启用外部视觉，PX4 的位置/高度估计就依赖 EKF 输出；EKF 失效时 PX4 的行为
  需要按 PX4 failsafe 逻辑重新评估，不要假设和以前一样。
- EKF 的 world 原点是起飞时刻的原点；PX4 的 local position 会以该原点为参考，上电时
  EKF 若已经移动过，PX4 的 local z 可能不为 0，属正常。
