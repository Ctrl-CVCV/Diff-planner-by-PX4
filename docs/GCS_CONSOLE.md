# GCS 地面站控制台（运行 / 调试）

> 更新：2026-11-20
> 程序：`diff_planner/plan_manage/scripts/gcs_console.py`
> 依赖：PyQt5（已装）、ROS Noetic、`~/gcs_ws` 已编译
> **状态：界面与进程管理只做过离屏构造/编译验证；打点与命令拼装有单元测试；未在真实飞行流程中整体验证。**

## 1. 启动

在带桌面的终端：

```bash
source ~/gcs_ws/devel/setup.bash
python3 ~/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/scripts/gcs_console.py
```

界面分两个页签：**运行功能**、**调试功能**；底部是共享运行日志，状态栏显示 ROS 连接状态和最新 EKF 位姿。

程序只负责启动/停止子进程、读写任务配置，**不会自行解锁或发控制指令**；任务脚本仍走它自己的确认流程。

## 1.1 实时监视面板（运行页顶部）

用来在报错或里程计异常时提前发现：

- **EKF 位姿**：x / y / z / yaw / 时间戳延迟；延迟 <300 ms 绿色，<1 s 黄色，超过或无数据红色
  （EKF 节点没起时显示"未运行"，不算异常）；
- **速度 / EKF-LIO 偏差**：速度来自 EKF 的 twist（>0.65 m/s 黄色、>1.0 m/s 红色）；
  偏差是 `/Odometry` 与 `/ekf/ekf_odom` 的位置差（>0.20 m 黄色、>0.35 m 红色，与任务
  `safety/max_ekf_lio_divergence` 一致）；
- **话题健康表**：`/Odometry`（期望 10 Hz）、`/ekf/ekf_odom`、`/mavros/imu/data`、
  `/mavros/state`、`/px4ctrl/state`、`/drone_0_mission/state` 的频率与延迟；
  "延迟"是**消息时间戳与当前 ROS 时间的差**（不是接收年龄），能直接看出 faster-lio 是否积压；
  任务脚本预检的硬门禁阈值是 `/Odometry` 延迟 ≤0.30 s、频率 ≥8 Hz、最近 30 s 无 EKF 超时告警；
- **节点灯**：mavros / livox / lio / ekf / vision_bridge / px4ctrl / planner / traj_server /
  yolo / servo 是否在 ROS 图里（绿=在，灰=未运行），每 2 s 刷新；
- **PX4 状态**：connected / armed / mode；
- **PX4Ctrl 状态**：MANUAL_CTRL / AUTO_HOVER / CMD_CTRL / AUTO_TAKEOFF / AUTO_LAND；
- **任务阶段**：来自 `/drone_0_mission/state`；
- **告警框**：最近 12 条，来源有两类：
  1. `/rosout` 里 WARN / ERROR 级别的日志（含各节点报错，如 Planner 拒轨迹、EKF 断言）；
  2. 内置异常检测：EKF 位姿 NaN、位置跳变（>1.5 m/s 黄色、>3 m/s 红色）、
     速度超限、EKF-LIO 偏差过大。同类告警 3 秒内只记一次，避免刷屏。

时间戳、频率和偏差都是**只读监测**，不改变任何飞行行为。

## 2. 运行功能

### 2.1 定位链

按钮「启动 faster-lio + EKF」执行：

```bash
bash ~/gcs_ws/src/livox_ros_driver2/scripts/start_fastlio2_ekf.sh
```

它会启动 roscore（按需）、MAVROS、IMU 频率保持、Livox 驱动、faster-lio、EKF 和视觉位姿桥，
不启动 PX4Ctrl、不解锁。日志同时输出到界面和 `~/gcs_ws/log/fastlio2_ekf_<时间>/`。
「停止」发送 SIGINT，脚本会按顺序清理它启动的进程。

### 2.2 控制 / 规划 / 视觉 / 舵机

| 按钮 | 命令 |
| --- | --- |
| PX4Ctrl | `roslaunch px4ctrl run_ctrl_lio.launch` |
| Diff-Planner | `roslaunch diff_planner run_exp_hybrid_mission.launch` |
| 视觉识别 | `roslaunch yolo_trt_ros yolo_trt.launch`；方案由右侧下拉框选择：**VLM + OpenCV**（默认，`use_vlm:=true`）或 **TensorRT YOLO**（`use_vlm:=false`）；可取消勾选显示窗口 |
| 启动 / 停止 VLM 服务 | `systemctl --user start/stop qwen2.5-vl.service`（llama-server，`127.0.0.1:8080`）；启动后自动等到 `/health` 就绪（约 10–30 s），状态每 3 s 检测一次 |
| 舵机控制 | `roslaunch px4_servo payload_servos.launch`（PWM_MAIN5..8，四路物资舵机，hold_s=2 s 自动回位） |

「一键启动全部」按 PX4Ctrl → Planner → 视觉识别 → 舵机 顺序间隔启动；「一键停止全部」逆序停止。

**识别方案与 VLM 触发（2026-11-25）**

- 下拉框「识别方案」在点「视觉识别」启动时生效：`VLM + OpenCV` 或 `TensorRT YOLO`。
  两种方案的节点名（`/yolo_trt_detector`）、话题和检测 JSON 都相同，**切换前先停掉当前方案**，
  不要同时启动。
- **VLM 方案需要两个东西**：`ROS 识别节点`（「视觉识别」按钮，含相机）和 `推理服务`
  （「启动 VLM 服务」按钮，llama-server）。只启动前者时节点能起来，但每次识别都会报
  `VLM request failed: Connection refused`，预览里也不会有结果。界面会检测服务状态并在启动
  视觉节点时提示。
- VLM 模式下，下方「VLM 单次识别（/yolo/enable）」组提供两个按钮：
  - **危险标志（C/D 点）→ 0**：发 `/yolo/enable = false`，识别一次危险标志；
  - **人像（F/G/H 点）→ 1**：发 `/yolo/enable = true`，识别一次人像。
  每点一次只识别一次。送给模型的是外层矩形边框，确认后按原格式发布约 4 s。
  **跑 `--execute` 任务时不需要点**：任务脚本到点会自己发这一条；按钮用于地面/悬停测试。
- TensorRT YOLO 模式下 `/yolo/enable` 是**电平开关**（`false` 停推理并发布空检测，`true`
  恢复），两个触发按钮会自动禁用，避免把单次触发当成开关误用。
- 触发按钮通过界面自带的 ROS 线程直接发布 `/yolo/enable`（`std_msgs/Bool`）；ROS 未连接时
  回退为 `rostopic pub -1`。识别节点没在运行时，界面日志会提示"消息没有接收者"。

### 2.3 任务脚本（独立入口）

- 模式：**预检（--check）** 或 **执行（--execute）**；
- 执行模式必须勾选「地图路径已确认」「直控路径已确认」（对应 `--map-verified` `--direct-path-verified`）；
- 可选 `--no-hazard-drop` / `--no-rescue-drop`；
- 执行时界面先弹确认框，确认后启动脚本，并在 1.5 s 后自动向脚本 stdin 写入 `MISSION`
  （脚本预检通过后会读取它，继续执行）；
- 「停止任务脚本」发送 SIGINT：脚本按自身逻辑中断并执行保护降落（若已起飞）。

### 2.4 航点打点

面向"飞到目标点上方悬停 → 一键记录坐标"：

- 选择航点（A～J 中配置存在者）；
- 读取当前 `/ekf/ekf_odom` 位姿（超过 1 s 未更新会拒绝打点）；
- **x、y** = 当前位姿相对任务原点的水平坐标；
- **z_offset** 取决于测量方式：
  - **地面放置（默认，推荐）**：把飞机放在该点地面上读 EKF，
    `z_offset = 实测 z − 原点 z`，即该点地面相对起飞点的高度差（例如 C 点实测 0.033）；
  - **空中悬停**：`z_offset = 实测 z − 原点 z − 该点配置 z`；
- 原点来源：优先用 `/drone_0_mission/state` 里的 `origin_world`（任务脚本预检后发布）；
  拿不到时按 `(0,0,0)` 计算并在预览里标注；
- 点击「打点并写入配置」后弹确认框；确认后：
  1. 自动备份 `hybrid_mission.yaml` 为 `.bak_<时间戳>`；
  2. 只改该航点那一行（保留文件注释和其它行）；`xyz` 的 z 保持不变，只替换 x/y，
     并写入 `z_offset`；
  3. 可勾选同时写 `measured: true`；
  4. 写完后重新解析校验，通过才落盘；
  5. **联动航点**：配置里某个点写了 `mirror: X`，写其中任意一个时，联动点的
     x / y / z_offset 一起更新，yaw 保持原值。当前有：
     - **C ↔ E**：E 带 `mirror_offset: [0.60, 0.0]`，始终保持在 C 的 +x 0.6 m；
       打 C 会同步 E，打 E 会反推 C；
     - **F ↔ I**：I 带 `mirror_offset: [-0.35, 0.0]`，始终保持在 F 的 −x 0.35 m；
       打 F 会同步 I，打 I 会反推 F（F 90°/I 180°）。

> 说明：z_offset 的语义是"该点地面/实测高度相对配置高度的偏差"，正数表示已记录的
> 实测高度比配置高。打点时请让飞机稳定悬停在该点上方再点，否则记录的会是过渡位置。

> 说明：z_offset 的语义是"该点地面/实测高度相对配置高度的偏差"，正数表示已记录的
> 实测高度比配置高。打点时请让飞机稳定悬停在该点上方再点，否则记录的会是过渡位置。

打点区域还提供：

- **实时 EKF 读数**：当前 x / y / z、时间戳延迟、所用任务原点及来源；
- **航点配置表**：A～J 的配置 x / y / z、z_offset、measured 一屏显示，点选某行会同步切换打点目标航点，写入配置后表格自动刷新。

### 2.5 显示（可选）

| 按钮 | 内容 |
| --- | --- |
| 障碍地图 RViz | `roslaunch diff_planner view_obstacle_map.launch`（实时障碍地图 + 实测/指令轨迹线） |
| 里程计 / 点云 RViz | `rviz -d .../launch/include/lio_cloud_odom.rviz`（`/cloud_registered` + `/Odometry` + `/path`） |

### 2.6 录包（可选）

「任务录包」调用 `record_diff_planner_debug.py`，输出到 `~/flight_logs/console_<时间>/`，
包含飞行相关话题并生成快照。停止时发送 SIGINT，脚本正常收尾。

## 3. 调试功能

### 3.1 USB 摄像头录像（640×480）

- 设备默认 `/dev/video0`，分辨率固定 640×480（`usb_camera_record.py` 里写死，界面只暴露 FPS 和输出）；
- 输出默认 `~/Videos/usb_640x480_<时间>.mp4`（文件已存在会自动加时间戳）；
- 开始/停止通过子进程 + SIGINT，停止后打印帧数和平均 FPS；
- **YOLO 正在使用摄像头时先停 YOLO**，否则摄像头打不开（界面日志会给出提示）。

### 3.2 调试录包（不飞行）

记录障碍地图、里程计、点云等：

```text
/Odometry  /cloud_registered  /cloud_registered_body  /ekf/ekf_odom
/ekf/flight_path  /path  /tf  /tf_static
/mavros/state  /mavros/extended_state  /mavros/imu/data  /mavros/imu/data_raw
/drone_0_diff_planner_node/grid_map/.*
```

可选包含 `/livox/lidar`（原始点云，体积大）。输出默认
`~/flight_logs/debug_<时间>.bag`，已存在会自动加时间戳，不会覆盖。

## 4. 安全与使用顺序

1. **不要用界面跳过检查**：执行模式的两个 verified 勾选是人的确认，不是自动验障；
2. 执行前按快速启动流程确认定位、Planner、PX4Ctrl、RC、降落区；
3. 打点写的是任务配置，正在执行的任务不会热加载；需要下一次 `--execute` 才生效；
4. 舱内/手持测试时用调试页的录包和摄像头功能，不要启动 PX4Ctrl；
5. 关闭窗口时若有子进程在跑，会提示并统一发送 SIGINT；roscore/MAVROS 若由脚本启动会一并退出；
6. 摄像头录像时注意磁盘空间；调试 bag 加原始点云时增长很快。

## 5. 常见问题

| 现象 | 处理 |
| --- | --- |
| 状态栏一直「等待 ROS master」 | 先启动定位链/roscore；界面会自动重试连接 |
| EKF 位姿不更新 / 拒绝打点 | `rostopic delay /ekf/ekf_odom` 检查是否实时（LIO 积压时会过期） |
| 打点时预览显示原点 `(0,0,0)` | 任务状态还没发布：先跑任务预检，或确认当前就在起飞点附近 |
| 子进程启动后立即结束 | 看界面日志；通常是包没编译、配置路径错、或设备被占用 |
| 摄像头打不开 | 停 YOLO 节点；`fuser /dev/video0` 查占用进程 |
| 录包路径已存在 | 界面会自动加时间戳；也可手动改输出路径 |
| 界面字体/缩放异常 | Qt 高分屏问题，可设 `QT_AUTO_SCREEN_SCALE_FACTOR=0` 再启动 |

## 6. 开发与测试

- 纯逻辑在 `gcs_console_core.py`（YAML 打点、命令拼接、备份），不依赖 PyQt/ROS；
- 单元测试：`scripts/tests/test_gcs_console_core.py`；
- GUI 本身只做了 `py_compile` 和离屏（`QT_QPA_PLATFORM=offscreen`）构造验证，
  没有自动化的按钮点击测试。
