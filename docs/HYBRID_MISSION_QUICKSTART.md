# 混合任务最简启动 / 看 RViz / 录 bag

更新：2026-10-31（危险品投放默认开启，新增投放舵机节点 `px4_servo`；第 7 步 YOLO 视觉识别）。摘录自 [混合任务全流程手册](HYBRID_MISSION_FLIGHT_AND_BAG_RUNBOOK.md)（下称"手册"），只保留**启动、看图、录包**三步链，删掉全部排查与回放内容。

**这是步骤摘录，不代表已经验证过实机飞行。** 每一条的实际含义、边界和异常处理以手册为准；起飞前必须完成手册第 8 节的 `--check` 与第 13 节核对，异常时飞手优先接管。

```text
FAST-LIO2 → /Odometry → EKF → /ekf/ekf_odom
                            ↓
Diff-Planner → traj_server → /drone_0_mission/planner_cmd
                                      ↓
                         hybrid_mission.py 选择/转发
                                      ↓
                           /setpoints_cmd → PX4Ctrl

YOLO → /yolo/detections → hybrid_mission.py（判定危险品）→ /servo/push → px4_servo → AUX6 舵机
```

## 终端一览

| 终端 | 内容 | 何时启动 | 何时停 |
|---|---|---|---|
| 0 | 建本轮目录、任务参数副本 | 最先 | 准备完即可关 |
| 1 | MAVROS + 雷达 + FAST-LIO2 + EKF | 第 1 步 | 落地锁桨后 |
| 2 | PX4Ctrl | 第 2 步 | 落地锁桨后 |
| 3 | 混合任务专用 Planner | 第 3 步 | 落地锁桨后 |
| 4 | **录 bag（地图监视）** | 第 4 步，**执行任务之前** | 落地锁桨后再录 5～10 秒 |
| 5 | RViz 实时看图 | 任意时刻，Planner 起来后 | 随时 |
| 6 | `--check` 预检 → `--execute` 执行 | 第 6 步 | 到 DONE |
| 7 | YOLO 视觉识别（可选，不参与飞行；会弹 `YOLO TensorRT` 预览窗口） | 第 7 步，地面插好相机后 | 随时，Ctrl+C 即停 |
| 8 | 投放舵机 `px4_servo`（`/servo/push`，危险品投放默认开启，**必需**） | 第 6 步之前，MAVROS 起来后、地面启动 | 落地锁桨后 |

所有终端必须连同一个实机 `ROS_MASTER_URI`；新终端都要重新 `source ~/gcs_ws/devel/setup.bash`。

## 第 0 步（终端 0）：建本轮目录

```bash
source ~/gcs_ws/devel/setup.bash
export FLIGHT_DIR="$HOME/flight_logs/$(date +%Y%m%d_%H%M%S)_hybrid"
mkdir -p "$FLIGHT_DIR"
printf '%s\n' "$FLIGHT_DIR" > "$HOME/flight_logs/current_hybrid_session.txt"
cp ~/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/config/hybrid_mission.yaml "$FLIGHT_DIR/mission.yaml"
df -h "$HOME/flight_logs"
```

每轮只建一次。改了 C++ 源码（`px4ctrl_node` / `diff_planner_node` / `traj_server`）才需要在地面重编译：

```bash
cmake --build "$HOME/gcs_ws/build" --target px4ctrl_node diff_planner_node traj_server -- -j2
```

## 第 1 步（终端 1）：定位链

```bash
bash ~/gcs_ws/src/livox_ros_driver2/scripts/start_fastlio2_ekf.sh
```

它启动 roscore、MAVROS、Livox MID360s、FAST-LIO2/RViz、EKF；**不启动 PX4Ctrl、不解锁、不起飞**。落地锁桨前**不要在这个终端按 Ctrl+C**，会断掉定位与飞控通信链。

另开检查终端确认：

```bash
source ~/gcs_ws/devel/setup.bash
rostopic echo -n 1 /mavros/state               # 期望 connected: True, armed: False
rostopic echo -n 1 /mavros/extended_state      # 期望 landed_state: 1
```

逐条跑、各看数秒后 Ctrl+C：`rostopic hz /ekf/ekf_odom`、`/Odometry`、`/mavros/imu/data`、`/cloud_registered`。EKF/IMU 应稳定高于 100 Hz，地面静置不得漂移跳变。

## 第 2 步（终端 2）：PX4Ctrl

```bash

roslaunch px4ctrl run_ctrl_lio.launch
```

加载 `px4ctrl/config/ctrl_param_fpv.yaml`（含 `gain:` 与 `max_auto_vel`）。不要另起第二套 PX4Ctrl，不要盲调增益/质量/悬停油门。

```bash
rosparam get /px4ctrl/auto_takeoff_land      # enable: true, enable_auto_arm: true, no_RC: false, takeoff_height 与本轮一致
```

## 第 3 步（终端 3）：混合任务专用 Planner

```bash
roslaunch diff_planner run_exp_hybrid_mission.launch 
```

**只启动这一个 launch**，它已包含 `run_exp_single_lio.launch`；再另启后者会争抢话题。这一步不发目标、不点 RViz 发目标工具、不运行 multipoint 或旧起飞脚本。

```bash
rostopic info /drone_0_mission/planner_cmd              # 发布者应为 /drone_0_traj_server
rosparam get /drone_0_traj_server/traj_server/hold_at_end   # true
```

## 第 4 步（终端 4）：开始录 bag —— 必须在执行任务之前

```bash
python3 ~/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/scripts/record_diff_planner_debug.py
```

- `--output` 指向的目录必须是**尚不存在**的：不要事先 `mkdir`，脚本会拒绝覆盖已有目录。
- 不用 `sudo`；不加 `--execute`；默认录到 Ctrl+C，LZ4 压缩、约 1 GB 分片。
- 需要重算 LIO 的原始 Livox 数据时加 `--raw-lidar`（存储/负载更高，先地面试）。

确认**真的在录**：终端出现 rosbag 录制输出、`ls -lh "$FLIGHT_DIR/recording/"` 有增长中的文件，并且地图监视在滚动：

```text
[地图监视] RAW=... 帧/... 点，距上一帧 ...s; INFLATE=... 帧/... 点，距上一帧 ...s
```

`RAW`/`INFLATE` 都应持续更新；`尚未收到` 不是空地图，先查地图发布，不能带着这个状态起飞。只打印"待执行命令"不等于已经开始录。

## 第 5 步（终端 5）：RViz 看实时障碍地图

前提：第 1、3 步已在运行。命令不依赖 `FLIGHT_DIR`，不起飞、不发目标：

```bash
roslaunch diff_planner view_obstacle_map.launch
```

这个 launch 同时起两个东西：`flight_path.py`（只读，把 `/ekf/ekf_odom` 累积成 `/ekf/flight_path`）和 RViz。第一次用要先编一次以安装脚本：`cd ~/gcs_ws && catkin_make -j2`。只想看图、不想让 launch 起 RViz 时用 `start_rviz:=false`，自己再 `rviz -d`；但那样不会有轨迹线。

确认 `Global Options → Fixed Frame = world`，然后看这些显示：

| 显示名 | 话题 | 含义 |
|---|---|---|
| `Occupancy Raw` | `/drone_0_diff_planner_node/grid_map/occupancy` | 橙色，未膨胀障碍 |
| `Occupancy Inflated` | `/drone_0_diff_planner_node/grid_map/occupancy_inflate` | 半透明蓝，膨胀障碍 |
| `Flight Path (ekf_odom)` | `/ekf/flight_path` | 黄色连续线条，实际飞行轨迹（不再用里程计箭头） |
| `Registered Cloud` | `/cloud_registered` | 灰色点云，默认关闭，需真实 TF |

交替开关 RAW / Inflated，从俯视和侧视检查起飞竖直通道、各 Planner 段与直控段的可用性，以及 I 下方降落区；膨胀地图不应把悬停点或目标包住。地图不在视野内时调 `Views → Current → Focal Point` 到 EKF 的 world 坐标，**不要改 frame 或发假 TF** 把它拉回屏幕。

注意：地图显示正常**不证明**路径可达或可安全起飞；实时看图用 `live_obstacle_map.rviz`，历史包回放用 `flight_replay.rviz`（手册第 12 节，独立 master）。

## 第 6 步之前（终端 8）：投放舵机 px4_servo —— 不起就拒绝起飞

`hybrid_mission.yaml` 里 `hazard_drop/enabled: true`（2026-10-31 起默认开启），任务预检会等
`/servo/push` 服务；**节点没起时会直接报下面的错误并停止**：

```text
[ERROR] 任务停止：hazard_drop 已启用但投放服务 /servo/push 不可用（timeout exceeded while waiting for service /servo/push）；先启动 px4_servo
```

启动（MAVROS 已由第 1 步起来；只在地面、拆桨或载荷已固定的状态下启动）：

```bash
source ~/gcs_ws/devel/setup.bash
roslaunch px4_servo servo_aux_pwm.launch
```

启动日志里应看到自检行，并且 `ready` 为 `true` 才能动作：

```text
AUX6：输出功能 306（Offboard Actuator Set 6），由 DO_SET_ACTUATOR 的 param6 驱动；
区间 500..2500 us（中点 1500）；rest 500 us；push 1400 us（归一化 -0.100）；收起归一化 -1.000；上锁输出 500 us
```

另开检查终端核对：

```bash
source ~/gcs_ws/devel/setup.bash
rosservice list | grep /servo            # 应有 /servo/push /servo/retract /servo/reload
rostopic echo -n 1 /servo/state          # 期望 ready: true、push_ok: true
rosservice call /mavros/param/get "{param_id: 'PWM_AUX_DIS6'}"   # 必须是 500；2500 = 上锁即顶到推出位
```

- 本 launch 默认 `push_us=1400`（归一化 −0.1，2026-10-31 实测推出行程）、`hold_s=0`：
  投放后**保持推出、不自动收回**，任务脚本不会调 `/servo/retract`；只有上锁且 `PWM_AUX_DIS6=500`
  时才会回收起位。
- `ready: false` 时先看日志里的 `节点已启动但当前不可动作：<原因>`：多数是 `PWM_AUX_*` 参数不对或
  MAVROS 参数缓存没同步。改完 PX4 参数后 `rosservice call /servo/reload`，不用重启节点。
- 只有确认舵机动作正常才起飞；完整的分层测试（拆桨、示波器、带料标定）见
  [px4_servo AUX6 测试流程](PX4_SERVO_AUX6_TEST_RUNBOOK.md)。
- 本轮**不投放**时，可以不起本节点：在第 6 步的预检和执行命令后加 `--no-hazard-drop`，
  脚本就不订阅检测、不调用舵机服务。

## 第 6 步（终端 6）：预检，然后执行

先只读预检（不起飞、不注册指令发布端）：

```bash
python3 ~/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/scripts/hybrid_mission.py 
```

核对打印出的 A 原点与 B/C/D/E/F/G/H/I 的 **world** 坐标和 RViz 一致；确认没有提前手动起飞。有 `measured: false` 的占位航点时 `--execute` 会被拒绝，必须实测后改 `true`。

预检进程退出后，在同一终端执行：

```bash
python3 ~/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/scripts/hybrid_mission.py \
  --execute --map-verified --direct-path-verified
```

两个 `verified` 是**人的确认**，不是自动验障；确认场地、通道、降落区都检查过才加。输入 `MISSION` 后脚本发送一次 TAKEOFF，其余目标全部自动发送。另开终端只读观察阶段：

```bash
rostopic echo /drone_0_mission/state
```

## 第 7 步（终端 7，可选）：YOLO 视觉识别

队友新加的 `yolo_trt_ros`（USB 相机 + TensorRT 检测）。它只**订阅图像、发布检测结果**，不订阅也不发布任何飞行指令话题，开不开都不影响任务链；但它在 NX 上占 GPU 和 3 个推理线程，所以**不要在空中第一次启动**——先在地面跑起来看一眼负载。

```bash
ls -l /dev/video0                      # 相机没插时这里不存在，后面一定失败
source ~/gcs_ws/devel/setup.bash
roslaunch yolo_trt_ros yolo_trt.launch
```

这条命令**默认会弹出一个预览窗口**：标题 `YOLO TensorRT`、初始 640×480、可拖拽缩放，刷新率
`display_fps` 默认 **10 FPS**（比推理帧率低，是故意的——NX 在 15W 下同时跑 60 FPS 预览和 60 FPS
推理会掉帧）。窗口内容与 `/yolo/preview/image` 同一路，带中文类别标注。
没有桌面（SSH 或 `DISPLAY` 为空）时节点会打印 `show_window is set but DISPLAY is empty; preview
window disabled` 并自动关掉窗口，检测照常跑，不用管。

| 话题 | 类型 | 方向 |
|---|---|---|
| `/usb_cam/image_raw` | `sensor_msgs/Image` | 输入（`usb_camera` 节点采集） |
| `/yolo/preview/image` | `sensor_msgs/Image` | 输出（带中文类别标注的预览图） |
| `/yolo/detections` | `std_msgs/String` | 输出（JSON 检测结果） |

确认真的在跑：

```bash
rostopic hz /yolo/preview/image                    # 应接近实际处理帧率（预览窗口只显示 10 FPS）
rostopic echo -n 1 /yolo/detections                # count 与 detections 数组对应
rosrun rqt_image_view rqt_image_view /yolo/preview/image   # 想满帧率看图时用这个，和预览窗口不冲突
```

`/yolo/detections` 的 `data` 是单行 JSON：`stamp` / `frame_id` / `sequence` / `count` / `detections`，每个检测项含 `class_id`、`label`（`其它标识` / `危险品标识` / `救助人员`）、`confidence`、`x1 y1 x2 y2`、`cx cy`、`width height`、`norm_cx norm_cy`（0～1 归一化中心，接其它模块时用它最省事）。

常用覆盖参数：

```bash
# 无桌面/远程 SSH 时关掉 OpenCV 预览窗口（节点原本也会自动降级）
roslaunch yolo_trt_ros yolo_trt.launch show_window:=false
# 换相机设备或换引擎
roslaunch yolo_trt_ros yolo_trt.launch device:=/dev/video1 engine_path:=$HOME/yolo/best.engine
```

- 引擎固定在 `/home/nvidia/yolo/best.engine`（检测阈值 `conf_thres` 默认 0.60，类别名由 `label_names` 参数给出，顺序即 `class_id` 0/1/2）。
- 相机节点打不开设备会 `logfatal` 退出，检测节点随后一直等图像——看到 `/yolo/preview/image` 没数据先查 `ls -l /dev/video0` 和 `rosparam get /usb_camera`。
- 队友改了 `yolo_trt_ros` 源码或新增了 `.engine` 才需要重编译：`cd ~/gcs_ws && catkin_make -j2`（**只在地面做**，devel 里已经编好当前版本）。
- 终端 4 的录包器话题表是固定的（`BASE_TOPICS` + `/drone_0_*`），**不含 YOLO 话题**。要留飞行中的检测结果就另开一个终端、用独立目录并行录（`/yolo/detections` 很小，原始图像 / 预览图带宽大，先地面试一次看磁盘增长）：

  ```bash
  export FLIGHT_DIR="$(cat "$HOME/flight_logs/current_hybrid_session.txt")"
  rosbag record -O "$FLIGHT_DIR/yolo.bag" /yolo/detections /usb_cam/image_raw
  ```


## 落地后：停录制并验收

1. 确认实际落地、已锁桨，让录制再跑 5～10 秒。
2. 只在**终端 4** 按一次 `Ctrl+C`，等索引与结束快照写完；不要 `kill -9`、不要直接断电。
3. 验收：

```bash
source ~/gcs_ws/devel/setup.bash
export FLIGHT_DIR="$(cat "$HOME/flight_logs/current_hybrid_session.txt")"
ls -lh "$FLIGHT_DIR/recording/"
python3 -m json.tool "$FLIGHT_DIR/recording/manifest.json"
find "$FLIGHT_DIR/recording" -maxdepth 1 -name '*.active' -print
```

要求：有正常结束的 `.bag`（可能多分片）、没有残留 `.active`、`coverage.missing_inputs_or_maps` 为 `[]`。分片要多片合计看，只看第一片会漏掉后半段的目标与降落。

## 每次都必须守住的几条

- 只有一套 EKF、一套 PX4Ctrl、一套混合专用 Planner；不并行第二份任务脚本。
- 危险品投放默认开启，**先起 `px4_servo` 再跑任务**；不投放就用 `--no-hazard-drop`，不要用 `retract` 或改 `hold_s` 去"关"它。
- 舵机推出后按设计**不收回**；空中不要手调 `/servo/retract`、`/servo/set_pulse_us` 或重启舵机节点。
- 实机 master 上**绝不** `rosbag play`；回放必须用独立 master（手册第 12 节）。
- 空中不编译、不重启节点、不重跑任务、不改控制参数、不重置定位；新一轮只能在落地锁桨后准备。
- 脚本退出、报错或 Ctrl+C **不等于**飞机停桨或降落；异常一律飞手立即接管，不靠脚本自己纠正。
- 录包器退出不影响飞行安全，但不要在空中为补包重启控制/定位链。
