# 救援物资无投放测试与航点高度补偿操作手册

> 更新：2026-11-14
> 适用：`hybrid_mission.py` 的 `rescue_drop`（dry_run）与 `points/*/z_offset`
> 相关实现说明：`docs/HYBRID_PLANNER_PX4CTRL_MISSION.md` 第 12、13 节
> **状态：只通过离线单元测试，尚未实飞验证。首次测试请按本手册逐项核对。**

## 1. 这次测试要验证什么

1. YOLO 能稳定识别"救助人员"，任务脚本的投票判定能确认目标；
2. 在 E、F、G 任一点确认目标后，飞机能按设定速度下降到离地 `height`，保持
   `push_wait` 秒，再爬回航点高度继续任务；
3. **全程不调用舵机、不投放**（`dry_run: true`）；
4. 航点地面高度误差 `z_offset` 能按点修正高度目标。

## 2. 当前默认配置（`config/hybrid_mission.yaml`）

```yaml
rescue_drop:
  enabled: true
  dry_run: true        # 当前测试模式：不推舵机
  once: false          # E、F、G 各自独立判断
  points: [E, F, G]
  label: 救助人员
  min_confidence: 0.70
  # 投票参数默认取 detection 段：vote_mode/vote_window/vote_frames/vote_ratio
  hover_seconds: 3.0
  wait_timeout: 10.0
  height: 0.15         # 离该点地面的高度
  speed: 0.15
  tolerance: 0.02
  timeout: 20.0
  push_wait: 2.5
  max_age: 0.5
  # dry_run=true 时整段不会被执行；测试通过后按顺序投放 2 号 →（下放）→ 3 号 → 4 号
  payloads:
    - steps:
        - {services: [/servo4/push], hold: 3.0}                    # 2 号
    - steps:
        - {services: [/servo1/push, /servo2/push], hold: 3.0}      # 下放 3/4 号
        - {services: [/servo3/push], hold: 3.0}                    # 3 号
    - steps:
        - {services: [/servo4/push], hold: 3.0}                    # 4 号
```

航点高度补偿（默认 0，需要按实测地面填写）：

```yaml
points:
  C: {xyz: [0.43, 1.33, 0.80], yaw_deg: 180.0, z_offset: 0.0}
  D: {xyz: [-0.03, 1.35, 0.8], yaw_deg: 180.0, z_offset: 0.0}
  E: {xyz: [1.90, 2.66, 0.8], yaw_deg: 90.0, measured: true, z_offset: 0.0}
```

## 3. 测试前检查（地面、锁桨）

### 3.1 确认 YOLO 标签

```bash
roslaunch yolo_trt_ros yolo_trt.launch
```

节点日志应打印：

```text
display labels: 0=危险品标识, 1=其它标识, 2=救助人员
```

另开终端：

```bash
source ~/gcs_ws/devel/setup.bash
rostopic hz /yolo/detections
rostopic echo -n 1 /yolo/detections
```

把打印出来的目标物放到画面中央，确认 `label` 字段确实是 `救助人员`、置信度 ≥ 0.70。
如果标签名不同，改 `rescue_drop/label`，不要改代码。

### 3.2 决定本次是否做危险品投放

`hazard_drop` 默认也是开启的，预检会要求它的投放服务 `/servo3/push` 可用。**本轮只测救援
降高度、不做危险品投放时，执行和预检都加 `--no-hazard-drop`**，这样不会等舵机服务、
也不会在 C/D 触发投放。救援的 `dry_run: true` 不要求舵机服务，预检不会检查
`/servo4/push` 等。

### 3.3 离线预览（不需要 ROS）

```bash
python3 ~/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/scripts/hybrid_mission.py
```

确认输出里同时出现：

```text
救援物资降高度测试已启用：E/F/G 点连续稳定悬停 3.0s ... 测试模式不调用舵机、不投放 ...
已计入航点地面高度误差 z_offset：... （或"所有航点 z_offset=0"）
```

### 3.4 只读预检（需要 ROS，不发飞行指令）

飞控地面、未解锁，且定位/Planner/PX4Ctrl 已按快速启动流程运行时：

```bash
python3 ~/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/scripts/hybrid_mission.py \
  --check --no-hazard-drop
```

预检日志里应看到 `rescue_drop 已启用（测试模式 dry_run=true）...`。如果这里报
"航点 X 尚未实测"或服务不可用，先解决再进入下一步。

## 4. 执行无投放测试

### 4.1 启动顺序

按 [混合任务最简步骤](HYBRID_MISSION_QUICKSTART.md) 的第 1～4 步启动：

1. 定位 + EKF：`bash ~/gcs_ws/src/livox_ros_driver2/scripts/start_fastlio2_ekf.sh`
   （脚本默认已切换为 faster-lio，输出仍是 `/Odometry`）
2. PX4Ctrl：`roslaunch px4ctrl run_ctrl_lio.launch`
3. 专用 Planner：`roslaunch diff_planner run_exp_hybrid_mission.launch`
4. 录包：`python3 .../record_diff_planner_debug.py`

YOLO 节点按 3.1 启动并保持运行。

### 4.2 执行任务

```bash
python3 ~/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/scripts/hybrid_mission.py \
  --execute --map-verified --direct-path-verified --no-hazard-drop
```

输入 `MISSION` 确认后起飞。另开终端观察阶段：

```bash
rostopic echo /drone_0_mission/state
```

### 4.3 E/F/G 上会发生什么

到点后的时间线（`t=0` 为到达并开始悬停）：

| 时间 | 行为 |
| --- | --- |
| 0–3 s | 到点稳定悬停（`mission/arrival_dwell`） |
| 3 s 起 | 检查投票；已确认目标 → `DROP_DESCEND` |
| 3–11 s | 以 ≤0.15 m/s 从航点高度降到离地 0.15 m（约 0.65 m 高度差，参考时长约 8 s） |
| +2.5 s | `DROP_PUSH`：低空保持，**测试模式不调用任何舵机** |
| 约 8 s | `DROP_ASCEND` 爬回航点高度，继续飞往下一点 |

未确认目标时：从到点起最多等 `wait_timeout`（10 s），然后按原任务流飞走，不下降。

日志关键字：

```text
已稳定检测到"救助人员"（rescue_drop），下降到离地 0.15m ...
测试模式（dry_run）：已降到离地 ... 不调用 /servo1../servo4、不投放
救援物资降高度测试结束（dry_run，未投放），已爬回 ... 继续飞往 ...
```

### 4.4 如何确认"确实没有投放"

同时满足这三条才算通过：

1. 任务日志出现上面的 `dry_run` 字样，且**没有**"舵机推出成功"；
2. `px4_servo` 节点日志中没有任何一次 push 动作记录；
3. 回放 bag 能看到 `DROP_DESCEND → DROP_PUSH → DROP_ASCEND` 阶段和高度曲线，
   但飞机没有在低空停留超过 `push_wait`。

可选：用 `rosbag` 里的 `/drone_0_mission/state` 核对每个点的阶段顺序。

### 4.5 想先单独验证"降高度动作"（不依赖 YOLO）

如果现场识别不稳定，可以在飞机悬停在目标点上方时，由另一终端**持续**发布合成检测
（注意：这会真实触发一次下降，只在你确认目标点下方空旷时使用）：

```bash
rostopic pub -r 20 /yolo/detections std_msgs/String \
  "data: '{\"image_width\": 640, \"image_height\": 480, \"count\": 1, \"detections\": [{\"label\": \"救助人员\", \"confidence\": 0.95, \"cx\": 320, \"cy\": 240}]}'"
```

触发后按 Ctrl-C 停止发布；`max_age` 0.5 s 之后投票自然失效。没有特别需要时不要用这条，
优先验证真实检测。

## 5. 航点高度补偿 `z_offset`

### 5.1 定义

`z_offset` = **该航点地面相对起飞原点地面的高度差**，单位米：

- 该点地面比起飞点**高** → 正数（例如 +0.07）；
- 该点地面**低** → 负数（例如 -0.05）；
- 允许范围 ±0.30，超出直接报配置错误。

脚本把 `z_offset` 加到：

- 该航点所有高度目标（Planner 目标、直控终点、悬停）；
- 危险品/救援的投放低点：`起飞原点 z + z_offset + height`；
- I 点降落进近高度（如果启用 `landing/approach_height`）。

### 5.2 怎么测

**方法 A：地面放置读 EKF（与 GCS 控制台的默认打点模式一致）**

1. 在起飞点记录 EKF 原点（任务预检后会发布 `origin_world`；没有就按 0,0,0 近似）；
2. 把飞机平放在 C 点对应的地面上，等 `/ekf/ekf_odom` 稳定后读 z；
3. `z_offset = 实测 z − 原点 z`，就是该点地面相对起飞原点的高度（实测 C 点约 0.033 m）。
4. 同理逐点测量 D/E/F/G。GCS 控制台"地面放置（推荐）"模式会直接算出这个值。

**方法 B：尺量法（没有定位时使用）**

1. 在起飞点 A 的地面放一根水平基准（铝合金型材、水平尺或拉紧的细线），用水准泡调平；
2. 依次到 C、D、E（以及需要补偿的点），用钢尺量"基准线到该点地面"的垂直距离；
3. 与 A 点的读数相减，差值就是该点的 `z_offset`。

**方法 C：手持搬运读 EKF（粗略，仅作交叉核对）**

把未上电的飞机平放在各点地面上，读 `/ekf/ekf_odom` 的 z，与起飞原点 z 相减。
手持搬运会让 LIO 漂移，只能当粗估；测完回到 A 点，读数应回到接近 0，否则说明漂移太大、
以方法 A 为准。

> 不要用"飞行时的高差"反推：飞行高度本来就是按设定值控制的，直接反推会把误差算进
> 目标里，越调越乱。

### 5.3 怎么填写

编辑 `config/hybrid_mission.yaml`：

```yaml
points:
  C: {xyz: [0.43, 1.33, 0.80], yaw_deg: 180.0, z_offset: 0.07}
  D: {xyz: [-0.03, 1.35, 0.8], yaw_deg: 180.0, z_offset: -0.03}
  E: {xyz: [1.90, 2.66, 0.8], yaw_deg: 90.0, measured: true, z_offset: 0.05}
  F: {xyz: [1.08, 2.65, 0.8], yaw_deg: 90.0, measured: true, z_offset: 0.02}
  G: {xyz: [2.66, 2.66, 0.8], yaw_deg: 90.0, measured: true, z_offset: -0.01}
```

**E、F、G 三个救援点都要单独记录**（它们是各自独立的检测/降高度点，地面高差互不相同）。
只填需要补偿的点，没写的点按 0 处理。

### 5.4 检查是否生效

```bash
python3 ~/gcs_ws/src/Diff-Planner/src/diff_planner/plan_manage/scripts/hybrid_mission.py
```

输出应包含：

```text
已计入航点地面高度误差 z_offset：C +0.07m，D -0.03m，E +0.05m（正数=该点地面高于起飞原点，...）
```

预检时还会打印每个点最终的 world 目标坐标，核对 z 是否等于
`起飞原点 z + 设定高度 + z_offset`。

### 5.5 飞行中验证

- 悬停在某点（建议先用 `--no-rescue-drop` 或未检测到目标时的普通悬停）时，
  读 `/ekf/ekf_odom` 的 z，应比不带补偿时高 `z_offset` 米；
- 真要核对"实际离地间隙"时，用带刻度的标尺/测距仪从地面量到机体参考点，
  差值应接近设定的离地高度。

## 6. 常用参数速查

| 想改什么 | 改哪里 | 说明 |
| --- | --- | --- |
| 临时关闭救援测试 | 命令行加 `--no-rescue-drop` | 不订阅检测、不降高度 |
| 正式投放（以后） | `rescue_drop/dry_run: false` | 测试通过后再改 |
| 哪些点允许触发 | `rescue_drop/points` | 只允许 E/F/G |
| 检测灵敏度 | `min_confidence` / `detection/vote_*` | 误检多就提高门槛或投票比例 |
| 检测等待时间 | `rescue_drop/wait_timeout` | 默认 10 s，到点后等这么久 |
| 下降高度 | `rescue_drop/height` | 相对该点地面 |
| 下降速度 | `rescue_drop/speed` | 不得大于 `direct/max_speed`（0.20） |
| 低空保持 | `rescue_drop/push_wait` | 测试模式下同样保持，用来模拟投放时长 |
| 地面误差 | `points/<点>/z_offset` | ±0.30 m |

## 7. 常见问题

| 现象 | 可能原因 | 处理 |
| --- | --- | --- |
| E/F/G 完全不下降 | YOLO 没运行 / 标签不是"救助人员" / 置信度不足 | `rostopic hz /yolo/detections`，核对 label 与 confidence |
| 画面里有目标但不触发 | 离图像中心最近的是别的类别（比如"其它标识"），中心目标才参与判定 | 让目标位于画面中央，或调整悬停位置 |
| 只悬停不下降，10 s 后飞走 | 投票未达 `vote_ratio`，或检测断更超 `max_age` | 看日志"命中 x 帧"；检查 YOLO 帧率/延迟 |
| 预检报 `/servo3/push` 不可用 | `hazard_drop` 仍启用 | 加 `--no-hazard-drop`（本轮不测危险品时） |
| 预检报航点未实测 | `measured: false` | 实测后改 `true`，不要为了跑通删标记 |
| 下降高度看起来不对 | `height` 与 `z_offset` 混淆 | `height` 是离地高度，`z_offset` 是地面高差 |
| 填 0.7 报配置错误 | 超过 ±0.30 上限 | 确认单位是米、正负号含义 |
| 每点多等 10 s | `wait_timeout` 默认值 | 测试阶段可接受；不想等就关 `--no-rescue-drop` |

## 8. 安全与限制

- `dry_run` 只保证"不推舵机"，飞机仍会**真实下降到离地 0.15 m**：E/F/G 下方必须空旷、
  无人员无杂物，飞手全程在控，随时准备接管。
- 该功能只有离线单元测试（`tests/test_hybrid_mission.py` 的
  `RescueDropAndGroundOffsetTest`），**没有实飞验证**；第一次建议单独飞一轮，
  不要和危险品投放测试混在一起。
- 默认开启会让 E/F/G 每点多等最多 10 s；比赛正式跑之前根据实测效果决定是否常开。
- `z_offset` 有 ±0.30 m 的硬上限，是用来修正地面高差的，不是用来改变飞行高度的；
  要改整体飞行高度请改航点 `xyz` 的 z。
