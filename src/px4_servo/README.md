# px4_servo：AUX 输出舵机（默认 AUX6，500 → 1400 µs 推出物料）

通过 `MAV_CMD_DO_SET_ACTUATOR`（187）驱动 PX4 AUX 输出上的舵机。**不经过 RC 通道，
也不占用姿态/电机输出**：只写 "Offboard Actuator Set N" 这一个外设执行机构集合。

## 1. 原理：为什么必须改 PX4 参数

| 环节 | 内容 |
|---|---|
| 输出功能 | `PWM_AUX_FUNC6` 设成 **306** = "Offboard Actuator Set 6"（301–306 依次对应 set 1–6） |
| 命令 | 本节点持续发 `MAV_CMD_DO_SET_ACTUATOR`，**param6** 填归一化量，param1–5 填 NaN（PX4 忽略非有限值，不会动别的输出） |
| 还原脉宽 | PX4 用该通道的 `PWM_AUX_MIN6` / `PWM_AUX_MAX6` 把归一化量算回脉宽（`output_limit_calc`）：`pulse = v·(max−min)/2 + (min+max)/2`，**没有 trim 项**，0 点就是 min/max 的中点 |
| 结论 | 本机 AUX6 配成 `MIN6 = 500` / `MAX6 = 2500`，中点自然是 1500，映射就是 `500↔-1.0`、`1400↔-0.1`、`1500↔0.0`、`2500↔+1.0`；区间不含目标脉宽时节点直接拒绝并报错 |

本节点每次都从飞控读 `_MIN` / `_MAX` 自己算归一化量，所以改完飞控参数不用改代码；反过来，
参数不匹配时它会拒绝动作并打印具体要改哪个参数，不会静默把舵机卡在错误位置。

> 旧固件（以及很多网上的 PX4 资料）里还有 `PWM_AUX_TRIM1..8`。那是**归一化小偏置**（约 ±0.2），
> 不是"中位脉宽"，换算方式也不同。本机 `mavparam dump` 里**根本没有** `PWM_AUX_TRIM*`，
> 所以本节点不读它，只按 min/max 中点换算。

## 2. PX4 侧配置（QGC → Vehicle Setup → Parameters）

| 参数 | 值 | 说明 |
|---|---|---|
| `SYS_CTRL_ALLOC` | 1 | 新版执行机构配置；旧固件用 mixer 文件时 187 命令不一定被处理 |
| `PWM_AUX_FUNC6` | 306 | Offboard Actuator Set 6 |
| `PWM_AUX_MIN6` | **500** | 解锁后的最小输出，等于收起位（本机已是 500） |
| `PWM_AUX_MAX6` | **2500** | 解锁后的最大输出，只需覆盖推出位 1400（本机已是 2500，留着即可） |
| `PWM_AUX_DIS6` | **500** | **上锁时的输出**，必须与收起位一致。**本机现在是 2500**，等于上锁就把舵机顶到推出位，必须改 |
| `PWM_AUX_FAIL6` | **500** | 失效保护位。本机是 `-1` = 不做任何输出（保持最后值），建议改成 500 让失控时回到收起位 |

本机 2026-10-30 的 `mavparam dump` 实测：`FUNC6=306`（已正确）、`MIN6=500`、`MAX6=2500`、
`DIS6=2500`（**错**）、`FAIL6=-1`、无 `TRIM*`。节点启动时会核对 DIS 与 `rest_us`，
不一致就打印警告并在 `/servo/state` 里标 `disarmed_matches_rest: false`。

另外确认该通道的输出协议是普通 PWM（不是 DShot/OneShot），舵机才认；AUX 默认 50 Hz，
20 ms 周期下 2500 µs 即 12.5% 占空比。

改完参数重启飞控（或至少确认参数写入成功），再用 `/servo/reload` 让节点重读，不必重启节点。

## 3. 编译与启动

```bash
cd ~/gcs_ws && catkin_make -j2            # 只有 Python 脚本，很快
source ~/gcs_ws/devel/setup.bash
roslaunch px4_servo servo_aux_pwm.launch
```

启动日志会打印自检结果，例如：

```text
AUX6：输出功能 306（Offboard Actuator Set 6），由 DO_SET_ACTUATOR 的 param6 驱动；
区间 500..2500 us（中点 1500）；rest 500 us；push 1400 us（归一化 -0.100）；收起归一化 -1.000；上锁输出 500 us
```

不通过时（通道没配成 Offboard Actuator Set、读不到参数、脉宽区间不含 rest 或 push 位）会打印
`节点已启动但当前不可动作：<原因>（修好后调用 /servo/reload）`，此时所有动作都会被拒绝。

## 4. 接口

| 名称 | 类型 | 作用 |
|---|---|---|
| `/servo/push` | `std_srvs/Trigger` | 到推出位（`push_us`）；默认 `hold_s=0` 一直保持，`hold_s>0` 时保持该秒数后自动回收起位 |
| `/servo/retract` | `std_srvs/Trigger` | 回收起位（`rest_us`），保持到下一次指令 |
| `/servo/set_pulse_us` | `std_msgs/Float32` | 发任意脉宽（调试用，同样受区间校验与 `hold_s` 约束） |
| `/servo/reload` | `std_srvs/Trigger` | 重读飞控参数（改完 PX4 参数后调用） |
| `/servo/state` | `std_msgs/String` | 1 Hz JSON：是否就绪、输出功能、序号、区间 `[min, 中点, max]`、上锁输出 `disarmed_us` 与 `disarmed_matches_rest`、当前目标、ACK 计数 |
| `servo_watch.py` | 脚本 | 把上面那行 JSON 压成一行打印，测试时对照示波器/舵机动作用 |

```bash
rosservice call /servo/push
rosservice call /servo/retract
rostopic pub -1 /servo/set_pulse_us std_msgs/Float32 "data: 1800.0"
rosrun px4_servo servo_watch.py     # /servo/state 压缩成每秒一行
```

主要参数（`roslaunch ... key:=value` 覆盖）：

| 参数 | 默认 | 说明 |
|---|---|---|
| `channel` | 6 | AUX 通道号，决定读哪组 `PWM_AUX_*` 参数 |
| `param_prefix` | `PWM_AUX` | MAIN 输出改成 `PWM_MAIN` |
| `rest_us` / `push_us` | 500 / **1400** | 收起位与推出位（500 与 `PWM_AUX_DIS6` 一致；1400 为实测推出行程，归一化 -0.1；2026-10-31 由 2500 改 1200、再改 1400） |
| `hold_s` | **0.0**（2026-10-31 起） | 推到位后保持多久自动回位；填 0 表示一直保持到 `retract`。本机投放任务要求推出后不收回，节点与 launch 默认都是 0.0 |
| `publish_hz` | 5.0 | 持续下发频率（防止飞控端超时/被其它来源覆盖） |
| `arming_gate` | `any` | 可设 `armed` / `disarmed` 限制在特定解锁状态下动作 |
| `shutdown_retract` | `false` | 退出时是否尽力回位；默认不回，位置是锁存的，要回位请显式调 `retract` |
| `dry_run` | `false` | 只打印将要发的指令，不调用 MAVROS |
| `limits_override` | 空 | `{min_us, max_us}`，仅地面无飞控时绕过参数读取；缺的项按 500/2500 兜底 |

## 5. 测试顺序（分三层，前两层不需要舵机通电；第三层务必拆桨）

### 5.1 纯逻辑（不需要 ROS、不需要飞控）

```bash
cd ~/gcs_ws/src/px4_servo && python3 -m unittest discover -s tests -p 'test_*.py'
```

期望 `Ran 28 tests ... OK`。这一层验证 min/max 中点换算（含非对称区间）、越界拒绝、
序号 301–306→1–6 和 NaN 填充。

### 5.2 无飞控 dry_run（验证节点自身：服务、日志、保持/回位）

```bash
source ~/gcs_ws/devel/setup.bash
roslaunch px4_servo servo_aux_pwm.launch dry_run:=true
```

launch 默认 `hold_s=0.0`（推出后保持不收回）。要看 2 s 自动回位就用 `hold_s:=2.0` 启动。

另开终端：

```bash
source ~/gcs_ws/devel/setup.bash
rosservice call /servo/push        # 期望 success: True, "push -> 1400 us"
rostopic echo /servo/state         # limits_us [500,1500,2500]，normalized -0.1，holding true，disarmed_us null
rosservice call /servo/retract     # 期望 "retract -> 500 us"，normalized -1.0
rostopic pub -1 /servo/set_pulse_us std_msgs/Float32 "data: 1800.0"   # 中间位置，归一化 0.3
rosservice call /servo/reload
```

dry_run 下节点打印的是 `[dry_run] MAV_CMD_DO_SET_ACTUATOR(187) params=[...]`，
只在本值变化时打印；确认**只有第 6 位有数，前 5 位是 `nan`**。

### 5.3 接飞控（拆桨！先不接舵机动力，只接信号线到示波器/逻辑分析仪）

1. 只起 MAVROS（不用起雷达和 FAST-LIO）：

   ```bash
   source ~/gcs_ws/devel/setup.bash
   roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600
   rostopic echo -n 1 /mavros/state          # 期望 connected: True, armed: False
   ```

2. 确认飞控参数（节点启动时会自己读，这里人工再核一遍）：

   ```bash
   for p in PWM_AUX_FUNC6 PWM_AUX_MIN6 PWM_AUX_MAX6 PWM_AUX_DIS6 PWM_AUX_FAIL6; do
     rosservice call /mavros/param/get "{param_id: '$p'}"
   done
   # 期望 306 / 500 / 2500 / 500 / 500
   ```

   参数缓存没同步时报 `PR: Unknown parameter to get`，见 5.4。

3. 上锁状态先看：舵机应停在 500 µs。若 `PWM_AUX_DIS6` 还是 2500，上锁时舵机会被顶在推出位，
   节点会打印 `PWM_AUX_DIS6 = 2500 us，与收起位 rest_us = 500 us 不一致 ...` —— 先改参数再往下做。

4. 起节点（不带 `dry_run`），看自检行是否 `区间 500..2500 us（中点 1500）... push 1400 us（归一化 -0.100）`：

   ```bash
   roslaunch px4_servo servo_aux_pwm.launch
   ```

5. **拆桨**，然后解锁（QGC 或 `rosservice call /mavros/cmd/arming "value: true"`），
   再 `rosservice call /servo/push`，示波器上应看到 1400 µs（±10 µs），机构行程正好推到位。默认 `hold_s=0` 会一直保持；
   用 `hold_s:=2.0` 启动时应在 2 s 后回到 500 µs。两种都试一次，确认 `retract` 能收回。

6. 判读 `/servo/state`：

   | 现象 | 含义 / 处理 |
   |---|---|
   | `acks.ok` 持续增长 | 飞控已接受命令；舵机不动就查接线、输出协议（必须 PWM 非 DShot）、`PWM_AUX_FUNC6` |
   | `last_result: 3` | UNSUPPORTED：固件不认 187，或 `SYS_CTRL_ALLOC` 没开 / 该通道没设成 306 |
   | `last_result: 2` / `4` | DENIED / FAILED：多半是输出功能不是 Offboard Actuator Set，或参数写错 |
   | `ready: false` | 看 `reason`，按提示改参数后 `rosservice call /servo/reload` |
   | 上锁时就能动 / 停在推出位 | `PWM_AUX_DIS6` 不是 500（本机是 2500），改回 500 |

7. 确认 `retract` 回到 500 µs、再上锁确认舵机位置不变，才装桨。

### 5.4 报 "PR: Unknown parameter to get: PWM_AUX_FUNC6"

这是 MAVROS 说**飞控上没有这个名字的参数**，不是节点算错。先分清两种情况：

```bash
# 1) 参数服务本身通不通：换一个肯定存在的参数
rosservice call /mavros/param/get "{param_id: 'SYS_AUTOSTART'}"
```

- 这个也报 Unknown → MAVROS 参数没同步或没连上飞控，先看 `/mavros/state` 的 `connected`。
- 这个正常返回 → 名字不对。查真实名字：

```bash
rosrun mavros mavparam dump /tmp/px4_params.txt -f -qgc     # 从飞控强制拉全量参数，约十几秒
grep -in "PWM_AUX\|PWM_MAIN\|_FUNC" /tmp/px4_params.txt | head -30
```

节点自己也会报：读不到 `PWM_AUX_FUNC6` 时会打印一行"飞控上实际存在：…"，直接把真实参数名列出来，
按它改 `param_prefix` / `channel` 后 `rosservice call /servo/reload`，不用重启节点。

常见原因：输出组叫 `PWM_MAIN_*`（用 `param_prefix:=PWM_MAIN`）、通道号不是 6、或在 QGC 的
Actuators 页面用新界面配过而没写入参数。都不匹配时要先确认这架飞控跑的确实是 PX4。

**本机 2026-10-30 的结论**：`mavparam dump -f` 拉出来的参数里 `PWM_AUX_FUNC6 = 306` 是对的，
所以那次 `Unknown parameter to get: PWM_AUX_FUNC6` 是 **MAVROS 参数缓存没同步**，不是名字错、
也不是节点算错。`dump -f` 会顺带把缓存填满，之后 `rosservice call /mavros/param/get` 就正常了。

## 6. 与任务脚本的关系

`hybrid_mission.py` 通过 `hazard_drop` 配置块调用本节点的 `/servo/push`，自 2026-10-31 起
**默认开启**（`hazard_drop/enabled: true`，可用 `--no-hazard-drop` 临时关闭）。
开启后的行为：在 C/D 点连续稳定悬停 `hover_seconds`、且最近 `vote_frames` 帧的投票确认
"危险品"（每帧只认离图像中心最近的目标，命中比例需达 `vote_ratio`）后，
任务脚本先在位置控制下降到离地 `height`，调用 `/servo/push`，保持 `push_wait` 秒
（这是低空停留时间，让载荷脱离；与 `hold_s` 无关）再爬回该点高度继续任务。调用发生在
单独的后台线程里，不阻塞 50 Hz 位置指令流；服务缺失或返回失败只告警，不中止飞行。
启用时任务脚本的预检要求本节点已启动（服务可发现），否则拒绝起飞。
关闭时任务脚本不订阅检测话题、不调用任何舵机服务。

**投放后舵机不收回**：按本机要求，`servo_aux_pwm.launch` 默认 `hold_s=0.0`，`push` 之后
保持推出位，直到显式 `/servo/retract`、`/servo/set_pulse_us` 或飞控上锁输出
（`PWM_AUX_DIS6`；改回 500 时上锁即收回）。任务脚本不会调用 `retract`。

## 7. 已验证 / 未验证

- 已验证（本机，私有 master，`dry_run`）：节点启动、min/max 中点换算（500→-1.000、1500→0.0、
  2200→0.700、2500→+1.000）、`push`/`retract`/`reload` 服务、`hold_s` 2 s 自动回位
  （日志里 0.7 → -1.0）、只填 param6 其余 NaN、区间不含目标脉宽时拒绝动作并给出修复提示。
  `servo_map` 单元测试 28 项通过（含 1200→-0.300 / 1400→-0.100 的换算与往返）。
- 已核对（本机飞控参数 dump）：`FUNC6=306`、`MIN6=500`、`MAX6=2500` 与预期一致；`DIS6=2500`、
  `FAIL6=-1` 与预期不一致，需在 QGC 改。
- **未验证**：真实飞控上的输出时序与脉宽、`PWM_AUX_DIS6` 改动后的上锁行为、飞控固件对 187 的
  实际返回码、与飞控飞行中同时运行的影响。第 5.3 节必须在实机上走一遍（拆桨）。
  `hold_s=0`（推出后一直保持、不回位）只在 `dry_run` 里确认过日志，实机长时间保持对舵机的
  电流/发热影响未测；首次带投放飞行前先用 `retract` 确认能收回。
- **2026-10-31 改动**：`push_us` 由 2500 改为 1200、再改为 **1400**（实测推出行程，
  归一化 −0.1），`hold_s` 默认改为 **0**（推出后不收回）。这两项只做了代码/配置与单元测试
  层面的核对，**尚未在实机上验证** 1400 µs 的实际行程和长时间保持表现。
- `param7` 默认发 0（MAVLink 把它定义为执行机构组索引），PX4 侧通常忽略；若固件不认，用
  `~param7` 调。
