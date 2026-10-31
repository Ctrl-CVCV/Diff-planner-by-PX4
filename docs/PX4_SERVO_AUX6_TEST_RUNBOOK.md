# PX4 AUX6 舵机测试全流程与注意事项

更新：2026-10-30。配套代码：[`src/px4_servo`](../src/px4_servo/)（节点 + 单测 + 包内 README）。

本文整理**从飞控参数到真实脉宽**的完整分层测试：每一层都给出命令、期望输出和判据，
没通过的层不要往下一层走。文末有记录模板，测试完把实测值填进去，后续排故就靠它。

> **边界说明**：截至 2026-10-30，第 1～3 层已在本机实测通过；**第 4、5 层（解锁后真实脉宽、
> 舵机实际动作、带负载推出位置）尚未做过**，必须拆桨在实机上走一遍才算验证。本文不宣称飞过。

---

## 1. 链路与关键数字

```text
/servo/push 或 /servo/set_pulse_us
        ↓
servo_aux_pwm.py   把 µs 换成归一化量（读 PWM_AUX_MIN6 / MAX6 自己算中点）
        ↓  /mavros/cmd/command  (MAV_CMD_DO_SET_ACTUATOR = 187, param6)
      MAVROS
        ↓  串口 / USB
      PX4     输出功能 306 = "Offboard Actuator Set 6"
        ↓  pulse = v·(max−min)/2 + (min+max)/2
   AUX6 PWM 引脚 → 舵机
```

| 量 | 值 | 说明 |
|---|---|---|
| 脉宽区间 | 500 – 2500 µs | 来自 `PWM_AUX_MIN6` / `PWM_AUX_MAX6` |
| 归一化中点 | 1500 µs | **由 (min+max)/2 决定**，本固件没有 `PWM_AUX_TRIM6` |
| 收起位 `rest_us` | 500 µs | 归一化 −1.0 |
| 推出位 `push_us` | **1400 µs** | 归一化 −0.1；2026-10-31 实测的推出行程（原为 2500 → 1200，再改 1400） |
| 上锁输出 `PWM_AUX_DIS6` | 500 µs | 必须等于收起位 |
| 失效保护 `PWM_AUX_FAIL6` | 500 µs | 要"保持最后值"就设 −1（不推荐用于推料机构） |
| 输出频率 | 50 Hz（20 ms 周期） | 1400 µs ≈ 7%，500 µs ≈ 2.5%；舵机认的是脉宽，不是占空比 |
| 下发频率 | 5 Hz | `publish_hz`，持续覆盖，防止被别的来源改写 |
| 保持时间 | **0 s（不自动收回）** | `hold_s` 默认 0：推到位后一直保持，直到 `retract` 或下一条动作指令；调 >0 才自动收回 |

---

## 2. 安全红线（先看这一节）

1. **拆桨。** 第 2 层以后都要接飞控，第 4 层要解锁。桨不拆不许往下做。
2. **舵机单独供电。** 用独立 BEC/电池给舵机供电，**信号地必须和飞控共地**。
   大扭矩舵机不要从飞控 AUX 的 +5V 取电，电流冲高会把飞控拉复位。
3. **先空载，后带料。** 第 4 层舵机空转即可；带物料定位放到第 5 层，且不许飞行中做。
4. **节点退出不会自动回位**（`shutdown_retract:=false`）：位置是锁存的。
   带电带桨时，先 `rosservice call /servo/retract` 再停节点，或启动时给 `shutdown_retract:=true`。
5. **上锁时舵机不会响应。** PX4 上锁时输出被强制成 `PWM_AUX_DIS6`，与节点算的值无关。
   "未解锁下 push 不动"是正常现象，不是故障。
6. **飞行中不要测。** 这个节点只发执行机构指令、不碰 setpoint，不会干扰姿态控制，
   但推料是落地动作；任何飞行中的改动都要当成改飞行流程，单独评审。
7. 改动 `PWM_AUX_*` 参数会立刻改变上锁/失效时的输出位置，**改之前确认舵机空载、桨已拆**。

---

## 3. 第 0 层：飞控参数核对与修改

```bash
source ~/gcs_ws/devel/setup.bash
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600   # 只起 MAVROS，不用雷达
rostopic echo -n 1 /mavros/state                          # 期望 connected: True
```

逐项核对（本机 2026-10-30 的 `mavparam dump` 值见"本机"列）：

```bash
for p in PWM_AUX_FUNC6 PWM_AUX_MIN6 PWM_AUX_MAX6 PWM_AUX_DIS6 PWM_AUX_FAIL6; do
  echo "== $p"; rosservice call /mavros/param/get "{param_id: '$p'}"
done
```

| 参数 | 期望 | 本机 | 说明 |
|---|---|---|---|
| `PWM_AUX_FUNC6` | 306 | 306 ✅ | Offboard Actuator Set 6；301–306 对应 set 1–6 |
| `PWM_AUX_MIN6` | 500 | 500 ✅ | 解锁后最小输出 = 收起位 |
| `PWM_AUX_MAX6` | 2500 | 2500 ✅ | 解锁后最大输出。只需覆盖推出位 1400，保持 2500 不用改 |
| `PWM_AUX_DIS6` | 500 | 500 ✅（2026-10-30 已改） | 上锁时的输出。原来是 2500 = 上锁就把舵机顶到推出位 |
| `PWM_AUX_FAIL6` | 500 | 500 ✅（2026-10-30 已改） | 原来是 −1 = 保持最后值 |
| `PWM_AUX_TRIM*` | — | 不存在 | 本固件没有这个参数，**不要去找它** |

需要改的时候：

```bash
rosrun mavros mavparam set PWM_AUX_DIS6 500
rosrun mavros mavparam set PWM_AUX_FAIL6 500
rosservice call /mavros/param/get "{param_id: 'PWM_AUX_DIS6'}"   # 回读确认
```

补充确认（可选，但输出不是普通 PWM 时下面全都不成立）：

```bash
rosservice call /mavros/param/get "{param_id: 'PWM_AUX_RATE'}"    # 50 左右为常见值
rosservice call /mavros/param/get "{param_id: 'DSHOT_CONFIG'}"    # 非 0 且覆盖该组 = 不是 PWM，舵机不认
```

**判据**：五个关键参数与上表一致；`PWM_AUX_FUNC6 = 306`。不一致就先改，改完再做第 1 层。

---

## 4. 第 1 层：纯逻辑单测（不需要 ROS、不需要飞控）

```bash
cd ~/gcs_ws/src/px4_servo
python3 -m unittest discover -s tests -p 'test_*.py'
```

**期望**：`Ran 28 tests ... OK`。

覆盖：min/max 中点换算（含非对称区间）、500→−1.0 / 1400→−0.1 / 1500→0.0 / 2200→0.7 / 2500→+1.0、
往返换算、区间不含目标脉宽时报错、功能号 301–306→1–6、只填 param6 其余 NaN。

**判据**：全绿。红了不要往下走，先把换算修对——后面所有现象都建立在换算正确的前提上。

---

## 5. 第 2 层：dry_run（验证节点自身，不碰飞控）

```bash
source ~/gcs_ws/devel/setup.bash
roslaunch px4_servo servo_aux_pwm.launch dry_run:=true
```

另开终端：

```bash
source ~/gcs_ws/devel/setup.bash
rosservice call /servo/push            # 期望 success: True, "push -> 1400 us"
rosservice call /servo/retract         # 期望 "retract -> 500 us"
rostopic pub -1 /servo/set_pulse_us std_msgs/Float32 "data: 1800.0"   # 归一化 0.3
rosservice call /servo/reload
```

节点日志里应出现（只在本值变化时打印）：

```text
[dry_run] MAV_CMD_DO_SET_ACTUATOR(187) params=['nan','nan','nan','nan','nan', -0.1] param7=0.0
```

**判据**：
- 自检行 `区间 500..2500 us（中点 1500）... push 1400 us（归一化 -0.100）` 且没有 `节点已启动但当前不可动作`
- 只有第 6 位有数，前 5 位是 `nan`
- 默认 `hold_s=0`：`push` 后一直保持 `-0.1`，只有 `retract` 才回 `-1.0`；用 `hold_s:=2.0` 启动时才在 2 s 后自动回 `-1.0`

> 改了 `scripts/*.py` **不用重新 `catkin_make`**：`devel/lib/px4_servo/*.py` 是 relay 脚本，
> 直接 `exec` 源码。只有改 `CMakeLists.txt` / `package.xml` / 新增文件才需要重新编译。

---

## 6. 第 3 层：接 MAVROS 只读核对（未解锁）

飞控保持**上锁**。起节点（不带 `dry_run`），只看它读到了什么：

```bash
roslaunch px4_servo servo_aux_pwm.launch
```

期望自检行带"上锁输出"：

```text
AUX6：输出功能 306（Offboard Actuator Set 6），由 DO_SET_ACTUATOR 的 param6 驱动；
区间 500..2500 us（中点 1500）；rest 500 us；push 1400 us（归一化 -0.100）；收起归一化 -1.000；上锁输出 500 us
[WARN] 飞控未解锁：AUX 输出此时是 PWM_AUX_DIS6 = 500 us，不是本节点算出的位置；收起位 500 us
```

再读状态（1 Hz JSON）：

```bash
rostopic echo /servo/state
```

```json
{"ready":true,"reason":null,"channel":6,"function":306,"actuator_set":6,
 "limits_us":[500,1500,2500],"disarmed_us":500,"disarmed_matches_rest":true,
 "commanded_us":500,"normalized":-1.0,"holding":false,"rest_ok":true,"push_ok":true,
 "connected":true,"armed":false,"acks":{"ok":34,"fail":0},"last_result":0,"dry_run":false}
```

**判据**：
- `ready: true`，`disarmed_matches_rest: true`
- `acks.ok` 在增长（约 5 次/秒），`acks.fail: 0`，`last_result: 0`（ACCEPTED）
- 若出现 `上锁输出 2500 us` 或 `disarmed_matches_rest: false` → 回到第 0 层改 `PWM_AUX_DIS6`

本机实测（2026-10-30）：命令往返 **12 ms**，未解锁状态下 PX4 直接回 `result: 0`。

**此时舵机不会动**（上锁 → 输出固定 500 µs）。这是预期的，不要来回报 bug。

---

## 7. 第 4 层：解锁实测（**拆桨**）

这一层才真正验证"1400 µs 有没有出到引脚"。**示波器或逻辑分析仪接 AUX6 信号脚—GND**，
舵机可以先不接，先看波形；或者接空载舵机直接看转角和行程。

先按默认 `hold_s=0` 走一遍（推出后不收回，这是投放任务要的行为）：

```bash
# 终端 A：MAVROS（第 0 层已起可复用）
# 终端 B：节点
roslaunch px4_servo servo_aux_pwm.launch

# 终端 C：解锁 → 推 → 收回 → 上锁
rosservice call /mavros/cmd/arming "value: true"
rostopic echo -n 1 /mavros/state          # 确认 armed: True
rosservice call /servo/push               # 看示波器：跳到 1400 µs 并一直保持
rosservice call /servo/retract            # 显式回 500 µs（hold_s=0 时必须显式调）
rosservice call /mavros/cmd/arming "value: false"
```

期望时序（默认 `hold_s=0`）：`500 µs → 1400 µs（保持，不自动回）→ retract → 500 µs`。
容差按舵机手册，一般 ±10 µs 内算正常。

再做一次自动回位（`hold_s:=2.0`，验证计时路径）：

```bash
roslaunch px4_servo servo_aux_pwm.launch hold_s:=2.0
rosservice call /servo/push               # 1400 µs 保持 2 s，然后自动回 500 µs
```

同时用 `servo_watch` 边操作边看节点侧（每秒一行，和示波器/舵机动作对照）：

```bash
rosrun px4_servo servo_watch.py
# 首次使用需要先 catkin_make；不想编译就直接跑源码：
# python3 ~/gcs_ws/src/px4_servo/scripts/servo_watch.py
```

默认 `hold_s=0` 时的样子（`push` 后一直 holding，直到 `retract`）：

```text
us=  500 norm=  -1.0 holding=False armed=False ok=0 fail=0 result=None
us= 1400 norm=  -0.1 holding=True  armed=False ok=0 fail=0 result=None
us= 1400 norm=  -0.1 holding=True  armed=False ok=0 fail=0 result=None
us= 1400 norm=  -0.1 holding=True  armed=False ok=0 fail=0 result=None
us=  500 norm=  -1.0 holding=False armed=False ok=0 fail=0 result=None
```

（上面是 dry_run 的样子：`ok/fail` 恒为 0、`result=None`。接真飞控解锁后应为
`armed=True`、`ok` 每秒约涨 5、`result=0`。）

| 时间点 | `commanded_us` | `normalized` | `holding` |
|---|---|---|---|
| push 前 | 500 | −1.0 | false |
| push 后（默认 hold_s=0） | 1400 | −0.1 | true，一直保持 |
| 显式 `retract` 后 | 500 | −1.0 | false |
| （对照）`hold_s=2.0` 时 push 后 0～2 s | 1400 | −0.1 | true |
| （对照）`hold_s=2.0` 时 2 s 后 | 500 | −1.0 | false |

**判据**（四条全过才算这一层通过）：
1. 示波器：解锁后静止时 500 µs，`push` 时 1400 µs 并保持；`retract` 后回 500 µs；
   `hold_s:=2.0` 时 2 s 后自动回 500 µs
2. `acks.fail` 仍为 0，`last_result` 为 0
3. 上锁后输出回到 500 µs（`PWM_AUX_DIS6`）
4. 舵机实际行程是从收起位到推出位的行程，没有卡顿/抖动/异响

四条缺任何一条，按第 9 节判读表处理，不要进入第 5 层。

> 注意 `push_us = 1400` 只是区间 `500..2500` 内的一个点（归一化 −0.1），**不是**区间上限。
> 2026-10-31 实测推出行程就是 1400 µs，所以 `PWM_AUX_MAX6` 保持 2500 即可，不用改。
> 机械行程仍以第 8 节带负载扫描的结果为准；若发现 1400 推不到位或顶死，再按第 8 节调整。

---

## 8. 第 5 层：带负载定位（标定真正能推出物料的脉宽）

装料、舵机连上机构，**仍然拆桨、仍然在地面**，用 `set_pulse_us` 在 1400 附近逐点扫：

```bash
for us in 1200 1300 1400 1500 1600; do
  echo "== $us"; rostopic pub -1 /servo/set_pulse_us std_msgs/Float32 "data: $us"
  read -p "观察物料是否被推出，回车继续" _
  rostopic pub -1 /servo/set_pulse_us std_msgs/Float32 "data: 500.0"   # 每次回收起位，便于重复观察
done
```

- 2026-10-31 的实测结论是 **1400 µs 能推出**。这一层要做的是确认它在带料状态下也成立，
  以及推到位后是否顶死（有异响/电流声就说明行程过冲）。
- 若 1300 µs 就够，把 `push_us` 改小更稳妥；若 1400 推不到位，再往上试 1500/1600，
  找到**刚好推出**的最小值，改 launch 参数或写死默认值。
- 默认 `hold_s=0`，每次 `set_pulse_us` 后会**一直保持**，所以上面每轮手动发 500 回位；
  用 `hold_s:=2.0` 启动则 2 s 后自动回收起位，扫描时更省事。
- 不要靠抬 `PWM_AUX_MAX6` 来加大行程：行程由 `push_us` 决定，MAX 只是输出区间上限，
  抬 MAX 会让同一脉宽的归一化值变小、调试时更容易搞混。抬 MAX 前先确认机构机械限位。
- 把标定结果填进第 11 节记录模板。

---

## 9. 判读表：现象 → 含义 → 处理

| 现象 | 含义 | 处理 |
|---|---|---|
| `节点已启动但当前不可动作：读不到飞控参数 PWM_AUX_FUNC6` | MAVROS 参数缓存没同步 / 名字不对 | 见第 10.1 节 |
| 自检说"输出功能 0（Disabled）" | `PWM_AUX_FUNC6` 不是 301–306 | 改成 306，`/servo/reload` |
| `last_result: 3`（UNSUPPORTED） | 固件不认 187，或该通道不是 Offboard Actuator Set | 核对 FUNC6、固件版本 |
| `last_result: 2` / `4`（DENIED / FAILED） | 参数写错，或输出被别的功能占用 | 核对 `PWM_AUX_FUNC6`、`SYS_CTRL_ALLOC` |
| `acks.ok` 一直涨但舵机不动 | 命令被接受了，问题在输出侧 | 查接线、共地、输出协议（必须 PWM 非 DShot）、舵机供电 |
| 上锁时舵机被顶到推出位 | `PWM_AUX_DIS6` 不是 500 | 改回 500（第 3 节） |
| 解锁后静止位置不是 500 µs | `PWM_AUX_MIN6` 不是 500，或机构机械零点不对 | 先核对参数，再查机械 |
| `push` 被拒：`目标脉宽 1400 us 超出当前输出区间` | 区间不含 1400（`MIN6`/`MAX6` 配错） | 核对 MIN6=500 / MAX6=2500，`/servo/reload` |
| 舵机抖动/复位/飞控重启 | 舵机从飞控 +5V 取电，电流冲击 | 换独立 BEC，共地 |
| 关节点时刷 `DO_SET_ACTUATOR 调用失败：returned no response` | Ctrl-C 时有调用卡在等 ack，连接被拆 | 不是故障，2026-10-30 已改成 shutdown 时静默 |
| `arming` 被拒、`armed` 一直是 False | 预检没过（EKF/GPS/传感器），不是舵机问题 | `rostopic echo /mavros/statustext/recv` 看原因；必要先在 QGC 里确认能解锁 |

---

## 10. 常见问题

### 10.1 `PR: Unknown parameter to get: PWM_AUX_FUNC6`

这是 MAVROS 说**飞控上没有这个名字**，或者它的参数缓存没同步，不是节点算错。先分清两种情况：

```bash
rosservice call /mavros/param/get "{param_id: 'SYS_AUTOSTART'}"   # 肯定存在的参数
```

- 这个也报 Unknown → MAVROS 没同步或没连上飞控，先看 `/mavros/state` 的 `connected`。
- 这个正常 → 强制拉全量参数（顺便把缓存填满）：

```bash
rosrun mavros mavparam dump /tmp/px4_params.txt -f -qgc     # 约十几秒
grep -in "PWM_AUX\|PWM_MAIN\|_FUNC" /tmp/px4_params.txt | head -30
```

**本机 2026-10-30**：dump 出来 `PWM_AUX_FUNC6 = 306` 是对的，`-f` 拉完之后 `param/get` 就正常了。
结论是缓存没同步，不是参数名错、也不是节点算错。

### 10.2 为什么不用 `PWM_AUX_TRIM6`

PX4 的 `output_limit_calc` 是围绕 min/max 中点的对称映射：`pulse = v·(max−min)/2 + (min+max)/2`，
**没有 trim 项**。旧固件里的 `PWM_AUX_TRIMn` 是"归一化小偏置"（约 ±0.2），不是中位脉宽；
本机固件里这个参数根本不存在。所以节点只读 `MIN` / `MAX`，中点自己算。

### 10.3 上锁 / 失效时舵机在哪

- 上锁：输出 = `PWM_AUX_DIS6`，与节点当前算的位置无关。
- 失效保护触发：`PWM_AUX_FAIL6`，本机已设为 500（回收起位）。设成 −1 会保持最后值，
  意味着空中失控时物料可能停在推出状态。
- 断电：舵机断电，位置由机械回位（如果有弹簧）。

### 10.4 节点退出后舵机停在推出位

输出是锁存的，节点退出不会自动回位。要么先 `retract` 再停，要么启动时给
`shutdown_retract:=true`（尽力而为，不保证送达）。**带桨时不要留下推出状态。**

---

## 11. 收尾与回滚

```bash
rosservice call /servo/retract
rosservice call /mavros/cmd/arming "value: false"
# 停节点、停 MAVROS
```

参数回滚（如果临时改过）：

```bash
rosrun mavros mavparam set PWM_AUX_DIS6 500
rosrun mavros mavparam set PWM_AUX_FAIL6 500
rosrun mavros mavparam set PWM_AUX_FUNC6 306
```

`PWM_AUX_FUNC6 = 0` 可以把该通道彻底关掉（该通道不再输出），是最后的安全阀。

---

## 12. 测试记录模板

复制到 `~/flight_logs/<日期>_servo/README.md`，或直接贴进汇报：

```markdown
### AUX6 舵机测试记录 <日期 时间>
- 飞控/固件：
- 参数快照（mavparam dump）：FUNC6=__ MIN6=__ MAX6=__ DIS6=__ FAIL6=__ TRIM*存在?
- 第 1 层单测：Ran __ tests 结果 __
- 第 2 层 dry_run：自检行 __ ；push 回位 __
- 第 3 层 未解锁：ready=__ acks.ok=__ fail=__ last_result=__ 往返 __ms
- 第 4 层 解锁（拆桨）：
  - 示波器/逻辑分析仪：静止 __µs，push __µs，保持 __s，回位 __µs
  - 舵机动作：□ 完整行程 □ 卡顿 □ 无动作
- 第 5 层 带料：刚好推出的脉宽 __µs，机构限位 __µs，最终 push_us=__
- 异常与处理：
- 结论：□ 通过，可用 □ 不通过，原因
```

---

## 13. 与飞行流程的关系

- 自 2026-10-31 起，`hybrid_mission.py` 的 `hazard_drop` **默认开启**：
  在 C/D 点稳定悬停并投票确认"危险品"后，先降到离地 `height`，调用 `/servo/push`，
  保持 `push_wait` 秒让载荷脱离，再爬回该点高度继续任务。调用在后台线程里，
  不阻塞 50 Hz 位置指令流；服务缺失或返回失败只告警，不中止飞行。
- 启用时任务预检要求本节点已启动（服务可发现），否则拒绝起飞；`--no-hazard-drop`
  可以临时关闭。关闭时不订阅检测话题、不调用任何舵机服务。
- **推出后不收回**：默认 `hold_s=0`，任务脚本不会调用 `retract`。要收回只有两种情况：
  显式调 `/servo/retract`，或飞控上锁且 `PWM_AUX_DIS6=500`（本机已改）。
- 节点只发 `DO_SET_ACTUATOR`，不占用 setpoint 话题，不改变 PX4Ctrl 的控制方式；
  但它是**执行机构**，飞行中任何一次误触发都会改变飞机上物料的机械状态。
  改 `hazard_drop` 的触发条件属于改飞行流程，要单独评审。

---

## 14. 接口与参数速查

| 名称 | 类型 | 作用 |
|---|---|---|
| `/servo/push` | `std_srvs/Trigger` | 到 `push_us`（1400）；默认 `hold_s=0` 一直保持，`hold_s>0` 时保持该秒数后自动回收起位 |
| `/servo/retract` | `std_srvs/Trigger` | 回收起位（500），保持到下一次指令 |
| `/servo/set_pulse_us` | `std_msgs/Float32` | 发任意脉宽（受区间校验与 `hold_s` 约束） |
| `/servo/reload` | `std_srvs/Trigger` | 重读飞控参数（改完 PX4 参数后调用） |
| `/servo/state` | `std_msgs/String` | 1 Hz JSON 状态 |

| launch 参数 | 默认 | 说明 |
|---|---|---|
| `channel` | 6 | 决定读哪组 `PWM_AUX_*` |
| `param_prefix` | `PWM_AUX` | MAIN 输出改 `PWM_MAIN` |
| `rest_us` / `push_us` | 500 / **1400** | 收起位、推出位（1400 = 归一化 −0.1，2026-10-31 实测行程） |
| `hold_s` | **0.0** | 0 = 一直保持到 `retract`（投放任务要求推出后不收回） |
| `publish_hz` | 5.0 | 持续下发频率 |
| `arming_gate` | `any` | 可设 `armed` / `disarmed` |
| `shutdown_retract` | `false` | 退出时是否尽力回位 |
| `dry_run` | `false` | 只打印不调用 MAVROS |
| `mavros_ns` / `topic_prefix` | `/mavros` / `/servo` | 命名空间 |

包内私有参数（launch 未暴露）：`~limits_override {min_us, max_us}`（无飞控时的兜底）、
`~param_timeout`、`~param7`、`~warn_when_disarmed`、`~dry_run_index`。
