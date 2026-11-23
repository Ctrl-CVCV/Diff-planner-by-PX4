# 四路物资舵机测试工具

> 更新：2026-11-22
> 程序：`~/gcs_ws/src/px4_servo/scripts/payload_servo_test.py`
> 配套：`roslaunch px4_servo payload_servos.launch`

## 1. 它能做什么

按逻辑编号测试四路物资舵机，只调用 `/servoN/push`、`/servoN/retract`、
`/servoN/set_pulse_us`，**不会解锁、不会切模式、不会发任何飞控指令**。

| 选项 | 动作 | 服务 |
| --- | --- | --- |
| 1 | 1 号物资（消防） | `/servo3/push`（MAIN7，500→2500） |
| 2 | 2 号物资（救援） | `/servo4/push`（MAIN8，500→2500） |
| 3 | 下放 3/4 号物资 | `/servo1/push` + `/servo2/push`（MAIN5/6，500→1143） |
| 4 | 3 号物资（救援） | `/servo3/push` |
| 5 | 4 号物资（救援） | `/servo4/push` |
| r | 全部收舵 | `/servo1../servo4/retract` |
| p | 自定义脉宽 | `/servoN/set_pulse_us`（Float32，调试用） |
| s | 显示状态 | 读取 `/servoN/state` 与 `/mavros/state` |

## 2. 测试前

1. **拆桨**，机架固定，电池/飞控连接正常；
2. 启动 MAVROS 与四路舵机：

   ```bash
   roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600
   roslaunch px4_servo payload_servos.launch
   ```

3. 确认四路 ready：

   ```bash
   source ~/gcs_ws/devel/setup.bash
   python3 ~/gcs_ws/src/px4_servo/scripts/payload_servo_test.py --status
   ```

   期望每一路 `ready=True`，并显示 channel/function/actuator set/limits。
   若 `ready=False`，`错误` 行会给出原因，常见是飞控参数未配置：

   ```text
   PWM_MAIN_FUNC5..8 = 301 / 302 / 303 / 304（任意 301..308 均可，四路必须唯一）
   PWM_MAIN_MIN5..8  = 500
   PWM_MAIN_MAX5..8  = 2500
   PWM_MAIN_DIS5..8  = 500
   PWM_MAIN_FAIL5..8 = 500
   ```

4. **PX4 上锁时 MAIN 输出是 `PWM_MAIN_DIS5..8`（500 µs）**，舵机动作看不到。
   要在真实输出上验证，需要拆桨后解锁（RC 或 QGC），并保持机架固定。

## 3. 运行

交互菜单（推荐）：

```bash
python3 ~/gcs_ws/src/px4_servo/scripts/payload_servo_test.py
```

脚本化单步：

```bash
python3 .../payload_servo_test.py --action 1        # 1 号
python3 .../payload_servo_test.py --action lower    # 下放 3/4
python3 .../payload_servo_test.py --action all      # 1→2→下放→3→4
python3 .../payload_servo_test.py --action retract  # 全部收舵
python3 .../payload_servo_test.py --pulse 3 1143    # 直接发脉宽（调试）
```

每次动作都会要求确认；`--yes` 可跳过（不建议）。动作后工具等待约 3 s
（`hold_s=2 s` 自动回位），再打印各路状态。

## 4. 建议的测试顺序

1. `--status` 确认四路 ready；
2. `--action retract`，确认 `commanded_us=500`；
3. 解锁（拆桨）后依次测 `1`、`2`、`lower`、`3`、`4`，观察舵机方向和行程；
4. 跑一次 `--action all` 验证顺序；
5. 最后 `--action retract`，上锁。

## 5. 故障处理

| 现象 | 处理 |
| --- | --- |
| 某路无状态 | `roslaunch px4_servo payload_servos.launch` 没起，或节点名冲突 |
| `ready=False`、读不到 `PWM_MAIN_FUNC*` | 飞控参数未配置，按第 2 节设置并 `rosservice call /servoN/reload` |
| 服务调用成功但舵机不动 | PX4 上锁（DIS=500）；解锁后才有实际输出 |
| `push_ok=False` | push 脉宽超出 MIN/MAX 区间，检查 `PWM_MAIN_MIN5..8/MAX5..8` |
| 舵机方向反了 | 不要改工具；确认机械安装和脉宽对应关系 |

## 6. 与任务脚本的关系

任务脚本调用的就是同一组服务（`hazard_drop/steps`、`rescue_drop/payloads`）。
本工具验证通过后，任务里的动作序列才有意义；但它**不验证**任务状态机、检测投票和降高度逻辑。
