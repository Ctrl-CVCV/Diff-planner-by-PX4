# 定点起飞后交给 Diff-Planner：顺序脚本使用说明

日期：2026-10-19

## 1. 先理解“接管”

实际控制链始终是：

```text
Diff-Planner → /setpoints_cmd → PX4Ctrl → MAVROS → PX4
```

并不是让 Diff-Planner 直接给电机下指令。顺序脚本只做：

1. 检查现有节点、参数、话题、频率、遥控器与地面状态；
2. 发送一次 `/px4ctrl/takeoff_land` 起飞请求；
3. 等待 `/px4ctrl/state` 确实进入 `AUTO_HOVER(2)`；
4. 连续稳定悬停至少3秒；
5. 在稳定悬停点的基础上，向 `/goal` 发送一个短距离目标；
6. 确认收到新的 `/setpoints_cmd` 且 PX4Ctrl 进入 `CMD_CTRL(3)`。

脚本不会启动/终止控制节点，不调用解锁或模式服务，不重复起飞，不自动降落、不强制停桨。**但当前 PX4Ctrl 的 `enable_auto_arm=true` 会使起飞请求间接触发自动解锁。**

## 2. 当前不应跳过的前提

此前反馈的“空旷处有方块、障碍物处空白”必须先确认解决。不要为了让脚本通过而直接勾选地图确认选项。

至少验证：

- 原始点云和 EKF 数值坐标对齐；
- 未膨胀地图正确对应实际障碍；
- 膨胀地图中，起点、目标点以及预期飞行高度有足够自由空间；
- 没有因地面向上膨胀而把飞行高度占满；
- 目标方向无人、无硬墙，遥控器能及时接管。

`--map-verified` 仅表示操作者人工确认，不是自动碰撞检测。脚本只检查地图消息新鲜度；当前发布地图存在高度和前向裁剪，不能用它证明整条航线自由。

## 3. 文件

脚本：

```text
~/gcs_ws/src/livox_ros_driver2/scripts/takeoff_then_diff_planner.py
```

## 4. 启动顺序

终端1，已有感知与融合脚本：

```bash
source ~/gcs_ws/devel/setup.bash
bash ~/gcs_ws/src/livox_ros_driver2/scripts/start_fastlio2_ekf.sh
```

终端2，规划器与轨迹服务器：

```bash
source ~/gcs_ws/devel/setup.bash
roslaunch diff_planner run_exp_single_lio.launch
```

终端3，控制器：

```bash
source ~/gcs_ws/devel/setup.bash
roslaunch px4ctrl run_ctrl_lio.launch
```

**本次不要启动 multipoint 节点，也不要提前点击 RViz 目标。** 这样既避免多个目标源，又避免起飞前 `/setpoints_cmd` 已经持续输出。若本轮已发过规划目标，轨迹服务器可能继续输出终点指令，需要在地面、未解锁且确认安全时重启规划器启动流程；不要在空中重启控制链。

将遥控器准备到你已经验证过的中油门自动起飞组合。当前源码要求：

- CH1~CH4 全部居中，油门也居中；脚本按1500±125检查，对应源码死区；
- CH5 > 1750，CH6 > 1750；
- RC数据持续有效；
- 飞机落地、未解锁，PX4Ctrl处于 `MANUAL_CTRL(1)`。

这几个通道是本地 `input.cpp` 的实际解释，并不等同于QGC中的飞行模式名称。脚本不修改或绕过飞控的解锁条件、kill-switch、模式映射。

若控制器已经进入 `AUTO_HOVER(2)`，脚本会拒绝重复起飞；不要通过空中重启或反复发起飞命令解决。应先安全落地并恢复已验证的起飞初始流程。

## 5. 先只检查，不起飞

终端4：

```bash
source ~/gcs_ws/devel/setup.bash
python3 ~/gcs_ws/src/livox_ros_driver2/scripts/takeoff_then_diff_planner.py
```

默认只检查，没有任何起飞或目标发布。脚本会检查：

- `/px4ctrl/state` 是否存在且为1；
- PX4是否连接、未解锁且报告在地面；
- EKF、IMU、RC消息是否新鲜，EKF/IMU时间戳是否合理；
- EKF、IMU接收频率是否达到100Hz；
- 当前静止速度是否不超过0.1m/s；
- 起飞前至少2秒没有收到 `/setpoints_cmd`；
- `/setpoints_cmd` 唯一发布者是否为当前轨迹服务器；
- 是否没有其他 `/goal` 发布者；
- 规划器最大速度是否不超过0.3m/s；
- 规划器高度边界已启用，预期起飞高度与上下边界至少留0.2m余量。

默认节点名固定为本工作空间的 `/px4ctrl`、`/drone_0_diff_planner_node`、`/drone_0_traj_server`。如果使用了命名空间或修改节点名，应先修改脚本常量与检查逻辑，不要盲目删除安全检查。

## 6. 第一次建议：只验证起飞与悬停

仅在地图及场地确认安全后：

```bash
python3 ~/gcs_ws/src/livox_ros_driver2/scripts/takeoff_then_diff_planner.py \
  --execute --map-verified --takeoff-only
```

终端会要求输入 `TAKEOFF`。输入后会再次检查条件，然后只发送一次起飞请求。

此模式确认连续稳定悬停后退出，不发送规划目标；降落和后续接管由飞手负责。

## 7. 完整流程：起飞、稳定悬停、发送短距离目标

应从新一轮安全的地面未解锁状态开始，不要在上一轮已悬停时重复运行。

```bash
python3 ~/gcs_ws/src/livox_ros_driver2/scripts/takeoff_then_diff_planner.py \
  --execute --map-verified \
  --goal-dx 0.5 --goal-dy 0.0
```

输入 `TAKEOFF` 后，脚本将：

```text
一次起飞请求
    ↓
AUTO_TAKEOFF(4)
    ↓
AUTO_HOVER(2)
    ↓
连续3秒位置/速度稳定
    ↓
/goal = 当前悬停位置 + (0.5, 0.0, 0.0)
    ↓
确认新轨迹指令与CMD_CTRL(3)
```

**偏移使用世界/EKF坐标轴，不是机头前方！** 若机头没有对齐world +X，`--goal-dx 0.5` 不一定让飞机沿机头方向前进。

目标Z保持交接时实测悬停高度。起飞相对高度从运行时参数 `/px4ctrl/auto_takeoff_land/takeoff_height` 读取，脚本不修改它。

如果地面EKF z约为0，当前起飞高度0.8m、虚拟天花板1.2m，预期目标高度约为0.8m。脚本不会自动把天花板改成1.2m，应先在launch中设置并重启规划器。

## 8. 稳定悬停判断

脚本同时要求：

- 控制器真实状态为 `AUTO_HOVER(2)`；
- 飞控已解锁且处于 `OFFBOARD`；
- 与起飞点的XY距离不超过0.25m；
- 相对预期起飞高度误差不超过0.12m；
- EKF速度模长不超过0.12m/s；
- 连续满足至少3秒，中断会重新计时。

因此不是简单 `sleep 10` 后盲目发目标。

## 9. 停止脚本不等于停止飞机

**非常重要：**

- `Ctrl-C` 只停止这个顺序脚本；
- 已发送的起飞请求不会因此撤回；
- 已发送的目标不会因此撤回；
- Diff-Planner和轨迹服务器可能继续输出轨迹；
- 脚本成功退出只表示交接已确认，不代表已到点或已降落；
- 超时/报错后，飞手必须观察飞机状态并使用已验证的RC接管流程。

不要在空中通过杀掉MAVROS、EKF、PX4Ctrl或直接停桨来“结束脚本”。需要结束飞行时，使用已经验证的接管与降落流程。

## 10. 测试范围

此脚本进行了离线逻辑测试和ROS消息接口导入检查，**没有替用户发送起飞/目标，也没有进行实机飞行验证**。部署后先运行默认只检查模式；地图异常未解决时，不进入执行模式。
