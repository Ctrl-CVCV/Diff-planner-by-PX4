# Faster-LIO / EKF 高度异常与全套运行负载（2026-11-16）

> 后续新包已复现平地静止点约 0.33 m 的 Z 差，而且 LIO 无持续积压：见 [平地手持新包分析](HEIGHT_BAG_ANALYSIS_20261116.md)。下文是较早一次全负载采样，不能用其超时结论解释所有后续高度问题。

## 1. 结论和观察范围

本次只读检查正在运行的传感器、Faster-LIO、ROS EKF、视觉位姿桥、PX4Ctrl、Diff-Planner、相机和 YOLO；Sunshine、ToDesk、VS Code 同时运行。没有修改参数、停止用户节点、解锁、切模式或发送控制指令。采样期间 FCU 始终 connected=true、armed=false、mode=AUTO.LOITER。

已经确认**全套负载下存在 LIO 实时性故障**：平均接收频率看似约 10 Hz，但旧帧追赶期间里程计消息年龄 P95 达到 1.283 s、最大 1.883 s，并导致 ROS EKF 的观测时间落在历史缓冲范围之外。

**尚未复现用户描述的“基本同高的两个位置，Z 相差 0.8 m”移动过程，因此不能据此认定唯一根因。** 本次位置近似静止，LIO、ROS EKF、PX4 三层平均 Z 均约 1.80 m，没有观测到只在 PX4 层新增的 0.8 m 偏差。静止波动小不能证明移动到另一位置的高度估计准确，当前结果不能作为飞行验收。

## 2. 实际配置与链路

本次实际运行参数，而非历史测试的临时覆盖：

```yaml
filter_size_surf: 0.05
filter_size_map: 0.15
point_filter_num: 4
mapping/max_threads: 3
ivox_grid_resolution: 0.5
ivox_max_points_per_grid: 64
common/time_offset_lidar_to_imu: -0.07625571
```

每格 64 点为本地已修改的“格满后保留早期点”实现，不是上游无限点数。此前只有传感器和 LIO 的精细档测试用了抽点 3、自动线程，不能直接移用其性能结论。

实时检查到的链路为：

```text
/laserMapping -> /Odometry (camera_init)
             -> /ekf -> /ekf/ekf_odom (world)
                    -> /ekf_vision_bridge -> /mavros/vision_pose/pose
                                        -> PX4 EKF2 -> /mavros/local_position/odom (map)
```

这三个入口各只有一个预期发布者。当前 `imu_body_R`、`rotation_imu` 和 `Rr_i` 为单位旋转，相关平移与 EKF 的 `offset_pz` 为零。ROS EKF 输出函数直接写 `frame_id=world`，位置没有执行 camera_init 到 world 的额外旋转；视觉桥复制位姿，不额外转轴。**这里只能确认代码和发布关系，不能仅凭 frame 名称或当前静止数据证明世界系已经严格水平对齐。**

只读读取的 PX4 当前值：`EKF2_EV_CTRL=11`、`EKF2_HGT_REF=3`、`EKF2_BARO_CTRL=1`、`EKF2_EV_DELAY=0`、`EKF2_EV_POS_X/Y/Z=0`。不能凭同时启用气压计就认定其造成高度异常；本次也没有据此关闭任何融合源。

本地 PX4Ctrl 配置的用户改动为 `mass=3.20`、`hover_percentage=0.440`。这些控制参数不是 ROS EKF 的位置观测或坐标转换参数；未解锁的本次观察不能验证它们在飞行中的控制效果。

## 3. 同步采样结果

2026-11-16 22:20 左右（北京时间）进行了 60 次一秒间隔 CPU 采样；计入读取开销，位姿实际覆盖约 65.4 秒。

| 数据源 | 接收率 | Z 均值 | Z 最大值减最小值 |
|---|---:|---:|---:|
| `/Odometry` | 10.10 Hz | 1.8057 m | 0.0394 m |
| `/ekf/ekf_odom` | 196.86 Hz | 1.8076 m | 0.0240 m |
| `/mavros/local_position/odom` | 30.05 Hz | 1.8081 m | 0.0296 m |

以上是当前各自坐标原点下的数值，不是对地高度；各层接收时刻、滤波和预测不同，均值接近不能替代运动中的时间对齐核查。

`/Odometry` 消息年龄为收到消息时的 ROS 时间减 header.stamp：中位数 91.4 ms、P95 1282.6 ms、最大 1883.3 ms。采样开始前已有少量旧帧，期间发生追赶，所以接收均值略高于 10 Hz **不表示计算能力超过输入，也不表示实时性通过**。性能日志采样到 `pending_lidar=13`、`pending_imu=286`、`scan_end_age_s=1.50502`。

同期 ROS EKF 报出 126 次：

```text
odom time out of buffer time range: ...ms, ekf may be wrong !!!!
```

实现中 `seqsize=100`；在约 197 Hz IMU 下只能覆盖约 0.5 秒历史。超过这一范围的 LIO 观测无法按预期匹配历史状态。实际日志证明该问题发生了，而不是仅凭一个脚本的超时阈值推测。当前静止不一定表现为大漂移，但运动中必须排查其影响；不要通过取消时间检查、改成当前时间戳或强行扩大缓冲来掩盖上游积压。

IMU / ROS EKF 时间戳本次部分超前主机约几十毫秒，时间同步也应纳入后续运动数据核查；本轮没有修改 `time_offset_lidar_to_imu` 或 PX4 视觉延迟。

### CPU / GPU

8 核、25 W 模式。同步多话题采样期间 psutil 的全机 CPU 平均约 89.9%，单次全机平均最高 98.4%；采样器本身也有开销。

停止多话题采样后，以轻量 tegrastats 独立复测 30 秒：**全机 CPU 平均 78.8%、P95 81.8%，GPU busy 平均 95.2%**。该测量不能与前一阶段的差额全部归因于采样器；后台任务负载也可能变化。CPU 约 54℃，各核报告 1497 MHz，内存使用约 58%，无 swap 使用；没有观察到明显高温，但本轮没有读取完整降频原因，不能断言绝无功率限制。

同步采样阶段主要进程 CPU 均值如下。此处 **100% 表示一个 CPU 核，8 核总量为 800%**，不是全机使用率：

| 进程 | CPU 均值 |
|---|---:|
| Faster-LIO | 130.3% |
| VS Code 进程合计，不含 cpptools | 103.0% |
| Sunshine | 94.9% |
| Xorg | 48.2% |
| YOLO | 46.3% |
| cpptools | 38.0% |
| 相机发布 | 36.8% |

还发现一个持续运行的 `rostopic echo /ekf/ekf_odom/pose`，CPU 约 9.2%。这在约 197 Hz 刷新终端；不需要观察时应由操作者 Ctrl+C 停止，避免不必要的序列化和桌面刷新。本轮没有替用户终止它。

## 4. 下一步排查顺序

1. 动力故障支路保持隔离，不以这份静止检查作为重新上桨或起飞的依据。
2. 先停止不必要的高频 `rostopic echo`、YOLO 预览窗口、VS Code 索引等；需要保留远程控制时不要贸然停止唯一远程连接。单次看坐标可用 `rostopic echo -n 1 /ekf/ekf_odom/pose/pose/position`。
3. 保留 `0.05/0.15`，在相同场景、相同全套负载下比较线程配置和匹配耗时，而不是直接认为自动线程一定适合全套负载；必要时再逐步加粗 surf 参数。改参数、编译或重启都只能在地面、任务停止后进行，本轮未执行。
4. 优先消除持续队列和超出 EKF 历史窗口的观测。可将 LIO 消息年龄 P95 小于 0.10 s、无持续积压、无上述缓冲错误作为工程初筛目标，但不是官方安全保证，更不能代替运动精度验收。
5. 在动力隔离的条件下手持完成“同高 P1 → 同高 P2 → 返回 P1”，记录两点水平距离、物理高度、机身姿态及停留时间。保持同一轮定位进程，不在两个端点之间重启。需要同时比较三层 Z，而不是只看 QGC 或终端里的一个数。

建议先降低无关桌面负载，再记录这段运动。以下命令只录制，不起飞；数据可能含几十至数百 MB 的原始雷达输入，确保空间足够，移动前录几秒、每个端点停留几秒，结束用 Ctrl+C：

```bash
source ~/gcs_ws/devel/setup.bash
CASE_DIR="$HOME/gcs_ws/log/z_height_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$CASE_DIR"
rosparam dump "$CASE_DIR/runtime_all.yaml"
rosbag record -b 256 -O "$CASE_DIR/height.bag" \
  /livox/lidar /mavros/imu/data_raw /mavros/imu/data \
  /Odometry /ekf/ekf_odom /mavros/vision_pose/pose \
  /mavros/local_position/odom /mavros/state /mavros/estimator_status \
  /mavros/timesync_status /tf /tf_static /rosout_agg
```

判别方向：LIO 本身也产生同等 Z 差，优先查 LIO 重力对齐、外参与同步、场景约束和局部地图；LIO 正常而 ROS EKF 异常，查历史匹配、IMU 预测及转换；前两层正常而只有 PX4 异常，再查 PX4 高度融合、原点重置和创新量。这里只是定位故障层级，不是根因已确认。

## 5. 本次原始证据

- `/tmp/live_height_cpu_20261116/samples.json`：位姿、IMU、状态、少量点云、日志和 CPU 样本。
- `/tmp/live_height_cpu_20261116/params.json`：ROS 参数快照与 PX4 只读 ParamGet 返回值。
- `/tmp/live_height_cpu_20261116/path_snapshot.json`：约 99.9 秒的 LIO 历史轨迹，仍主要为静止段，不包含完整的两点复现。
- `/tmp/live_height_cpu_20261116/tegrastats.txt`、`tegrastats_after_probe.txt`：采样期间与停采后的系统负载。
- `/tmp/check_live_height_cpu_20261116.py`：本次只读观察脚本。
- `~/gcs_ws/log/fastlio2_ekf_20261116_220752/`：用户当前启动会话中的 LIO、EKF、MAVROS 等日志。

`/tmp` 下材料可能在清理或重启后消失，需要保留时复制到自己的测试目录；不要把它们当作已经录制了完整移动 rosbag。
