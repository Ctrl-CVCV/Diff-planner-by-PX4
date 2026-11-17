# Faster-LIO 全负载低帧率排查与实时参数

更新：2026-11-16。工作区为 `~/gcs_ws`。已完成在线传感器测试和同包对比，复现了地图点数膨胀引起的长期掉帧；**仍未证明在 PX4Ctrl + Diff-Planner + YOLO + Sunshine 全开时能稳定达到 10 Hz**。实测结果与原始数据见 [在线测试记录](FASTER_LIO_LIVE_TEST_20261116.md)。

后续精细档复测：采用已限额实现，`surf/map=0.05/0.15, point_filter_num=3, TBB自动` 在仅传感器 + LIO 的五分钟真实输入测试中达到平均 9.9997 Hz、消息年龄 P95 39.61 ms；见 [精细参数测试](FASTER_LIO_PRECISION_TEST_20261116.md)。随后已按用户要求将正式配置改为 `0.05/0.15`，抽点 4、线程 3 不变。夜间全套负载实测又发现 LIO 延迟与 EKF 缓冲越界，见 [高度与全套负载核查](FASTER_LIO_Z_CPU_20261116.md)；下文 0.30/0.50 为较早的起始档记录，不是当前值，完整任务栈仍未通过实时性验收。

## 1. 已找到的证据

实际起飞启动链 `src/livox_ros_driver2/scripts/start_fastlio2_ekf.sh` 默认使用 `fasterlio`，加载：

```text
~/gcs_ws/src/Diff-Planner/src/realflight_modules/faster-lio/faster-lio/config/mid360_px4.yaml
```

它将里程计和点云映射回 `/Odometry`、`/cloud_registered`、`/cloud_registered_body`、`/path`，并且默认不开 LIO 的 RViz。已有 `/Odometry` 时启动脚本会复用，**改完配置但没有在地面停掉旧 LIO 进程，参数可能根本没生效**。另一个 `~/fasterlio_ws` 也有旧二进制，不要混用 overlay。

### 1.1 与旧 FAST-LIO2 的有效参数对比

旧 FAST-LIO2 的 `config/mid360_px4.yaml` 并没有写全部滤波参数；`launch/mapping_mid360_px4.launch` 在加载 YAML 后将滤波尺寸设为 **0.5/0.5**。这不是相同精度下的算法速度对比。

| 参数 | 旧 FAST-LIO2 实际 launch | Faster-LIO 高精度日志 | 修改前 Faster-LIO | 本次实时起始档 |
|---|---|---|---|---|
| `filter_size_surf` | 0.50 m | 0.05 m | 0.15 m | **0.30 m** |
| `filter_size_map` | 0.50 m | 0.15 m | 0.30 m | **0.50 m** |
| `point_filter_num` | 3 | 3 | 3 | **4** |
| `max_iteration` | 3 | 3 | 3 | **3，保留** |
| `dense_publish_en` | true | false | false | **false，保留** |
| `pcd_save_en` | true，100 帧一存 | false | false | **false，保留** |

`0.05 → 0.15` 的“点数减少约 9 倍”只是稠密平面场景的面积比例估计，不是这段代码的固定点数或耗时比例。雷达采样、遮挡、地形和取点规则都会影响实际值。

旧 FAST-LIO2 在本机 ARM 编译分支为 `MP_PROC_NUM=1`；Faster-LIO 的匹配代码使用 `std::execution::par_unseq` 和 TBB。全负载下并行调度和共享资源竞争是需验证的因素，不能只凭名称认定新算法一定更快，也不能仅凭日志断言 Sunshine 或 YOLO 是唯一原因。

### 1.2 历史日志定位到匹配/IEKF，而非主要卡在体素滤波

日志目录：`~/gcs_ws/log/fastlio2_ekf_<日期时间>/faster_lio.log`。以下是退出时打印的累计平均值，不是当前瞬时耗时：

| 会话目录后缀 | surf/map | IEKF 平均耗时 | 每次 Lidar Match 平均耗时 |
|---|---|---|---|
| `20261114_215529` | 0.05 / 0.15 | 180.662 ms | 55.3464 ms |
| `20261114_221826` | 0.05 / 0.15 | 202.802 ms | 62.3753 ms |
| `20261114_224352` | 0.15 / 0.30 | 26.7491 ms | 6.65763 ms |
| `20261114_225850` | 0.15 / 0.30 | 142.499 ms | 40.4975 ms |

最后一次会话体素降采样仅约 0.484 ms，预处理约 1.165 ms，去畸变约 1.614 ms；同一帧可能多次调用匹配，不能将 Lidar Match 和 IEKF 总时间再次相加。142.5 ms 的 IEKF 已超过 10 Hz 的 100 ms 帧预算，还没计入所有外围开销。

最后一次预处理调用 29261 次，IEKF 更新仅 17684 次，结合单帧耗时，说明存在明显处理跟不上的迹象；不能把调用次数差全部视作丢帧（初始化、同步等也影响次数）。同样配置的两次会话差异很大，需要继续区分外部负载、地图随时间变密、温度/功耗和调度影响。

### 1.3 另外发现的开销与构建问题

- `/path` 原来每帧追加并发布从启动至今的完整轨迹，大小随运行时长增长。本次保留路径话题，但配置为最多 1000 个位姿；10 Hz 时约最近 100 秒。bag 中的完整里程计仍可用于重建全程轨迹。
- 原生成的 Faster-LIO 编译标志含 `-O3`，**不是没有优化的 Debug 构建**；但包内覆盖了 `CMAKE_CXX_FLAGS_RELEASE`，丢掉标准 Release 的 `-DNDEBUG`。本次移除覆盖，恢复 CMake 的标准构建类型标志。不要把 `-g` 调试符号本身当作主要掉帧证据。
- iVox 默认节点内部存储点列表，查询需要遍历候选格内点。**增大 `ivox_grid_resolution` 并不保证更快**，可能让每格候选点更多。本次保持 0.5 和 18 邻域，不盲目扩大网格或减小匹配邻域。
- 本版 `cube_side_length` / `mapping/det_range` 在已检查的 mapping 源码中只是加载，未参与后续局部地图裁剪；不能靠缩小它们来承诺提速。

## 2. 本次变更与风险

唯一飞行调参文件仍是上面的 `mid360_px4.yaml`。新增项为：

```yaml
mapping:
    max_threads: 3
publish:
    path_max_poses: 1000
performance_log_enable: true
ivox_max_points_per_grid: 64
```

这是原 YAML 对应小节中的字段，**不要用这段片段覆盖完整标定文件**。`max_threads` 是本次新增的在线进程 TBB 并行度限制，不是 CPU 核绑定；0 表示恢复 TBB 自动选择。1000 是路径消息的显示窗口，不是 LIO 地图容量限制。`path_max_poses: 0` 恢复不限制；如开启轨迹文件保存，保存的路径也受这个窗口影响。

`ivox_max_points_per_grid` 是根级字段，与 `mapping` / `publish` 同级。默认 iVox 点列表节点每格最多保留 64 个早期进入的点，达到限额后不再向该格追加，限制反复扫描同一区域时的候选点数量；不增加候选格、不修改标定和时间戳，也不直接改变发布给 Planner 的扫描点云密度。未设置或设为 0 保留原先不限制的行为；负值拒绝初始化。这个上限只用于默认点列表后端，PHC 后端仍沿用原算法。

这是定位地图的保留策略变化，不是无代价的纯加速开关；已满的格不能接纳之后看到的新表面，可能影响新视角和动态场景。曾试验满格循环替换，但同包高度变化增大，已放弃该实现。64 是地面起始值，尚需手持运动、退化场景、闭环回到旧区域和全负载验收。它只限制每格，不是整个地图的点数上限。不要为了增密障碍图去无限增大这个值。

在线长测已证明仅改成 surf=0.30/map=0.50 仍不够：未限制时地图点数由约 3984 增至 980520，5 分钟末段约 5.46 Hz、里程计延迟约 47 秒。40 秒同包测试也显示，旧密度下将 TBB 从自动改成 3 线程反而更慢；3 不是普适最优值，不能将限线程本身当作本次故障的修复。

最终保留早期点版本已编译并完成 300 秒在线采样：平均 10.000 Hz，最后 10 秒 10.009 Hz，消息年龄 P95 31.2 ms，末次地图 86649 点、LiDAR 内部积压 0。该次 Z 输出范围为 5.40 cm，没有外部真值；这是仅传感器 + LIO 的地面测试，不能替代完整任务栈、手持及飞行精度验收。原始材料见上方在线测试记录。

外参、IMU 噪声、时间偏移、坐标变换、10 Hz 雷达发布频率、Planner 膨胀参数和飞行安全检查均未因本次性能调整修改。不通过伪造时间戳、强行输出重复位姿或清空 IMU 缓冲来冒充 10 Hz。

**精度和障碍地图需要重新验收：** `dense_publish_en: false` 时 Faster-LIO 世界点云来自降采样扫描，增大 surf / 取点间隔也会降低 Planner 可见的障碍点密度。旧 FAST-LIO2 `dense_publish_en: true` 则输出更密的点云；即使把两者 surf 都设为 0.5，障碍图输入密度也未必相同。细杆、边缘、小障碍和定位约束较少的通道必须在 RViz 中重新检查；不要只看帧率决定可以飞。

## 3. 地面编译、重启和核对实际生效值

所有重启、编译和调参均在**落地、锁桨、拆桨或可靠隔离动力**状态完成，不在空中操作。不发起飞命令，不执行任务脚本。确认旧定位/控制进程安全停止后：

```bash
source /opt/ros/noetic/setup.bash
source ~/gcs_ws/devel/setup.bash
rospack find faster_lio
cmake --build "$HOME/gcs_ws/build" --target run_mapping_online -- -j2
```

`rospack find` 应指向 `gcs_ws` 下的 Faster-LIO，而不是 `~/fasterlio_ws`。构建失败不要继续。用现有启动入口重启定位（它不主动解锁/起飞）：

```bash
bash ~/gcs_ws/src/livox_ros_driver2/scripts/start_fastlio2_ekf.sh --lio fasterlio
```

新终端核对（参数在根命名空间，不是 `/laserMapping/filter_size_surf`）：

```bash
source ~/gcs_ws/devel/setup.bash
rosparam get /filter_size_surf          # 0.3
rosparam get /filter_size_map           # 0.5
rosparam get /point_filter_num          # 4
rosparam get /max_iteration             # 3
rosparam get /mapping/max_threads       # 3
rosparam get /ivox_max_points_per_grid  # 64
rosparam get /publish/path_max_poses    # 1000
rosnode info /laserMapping
rostopic info /Odometry
```

启动日志还应出现 `Faster-LIO TBB max_threads=3`。里程计应只有预期 LIO 发布者；不能同时运行两套 LIO 后把同名输出混到一起。参数服务器显示新值不代表旧进程已经重新读取，修改后必须重启；确认启动脚本没有“复用现有 /Odometry”。

## 4. 全负载地面验收：不仅看 Hz，还看延迟与积压

先仅开定位，然后在不允许自动解锁的条件下逐项增加原有 EKF、Planner、控制节点、YOLO、RViz、Sunshine；每项记录至少 60 秒，最终组合持续 5～10 分钟。不要在飞行或录包中途关闭远程连接导致失去操作通道。首次对比不要同时启动两套 LIO，也不要一边编译一边测速。

在不同终端分别运行，观察 `/Odometry` 而不是高频 EKF 输出来判断 LIO 帧率：

```bash
timeout 30s rostopic hz -w 100 /livox/lidar
timeout 30s rostopic hz -w 100 /Odometry
timeout 30s rostopic delay -w 100 /Odometry
```

`timeout` 到时返回 124 是正常结束。雷达输入应约 10 Hz；LIO 应接近输入频率，且时间延迟不持续增长。扫描结束时刻与系统时钟可能有固定偏差，不能简单要求 delay=0，也不要为把 delay 变小修改标定偏移。要关注是否从小延迟一路累积成秒级、分钟级。

新 LIO 日志每约 5 秒输出一条 `[faster_lio_perf]`：

- `last_frame_ms`：最近这一个成功处理帧从同步完成至发布结束的墙钟耗时；不含之前 `spinOnce` 的回调成本，也不是平均值或 P95。全负载尽量留出余量，不能长期接近/超过 100 ms。
- `input_points` / `downsampled_points` / `effective_points`：去畸变后输入点数、参与扫描匹配的降采样点数、有效约束点数。极少有效点或连续 `No Effective Points` 时不能仅因帧率改善就继续降精度。
- `map_grids` / `map_points`：地图格数和实际保留点数；点数每约 5 秒统计一次。配置上限 64 时，默认后端应满足 `map_points <= 64 * map_grids`。不能只看格数判断地图大小。
- `pending_lidar` / `pending_imu`：内部待处理数量，不包括 ROS 传输队列。持续增长表明可能跟不上或同步有问题。
- `scan_end_age_s`：日志时系统 ROS 时间减扫描结束时间，用于看积压趋势；不是发布频率，也不做新的飞行控制判断。

可查看最新一轮的现有日志（目录以启动终端实际输出为准）：

```bash
LOG_DIR="$(ls -dt "$HOME"/gcs_ws/log/fastlio2_ekf_* | head -n 1)"
tail -f "$LOG_DIR/faster_lio.log"
```

正常地面结束进程后，日志中的 `average time usage` 会给出 IEKF / Lidar Match 等累计均值。不要把退出时的均值当成某次负载改变之后的独立均值；比较不同配置应分开新会话。

同时记录 `top -H` 和可用的 `tegrastats` 输出，检查 CPU 单核占用、GPU/内存带宽、温度、频率及内存压力；读取功耗模式可以用 `sudo nvpmodel -q`。这些是排查项，当前日志不足以证明存在降频，不应盲目切最高功耗模式。YOLO 当前 launch 的 `infer_threads` 默认 3；可在独立对比中试 1～2，降低显示输出频率、减少重复 RViz 窗口，避免以牺牲控制/定位新鲜度换视觉显示帧率。

## 5. 不够快时怎样继续减弱，怎样回退

每次只改一项，在地面重启并重复上述测试：

1. 先确认已编译新二进制、`ivox_max_points_per_grid=64` 实际生效，再测试本次起始档 `surf=0.30, map=0.50, point_filter_num=4, max_iteration=3`；仅减小扫描点数不能根治地图点列表的长期膨胀。
2. 仍不足 10 Hz 时，先将 `filter_size_surf` 增至 **0.50**，与旧 FAST-LIO2 的匹配降采样尺寸对齐；`filter_size_map` 保持 0.50。重新检查小障碍可见性和静止/手持定位。
3. 同档配置分别测 `mapping/max_threads=2、3、4、0`，找全负载延迟最小且无积压的值；3 只是起始值，不保证对所有场景最优。
4. 还不够时才试 `point_filter_num=5`；`max_iteration=2` 属于进一步牺牲配准收敛余量的实验档，不作为本次默认。不要同时减小邻域、放宽拟合阈值或改外参，避免无法判断误差来自哪里。
5. 若相同 0.50/0.50 档仍明显慢于 FAST-LIO2，应先排查 TBB 调度、长期地图增长和全机负载，再继续牺牲精度。需要退回旧实现时，仅在地面停止当前整套启动链及依赖节点后，用原入口 `--lio fastlio` 另开会话；旧定位实现同样需要重新验证，不在空中热切换。

恢复修改前性能行为可设 `surf=0.15, map=0.30, point_filter_num=3, ivox_max_points_per_grid=0, mapping/max_threads=0, publish/path_max_poses=0, performance_log_enable=false`，但这不是稳定 10 Hz 的保证，且会重新放开已复现的地图点数膨胀。仅改 YAML 不会回退 C++ 二进制；这些开关在新二进制中兼容旧配置。

验证地图请继续参考 `OBSTACLE_MAP_CHECK.md` 以及 `HYBRID_MISSION_FLIGHT_AND_BAG_RUNBOOK.md` 的实时 RViz 章节。性能验收和地图验收都通过之后，再考虑进入原有飞行流程。
