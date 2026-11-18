# gcs_ws 实机文档总入口

更新：2026-11-17。工作区指 `/home/nvidia/gcs_ws`（即 `~/gcs_ws`），不是另建根路径 `/gcs_ws`。
本文档目录是本机实机操作说明的唯一正文来源，原包内 docs 文件仅作迁移入口。
第三方包的上游 README/CHANGELOG、许可证和代码示例不搬动，避免破坏上游结构。

Faster-LIO 当前选定的无 IVox 上限版本、实机配置与启动入口统一放回 `gcs_ws`，降采样为 0.20/0.40。启动与地面验证统一看下方当前指南，不再套用历史调参文档。第三方源码仍保留包结构，不平铺代码文件。

## 当前启动与飞行

| 目的 | 文档 |
|---|---|
| 当前 Faster-LIO：统一源码、0.20/0.40 参数、地面编译、EKF/视觉桥、点云里程计 RViz、录包回放 | [Faster-LIO 当前指南](FASTER_LIO_GUIDE.md) |
| 只要命令清单：启动 → 看 RViz → 录 bag → 预检 → 执行 → 停录制 | [混合任务最简步骤](HYBRID_MISSION_QUICKSTART.md) |
| 所有为保守飞行改过的参数：当前值、原值、文件:行、含义 | [保守飞行参数总表](CONSERVATIVE_FLIGHT_PARAMS.md) |
| 当前混合任务完整操作：EKF → PX4Ctrl → 专用 Planner → 先录包 → 程控起飞 → B/C/D → 降落与隔离回放 | [混合任务起飞与录包全流程](HYBRID_MISSION_FLIGHT_AND_BAG_RUNBOOK.md) |
| 直接复制 RViz 实时看障碍地图的命令，RAW/膨胀图切换、无显示排查 | [混合任务全流程第 7 节](HYBRID_MISSION_FLIGHT_AND_BAG_RUNBOOK.md#7-终端-5rviz-看实时地图和地面诊断) |
| 比赛分段任务：程控起飞、Planner 到 B、纯 PX4Ctrl 到 C、切回 Planner 到 D、悬停 3 秒后降落 | [Planner/直控切换任务](HYBRID_PLANNER_PX4CTRL_MISSION.md) |
| 救援物资"识别后降高度、不投放"测试，以及航点地面高度误差 z_offset 的测量与填写 | [救援无投放测试与高度补偿](RESCUE_DRY_RUN_AND_Z_OFFSET_TEST.md) |
| 实时 RViz 看当前 RAW/膨胀地图、地面检查、分辨率、限制越障高度 | [地图检查（实时看图见第 3.1 节）](OBSTACLE_MAP_CHECK.md) |

## 标定与硬件

| 目的 | 文档 |
|---|---|
| 摄像头请求 120 FPS、持续观看与可见高度测试、10 秒 YOLO 图片采集 | [视觉测试与图片采集](CAMERA_120FPS_YOLO_COLLECTION.md) |
| AUX6 舵机（推料机构）：飞控参数 → 单测 → dry_run → 未解锁核对 → 拆桨解锁实测 → 带料标定 | [AUX6 舵机测试全流程](PX4_SERVO_AUX6_TEST_RUNBOOK.md) |
| EKF → MAVROS → PX4 视觉位姿桥：把 `/ekf/ekf_odom` 送进 PX4（含 PX4 参数与验证步骤） | [PX4 视觉位姿桥](PX4_VISION_POSE_BRIDGE.md) |
| 外参与时间偏移标定 | [LiDAR/IMU标定](LIDAR_IMU_EXTRINSIC_CALIBRATION.md) |

## 历史分析与版本管理

以下是特定日期的证据，不作为当前参数表或启动入口。已删除的 Faster-LIO 阶段性排查文档可从 Git 检查点 `1dbfede` 恢复，不再与现行指南并列。

- [2026-10-20 坐标链路核查](FRAME_TRANSFORM_AUDIT_20261020.md)
- [2026-10-21 带动力定位故障排查](POWERED_TAKEOFF_ODOMETRY_20261021.md)
- [2026-10-21 LIO 中断后 EKF 上漂分析](FLIGHT_ANALYSIS_20261021_210844.md)
- [2026-10-20 航点后仍悬停分析](FLIGHT_ANALYSIS_20261020_2145.md)
- [2026-11-17 当前版本地面测试检查点](checkpoints/faster_lio_20261117/README.md)
- [工作区 Git 管理、编译和日志排除](WORKSPACE_GIT.md)

## 历史部署资料（不是当前默认参数或替代操作清单）

- [FAST-LIO2/EKF/PX4Ctrl部署和定点起飞](FAST_LIO2_EKF_PX4CTRL_TAKEOFF.md)
- [Diff-Planner接入与障碍地图调参](DIFF_PLANNER_INTEGRATION_AND_OBSTACLE_MAP_TUNING.md)
- [PX4与MID360s接线/同步/部署](PX4_MID360S_FASTLIO.md)

当前参数以本次运行节点、连接关系和录包时 `runtime_all.yaml` 为准；参数服务器可能保留已退出节点的值。
历史包缺少的地图无法靠RViz恢复，不要以过往文档的膨胀值代替当时运行参数。
地图和坐标异常未排除前先做地面检查，自动降落保护不能代替飞手或证明可以安全起飞。
