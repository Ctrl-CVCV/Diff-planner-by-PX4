# gcs_ws 实机文档总入口

更新：2026-10-27。工作区指 `/home/nvidia/gcs_ws`（即 `~/gcs_ws`），不是另建根路径 `/gcs_ws`。
本文档目录是本机实机操作说明的唯一正文来源，原包内 docs 文件仅作迁移入口。
第三方包的上游 README/CHANGELOG、许可证和代码示例不搬动，避免破坏上游结构。

## 推荐阅读顺序

| 目的 | 文档 |
|---|---|
| 当前混合任务完整操作：EKF → PX4Ctrl → 专用 Planner → 先录包 → 程控起飞 → B/C/D → 降落与隔离回放 | [混合任务起飞与录包全流程](HYBRID_MISSION_FLIGHT_AND_BAG_RUNBOOK.md) |
| 直接复制 RViz 实时看障碍地图的命令，RAW/膨胀图切换、无显示排查 | [混合任务全流程第 7 节](HYBRID_MISSION_FLIGHT_AND_BAG_RUNBOOK.md#7-终端-5rviz-看实时地图和地面诊断) |
| 摄像头请求 120 FPS、持续观看与可见高度测试、10 秒 YOLO 图片采集 | [视觉测试与图片采集](CAMERA_120FPS_YOLO_COLLECTION.md) |
| 旧单段航点脚本的启动、录包和隔离 RViz 回放；不是混合任务启动清单 | [单段任务录包回放手册](ROSBAG_RECORD_REPLAY.md) |
| 比赛分段任务：程控起飞、Planner 到 B、纯 PX4Ctrl 到 C、切回 Planner 到 D、悬停 3 秒后降落 | [Planner/直控切换任务](HYBRID_PLANNER_PX4CTRL_MISSION.md) |
| 脚本参数、遥控状态要求、3秒无位移降落保护与限制 | [顺序脚本](TAKEOFF_THEN_DIFF_PLANNER.md) |
| 实时 RViz 看当前 RAW/膨胀地图、地面检查、分辨率、限制越障高度 | [地图检查（实时看图见第 3.1 节）](OBSTACLE_MAP_CHECK.md) |
| LiDAR→IMU→LIO→EKF→地图→目标→控制链证据与未决风险 | [坐标链路核查](FRAME_TRANSFORM_AUDIT_20261020.md) |
| 2026-10-21 起飞发散、手持正常：三次雷达状态序号回退与 LIO 断更 | [带动力定位故障排查（最新）](POWERED_TAKEOFF_ODOMETRY_20261021.md) |
| 2026-10-21 新包：LIO 中断后 EKF 上漂至 10.46 米 | [本次里程计漂移分析](FLIGHT_ANALYSIS_20261021_210844.md) |
| 2026-10-20 21:45航点后仍悬停的历史bag分析 | [历史飞行分析](FLIGHT_ANALYSIS_20261020_2145.md) |
| 外参与时间偏移标定 | [LiDAR/IMU标定](LIDAR_IMU_EXTRINSIC_CALIBRATION.md) |
| Git管理、编译/日志文件排除 | [工作区Git说明](WORKSPACE_GIT.md) |

## 历史部署资料（不是当前默认参数或替代操作清单）

- [FAST-LIO2/EKF/PX4Ctrl部署和定点起飞](FAST_LIO2_EKF_PX4CTRL_TAKEOFF.md)
- [Diff-Planner接入与障碍地图调参](DIFF_PLANNER_INTEGRATION_AND_OBSTACLE_MAP_TUNING.md)
- [PX4与MID360s接线/同步/部署](PX4_MID360S_FASTLIO.md)

当前参数以本次运行节点、连接关系和录包时 `runtime_all.yaml` 为准；参数服务器可能保留已退出节点的值。
历史包缺少的地图无法靠RViz恢复，不要以过往文档的膨胀值代替当时运行参数。
地图和坐标异常未排除前先做地面检查，自动降落保护不能代替飞手或证明可以安全起飞。
