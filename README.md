# gcs_ws：PX4Ctrl + Diff-Planner

本工作区实机操作说明统一维护在 [docs/README.md](docs/README.md)。
优先阅读 [混合任务全流程：EKF → PX4Ctrl → Planner → 录包 → 起飞执行 → 隔离回放](docs/HYBRID_MISSION_FLIGHT_AND_BAG_RUNBOOK.md)。

- 坐标与占据异常先按文档做地面检查；不以脚本通过代替安全验证。
- 起飞脚本默认只检查，执行需人工确认；新增航点后3秒无位移降落保护需要新版PX4Ctrl。
- rosbag离线回放必须隔离ROS master，不连接实机控制链。
- 编译产物、传感器日志、bag和本地凭据不入库，详见 [Git说明](docs/WORKSPACE_GIT.md)。
