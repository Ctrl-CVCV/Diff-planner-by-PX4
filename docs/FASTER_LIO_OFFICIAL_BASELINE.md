# Faster-LIO 官方基线与标定外参

官方定位稳定后，追加外部 EKF、可选 MAVROS 视觉桥和三路 Z 数值对比的方法见 [EKF 桥接地面测试](FASTER_LIO_EKF_BRIDGE_TEST.md)。新入口不改官方算法，也不依赖旧一键脚本。

## 恢复范围

本次从现有 `origin` 拉取并核实 `DifferentialRobotics/faster-lio` 的官方 `main`，版本为 `c8d635c4689f5ebd89a9532411254c93eaa35adc`。这是设备配套版，包含其原有重力对齐和 MID360 适配，不是切换到 `gaoxiang12/faster-lio` 作者原版。

仅恢复 `realflight_modules/faster-lio/faster-lio`。算法源码、头文件、IVox、入口程序和构建文件与该官方提交一致；撤销本地时间补偿、每格点数限制、线程限制、性能日志和源码默认关闭外参估计等补丁。不改变 EKF、雷达驱动、PX4Ctrl、Planner、任务脚本和视觉桥；不切换分支、改写 HEAD、提交或推送。

唯一配置差异在 `faster-lio/config/mid360.yaml`：IMU 使用标定对应的 `/mavros/imu/data_raw`，并保留此前标定：

```yaml
extrinsic_T: [0.032600, 0.027755, 0.086446]
extrinsic_R: [0.856373, -0.003504, 0.516346,
              0.007575, 0.999955, -0.005777,
              -0.516302, 0.008859, 0.856361]
imu_body_T: [0.0, 0.0, 0.0]
imu_body_R: [1, 0, 0, 0, 1, 0, 0, 0, 1]
```

其余使用官方 MID360 配置：滤波尺寸 `0.5/0.5`，`point_filter_num: 3`。此配置原本就有 `mapping/extrinsic_est_en: false`；源码缺省恢复官方的 `true`，因此必须加载本配置，不能以源码缺省已关闭为前提启动。

## 备份与编译

恢复前的完整源码、补丁、配置、状态和二进制备份在 `~/gcs_ws/log/faster_lio_before_official_restore_gbJ6Pmz3/`。`source_before.tar.gz` 是完整源码备份，`calibration_before.yaml` 保存原配置，`build_official.log` 是本次编译日志。恢复备份会覆盖文件，应先保存后续新改动。

按官方 README 的 `catkin_make` 流程重新配置并构建，仅构建本包、限制编译内存占用：

```bash
source /opt/ros/noetic/setup.bash
source ~/gcs_ws/devel/setup.bash
catkin_make -C ~/gcs_ws --pkg faster_lio --force-cmake \
  -DCMAKE_BUILD_TYPE=Release -j1 -l2
```

编译不是飞行验证，也不证明 Z 漂移已修复。

## 启动接口变化

自定义的 `mid360_px4.yaml` 和官方 launch 中原先加入的 `config_file`、`start_lidar`、`mapping_node_name`、`odom_topic` 等参数已撤销。旧 `start_fastlio2_ekf.sh` 的 Faster-LIO 分支仍传这些参数，本次没有修改该脚本，暂不能照旧使用。

官方 `mapping_mid360.launch` 会自行启动驱动，并要求环境变量 `BD_LIST`，不能与现有雷达驱动重复启动。为单独核验官方定位，下面直接运行官方可执行程序，用 ROS 参数和 remap 保持原点云/里程计显示接口，不改官方算法。

## 地面定位测试

保持未解锁，不运行任务脚本、PX4Ctrl 自动起飞、Planner 或视觉位姿桥。保留 MAVROS 提供飞控 IMU，停止旧 FAST-LIO2/Faster-LIO 定位实例，避免同名里程计或 TF 多源发布。不要在飞行时停止节点。

每个终端先 source 上面的 ROS 和工作区环境。ROS master 未运行时另开 `roscore`；雷达驱动未运行时才启动 `roslaunch livox_ros_driver2 msg_MID360.launch`。保留现有且端口已核对的 MAVROS，不另开第二个 MAVROS。确认 `/livox/lidar` 和 `/mavros/imu/data_raw` 均有数据。

需要观察初始化时，在另一个终端先录包，再启动定位：

```bash
TEST_DIR="$HOME/gcs_ws/log/faster_lio_official_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$TEST_DIR"
rosbag record --lz4 --repeat-latched -O "$TEST_DIR/height.bag" \
  /livox/lidar /livox/imu /mavros/imu/data_raw /mavros/imu/data \
  /mavros/state /Odometry /cloud_registered /cloud_registered_body \
  /path /tf /tf_static /rosout /rosout_agg
```

定位终端加载标定配置，核对在线估计关闭和外参正确，再启动。额外 ROS 参数仅统一显示坐标名并关闭测试文件保存，不添加时间/高度补偿：

```bash
rosparam load "$(rospack find faster_lio)/config/mid360.yaml"
rosparam set /publish/tf_world_frame camera_init
rosparam set /pcd_save/pcd_save_en false
rosparam set /path_save_en false
rosparam get /mapping/extrinsic_est_en
rosparam get /mapping/extrinsic_T
rosparam get /mapping/extrinsic_R
rosrun faster_lio run_mapping_online __name:=laserMapping_official \
  /laserMapping/odometry:=/Odometry \
  /laserMapping/cloud_registered:=/cloud_registered \
  /laserMapping/cloud_registered_body:=/cloud_registered_body \
  /laserMapping/path:=/path
```

查看点云和里程计，使用现有工作区显示配置：

```bash
rviz -d "$(rospack find diff_planner)/launch/include/lio_cloud_odom.rviz"
```

Fixed Frame 为 `camera_init`，点云 `/cloud_registered`，里程计 `/Odometry`，轨迹 `/path`。静止完成初始化，再手持沿平地往返，比较初始和返回同高度后的静止 Z，同时检查 `rostopic hz /Odometry`、`rostopic delay /Odometry`。本次尚未启动节点、录包或验证漂移是否改善。
