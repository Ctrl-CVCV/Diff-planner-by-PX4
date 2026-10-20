#!/usr/bin/env bash
# One-click startup: PX4/MAVROS -> IMU rate keeper -> Livox MID360s driver
#                    -> FAST-LIO2 + RViz -> EKF.
#
# This script intentionally does NOT start px4ctrl, arm the vehicle, or send any
# takeoff command. Press Ctrl-C to stop the processes started by this script.

set -Eeuo pipefail

WS="${GCS_WS:-$HOME/gcs_ws}"
ROS_SETUP="/opt/ros/noetic/setup.bash"
WS_SETUP="$WS/devel/setup.bash"
FCU_URL="${FCU_URL:-/dev/ttyACM0:921600}"
IMU_RATE="${IMU_RATE:-200}"
LIVOX_HOST_IP="${LIVOX_HOST_IP:-192.168.1.5}"
LIVOX_DEVICE_IP="${LIVOX_DEVICE_IP:-192.168.1.124}"
LIVOX_LAUNCH="${LIVOX_LAUNCH:-msg_MID360s.launch}"
LIVOX_NODE="/livox_lidar_publisher2"
LIVOX_TOPIC="/livox/lidar"
START_MAVROS=1
CHECK_ONLY=0
WAIT_MAVROS=35
WAIT_LIDAR=45
LIVOX_START_ATTEMPTS=2
WAIT_ODOM=45
WAIT_EKF=30

usage() {
  cat <<USAGE
用法：$(basename "$0") [选项]

默认启动：
  1. roscore（若尚未运行）
  2. MAVROS: ${FCU_URL}
  3. PX4 IMU 频率保持节点: ${IMU_RATE} Hz
  4. Livox MID360s 驱动节点 /livox_lidar_publisher2
     并确认点云话题 /livox/lidar 有实际数据
  5. FAST-LIO2，并打开 RViz
  6. EKF，输出 /ekf/ekf_odom

选项：
  --fcu-url URL       MAVROS FCU地址（默认：${FCU_URL}）
  --imu-rate HZ       请求的PX4 IMU消息频率（默认：${IMU_RATE}）
  --no-mavros         不启动MAVROS，但仍要求现有/mavros话题可用
  --check-only        只检查文件和ROS包，不启动任何节点
  -h, --help          显示帮助

停止：在脚本终端按 Ctrl-C。
日志：$WS/log/fastlio2_ekf_<时间>/
USAGE
}

while (($#)); do
  case "$1" in
    --fcu-url)
      [[ $# -ge 2 ]] || { echo "错误：--fcu-url 缺少参数" >&2; exit 2; }
      FCU_URL="$2"; shift 2 ;;
    --imu-rate)
      [[ $# -ge 2 ]] || { echo "错误：--imu-rate 缺少参数" >&2; exit 2; }
      IMU_RATE="$2"; shift 2 ;;
    --no-mavros) START_MAVROS=0; shift ;;
    --check-only) CHECK_ONLY=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "错误：未知参数 $1" >&2; usage >&2; exit 2 ;;
  esac
done

if ! [[ "$IMU_RATE" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "错误：--imu-rate 必须是正数。" >&2
  exit 2
fi

for file in "$ROS_SETUP" "$WS_SETUP"; do
  [[ -r "$file" ]] || { echo "错误：找不到 $file" >&2; exit 1; }
done

# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1090
source "$WS_SETUP"

required_packages=(livox_ros_driver2 fast_lio ekf)
(( START_MAVROS )) && required_packages+=(mavros)
for pkg in "${required_packages[@]}"; do
  if ! rospack find "$pkg" >/dev/null 2>&1; then
    echo "错误：ROS包 '$pkg' 不可见。请重新编译并 source $WS_SETUP" >&2
    exit 1
  fi
done

required_files=(
  "$WS/src/livox_ros_driver2/launch_ROS1/$LIVOX_LAUNCH"
  "$WS/src/livox_ros_driver2/config/MID360s_config.json"
  "$WS/src/livox_ros_driver2/launch_ROS1/set_px4_imu_rate.launch"
  "$WS/src/FAST_LIO/launch/mapping_mid360_px4.launch"
  "$WS/src/FAST_LIO/config/mid360_px4.yaml"
  "$WS/src/Diff-Planner/src/realflight_modules/faster-lio/ekf_pose/launch/ekf_fastlio2.launch"
)
for file in "${required_files[@]}"; do
  [[ -r "$file" ]] || { echo "错误：缺少文件 $file" >&2; exit 1; }
done

if (( CHECK_ONLY )); then
  python3 -m json.tool "$WS/src/livox_ros_driver2/config/MID360s_config.json" >/dev/null || {
    echo "错误：MID360s_config.json 不是有效的JSON文件。" >&2
    exit 1
  }
  echo "检查通过：工作空间、ROS包、launch文件和配置文件均存在。"
  echo "Livox驱动：roslaunch livox_ros_driver2 $LIVOX_LAUNCH"
  echo "Livox配置：主机 $LIVOX_HOST_IP，雷达 $LIVOX_DEVICE_IP"
  echo "FAST-LIO2配置：$WS/src/FAST_LIO/config/mid360_px4.yaml"
  echo "EKF启动文件：$WS/src/Diff-Planner/src/realflight_modules/faster-lio/ekf_pose/launch/ekf_fastlio2.launch"
  exit 0
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$WS/log/fastlio2_ekf_$STAMP"
mkdir -p "$LOG_DIR"

PIDS=()
NAMES=()
STOPPING=0

cleanup() {
  local rc=$?
  (( STOPPING )) && return
  STOPPING=1
  trap - INT TERM EXIT
  echo
  echo "正在停止本脚本启动的节点……"

  local i pid name
  for ((i=${#PIDS[@]}-1; i>=0; --i)); do
    pid="${PIDS[$i]}"
    name="${NAMES[$i]}"
    if kill -0 "$pid" 2>/dev/null; then
      echo "  停止 $name (PID $pid)"
      kill -INT "$pid" 2>/dev/null || true
    fi
  done

  sleep 2
  for pid in "${PIDS[@]}"; do
    kill -TERM "$pid" 2>/dev/null || true
  done


  wait 2>/dev/null || true
  echo "日志保存在：$LOG_DIR"
  exit "$rc"
}
trap cleanup INT TERM EXIT

start_component() {
  local name="$1"
  shift
  local logfile="$LOG_DIR/${name}.log"
  echo "启动 $name；日志：$logfile"
  (
    # shellcheck disable=SC1090
    source "$ROS_SETUP"
    # shellcheck disable=SC1090
    source "$WS_SETUP"
    exec "$@"
  ) >"$logfile" 2>&1 &
  local pid=$!
  PIDS+=("$pid")
  NAMES+=("$name")
  sleep 0.5
  if ! kill -0 "$pid" 2>/dev/null; then
    echo "错误：$name 启动后立即退出：" >&2
    tail -n 40 "$logfile" >&2 || true
    exit 1
  fi
}

stop_component() {
  local target_name="$1"
  local i pid
  for i in "${!NAMES[@]}"; do
    if [[ "${NAMES[$i]}" == "$target_name" ]]; then
      pid="${PIDS[$i]}"
      if kill -0 "$pid" 2>/dev/null; then
        echo "停止 $target_name (PID $pid)"
        kill -INT "$pid" 2>/dev/null || true
        for _ in {1..20}; do
          kill -0 "$pid" 2>/dev/null || break
          sleep 0.25
        done
        kill -TERM "$pid" 2>/dev/null || true
        wait "$pid" 2>/dev/null || true
      fi
      unset 'PIDS[i]' 'NAMES[i]'
      PIDS=("${PIDS[@]}")
      NAMES=("${NAMES[@]}")
      return 0
    fi
  done
}

ros_master_ready() {
  timeout 2 rosparam list >/dev/null 2>&1
}

wait_for_master() {
  local timeout_s="$1"
  local end=$((SECONDS + timeout_s))
  until ros_master_ready; do
    (( SECONDS < end )) || { echo "错误：等待ROS master超时。" >&2; return 1; }
    sleep 0.5
  done
}

wait_for_message() {
  local topic="$1"
  local timeout_s="$2"
  local label="$3"
  printf '等待 %-24s (%s) ' "$topic" "$label"

  # Do not use `rostopic echo -n 1` here. A single Livox CustomMsg can be
  # several megabytes, and converting it to YAML may exceed a short timeout
  # even though the point cloud is arriving normally. AnyMsg confirms receipt
  # without deserializing or printing the full payload.
  if timeout "${timeout_s}s" python3 -c '
import sys
import threading
import rospy

received = threading.Event()
rospy.init_node("wait_for_topic_message", anonymous=True, disable_signals=True)
rospy.Subscriber(sys.argv[1], rospy.AnyMsg,
                 lambda _msg: received.set(), queue_size=1,
                 tcp_nodelay=True)
while not received.wait(0.1) and not rospy.is_shutdown():
    pass
sys.exit(0 if received.is_set() else 1)
' "$topic" >/dev/null 2>&1; then
    echo "[OK]"
    return 0
  fi

  echo "[超时]"
  echo "错误：未收到 $topic。请查看 $LOG_DIR 中对应日志。" >&2
  return 1
}

message_available() {
  local topic="$1"
  local timeout_s="${2:-3}"
  timeout "${timeout_s}s" python3 -c '
import sys
import threading
import rospy

received = threading.Event()
rospy.init_node("probe_topic_message", anonymous=True, disable_signals=True)
rospy.Subscriber(sys.argv[1], rospy.AnyMsg,
                 lambda _msg: received.set(), queue_size=1,
                 tcp_nodelay=True)
while not received.wait(0.1) and not rospy.is_shutdown():
    pass
sys.exit(0 if received.is_set() else 1)
' "$topic" >/dev/null 2>&1
}

wait_for_mavros_connection() {
  local timeout_s="$1"
  local end=$((SECONDS + timeout_s))
  printf '等待 %-24s (PX4连接) ' "/mavros/state"
  while (( SECONDS < end )); do
    if timeout 2 rostopic echo -n 1 /mavros/state 2>/dev/null | grep -q 'connected: True'; then
      echo "[OK]"
      return 0
    fi
    printf '.'
    sleep 0.5
  done
  echo "[超时]"
  echo "错误：MAVROS未连接PX4。请检查飞控电源、USB设备和FCU地址：$FCU_URL" >&2
  return 1
}

wait_for_node() {
  local node="$1"
  local timeout_s="$2"
  local end=$((SECONDS + timeout_s))
  printf '等待 %-24s (ROS节点) ' "$node"
  while (( SECONDS < end )); do
    if rosnode list 2>/dev/null | grep -Fxq "$node"; then
      echo "[OK]"
      return 0
    fi
    printf '.'
    sleep 0.5
  done
  echo "[超时]"
  return 1
}

show_livox_network() {
  echo "当前网卡地址："
  ip -br -4 address 2>/dev/null || true
  echo "当前网卡链路："
  ip -br link 2>/dev/null || true
}

check_livox_network() {
  if ! ip -o -4 address show 2>/dev/null \
      | awk '{print $4}' \
      | cut -d/ -f1 \
      | grep -Fxq "$LIVOX_HOST_IP"; then
    echo "错误：没有网卡配置 Livox 主机地址 $LIVOX_HOST_IP。" >&2
    echo "MID360s配置要求：Jetson=$LIVOX_HOST_IP，雷达=$LIVOX_DEVICE_IP。" >&2
    show_livox_network >&2
    echo "请先连接雷达网线，并给对应有线网卡配置 $LIVOX_HOST_IP/24。" >&2
    return 1
  fi
}

show_livox_failure() {
  echo >&2
  echo "========== Livox 驱动诊断 ==========" >&2
  echo "期望节点：$LIVOX_NODE" >&2
  echo "期望话题：$LIVOX_TOPIC" >&2
  echo "主机地址：$LIVOX_HOST_IP" >&2
  echo "雷达地址：$LIVOX_DEVICE_IP" >&2
  show_livox_network >&2
  echo "当前Livox节点：" >&2
  rosnode list 2>/dev/null | grep -i livox >&2 || true
  echo "当前Livox话题：" >&2
  rostopic list 2>/dev/null | grep -i livox >&2 || true
  if [[ -r "$LOG_DIR/livox_driver.log" ]]; then
    echo "Livox驱动最近80行日志：" >&2
    tail -n 80 "$LOG_DIR/livox_driver.log" >&2 || true
  fi
  echo "====================================" >&2
}

printf '\n========== FAST-LIO2 + EKF 一键启动 ==========\n'
printf '工作空间 : %s\n' "$WS"
printf 'MAVROS    : %s\n' "$FCU_URL"
printf 'IMU请求频率: %s Hz\n' "$IMU_RATE"
printf '日志目录 : %s\n\n' "$LOG_DIR"

# Check the wired LiDAR network before starting the rest of the stack. This
# avoids starting MAVROS and waiting for IMU topics only to discover later that
# the Jetson has no interface on the LiDAR subnet.
check_livox_network

# 1. ROS master
if ros_master_ready; then
  echo "复用现有 ROS master。"
else
  start_component roscore roscore
  wait_for_master 15
fi

# 2. MAVROS
if timeout 2 rostopic echo -n 1 /mavros/state 2>/dev/null | grep -q 'connected: True'; then
  echo "复用现有 MAVROS/PX4 连接。"
else
  if (( START_MAVROS )); then
    start_component mavros roslaunch mavros px4.launch "fcu_url:=$FCU_URL"
  else
    echo "--no-mavros 已指定，等待外部MAVROS连接。"
  fi
  wait_for_mavros_connection "$WAIT_MAVROS"
fi

# 3. Keep PX4 IMU stream at a usable rate.
if rosnode list 2>/dev/null | grep -qx '/px4_imu_rate_keeper'; then
  echo "复用现有 /px4_imu_rate_keeper。"
else
  start_component imu_rate_keeper roslaunch livox_ros_driver2 set_px4_imu_rate.launch "rate_hz:=$IMU_RATE"
fi
wait_for_message /mavros/imu/data_raw "$WAIT_MAVROS" "FAST-LIO2 IMU"
wait_for_message /mavros/imu/data "$WAIT_MAVROS" "EKF IMU"

# 4. Livox MID360s driver. Do not start FAST-LIO2 until actual point data arrives.
if message_available "$LIVOX_TOPIC" 5; then
  echo "复用现有 Livox 点云话题。"
else
  # A registered node without point data is normally a stale or half-started
  # driver. Stop that one before starting a fresh driver to avoid duplicate
  # node names and UDP-port conflicts.
  if rosnode list 2>/dev/null | grep -Fxq "$LIVOX_NODE"; then
    echo "发现已有 $LIVOX_NODE，但没有点云数据；先停止旧驱动。"
    rosnode kill "$LIVOX_NODE" >/dev/null 2>&1 || true
    sleep 2
  fi

  livox_ready=0
  for ((attempt=1; attempt<=LIVOX_START_ATTEMPTS; ++attempt)); do
    echo "启动Livox驱动（第 $attempt/$LIVOX_START_ATTEMPTS 次）：roslaunch livox_ros_driver2 $LIVOX_LAUNCH"
    start_component livox_driver roslaunch livox_ros_driver2 "$LIVOX_LAUNCH"

    if wait_for_node "$LIVOX_NODE" 12 \
        && wait_for_message "$LIVOX_TOPIC" "$WAIT_LIDAR" "Livox点云"; then
      livox_ready=1
      break
    fi

    echo "警告：本次Livox驱动已经启动，但没有收到点云数据。" >&2
    show_livox_failure
    stop_component livox_driver

    if (( attempt < LIVOX_START_ATTEMPTS )); then
      echo "雷达冷启动时数据流可能尚未就绪，等待5秒后自动重启驱动……"
      sleep 5
    fi
  done

  if (( ! livox_ready )); then
    echo "错误：重试 $LIVOX_START_ATTEMPTS 次后仍未收到 $LIVOX_TOPIC。" >&2
    exit 1
  fi
fi

# 5. FAST-LIO2 with RViz
if timeout 2 rostopic echo -n 1 /Odometry >/dev/null 2>&1; then
  echo "复用现有 FAST-LIO2 /Odometry。"
  if ! rosnode list 2>/dev/null | grep -qx '/rviz'; then
    start_component rviz rosrun rviz rviz -d "$WS/src/FAST_LIO/rviz_cfg/loam_livox.rviz"
  fi
else
  start_component fast_lio_rviz roslaunch fast_lio mapping_mid360_px4.launch rviz:=true
  wait_for_message /Odometry "$WAIT_ODOM" "FAST-LIO2里程计"
fi

# 6. EKF
if timeout 2 rostopic echo -n 1 /ekf/ekf_odom >/dev/null 2>&1; then
  echo "复用现有 EKF /ekf/ekf_odom。"
else
  start_component ekf roslaunch ekf ekf_fastlio2.launch
  wait_for_message /ekf/ekf_odom "$WAIT_EKF" "融合里程计"
fi

cat <<SUMMARY

================ 启动完成 ================
LiDAR       : /livox/lidar
FAST-LIO2   : /Odometry
注册点云    : /cloud_registered
EKF         : /ekf/ekf_odom
RViz        : 已请求启动

请在另外一个终端检查：
  source $WS_SETUP
  rostopic hz -w 1000 /mavros/imu/data
  rostopic hz -w 1000 /Odometry
  rostopic hz -w 1000 /ekf/ekf_odom

静止检查：
  rostopic echo -n 1 /ekf/ekf_odom

注意：本脚本不会启动px4ctrl，不会解锁，也不会发送起飞命令。
按 Ctrl-C 停止本脚本启动的全部组件。
==========================================
SUMMARY

# Supervise children and keep the stack alive.
while true; do
  sleep 2
  for i in "${!PIDS[@]}"; do
    if ! kill -0 "${PIDS[$i]}" 2>/dev/null; then
      echo "错误：组件 ${NAMES[$i]} 已退出。最近日志：" >&2
      tail -n 50 "$LOG_DIR/${NAMES[$i]}.log" >&2 || true
      exit 1
    fi
  done
done
