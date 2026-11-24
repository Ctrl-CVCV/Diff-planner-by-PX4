#!/usr/bin/env python3
"""地面站控制台：运行功能 + 调试功能。

运行：定位链、控制/规划/视觉/舵机节点、任务脚本、航点打点、地图显示、录包。
调试：USB 摄像头录像（640x480）、不飞行时的地图/里程计录包。

本程序只负责启动/停止子进程和读写任务配置，不自行解锁、不发控制指令；
任务脚本的 --execute 仍由脚本自己的 MISSION 确认流程把关（本界面会再确认一次）。
"""

import json
import math
import os
import signal
import sys
import time
from pathlib import Path

from PyQt5 import QtCore, QtGui, QtWidgets

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gcs_console_core as core  # noqa: E402


class RosWorker(QtCore.QThread):
    """后台跑 rospy：EKF 位姿、话题频率/延迟、ROS 节点存活、PX4/PX4Ctrl 状态、/rosout 告警。"""

    pose_signal = QtCore.pyqtSignal(float, float, float, float)   # x, y, z, age
    mission_signal = QtCore.pyqtSignal(dict)
    status_signal = QtCore.pyqtSignal(bool, str)
    health_signal = QtCore.pyqtSignal(dict)

    TOPIC_SPECS = (
        ('lio', '/Odometry', 10.0, 'LIO /Odometry'),
        ('ekf', '/ekf/ekf_odom', None, 'EKF /ekf/ekf_odom'),
        ('imu', '/mavros/imu/data', None, 'IMU /mavros/imu/data'),
        ('fcu', '/mavros/state', None, 'PX4 /mavros/state'),
        ('ctrl', '/px4ctrl/state', None, 'PX4Ctrl 状态'),
        ('mission', '/drone_0_mission/state', None, '任务状态'),
    )
    NODE_SPECS = (
        ('mavros', '/mavros'),
        ('livox', '/livox_lidar_publisher2'),
        ('lio', '/laserMapping'),
        ('ekf', '/ekf'),
        ('vision_bridge', '/ekf_vision_bridge'),
        ('px4ctrl', '/px4ctrl'),
        ('planner', '/drone_0_diff_planner_node'),
        ('traj_server', '/drone_0_traj_server'),
        ('yolo', '/yolo_trt_detector'),
        ('servo', '/servo_aux_pwm'),
    )
    # 注册表里有名字不代表进程还活着：这些节点对应话题断更时，视为进程已退出/断连。
    NODE_TOPICS = dict(mavros='fcu', lio='lio', ekf='ekf', px4ctrl='ctrl')

    def __init__(self):
        super().__init__()
        from collections import deque
        self._stop = False
        self.pose = None          # (x, y, z, age, yaw)
        self.origin = None        # [x, y, z] or None
        self.stage = ''
        self._last_emit = 0.0
        self.arrivals = {key: deque(maxlen=300) for key, _, _, _ in self.TOPIC_SPECS}
        self.messages = {}
        self.nodes = {label: None for label, _ in self.NODE_SPECS}
        self.alerts = deque(maxlen=200)
        self._alert_last = {}
        self._last_pose_sample = None    # (monotonic, xyz)
        self._lio_position = None        # (monotonic, xyz)
        self._node_poll = 0.0
        self._last_mission = ''
        self.rospy = None

    # ------------------------------------------------------------ helpers
    def _add_alert(self, level, text, node='monitor'):
        now = time.monotonic()
        if now - self._alert_last.get(text, 0.0) < 3.0:
            return
        self._alert_last[text] = now
        self.alerts.append(dict(time=time.strftime('%H:%M:%S'), level=level,
                                node=node, text=text))

    def _message_cb(self, key, message):
        now = time.monotonic()
        self.arrivals[key].append(now)
        self.messages[key] = (message, now)
        if key == 'ekf':
            position = message.pose.pose.position
            orientation = message.pose.pose.orientation
            yaw = math.atan2(2 * (orientation.w * orientation.z + orientation.x * orientation.y),
                             1 - 2 * (orientation.y ** 2 + orientation.z ** 2))
            now_ros = self.rospy.get_time() if self.rospy else time.time()
            age = now_ros - message.header.stamp.to_sec()
            self.pose = (position.x, position.y, position.z, age, yaw)
            if now_ros - self._last_emit >= 0.2:
                self._last_emit = now_ros
                self.pose_signal.emit(position.x, position.y, position.z, age)
        elif key == 'lio':
            position = message.pose.pose.position
            self._lio_position = (now, (position.x, position.y, position.z))
        elif key == 'mission':
            try:
                import json
                payload = json.loads(message.data)
            except Exception:
                return
            origin = payload.get('origin_world')
            if isinstance(origin, (list, tuple)) and len(origin) == 3:
                self.origin = [float(value) for value in origin]
            self.stage = str(payload.get('stage', ''))
            if self.stage != self._last_mission:
                self._last_mission = self.stage
                self.mission_signal.emit(payload)

    def _poll_nodes(self, now):
        if now - self._node_poll < 2.0:
            return
        self._node_poll = now
        try:
            import rosnode
            names = set(rosnode.get_node_names())
        except Exception:
            self.nodes = {label: None for label, _ in self.NODE_SPECS}
            return
        self.nodes = {label: (name in names) for label, name in self.NODE_SPECS}
        # 已收到过数据但超过 5 秒断更 → 认为该进程已退出或断连（避免把残留注册显示成绿色）。
        topic_names = {key: topic for key, topic, _, _ in self.TOPIC_SPECS}
        names_by_label = dict(self.NODE_SPECS)
        for label, key in self.NODE_TOPICS.items():
            if not self.nodes.get(label):
                continue
            entry = self.messages.get(key)
            if entry is not None and now - entry[1] > 5.0:
                self.nodes[label] = False
                self._add_alert('WARN', '%s 已注册但 %s 已 %.0f 秒无数据（进程可能已退出或断连）'
                                % (names_by_label[label], topic_names[key], now - entry[1]))

    def _health(self, now):
        topics = {}
        for key, topic, expected, display in self.TOPIC_SPECS:
            entry = self.messages.get(key)
            age = None if entry is None else now - entry[1]
            # 对带 header 的消息，"延迟"用消息时间戳与当前 ROS 时间的差（能反映 LIO 积压），
            # 接收年龄只作为回退。
            delay = age
            if entry is not None:
                try:
                    stamp = entry[0].header.stamp
                    current = self.rospy.get_time() if self.rospy else time.time()
                    delay = current - stamp.to_sec()
                except Exception:
                    delay = age
            rate = core.rate_from_stamps(self.arrivals[key], now)
            level = core.LEVEL_BAD if entry is None else core.topic_level(rate, delay, expected)
            topics[key] = dict(topic=topic, display=display, rate=rate, age=age,
                               delay=delay, expected=expected, level=level)

        ekf = self.messages.get('ekf')
        pose = None
        speed = None
        divergence = None
        if ekf is not None:
            message = ekf[0]
            position = message.pose.pose.position
            try:
                age = self.rospy.get_time() - message.header.stamp.to_sec() if self.rospy else now - ekf[1]
            except Exception:
                age = now - ekf[1]
            orientation = message.pose.pose.orientation
            yaw = math.atan2(2 * (orientation.w * orientation.z + orientation.x * orientation.y),
                             1 - 2 * (orientation.y ** 2 + orientation.z ** 2))
            pose = (position.x, position.y, position.z, age, yaw)
            twist = message.twist.twist.linear
            speed = math.sqrt(twist.x ** 2 + twist.y ** 2 + twist.z ** 2)
            lio = self._lio_position
            if lio is not None and now - lio[0] <= 0.5:
                divergence = math.dist((position.x, position.y, position.z), lio[1])

        # 异常检测（位置跳变 / 速度 / EKF-LIO 偏差），按消息去抖
        if pose is not None:
            dt = None
            prev = None
            if self._last_pose_sample is not None:
                prev = self._last_pose_sample[1]
                dt = now - self._last_pose_sample[0]
            self._last_pose_sample = (now, (pose[0], pose[1], pose[2]))
            for level, text in core.motion_anomalies(pose[:3], prev, dt, speed, divergence):
                self._add_alert(level, text)

        self._poll_nodes(now)

        fcu = self.messages.get('fcu')
        px4 = None
        if fcu is not None and now - fcu[1] <= 2.0:
            message = fcu[0]
            px4 = dict(connected=bool(message.connected), armed=bool(message.armed),
                       mode=str(message.mode))
        ctrl = self.messages.get('ctrl')
        control = None
        if ctrl is not None and now - ctrl[1] <= 2.0:
            state = int(ctrl[0].state)
            control = dict(state=state, name=core.ctrl_state_name(state))

        return dict(pose=pose, speed=speed, divergence=divergence,
                    topics=topics, nodes=dict(self.nodes),
                    px4=px4, ctrl=control, mission_stage=self.stage,
                    alerts=list(self.alerts)[-12:])

    # ---------------------------------------------------------------- run
    def run(self):
        import rospy
        from mavros_msgs.msg import State
        from nav_msgs.msg import Odometry
        from quadrotor_msgs.msg import Px4ctrlState
        from rosgraph_msgs.msg import Log
        from sensor_msgs.msg import Imu
        from std_msgs.msg import String

        while not self._stop:
            try:
                rospy.init_node('gcs_console', anonymous=True, disable_signals=True)
                break
            except Exception as error:  # master 未启动
                self.status_signal.emit(False, '等待 ROS master…（%s）' % error)
                time.sleep(2.0)
        if self._stop:
            return
        self.rospy = rospy

        types = dict(lio=Odometry, ekf=Odometry, imu=Imu, fcu=State,
                     ctrl=Px4ctrlState, mission=String)
        for key, topic, _, _ in self.TOPIC_SPECS:
            rospy.Subscriber(topic, types[key], lambda message, key=key: self._message_cb(key, message),
                             queue_size=1, tcp_nodelay=True)

        def rosout_cb(message):
            if message.level < 4:
                return
            if message.name.startswith('/gcs_console'):
                return
            level = 'ERROR' if message.level >= 8 else 'WARN'
            self._add_alert(level, message.msg.strip()[:200], message.name)

        rospy.Subscriber('/rosout', Log, rosout_cb, queue_size=100)

        self.status_signal.emit(True, 'ROS 已连接，等待 /ekf/ekf_odom…')
        last_health = 0.0
        while not self._stop and not rospy.is_shutdown():
            now = time.monotonic()
            if now - last_health >= 0.5:
                last_health = now
                try:
                    self.health_signal.emit(self._health(now))
                except Exception as error:
                    self._add_alert('WARN', '健康检查异常：%s' % error)
            time.sleep(0.1)
        try:
            rospy.signal_shutdown('gcs_console exit')
        except Exception:
            pass

    def stop(self):
        self._stop = True


class ProcessManager(QtCore.QObject):
    """管理子进程：统一日志、状态、SIGINT 停止。"""

    output = QtCore.pyqtSignal(str, str)          # name, text
    state = QtCore.pyqtSignal(str, bool, int)     # name, running, exit_code

    def __init__(self, parent=None):
        super().__init__(parent)
        self.processes = {}
        self.names = {}

    def running(self, name):
        process = self.processes.get(name)
        return process is not None and process.state() != QtCore.QProcess.NotRunning

    def start(self, name, command):
        if self.running(name):
            self.output.emit(name, '已在运行，忽略重复启动\n')
            return False
        process = QtCore.QProcess(self)
        process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(lambda: self._read(name))
        process.finished.connect(lambda code, status: self._finished(name, code))
        process.errorOccurred.connect(lambda error: self.output.emit(name, '进程错误：%s\n' % error))
        self.processes[name] = process
        self.output.emit(name, '启动：%s\n' % command)
        process.start('/bin/bash', ['-c', command])
        self.state.emit(name, True, 0)
        return True

    def _read(self, name):
        process = self.processes.get(name)
        if process is None:
            return
        data = bytes(process.readAllStandardOutput()).decode('utf-8', 'replace')
        if data:
            self.output.emit(name, data)

    def _finished(self, name, code):
        self.output.emit(name, '进程结束（exit=%d）\n' % code)
        self.state.emit(name, False, code)
        self.processes.pop(name, None)

    def stop(self, name, grace_ms=6000):
        process = self.processes.get(name)
        if process is None or process.state() == QtCore.QProcess.NotRunning:
            return
        pid = process.processId()
        self.output.emit(name, '发送 SIGINT 停止（pid=%d）…\n' % pid)
        try:
            os.kill(pid, signal.SIGINT)
        except OSError as error:
            self.output.emit(name, '发送 SIGINT 失败：%s\n' % error)
        QtCore.QTimer.singleShot(grace_ms, lambda: self._force_stop(name))

    def _force_stop(self, name):
        process = self.processes.get(name)
        if process is not None and process.state() != QtCore.QProcess.NotRunning:
            self.output.emit(name, '超时未退出，强制终止\n')
            process.terminate()
            QtCore.QTimer.singleShot(3000, lambda: process.kill() if process.state() != QtCore.QProcess.NotRunning else None)

    def stop_all(self):
        for name in list(self.processes):
            self.stop(name)


class Console(QtWidgets.QMainWindow):

    def __init__(self):
        super().__init__()
        self.setWindowTitle('GCS 地面站控制台')
        self.resize(1180, 860)
        self.ws = core.workspace_root()
        self.config_path = core.mission_config_path(self.ws)
        self.points = core.load_points(self.config_path)
        self.last_capture = None
        self.processes = ProcessManager(self)
        self.processes.output.connect(self.append_log)
        self.processes.state.connect(self.on_process_state)
        self._build_ui()
        self.ros = RosWorker()
        self.ros.pose_signal.connect(self.on_pose)
        self.ros.mission_signal.connect(self.on_mission)
        self.ros.status_signal.connect(self.on_ros_status)
        self.ros.health_signal.connect(self.on_health)
        self.ros.start()
        self._update_pose_timer = QtCore.QTimer(self)
        self._update_pose_timer.timeout.connect(self.refresh_pose_labels)
        self._update_pose_timer.start(500)

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        central = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(central)
        self.tabs = QtWidgets.QTabWidget()
        self.tabs.addTab(self._build_run_tab(), '运行功能')
        self.tabs.addTab(self._build_debug_tab(), '调试功能')
        layout.addWidget(self.tabs, 1)

        log_bar = QtWidgets.QHBoxLayout()
        log_bar.addWidget(QtWidgets.QLabel('运行日志'))
        self.autoscroll = QtWidgets.QCheckBox('自动滚动')
        self.autoscroll.setChecked(True)
        log_bar.addWidget(self.autoscroll)
        clear = QtWidgets.QPushButton('清空日志')
        clear.clicked.connect(lambda: self.log.clear())
        log_bar.addWidget(clear)
        layout.addLayout(log_bar)

        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(6000)
        self.log.setFont(QtGui.QFont('Monospace', 9))
        layout.addWidget(self.log, 1)
        self.setCentralWidget(central)
        self.setStatusBar(QtWidgets.QStatusBar())
        self.status_ros = QtWidgets.QLabel('ROS：未连接')
        self.status_pose = QtWidgets.QLabel('EKF：--')
        self.statusBar().addWidget(self.status_ros)
        self.statusBar().addPermanentWidget(self.status_pose)

    def _group(self, title):
        group = QtWidgets.QGroupBox(title)
        grid = QtWidgets.QGridLayout(group)
        return group, grid

    def _component(self, grid, row, label, component, command_fn, hint=''):
        state = QtWidgets.QLabel('未运行')
        state.setStyleSheet('color: #888;')
        start = QtWidgets.QPushButton('启动')
        stop = QtWidgets.QPushButton('停止')
        stop.setEnabled(False)
        start.clicked.connect(lambda: self.processes.start(component, command_fn()))
        stop.clicked.connect(lambda: self.processes.stop(component))
        grid.addWidget(QtWidgets.QLabel(label), row, 0)
        grid.addWidget(start, row, 1)
        grid.addWidget(stop, row, 2)
        grid.addWidget(state, row, 3)
        if hint:
            note = QtWidgets.QLabel(hint)
            note.setStyleSheet('color: #777;')
            note.setWordWrap(True)
            grid.addWidget(note, row, 4)
        setattr(self, 'state_' + component, state)
        setattr(self, 'start_' + component, start)
        setattr(self, 'stop_' + component, stop)
        return start, stop

    def _build_health_group(self):
        group = QtWidgets.QGroupBox('实时监视（EKF 位姿 / 话题 / 节点 / 告警）')
        grid = QtWidgets.QGridLayout(group)
        self.health_pose = QtWidgets.QLabel('EKF：等待数据')
        self.health_motion = QtWidgets.QLabel('速度 / 偏差：--')
        self.health_px4 = QtWidgets.QLabel('PX4：--')
        self.health_ctrl = QtWidgets.QLabel('PX4Ctrl：--')
        self.health_stage = QtWidgets.QLabel('任务阶段：--')
        grid.addWidget(self.health_pose, 0, 0, 1, 4)
        grid.addWidget(self.health_motion, 1, 0, 1, 2)
        grid.addWidget(self.health_stage, 1, 2, 1, 2)
        grid.addWidget(self.health_px4, 2, 0, 1, 2)
        grid.addWidget(self.health_ctrl, 2, 2, 1, 2)

        self.topic_table = QtWidgets.QTableWidget(len(RosWorker.TOPIC_SPECS), 3)
        self.topic_table.setHorizontalHeaderLabels(['话题', '频率', '延迟'])
        self.topic_table.verticalHeader().setVisible(False)
        self.topic_table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        self.topic_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.topic_table.setFixedHeight(24 + 22 * len(RosWorker.TOPIC_SPECS))
        self.topic_rows = {}
        for row, (key, _, _, display) in enumerate(RosWorker.TOPIC_SPECS):
            self.topic_table.setItem(row, 0, QtWidgets.QTableWidgetItem(display))
            self.topic_table.setItem(row, 1, QtWidgets.QTableWidgetItem('--'))
            self.topic_table.setItem(row, 2, QtWidgets.QTableWidgetItem('--'))
            self.topic_rows[key] = row
        grid.addWidget(self.topic_table, 3, 0, 1, 4)

        chips = QtWidgets.QWidget()
        chip_row = QtWidgets.QHBoxLayout(chips)
        chip_row.setContentsMargins(0, 0, 0, 0)
        self.node_labels = {}
        for label, _ in RosWorker.NODE_SPECS:
            chip = QtWidgets.QLabel('● ' + label)
            chip.setStyleSheet('color: #999;')
            chip_row.addWidget(chip)
            self.node_labels[label] = chip
        chip_row.addStretch(1)
        grid.addWidget(chips, 4, 0, 1, 4)

        alert_row = QtWidgets.QHBoxLayout()
        self.alert_count = QtWidgets.QLabel('告警：0')
        clear = QtWidgets.QPushButton('清空告警显示')
        clear.clicked.connect(lambda: self.alert_box.clear())
        alert_row.addWidget(self.alert_count)
        alert_row.addStretch(1)
        alert_row.addWidget(clear)
        alert_widget = QtWidgets.QWidget()
        alert_widget.setLayout(alert_row)
        grid.addWidget(alert_widget, 5, 0, 1, 4)
        self.alert_box = QtWidgets.QPlainTextEdit()
        self.alert_box.setReadOnly(True)
        self.alert_box.setMaximumBlockCount(200)
        self.alert_box.setFixedHeight(110)
        self.alert_box.setFont(QtGui.QFont('Monospace', 9))
        grid.addWidget(self.alert_box, 6, 0, 1, 4)
        return group

    def _paint(self, label, text, level):
        colors = {core.LEVEL_OK: '#1a7f37', core.LEVEL_WARN: '#b26a00', core.LEVEL_BAD: '#c62828'}
        label.setText(text)
        label.setStyleSheet('color: %s;' % colors.get(level, '#333'))

    def on_health(self, data):
        nodes = data.get('nodes') or {}
        pose = data.get('pose')
        if pose is None:
            if nodes.get('ekf'):
                self._paint(self.health_pose, 'EKF：节点在运行但收不到 /ekf/ekf_odom', core.LEVEL_BAD)
            else:
                self._paint(self.health_pose, 'EKF：未运行（等待定位链启动）', core.LEVEL_WARN)
                self.status_pose.setText('EKF：--')
        else:
            x, y, z, age, yaw = pose
            level = core.LEVEL_OK if age < 0.3 else core.LEVEL_WARN if age < 1.0 else core.LEVEL_BAD
            self._paint(self.health_pose,
                        'EKF：x=%.3f  y=%.3f  z=%.3f  yaw=%.1f°  延迟 %.0f ms'
                        % (x, y, z, math.degrees(yaw), max(0.0, age) * 1000.0), level)
            self.status_pose.setText('EKF：x=%.3f y=%.3f z=%.3f（%.0f ms）'
                                     % (x, y, z, max(0.0, age) * 1000.0))

        speed = data.get('speed')
        divergence = data.get('divergence')
        speed_text = '--' if speed is None else '%.2f m/s' % speed
        divergence_text = '--' if divergence is None else '%.2f m' % divergence
        level = core.LEVEL_OK
        if speed is not None:
            level = core.LEVEL_BAD if speed > 1.0 else core.LEVEL_WARN if speed > 0.65 else level
        if divergence is not None:
            if divergence > 0.35:
                level = core.LEVEL_BAD
            elif divergence > 0.20 and level == core.LEVEL_OK:
                level = core.LEVEL_WARN
        self._paint(self.health_motion,
                    '速度：%s    EKF-LIO 偏差：%s' % (speed_text, divergence_text), level)

        px4 = data.get('px4')
        if px4 is None:
            level = core.LEVEL_BAD if nodes.get('mavros') else core.LEVEL_WARN
            self._paint(self.health_px4, 'PX4：无 /mavros/state', level)
        else:
            level = core.LEVEL_OK if px4['connected'] else core.LEVEL_BAD
            self._paint(self.health_px4,
                        'PX4：%s  %s  %s' % ('已连接' if px4['connected'] else '未连接',
                                             '已解锁' if px4['armed'] else '未解锁', px4['mode']), level)

        ctrl = data.get('ctrl')
        if ctrl is None:
            self._paint(self.health_ctrl, 'PX4Ctrl：未运行 / 无状态', core.LEVEL_WARN)
        else:
            level = core.LEVEL_OK if ctrl['state'] in (1, 2, 3, 4, 5) else core.LEVEL_WARN
            self._paint(self.health_ctrl, 'PX4Ctrl：%s (%d)' % (ctrl['name'], ctrl['state']), level)

        self._paint(self.health_stage, '任务阶段：%s' % (data.get('mission_stage') or '--'),
                    core.LEVEL_OK)

        for key, info in data.get('topics', {}).items():
            row = self.topic_rows.get(key)
            if row is None:
                continue
            rate_text = '--' if info['rate'] <= 0 else '%.1f Hz' % info['rate']
            delay = info.get('delay', info['age'])
            age_text = '--' if delay is None else '%.0f ms' % (delay * 1000.0)
            if delay is None:
                color = '#999'   # 还没收到过：未运行，不算异常
            else:
                color = {core.LEVEL_OK: '#1a7f37', core.LEVEL_WARN: '#b26a00',
                         core.LEVEL_BAD: '#c62828'}[info['level']]
            for column, text in ((1, rate_text), (2, age_text)):
                item = self.topic_table.item(row, column)
                item.setText(text)
                item.setForeground(QtGui.QBrush(QtGui.QColor(color)))
            self.topic_table.item(row, 0).setForeground(
                QtGui.QBrush(QtGui.QColor(color if info['level'] != core.LEVEL_OK else '#333')))

        for label, alive in (data.get('nodes') or {}).items():
            chip = self.node_labels.get(label)
            if chip is None:
                continue
            if alive is None:
                chip.setStyleSheet('color: #999;')
            elif alive:
                chip.setStyleSheet('color: #1a7f37;')
            else:
                chip.setStyleSheet('color: #b0b0b0;')

        alerts = data.get('alerts') or []
        lines = ['[%s] %-5s %s: %s' % (item['time'], item['level'], item['node'], item['text'])
                 for item in alerts]
        self.alert_box.setPlainText('\n'.join(lines))
        self.alert_box.verticalScrollBar().setValue(self.alert_box.verticalScrollBar().maximum())
        bad = sum(1 for item in alerts if item['level'] in ('ERROR', 'BAD'))
        self.alert_count.setText('告警：最近 %d 条，严重 %d 条' % (len(alerts), bad))
        self.alert_count.setStyleSheet('color: %s;' % ('#c62828' if bad else '#b26a00' if alerts else '#1a7f37'))

    def _build_run_tab(self):
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)
        layout.addWidget(self._build_health_group())

        group, grid = self._group('定位链（MAVROS + Livox + faster-lio + EKF + 视觉位姿桥）')
        self._component(grid, 0, 'faster-lio + EKF', 'lio', core.cmd_lio,
                        '脚本不启动 PX4Ctrl、不解锁；日志在 ~/gcs_ws/log/')
        layout.addWidget(group)

        group, grid = self._group('控制 / 规划 / 视觉 / 舵机')
        start_all = QtWidgets.QPushButton('一键启动全部')
        stop_all = QtWidgets.QPushButton('一键停止全部')
        start_all.clicked.connect(self.start_control_stack)
        stop_all.clicked.connect(self.stop_control_stack)
        grid.addWidget(start_all, 0, 0)
        grid.addWidget(stop_all, 0, 1)
        note = QtWidgets.QLabel('按 PX4Ctrl → Planner → YOLO → 舵机 顺序间隔启动')
        note.setStyleSheet('color: #777;')
        grid.addWidget(note, 0, 2, 1, 3)
        self._component(grid, 1, 'PX4Ctrl', 'px4ctrl', core.cmd_px4ctrl)
        self._component(grid, 2, 'Diff-Planner', 'planner', core.cmd_planner,
                        '只启动 run_exp_hybrid_mission.launch，不要重复启动单 LIO launch')
        self.yolo_window = QtWidgets.QCheckBox('YOLO 显示窗口')
        self.yolo_window.setChecked(True)
        grid.addWidget(self.yolo_window, 3, 3)
        self._component(grid, 3, 'YOLO 视觉', 'yolo',
                        lambda: core.cmd_yolo(self.ws, self.yolo_window.isChecked()))
        self._component(grid, 4, '舵机控制', 'servo', core.cmd_servo,
                        'payload_servos.launch：PWM_MAIN5..8，服务 /servo1../servo4，hold_s=2s 自动回位')
        layout.addWidget(group)

        group, grid = self._group('任务脚本（独立入口）')
        self.mission_mode = QtWidgets.QComboBox()
        self.mission_mode.addItems(['预检（--check，只读，不发飞行指令）',
                                    '执行（--execute，会发送 TAKEOFF）'])
        grid.addWidget(self.mission_mode, 0, 0, 1, 3)
        self.mission_map = QtWidgets.QCheckBox('地图路径已确认 --map-verified')
        self.mission_direct = QtWidgets.QCheckBox('直控路径已确认 --direct-path-verified')
        self.mission_no_hazard = QtWidgets.QCheckBox('禁用危险品投放')
        self.mission_no_rescue = QtWidgets.QCheckBox('禁用救援降高度测试')
        grid.addWidget(self.mission_map, 1, 0, 1, 2)
        grid.addWidget(self.mission_direct, 1, 2, 1, 2)
        grid.addWidget(self.mission_no_hazard, 2, 0, 1, 2)
        grid.addWidget(self.mission_no_rescue, 2, 2, 1, 2)
        start = QtWidgets.QPushButton('启动任务脚本')
        stop = QtWidgets.QPushButton('停止任务脚本')
        stop.setEnabled(False)
        start.clicked.connect(self.start_mission)
        stop.clicked.connect(lambda: self.processes.stop('mission'))
        self.mission_state = QtWidgets.QLabel('未运行')
        grid.addWidget(start, 3, 0)
        grid.addWidget(stop, 3, 1)
        grid.addWidget(self.mission_state, 3, 2, 1, 2)
        note = QtWidgets.QLabel('执行模式会先弹窗确认，再自动向脚本输入 MISSION；'
                                '停止发送 SIGINT，脚本按自身保护逻辑处理。')
        note.setStyleSheet('color: #a33;')
        note.setWordWrap(True)
        grid.addWidget(note, 4, 0, 1, 4)
        setattr(self, 'start_mission_btn', start)
        setattr(self, 'stop_mission_btn', stop)
        layout.addWidget(group)

        group, grid = self._group('航点打点（基于当前 EKF 位姿）')
        self.point_combo = QtWidgets.QComboBox()
        self.point_combo.addItems([label for label in core.POINT_NAMES if label in self.points])
        self.point_combo.currentTextChanged.connect(self.refresh_capture_preview)
        grid.addWidget(QtWidgets.QLabel('目标航点'), 0, 0)
        grid.addWidget(self.point_combo, 0, 1)
        grid.addWidget(QtWidgets.QLabel('测量方式'), 0, 2)
        self.capture_mode = QtWidgets.QComboBox()
        self.capture_mode.addItems(['地面放置（推荐）', '空中悬停'])
        self.capture_mode.currentIndexChanged.connect(self.refresh_capture_preview)
        grid.addWidget(self.capture_mode, 0, 3)
        self.capture_live = QtWidgets.QLabel('当前 EKF：等待 /ekf/ekf_odom')
        self.capture_live.setStyleSheet('color: #555;')
        grid.addWidget(self.capture_live, 1, 0, 1, 4)
        self.capture_preview = QtWidgets.QLabel('等待 EKF 位姿…')
        self.capture_preview.setWordWrap(True)
        grid.addWidget(self.capture_preview, 2, 0, 1, 4)

        self.points_table = QtWidgets.QTableWidget(0, 6)
        self.points_table.setHorizontalHeaderLabels(
            ['航点', '配置 x', '配置 y', '配置 z', 'z_offset', 'measured'])
        self.points_table.verticalHeader().setVisible(False)
        self.points_table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.points_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.points_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.points_table.setFixedHeight(24 + 22 * max(1, len(self.points)))
        self.points_table.itemSelectionChanged.connect(self.on_points_table_select)
        grid.addWidget(self.points_table, 3, 0, 1, 4)
        self.refresh_points_table()

        self.mark_measured = QtWidgets.QCheckBox('写入时标记 measured: true')
        self.mark_measured.setChecked(True)
        grid.addWidget(self.mark_measured, 4, 0, 1, 2)
        capture = QtWidgets.QPushButton('打点并写入配置')
        capture.clicked.connect(self.capture_point)
        grid.addWidget(capture, 4, 2)
        origin_hint = QtWidgets.QLabel(
            '地面放置：把飞机放在该点地面上读 EKF，z_offset = 实测 z − 原点 z（该点地面高度差）；'
            '空中悬停：z_offset = 实测 z − 原点 z − 配置 z。写入前自动备份配置。')
        origin_hint.setStyleSheet('color: #777;')
        origin_hint.setWordWrap(True)
        grid.addWidget(origin_hint, 5, 0, 1, 4)
        layout.addWidget(group)

        group, grid = self._group('显示（可选）')
        self._component(grid, 0, '障碍地图 RViz', 'rviz_map', core.cmd_rviz_map)
        self._component(grid, 1, '里程计 / 点云 RViz', 'rviz_odom', core.cmd_rviz_odom)
        layout.addWidget(group)

        group, grid = self._group('录包（可选）')
        self.bag_raw_lidar = QtWidgets.QCheckBox('同时录原始激光 /livox/lidar（磁盘占用高）')
        grid.addWidget(self.bag_raw_lidar, 0, 0, 1, 3)
        self._component(grid, 1, '任务录包', 'run_bag',
                        lambda: core.cmd_run_bag(self.ws, raw_lidar=self.bag_raw_lidar.isChecked()))
        layout.addWidget(group)
        layout.addStretch(1)
        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(page)
        return scroll

    def _build_debug_tab(self):
        page = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(page)

        group, grid = self._group('USB 摄像头录像（640x480）')
        self.camera_device = QtWidgets.QLineEdit('/dev/video0')
        self.camera_fps = QtWidgets.QDoubleSpinBox()
        self.camera_fps.setRange(1.0, 120.0)
        self.camera_fps.setValue(30.0)
        self.camera_output = QtWidgets.QLineEdit(
            str(Path.home() / 'Videos' / ('usb_640x480_' + time.strftime('%Y%m%d_%H%M%S') + '.mp4')))
        browse = QtWidgets.QPushButton('选择…')
        browse.clicked.connect(self.choose_camera_output)
        grid.addWidget(QtWidgets.QLabel('设备'), 0, 0)
        grid.addWidget(self.camera_device, 0, 1, 1, 2)
        grid.addWidget(QtWidgets.QLabel('FPS'), 0, 3)
        grid.addWidget(self.camera_fps, 0, 4)
        grid.addWidget(QtWidgets.QLabel('输出'), 1, 0)
        grid.addWidget(self.camera_output, 1, 1, 1, 3)
        grid.addWidget(browse, 1, 4)
        self._component(grid, 2, '摄像头录像', 'camera',
                        lambda: core.cmd_camera(self.ws, self.camera_device.text().strip(),
                                                self.camera_fps.value(),
                                                self.camera_output.text().strip()),
                        'YOLO 正在用摄像头时先停 YOLO，否则设备打不开')
        layout.addWidget(group)

        group, grid = self._group('调试录包（不飞行：障碍地图、里程计、点云等）')
        self.debug_output = QtWidgets.QLineEdit(
            str(Path.home() / 'flight_logs' / ('debug_' + time.strftime('%Y%m%d_%H%M%S') + '.bag')))
        browse = QtWidgets.QPushButton('选择…')
        browse.clicked.connect(self.choose_debug_output)
        self.debug_raw_lidar = QtWidgets.QCheckBox('包含 /livox/lidar（原始点云，体积大）')
        grid.addWidget(QtWidgets.QLabel('输出'), 0, 0)
        grid.addWidget(self.debug_output, 0, 1, 1, 3)
        grid.addWidget(browse, 0, 4)
        grid.addWidget(self.debug_raw_lidar, 1, 0, 1, 3)
        self._component(grid, 2, '调试录包', 'debug_bag',
                        lambda: core.cmd_debug_bag(self.ws, self.debug_output.text().strip(),
                                                   self.debug_raw_lidar.isChecked()),
                        '话题：/Odometry、/ekf/ekf_odom、点云、/tf、Planner grid_map 等')
        layout.addWidget(group)
        layout.addStretch(1)
        return page

    # ------------------------------------------------------------- actions
    def append_log(self, name, text):
        stamp = time.strftime('%H:%M:%S')
        for line in text.splitlines():
            self.log.appendPlainText('[%s][%s] %s' % (stamp, name, line))
        if self.autoscroll.isChecked():
            self.log.verticalScrollBar().setValue(self.log.verticalScrollBar().maximum())

    def on_process_state(self, name, running, code):
        if name == 'mission':
            self.mission_state.setText('运行中' if running else ('已结束 (%d)' % code))
            self.mission_state.setStyleSheet('color: #2a7;' if running else 'color: #888;')
            self.start_mission_btn.setEnabled(not running)
            self.stop_mission_btn.setEnabled(running)
            return
        label = getattr(self, 'state_' + name, None)
        if label is not None:
            label.setText('运行中' if running else ('已结束 (%d)' % code))
            label.setStyleSheet('color: #2a7;' if running else 'color: #888;')
        start = getattr(self, 'start_' + name, None)
        stop = getattr(self, 'stop_' + name, None)
        if start is not None:
            start.setEnabled(not running)
        if stop is not None:
            stop.setEnabled(running)

    def start_control_stack(self):
        self.append_log('console', '一键启动：PX4Ctrl → Diff-Planner → YOLO → 舵机\n')
        self.processes.start('px4ctrl', core.cmd_px4ctrl(self.ws))
        QtCore.QTimer.singleShot(1200, lambda: self.processes.start(
            'planner', core.cmd_planner(self.ws)))
        QtCore.QTimer.singleShot(2400, lambda: self.processes.start(
            'yolo', core.cmd_yolo(self.ws, self.yolo_window.isChecked())))
        QtCore.QTimer.singleShot(3600, lambda: self.processes.start(
            'servo', core.cmd_servo(self.ws)))

    def stop_control_stack(self):
        for name in ('servo', 'yolo', 'planner', 'px4ctrl'):
            self.processes.stop(name)

    def start_mission(self):
        execute = self.mission_mode.currentIndex() == 1
        no_hazard = self.mission_no_hazard.isChecked()
        no_rescue = self.mission_no_rescue.isChecked()
        if execute:
            if not (self.mission_map.isChecked() and self.mission_direct.isChecked()):
                QtWidgets.QMessageBox.warning(
                    self, '缺少人工确认',
                    '执行模式必须同时勾选"地图路径已确认"和"直控路径已确认"。')
                return
            answer = QtWidgets.QMessageBox.question(
                self, '确认执行任务',
                '即将以 --execute 启动任务脚本：\n'
                '- 会切换到 OFFBOARD、解锁并发送 TAKEOFF\n'
                '- 预检通过后自动输入 MISSION\n\n确认场地、人员、降落区都已检查？',
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                QtWidgets.QMessageBox.No)
            if answer != QtWidgets.QMessageBox.Yes:
                return
        command = core.cmd_mission(
            self.ws, execute=execute,
            map_verified=self.mission_map.isChecked(),
            direct_verified=self.mission_direct.isChecked(),
            no_hazard=no_hazard, no_rescue=no_rescue)
        self.processes.start('mission', command)
        if execute:
            process = self.processes.processes.get('mission')
            if process is not None:
                QtCore.QTimer.singleShot(1500, lambda: self._feed_mission_confirm(process))

    def _feed_mission_confirm(self, process):
        if process.state() != QtCore.QProcess.NotRunning:
            process.write(b'MISSION\n')
            self.append_log('mission', '已自动输入 MISSION 确认\n')

    def on_ros_status(self, connected, text):
        self.status_ros.setText('ROS：%s' % text)
        self.status_ros.setStyleSheet('color: #2a7;' if connected else 'color: #a33;')

    def on_pose(self, x, y, z, age):
        self.status_pose.setText('EKF：x=%.3f y=%.3f z=%.3f（%.0f ms）' % (x, y, z, max(0.0, age) * 1000))

    def on_mission(self, payload):
        origin = self.ros.origin
        self.statusBar().showMessage('任务阶段：%s；原点：%s' % (
            payload.get('stage', ''), '未知' if origin is None else '[%.3f, %.3f, %.3f]' % tuple(origin)))

    def current_origin(self):
        ros = getattr(self, 'ros', None)
        if ros is not None and ros.origin is not None:
            return tuple(ros.origin), '任务状态 origin_world'
        return (0.0, 0.0, 0.0), '未收到任务状态，按 (0,0,0)'

    def compute_capture(self):
        label = self.point_combo.currentText()
        if not label:
            raise ValueError('没有可打点的航点')
        ros = getattr(self, 'ros', None)
        if ros is None or ros.pose is None:
            raise ValueError('还没有收到 /ekf/ekf_odom')
        x, y, z, age = ros.pose[:4]
        if age > 1.0:
            raise ValueError('EKF 位姿已过期 %.1fs，先检查定位链' % age)
        origin, source = self.current_origin()
        point = self.points[label]
        mode = core.CAPTURE_GROUND if self.capture_mode.currentIndex() == 0 else core.CAPTURE_HOVER
        values = core.capture_values(point, (x, y, z), origin, mode=mode)
        values.update(label=label, origin=origin, origin_source=source, age=age,
                      configured=point)
        return values

    def refresh_points_table(self):
        labels = [label for label in core.POINT_NAMES if label in self.points]
        self.points_table.setRowCount(len(labels))
        for row, label in enumerate(labels):
            point = self.points[label]
            values = [label, '%.3f' % point['xyz'][0], '%.3f' % point['xyz'][1],
                      '%.3f' % point['xyz'][2], '%+.3f' % point['z_offset'],
                      'true' if point.get('measured', True) else 'false']
            for column, text in enumerate(values):
                item = QtWidgets.QTableWidgetItem(text)
                if column == 0:
                    item.setData(QtCore.Qt.UserRole, label)
                self.points_table.setItem(row, column, item)

    def on_points_table_select(self):
        rows = self.points_table.selectionModel().selectedRows()
        if not rows:
            return
        item = self.points_table.item(rows[0].row(), 0)
        if item is not None:
            self.point_combo.setCurrentText(item.data(QtCore.Qt.UserRole))

    def refresh_capture_preview(self):
        ros = getattr(self, 'ros', None)
        if ros is not None and ros.pose is not None:
            x, y, z, age = ros.pose[:4]
            origin, source = self.current_origin()
            self.capture_live.setText(
                '当前 EKF  x=%.3f  y=%.3f  z=%.3f（%.0f ms）   原点 [%.3f, %.3f, %.3f]  %s'
                % (x, y, z, max(0.0, age) * 1000.0, origin[0], origin[1], origin[2], source))
        else:
            self.capture_live.setText('当前 EKF：等待 /ekf/ekf_odom')
        try:
            values = self.compute_capture()
        except ValueError as error:
            self.capture_preview.setText(str(error))
            return
        self.last_capture = values
        if values.get('mode') == core.CAPTURE_GROUND:
            result = 'z_offset=%.3f（该点地面高度，配置 z=%.3f）' % (values['z_offset'],
                                                                     values['configured_z'])
        else:
            result = 'z_offset=%.3f（实测高度 − 配置 z=%.3f）' % (values['z_offset'],
                                                                 values['configured_z'])
        self.capture_preview.setText(
            '%s：x=%.3f  y=%.3f  z(相对原点)=%.3f  →  %s  原点来源：%s（位姿 %.0f ms 前）'
            % (values['label'], values['x'], values['y'], values['z'], result,
               values['origin_source'], values['age'] * 1000))

    def refresh_pose_labels(self):
        if self.tabs.currentIndex() == 0 and self.point_combo.count():
            self.refresh_capture_preview()

    def capture_point(self):
        try:
            values = self.compute_capture()
        except ValueError as error:
            QtWidgets.QMessageBox.warning(self, '无法打点', str(error))
            return
        label = values['label']
        measured = self.mark_measured.isChecked()
        if values.get('mode') == core.CAPTURE_GROUND:
            detail = '测量方式：地面放置（z_offset 即该点地面相对起飞点的高度）\n'
        else:
            detail = '测量方式：空中悬停（z_offset = 实测高度 − 配置 z）\n'
        answer = QtWidgets.QMessageBox.question(
            self, '确认写入航点 %s' % label,
            '将写入 %s：\n\n'
            'x = %.3f\n'
            'y = %.3f\n'
            'z_offset = %+.3f（配置 z=%.3f）\n'
            '%s'
            'measured = %s\n\n'
            '原点来源：%s\n'
            '写入前会自动备份 hybrid_mission.yaml。确认？'
            % (label, values['x'], values['y'], values['z_offset'], values['configured_z'],
               detail, 'true' if measured else '保持原样', values['origin_source']),
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No)
        if answer != QtWidgets.QMessageBox.Yes:
            return
        try:
            text = self.config_path.read_text(encoding='utf-8')
            updated = core.update_point_line(text, label, values['x'], values['y'],
                                             values['z_offset'], measured=measured)
            backup = core.backup_config(self.config_path)
            self.config_path.write_text(updated, encoding='utf-8')
        except (OSError, ValueError) as error:
            QtWidgets.QMessageBox.critical(self, '写入失败', str(error))
            return
        self.points = core.load_points(self.config_path)
        self.refresh_points_table()
        self.append_log('waypoint', '%s 已写入：x=%.3f y=%.3f z_offset=%+.3f；备份 %s'
                        % (label, values['x'], values['y'], values['z_offset'], backup))
        QtWidgets.QMessageBox.information(self, '已写入', '航点 %s 已更新。' % label)

    def choose_camera_output(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, '选择视频文件', self.camera_output.text(), '视频 (*.mp4 *.avi)')
        if path:
            self.camera_output.setText(path)

    def choose_debug_output(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, '选择 bag 文件', self.debug_output.text(), 'ROS bag (*.bag)')
        if path:
            self.debug_output.setText(path)

    def closeEvent(self, event):
        running = [name for name in self.processes.processes if self.processes.running(name)]
        if running:
            answer = QtWidgets.QMessageBox.question(
                self, '仍有进程在运行',
                '以下进程仍在运行：%s\n\n关闭界面会向它们发送 SIGINT。确认退出？'
                % '、'.join(running),
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                QtWidgets.QMessageBox.No)
            if answer != QtWidgets.QMessageBox.Yes:
                event.ignore()
                return
        self.processes.stop_all()
        self.ros.stop()
        self.ros.wait(2000)
        event.accept()


def main(argv=None):
    app = QtWidgets.QApplication(argv or sys.argv)
    window = Console()
    window.show()
    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
