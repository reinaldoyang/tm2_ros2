import csv
import math
import os
import threading
import time

import cv2
from cv_bridge import CvBridge
from python_qt_binding.QtCore import Qt, QTimer, pyqtSignal
from python_qt_binding.QtGui import QFont
from python_qt_binding.QtWidgets import (
    QComboBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from geometry_msgs.msg import PoseStamped
from robotiq_85_msgs.msg import GripperCmd, GripperStat
from sensor_msgs.msg import Image
from tm_msgs.srv import SetPositions

_cv_bridge = CvBridge()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _quat_to_euler_deg(x, y, z, w):
    """Return (roll, pitch, yaw) in degrees from a unit quaternion."""
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def _gripper_state_str(msg: GripperStat) -> str:
    if not msg.is_ready:
        return 'NOT READY'
    if msg.is_moving:
        return 'MOVING'
    if msg.obj_detected:
        return 'HOLDING'
    return 'OPEN' if msg.position > 0.04 else 'CLOSED'


def _find_latest_traj_num(data_dir: str) -> int:
    max_num = 0
    try:
        for name in os.listdir(data_dir):
            suffix = name[len('trajectory_'):]
            if name.startswith('trajectory_') and suffix.isdigit():
                max_num = max(max_num, int(suffix))
    except OSError:
        pass
    return max_num


_CSV_HEADER = ['timestamp', 'state']
_DEFAULT_DATA_DIR = os.path.join(
    os.path.expanduser('~'), 'tm_ws', 'tm2_ros2', 'data'
)

# ---------------------------------------------------------------------------
# Widget
# ---------------------------------------------------------------------------

_GRIPPER_OPEN_POS = 0.085
_GRIPPER_SPEED = 0.1   # m/s (max)
_GRIPPER_FORCE = 50.0  # N
_GRIPPER_CMD_TOPIC = '/gripper/cmd'

_EXEC_VELOCITY = 0.40        # m/s Cartesian speed during playback
_EXEC_ACC_TIME = 200.0       # ms acceleration time
_EXEC_ARRIVE_TOL = 0.005     # 5 mm position tolerance for arrival detection
_EXEC_ARRIVE_TIMEOUT = 30.0  # seconds before giving up on arrival
_EXEC_GRIPPER_SETTLE = 0.3   # seconds to let gripper start moving


class WaypointCollectorWidget(QWidget):
    # Cross-thread signals — callbacks arrive on the ROS executor thread
    _sig_pose = pyqtSignal(object)
    _sig_gripper = pyqtSignal(object)
    _sig_exec_status = pyqtSignal(str, bool)  # (message, is_error)
    _sig_record_status = pyqtSignal(str, bool)

    def __init__(self, node):
        super().__init__()
        self._node = node
        self._pose_msg = None
        self._gripper_msg = None
        self._csv_file = None
        self._csv_writer = None
        self._waypoint_count = 0
        self._current_traj_folder = None
        self._data_dir = _DEFAULT_DATA_DIR
        self._last_gripper_cmd_pos = None
        self._exec_waypoints = []
        self._exec_thread = None
        self._exec_stop_event = threading.Event()
        self._camera_img_msg = None
        self._recording_active = threading.Event()
        self._record_thread = None
        self._build_ui()
        self._connect_signals()
        self._create_subscriptions()
        self._create_gripper_publisher()
        self._create_set_positions_client()

        # Status auto-clear after 4 s
        self._status_timer = QTimer(self)
        self._status_timer.setSingleShot(True)
        self._status_timer.timeout.connect(lambda: self._status_lbl.setText(''))

        # Open or create trajectory on startup
        self._initialize_trajectory()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self):
        self.setWindowTitle('Waypoint Collector')
        root = QVBoxLayout(self)
        root.setSpacing(8)
        root.setContentsMargins(10, 10, 10, 10)

        root.addWidget(self._build_pose_group())
        root.addWidget(self._build_gripper_group())
        root.addWidget(self._build_gripper_control_group())
        root.addWidget(self._build_status_group())
        root.addWidget(self._build_trajectory_exec_group())
        root.addLayout(self._build_buttons())
        root.addWidget(self._build_status_bar())

        self.setMinimumWidth(440)

    def _build_pose_group(self):
        group = QGroupBox('Current Pose')
        grid = QGridLayout(group)
        grid.setColumnStretch(1, 1)
        grid.setColumnStretch(3, 1)

        mono = QFont('Monospace')
        mono.setStyleHint(QFont.Monospace)

        self._pose_vals = {}
        fields = [
            ('X', 'm'), ('Y', 'm'), ('Z', 'm'),
            ('Rx', '°'), ('Ry', '°'), ('Rz', '°'),
        ]
        for i, (name, unit) in enumerate(fields):
            row, col = divmod(i, 2)
            grid.addWidget(QLabel(f'  {name} ({unit}):'), row, col * 2)
            lbl = QLabel('—')
            lbl.setFont(mono)
            lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            lbl.setMinimumWidth(85)
            grid.addWidget(lbl, row, col * 2 + 1)
            self._pose_vals[name] = lbl

        self._pose_no_data_lbl = QLabel('Waiting for /tool_pose …')
        self._pose_no_data_lbl.setAlignment(Qt.AlignCenter)
        self._pose_no_data_lbl.setStyleSheet('color: gray; font-style: italic;')
        grid.addWidget(self._pose_no_data_lbl, 3, 0, 1, 4)

        return group

    def _build_gripper_group(self):
        group = QGroupBox('Gripper State  (/robotiq_gripper/state)')
        grid = QGridLayout(group)
        grid.setColumnStretch(1, 1)

        grid.addWidget(QLabel('  State:'), 0, 0)
        self._gripper_state_lbl = QLabel('—')
        self._gripper_state_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        bold = QFont()
        bold.setBold(True)
        self._gripper_state_lbl.setFont(bold)
        grid.addWidget(self._gripper_state_lbl, 0, 1)

        grid.addWidget(QLabel('  Position (mm):'), 1, 0)
        self._gripper_pos_lbl = QLabel('—')
        self._gripper_pos_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        grid.addWidget(self._gripper_pos_lbl, 1, 1)

        self._gripper_no_data_lbl = QLabel('Waiting for /robotiq_gripper/state …')
        self._gripper_no_data_lbl.setAlignment(Qt.AlignCenter)
        self._gripper_no_data_lbl.setStyleSheet('color: gray; font-style: italic;')
        grid.addWidget(self._gripper_no_data_lbl, 2, 0, 1, 2)

        return group

    def _build_gripper_control_group(self):
        group = QGroupBox(f'Gripper Control  ({_GRIPPER_CMD_TOPIC})')
        layout = QVBoxLayout(group)
        layout.setSpacing(6)

        # Open button
        self._gripper_open_btn = QPushButton(f'Open  ({_GRIPPER_OPEN_POS * 1000:.0f} mm)')
        self._gripper_open_btn.setMinimumHeight(36)
        self._gripper_open_btn.setStyleSheet(
            'QPushButton { background-color: #1565c0; color: white; border-radius: 4px; }'
            'QPushButton:hover { background-color: #1976d2; }'
            'QPushButton:pressed { background-color: #0d47a1; }'
            'QPushButton:disabled { background-color: #aaa; }'
        )
        layout.addWidget(self._gripper_open_btn)

        # Close row: label + input + button
        close_row = QHBoxLayout()
        close_row.addWidget(QLabel('Close position (mm):'))
        self._close_pos_input = QLineEdit('0')
        self._close_pos_input.setFixedWidth(80)
        self._close_pos_input.setPlaceholderText('0–85')
        close_row.addWidget(self._close_pos_input)
        self._gripper_close_btn = QPushButton('Send')
        self._gripper_close_btn.setFixedWidth(60)
        self._gripper_close_btn.setStyleSheet(
            'QPushButton { background-color: #b71c1c; color: white; border-radius: 4px; }'
            'QPushButton:hover { background-color: #c62828; }'
            'QPushButton:pressed { background-color: #7f0000; }'
            'QPushButton:disabled { background-color: #aaa; }'
        )
        close_row.addWidget(self._gripper_close_btn)
        close_row.addStretch()
        layout.addLayout(close_row)

        # Action feedback label
        self._action_status_lbl = QLabel('—')
        self._action_status_lbl.setAlignment(Qt.AlignCenter)
        self._action_status_lbl.setStyleSheet('color: #555; font-style: italic;')
        layout.addWidget(self._action_status_lbl)

        return group

    def _build_status_group(self):
        group = QGroupBox('Recording Status')
        grid = QGridLayout(group)
        grid.setColumnStretch(1, 1)

        grid.addWidget(QLabel('  Trajectory:'), 0, 0)
        self._traj_lbl = QLabel('—')
        self._traj_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        grid.addWidget(self._traj_lbl, 0, 1)

        grid.addWidget(QLabel('  Waypoints logged:'), 1, 0)
        self._count_lbl = QLabel('0')
        self._count_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        bold = QFont()
        bold.setBold(True)
        self._count_lbl.setFont(bold)
        grid.addWidget(self._count_lbl, 1, 1)

        grid.addWidget(QLabel('  Data directory:'), 2, 0)
        self._dir_lbl = QLabel(self._data_dir)
        self._dir_lbl.setWordWrap(True)
        self._dir_lbl.setStyleSheet('color: #555;')
        grid.addWidget(self._dir_lbl, 2, 1)

        # Change dir button
        dir_btn = QPushButton('Change…')
        dir_btn.setFixedWidth(70)
        dir_btn.clicked.connect(self._on_change_dir)
        grid.addWidget(dir_btn, 2, 2)

        return group

    def _build_trajectory_exec_group(self):
        group = QGroupBox('Trajectory Execution')
        layout = QVBoxLayout(group)
        layout.setSpacing(6)

        # Dropdown + Load row
        load_row = QHBoxLayout()
        self._traj_select_combo = QComboBox()
        self._traj_select_combo.setMinimumWidth(160)
        load_row.addWidget(self._traj_select_combo)
        self._traj_refresh_btn = QPushButton('↻')
        self._traj_refresh_btn.setFixedWidth(30)
        self._traj_refresh_btn.setToolTip('Refresh trajectory list')
        load_row.addWidget(self._traj_refresh_btn)
        self._traj_load_btn = QPushButton('Load')
        self._traj_load_btn.setFixedWidth(60)
        load_row.addWidget(self._traj_load_btn)
        self._traj_loaded_lbl = QLabel('No trajectory loaded')
        self._traj_loaded_lbl.setStyleSheet('color: #555; font-style: italic;')
        load_row.addWidget(self._traj_loaded_lbl)
        load_row.addStretch()
        layout.addLayout(load_row)

        # Run / Stop row
        run_row = QHBoxLayout()
        self._traj_run_btn = QPushButton('Run Trajectory')
        self._traj_run_btn.setMinimumHeight(36)
        self._traj_run_btn.setEnabled(False)
        self._traj_run_btn.setStyleSheet(
            'QPushButton { background-color: #4a148c; color: white; border-radius: 4px; }'
            'QPushButton:hover { background-color: #6a1b9a; }'
            'QPushButton:pressed { background-color: #311b92; }'
            'QPushButton:disabled { background-color: #aaa; }'
        )
        self._traj_run_record_btn = QPushButton('Run && Record')
        self._traj_run_record_btn.setMinimumHeight(36)
        self._traj_run_record_btn.setEnabled(False)
        self._traj_run_record_btn.setStyleSheet(
            'QPushButton { background-color: #e65100; color: white; border-radius: 4px; }'
            'QPushButton:hover { background-color: #ef6c00; }'
            'QPushButton:pressed { background-color: #bf360c; }'
            'QPushButton:disabled { background-color: #aaa; }'
        )
        self._traj_stop_btn = QPushButton('Stop')
        self._traj_stop_btn.setFixedWidth(70)
        self._traj_stop_btn.setMinimumHeight(36)
        self._traj_stop_btn.setEnabled(False)
        self._traj_stop_btn.setStyleSheet(
            'QPushButton { background-color: #b71c1c; color: white; border-radius: 4px; }'
            'QPushButton:hover { background-color: #c62828; }'
            'QPushButton:disabled { background-color: #aaa; }'
        )
        run_row.addWidget(self._traj_run_btn)
        run_row.addWidget(self._traj_run_record_btn)
        run_row.addWidget(self._traj_stop_btn)
        layout.addLayout(run_row)

        # Progress label
        self._exec_status_lbl = QLabel('—')
        self._exec_status_lbl.setAlignment(Qt.AlignCenter)
        self._exec_status_lbl.setStyleSheet('color: #555; font-style: italic;')
        layout.addWidget(self._exec_status_lbl)

        return group

    def _build_buttons(self):
        layout = QHBoxLayout()
        layout.setSpacing(10)

        self._log_btn = QPushButton('Log Waypoint  [Space]')
        self._log_btn.setMinimumHeight(44)
        self._log_btn.setStyleSheet(
            'QPushButton { font-size: 13px; font-weight: bold; background-color: #2e7d32; color: white; border-radius: 4px; }'
            'QPushButton:hover { background-color: #388e3c; }'
            'QPushButton:pressed { background-color: #1b5e20; }'
            'QPushButton:disabled { background-color: #aaa; }'
        )
        self._log_btn.setShortcut('Space')

        self._new_traj_btn = QPushButton('Start New Trajectory')
        self._new_traj_btn.setMinimumHeight(44)
        self._new_traj_btn.setStyleSheet(
            'QPushButton { font-size: 13px; background-color: #1565c0; color: white; border-radius: 4px; }'
            'QPushButton:hover { background-color: #1976d2; }'
            'QPushButton:pressed { background-color: #0d47a1; }'
        )

        layout.addWidget(self._log_btn)
        layout.addWidget(self._new_traj_btn)
        return layout

    def _build_status_bar(self):
        self._status_lbl = QLabel('')
        self._status_lbl.setAlignment(Qt.AlignCenter)
        self._status_lbl.setStyleSheet('color: #444; font-style: italic;')
        self._status_lbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        return self._status_lbl

    # ------------------------------------------------------------------
    # Signal / subscription wiring
    # ------------------------------------------------------------------

    def _connect_signals(self):
        self._sig_pose.connect(self._on_pose_update)
        self._sig_gripper.connect(self._on_gripper_update)
        self._sig_exec_status.connect(self._on_exec_status)
        self._sig_record_status.connect(self._on_exec_status)
        self._log_btn.clicked.connect(self._on_log_waypoint)
        self._new_traj_btn.clicked.connect(self._on_start_new_trajectory)
        self._gripper_open_btn.clicked.connect(self._on_open_gripper)
        self._gripper_close_btn.clicked.connect(self._on_send_close)
        self._close_pos_input.returnPressed.connect(self._on_send_close)
        self._traj_refresh_btn.clicked.connect(self._populate_traj_dropdown)
        self._traj_load_btn.clicked.connect(self._on_load_trajectory)
        self._traj_run_btn.clicked.connect(self._on_run_trajectory)
        self._traj_run_record_btn.clicked.connect(self._on_run_trajectory_and_record)
        self._traj_stop_btn.clicked.connect(self._on_stop_trajectory)

    def _create_subscriptions(self):
        self._pose_sub = self._node.create_subscription(
            PoseStamped, '/tool_pose', self._pose_cb, 10
        )
        self._gripper_sub = self._node.create_subscription(
            GripperStat, '/robotiq_gripper/state', self._gripper_cb, 10
        )
        self._camera_sub = self._node.create_subscription(
            Image, '/camera/camera/color/image_raw', self._camera_cb, 1
        )

    def _create_gripper_publisher(self):
        self._gripper_cmd_pub = self._node.create_publisher(
            GripperCmd, _GRIPPER_CMD_TOPIC, 10
        )

    def _create_set_positions_client(self):
        self._set_pos_client = self._node.create_client(SetPositions, 'set_positions')
        self._populate_traj_dropdown()

    # ------------------------------------------------------------------
    # ROS callbacks (executor thread → emit signal to GUI thread)
    # ------------------------------------------------------------------

    def _pose_cb(self, msg: PoseStamped):
        self._pose_msg = msg
        self._sig_pose.emit(msg)

    def _gripper_cb(self, msg: GripperStat):
        self._gripper_msg = msg
        self._sig_gripper.emit(msg)

    def _camera_cb(self, msg: Image):
        self._camera_img_msg = msg

    # ------------------------------------------------------------------
    # GUI slots (Qt main thread)
    # ------------------------------------------------------------------

    def _on_pose_update(self, msg: PoseStamped):
        p = msg.pose.position
        q = msg.pose.orientation
        rx, ry, rz = _quat_to_euler_deg(q.x, q.y, q.z, q.w)

        self._pose_vals['X'].setText(f'{p.x:.4f}')
        self._pose_vals['Y'].setText(f'{p.y:.4f}')
        self._pose_vals['Z'].setText(f'{p.z:.4f}')
        self._pose_vals['Rx'].setText(f'{rx:.2f}')
        self._pose_vals['Ry'].setText(f'{ry:.2f}')
        self._pose_vals['Rz'].setText(f'{rz:.2f}')
        self._pose_no_data_lbl.hide()

    def _on_gripper_update(self, msg: GripperStat):
        state = _gripper_state_str(msg)
        colors = {'OPEN': '#1b5e20', 'CLOSED': '#b71c1c', 'HOLDING': '#e65100', 'MOVING': '#f57f17', 'NOT READY': '#888'}
        color = colors.get(state, '#333')
        self._gripper_state_lbl.setText(state)
        self._gripper_state_lbl.setStyleSheet(f'color: {color}; font-weight: bold;')
        self._gripper_pos_lbl.setText(f'{msg.position * 1000:.1f}')
        self._gripper_no_data_lbl.hide()

    def _on_log_waypoint(self):
        if self._pose_msg is None:
            self._set_status('No pose yet — /tool_pose not received', error=True)
            return
        if self._csv_writer is None:
            self._set_status('No trajectory open', error=True)
            return

        p = self._pose_msg.pose.position
        q = self._pose_msg.pose.orientation
        rx, ry, rz = _quat_to_euler_deg(q.x, q.y, q.z, q.w)

        if self._last_gripper_cmd_pos is None or self._last_gripper_cmd_pos == _GRIPPER_OPEN_POS:
            gripper = 0
        else:
            gripper = 1

        timestamp = f'{time.time():.3f}'
        state = (
            f'[{p.x:.6f},{p.y:.6f},{p.z:.6f},'
            f'{rx:.4f},{ry:.4f},{rz:.4f},{gripper}]'
        )
        self._csv_writer.writerow([timestamp, state])
        self._csv_file.flush()

        self._waypoint_count += 1
        self._count_lbl.setText(str(self._waypoint_count))
        self._set_status(f'Waypoint #{self._waypoint_count} logged')

    def _on_start_new_trajectory(self):
        next_num = _find_latest_traj_num(self._data_dir) + 1
        self._open_trajectory(next_num, new=True)

    def _on_change_dir(self):
        chosen = QFileDialog.getExistingDirectory(
            self, 'Select data directory', self._data_dir
        )
        if chosen:
            self.set_data_dir(chosen)

    # ------------------------------------------------------------------
    # Trajectory execution slots
    # ------------------------------------------------------------------

    def _populate_traj_dropdown(self):
        self._traj_select_combo.clear()
        try:
            folders = sorted(
                n for n in os.listdir(self._data_dir)
                if n.startswith('trajectory_') and n[len('trajectory_'):].isdigit()
            )
        except OSError:
            folders = []
        for f in folders:
            self._traj_select_combo.addItem(f)
        if not folders:
            self._traj_select_combo.addItem('(no trajectories found)')

    def _on_load_trajectory(self):
        folder = self._traj_select_combo.currentText()
        if not folder or folder.startswith('('):
            return
        csv_path = os.path.join(self._data_dir, folder, 'waypoints.csv')
        if not os.path.isfile(csv_path):
            self._set_exec_status(f'No waypoints.csv in {folder}', error=True)
            return

        waypoints = []
        try:
            with open(csv_path, 'r') as f:
                reader = csv.reader(f)
                next(reader)  # skip header
                for row in reader:
                    if len(row) < 2:
                        continue
                    state_str = row[1].strip().strip('"').strip('[').strip(']')
                    vals = [float(v) for v in state_str.split(',')]
                    waypoints.append({
                        'x': vals[0], 'y': vals[1], 'z': vals[2],
                        'rx': vals[3], 'ry': vals[4], 'rz': vals[5],
                        'gripper': int(vals[6]),
                    })
        except Exception as e:
            self._set_exec_status(f'Parse error: {e}', error=True)
            return

        self._exec_waypoints = waypoints
        self._traj_loaded_lbl.setText(f'{len(waypoints)} waypoints')
        self._traj_loaded_lbl.setStyleSheet('color: #1b5e20;')
        self._traj_run_btn.setEnabled(len(waypoints) > 0)
        self._traj_run_record_btn.setEnabled(len(waypoints) > 0)
        self._set_exec_status(f'Loaded {folder} — {len(waypoints)} waypoints')

    def _on_run_trajectory(self):
        if not self._exec_waypoints:
            return
        if self._exec_thread and self._exec_thread.is_alive():
            return

        self._exec_stop_event.clear()
        self._traj_run_btn.setEnabled(False)
        self._traj_run_record_btn.setEnabled(False)
        self._traj_stop_btn.setEnabled(True)
        self._traj_load_btn.setEnabled(False)
        self._log_btn.setEnabled(False)

        self._exec_thread = threading.Thread(
            target=self._trajectory_thread_fn,
            args=(list(self._exec_waypoints),),
            daemon=True,
        )
        self._exec_thread.start()

    def _on_run_trajectory_and_record(self):
        if not self._exec_waypoints:
            return
        if self._exec_thread and self._exec_thread.is_alive():
            return

        folder = self._traj_select_combo.currentText()
        self._exec_stop_event.clear()
        self._recording_active.clear()
        self._traj_run_btn.setEnabled(False)
        self._traj_run_record_btn.setEnabled(False)
        self._traj_stop_btn.setEnabled(True)
        self._traj_load_btn.setEnabled(False)
        self._log_btn.setEnabled(False)

        self._record_thread = threading.Thread(
            target=self._record_thread_fn,
            args=(folder,),
            daemon=True,
        )
        self._exec_thread = threading.Thread(
            target=self._trajectory_thread_fn,
            args=(list(self._exec_waypoints), True),
            daemon=True,
        )
        self._record_thread.start()
        self._exec_thread.start()

    def _on_stop_trajectory(self):
        self._exec_stop_event.set()
        self._set_exec_status('Stopping after current waypoint…')

    def _on_exec_status(self, msg: str, is_error: bool):
        self._set_exec_status(msg, error=is_error)
        # Re-enable buttons when done (thread finished)
        if not (self._exec_thread and self._exec_thread.is_alive()):
            self._traj_run_btn.setEnabled(bool(self._exec_waypoints))
            self._traj_run_record_btn.setEnabled(bool(self._exec_waypoints))
            self._traj_stop_btn.setEnabled(False)
            self._traj_load_btn.setEnabled(True)
            self._log_btn.setEnabled(True)

    def _set_exec_status(self, msg: str, error: bool = False):
        color = '#c62828' if error else '#4a148c'
        self._exec_status_lbl.setText(msg)
        self._exec_status_lbl.setStyleSheet(f'color: {color}; font-style: italic;')
        self._node.get_logger().info(f'[TrajectoryExec] {msg}')

    # ------------------------------------------------------------------
    # Execution thread (not the ROS executor thread)
    # ------------------------------------------------------------------

    def _trajectory_thread_fn(self, waypoints, record=False):
        total = len(waypoints)

        if not self._set_pos_client.wait_for_service(timeout_sec=3.0):
            self._sig_exec_status.emit('set_positions service not available', True)
            return

        if record:
            self._recording_active.set()

        for i, wp in enumerate(waypoints):
            if self._exec_stop_event.is_set():
                self._sig_exec_status.emit(f'Stopped at waypoint {i + 1}/{total}', False)
                return

            self._sig_exec_status.emit(f'Executing waypoint {i + 1}/{total}…', False)

            # 1. Send move command
            req = SetPositions.Request()
            req.motion_type = SetPositions.Request.PTP_T
            req.positions = [
                wp['x'], wp['y'], wp['z'],
                math.radians(wp['rx']),
                math.radians(wp['ry']),
                math.radians(wp['rz']),
            ]
            req.velocity = _EXEC_VELOCITY
            req.acc_time = _EXEC_ACC_TIME
            req.blend_percentage = 0
            req.fine_goal = False

            future = self._set_pos_client.call_async(req)
            # Wait for service response (executor is spinning separately)
            deadline = time.time() + 10.0
            while not future.done():
                if time.time() > deadline or self._exec_stop_event.is_set():
                    self._sig_exec_status.emit(
                        f'Service timeout at waypoint {i + 1}', True
                    )
                    return
                time.sleep(0.01)

            if not future.result().ok:
                self._sig_exec_status.emit(
                    f'set_positions rejected at waypoint {i + 1}', True
                )
                return

            # 2. Poll /tool_pose until within tolerance
            deadline = time.time() + _EXEC_ARRIVE_TIMEOUT
            while True:
                if self._exec_stop_event.is_set():
                    self._sig_exec_status.emit(
                        f'Stopped at waypoint {i + 1}/{total}', False
                    )
                    return
                if time.time() > deadline:
                    self._sig_exec_status.emit(
                        f'Arrival timeout at waypoint {i + 1}', True
                    )
                    return
                pose = self._pose_msg
                if pose is not None:
                    p = pose.pose.position
                    dist = math.sqrt(
                        (p.x - wp['x']) ** 2 +
                        (p.y - wp['y']) ** 2 +
                        (p.z - wp['z']) ** 2
                    )
                    if dist < _EXEC_ARRIVE_TOL:
                        break
                time.sleep(0.05)

            # 3. Send gripper command
            gripper_pos = _GRIPPER_OPEN_POS if wp['gripper'] == 0 else 0.0
            self._publish_gripper_cmd(gripper_pos)
            time.sleep(_EXEC_GRIPPER_SETTLE)

        if record:
            self._recording_active.clear()
        self._sig_exec_status.emit(f'Done — {total} waypoints executed', False)

    def _record_thread_fn(self, folder):
        log_path = os.path.join(self._data_dir, folder, 'trajectory_log.csv')
        img_dir = os.path.join(self._data_dir, folder, 'images')
        os.makedirs(img_dir, exist_ok=True)

        with open(log_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'state'])

            while not self._exec_stop_event.is_set():
                if not self._recording_active.is_set():
                    if not self._recording_active.wait(timeout=0.5):
                        if self._exec_thread and not self._exec_thread.is_alive():
                            break
                    continue

                ts = time.time()
                ts_ms = int(ts * 1000)

                pose = self._pose_msg
                if pose is not None:
                    p = pose.pose.position
                    q = pose.pose.orientation
                    rx, ry, rz = _quat_to_euler_deg(q.x, q.y, q.z, q.w)
                    gripper = 0 if (self._last_gripper_cmd_pos is None or
                                    self._last_gripper_cmd_pos == _GRIPPER_OPEN_POS) else 1
                    state = (f'[{p.x:.6f},{p.y:.6f},{p.z:.6f},'
                             f'{rx:.4f},{ry:.4f},{rz:.4f},{gripper}]')
                    writer.writerow([f'{ts:.3f}', state])
                    f.flush()

                img_msg = self._camera_img_msg
                if img_msg is not None:
                    try:
                        cv_img = _cv_bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
                        cv2.imwrite(os.path.join(img_dir, f'{ts_ms}.png'), cv_img)
                    except Exception:
                        pass

                time.sleep(0.2)  # 5 Hz

        self._sig_record_status.emit(f'Recording saved → {folder}', False)

    def _on_open_gripper(self):
        self._publish_gripper_cmd(_GRIPPER_OPEN_POS)

    def _on_send_close(self):
        text = self._close_pos_input.text().strip()
        try:
            pos_mm = float(text)
        except ValueError:
            self._set_gripper_status('Invalid input — enter a number in mm (e.g. 20)', error=True)
            return
        if not (0.0 <= pos_mm < _GRIPPER_OPEN_POS * 1000):
            self._set_gripper_status(
                f'Position must be in [0, {_GRIPPER_OPEN_POS * 1000:.0f}) mm', error=True
            )
            return
        self._publish_gripper_cmd(pos_mm / 1000.0)

    def _publish_gripper_cmd(self, position: float):
        cmd = GripperCmd()
        cmd.position = position
        cmd.speed = _GRIPPER_SPEED
        cmd.force = _GRIPPER_FORCE
        self._gripper_cmd_pub.publish(cmd)
        self._last_gripper_cmd_pos = position
        label = 'OPEN' if position == _GRIPPER_OPEN_POS else f'CLOSED @ {position * 1000:.1f} mm'
        self._set_gripper_status(f'Last cmd: {label}')

    def _set_gripper_status(self, msg: str, error: bool = False):
        color = '#c62828' if error else '#1b5e20'
        self._action_status_lbl.setText(msg)
        self._action_status_lbl.setStyleSheet(f'color: {color}; font-style: italic;')
        self._node.get_logger().info(f'[GripperControl] {msg}')

    # ------------------------------------------------------------------
    # Trajectory management
    # ------------------------------------------------------------------

    def _initialize_trajectory(self):
        os.makedirs(self._data_dir, exist_ok=True)
        latest = _find_latest_traj_num(self._data_dir)
        if latest == 0:
            self._open_trajectory(1, new=True)
        else:
            # Continue from the last trajectory
            self._open_trajectory(latest, new=False)

    def _open_trajectory(self, num: int, new: bool):
        """Open trajectory_NNN/waypoints.csv.

        new=True  → write mode (fresh CSV with header, waypoint count resets).
        new=False → append mode (continue existing; count rows if file exists).
        """
        if self._csv_file and not self._csv_file.closed:
            self._csv_file.close()

        folder = f'trajectory_{num:03d}'
        traj_path = os.path.join(self._data_dir, folder)
        os.makedirs(traj_path, exist_ok=True)
        csv_path = os.path.join(traj_path, 'waypoints.csv')

        if new:
            self._csv_file = open(csv_path, 'w', newline='')
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow(_CSV_HEADER)
            self._csv_file.flush()
            self._waypoint_count = 0
        else:
            file_existed = os.path.isfile(csv_path)
            self._csv_file = open(csv_path, 'a', newline='')
            self._csv_writer = csv.writer(self._csv_file)
            if not file_existed:
                self._csv_writer.writerow(_CSV_HEADER)
                self._csv_file.flush()
                self._waypoint_count = 0
            else:
                with open(csv_path, 'r') as f:
                    self._waypoint_count = max(0, sum(1 for _ in f) - 1)

        self._current_traj_folder = folder
        self._traj_lbl.setText(folder)
        self._count_lbl.setText(str(self._waypoint_count))
        verb = 'Created' if new else 'Resumed'
        self._set_status(f'{verb} → {csv_path}')
        self._node.get_logger().info(f'[WaypointCollector] {verb} {csv_path}')

    # ------------------------------------------------------------------
    # Public API (used by plugin for settings persistence)
    # ------------------------------------------------------------------

    @property
    def data_dir(self) -> str:
        return self._data_dir

    def set_data_dir(self, path: str):
        self._data_dir = path
        self._dir_lbl.setText(path)
        os.makedirs(path, exist_ok=True)
        self._initialize_trajectory()
        self._populate_traj_dropdown()

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def shutdown(self):
        self._exec_stop_event.set()
        self._recording_active.set()  # unblock any wait() in record thread
        if self._exec_thread and self._exec_thread.is_alive():
            self._exec_thread.join(timeout=2.0)
        if self._record_thread and self._record_thread.is_alive():
            self._record_thread.join(timeout=2.0)
        if self._csv_file and not self._csv_file.closed:
            self._csv_file.close()
        try:
            self._node.destroy_subscription(self._pose_sub)
            self._node.destroy_subscription(self._gripper_sub)
            self._node.destroy_subscription(self._camera_sub)
            self._node.destroy_publisher(self._gripper_cmd_pub)
            self._node.destroy_client(self._set_pos_client)
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _set_status(self, msg: str, error: bool = False):
        color = '#c62828' if error else '#444'
        self._status_lbl.setText(msg)
        self._status_lbl.setStyleSheet(f'color: {color}; font-style: italic;')
        self._status_timer.start(4000)
