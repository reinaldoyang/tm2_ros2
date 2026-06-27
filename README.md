## Waypoint Collector & Gripper Driver

This repository extends the Techman Robot ROS2 driver with packages for teleoperated waypoint collection and trajectory replay using a Robotiq 85 gripper and Intel RealSense camera.

This method is a 2 stage data collection pipeline for Techman robot for robot learning research. First stage is to collect waypoints, second stage is to replay those waypoints and have another process record the image and robot states at certain frequency.

### Added packages

| Package | Description |
|---|---|
| `src/waypoint_collector` | rqt GUI plugin for recording and replaying Cartesian waypoints |
| `src/robotiq_85_driver` | ROS2 driver for the Robotiq 85 gripper (RS-485 Modbus RTU) |
| `src/robotiq_85_msgs` | Message/service definitions for the gripper |
| `src/robotiq_85_description` | URDF description of the Robotiq 85 gripper |

---

### Hardware requirements

- Techman Robot (TM5S / TM7S / TM12S / TM14S or similar), connected via Ethernet
- Robotiq 85 gripper, connected via RS-485 USB adapter
- Intel RealSense camera (D435 or similar)

---

### Step 1 — Configure the TM robot (TMflow)

Before ROS2 can send commands to the robot, the robot must be running a TMflow project that includes a **Listen Node** as its entry point.

1. Create listen node loop on TMflow
2. Make sure to play the flow

If the project is not running, `tm_driver` will connect but the robot will not accept motion commands.

Make sure to wait until the listen node is properly established before sending commands.
---

### Step 2 — System dependencies

**ROS 2 Humble** must already be installed. Then install the following:

```bash
# RealSense SDK (required before the ROS wrapper)

# ROS and Python dependencies
sudo apt install -y \
  ros-humble-rqt \
  ros-humble-rqt-gui \
  ros-humble-cv-bridge \
  ros-humble-realsense2-camera \
  python3-opencv \
  python3-serial
```

#### USB serial port permissions

The gripper uses an RS-485 USB adapter. Add your user to the `dialout` group or the driver will fail with a permission error:

```bash
sudo usermod -aG dialout $USER
```

**Log out and log back in** for this to take effect. Verify with `groups | grep dialout`.

---

### Step 3 — Clone and build

Clone the repo into your ROS2 workspace src directory, then build from the workspace root:

```bash
mkdir -p ~/tm_ws
cd ~/tm_ws
git clone https://github.com/reinaldoyang/tm2_ros2.git tm2_ros2
cd tm2_ros2

# Install remaining ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build

# Source
source install/setup.bash
```

---

### Step 4 — Identify the gripper serial port

Plug in the RS-485 USB adapter and find the device name:

```bash
ls /dev/ttyUSB*
```

Typically `/dev/ttyUSB0` if it is the only USB serial device. If you have other USB serial devices connected, it may be `/dev/ttyUSB1` or higher.

---

### Step 5 — Launch

```bash
source /opt/ros/humble/setup.bash
source ~/tm_ws/tm2_ros2/install/setup.bash

# Replace 192.168.10.2 with your robot's IP
./src/waypoint_collector/scripts/start.sh 192.168.10.2
```

This launches three nodes:
- `tm_driver` — connects to the TM robot over Ethernet
- `robotiq_85_driver` — connects to the gripper over the specified serial port
- `realsense2_camera_node` — streams color images at 5 Hz
- `rqt` — opens the Waypoint Collector GUI

#### Verify connections

After launching, check that all three nodes are running without errors in the terminal output:
- `tm_driver` should print something like `TM_ROS: connected`
- `robotiq_85_driver` should print gripper status without serial errors
- If the RealSense camera is not connected, the camera node will error but waypoint collection still works

During the first stage of waypoint collection, it's normal if the driver for the gripper and the robot to have some warning or errors, they can still move properly.
---

### Waypoint Collector GUI

| Feature | Shortcut | Description |
|---|---|---|
| **Log Waypoint** | Space | Saves current robot pose + gripper state to `waypoints.csv` |
| **Start New Trajectory** | — | Opens a new numbered trajectory folder |
| **Gripper Open / Close** | — | Manually controls the gripper |
| **Run Trajectory** | — | Replays a recorded trajectory on the robot |
| **Run & Record** | — | Replays while recording pose+gripper at 5 Hz and camera images to `images/` |

---

### Output data format

Recorded data is saved to `~/tm_ws/tm2_ros2/data/` (excluded from git).

Each trajectory is saved in its own numbered folder:

```
data/
  trajectory_001/
    waypoints.csv       # manually logged waypoints
    trajectory_log.csv  # auto-recorded during Run & Record (5 Hz)
    images/             # camera frames recorded during Run & Record
```

Both CSV files share the same format:

```
timestamp,state
1782446262.045,"[x, y, z, rx, ry, rz, gripper]"
```

- `x y z` — TCP position in metres
- `rx ry rz` — TCP orientation in degrees (Euler)
- `gripper` — `1` = closed, `0` = open
