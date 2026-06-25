#!/bin/bash
set -e

if [ -z "$1" ]; then
  echo "Usage: start.sh <robot_ip> [comport]"
  exit 1
fi

ROBOT_IP="$1"
COMPORT="${2:-/dev/ttyUSB0}"

ros2 launch waypoint_collector waypoint_collector.launch.py \
  robot_ip:="$ROBOT_IP" \
  comport:="$COMPORT" &
LAUNCH_PID=$!

# Kill the launch when this script exits (Ctrl+C, rqt closed, etc.)
trap "kill $LAUNCH_PID 2>/dev/null; wait $LAUNCH_PID 2>/dev/null" EXIT INT TERM

rqt --standalone waypoint_collector
