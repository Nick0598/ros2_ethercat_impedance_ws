#!/usr/bin/env bash
# Open-loop torque step test for single-motor CST bringup.
# Prerequisite: ros2 launch trunk_impedance_control single_motor_bringup.launch.py use_effort_controller:=true

set -euo pipefail

TOPIC="${1:-/effort_controller/commands}"
TORQUES=(0.0 0.05 0.1 -0.05 0.0)
HOLD_SEC="${HOLD_SEC:-2}"

echo "Publishing torque steps to ${TOPIC} (hold ${HOLD_SEC}s each)"
for tau in "${TORQUES[@]}"; do
  echo "  tau = ${tau} Nm"
  ros2 topic pub --once "${TOPIC}" std_msgs/msg/Float64MultiArray "{data: [${tau}]}"
  sleep "${HOLD_SEC}"
done
echo "Done. Check: ros2 topic echo /joint_states --once"
