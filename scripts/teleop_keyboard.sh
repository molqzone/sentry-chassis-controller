#!/usr/bin/env bash
set -euo pipefail

if [ ! -t 0 ] || [ ! -t 1 ]; then
  echo "[teleop_keyboard] This command must run in an interactive terminal (TTY)." >&2
  exit 1
fi

source "/opt/ros/noetic/setup.bash"
if [ -f "/workspace/devel/setup.bash" ]; then
  source "/workspace/devel/setup.bash"
elif [ -f "${PWD}/devel/setup.bash" ]; then
  source "${PWD}/devel/setup.bash"
fi

cmd_vel_topic="${CMD_VEL_TOPIC:-/cmd_vel}"
teleop_speed="${TELEOP_SPEED:-0.5}"
teleop_turn="${TELEOP_TURN:-1.0}"
teleop_repeat_rate="${TELEOP_REPEAT_RATE:-20.0}"
teleop_key_timeout="${TELEOP_KEY_TIMEOUT:-0.6}"

echo "[teleop_keyboard] Publishing Twist to ${cmd_vel_topic}"
echo "[teleop_keyboard] speed=${teleop_speed}, turn=${teleop_turn}, repeat_rate=${teleop_repeat_rate}, key_timeout=${teleop_key_timeout}"

exec rosrun teleop_twist_keyboard teleop_twist_keyboard.py \
  _speed:="${teleop_speed}" \
  _turn:="${teleop_turn}" \
  _repeat_rate:="${teleop_repeat_rate}" \
  _key_timeout:="${teleop_key_timeout}" \
  cmd_vel:="${cmd_vel_topic}"
