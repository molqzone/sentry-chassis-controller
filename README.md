# sentry_chassis_controller

Sentry chassis controller package for ROS1 Noetic and Gazebo.

## Overview

This package keeps a single controller plugin:

- `sentry_chassis_controller/SentryChassisController`

It is built on top of `rm_chassis_controllers::ChassisBase` and keeps only the Sentry-specific controller logic.

The old `kinematics`, runtime parameter tuning, and PID tuning scripts are no longer part of the package.

## Installation

Build the package in a catkin workspace:

```bash
source /opt/ros/noetic/setup.bash
catkin build sentry_chassis_controller
```

## Dependencies

### Third-party base

- `rm_chassis_controllers::ChassisBase`

### Catkin dependencies

- `roscpp`
- `controller_interface`
- `hardware_interface`
- `pluginlib`
- `rm_chassis_controllers`
- `effort_controllers`
- `angles`
- `eigen`

## Configuration

The controller namespace is `/sentry_chassis_controller`.

### Base parameters

Provided by `ChassisBase`:

- `publish_rate`
- `enable_odom_tf`
- `publish_odom_tf`
- `timeout`
- `wheel_radius`
- `max_odom_vel`
- `twist_covariance_diagonal`
- `power`
- `pid_follow`

### Package parameters

Custom Sentry parameters:

- `wheel_rolling_signs`
- `wheel_direction_signs/vx`
- `wheel_direction_signs/vy`
- `wheel_direction_signs/wz`
- `modules`

Each module requires:

- `position`
- `pivot/joint`
- `pivot/offset`
- `pivot/pid`
- `wheel/joint`
- `wheel/radius`
- `wheel/pid`

Removed parameters:

- `steer_zero_offsets`
- `command_compensation_matrix`
- `enable_dynamic_reconfigure`

## Usage

Controller-only launch:

```bash
roslaunch sentry_chassis_controller sentry_chassis_controller.launch
```

Gazebo launch:

```bash
roslaunch sentry_chassis_controller sentry_sim.launch
```

Headless Gazebo:

```bash
roslaunch sentry_chassis_controller sentry_sim.launch gui:=false with_foxglove:=false
```

Teleop:

```bash
rosrun sentry_chassis_controller teleop_keyboard.sh
```

## Scripts

- `evaluate_teleop_behavior.py`
- `evaluate_teleop_keys.py`
- `auto_calibrate_signs.py`
- `auto_calibrate_sign_chain.py`
- `publish_default_chassis_cmd.py`

Container wrappers:

- `./scripts/_run_teleop_eval_inside.sh`
- `./scripts/_run_auto_calibrate_inside.sh`
- `./scripts/_run_auto_calibrate_sign_chain_inside.sh`
