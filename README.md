# sentry_chassis_controller

当前包只保留一个控制器插件：`sentry_chassis_controller/SentryChassisController`。

实现边界已经收成两层：

- 上游依赖：`rm_chassis_controllers::ChassisBase`
- 业务实现：`SentryChassisController`

不再保留旧版 `kinematics`、runtime-param、PID 调参脚本和对应测试。

## 关键文件

- 控制器头文件：`include/sentry_chassis_controller/sentry_chassis_controller.h`
- 控制器实现：`src/sentry_chassis_controller.cpp`
- 插件导出：`sentry_chassis_controller_plugins.xml`
- 默认参数：`config/chassis_controller.yaml`
- 启动文件：`launch/sentry_chassis_controller.launch`、`launch/sentry_sim.launch`

## 参数结构

控制器命名空间：`/sentry_chassis_controller`

基础参数由 `ChassisBase` 提供，当前实际使用的主要是：

- `publish_rate`
- `enable_odom_tf`
- `publish_odom_tf`
- `timeout`
- `wheel_radius`
- `max_odom_vel`
- `twist_covariance_diagonal`
- `power`
- `pid_follow`

任务控制器自身参数只有三组：

- `wheel_rolling_signs`
- `wheel_direction_signs/vx|vy|wz`
- `modules`

`modules` 下每个模块都需要：

- `position`
- `pivot/joint`
- `pivot/offset`
- `pivot/pid`
- `wheel/joint`
- `wheel/radius`
- `wheel/pid`

已经移除的旧参数：

- `steer_zero_offsets`
- `command_compensation_matrix`
- `enable_dynamic_reconfigure`

## 运行

仅控制器链路：

```bash
roslaunch sentry_chassis_controller sentry_chassis_controller.launch
```

Gazebo 一体链路：

```bash
roslaunch sentry_chassis_controller sentry_sim.launch
```

关闭 GUI 和 Foxglove：

```bash
roslaunch sentry_chassis_controller sentry_sim.launch gui:=false with_foxglove:=false
```

## 常用脚本

键盘控制：

```bash
rosrun sentry_chassis_controller teleop_keyboard.sh
```

行为评估：

```bash
rosrun sentry_chassis_controller evaluate_teleop_behavior.py
rosrun sentry_chassis_controller evaluate_teleop_keys.py
```

符号自动校准：

```bash
rosrun sentry_chassis_controller auto_calibrate_signs.py
rosrun sentry_chassis_controller auto_calibrate_sign_chain.py
```

容器内 headless 回归入口：

```bash
./scripts/_run_teleop_eval_inside.sh
./scripts/_run_auto_calibrate_inside.sh
./scripts/_run_auto_calibrate_sign_chain_inside.sh
```
