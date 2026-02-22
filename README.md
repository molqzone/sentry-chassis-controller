# sentry_chassis_controller (ROS package)

本包已按 `rm_template` 的结构组织，并保留哨兵底盘控制器特有的 plugin 控制器入口。

## 模板对齐内容

- 目录结构：`include/`、`src/`、`config/`、`launch/`、`test/`、`doc/`。
- 构建结构：核心算法库（`kinematics`）+ plugin 控制器（`SentryChassisController`）+ gtest。
- 参数加载：统一通过 `config/chassis_controller.yaml` 由 plugin 链路读取。

## 运行链路

- 主链路仅保留 plugin：`roslaunch sentry_chassis_controller sentry_chassis_controller.launch`

## 项目特有内容

- `SentryChassisController` pluginlib 控制器（8 路 PID 闭环）：
  - `include/sentry_chassis_controller/sentry_chassis_controller.h`
  - `src/sentry_chassis_controller.cpp`
  - `sentry_chassis_controller_plugins.xml`
- 控制器参数模板：`config/chassis_controller.yaml`（4 路舵向 + 4 路驱动独立 PID）

## PID 参数结构

控制器命名空间：`/sentry_chassis_controller`

- `cmd_vel_topic`：速度指令话题，默认 `/cmd_vel`
- `command_frame_id`：速度指令坐标系，当前阶段仅支持 `base_link`
- `cmd_vel_timeout`：指令超时秒数，超时自动置零，默认 `0.25`
- `enable_dynamic_reconfigure`：是否启用 `rqt_reconfigure` 在线调参（默认 `true`）
- `geometry/wheel_base`、`geometry/track_width`、`geometry/wheel_radius`
- `steer_zero_offsets`：四个舵向零位目标（rad）
- `wheel_direction_signs/vx`、`wheel_direction_signs/vy`、`wheel_direction_signs/wz`：
  轮向符号矩阵，轮序固定 `front_left, front_right, rear_left, rear_right`，每项只能为 `-1` 或 `1`
- 八路 PID（每路独立）：
  - `pid/steer/front_left/*`
  - `pid/steer/front_right/*`
  - `pid/steer/rear_left/*`
  - `pid/steer/rear_right/*`
  - `pid/wheel/front_left/*`
  - `pid/wheel/front_right/*`
  - `pid/wheel/rear_left/*`
  - `pid/wheel/rear_right/*`

每路 PID 统一字段：
- `p`、`i`、`d`
- `i_clamp_min`、`i_clamp_max`
- `antiwindup`
- `publish_state`

## 需求 5 对齐说明（逆运动学）

1. `/cmd_vel` 的 `geometry_msgs/Twist` 按 `base_link` 解释：
   - `linear.x`：前进方向为正
   - `linear.y`：左向为正
   - `angular.z`：绕 z 轴逆时针为正
2. 轮序固定为：`front_left`、`front_right`、`rear_left`、`rear_right`。
3. 逆运动学公式：
   - `fl = (sx_fl*vx - sy_fl*vy - sz_fl*((wheel_base + track_width) / 2)*wz) / wheel_radius`
   - `fr = (sx_fr*vx + sy_fr*vy + sz_fr*((wheel_base + track_width) / 2)*wz) / wheel_radius`
   - `rl = (sx_rl*vx + sy_rl*vy - sz_rl*((wheel_base + track_width) / 2)*wz) / wheel_radius`
   - `rr = (sx_rr*vx - sy_rr*vy + sz_rr*((wheel_base + track_width) / 2)*wz) / wheel_radius`
4. 几何参数 `geometry/wheel_base`、`geometry/track_width`、`geometry/wheel_radius`
   可直接在 `config/chassis_controller.yaml` 中配置并实时影响目标轮速解算。

## 轮向符号矩阵（wheel_direction_signs）

用于对齐仿真模型中每个轮子的实际正方向，避免“公式符号正确但关节方向相反”导致的验收误判。

- 默认行为：三个轴参数都缺失时，按全 `1` 处理，不改变原始逆解公式。
- 配置规则：只要配置了任意一个轴，`vx/vy/wz` 三组必须同时存在且每组长度为 4。
- 当前模型推荐值：
  - `vx: [1, -1, -1, -1]`
  - `vy: [-1, -1, -1, 1]`
  - `wz: [1, 1, 1, -1]`
- 调参方法：保持轮序不变，针对单轴输入（仅 `vx`、仅 `vy`、仅 `wz`）逐项调整到符号一致。

## 在线调参与观测

1. 启动控制器：
   - `roslaunch sentry_chassis_controller sentry_chassis_controller.launch`
2. 打开动态调参：
   - `rosrun rqt_reconfigure rqt_reconfigure`
3. 打开曲线观测：
   - `rosrun rqt_plot rqt_plot`

当 `publish_state: true` 时，每路 PID 会发布 `state` 话题，可用 `rqt_plot` 观察误差与输出变化。

## 后续扩展建议

1. 基于舵轮几何模型实现舵向角实时解算。
2. 基于轮速反馈实现正运动学里程计并发布 `/odom`。
3. 增加 `base_link`/`odom` 速度模式切换。
4. 加入特色功能（加速度限制、小陀螺、功率控制、自锁等）。
