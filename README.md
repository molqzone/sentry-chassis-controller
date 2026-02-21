# sentry_chassis_controller (ROS package)

本包已按 `rm_template` 的结构组织，并保留哨兵底盘控制器特有的 plugin 控制器入口。

## 模板对齐内容

- 目录结构：`include/`、`src/`、`config/`、`launch/`、`test/`、`doc/`。
- 构建结构：核心算法库（`kinematics`）+ 节点入口（`sentry_chassis_controller_node`）+ gtest。
- 参数加载：默认参数（`config/default.yaml`）+ 覆盖参数启动文件。

## 项目特有内容

- `SentryChassisController` pluginlib 控制器骨架：
  - `include/sentry_chassis_controller/sentry_chassis_controller.h`
  - `src/sentry_chassis_controller.cpp`
  - `sentry_chassis_controller_plugins.xml`
- 控制器参数模板：`config/chassis_controller.yaml`

## 下一步实现建议

1. 基于 `control_toolbox::Pid` 完成八路 PID（4 舵向 + 4 驱动）。
2. 订阅 `/cmd_vel`，完成舵轮逆运动学。
3. 基于轮速反馈计算里程计并发布 `/odom` + `odom -> base_link` tf。
4. 增加坐标系模式切换（`base_link` / `odom`）。
5. 加入特色功能（加速度限制、小陀螺、功率控制、自锁等）。
