# rm_template 对齐说明

当前 `sentry_chassis_controller` 包基于 `rm_template` 的工程组织进行了对齐：

- 目录结构：`include/`、`src/`、`config/`、`launch/`、`test/`、`doc/`
- 构建结构：核心库 + 节点入口 + gtest
- 参数加载：默认参数文件 + 覆盖参数启动文件

保留了本项目特有的 `pluginlib` 控制器入口：

- `sentry_chassis_controller_plugins.xml`
- `SentryChassisController` 控制器类
