#ifndef SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H_
#define SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H_

#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <ros/timer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "sentry_chassis_controller/kinematics.hpp"

/**
 * @file   sentry_chassis_controller.h
 * @brief  舵轮底盘 ros_control 插件控制器声明
 *         Declaration of sentry chassis ros_control plugin controller
 */

namespace sentry_chassis_controller
{

/**
 * @brief  舵轮底盘 ros_control 插件控制器
 *         Sentry chassis ros_control plugin controller
 *
 * @details
 * 该控制器通过 pluginlib 被 `controller_manager` 加载，在实时 `update()` 循环中完成：
 * 1) 读取 `/cmd_vel` 指令；2) 逆运动学解算轮速目标；3) 8 路 PID（4 舵向 + 4 轮速）闭环；
 * 4) 向 effort joint 写入控制指令。
 *
 * The controller is loaded by `controller_manager` via pluginlib and performs:
 * 1) `/cmd_vel` command ingestion, 2) inverse-kinematics wheel target computation,
 * 3) eight PID loops (4 steering + 4 wheel), and 4) effort command output.
 */
class SentryChassisController
    : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
 public:
  /**
   * @brief  轮子数量常量（前左、前右、后左、后右）
   *         Wheel count constant (front-left, front-right, rear-left, rear-right)
   */
  static constexpr std::size_t WHEEL_COUNT = 4U;

  /**
   * @brief  判断速度指令是否超时
   *         Checks whether the velocity command is timed out
   * @param  command_valid 指令有效标记 Command validity flag
   * @param  command_stamp 指令时间戳 Command timestamp
   * @param  now 当前控制周期时间 Current control-cycle time
   * @param  timeout_sec 超时阈值（秒） Timeout threshold in seconds
   * @return `true` 表示超时或无效，`false` 表示仍可用
   *         `true` means timed out/invalid, `false` means still valid
   */
  static bool IsCommandTimedOut(bool command_valid, const ros::Time& command_stamp,
                                const ros::Time& now, double timeout_sec);

  /**
   * @brief  2D 里程计状态结构
   *         2D odometry state structure
   */
  struct OdomState
  {
    double x = 0.0;    ///< 世界系 x 坐标 World-frame x position
    double y = 0.0;    ///< 世界系 y 坐标 World-frame y position
    double yaw = 0.0;  ///< 世界系偏航角 World-frame yaw angle
  };

  /**
   * @brief  将角度归一化到 [-pi, pi]
   *         Normalizes angle to [-pi, pi]
   * @param  angle 输入角度 Input angle
   * @return 归一化结果 Normalized angle
   */
  static double NormalizeAngle(double angle);

  /**
   * @brief  基于中点法积分里程计状态
   *         Integrates odometry state using midpoint method
   * @param  state 当前里程计状态 Current odometry state
   * @param  twist 底盘速度（base_link） Chassis twist in base_link
   * @param  dt 积分步长（秒） Integration time step in seconds
   * @return 积分后的里程计状态 Integrated odometry state
   */
  static OdomState IntegrateOdom(const OdomState& state,
                                 const Kinematics::ChassisTwist& twist, double dt);

  /**
   * @brief  速度指令解析模式
   *         Velocity command interpretation mode
   */
  enum class CommandVelocityMode
  {
    BASE_LINK = 0,  ///< 直接按底盘坐标解释 / Interpret command as base frame
    GLOBAL = 1,     ///< 按全局坐标解释并转换 / Interpret command as global frame
  };

  /**
   * @brief  根据 TF 变换将速度从源坐标系转换到目标坐标系
   *         Transforms twist from source frame to target frame via TF transform
   * @param  input 输入速度 Input twist
   * @param  transform 源到目标的 TF 变换 Source-to-target transform
   * @param  output 输出速度 Output twist
   * @return 转换是否成功 Transform success flag
   */
  static bool TransformTwistWithTransform(
      const Kinematics::ChassisTwist& input,
      const geometry_msgs::TransformStamped& transform, Kinematics::ChassisTwist* output);

  /**
   * @brief  初始化控制器
   *         Initializes the controller
   * @param  hw EffortJointInterface 句柄接口 EffortJointInterface handle
   * @param  nh 控制器私有命名空间 NodeHandle Private controller namespace
   * @return 初始化是否成功 Initialization success flag
   */
  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& nh) override;

  /**
   * @brief  控制器启动回调
   *         Controller start callback
   * @param  time 启动时刻 Start time
   */
  void starting(const ros::Time& time) override;

  /**
   * @brief  实时控制更新入口
   *         Realtime control update entry
   * @param  time 当前时刻 Current time
   * @param  period 当前周期 Current control period
   */
  void update(const ros::Time& time, const ros::Duration& period) override;

  /**
   * @brief  控制器停止回调
   *         Controller stop callback
   * @param  time 停止时刻 Stop time
   */
  void stopping(const ros::Time& time) override;

 private:
  friend class SentryChassisControllerRuntimeParamsTestAccessor;

  /**
   * @brief  实时缓存中的速度指令快照
   *         Velocity command snapshot stored in realtime buffer
   */
  struct CommandData
  {
    double vx = 0.0;     ///< x 向线速度（base_link） x-axis linear velocity in base_link
    double vy = 0.0;     ///< y 向线速度（base_link） y-axis linear velocity in base_link
    double wz = 0.0;     ///< z 向角速度（base_link） z-axis angular velocity in base_link
    ros::Time stamp;     ///< 指令时间戳 Command timestamp
    bool valid = false;  ///< 指令有效标记 Command validity flag
  };

  /**
   * @brief  实时控制循环只读的参数快照
   *         Runtime parameter snapshot consumed by realtime update loop
   */
  struct RuntimeParams
  {
#define SENTRY_RUNTIME_PARAM_FIELD(type, name, default_value) type name = default_value;
#include "sentry_chassis_controller/runtime_param_fields.inc"
#undef SENTRY_RUNTIME_PARAM_FIELD
  };

  /**
   * @brief  全局指令模式下的坐标变换缓存（非实时线程刷新，实时线程只读）
   *         Cached transform for global command mode (non-RT refresh, RT read-only)
   */
  struct CommandTransformCache
  {
    std::array<double, 9> rotation_matrix_row_major{
        {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}};
    ros::Time stamp;
    bool valid = false;
  };

  /**
   * @brief  `/cmd_vel` 订阅回调
   *         `/cmd_vel` subscriber callback
   * @param  message 速度指令消息 Incoming velocity command message
   */
  void CmdVelCallback(const geometry_msgs::TwistConstPtr& message);

  /**
   * @brief  初始化指定分组的四路 PID
   *         Initializes four PID instances in a group
   * @param  nh 控制器命名空间 Controller namespace
   * @param  pid_group PID 分组名（`steer` 或 `wheel`）
   *                   PID group name (`steer` or `wheel`)
   * @param  pid_array PID 数组输出指针 PID array output pointer
   * @return 初始化是否成功 Initialization success flag
   */
  bool InitPidGroup(ros::NodeHandle& nh, const std::string& pid_group,
                    std::array<control_toolbox::Pid, WHEEL_COUNT>* pid_array);

  /**
   * @brief  加载控制器静态配置并初始化运行参数快照
   *         Loads static controller config and initializes runtime parameter snapshot
   */
  bool LoadControllerConfig(
      ros::NodeHandle& nh, std::array<std::string, WHEEL_COUNT>* steer_joint_names,
      std::array<std::string, WHEEL_COUNT>* wheel_joint_names);

  /**
   * @brief  注册动态参数（供初始化阶段和运行时复用）
   *         Registers shared dynamic parameters (reused by init and runtime)
   */
  void RegisterSharedRuntimeParameters(
      ddynamic_reconfigure::DDynamicReconfigure* parameter_loader);

  /**
   * @brief  初始化硬件关节句柄
   *         Initializes hardware joint handles
   */
  bool InitJointHandles(
      hardware_interface::EffortJointInterface* hw,
      const std::array<std::string, WHEEL_COUNT>& steer_joint_names,
      const std::array<std::string, WHEEL_COUNT>& wheel_joint_names);

  /**
   * @brief  初始化控制器 PID 组
   *         Initializes controller PID groups
   */
  bool InitControllerPids(ros::NodeHandle& nh);

  /**
   * @brief  初始化实时状态与缓存
   *         Initializes realtime state and buffers
   */
  void InitRealtimeState();

  /**
   * @brief  初始化 ROS 话题接口
   *         Initializes ROS topic interfaces
   */
  void InitRosInterfaces(ros::NodeHandle& nh);

  /**
   * @brief  初始化 TF 相关资源
   *         Initializes TF-related resources
   */
  void InitTfResources();

  /**
   * @brief  初始化全局指令 TF 缓存刷新定时器（非实时线程）
   *         Initializes global-command TF cache refresh timer (non-RT thread)
   */
  void InitCommandTransformCacheTimer(ros::NodeHandle& nh);

  /**
   * @brief  初始化运行时动态调参服务
   *         Initializes runtime dynamic reconfigure service
   */
  bool InitRuntimeDynamicReconfigure(ros::NodeHandle& nh);

  /**
   * @brief  读取并校验轮向符号矩阵参数
   *         Loads and validates wheel direction sign matrix parameters
   * @param  nh 控制器命名空间 Controller namespace
   * @param  direction_signs 轮向符号输出 Output direction signs
   * @return 读取是否成功 Load success flag
   */
  bool LoadDirectionSigns(ros::NodeHandle& nh,
                          Kinematics::DirectionSigns* direction_signs);

  /**
   * @brief  读取并校验轮速方向符号参数
   *         Loads and validates wheel rolling signs
   * @param  nh 控制器命名空间 Controller namespace
   * @param  rolling_signs 符号输出 Output rolling signs
   * @return 读取是否成功 Load success flag
   */
  bool LoadRollingSigns(ros::NodeHandle& nh, std::array<int, WHEEL_COUNT>* rolling_signs);

  /**
   * @brief  解析并校验速度指令模式参数
   *         Parses and validates velocity command mode parameter
   * @param  mode_text 原始参数文本 Raw parameter text
   * @param  mode 解析输出 Parsed mode output
   * @return 解析是否成功 Parse success flag
   */
  static bool ParseCommandVelocityMode(const std::string& mode_text,
                                       CommandVelocityMode* mode);

  /**
   * @brief  将当前命令统一转换为底盘坐标系速度
   *         Resolves current command into base-frame twist
   * @param  command 原始命令 Raw command
   * @param  time 当前控制时刻 Current control time
   * @param  runtime_params 实时参数快照 Runtime parameter snapshot
   * @param  base_twist 输出到底盘坐标系的速度 Output base-frame twist
   * @return 转换是否成功 Resolve success flag
   */
  bool ResolveCommandInBaseFrame(const CommandData& command, const ros::Time& time,
                                 const RuntimeParams& runtime_params,
                                 Kinematics::ChassisTwist* base_twist);

  /**
   * @brief  准备本周期底盘指令（超时判定+坐标转换+加速度限幅）
   *         Prepares per-cycle chassis command (timeout, frame resolve, accel limits)
   */
  bool PrepareCommandForControl(const CommandData& command, const ros::Time& time,
                                double dt, const RuntimeParams& runtime_params,
                                Kinematics::ChassisTwist* limited_command_twist_base,
                                bool* timeout);

  /**
   * @brief  基于底盘指令计算并下发舵向/轮速控制
   *         Computes and applies steering/wheel control from chassis command
   */
  void ComputeAndApplyWheelControl(
      const Kinematics::ChassisTwist& limited_command_twist_base,
      const ros::Duration& period, const RuntimeParams& runtime_params);

  /**
   * @brief  基于轮反馈计算里程计速度并执行积分
   *         Computes odometry twist from wheel feedback and integrates state
   */
  Kinematics::ChassisTwist ComputeAndIntegrateOdometry(
      const ros::Time& time, double dt, bool timeout,
      const RuntimeParams& runtime_params);

  /**
   * @brief  解析单轴方向符号参数
   *         Parses one axis of wheel direction signs
   * @param  axis_values 参数原始数组 Raw parameter array
   * @param  param_name 参数名 Parameter name
   * @param  output 解析输出 Parsed output
   * @return 解析是否成功 Parse success flag
   */
  static bool ParseDirectionAxis(const std::vector<int>& axis_values,
                                 const std::string& param_name,
                                 std::array<int, WHEEL_COUNT>* output);

  /**
   * @brief  判断正运动学输出是否在可接受范围内
   *         Checks whether forward-kinematics twist is within safe bounds
   * @param  twist 本周期底盘速度 Current cycle chassis twist
   * @param  runtime_params 实时参数快照 Runtime parameter snapshot
   * @return 可接受返回 true，否则返回 false
   *         Returns true when acceptable, false otherwise
   */
  bool IsOdomTwistAcceptable(const Kinematics::ChassisTwist& twist,
                             const RuntimeParams& runtime_params) const;

  /**
   * @brief  同步并约束控制器运行参数
   *         Validates and applies controller runtime parameters
   * @param  strict_validation 严格模式下参数非法返回失败
   *         Strict mode returns false on invalid parameter
   * @return 参数有效返回 true，否则返回 false
   *         Returns true when parameters are valid
   */
  bool ValidateAndApplyControllerParams(bool strict_validation);

  /**
   * @brief  将全局指令 TF 缓存标记为无效
   *         Marks global-command TF cache as invalid
   */
  void InvalidateCommandTransformCache();

  /**
   * @brief  非实时线程刷新全局指令 TF 缓存
   *         Refreshes global-command TF cache in non-RT thread
   */
  void RefreshCommandTransformCache(const ros::TimerEvent& event);

  /**
   * @brief  在非实时线程汇总输出实时循环延迟告警
   *         Aggregates deferred realtime-loop warnings in non-realtime thread
   */
  void FlushDeferredRealtimeWarnings();

  /**
   * @brief  将实时参数快照应用到控制循环上下文
   *         Applies runtime snapshot to update-loop local context
   * @param  runtime_params 实时参数快照 Runtime parameter snapshot
   */
  void ApplyRuntimeParamsInUpdate(const RuntimeParams& runtime_params);

  /**
   * @brief  对底盘速度指令应用加速度限制
   *         Applies acceleration limits to chassis command twist
   * @param  input 输入速度指令 Input command twist
   * @param  dt 控制周期（秒） Control period in seconds
   * @param  runtime_params 实时参数快照 Runtime parameter snapshot
   * @param  output 限幅后的速度指令 Limited command twist
   */
  void ApplyAccelerationLimits(const Kinematics::ChassisTwist& input, double dt,
                               const RuntimeParams& runtime_params,
                               Kinematics::ChassisTwist* output);

  /**
   * @brief  对轮速环输出应用功率限制
   *         Applies power limiting to wheel effort outputs
   * @param  runtime_params 实时参数快照 Runtime parameter snapshot
   * @param  signed_wheel_velocities 轮速反馈（已按 rolling_sign 统一） Signed wheel velocities
   * @param  signed_wheel_efforts 轮速控制输出（已按 rolling_sign 统一） Signed wheel efforts
   */
  void ApplyPowerLimiting(const RuntimeParams& runtime_params,
                          const std::array<double, WHEEL_COUNT>& signed_wheel_velocities,
                          std::array<double, WHEEL_COUNT>* signed_wheel_efforts);

  /**
   * @brief  将一组关节命令统一设置为同一数值
   *         Sets one command value for all joints in a group
   * @param  joints 关节句柄组 Joint handle group
   * @param  command 目标命令值 Target command value
   */
  static void SetAllCommands(
      std::array<hardware_interface::JointHandle, WHEEL_COUNT>* joints,
      double command);

  /**
   * @brief  重置执行器输出与 PID
   *         Resets actuator outputs and PID states
   */
  void ResetControllerOutputsAndPids();

  /**
   * @brief  重置控制循环跟踪状态
   *         Resets controller runtime tracking state
   */
  void ResetControllerTrackingState(const ros::Time& start_time);

  /**
   * @brief  发布当前里程计与 TF
   *         Publishes current odometry and TF
   * @param  time 当前控制时刻 Current control time
   * @param  twist 本周期底盘速度 Current cycle chassis twist
   * @param  runtime_params 实时参数快照 Runtime parameter snapshot
   */
  void PublishOdometry(const ros::Time& time, const Kinematics::ChassisTwist& twist,
                       const RuntimeParams& runtime_params);

  std::array<hardware_interface::JointHandle, WHEEL_COUNT>
      steer_joints_;  ///< 舵向关节句柄 Steering joint handles
  std::array<hardware_interface::JointHandle, WHEEL_COUNT>
      wheel_joints_;  ///< 轮速关节句柄 Wheel joint handles
  std::array<control_toolbox::Pid, WHEEL_COUNT>
      steer_pids_;  ///< 4 路舵向 PID 4 steering PID loops
  std::array<control_toolbox::Pid, WHEEL_COUNT>
      wheel_pids_;  ///< 4 路轮速 PID 4 wheel PID loops
  std::array<double, WHEEL_COUNT>
      steer_zero_offsets_{};  ///< 舵向零位目标 Steering zero-position targets
  Kinematics::DirectionSigns
      direction_signs_{};  ///< 三轴符号矩阵缓存 Cached per-axis sign matrix
  std::array<int, WHEEL_COUNT> wheel_rolling_signs_{
      {1, 1, 1, 1}};                    ///< 轮速方向符号 Wheel rolling direction signs
  Kinematics::Geometry geometry_{};     ///< 底盘几何参数缓存 Cached chassis geometry
  Kinematics kinematics_;               ///< 逆运动学求解器 Inverse kinematics solver
  ros::Subscriber cmd_vel_subscriber_;  ///< 速度指令订阅器 Velocity command subscriber
  ros::Publisher odom_publisher_;       ///< 里程计发布器 Odometry publisher
  std::unique_ptr<tf2_ros::TransformBroadcaster>
      tf_broadcaster_;  ///< TF 广播器 TF broadcaster
  realtime_tools::RealtimeBuffer<CommandData>
      command_buffer_;  ///< 实时安全指令缓存 Realtime-safe command buffer
  realtime_tools::RealtimeBuffer<RuntimeParams>
      runtime_params_buffer_;  ///< 实时循环参数快照缓存 Runtime parameter snapshot buffer
  realtime_tools::RealtimeBuffer<CommandTransformCache> command_transform_buffer_;
  ros::Timer command_transform_cache_timer_;
  std::unique_ptr<ddynamic_reconfigure::DDynamicReconfigure>
      controller_params_reconfigure_;       ///< 控制器参数动态调参服务 Controller param dynamic server
  RuntimeParams runtime_params_shadow_;     ///< 非实时线程参数缓存 Non-realtime parameter cache
  std::string cmd_vel_topic_ = "/cmd_vel";  ///< 指令话题 Command topic
  std::string command_velocity_mode_text_ =
      "base_link";  ///< 指令模式参数文本 Command mode parameter text
  CommandVelocityMode command_velocity_mode_ =
      CommandVelocityMode::BASE_LINK;  ///< 指令解析模式 Command interpretation mode
  std::string command_frame_id_ = "base_link";  ///< 指令坐标系 Command frame id
  double cmd_vel_timeout_ = 0.25;  ///< 指令超时阈值（秒） Command timeout in seconds
  bool enable_dynamic_reconfigure_ =
      true;                             ///< 是否启用动态调参 Enable dynamic reconfigure
  std::string odom_topic_ = "/odom";    ///< 里程计话题 Odometry topic
  std::string odom_frame_id_ = "odom";  ///< 里程计父坐标系 Odom frame id
  std::string base_frame_id_ = "base_link";  ///< 底盘坐标系 Base frame id
  Kinematics::Geometry applied_geometry_{};  ///< update线程生效几何参数 Runtime-applied geometry
  std::string applied_odom_frame_id_;        ///< 最近一次生效的 odom 帧 Last applied odom frame id
  std::string applied_base_frame_id_;        ///< 最近一次生效的 base 帧 Last applied base frame id
  double odom_startup_hold_sec_ =
      1.0;  ///< 启动静置窗口（秒） Startup settling window in seconds
  double odom_max_linear_speed_ =
      8.0;  ///< 里程计线速度上限（m/s） Odom linear speed limit in m/s
  double odom_max_angular_speed_ =
      16.0;  ///< 里程计角速度上限（rad/s） Odom angular speed limit in rad/s
  bool odom_integrate_on_timeout_ =
      false;  ///< 指令超时时是否继续积分 Odom integration when cmd times out
  bool publish_tf_ = true;  ///< 是否发布 odom->base_link TF Whether to publish odom TF
  double wheel_effort_limit_ =
      12.0;  ///< 轮速回路输出限幅（绝对值） Wheel effort command limit (absolute)
  double reverse_ccw_vx_scale_ =
      1.0;  ///< 反向左转补偿：vx 缩放 Reverse-CCW compensation vx scale
  double reverse_ccw_wz_gain_ =
      1.0;  ///< 反向左转补偿：wz 增益 Reverse-CCW compensation wz gain
  double reverse_ccw_vy_threshold_ =
      0.03;  ///< 反向左转补偿触发阈值 Reverse-CCW compensation |vy| trigger threshold
  double reverse_ccw_steer_priority_error_ =
      0.6;  ///< 反向左转补偿：舵向优先误差阈值 Reverse-CCW steering-priority threshold
  bool enable_acceleration_limits_ =
      false;  ///< 是否启用底盘加速度限制 Enable chassis acceleration limits
  double max_linear_acceleration_ =
      3.0;  ///< 最大线加速度（m/s^2） Maximum linear acceleration in m/s^2
  double max_angular_acceleration_ =
      5.0;  ///< 最大角加速度（rad/s^2） Maximum angular acceleration in rad/s^2
  bool enable_power_limit_ = false;  ///< 是否启用功率限制 Enable power limiting
  bool enable_power_limit_logging_ =
      false;  ///< 是否打印功率限制日志 Enable power-limit logging
  double max_power_ = 360.0;  ///< 功率上限（W） Maximum power limit in watts
  double power_loss_k1_ =
      0.001;  ///< 力矩损耗项系数 Coefficient for torque-squared loss term
  double power_loss_k2_ =
      0.0001;  ///< 转速损耗项系数 Coefficient for velocity-squared loss term
  double min_power_scale_ = 0.3;  ///< 功率限制最小缩放 Minimum power-limiting scale
  std::array<double, 9> command_compensation_matrix_{
      {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}};  ///< 速度指令线性补偿矩阵 Row-major 3x3 command compensation matrix
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;               ///< TF 缓冲区 TF buffer
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;  ///< TF 监听器 TF listener
  OdomState odom_state_;             ///< 累计里程计状态 Integrated odometry state
  Kinematics::ChassisTwist
      last_limited_command_;  ///< 上次限幅后的底盘速度 Last acceleration-limited command
  bool has_last_limited_command_ =
      false;  ///< 限幅状态是否已初始化 Whether limiter state has been initialized
  ros::Time controller_start_time_;  ///< 控制器启动时刻 Controller start timestamp
  bool last_command_timed_out_ = true;  ///< 上周期超时状态 Previous-cycle timeout flag
  std::atomic<uint32_t> rt_warn_invalid_period_count_{0};
  std::atomic<uint32_t> rt_warn_runtime_params_unready_count_{0};
  std::atomic<uint32_t> rt_warn_command_buffer_unready_count_{0};
  std::atomic<uint32_t> rt_warn_prepare_command_failed_count_{0};
  std::atomic<uint32_t> rt_warn_transform_cache_unready_count_{0};
  std::atomic<uint32_t> rt_warn_odom_singular_count_{0};
  std::atomic<uint32_t> rt_warn_odom_startup_hold_count_{0};
  std::atomic<uint32_t> rt_warn_odom_rejected_count_{0};
  std::atomic<uint32_t> rt_warn_power_limit_active_count_{0};
};

}  // namespace sentry_chassis_controller

#endif  // SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H_
