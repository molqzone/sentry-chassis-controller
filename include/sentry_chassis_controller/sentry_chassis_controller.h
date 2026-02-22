#ifndef SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H_
#define SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H_

#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <array>
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
    BASE_LINK = 0,   ///< 直接按底盘坐标解释 / Interpret command as base frame
    GLOBAL = 1,      ///< 按全局坐标解释并转换 / Interpret command as global frame
  };

  /**
   * @brief  根据 TF 变换将速度从源坐标系转换到目标坐标系
   *         Transforms twist from source frame to target frame via TF transform
   * @param  input 输入速度 Input twist
   * @param  transform 源到目标的 TF 变换 Source-to-target transform
   * @param  output 输出速度 Output twist
   * @return 转换是否成功 Transform success flag
   */
  static bool TransformTwistWithTransform(const Kinematics::ChassisTwist& input,
                                          const geometry_msgs::TransformStamped& transform,
                                          Kinematics::ChassisTwist* output);

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
  /**
   * @brief  实时缓存中的速度指令快照
   *         Velocity command snapshot stored in realtime buffer
   */
  struct CommandData
  {
    double vx = 0.0;   ///< x 向线速度（base_link） x-axis linear velocity in base_link
    double vy = 0.0;   ///< y 向线速度（base_link） y-axis linear velocity in base_link
    double wz = 0.0;   ///< z 向角速度（base_link） z-axis angular velocity in base_link
    ros::Time stamp;   ///< 指令时间戳 Command timestamp
    bool valid = false;  ///< 指令有效标记 Command validity flag
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
  bool LoadRollingSigns(ros::NodeHandle& nh,
                        std::array<int, WHEEL_COUNT>* rolling_signs);

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
   * @param  base_twist 输出到底盘坐标系的速度 Output base-frame twist
   * @return 转换是否成功 Resolve success flag
   */
  bool ResolveCommandInBaseFrame(const CommandData& command, const ros::Time& time,
                                 Kinematics::ChassisTwist* base_twist);

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
   * @return 可接受返回 true，否则返回 false
   *         Returns true when acceptable, false otherwise
   */
  bool IsOdomTwistAcceptable(const Kinematics::ChassisTwist& twist) const;

  /**
   * @brief  将一组关节命令统一设置为同一数值
   *         Sets one command value for all joints in a group
   * @param  joints 关节句柄组 Joint handle group
   * @param  command 目标命令值 Target command value
   */
  static void SetAllCommands(std::vector<hardware_interface::JointHandle>* joints,
                             double command);

  /**
   * @brief  发布当前里程计与 TF
   *         Publishes current odometry and TF
   * @param  time 当前控制时刻 Current control time
   * @param  twist 本周期底盘速度 Current cycle chassis twist
   */
  void PublishOdometry(const ros::Time& time, const Kinematics::ChassisTwist& twist);

  std::vector<hardware_interface::JointHandle>
      steer_joints_;  ///< 舵向关节句柄 Steering joint handles
  std::vector<hardware_interface::JointHandle>
      wheel_joints_;  ///< 轮速关节句柄 Wheel joint handles
  std::array<control_toolbox::Pid, WHEEL_COUNT>
      steer_pids_;  ///< 4 路舵向 PID 4 steering PID loops
  std::array<control_toolbox::Pid, WHEEL_COUNT>
      wheel_pids_;  ///< 4 路轮速 PID 4 wheel PID loops
  std::array<double, WHEEL_COUNT>
      steer_zero_offsets_{};  ///< 舵向零位目标 Steering zero-position targets
  std::array<int, WHEEL_COUNT>
      wheel_rolling_signs_{{1, 1, 1, 1}};  ///< 轮速方向符号 Wheel rolling direction signs
  Kinematics kinematics_;  ///< 逆运动学求解器 Inverse kinematics solver
  ros::Subscriber cmd_vel_subscriber_;  ///< 速度指令订阅器 Velocity command subscriber
  ros::Publisher odom_publisher_;  ///< 里程计发布器 Odometry publisher
  std::unique_ptr<tf2_ros::TransformBroadcaster>
      tf_broadcaster_;  ///< TF 广播器 TF broadcaster
  realtime_tools::RealtimeBuffer<CommandData>
      command_buffer_;  ///< 实时安全指令缓存 Realtime-safe command buffer
  std::string cmd_vel_topic_ = "/cmd_vel";  ///< 指令话题 Command topic
  CommandVelocityMode command_velocity_mode_ =
      CommandVelocityMode::BASE_LINK;  ///< 指令解析模式 Command interpretation mode
  std::string command_frame_id_ = "base_link";  ///< 指令坐标系 Command frame id
  double cmd_vel_timeout_ = 0.25;  ///< 指令超时阈值（秒） Command timeout in seconds
  bool enable_dynamic_reconfigure_ = true;  ///< 是否启用动态调参 Enable dynamic reconfigure
  std::string odom_topic_ = "/odom";  ///< 里程计话题 Odometry topic
  std::string odom_frame_id_ = "odom";  ///< 里程计父坐标系 Odom frame id
  std::string base_frame_id_ = "base_link";  ///< 底盘坐标系 Base frame id
  double odom_startup_hold_sec_ =
      1.0;  ///< 启动静置窗口（秒） Startup settling window in seconds
  double odom_max_linear_speed_ =
      8.0;  ///< 里程计线速度上限（m/s） Odom linear speed limit in m/s
  double odom_max_angular_speed_ =
      16.0;  ///< 里程计角速度上限（rad/s） Odom angular speed limit in rad/s
  bool odom_integrate_on_timeout_ =
      false;  ///< 指令超时时是否继续积分 Odom integration when cmd times out
  bool publish_tf_ = true;  ///< 是否发布 odom->base_link TF Whether to publish odom TF
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;  ///< TF 缓冲区 TF buffer
  std::unique_ptr<tf2_ros::TransformListener>
      tf_listener_;  ///< TF 监听器 TF listener
  OdomState odom_state_;  ///< 累计里程计状态 Integrated odometry state
  ros::Time controller_start_time_;  ///< 控制器启动时刻 Controller start timestamp
};

}  // namespace sentry_chassis_controller

#endif  // SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H_
