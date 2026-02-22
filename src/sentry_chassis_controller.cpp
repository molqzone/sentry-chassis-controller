#include "sentry_chassis_controller/sentry_chassis_controller.h"

#include <geometry_msgs/TransformStamped.h>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <nav_msgs/Odometry.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/LinearMath/Quaternion.h>

#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace sentry_chassis_controller
{
namespace
{
// 运行时最小轮半径保护值，避免除零。
// Runtime lower bound for wheel radius to avoid division by zero.
constexpr double MIN_WHEEL_RADIUS = 1e-9;
constexpr double MIN_VALID_DT = 1e-9;
// 轮序后缀固定为前左、前右、后左、后右，需与配置文件和运动学保持一致。
// Wheel order suffix must stay aligned with config and kinematics.
const std::array<std::string, SentryChassisController::WHEEL_COUNT> WHEEL_NAME_SUFFIX = {
    "front_left", "front_right", "rear_left", "rear_right"};
}  // namespace

bool SentryChassisController::init(hardware_interface::EffortJointInterface* hw,
                                   ros::NodeHandle& nh)
{
  std::vector<std::string> steer_joint_names;
  std::vector<std::string> wheel_joint_names;
  std::vector<double> steer_zero_offsets;

  if (!nh.getParam("steer_joints", steer_joint_names) ||
      steer_joint_names.size() != WHEEL_COUNT)
  {
    ROS_ERROR("Parameter 'steer_joints' must exist and contain exactly %zu items.",
              WHEEL_COUNT);
    return false;
  }

  if (!nh.getParam("wheel_joints", wheel_joint_names) ||
      wheel_joint_names.size() != WHEEL_COUNT)
  {
    ROS_ERROR("Parameter 'wheel_joints' must exist and contain exactly %zu items.",
              WHEEL_COUNT);
    return false;
  }

  if (!nh.getParam("steer_zero_offsets", steer_zero_offsets))
  {
    steer_zero_offsets.assign(WHEEL_COUNT, 0.0);
  }
  if (steer_zero_offsets.size() != WHEEL_COUNT)
  {
    ROS_ERROR("Parameter 'steer_zero_offsets' must contain exactly %zu items.",
              WHEEL_COUNT);
    return false;
  }
  for (std::size_t i = 0; i < WHEEL_COUNT; ++i)
  {
    steer_zero_offsets_[i] = steer_zero_offsets[i];
  }

  Kinematics::Geometry geometry;
  nh.param("geometry/wheel_base", geometry.wheel_base, 0.50);
  nh.param("geometry/track_width", geometry.track_width, 0.40);
  nh.param("geometry/wheel_radius", geometry.wheel_radius, 0.076);
  if (geometry.wheel_radius <= MIN_WHEEL_RADIUS)
  {
    ROS_WARN("Parameter 'geometry/wheel_radius' must be positive. Clamping to %.9f.",
             MIN_WHEEL_RADIUS);
    geometry.wheel_radius = MIN_WHEEL_RADIUS;
  }
  kinematics_.SetGeometry(geometry);

  Kinematics::DirectionSigns direction_signs;
  if (!LoadDirectionSigns(nh, &direction_signs))
  {
    return false;
  }
  kinematics_.SetDirectionSigns(direction_signs);
  if (!LoadRollingSigns(nh, &wheel_rolling_signs_))
  {
    return false;
  }

  nh.param("cmd_vel_topic", cmd_vel_topic_, std::string("/cmd_vel"));
  nh.param("command_frame_id", command_frame_id_, std::string("base_link"));
  nh.param("cmd_vel_timeout", cmd_vel_timeout_, 0.25);
  nh.param("enable_dynamic_reconfigure", enable_dynamic_reconfigure_, true);
  nh.param("odom_topic", odom_topic_, std::string("/odom"));
  nh.param("odom_frame_id", odom_frame_id_, std::string("odom"));
  nh.param("base_frame_id", base_frame_id_, std::string("base_link"));
  nh.param("publish_tf", publish_tf_, true);
  if (command_frame_id_ != "base_link")
  {
    ROS_WARN(
        "Parameter 'command_frame_id' is '%s', but only 'base_link' is supported in this "
        "stage. Falling back to 'base_link'.",
        command_frame_id_.c_str());
    command_frame_id_ = "base_link";
  }
  if (cmd_vel_timeout_ < 0.0)
  {
    ROS_WARN("Parameter 'cmd_vel_timeout' is negative. Clamping to 0.0.");
    cmd_vel_timeout_ = 0.0;
  }

  steer_joints_.clear();
  wheel_joints_.clear();
  steer_joints_.reserve(WHEEL_COUNT);
  wheel_joints_.reserve(WHEEL_COUNT);

  try
  {
    for (const auto& name : steer_joint_names)
    {
      steer_joints_.push_back(hw->getHandle(name));
    }
    for (const auto& name : wheel_joint_names)
    {
      wheel_joints_.push_back(hw->getHandle(name));
    }
  }
  catch (const hardware_interface::HardwareInterfaceException& exception)
  {
    ROS_ERROR("Failed to get joint handle: %s", exception.what());
    return false;
  }

  if (!InitPidGroup(nh, "steer", &steer_pids_))
  {
    return false;
  }
  if (!InitPidGroup(nh, "wheel", &wheel_pids_))
  {
    return false;
  }

  CommandData command;
  command.stamp = ros::Time(0);
  command.valid = false;
  command_buffer_.writeFromNonRT(command);
  odom_state_ = OdomState();

  cmd_vel_subscriber_ =
      nh.subscribe(cmd_vel_topic_, 1, &SentryChassisController::CmdVelCallback, this);
  odom_publisher_ = nh.advertise<nav_msgs::Odometry>(odom_topic_, 10);
  if (publish_tf_)
  {
    tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster());
  }
  else
  {
    tf_broadcaster_.reset();
  }

  ROS_INFO(
      "SentryChassisController initialized with cmd_vel_topic='%s', "
      "command_frame_id='%s', "
      "timeout=%.3fs.",
      cmd_vel_topic_.c_str(), command_frame_id_.c_str(), cmd_vel_timeout_);
  ROS_INFO(
      "odometry output configured with topic='%s', odom_frame='%s', base_frame='%s', "
      "publish_tf=%s.",
      odom_topic_.c_str(), odom_frame_id_.c_str(), base_frame_id_.c_str(),
      publish_tf_ ? "true" : "false");
  ROS_INFO(
      "wheel_direction_signs loaded: vx=[%d,%d,%d,%d], vy=[%d,%d,%d,%d], "
      "wz=[%d,%d,%d,%d].",
      direction_signs.vx[0], direction_signs.vx[1], direction_signs.vx[2],
      direction_signs.vx[3], direction_signs.vy[0], direction_signs.vy[1],
      direction_signs.vy[2], direction_signs.vy[3], direction_signs.wz[0],
      direction_signs.wz[1], direction_signs.wz[2], direction_signs.wz[3]);
  ROS_INFO("wheel_rolling_signs loaded: [%d,%d,%d,%d].", wheel_rolling_signs_[0],
           wheel_rolling_signs_[1], wheel_rolling_signs_[2], wheel_rolling_signs_[3]);
  return true;
}

bool SentryChassisController::IsCommandTimedOut(bool command_valid,
                                                const ros::Time& command_stamp,
                                                const ros::Time& now, double timeout_sec)
{
  if (!command_valid)
  {
    return true;
  }
  if (timeout_sec < 0.0)
  {
    return true;
  }
  const double AGE = (now - command_stamp).toSec();
  return AGE < 0.0 || AGE > timeout_sec;
}

double SentryChassisController::NormalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

SentryChassisController::OdomState SentryChassisController::IntegrateOdom(
    const OdomState& state, const Kinematics::ChassisTwist& twist, double dt)
{
  if (dt <= 0.0)
  {
    return state;
  }

  OdomState result = state;
  const double YAW_MID = result.yaw + 0.5 * twist.wz * dt;
  const double DELTA_X =
      (twist.vx * std::cos(YAW_MID) - twist.vy * std::sin(YAW_MID)) * dt;
  const double DELTA_Y =
      (twist.vx * std::sin(YAW_MID) + twist.vy * std::cos(YAW_MID)) * dt;

  result.x += DELTA_X;
  result.y += DELTA_Y;
  result.yaw = NormalizeAngle(result.yaw + twist.wz * dt);
  return result;
}

void SentryChassisController::starting(const ros::Time& time)
{
  SetAllCommands(&steer_joints_, 0.0);
  SetAllCommands(&wheel_joints_, 0.0);
  for (auto& pid : steer_pids_)
  {
    pid.reset();
  }
  for (auto& pid : wheel_pids_)
  {
    pid.reset();
  }

  CommandData command;
  command.stamp = time;
  command.valid = false;
  command_buffer_.writeFromNonRT(command);
  odom_state_ = OdomState();
}

void SentryChassisController::update(const ros::Time& time, const ros::Duration& period)
{
  const double DT = period.toSec();
  if (DT <= MIN_VALID_DT)
  {
    ROS_WARN_THROTTLE(1.0, "Skip update due to non-positive period.");
    return;
  }

  CommandData command = *command_buffer_.readFromRT();
  const bool TIMEOUT =
      IsCommandTimedOut(command.valid, command.stamp, time, cmd_vel_timeout_);

  // cmd_vel is interpreted in base_link: +x forward, +y left, +z yaw CCW.
  const double VX = TIMEOUT ? 0.0 : command.vx;
  const double VY = TIMEOUT ? 0.0 : command.vy;
  const double WZ = TIMEOUT ? 0.0 : command.wz;
  const Kinematics::WheelTargets WHEEL_TARGETS =
      kinematics_.ComputeWheelAngularVelocity(VX, VY, WZ);
  const std::array<double, WHEEL_COUNT> WHEEL_TARGET_VALUES = {
      WHEEL_TARGETS.front_left, WHEEL_TARGETS.front_right, WHEEL_TARGETS.rear_left,
      WHEEL_TARGETS.rear_right};

  for (std::size_t i = 0; i < WHEEL_COUNT; ++i)
  {
    // 舵向按位置误差闭环，目标值来自 steer_zero_offsets_。
    // Steering loop closes on position error using steer_zero_offsets_ as target.
    const double STEER_ERROR = steer_zero_offsets_[i] - steer_joints_[i].getPosition();
    const double STEER_EFFORT = steer_pids_[i].computeCommand(STEER_ERROR, period);
    steer_joints_[i].setCommand(STEER_EFFORT);
  }

  for (std::size_t i = 0; i < WHEEL_COUNT; ++i)
  {
    // 轮速按速度误差闭环，目标值由逆运动学解算得到。
    // Wheel loop closes on velocity error using IK-computed targets.
    const double WHEEL_ERROR = WHEEL_TARGET_VALUES[i] - wheel_joints_[i].getVelocity();
    const double WHEEL_EFFORT = wheel_pids_[i].computeCommand(WHEEL_ERROR, period);
    wheel_joints_[i].setCommand(WHEEL_EFFORT);
  }

  Kinematics::WheelFeedback feedback;
  for (std::size_t i = 0; i < WHEEL_COUNT; ++i)
  {
    feedback.steer_position[i] = steer_joints_[i].getPosition();
    feedback.wheel_angular_velocity[i] = wheel_joints_[i].getVelocity();
  }

  Kinematics::ChassisTwist odom_twist;
  const bool ODOM_SOLVED = kinematics_.ComputeChassisTwistFromWheelFeedback(
      feedback, steer_zero_offsets_, wheel_rolling_signs_, &odom_twist);
  if (!ODOM_SOLVED)
  {
    ROS_WARN_THROTTLE(1.0,
                      "Forward kinematics matrix is singular. Skip odometry integration "
                      "in this cycle.");
    odom_twist = Kinematics::ChassisTwist();
  }
  else
  {
    odom_state_ = IntegrateOdom(odom_state_, odom_twist, DT);
  }

  PublishOdometry(time, odom_twist);
}

void SentryChassisController::stopping(const ros::Time& time)
{
  (void)time;
  SetAllCommands(&steer_joints_, 0.0);
  SetAllCommands(&wheel_joints_, 0.0);
  for (auto& pid : steer_pids_)
  {
    pid.reset();
  }
  for (auto& pid : wheel_pids_)
  {
    pid.reset();
  }
  odom_state_ = OdomState();
}

void SentryChassisController::CmdVelCallback(const geometry_msgs::TwistConstPtr& message)
{
  CommandData command;
  command.vx = message->linear.x;
  command.vy = message->linear.y;
  command.wz = message->angular.z;
  command.stamp = ros::Time::now();
  command.valid = true;
  command_buffer_.writeFromNonRT(command);
}

bool SentryChassisController::InitPidGroup(
    ros::NodeHandle& nh, const std::string& pid_group,
    std::array<control_toolbox::Pid, WHEEL_COUNT>* pid_array)
{
  for (std::size_t i = 0; i < WHEEL_COUNT; ++i)
  {
    const std::string NS = "pid/" + pid_group + "/" + WHEEL_NAME_SUFFIX[i];
    ros::NodeHandle pid_nh(nh, NS);
    if (enable_dynamic_reconfigure_)
    {
      // 动态调参模式：由 control_toolbox 自动挂载 dynamic_reconfigure 服务。
      // Dynamic mode: control_toolbox internally provides dynamic_reconfigure services.
      if (!pid_array->at(i).init(pid_nh, false))
      {
        ROS_ERROR("Failed to initialize PID at namespace '%s'.",
                  pid_nh.getNamespace().c_str());
        return false;
      }
    }
    else
    {
      // 固定参数模式：直接读取静态 YAML 参数，不注册动态调参接口。
      // Fixed mode: read static YAML parameters without dynamic reconfigure endpoint.
      double p = 0.0;
      double i_gain = 0.0;
      double d = 0.0;
      double i_clamp_min = 0.0;
      double i_clamp_max = 0.0;
      bool antiwindup = false;

      if (!pid_nh.getParam("p", p) || !pid_nh.getParam("i", i_gain) ||
          !pid_nh.getParam("d", d) || !pid_nh.getParam("i_clamp_min", i_clamp_min) ||
          !pid_nh.getParam("i_clamp_max", i_clamp_max))
      {
        ROS_ERROR("Missing PID parameters at namespace '%s'.",
                  pid_nh.getNamespace().c_str());
        return false;
      }
      pid_nh.param("antiwindup", antiwindup, false);
      pid_array->at(i).initPid(p, i_gain, d, i_clamp_max, i_clamp_min, antiwindup);
    }
  }
  return true;
}

bool SentryChassisController::LoadDirectionSigns(
    ros::NodeHandle& nh, Kinematics::DirectionSigns* direction_signs)
{
  std::vector<int> vx_values;
  std::vector<int> vy_values;
  std::vector<int> wz_values;
  const bool HAS_VX = nh.getParam("wheel_direction_signs/vx", vx_values);
  const bool HAS_VY = nh.getParam("wheel_direction_signs/vy", vy_values);
  const bool HAS_WZ = nh.getParam("wheel_direction_signs/wz", wz_values);

  if (!HAS_VX && !HAS_VY && !HAS_WZ)
  {
    // 三轴都未配置时使用全 1 默认值，保持历史行为。
    // Keep legacy behavior when all three axes are omitted.
    *direction_signs = Kinematics::DirectionSigns();
    return true;
  }
  if (!(HAS_VX && HAS_VY && HAS_WZ))
  {
    ROS_ERROR(
        "Parameters 'wheel_direction_signs/vx', 'wheel_direction_signs/vy' and "
        "'wheel_direction_signs/wz' must all be set when any one is provided.");
    return false;
  }

  if (!ParseDirectionAxis(vx_values, "wheel_direction_signs/vx", &direction_signs->vx))
  {
    return false;
  }
  if (!ParseDirectionAxis(vy_values, "wheel_direction_signs/vy", &direction_signs->vy))
  {
    return false;
  }
  if (!ParseDirectionAxis(wz_values, "wheel_direction_signs/wz", &direction_signs->wz))
  {
    return false;
  }
  return true;
}

bool SentryChassisController::LoadRollingSigns(
    ros::NodeHandle& nh, std::array<int, WHEEL_COUNT>* rolling_signs)
{
  std::vector<int> raw_values;
  if (!nh.getParam("wheel_rolling_signs", raw_values))
  {
    ROS_ERROR("Parameter 'wheel_rolling_signs' must be provided.");
    return false;
  }
  return ParseDirectionAxis(raw_values, "wheel_rolling_signs", rolling_signs);
}

bool SentryChassisController::ParseDirectionAxis(const std::vector<int>& axis_values,
                                                 const std::string& param_name,
                                                 std::array<int, WHEEL_COUNT>* output)
{
  // 参数必须完整覆盖四个轮子，且每项仅允许 {-1, 1}。
  // Each axis must provide exactly four entries and each value must be {-1, 1}.
  if (axis_values.size() != WHEEL_COUNT)
  {
    ROS_ERROR("Parameter '%s' must contain exactly %zu items.", param_name.c_str(),
              WHEEL_COUNT);
    return false;
  }

  for (std::size_t i = 0; i < WHEEL_COUNT; ++i)
  {
    if (axis_values[i] != -1 && axis_values[i] != 1)
    {
      ROS_ERROR("Parameter '%s[%zu]' must be -1 or 1, got %d.", param_name.c_str(), i,
                axis_values[i]);
      return false;
    }
    output->at(i) = axis_values[i];
  }
  return true;
}

void SentryChassisController::PublishOdometry(const ros::Time& time,
                                              const Kinematics::ChassisTwist& twist)
{
  nav_msgs::Odometry odometry;
  odometry.header.stamp = time;
  odometry.header.frame_id = odom_frame_id_;
  odometry.child_frame_id = base_frame_id_;
  odometry.pose.pose.position.x = odom_state_.x;
  odometry.pose.pose.position.y = odom_state_.y;
  odometry.pose.pose.position.z = 0.0;

  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, odom_state_.yaw);
  quaternion.normalize();

  odometry.pose.pose.orientation.x = quaternion.x();
  odometry.pose.pose.orientation.y = quaternion.y();
  odometry.pose.pose.orientation.z = quaternion.z();
  odometry.pose.pose.orientation.w = quaternion.w();
  odometry.twist.twist.linear.x = twist.vx;
  odometry.twist.twist.linear.y = twist.vy;
  odometry.twist.twist.angular.z = twist.wz;
  odom_publisher_.publish(odometry);

  if (!publish_tf_ || !tf_broadcaster_)
  {
    return;
  }

  geometry_msgs::TransformStamped transform;
  transform.header.stamp = time;
  transform.header.frame_id = odom_frame_id_;
  transform.child_frame_id = base_frame_id_;
  transform.transform.translation.x = odom_state_.x;
  transform.transform.translation.y = odom_state_.y;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation = odometry.pose.pose.orientation;
  tf_broadcaster_->sendTransform(transform);
}

void SentryChassisController::SetAllCommands(
    std::vector<hardware_interface::JointHandle>* joints, double command)
{
  for (auto& joint : *joints)
  {
    joint.setCommand(command);
  }
}

}  // namespace sentry_chassis_controller

PLUGINLIB_EXPORT_CLASS(sentry_chassis_controller::SentryChassisController,
                       controller_interface::ControllerBase)
