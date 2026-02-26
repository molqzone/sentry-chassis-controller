#include "sentry_chassis_controller/sentry_chassis_controller.h"

#include <Eigen/Dense>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <nav_msgs/Odometry.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <array>
#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace sentry_chassis_controller
{
namespace
{
// 运行时最小轮半径保护值，避免除零。
// Runtime lower bound for wheel radius to avoid division by zero.
constexpr double MIN_WHEEL_RADIUS = 1e-9;
constexpr double MIN_VALID_DT = 1e-9;
constexpr double TF_LOOKUP_TIMEOUT_SEC = 0.02;
constexpr double ZERO_CMD_EPS = 1e-4;
constexpr double GLOBAL_ALIGNMENT_GATE = 0.20;
constexpr double PI = 3.14159265358979323846;
constexpr double HALF_PI = PI * 0.5;
// 轮序后缀固定为前左、前右、后左、后右，需与配置文件和运动学保持一致。
// Wheel order suffix must stay aligned with config and kinematics.
const std::array<std::string, SentryChassisController::WHEEL_COUNT> WHEEL_NAME_SUFFIX = {
    "front_left", "front_right", "rear_left", "rear_right"};

template <typename T, std::size_t N>
bool CopyVectorToArray(const std::vector<T>& source, std::array<T, N>* output)
{
  if (output == nullptr || source.size() != N)
  {
    return false;
  }

  for (std::size_t i = 0; i < N; ++i)
  {
    output->at(i) = source[i];
  }
  return true;
}

template <typename T, std::size_t N>
bool LoadRequiredArrayParam(ros::NodeHandle& nh, const std::string& param_name,
                            std::array<T, N>* output)
{
  std::vector<T> raw_values;
  if (!nh.getParam(param_name, raw_values) || raw_values.size() != N)
  {
    ROS_ERROR("Parameter '%s' must exist and contain exactly %zu items.",
              param_name.c_str(), N);
    return false;
  }
  return CopyVectorToArray(raw_values, output);
}

template <typename T, std::size_t N>
bool LoadOptionalArrayParam(ros::NodeHandle& nh, const std::string& param_name,
                            const std::array<T, N>& default_values,
                            std::array<T, N>* output)
{
  std::vector<T> raw_values;
  if (!nh.getParam(param_name, raw_values))
  {
    *output = default_values;
    return true;
  }

  if (raw_values.size() != N)
  {
    ROS_ERROR("Parameter '%s' must contain exactly %zu items.", param_name.c_str(), N);
    return false;
  }
  return CopyVectorToArray(raw_values, output);
}

void ClampNonNegativeParam(const char* param_name, double* value)
{
  if (*value >= 0.0)
  {
    return;
  }
  ROS_WARN("Parameter '%s' is negative. Clamping to 0.0.", param_name);
  *value = 0.0;
}

void ClampPositiveParam(const char* param_name, double fallback, double* value)
{
  if (*value > 0.0)
  {
    return;
  }
  ROS_WARN("Parameter '%s' must be positive. Clamping to %.3f.", param_name, fallback);
  *value = fallback;
}

void ClampMinParam(const char* param_name, double min_value, double* value)
{
  if (*value >= min_value)
  {
    return;
  }
  ROS_WARN("Parameter '%s' must be >= %.9f. Clamping to %.9f.", param_name, min_value,
           min_value);
  *value = min_value;
}

Kinematics::ChassisTwist ApplyCommandCompensation(
    const std::array<double, 9>& matrix, const Kinematics::ChassisTwist& input)
{
  const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> COMPENSATION(
      matrix.data());
  const Eigen::Vector3d INPUT(input.vx, input.vy, input.wz);
  const Eigen::Vector3d OUTPUT = COMPENSATION * INPUT;

  Kinematics::ChassisTwist output;
  output.vx = OUTPUT.x();
  output.vy = OUTPUT.y();
  output.wz = OUTPUT.z();
  return output;
}
}  // namespace

bool SentryChassisController::init(hardware_interface::EffortJointInterface* hw,
                                   ros::NodeHandle& nh)
{
  std::array<std::string, WHEEL_COUNT> steer_joint_names{};
  std::array<std::string, WHEEL_COUNT> wheel_joint_names{};
  const std::array<double, WHEEL_COUNT> DEFAULT_STEER_ZERO_OFFSETS = {{0.0, 0.0, 0.0, 0.0}};

  if (!LoadRequiredArrayParam(nh, "steer_joints", &steer_joint_names))
  {
    return false;
  }
  if (!LoadRequiredArrayParam(nh, "wheel_joints", &wheel_joint_names))
  {
    return false;
  }
  if (!LoadOptionalArrayParam(
          nh, "steer_zero_offsets", DEFAULT_STEER_ZERO_OFFSETS, &steer_zero_offsets_))
  {
    return false;
  }

  Kinematics::Geometry geometry;
  nh.param("geometry/wheel_base", geometry.wheel_base, 0.50);
  nh.param("geometry/track_width", geometry.track_width, 0.40);
  nh.param("geometry/wheel_radius", geometry.wheel_radius, 0.076);
  ClampMinParam("geometry/wheel_radius", MIN_WHEEL_RADIUS, &geometry.wheel_radius);
  geometry_ = geometry;
  kinematics_.SetGeometry(geometry);

  Kinematics::DirectionSigns direction_signs;
  if (!LoadDirectionSigns(nh, &direction_signs))
  {
    return false;
  }
  direction_signs_ = direction_signs;
  kinematics_.SetDirectionSigns(direction_signs);
  if (!LoadRollingSigns(nh, &wheel_rolling_signs_))
  {
    return false;
  }

  nh.param("cmd_vel_topic", cmd_vel_topic_, std::string("/cmd_vel"));
  std::string command_velocity_mode_text = "base_link";
  nh.param("command_velocity_mode", command_velocity_mode_text,
           std::string("base_link"));
  nh.param("command_frame_id", command_frame_id_, std::string("base_link"));
  nh.param("cmd_vel_timeout", cmd_vel_timeout_, 0.25);
  nh.param("enable_dynamic_reconfigure", enable_dynamic_reconfigure_, true);
  nh.param("odom_topic", odom_topic_, std::string("/odom"));
  nh.param("odom_frame_id", odom_frame_id_, std::string("odom"));
  nh.param("base_frame_id", base_frame_id_, std::string("base_link"));
  nh.param("odom_startup_hold_sec", odom_startup_hold_sec_, 1.0);
  nh.param("odom_max_linear_speed", odom_max_linear_speed_, 8.0);
  nh.param("odom_max_angular_speed", odom_max_angular_speed_, 16.0);
  nh.param("odom_integrate_on_timeout", odom_integrate_on_timeout_, false);
  nh.param("publish_tf", publish_tf_, true);
  nh.param("wheel_effort_limit", wheel_effort_limit_, 12.0);
  if (!LoadOptionalArrayParam(
          nh, "command_compensation_matrix", command_compensation_matrix_,
          &command_compensation_matrix_))
  {
    return false;
  }

  if (!ParseCommandVelocityMode(command_velocity_mode_text, &command_velocity_mode_))
  {
    ROS_ERROR(
        "Parameter 'command_velocity_mode' must be 'base_link' or 'global', got '%s'.",
        command_velocity_mode_text.c_str());
    return false;
  }
  if (command_velocity_mode_ == CommandVelocityMode::BASE_LINK &&
      command_frame_id_ != base_frame_id_)
  {
    ROS_WARN(
        "Parameter 'command_frame_id' is '%s' while command_velocity_mode is "
        "'base_link'. Falling back to '%s'.",
        command_frame_id_.c_str(), base_frame_id_.c_str());
    command_frame_id_ = base_frame_id_;
  }
  if (command_velocity_mode_ == CommandVelocityMode::GLOBAL &&
      command_frame_id_ == base_frame_id_)
  {
    ROS_WARN(
        "command_velocity_mode is 'global' but command_frame_id equals base frame '%s'. "
        "Global transform will have no effect.",
        base_frame_id_.c_str());
  }
  ClampNonNegativeParam("cmd_vel_timeout", &cmd_vel_timeout_);
  ClampNonNegativeParam("odom_startup_hold_sec", &odom_startup_hold_sec_);
  ClampPositiveParam("odom_max_linear_speed", 8.0, &odom_max_linear_speed_);
  ClampPositiveParam("odom_max_angular_speed", 16.0, &odom_max_angular_speed_);
  ClampPositiveParam("wheel_effort_limit", 12.0, &wheel_effort_limit_);

  try
  {
    for (std::size_t i = 0; i < WHEEL_COUNT; ++i)
    {
      steer_joints_[i] = hw->getHandle(steer_joint_names[i]);
      wheel_joints_[i] = hw->getHandle(wheel_joint_names[i]);
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
  controller_start_time_ = ros::Time(0);
  last_command_timed_out_ = true;

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
  if (command_velocity_mode_ == CommandVelocityMode::GLOBAL)
  {
    tf_buffer_.reset(new tf2_ros::Buffer(ros::Duration(10.0)));
    tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));
  }
  else
  {
    tf_buffer_.reset();
    tf_listener_.reset();
  }

  ROS_INFO(
      "SentryChassisController initialized with cmd_vel_topic='%s', "
      "command_velocity_mode='%s', command_frame_id='%s', "
      "timeout=%.3fs.",
      cmd_vel_topic_.c_str(),
      command_velocity_mode_ == CommandVelocityMode::BASE_LINK ? "base_link"
                                                                : "global",
      command_frame_id_.c_str(), cmd_vel_timeout_);
  ROS_INFO(
      "odometry output configured with topic='%s', odom_frame='%s', base_frame='%s', "
      "publish_tf=%s.",
      odom_topic_.c_str(), odom_frame_id_.c_str(), base_frame_id_.c_str(),
      publish_tf_ ? "true" : "false");
  ROS_INFO(
      "odometry stabilization configured with startup_hold=%.3fs, max_linear=%.3fm/s, "
      "max_angular=%.3frad/s, integrate_on_timeout=%s.",
      odom_startup_hold_sec_, odom_max_linear_speed_, odom_max_angular_speed_,
      odom_integrate_on_timeout_ ? "true" : "false");
  ROS_INFO("wheel effort command limit configured as %.3f.", wheel_effort_limit_);
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

bool SentryChassisController::TransformTwistWithTransform(
    const Kinematics::ChassisTwist& input,
    const geometry_msgs::TransformStamped& transform, Kinematics::ChassisTwist* output)
{
  if (output == nullptr)
  {
    return false;
  }

  geometry_msgs::Vector3Stamped linear_input;
  linear_input.header.stamp = transform.header.stamp;
  linear_input.header.frame_id = transform.child_frame_id;
  linear_input.vector.x = input.vx;
  linear_input.vector.y = input.vy;
  linear_input.vector.z = 0.0;

  geometry_msgs::Vector3Stamped linear_output;
  tf2::doTransform(linear_input, linear_output, transform);

  geometry_msgs::Vector3Stamped angular_input;
  angular_input.header.stamp = transform.header.stamp;
  angular_input.header.frame_id = transform.child_frame_id;
  angular_input.vector.x = 0.0;
  angular_input.vector.y = 0.0;
  angular_input.vector.z = input.wz;

  geometry_msgs::Vector3Stamped angular_output;
  tf2::doTransform(angular_input, angular_output, transform);

  output->vx = linear_output.vector.x;
  output->vy = linear_output.vector.y;
  output->wz = angular_output.vector.z;
  return true;
}

bool SentryChassisController::ParseCommandVelocityMode(
    const std::string& mode_text, CommandVelocityMode* mode)
{
  if (mode == nullptr)
  {
    return false;
  }

  if (mode_text == "base_link")
  {
    *mode = CommandVelocityMode::BASE_LINK;
    return true;
  }
  if (mode_text == "global")
  {
    *mode = CommandVelocityMode::GLOBAL;
    return true;
  }

  return false;
}

bool SentryChassisController::ResolveCommandInBaseFrame(const CommandData& command,
                                                        const ros::Time& time,
                                                        Kinematics::ChassisTwist* base_twist)
{
  if (base_twist == nullptr)
  {
    return false;
  }

  Kinematics::ChassisTwist source_twist;
  source_twist.vx = command.vx;
  source_twist.vy = command.vy;
  source_twist.wz = command.wz;

  if (command_velocity_mode_ == CommandVelocityMode::BASE_LINK)
  {
    *base_twist = source_twist;
    return true;
  }

  if (!tf_buffer_)
  {
    ROS_WARN_THROTTLE(
        1.0,
        "Global command mode requires TF listener, but tf_buffer is not initialized.");
    return false;
  }

  geometry_msgs::TransformStamped command_to_base_transform;
  try
  {
    command_to_base_transform = tf_buffer_->lookupTransform(
        base_frame_id_, command_frame_id_, time, ros::Duration(TF_LOOKUP_TIMEOUT_SEC));
  }
  catch (const tf2::TransformException& exception)
  {
    ROS_WARN_THROTTLE(1.0,
                      "Failed to transform cmd_vel from '%s' to '%s': %s",
                      command_frame_id_.c_str(), base_frame_id_.c_str(),
                      exception.what());
    return false;
  }

  return TransformTwistWithTransform(source_twist, command_to_base_transform, base_twist);
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
  controller_start_time_ = time;
  last_command_timed_out_ = true;
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
  if (TIMEOUT && !last_command_timed_out_)
  {
    for (auto& pid : wheel_pids_)
    {
      pid.reset();
    }
  }
  last_command_timed_out_ = TIMEOUT;

  // Resolve cmd_vel into base frame before IK:
  // - base_link mode: direct use
  // - global mode: transform command_frame -> base_frame via TF.
  Kinematics::ChassisTwist command_twist_base{};
  if (!TIMEOUT)
  {
    const bool COMMAND_RESOLVED =
        ResolveCommandInBaseFrame(command, time, &command_twist_base);
    if (!COMMAND_RESOLVED)
    {
      command_twist_base = Kinematics::ChassisTwist();
    }
  }

  const Kinematics::ChassisTwist COMPENSATED_COMMAND =
      ApplyCommandCompensation(command_compensation_matrix_, command_twist_base);
  const double VX = COMPENSATED_COMMAND.vx;
  const double VY = COMPENSATED_COMMAND.vy;
  const double WZ = COMPENSATED_COMMAND.wz;

  const bool ZERO_CMD_REQUESTED =
      std::fabs(VX) < ZERO_CMD_EPS && std::fabs(VY) < ZERO_CMD_EPS &&
      std::fabs(WZ) < ZERO_CMD_EPS;
  const double HALF_WHEEL_BASE = geometry_.wheel_base * 0.5;
  const double HALF_TRACK_WIDTH = geometry_.track_width * 0.5;
  const double WHEEL_RADIUS =
      geometry_.wheel_radius > MIN_WHEEL_RADIUS ? geometry_.wheel_radius : MIN_WHEEL_RADIUS;
  const std::array<std::pair<double, double>, WHEEL_COUNT> MODULE_POSITIONS = {{
      {HALF_WHEEL_BASE, HALF_TRACK_WIDTH},
      {HALF_WHEEL_BASE, -HALF_TRACK_WIDTH},
      {-HALF_WHEEL_BASE, HALF_TRACK_WIDTH},
      {-HALF_WHEEL_BASE, -HALF_TRACK_WIDTH},
  }};
  std::array<double, WHEEL_COUNT> STEER_ERRORS{};
  std::array<double, WHEEL_COUNT> WHEEL_TARGET_VALUES{};
  std::array<double, WHEEL_COUNT> ALIGNMENTS{};

  for (std::size_t i = 0; i < WHEEL_COUNT; ++i)
  {
    double steer_error =
        NormalizeAngle(steer_zero_offsets_[i] - steer_joints_[i].getPosition());
    double wheel_target = 0.0;
    double alignment = 1.0;
    if (!ZERO_CMD_REQUESTED)
    {
      // Swerve IK: project chassis twist to each module velocity, then convert
      // to target steering angle and wheel speed.
      const double MODULE_X = MODULE_POSITIONS[i].first;
      const double MODULE_Y = MODULE_POSITIONS[i].second;
      const double SIGNED_VX = static_cast<double>(direction_signs_.vx[i]) * VX;
      const double SIGNED_VY = static_cast<double>(direction_signs_.vy[i]) * VY;
      const double SIGNED_WZ = static_cast<double>(direction_signs_.wz[i]) * WZ;
      const double MODULE_VX = SIGNED_VX - SIGNED_WZ * MODULE_Y;
      const double MODULE_VY = SIGNED_VY + SIGNED_WZ * MODULE_X;
      const double MODULE_SPEED = std::hypot(MODULE_VX, MODULE_VY);
      if (MODULE_SPEED > ZERO_CMD_EPS)
      {
        const double TARGET_STEER =
            std::atan2(MODULE_VY, MODULE_VX) + steer_zero_offsets_[i];
        steer_error = NormalizeAngle(TARGET_STEER - steer_joints_[i].getPosition());
        wheel_target = MODULE_SPEED / WHEEL_RADIUS;

        // Flip wheel direction if turning more than 90deg to reach target.
        if (steer_error > HALF_PI)
        {
          steer_error -= PI;
          wheel_target = -wheel_target;
        }
        else if (steer_error < -HALF_PI)
        {
          steer_error += PI;
          wheel_target = -wheel_target;
        }

        // Reduce wheel speed while steering is not aligned to avoid transient spin.
        alignment = std::max(0.0, std::cos(steer_error));
        wheel_target *= alignment;
      }
    }
    STEER_ERRORS[i] = steer_error;
    WHEEL_TARGET_VALUES[i] = wheel_target;
    ALIGNMENTS[i] = alignment;

    // 舵向按位置误差闭环，目标值按每轮目标舵角实时计算。
    // Steering loop closes on per-wheel target steering angle error.
    const double STEER_EFFORT = steer_pids_[i].computeCommand(STEER_ERRORS[i], period);
    steer_joints_[i].setCommand(STEER_EFFORT);
  }

  if (ZERO_CMD_REQUESTED)
  {
    for (auto& pid : wheel_pids_)
    {
      pid.reset();
    }
    SetAllCommands(&wheel_joints_, 0.0);
  }
  else
  {
    double global_alignment = 1.0;
    for (const double alignment : ALIGNMENTS)
    {
      global_alignment = std::min(global_alignment, alignment);
    }
    if (global_alignment < GLOBAL_ALIGNMENT_GATE)
    {
      global_alignment = 0.0;
    }

    for (std::size_t i = 0; i < WHEEL_COUNT; ++i)
    {
      // 轮速按速度误差闭环，目标值由逆运动学解算得到。
      // Wheel loop closes on velocity error using IK-computed targets.
      const double ROLLING_SIGN = static_cast<double>(wheel_rolling_signs_[i]);
      const double SIGNED_WHEEL_VELOCITY = ROLLING_SIGN * wheel_joints_[i].getVelocity();
      const double WHEEL_TARGET = WHEEL_TARGET_VALUES[i] * global_alignment;
      const double WHEEL_ERROR = WHEEL_TARGET - SIGNED_WHEEL_VELOCITY;
      const double SIGNED_WHEEL_EFFORT = wheel_pids_[i].computeCommand(WHEEL_ERROR, period);
      const double LIMITED_SIGNED_WHEEL_EFFORT =
          std::max(-wheel_effort_limit_, std::min(wheel_effort_limit_, SIGNED_WHEEL_EFFORT));
      wheel_joints_[i].setCommand(ROLLING_SIGN * LIMITED_SIGNED_WHEEL_EFFORT);
    }
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
    if (TIMEOUT && !odom_integrate_on_timeout_)
    {
      odom_twist = Kinematics::ChassisTwist();
    }
    else
    {
      const double STARTUP_AGE = (time - controller_start_time_).toSec();
      const bool IN_STARTUP_HOLD =
          STARTUP_AGE >= 0.0 && STARTUP_AGE < odom_startup_hold_sec_;
      const bool SHOULD_SUPPRESS_STARTUP_DRIFT =
          odom_integrate_on_timeout_ && TIMEOUT && IN_STARTUP_HOLD;

      if (SHOULD_SUPPRESS_STARTUP_DRIFT)
      {
        odom_twist = Kinematics::ChassisTwist();
        ROS_WARN_THROTTLE(
            1.0,
            "Suppress odometry integration during startup hold window (%.3fs remaining).",
            odom_startup_hold_sec_ - STARTUP_AGE);
      }
      else if (!IsOdomTwistAcceptable(odom_twist))
      {
        ROS_WARN_THROTTLE(
            1.0,
            "Reject abnormal odom twist (vx=%.3f, vy=%.3f, wz=%.3f). Check "
            "steering alignment/rolling signs.",
            odom_twist.vx, odom_twist.vy, odom_twist.wz);
        odom_twist = Kinematics::ChassisTwist();
      }
      else
      {
        odom_state_ = IntegrateOdom(odom_state_, odom_twist, DT);
      }
    }
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
  controller_start_time_ = ros::Time(0);
  last_command_timed_out_ = true;
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

bool SentryChassisController::IsOdomTwistAcceptable(
    const Kinematics::ChassisTwist& twist) const
{
  if (!std::isfinite(twist.vx) || !std::isfinite(twist.vy) || !std::isfinite(twist.wz))
  {
    return false;
  }

  const double LINEAR_SPEED = std::hypot(twist.vx, twist.vy);
  const double ANGULAR_SPEED = std::fabs(twist.wz);
  return LINEAR_SPEED <= odom_max_linear_speed_ && ANGULAR_SPEED <= odom_max_angular_speed_;
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
    std::array<hardware_interface::JointHandle, WHEEL_COUNT>* joints, double command)
{
  for (auto& joint : *joints)
  {
    joint.setCommand(command);
  }
}

}  // namespace sentry_chassis_controller

PLUGINLIB_EXPORT_CLASS(sentry_chassis_controller::SentryChassisController,
                       controller_interface::ControllerBase)
