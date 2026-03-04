#include "sentry_chassis_controller/sentry_chassis_controller.h"
#include "sentry_chassis_controller/controller_math.hpp"
#include "controller_internal.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>

namespace sentry_chassis_controller
{
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

bool SentryChassisController::ValidateAndApplyControllerParams(bool strict_validation)
{
  if (geometry_.wheel_radius < MIN_WHEEL_RADIUS)
  {
    ROS_WARN("Parameter 'geometry/wheel_radius' must be >= %.9f. Clamping to %.9f.",
             MIN_WHEEL_RADIUS, MIN_WHEEL_RADIUS);
    geometry_.wheel_radius = MIN_WHEEL_RADIUS;
  }
  if (cmd_vel_timeout_ < 0.0)
  {
    ROS_WARN("Parameter 'cmd_vel_timeout' is negative. Clamping to 0.0.");
    cmd_vel_timeout_ = 0.0;
  }
  if (odom_startup_hold_sec_ < 0.0)
  {
    ROS_WARN("Parameter 'odom_startup_hold_sec' is negative. Clamping to 0.0.");
    odom_startup_hold_sec_ = 0.0;
  }
  if (odom_max_linear_speed_ <= 0.0)
  {
    ROS_WARN("Parameter 'odom_max_linear_speed' must be positive. Clamping to %.3f.",
             DEFAULT_ODOM_MAX_LINEAR_SPEED);
    odom_max_linear_speed_ = DEFAULT_ODOM_MAX_LINEAR_SPEED;
  }
  if (odom_max_angular_speed_ <= 0.0)
  {
    ROS_WARN("Parameter 'odom_max_angular_speed' must be positive. Clamping to %.3f.",
             DEFAULT_ODOM_MAX_ANGULAR_SPEED);
    odom_max_angular_speed_ = DEFAULT_ODOM_MAX_ANGULAR_SPEED;
  }
  if (wheel_effort_limit_ <= 0.0)
  {
    ROS_WARN("Parameter 'wheel_effort_limit' must be positive. Clamping to %.3f.",
             DEFAULT_WHEEL_EFFORT_LIMIT);
    wheel_effort_limit_ = DEFAULT_WHEEL_EFFORT_LIMIT;
  }
  if (reverse_ccw_vx_scale_ < MIN_REVERSE_CCW_VX_SCALE ||
      reverse_ccw_vx_scale_ > MAX_REVERSE_CCW_VX_SCALE)
  {
    ROS_WARN(
        "Parameter 'reverse_ccw_vx_scale' must be in [%.3f, %.3f]. Clamping.",
        MIN_REVERSE_CCW_VX_SCALE, MAX_REVERSE_CCW_VX_SCALE);
    reverse_ccw_vx_scale_ =
        std::max(MIN_REVERSE_CCW_VX_SCALE,
                 std::min(MAX_REVERSE_CCW_VX_SCALE, reverse_ccw_vx_scale_));
  }
  if (reverse_ccw_wz_gain_ < MIN_REVERSE_CCW_WZ_GAIN ||
      reverse_ccw_wz_gain_ > MAX_REVERSE_CCW_WZ_GAIN)
  {
    ROS_WARN(
        "Parameter 'reverse_ccw_wz_gain' must be in [%.3f, %.3f]. Clamping.",
        MIN_REVERSE_CCW_WZ_GAIN, MAX_REVERSE_CCW_WZ_GAIN);
    reverse_ccw_wz_gain_ =
        std::max(MIN_REVERSE_CCW_WZ_GAIN,
                 std::min(MAX_REVERSE_CCW_WZ_GAIN, reverse_ccw_wz_gain_));
  }
  if (reverse_ccw_vy_threshold_ < MIN_REVERSE_CCW_VY_THRESHOLD ||
      reverse_ccw_vy_threshold_ > MAX_REVERSE_CCW_VY_THRESHOLD)
  {
    ROS_WARN(
        "Parameter 'reverse_ccw_vy_threshold' must be in [%.3f, %.3f]. Clamping.",
        MIN_REVERSE_CCW_VY_THRESHOLD, MAX_REVERSE_CCW_VY_THRESHOLD);
    reverse_ccw_vy_threshold_ =
        std::max(MIN_REVERSE_CCW_VY_THRESHOLD,
                 std::min(MAX_REVERSE_CCW_VY_THRESHOLD, reverse_ccw_vy_threshold_));
  }
  if (reverse_ccw_steer_priority_error_ < MIN_REVERSE_CCW_STEER_PRIORITY_ERROR ||
      reverse_ccw_steer_priority_error_ > MAX_REVERSE_CCW_STEER_PRIORITY_ERROR)
  {
    ROS_WARN(
        "Parameter 'reverse_ccw_steer_priority_error' must be in [%.3f, %.3f]. "
        "Clamping.",
        MIN_REVERSE_CCW_STEER_PRIORITY_ERROR,
        MAX_REVERSE_CCW_STEER_PRIORITY_ERROR);
    reverse_ccw_steer_priority_error_ =
        std::max(MIN_REVERSE_CCW_STEER_PRIORITY_ERROR,
                 std::min(MAX_REVERSE_CCW_STEER_PRIORITY_ERROR,
                          reverse_ccw_steer_priority_error_));
  }
  if (max_linear_acceleration_ <= 0.0)
  {
    ROS_WARN("Parameter 'max_linear_acceleration' must be positive. Clamping to %.3f.",
             DEFAULT_MAX_LINEAR_ACCELERATION);
    max_linear_acceleration_ = DEFAULT_MAX_LINEAR_ACCELERATION;
  }
  if (max_angular_acceleration_ <= 0.0)
  {
    ROS_WARN(
        "Parameter 'max_angular_acceleration' must be positive. Clamping to %.3f.",
        DEFAULT_MAX_ANGULAR_ACCELERATION);
    max_angular_acceleration_ = DEFAULT_MAX_ANGULAR_ACCELERATION;
  }
  if (max_power_ <= 0.0)
  {
    ROS_WARN("Parameter 'max_power' must be positive. Clamping to %.3f.",
             DEFAULT_MAX_POWER);
    max_power_ = DEFAULT_MAX_POWER;
  }
  if (power_loss_k1_ < 0.0)
  {
    ROS_WARN("Parameter 'power_loss_k1' must be non-negative. Clamping to %.6f.",
             DEFAULT_POWER_LOSS_K1);
    power_loss_k1_ = DEFAULT_POWER_LOSS_K1;
  }
  if (power_loss_k2_ < 0.0)
  {
    ROS_WARN("Parameter 'power_loss_k2' must be non-negative. Clamping to %.6f.",
             DEFAULT_POWER_LOSS_K2);
    power_loss_k2_ = DEFAULT_POWER_LOSS_K2;
  }
  if (min_power_scale_ < MIN_POWER_SCALE || min_power_scale_ > MAX_POWER_SCALE)
  {
    ROS_WARN("Parameter 'min_power_scale' must be in [%.3f, %.3f]. Clamping.",
             MIN_POWER_SCALE, MAX_POWER_SCALE);
    min_power_scale_ = std::max(MIN_POWER_SCALE,
                                std::min(MAX_POWER_SCALE, min_power_scale_));
  }

  CommandVelocityMode parsed_mode = command_velocity_mode_;
  if (!ParseCommandVelocityMode(command_velocity_mode_text_, &parsed_mode))
  {
    if (strict_validation)
    {
      ROS_ERROR(
          "Parameter 'command_velocity_mode' must be 'base_link' or 'global', got '%s'.",
          command_velocity_mode_text_.c_str());
      return false;
    }
    ROS_WARN_THROTTLE(
        1.0,
        "Dynamic command_velocity_mode '%s' is invalid. Keeping previous mode.",
        command_velocity_mode_text_.c_str());
    command_velocity_mode_text_ =
        command_velocity_mode_ == CommandVelocityMode::BASE_LINK ? "base_link" : "global";
    parsed_mode = command_velocity_mode_;
  }
  command_velocity_mode_ = parsed_mode;

  if (base_frame_id_.empty())
  {
    ROS_WARN("Parameter 'base_frame_id' is empty. Falling back to 'base_link'.");
    base_frame_id_ = "base_link";
  }
  if (odom_frame_id_.empty())
  {
    ROS_WARN("Parameter 'odom_frame_id' is empty. Falling back to 'odom'.");
    odom_frame_id_ = "odom";
  }
  if (command_frame_id_.empty())
  {
    const std::string FALLBACK_COMMAND_FRAME =
        command_velocity_mode_ == CommandVelocityMode::BASE_LINK ? base_frame_id_
                                                                 : odom_frame_id_;
    ROS_WARN("Parameter 'command_frame_id' is empty. Falling back to '%s'.",
             FALLBACK_COMMAND_FRAME.c_str());
    command_frame_id_ = FALLBACK_COMMAND_FRAME;
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
    ROS_WARN_THROTTLE(
        1.0,
        "command_velocity_mode is 'global' but command_frame_id equals base frame '%s'. "
        "Global transform will have no effect.",
        base_frame_id_.c_str());
  }
  runtime_params_shadow_.command_velocity_mode = command_velocity_mode_;
  runtime_params_shadow_.command_frame_id = command_frame_id_;
  runtime_params_shadow_.odom_frame_id = odom_frame_id_;
  runtime_params_shadow_.base_frame_id = base_frame_id_;
  runtime_params_shadow_.cmd_vel_timeout = cmd_vel_timeout_;
  runtime_params_shadow_.odom_startup_hold_sec = odom_startup_hold_sec_;
  runtime_params_shadow_.odom_max_linear_speed = odom_max_linear_speed_;
  runtime_params_shadow_.odom_max_angular_speed = odom_max_angular_speed_;
  runtime_params_shadow_.odom_integrate_on_timeout = odom_integrate_on_timeout_;
  runtime_params_shadow_.publish_tf = publish_tf_;
  runtime_params_shadow_.wheel_effort_limit = wheel_effort_limit_;
  runtime_params_shadow_.reverse_ccw_vx_scale = reverse_ccw_vx_scale_;
  runtime_params_shadow_.reverse_ccw_wz_gain = reverse_ccw_wz_gain_;
  runtime_params_shadow_.reverse_ccw_vy_threshold = reverse_ccw_vy_threshold_;
  runtime_params_shadow_.reverse_ccw_steer_priority_error =
      reverse_ccw_steer_priority_error_;
  runtime_params_shadow_.enable_acceleration_limits = enable_acceleration_limits_;
  runtime_params_shadow_.max_linear_acceleration = max_linear_acceleration_;
  runtime_params_shadow_.max_angular_acceleration = max_angular_acceleration_;
  runtime_params_shadow_.enable_power_limit = enable_power_limit_;
  runtime_params_shadow_.enable_power_limit_logging = enable_power_limit_logging_;
  runtime_params_shadow_.max_power = max_power_;
  runtime_params_shadow_.power_loss_k1 = power_loss_k1_;
  runtime_params_shadow_.power_loss_k2 = power_loss_k2_;
  runtime_params_shadow_.min_power_scale = min_power_scale_;
  runtime_params_shadow_.geometry = geometry_;
  runtime_params_buffer_.writeFromNonRT(runtime_params_shadow_);
  InvalidateCommandTransformCache();
  RefreshCommandTransformCache(ros::TimerEvent());
  return true;
}

void SentryChassisController::InvalidateCommandTransformCache()
{
  CommandTransformCache cache;
  cache.valid = false;
  command_transform_buffer_.writeFromNonRT(cache);
}

void SentryChassisController::RefreshCommandTransformCache(
    const ros::TimerEvent& event)
{
  (void)event;
  const RuntimeParams* runtime_params = runtime_params_buffer_.readFromRT();
  if (runtime_params == nullptr)
  {
    InvalidateCommandTransformCache();
    return;
  }

  if (runtime_params->command_velocity_mode != CommandVelocityMode::GLOBAL)
  {
    InvalidateCommandTransformCache();
    return;
  }

  CommandTransformCache cache;
  cache.valid = false;
  cache.stamp = ros::Time(0);
  if (runtime_params->command_frame_id == runtime_params->base_frame_id)
  {
    cache.valid = true;
    cache.stamp = ros::Time::now();
    command_transform_buffer_.writeFromNonRT(cache);
    return;
  }

  if (!tf_buffer_)
  {
    ROS_WARN_THROTTLE(
        1.0,
        "Global command mode requires TF listener, but tf_buffer is not initialized.");
    InvalidateCommandTransformCache();
    return;
  }

  std::string transform_error;
  const bool TRANSFORM_READY = tf_buffer_->canTransform(
      runtime_params->base_frame_id, runtime_params->command_frame_id,
      ros::Time(0), ros::Duration(0.0), &transform_error);
  if (!TRANSFORM_READY)
  {
    ROS_WARN_THROTTLE(1.0,
                      "Failed to refresh cmd_vel transform from '%s' to '%s': %s",
                      runtime_params->command_frame_id.c_str(),
                      runtime_params->base_frame_id.c_str(),
                      transform_error.c_str());
    InvalidateCommandTransformCache();
    return;
  }

  geometry_msgs::TransformStamped command_to_base_transform;
  try
  {
    command_to_base_transform = tf_buffer_->lookupTransform(
        runtime_params->base_frame_id, runtime_params->command_frame_id,
        ros::Time(0), ros::Duration(0.0));
  }
  catch (const tf2::TransformException& exception)
  {
    ROS_WARN_THROTTLE(1.0,
                      "Failed to refresh cmd_vel transform from '%s' to '%s': %s",
                      runtime_params->command_frame_id.c_str(),
                      runtime_params->base_frame_id.c_str(),
                      exception.what());
    InvalidateCommandTransformCache();
    return;
  }

  Eigen::Matrix3d source_to_target_rotation;
  if (!controller_math::BuildRotationFromQuaternion(
          command_to_base_transform.transform.rotation, MIN_QUATERNION_NORM,
          &source_to_target_rotation))
  {
    ROS_WARN_THROTTLE(
        1.0,
        "Failed to refresh cmd_vel transform from '%s' to '%s': invalid quaternion.",
        runtime_params->command_frame_id.c_str(),
        runtime_params->base_frame_id.c_str());
    InvalidateCommandTransformCache();
    return;
  }

  for (int row = 0; row < 3; ++row)
  {
    for (int col = 0; col < 3; ++col)
    {
      cache.rotation_matrix_row_major[static_cast<std::size_t>(row * 3 + col)] =
          source_to_target_rotation(row, col);
    }
  }
  cache.valid = true;
  cache.stamp = command_to_base_transform.header.stamp;
  command_transform_buffer_.writeFromNonRT(cache);
}

void SentryChassisController::ApplyRuntimeParamsInUpdate(
    const RuntimeParams& runtime_params)
{
  const bool GEOMETRY_CHANGED =
      runtime_params.geometry.wheel_base != applied_geometry_.wheel_base ||
      runtime_params.geometry.track_width != applied_geometry_.track_width ||
      runtime_params.geometry.wheel_radius != applied_geometry_.wheel_radius;
  if (GEOMETRY_CHANGED)
  {
    applied_geometry_ = runtime_params.geometry;
    kinematics_.SetGeometry(applied_geometry_);
  }

  const bool FRAME_IDS_CHANGED =
      !applied_odom_frame_id_.empty() && !applied_base_frame_id_.empty() &&
      (runtime_params.odom_frame_id != applied_odom_frame_id_ ||
       runtime_params.base_frame_id != applied_base_frame_id_);
  if (FRAME_IDS_CHANGED)
  {
    ROS_WARN(
        "Odometry frame parameters changed (odom: '%s' -> '%s', base: '%s' -> '%s'). "
        "Resetting accumulated odometry state.",
        applied_odom_frame_id_.c_str(), runtime_params.odom_frame_id.c_str(),
        applied_base_frame_id_.c_str(), runtime_params.base_frame_id.c_str());
    odom_state_ = OdomState();
  }

  if (applied_odom_frame_id_ != runtime_params.odom_frame_id)
  {
    applied_odom_frame_id_ = runtime_params.odom_frame_id;
  }
  if (applied_base_frame_id_ != runtime_params.base_frame_id)
  {
    applied_base_frame_id_ = runtime_params.base_frame_id;
  }
}

void SentryChassisController::ApplyAccelerationLimits(
    const Kinematics::ChassisTwist& input, double dt,
    const RuntimeParams& runtime_params, Kinematics::ChassisTwist* output)
{
  if (output == nullptr)
  {
    return;
  }
  *output = input;
  if (!runtime_params.enable_acceleration_limits || dt <= MIN_VALID_DT)
  {
    return;
  }
  if (!has_last_limited_command_)
  {
    last_limited_command_ = input;
    has_last_limited_command_ = true;
    return;
  }

  const double max_linear_delta = runtime_params.max_linear_acceleration * dt;
  const double max_angular_delta = runtime_params.max_angular_acceleration * dt;
  const Eigen::Vector2d previous_linear(last_limited_command_.vx,
                                        last_limited_command_.vy);
  const Eigen::Vector2d target_linear(input.vx, input.vy);
  Eigen::Vector2d delta_linear = target_linear - previous_linear;
  const double delta_linear_norm = delta_linear.norm();
  if (delta_linear_norm > max_linear_delta && delta_linear_norm > MIN_VALID_DT)
  {
    delta_linear *= max_linear_delta / delta_linear_norm;
  }

  output->vx = previous_linear.x() + delta_linear.x();
  output->vy = previous_linear.y() + delta_linear.y();
  const double delta_wz = input.wz - last_limited_command_.wz;
  output->wz = last_limited_command_.wz +
               std::max(-max_angular_delta, std::min(max_angular_delta, delta_wz));
  last_limited_command_ = *output;
}

void SentryChassisController::ApplyPowerLimiting(
    const RuntimeParams& runtime_params,
    const std::array<double, WHEEL_COUNT>& signed_wheel_velocities,
    std::array<double, WHEEL_COUNT>* signed_wheel_efforts) const
{
  if (signed_wheel_efforts == nullptr || !runtime_params.enable_power_limit)
  {
    return;
  }

  double output_power = 0.0;
  double effort_square_sum = 0.0;
  double velocity_square_sum = 0.0;
  for (std::size_t index = 0; index < WHEEL_COUNT; ++index)
  {
    const double effort = signed_wheel_efforts->at(index);
    const double velocity = signed_wheel_velocities[index];
    output_power += std::fabs(effort * velocity);
    effort_square_sum += effort * effort;
    velocity_square_sum += velocity * velocity;
  }

  const double QUADRATIC_EFFORT_TERM =
      runtime_params.power_loss_k1 * effort_square_sum;
  const double VELOCITY_LOSS_TERM =
      runtime_params.power_loss_k2 * velocity_square_sum;
  const double PREDICTED_INPUT_POWER =
      output_power + QUADRATIC_EFFORT_TERM + VELOCITY_LOSS_TERM;
  if (PREDICTED_INPUT_POWER <= runtime_params.max_power)
  {
    return;
  }

  const double RHS = runtime_params.max_power - VELOCITY_LOSS_TERM;
  double scale = runtime_params.min_power_scale;
  if (RHS > 0.0)
  {
    if (QUADRATIC_EFFORT_TERM > MIN_VALID_DT)
    {
      // Solve QUADRATIC_EFFORT_TERM * s^2 + output_power * s = RHS for s in (0, 1].
      const double DISCRIMINANT =
          output_power * output_power + 4.0 * QUADRATIC_EFFORT_TERM * RHS;
      if (DISCRIMINANT > 0.0)
      {
        scale = (-output_power + std::sqrt(DISCRIMINANT)) /
                (2.0 * QUADRATIC_EFFORT_TERM);
      }
    }
    else if (output_power > MIN_VALID_DT)
    {
      scale = RHS / output_power;
    }
  }
  scale = std::max(runtime_params.min_power_scale, std::min(1.0, scale));
  for (auto& effort : *signed_wheel_efforts)
  {
    effort *= scale;
  }

  if (runtime_params.enable_power_limit_logging)
  {
    ROS_WARN_THROTTLE(1.0,
                      "Power limit active: predicted=%.3fW, max=%.3fW, scale=%.3f.",
                      PREDICTED_INPUT_POWER, runtime_params.max_power, scale);
  }
}

bool SentryChassisController::ResolveCommandInBaseFrame(const CommandData& command,
                                                        const ros::Time& time,
                                                        const RuntimeParams& runtime_params,
                                                        Kinematics::ChassisTwist* base_twist)
{
  (void)time;
  if (base_twist == nullptr)
  {
    return false;
  }

  Kinematics::ChassisTwist source_twist;
  source_twist.vx = command.vx;
  source_twist.vy = command.vy;
  source_twist.wz = command.wz;

  if (runtime_params.command_velocity_mode == CommandVelocityMode::BASE_LINK)
  {
    *base_twist = source_twist;
    return true;
  }

  const CommandTransformCache* command_transform = command_transform_buffer_.readFromRT();
  if (command_transform == nullptr || !command_transform->valid)
  {
    ROS_WARN_THROTTLE(
        1.0,
        "Global command mode transform cache is not ready. "
        "Failed to transform cmd_vel from '%s' to '%s'.",
        runtime_params.command_frame_id.c_str(), runtime_params.base_frame_id.c_str());
    return false;
  }

  const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>
      SOURCE_TO_TARGET_ROTATION(command_transform->rotation_matrix_row_major.data());
  const Eigen::Vector3d LINEAR_INPUT(source_twist.vx, source_twist.vy, 0.0);
  const Eigen::Vector3d ANGULAR_INPUT(0.0, 0.0, source_twist.wz);
  const Eigen::Vector3d LINEAR_OUTPUT = SOURCE_TO_TARGET_ROTATION * LINEAR_INPUT;
  const Eigen::Vector3d ANGULAR_OUTPUT = SOURCE_TO_TARGET_ROTATION * ANGULAR_INPUT;
  base_twist->vx = LINEAR_OUTPUT.x();
  base_twist->vy = LINEAR_OUTPUT.y();
  base_twist->wz = ANGULAR_OUTPUT.z();
  return true;
}

bool SentryChassisController::PrepareCommandForControl(
    const CommandData& command, const ros::Time& time, double dt,
    const RuntimeParams& runtime_params,
    Kinematics::ChassisTwist* limited_command_twist_base, bool* timeout)
{
  if (limited_command_twist_base == nullptr || timeout == nullptr)
  {
    return false;
  }

  const bool COMMAND_FROM_CURRENT_SESSION = command.stamp >= controller_start_time_;
  const bool COMMAND_VALID_FOR_UPDATE = command.valid && COMMAND_FROM_CURRENT_SESSION;
  const bool TIMEOUT = IsCommandTimedOut(COMMAND_VALID_FOR_UPDATE, command.stamp, time,
                                         runtime_params.cmd_vel_timeout);
  if (TIMEOUT && !last_command_timed_out_)
  {
    for (auto& pid : wheel_pids_)
    {
      pid.reset();
    }
  }
  last_command_timed_out_ = TIMEOUT;

  Kinematics::ChassisTwist command_twist_base{};
  if (!TIMEOUT)
  {
    const bool COMMAND_RESOLVED =
        ResolveCommandInBaseFrame(command, time, runtime_params, &command_twist_base);
    if (!COMMAND_RESOLVED)
    {
      command_twist_base = Kinematics::ChassisTwist();
    }
  }
  else
  {
    has_last_limited_command_ = false;
  }

  *limited_command_twist_base = command_twist_base;
  ApplyAccelerationLimits(command_twist_base, dt, runtime_params,
                          limited_command_twist_base);
  *timeout = TIMEOUT;
  return true;
}

void SentryChassisController::ComputeAndApplyWheelControl(
    const Kinematics::ChassisTwist& limited_command_twist_base,
    const ros::Duration& period, const RuntimeParams& runtime_params)
{
  const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>
      COMMAND_COMPENSATION_BASE(command_compensation_matrix_.data());
  const Eigen::Vector3d COMMAND_INPUT_BASE(limited_command_twist_base.vx,
                                           limited_command_twist_base.vy,
                                           limited_command_twist_base.wz);
  Eigen::Matrix3d command_compensation_effective = COMMAND_COMPENSATION_BASE;
  Eigen::Vector3d command_effective =
      command_compensation_effective * COMMAND_INPUT_BASE;
  const controller_math::ReverseCompensationParams reverse_compensation_params{
      runtime_params.reverse_ccw_vy_threshold,
      runtime_params.reverse_ccw_vx_scale,
      runtime_params.reverse_ccw_wz_gain,
      ZERO_CMD_EPS,
      REVERSE_STRAIGHT_VX_BOOST};
  controller_math::ApplyNonLinearReverseCompensation(
      reverse_compensation_params, &command_compensation_effective, &command_effective);
  const double VX = command_effective.x();
  const double VY = command_effective.y();
  const double WZ = command_effective.z();

  const bool ZERO_CMD_REQUESTED =
      std::fabs(VX) < ZERO_CMD_EPS && std::fabs(VY) < ZERO_CMD_EPS &&
      std::fabs(WZ) < ZERO_CMD_EPS;
  const bool HAS_YAW_COMMAND = std::fabs(WZ) >= ZERO_CMD_EPS;
  const bool HAS_TRANSLATION_COMMAND =
      std::fabs(VX) >= ZERO_CMD_EPS || std::fabs(VY) >= ZERO_CMD_EPS;
  const bool REVERSE_CCW_MODE =
      VX < -ZERO_CMD_EPS && std::fabs(VY) <= runtime_params.reverse_ccw_vy_threshold &&
      WZ > ZERO_CMD_EPS;
  const double HALF_WHEEL_BASE = applied_geometry_.wheel_base * 0.5;
  const double HALF_TRACK_WIDTH = applied_geometry_.track_width * 0.5;
  const double WHEEL_RADIUS =
      applied_geometry_.wheel_radius > MIN_WHEEL_RADIUS ? applied_geometry_.wheel_radius
                                                        : MIN_WHEEL_RADIUS;
  const std::array<std::pair<double, double>, WHEEL_COUNT> MODULE_POSITIONS = {{
      {HALF_WHEEL_BASE, HALF_TRACK_WIDTH},
      {HALF_WHEEL_BASE, -HALF_TRACK_WIDTH},
      {-HALF_WHEEL_BASE, HALF_TRACK_WIDTH},
      {-HALF_WHEEL_BASE, -HALF_TRACK_WIDTH},
  }};
  std::array<double, WHEEL_COUNT> steer_errors{};
  std::array<double, WHEEL_COUNT> wheel_target_values{};
  std::array<double, WHEEL_COUNT> alignments{};
  std::array<bool, WHEEL_COUNT> wheel_pid_reset_flags{};

  // Unified control flow: compute per-wheel targets first, then execute control.
  // 零指令与非零指令仅在目标值与轮速环策略上不同，执行层保持一致。
  for (std::size_t i = 0; i < WHEEL_COUNT; ++i)
  {
    double steer_error =
        NormalizeAngle(steer_zero_offsets_[i] - steer_joints_[i].getPosition());
    double wheel_target = 0.0;
    double alignment = 1.0;
    bool reset_wheel_pid = ZERO_CMD_REQUESTED;

    if (!ZERO_CMD_REQUESTED)
    {
      // Swerve IK: project chassis twist to each module velocity, then convert
      // to target steering angle and wheel speed.
      const double MODULE_X = MODULE_POSITIONS[i].first;
      const double MODULE_Y = MODULE_POSITIONS[i].second;
      const double VX_SIGN = static_cast<double>(direction_signs_.vx[i]);
      const double VY_SIGN = static_cast<double>(direction_signs_.vy[i]);
      const double WZ_SIGN = static_cast<double>(direction_signs_.wz[i]);
      const Eigen::Matrix<double, 2, 3> MODULE_PROJECTION =
          (Eigen::Matrix<double, 2, 3>() << VX_SIGN, 0.0, -WZ_SIGN * MODULE_Y, 0.0,
           VY_SIGN, WZ_SIGN * MODULE_X)
              .finished();
      const auto SOLVE_MODULE_TARGET =
          [&](const Eigen::Matrix3d& merged_compensation, double* solved_steer_error,
              double* solved_wheel_target) -> bool
      {
        if (solved_steer_error == nullptr || solved_wheel_target == nullptr)
        {
          return false;
        }
        const Eigen::Matrix<double, 2, 3> MODULE_JACOBIAN =
            MODULE_PROJECTION * merged_compensation;
        const Eigen::Vector2d MODULE_VELOCITY = MODULE_JACOBIAN * COMMAND_INPUT_BASE;
        const double MODULE_VX = MODULE_VELOCITY.x();
        const double MODULE_VY = MODULE_VELOCITY.y();
        const double MODULE_SPEED = std::hypot(MODULE_VX, MODULE_VY);
        if (MODULE_SPEED <= ZERO_CMD_EPS)
        {
          return false;
        }

        const double TARGET_STEER =
            std::atan2(MODULE_VY, MODULE_VX) + steer_zero_offsets_[i];
        double solved_error =
            NormalizeAngle(TARGET_STEER - steer_joints_[i].getPosition());
        double solved_target = MODULE_SPEED / WHEEL_RADIUS;
        // Flip wheel direction if turning more than 90deg to reach target.
        if (solved_error > HALF_PI + STEER_FLIP_EPS)
        {
          solved_error -= PI;
          solved_target = -solved_target;
        }
        else if (solved_error < -HALF_PI - STEER_FLIP_EPS)
        {
          solved_error += PI;
          solved_target = -solved_target;
        }
        *solved_steer_error = solved_error;
        *solved_wheel_target = solved_target;
        return true;
      };

      bool has_module_target = SOLVE_MODULE_TARGET(command_compensation_effective,
                                                   &steer_error, &wheel_target);
      if (has_module_target && REVERSE_CCW_MODE &&
          std::fabs(steer_error) > runtime_params.reverse_ccw_steer_priority_error)
      {
        // Steering-priority mode: when reverse-CCW command arrives while steering
        // still has large error, temporarily solve wheel target without the reverse
        // translation component to avoid opposite-yaw transients.
        Eigen::Matrix3d steer_priority_compensation = command_compensation_effective;
        steer_priority_compensation.row(0).setZero();
        has_module_target = SOLVE_MODULE_TARGET(steer_priority_compensation, &steer_error,
                                                &wheel_target);
      }

      if (has_module_target)
      {
        // Gate wheel speed only when yaw command is active so steering direction
        // switches (e.g. reverse turn) do not inject opposite yaw transients.
        alignment = 1.0;
        if (HAS_YAW_COMMAND)
        {
          const double ALIGNMENT_FLOOR =
              HAS_TRANSLATION_COMMAND ? 0.0 : GLOBAL_ALIGNMENT_GATE;
          alignment = std::max(0.0, std::cos(steer_error));
          alignment = std::max(alignment, ALIGNMENT_FLOOR);
        }
      }
      reset_wheel_pid = false;
    }

    steer_errors[i] = steer_error;
    wheel_target_values[i] = wheel_target;
    alignments[i] = alignment;
    wheel_pid_reset_flags[i] = reset_wheel_pid;
  }

  // Steering actuation (always active).
  // 舵向控制统一执行，区别仅来自上游目标计算结果。
  for (std::size_t i = 0; i < WHEEL_COUNT; ++i)
  {
    const double STEER_EFFORT = steer_pids_[i].computeCommand(steer_errors[i], period);
    steer_joints_[i].setCommand(STEER_EFFORT);
  }

  std::array<double, WHEEL_COUNT> signed_wheel_velocities{};
  std::array<double, WHEEL_COUNT> signed_wheel_efforts{};
  for (std::size_t i = 0; i < WHEEL_COUNT; ++i)
  {
    if (wheel_pid_reset_flags[i])
    {
      wheel_pids_[i].reset();
      signed_wheel_velocities[i] = 0.0;
      signed_wheel_efforts[i] = 0.0;
      continue;
    }

    // 轮速按速度误差闭环，目标值由逆运动学解算得到。
    // Wheel loop closes on velocity error using IK-computed targets.
    const double ROLLING_SIGN = static_cast<double>(wheel_rolling_signs_[i]);
    const double SIGNED_WHEEL_VELOCITY = ROLLING_SIGN * wheel_joints_[i].getVelocity();
    const double WHEEL_TARGET = wheel_target_values[i] * alignments[i];
    const double WHEEL_ERROR = WHEEL_TARGET - SIGNED_WHEEL_VELOCITY;
    const double SIGNED_WHEEL_EFFORT = wheel_pids_[i].computeCommand(WHEEL_ERROR, period);
    const double LIMITED_SIGNED_WHEEL_EFFORT =
        std::max(-runtime_params.wheel_effort_limit,
                 std::min(runtime_params.wheel_effort_limit, SIGNED_WHEEL_EFFORT));
    signed_wheel_velocities[i] = SIGNED_WHEEL_VELOCITY;
    signed_wheel_efforts[i] = LIMITED_SIGNED_WHEEL_EFFORT;
  }
  ApplyPowerLimiting(runtime_params, signed_wheel_velocities, &signed_wheel_efforts);
  for (std::size_t i = 0; i < WHEEL_COUNT; ++i)
  {
    if (wheel_pid_reset_flags[i])
    {
      wheel_joints_[i].setCommand(0.0);
      continue;
    }
    const double ROLLING_SIGN = static_cast<double>(wheel_rolling_signs_[i]);
    wheel_joints_[i].setCommand(ROLLING_SIGN * signed_wheel_efforts[i]);
  }
}

Kinematics::ChassisTwist SentryChassisController::ComputeAndIntegrateOdometry(
    const ros::Time& time, double dt, bool timeout,
    const RuntimeParams& runtime_params)
{
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
    if (timeout && !runtime_params.odom_integrate_on_timeout)
    {
      odom_twist = Kinematics::ChassisTwist();
    }
    else
    {
      const double STARTUP_AGE = (time - controller_start_time_).toSec();
      const bool IN_STARTUP_HOLD =
          STARTUP_AGE >= 0.0 && STARTUP_AGE < runtime_params.odom_startup_hold_sec;
      const bool SHOULD_SUPPRESS_STARTUP_DRIFT =
          runtime_params.odom_integrate_on_timeout && timeout && IN_STARTUP_HOLD;

      if (SHOULD_SUPPRESS_STARTUP_DRIFT)
      {
        odom_twist = Kinematics::ChassisTwist();
        ROS_WARN_THROTTLE(
            1.0,
            "Suppress odometry integration during startup hold window (%.3fs remaining).",
            runtime_params.odom_startup_hold_sec - STARTUP_AGE);
      }
      else if (!IsOdomTwistAcceptable(odom_twist, runtime_params))
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
        odom_state_ = IntegrateOdom(odom_state_, odom_twist, dt);
      }
    }
  }
  return odom_twist;
}

void SentryChassisController::ResetControllerOutputsAndPids()
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
}

void SentryChassisController::ResetControllerTrackingState(const ros::Time& start_time)
{
  odom_state_ = OdomState();
  last_limited_command_ = Kinematics::ChassisTwist();
  has_last_limited_command_ = false;
  controller_start_time_ = start_time;
  last_command_timed_out_ = true;
}

void SentryChassisController::starting(const ros::Time& time)
{
  ResetControllerOutputsAndPids();
  ResetControllerTrackingState(time);
}

void SentryChassisController::update(const ros::Time& time, const ros::Duration& period)
{
  const double DT = period.toSec();
  if (DT <= MIN_VALID_DT)
  {
    ROS_WARN_THROTTLE(1.0, "Skip update due to non-positive period.");
    return;
  }

  const RuntimeParams* runtime_params = runtime_params_buffer_.readFromRT();
  if (runtime_params == nullptr)
  {
    ROS_WARN_THROTTLE(1.0, "Runtime parameter snapshot is not initialized yet.");
    return;
  }
  ApplyRuntimeParamsInUpdate(*runtime_params);

  const CommandData* command = command_buffer_.readFromRT();
  if (command == nullptr)
  {
    ROS_WARN_THROTTLE(1.0, "Command buffer is not initialized yet.");
    return;
  }

  Kinematics::ChassisTwist limited_command_twist_base{};
  bool timeout = false;
  if (!PrepareCommandForControl(*command, time, DT, *runtime_params,
                                &limited_command_twist_base, &timeout))
  {
    ROS_WARN_THROTTLE(1.0, "Failed to prepare command for control.");
    return;
  }

  ComputeAndApplyWheelControl(limited_command_twist_base, period, *runtime_params);
  const Kinematics::ChassisTwist ODOM_TWIST =
      ComputeAndIntegrateOdometry(time, DT, timeout, *runtime_params);
  PublishOdometry(time, ODOM_TWIST, *runtime_params);
}

void SentryChassisController::stopping(const ros::Time& time)
{
  (void)time;
  ResetControllerOutputsAndPids();
  ResetControllerTrackingState(ros::Time(0));
}

bool SentryChassisController::IsOdomTwistAcceptable(
    const Kinematics::ChassisTwist& twist, const RuntimeParams& runtime_params) const
{
  if (!std::isfinite(twist.vx) || !std::isfinite(twist.vy) || !std::isfinite(twist.wz))
  {
    return false;
  }

  const double LINEAR_SPEED = std::hypot(twist.vx, twist.vy);
  const double ANGULAR_SPEED = std::fabs(twist.wz);
  return LINEAR_SPEED <= runtime_params.odom_max_linear_speed &&
         ANGULAR_SPEED <= runtime_params.odom_max_angular_speed;
}

void SentryChassisController::PublishOdometry(const ros::Time& time,
                                              const Kinematics::ChassisTwist& twist,
                                              const RuntimeParams& runtime_params)
{
  nav_msgs::Odometry odometry;
  odometry.header.stamp = time;
  odometry.header.frame_id = runtime_params.odom_frame_id;
  odometry.child_frame_id = runtime_params.base_frame_id;
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

  if (!runtime_params.publish_tf || !tf_broadcaster_)
  {
    return;
  }

  geometry_msgs::TransformStamped transform;
  transform.header.stamp = time;
  transform.header.frame_id = runtime_params.odom_frame_id;
  transform.child_frame_id = runtime_params.base_frame_id;
  transform.transform.translation.x = odom_state_.x;
  transform.transform.translation.y = odom_state_.y;
  transform.transform.translation.z = 0.0;
  transform.transform.rotation = odometry.pose.pose.orientation;
  tf_broadcaster_->sendTransform(transform);
}


}  // namespace sentry_chassis_controller
