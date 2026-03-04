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
namespace
{
struct WheelCommandContext
{
  Eigen::Vector3d command_input_base = Eigen::Vector3d::Zero();
  Eigen::Matrix3d primary_compensation = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d steer_priority_compensation = Eigen::Matrix3d::Identity();
  bool use_steer_priority = false;
  bool zero_command_requested = true;
  bool has_yaw_command = false;
  bool has_translation_command = false;
};

struct WheelTargetPlan
{
  std::array<double, SentryChassisController::WHEEL_COUNT> steer_errors{};
  std::array<double, SentryChassisController::WHEEL_COUNT> wheel_targets{};
  std::array<double, SentryChassisController::WHEEL_COUNT> alignments{};
  std::array<bool, SentryChassisController::WHEEL_COUNT> wheel_pid_reset_flags{};
};

bool ShouldEnableSteerPriorityMode(
    const Eigen::Vector3d& command_effective, double reverse_ccw_vy_threshold)
{
  return command_effective.x() < -ZERO_CMD_EPS &&
         std::fabs(command_effective.y()) <= reverse_ccw_vy_threshold &&
         command_effective.z() > ZERO_CMD_EPS;
}

double ComputeWheelAlignment(bool has_yaw_command, bool has_translation_command,
                             double steer_error)
{
  if (!has_yaw_command)
  {
    return 1.0;
  }
  const double ALIGNMENT_FLOOR = has_translation_command ? 0.0 : GLOBAL_ALIGNMENT_GATE;
  const double ALIGNMENT = std::max(0.0, std::cos(steer_error));
  return std::max(ALIGNMENT, ALIGNMENT_FLOOR);
}

WheelCommandContext ApplyWheelCommandConstraintsAndCompensation(
    const Kinematics::ChassisTwist& limited_command_twist_base,
    const std::array<double, 9>& command_compensation_matrix,
    double reverse_ccw_vy_threshold, double reverse_ccw_vx_scale,
    double reverse_ccw_wz_gain)
{
  WheelCommandContext context;
  context.command_input_base = Eigen::Vector3d(
      limited_command_twist_base.vx, limited_command_twist_base.vy,
      limited_command_twist_base.wz);
  const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>
      command_compensation_base(command_compensation_matrix.data());
  context.primary_compensation = command_compensation_base;
  Eigen::Vector3d command_effective =
      context.primary_compensation * context.command_input_base;
  const controller_math::ReverseCompensationParams reverse_compensation_params{
      reverse_ccw_vy_threshold,
      reverse_ccw_vx_scale,
      reverse_ccw_wz_gain,
      ZERO_CMD_EPS,
      REVERSE_STRAIGHT_VX_BOOST};
  controller_math::ApplyNonLinearReverseCompensation(reverse_compensation_params,
                                                     &context.primary_compensation,
                                                     &command_effective);
  context.zero_command_requested =
      std::fabs(command_effective.x()) < ZERO_CMD_EPS &&
      std::fabs(command_effective.y()) < ZERO_CMD_EPS &&
      std::fabs(command_effective.z()) < ZERO_CMD_EPS;
  context.has_yaw_command = std::fabs(command_effective.z()) >= ZERO_CMD_EPS;
  context.has_translation_command =
      std::fabs(command_effective.x()) >= ZERO_CMD_EPS ||
      std::fabs(command_effective.y()) >= ZERO_CMD_EPS;
  context.use_steer_priority =
      ShouldEnableSteerPriorityMode(command_effective, reverse_ccw_vy_threshold);
  context.steer_priority_compensation = context.primary_compensation;
  context.steer_priority_compensation.row(0).setZero();
  return context;
}

bool SolveModuleTarget(const WheelCommandContext& command_context,
                       const Kinematics::DirectionSigns& direction_signs,
                       std::size_t module_index, double module_x, double module_y,
                       double steer_zero_offset, double steer_position,
                       double wheel_radius, const Eigen::Matrix3d& compensation,
                       double* steer_error, double* wheel_target)
{
  if (steer_error == nullptr || wheel_target == nullptr)
  {
    return false;
  }
  const double VX_SIGN = static_cast<double>(direction_signs.vx[module_index]);
  const double VY_SIGN = static_cast<double>(direction_signs.vy[module_index]);
  const double WZ_SIGN = static_cast<double>(direction_signs.wz[module_index]);
  const Eigen::Matrix<double, 2, 3> MODULE_PROJECTION =
      (Eigen::Matrix<double, 2, 3>() << VX_SIGN, 0.0, -WZ_SIGN * module_y, 0.0, VY_SIGN,
       WZ_SIGN * module_x)
          .finished();
  const Eigen::Matrix<double, 2, 3> MODULE_JACOBIAN = MODULE_PROJECTION * compensation;
  const Eigen::Vector2d MODULE_VELOCITY =
      MODULE_JACOBIAN * command_context.command_input_base;
  const double MODULE_VX = MODULE_VELOCITY.x();
  const double MODULE_VY = MODULE_VELOCITY.y();
  const double MODULE_SPEED = std::hypot(MODULE_VX, MODULE_VY);
  if (MODULE_SPEED <= ZERO_CMD_EPS)
  {
    return false;
  }

  const double TARGET_STEER = std::atan2(MODULE_VY, MODULE_VX) + steer_zero_offset;
  double solved_error =
      SentryChassisController::NormalizeAngle(TARGET_STEER - steer_position);
  double solved_target = MODULE_SPEED / wheel_radius;
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
  *steer_error = solved_error;
  *wheel_target = solved_target;
  return true;
}

WheelTargetPlan SolveWheelTargets(
    const WheelCommandContext& command_context,
    const Kinematics::DirectionSigns& direction_signs,
    const std::array<double, SentryChassisController::WHEEL_COUNT>& steer_zero_offsets,
    const std::array<double, SentryChassisController::WHEEL_COUNT>& steer_positions,
    const Kinematics::Geometry& geometry, double steer_priority_error_threshold)
{
  const double HALF_WHEEL_BASE = geometry.wheel_base * 0.5;
  const double HALF_TRACK_WIDTH = geometry.track_width * 0.5;
  const double WHEEL_RADIUS =
      geometry.wheel_radius > MIN_WHEEL_RADIUS ? geometry.wheel_radius
                                               : MIN_WHEEL_RADIUS;
  const std::array<std::pair<double, double>, SentryChassisController::WHEEL_COUNT>
      MODULE_POSITIONS = {{
          {HALF_WHEEL_BASE, HALF_TRACK_WIDTH},
          {HALF_WHEEL_BASE, -HALF_TRACK_WIDTH},
          {-HALF_WHEEL_BASE, HALF_TRACK_WIDTH},
          {-HALF_WHEEL_BASE, -HALF_TRACK_WIDTH},
      }};

  WheelTargetPlan target_plan;
  for (std::size_t i = 0; i < SentryChassisController::WHEEL_COUNT; ++i)
  {
    target_plan.steer_errors[i] =
        SentryChassisController::NormalizeAngle(steer_zero_offsets[i] - steer_positions[i]);
    target_plan.wheel_targets[i] = 0.0;
    target_plan.alignments[i] = 1.0;
    target_plan.wheel_pid_reset_flags[i] = command_context.zero_command_requested;

    if (command_context.zero_command_requested)
    {
      continue;
    }

    double steer_error = target_plan.steer_errors[i];
    double wheel_target = 0.0;
    bool has_module_target = SolveModuleTarget(
        command_context, direction_signs, i, MODULE_POSITIONS[i].first,
        MODULE_POSITIONS[i].second, steer_zero_offsets[i], steer_positions[i], WHEEL_RADIUS,
        command_context.primary_compensation, &steer_error, &wheel_target);
    if (has_module_target && command_context.use_steer_priority &&
        std::fabs(steer_error) > steer_priority_error_threshold)
    {
      has_module_target = SolveModuleTarget(
          command_context, direction_signs, i, MODULE_POSITIONS[i].first,
          MODULE_POSITIONS[i].second, steer_zero_offsets[i], steer_positions[i],
          WHEEL_RADIUS, command_context.steer_priority_compensation, &steer_error,
          &wheel_target);
    }
    if (has_module_target)
    {
      target_plan.steer_errors[i] = steer_error;
      target_plan.wheel_targets[i] = wheel_target;
      target_plan.alignments[i] = ComputeWheelAlignment(
          command_context.has_yaw_command, command_context.has_translation_command,
          steer_error);
    }
    target_plan.wheel_pid_reset_flags[i] = false;
  }
  return target_plan;
}

std::array<double, SentryChassisController::WHEEL_COUNT> BuildWheelDispatchCommands(
    const std::array<int, SentryChassisController::WHEEL_COUNT>& wheel_rolling_signs,
    const std::array<double, SentryChassisController::WHEEL_COUNT>& signed_wheel_efforts,
    const std::array<bool, SentryChassisController::WHEEL_COUNT>& wheel_pid_reset_flags)
{
  std::array<double, SentryChassisController::WHEEL_COUNT> wheel_commands{};
  for (std::size_t i = 0; i < SentryChassisController::WHEEL_COUNT; ++i)
  {
    if (wheel_pid_reset_flags[i])
    {
      wheel_commands[i] = 0.0;
      continue;
    }
    const double ROLLING_SIGN = static_cast<double>(wheel_rolling_signs[i]);
    wheel_commands[i] = ROLLING_SIGN * signed_wheel_efforts[i];
  }
  return wheel_commands;
}
}  // namespace

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
  struct PositiveFallbackRule
  {
    const char* name = "";
    double* value = nullptr;
    double fallback = 0.0;
  };
  struct NonNegativeFallbackRule
  {
    const char* name = "";
    double* value = nullptr;
    double fallback = 0.0;
  };
  struct NonNegativeZeroRule
  {
    const char* name = "";
    double* value = nullptr;
  };
  struct RangeClampRule
  {
    const char* name = "";
    double* value = nullptr;
    double min = 0.0;
    double max = 0.0;
  };

  if (geometry_.wheel_radius < MIN_WHEEL_RADIUS)
  {
    ROS_WARN("Parameter 'geometry/wheel_radius' must be >= %.9f. Clamping to %.9f.",
             MIN_WHEEL_RADIUS, MIN_WHEEL_RADIUS);
    geometry_.wheel_radius = MIN_WHEEL_RADIUS;
  }

  const std::array<NonNegativeZeroRule, 2> NON_NEGATIVE_ZERO_RULES = {{
      {"cmd_vel_timeout", &cmd_vel_timeout_},
      {"odom_startup_hold_sec", &odom_startup_hold_sec_},
  }};
  for (const auto& rule : NON_NEGATIVE_ZERO_RULES)
  {
    if (rule.value != nullptr && *rule.value < 0.0)
    {
      ROS_WARN("Parameter '%s' is negative. Clamping to 0.0.", rule.name);
      *rule.value = 0.0;
    }
  }

  const std::array<PositiveFallbackRule, 6> POSITIVE_FALLBACK_RULES = {{
      {"odom_max_linear_speed", &odom_max_linear_speed_, DEFAULT_ODOM_MAX_LINEAR_SPEED},
      {"odom_max_angular_speed", &odom_max_angular_speed_, DEFAULT_ODOM_MAX_ANGULAR_SPEED},
      {"wheel_effort_limit", &wheel_effort_limit_, DEFAULT_WHEEL_EFFORT_LIMIT},
      {"max_linear_acceleration", &max_linear_acceleration_, DEFAULT_MAX_LINEAR_ACCELERATION},
      {"max_angular_acceleration", &max_angular_acceleration_,
       DEFAULT_MAX_ANGULAR_ACCELERATION},
      {"max_power", &max_power_, DEFAULT_MAX_POWER},
  }};
  for (const auto& rule : POSITIVE_FALLBACK_RULES)
  {
    if (rule.value != nullptr && *rule.value <= 0.0)
    {
      ROS_WARN("Parameter '%s' must be positive. Clamping to %.3f.",
               rule.name, rule.fallback);
      *rule.value = rule.fallback;
    }
  }

  const std::array<NonNegativeFallbackRule, 2> NON_NEGATIVE_FALLBACK_RULES = {{
      {"power_loss_k1", &power_loss_k1_, DEFAULT_POWER_LOSS_K1},
      {"power_loss_k2", &power_loss_k2_, DEFAULT_POWER_LOSS_K2},
  }};
  for (const auto& rule : NON_NEGATIVE_FALLBACK_RULES)
  {
    if (rule.value != nullptr && *rule.value < 0.0)
    {
      ROS_WARN("Parameter '%s' must be non-negative. Clamping to %.6f.",
               rule.name, rule.fallback);
      *rule.value = rule.fallback;
    }
  }

  const std::array<RangeClampRule, 5> RANGE_CLAMP_RULES = {{
      {"reverse_ccw_vx_scale", &reverse_ccw_vx_scale_, MIN_REVERSE_CCW_VX_SCALE,
       MAX_REVERSE_CCW_VX_SCALE},
      {"reverse_ccw_wz_gain", &reverse_ccw_wz_gain_, MIN_REVERSE_CCW_WZ_GAIN,
       MAX_REVERSE_CCW_WZ_GAIN},
      {"reverse_ccw_vy_threshold", &reverse_ccw_vy_threshold_, MIN_REVERSE_CCW_VY_THRESHOLD,
       MAX_REVERSE_CCW_VY_THRESHOLD},
      {"reverse_ccw_steer_priority_error", &reverse_ccw_steer_priority_error_,
       MIN_REVERSE_CCW_STEER_PRIORITY_ERROR, MAX_REVERSE_CCW_STEER_PRIORITY_ERROR},
      {"min_power_scale", &min_power_scale_, MIN_POWER_SCALE, MAX_POWER_SCALE},
  }};
  for (const auto& rule : RANGE_CLAMP_RULES)
  {
    if (rule.value != nullptr && (*rule.value < rule.min || *rule.value > rule.max))
    {
      ROS_WARN("Parameter '%s' must be in [%.3f, %.3f]. Clamping.",
               rule.name, rule.min, rule.max);
      *rule.value = std::max(rule.min, std::min(rule.max, *rule.value));
    }
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
  struct DoubleSnapshotBinding
  {
    double SentryChassisController::*source = nullptr;
    double RuntimeParams::*target = nullptr;
  };
  struct BoolSnapshotBinding
  {
    bool SentryChassisController::*source = nullptr;
    bool RuntimeParams::*target = nullptr;
  };
  struct StringSnapshotBinding
  {
    std::string SentryChassisController::*source = nullptr;
    std::string RuntimeParams::*target = nullptr;
  };
  struct ModeSnapshotBinding
  {
    CommandVelocityMode SentryChassisController::*source = nullptr;
    CommandVelocityMode RuntimeParams::*target = nullptr;
  };
  struct GeometrySnapshotBinding
  {
    Kinematics::Geometry SentryChassisController::*source = nullptr;
    Kinematics::Geometry RuntimeParams::*target = nullptr;
  };
  const auto APPLY_BINDINGS = [this](const auto& bindings) {
    for (const auto& binding : bindings)
    {
      runtime_params_shadow_.*(binding.target) = this->*(binding.source);
    }
  };
  const std::array<DoubleSnapshotBinding, 15> DOUBLE_BINDINGS = {{
      {&SentryChassisController::cmd_vel_timeout_, &RuntimeParams::cmd_vel_timeout},
      {&SentryChassisController::odom_startup_hold_sec_, &RuntimeParams::odom_startup_hold_sec},
      {&SentryChassisController::odom_max_linear_speed_, &RuntimeParams::odom_max_linear_speed},
      {&SentryChassisController::odom_max_angular_speed_, &RuntimeParams::odom_max_angular_speed},
      {&SentryChassisController::wheel_effort_limit_, &RuntimeParams::wheel_effort_limit},
      {&SentryChassisController::reverse_ccw_vx_scale_, &RuntimeParams::reverse_ccw_vx_scale},
      {&SentryChassisController::reverse_ccw_wz_gain_, &RuntimeParams::reverse_ccw_wz_gain},
      {&SentryChassisController::reverse_ccw_vy_threshold_,
       &RuntimeParams::reverse_ccw_vy_threshold},
      {&SentryChassisController::reverse_ccw_steer_priority_error_,
       &RuntimeParams::reverse_ccw_steer_priority_error},
      {&SentryChassisController::max_linear_acceleration_,
       &RuntimeParams::max_linear_acceleration},
      {&SentryChassisController::max_angular_acceleration_,
       &RuntimeParams::max_angular_acceleration},
      {&SentryChassisController::max_power_, &RuntimeParams::max_power},
      {&SentryChassisController::power_loss_k1_, &RuntimeParams::power_loss_k1},
      {&SentryChassisController::power_loss_k2_, &RuntimeParams::power_loss_k2},
      {&SentryChassisController::min_power_scale_, &RuntimeParams::min_power_scale},
  }};
  const std::array<BoolSnapshotBinding, 5> BOOL_BINDINGS = {{
      {&SentryChassisController::odom_integrate_on_timeout_,
       &RuntimeParams::odom_integrate_on_timeout},
      {&SentryChassisController::publish_tf_, &RuntimeParams::publish_tf},
      {&SentryChassisController::enable_acceleration_limits_,
       &RuntimeParams::enable_acceleration_limits},
      {&SentryChassisController::enable_power_limit_, &RuntimeParams::enable_power_limit},
      {&SentryChassisController::enable_power_limit_logging_,
       &RuntimeParams::enable_power_limit_logging},
  }};
  const std::array<StringSnapshotBinding, 3> STRING_BINDINGS = {{
      {&SentryChassisController::command_frame_id_, &RuntimeParams::command_frame_id},
      {&SentryChassisController::odom_frame_id_, &RuntimeParams::odom_frame_id},
      {&SentryChassisController::base_frame_id_, &RuntimeParams::base_frame_id},
  }};
  const std::array<ModeSnapshotBinding, 1> MODE_BINDINGS = {{
      {&SentryChassisController::command_velocity_mode_, &RuntimeParams::command_velocity_mode},
  }};
  const std::array<GeometrySnapshotBinding, 1> GEOMETRY_BINDINGS = {{
      {&SentryChassisController::geometry_, &RuntimeParams::geometry},
  }};
  APPLY_BINDINGS(DOUBLE_BINDINGS);
  APPLY_BINDINGS(BOOL_BINDINGS);
  APPLY_BINDINGS(STRING_BINDINGS);
  APPLY_BINDINGS(MODE_BINDINGS);
  APPLY_BINDINGS(GEOMETRY_BINDINGS);
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
  std::array<double, WHEEL_COUNT> steer_positions{};
  std::array<double, WHEEL_COUNT> wheel_velocities{};
  for (std::size_t i = 0; i < WHEEL_COUNT; ++i)
  {
    steer_positions[i] = steer_joints_[i].getPosition();
    wheel_velocities[i] = wheel_joints_[i].getVelocity();
  }

  // Stage 1: constraint/compensation on chassis command (pure).
  const WheelCommandContext COMMAND_CONTEXT = ApplyWheelCommandConstraintsAndCompensation(
      limited_command_twist_base, command_compensation_matrix_,
      runtime_params.reverse_ccw_vy_threshold, runtime_params.reverse_ccw_vx_scale,
      runtime_params.reverse_ccw_wz_gain);
  // Stage 2: per-wheel target solving (pure).
  const WheelTargetPlan TARGET_PLAN =
      SolveWheelTargets(COMMAND_CONTEXT, direction_signs_, steer_zero_offsets_,
                        steer_positions, applied_geometry_,
                        runtime_params.reverse_ccw_steer_priority_error);

  // Stage 3 preparation: run stateful PID loops against solved targets.
  for (std::size_t i = 0; i < WHEEL_COUNT; ++i)
  {
    const double STEER_EFFORT =
        steer_pids_[i].computeCommand(TARGET_PLAN.steer_errors[i], period);
    steer_joints_[i].setCommand(STEER_EFFORT);
  }

  std::array<double, WHEEL_COUNT> signed_wheel_velocities{};
  std::array<double, WHEEL_COUNT> signed_wheel_efforts{};
  for (std::size_t i = 0; i < WHEEL_COUNT; ++i)
  {
    if (TARGET_PLAN.wheel_pid_reset_flags[i])
    {
      wheel_pids_[i].reset();
      signed_wheel_velocities[i] = 0.0;
      signed_wheel_efforts[i] = 0.0;
      continue;
    }
    const double ROLLING_SIGN = static_cast<double>(wheel_rolling_signs_[i]);
    const double SIGNED_WHEEL_VELOCITY = ROLLING_SIGN * wheel_velocities[i];
    const double WHEEL_TARGET = TARGET_PLAN.wheel_targets[i] * TARGET_PLAN.alignments[i];
    const double WHEEL_ERROR = WHEEL_TARGET - SIGNED_WHEEL_VELOCITY;
    const double SIGNED_WHEEL_EFFORT = wheel_pids_[i].computeCommand(WHEEL_ERROR, period);
    signed_wheel_velocities[i] = SIGNED_WHEEL_VELOCITY;
    signed_wheel_efforts[i] =
        std::max(-runtime_params.wheel_effort_limit,
                 std::min(runtime_params.wheel_effort_limit, SIGNED_WHEEL_EFFORT));
  }
  ApplyPowerLimiting(runtime_params, signed_wheel_velocities, &signed_wheel_efforts);

  // Stage 3: build per-wheel dispatch commands (pure), then apply to hardware.
  const std::array<double, WHEEL_COUNT> WHEEL_COMMANDS = BuildWheelDispatchCommands(
      wheel_rolling_signs_, signed_wheel_efforts, TARGET_PLAN.wheel_pid_reset_flags);
  for (std::size_t i = 0; i < WHEEL_COUNT; ++i)
  {
    wheel_joints_[i].setCommand(WHEEL_COMMANDS[i]);
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
  if (odom_publisher_)
  {
    odom_publisher_.publish(odometry);
  }

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
