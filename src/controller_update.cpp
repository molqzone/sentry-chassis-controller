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

constexpr int64_t DEFERRED_RT_WARN_FLUSH_INTERVAL_NS = 1000000000LL;

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
  RuntimeParams& params = runtime_params_shadow_;
  const std::string PREVIOUS_ODOM_FRAME_ID = params.odom_frame_id;
  const std::string PREVIOUS_BASE_FRAME_ID = params.base_frame_id;

  struct PositiveFallbackRule
  {
    const char* name;
    double* value;
    double fallback;
  };
  struct NonNegativeFallbackRule
  {
    const char* name;
    double* value;
    double fallback;
  };
  struct NonNegativeZeroRule
  {
    const char* name;
    double* value;
  };
  struct RangeClampRule
  {
    const char* name;
    double* value;
    double min;
    double max;
  };

  if (params.geometry.wheel_radius < MIN_WHEEL_RADIUS)
  {
    ROS_WARN("Parameter 'geometry/wheel_radius' must be >= %.9f. Clamping to %.9f.",
             MIN_WHEEL_RADIUS, MIN_WHEEL_RADIUS);
    params.geometry.wheel_radius = MIN_WHEEL_RADIUS;
  }

  const std::array<NonNegativeZeroRule, 2> NON_NEGATIVE_ZERO_RULES = {{
      {"cmd_vel_timeout", &params.cmd_vel_timeout},
      {"odom_startup_hold_sec", &params.odom_startup_hold_sec},
  }};
  for (const auto& rule : NON_NEGATIVE_ZERO_RULES)
  {
    if (*rule.value < 0.0)
    {
      ROS_WARN("Parameter '%s' is negative. Clamping to 0.0.", rule.name);
      *rule.value = 0.0;
    }
  }

  const std::array<PositiveFallbackRule, 6> POSITIVE_FALLBACK_RULES = {{
      {"odom_max_linear_speed", &params.odom_max_linear_speed,
       DEFAULT_ODOM_MAX_LINEAR_SPEED},
      {"odom_max_angular_speed", &params.odom_max_angular_speed,
       DEFAULT_ODOM_MAX_ANGULAR_SPEED},
      {"wheel_effort_limit", &params.wheel_effort_limit, DEFAULT_WHEEL_EFFORT_LIMIT},
      {"max_linear_acceleration", &params.max_linear_acceleration,
       DEFAULT_MAX_LINEAR_ACCELERATION},
      {"max_angular_acceleration", &params.max_angular_acceleration,
       DEFAULT_MAX_ANGULAR_ACCELERATION},
      {"max_power", &params.max_power, DEFAULT_MAX_POWER},
  }};
  for (const auto& rule : POSITIVE_FALLBACK_RULES)
  {
    if (*rule.value <= 0.0)
    {
      ROS_WARN("Parameter '%s' must be positive. Clamping to %.3f.",
               rule.name, rule.fallback);
      *rule.value = rule.fallback;
    }
  }

  const std::array<NonNegativeFallbackRule, 2> NON_NEGATIVE_FALLBACK_RULES = {{
      {"power_loss_k1", &params.power_loss_k1, DEFAULT_POWER_LOSS_K1},
      {"power_loss_k2", &params.power_loss_k2, DEFAULT_POWER_LOSS_K2},
  }};
  for (const auto& rule : NON_NEGATIVE_FALLBACK_RULES)
  {
    if (*rule.value < 0.0)
    {
      ROS_WARN("Parameter '%s' must be non-negative. Clamping to %.6f.",
               rule.name, rule.fallback);
      *rule.value = rule.fallback;
    }
  }

  const std::array<RangeClampRule, 5> RANGE_CLAMP_RULES = {{
      {"reverse_ccw_vx_scale", &params.reverse_ccw_vx_scale, MIN_REVERSE_CCW_VX_SCALE,
       MAX_REVERSE_CCW_VX_SCALE},
      {"reverse_ccw_wz_gain", &params.reverse_ccw_wz_gain, MIN_REVERSE_CCW_WZ_GAIN,
       MAX_REVERSE_CCW_WZ_GAIN},
      {"reverse_ccw_vy_threshold", &params.reverse_ccw_vy_threshold,
       MIN_REVERSE_CCW_VY_THRESHOLD, MAX_REVERSE_CCW_VY_THRESHOLD},
      {"reverse_ccw_steer_priority_error", &params.reverse_ccw_steer_priority_error,
       MIN_REVERSE_CCW_STEER_PRIORITY_ERROR, MAX_REVERSE_CCW_STEER_PRIORITY_ERROR},
      {"min_power_scale", &params.min_power_scale, MIN_POWER_SCALE, MAX_POWER_SCALE},
  }};
  for (const auto& rule : RANGE_CLAMP_RULES)
  {
    if (*rule.value < rule.min || *rule.value > rule.max)
    {
      ROS_WARN("Parameter '%s' must be in [%.3f, %.3f]. Clamping.",
               rule.name, rule.min, rule.max);
      *rule.value = std::max(rule.min, std::min(rule.max, *rule.value));
    }
  }

  CommandVelocityMode parsed_mode = params.command_velocity_mode;
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
        params.command_velocity_mode == CommandVelocityMode::BASE_LINK ? "base_link"
                                                                       : "global";
    parsed_mode = params.command_velocity_mode;
  }
  params.command_velocity_mode = parsed_mode;

  if (params.base_frame_id.empty())
  {
    ROS_WARN("Parameter 'base_frame_id' is empty. Falling back to 'base_link'.");
    params.base_frame_id = "base_link";
  }
  if (params.odom_frame_id.empty())
  {
    ROS_WARN("Parameter 'odom_frame_id' is empty. Falling back to 'odom'.");
    params.odom_frame_id = "odom";
  }
  if (params.command_frame_id.empty())
  {
    const std::string FALLBACK_COMMAND_FRAME =
        params.command_velocity_mode == CommandVelocityMode::BASE_LINK
            ? params.base_frame_id
            : params.odom_frame_id;
    ROS_WARN("Parameter 'command_frame_id' is empty. Falling back to '%s'.",
             FALLBACK_COMMAND_FRAME.c_str());
    params.command_frame_id = FALLBACK_COMMAND_FRAME;
  }

  if (params.command_velocity_mode == CommandVelocityMode::BASE_LINK &&
      params.command_frame_id != params.base_frame_id)
  {
    ROS_WARN(
        "Parameter 'command_frame_id' is '%s' while command_velocity_mode is "
        "'base_link'. Falling back to '%s'.",
        params.command_frame_id.c_str(), params.base_frame_id.c_str());
    params.command_frame_id = params.base_frame_id;
  }
  if (params.command_velocity_mode == CommandVelocityMode::GLOBAL &&
      params.command_frame_id == params.base_frame_id)
  {
    ROS_WARN_THROTTLE(
        1.0,
        "command_velocity_mode is 'global' but command_frame_id equals base frame '%s'. "
        "Global transform will have no effect.",
        params.base_frame_id.c_str());
  }
  if (!strict_validation &&
      (PREVIOUS_ODOM_FRAME_ID != params.odom_frame_id ||
       PREVIOUS_BASE_FRAME_ID != params.base_frame_id))
  {
    ROS_WARN(
        "Odometry frame parameters changed (odom: '%s' -> '%s', base: '%s' -> '%s'). "
        "Resetting accumulated odometry state.",
        PREVIOUS_ODOM_FRAME_ID.c_str(), params.odom_frame_id.c_str(),
        PREVIOUS_BASE_FRAME_ID.c_str(), params.base_frame_id.c_str());
  }

  runtime_params_buffer_.writeFromNonRT(params);
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
  FlushDeferredRealtimeWarnings();
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

void SentryChassisController::FlushDeferredRealtimeWarnings()
{
  const int64_t NOW_NS = static_cast<int64_t>(ros::WallTime::now().toNSec());
  int64_t next_flush_time_ns =
      rt_warn_next_flush_time_ns_.load(std::memory_order_relaxed);
  if (NOW_NS < next_flush_time_ns)
  {
    return;
  }
  const int64_t NEXT_FLUSH_TIME_NS =
      NOW_NS + DEFERRED_RT_WARN_FLUSH_INTERVAL_NS;
  if (!rt_warn_next_flush_time_ns_.compare_exchange_strong(
          next_flush_time_ns, NEXT_FLUSH_TIME_NS, std::memory_order_relaxed,
          std::memory_order_relaxed))
  {
    return;
  }

  const auto FLUSH_WARNING = [](std::atomic<uint32_t>* counter,
                                const char* message) {
    const uint32_t COUNT = counter->exchange(0U, std::memory_order_relaxed);
    if (COUNT > 0U)
    {
      ROS_WARN("%s (count=%u).", message, COUNT);
    }
  };
  FLUSH_WARNING(&rt_warn_invalid_period_count_,
                "Realtime update skipped due to non-positive period");
  FLUSH_WARNING(&rt_warn_runtime_params_unready_count_,
                "Realtime update skipped because runtime params are not ready");
  FLUSH_WARNING(&rt_warn_command_buffer_unready_count_,
                "Realtime update skipped because command buffer is not ready");
  FLUSH_WARNING(&rt_warn_prepare_command_failed_count_,
                "Realtime update skipped because command preparation failed");
  FLUSH_WARNING(&rt_warn_transform_cache_unready_count_,
                "Global command transform cache was not ready in realtime path");
  FLUSH_WARNING(&rt_warn_odom_singular_count_,
                "Forward kinematics was singular during realtime odometry");
  FLUSH_WARNING(&rt_warn_odom_startup_hold_count_,
                "Odom integration was suppressed by startup hold in realtime path");
  FLUSH_WARNING(&rt_warn_odom_rejected_count_,
                "Abnormal odom twist was rejected in realtime path");

  const uint32_t POWER_LIMIT_COUNT =
      rt_warn_power_limit_active_count_.exchange(0U, std::memory_order_relaxed);
  if (POWER_LIMIT_COUNT > 0U)
  {
    const double LAST_PREDICTED_POWER_W =
        static_cast<double>(rt_warn_power_limit_last_predicted_milliwatt_.load(
            std::memory_order_relaxed)) /
        1000.0;
    const double LAST_MAX_POWER_W =
        static_cast<double>(rt_warn_power_limit_last_max_milliwatt_.load(
            std::memory_order_relaxed)) /
        1000.0;
    const double LAST_SCALE =
        static_cast<double>(rt_warn_power_limit_last_scale_milli_.load(
            std::memory_order_relaxed)) /
        1000.0;
    ROS_WARN(
        "Power limit was active in realtime path (count=%u, "
        "last_predicted=%.3fW, last_max=%.3fW, last_scale=%.3f).",
        POWER_LIMIT_COUNT, LAST_PREDICTED_POWER_W, LAST_MAX_POWER_W, LAST_SCALE);
  }
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
    std::array<double, WHEEL_COUNT>* signed_wheel_efforts)
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
    rt_warn_power_limit_active_count_.fetch_add(1U, std::memory_order_relaxed);
    rt_warn_power_limit_last_predicted_milliwatt_.store(
        static_cast<int32_t>(std::round(PREDICTED_INPUT_POWER * 1000.0)),
        std::memory_order_relaxed);
    rt_warn_power_limit_last_max_milliwatt_.store(
        static_cast<int32_t>(std::round(runtime_params.max_power * 1000.0)),
        std::memory_order_relaxed);
    rt_warn_power_limit_last_scale_milli_.store(
        static_cast<int32_t>(std::round(scale * 1000.0)),
        std::memory_order_relaxed);
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
    rt_warn_transform_cache_unready_count_.fetch_add(1U, std::memory_order_relaxed);
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
    rt_warn_odom_singular_count_.fetch_add(1U, std::memory_order_relaxed);
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
        rt_warn_odom_startup_hold_count_.fetch_add(1U, std::memory_order_relaxed);
      }
      else if (!IsOdomTwistAcceptable(odom_twist, runtime_params))
      {
        rt_warn_odom_rejected_count_.fetch_add(1U, std::memory_order_relaxed);
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
    rt_warn_invalid_period_count_.fetch_add(1U, std::memory_order_relaxed);
    return;
  }

  const RuntimeParams* runtime_params = runtime_params_buffer_.readFromRT();
  if (runtime_params == nullptr)
  {
    rt_warn_runtime_params_unready_count_.fetch_add(1U, std::memory_order_relaxed);
    return;
  }
  ApplyRuntimeParamsInUpdate(*runtime_params);

  const CommandData* command = command_buffer_.readFromRT();
  if (command == nullptr)
  {
    rt_warn_command_buffer_unready_count_.fetch_add(1U, std::memory_order_relaxed);
    return;
  }

  Kinematics::ChassisTwist limited_command_twist_base{};
  bool timeout = false;
  if (!PrepareCommandForControl(*command, time, DT, *runtime_params,
                                &limited_command_twist_base, &timeout))
  {
    rt_warn_prepare_command_failed_count_.fetch_add(1U, std::memory_order_relaxed);
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
