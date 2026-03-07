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
  Kinematics::ChassisTwist primary_command_twist_base{};
  Kinematics::ChassisTwist steer_priority_command_twist_base{};
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

struct PowerLimitComputation
{
  double output_power = 0.0;
  double quadratic_effort_term = 0.0;
  double velocity_loss_term = 0.0;
  double predicted_input_power = 0.0;
};

template <typename RuntimeParams>
PowerLimitComputation ComputePowerLimitComputation(
    const RuntimeParams& runtime_params,
    const std::array<double, SentryChassisController::WHEEL_COUNT>& signed_wheel_velocities,
    const std::array<double, SentryChassisController::WHEEL_COUNT>& signed_wheel_efforts)
{
  PowerLimitComputation computation;
  double effort_square_sum = 0.0;
  double velocity_square_sum = 0.0;
  for (std::size_t index = 0; index < SentryChassisController::WHEEL_COUNT; ++index)
  {
    const double effort = signed_wheel_efforts[index];
    const double velocity = signed_wheel_velocities[index];
    computation.output_power += std::fabs(effort * velocity);
    effort_square_sum += effort * effort;
    velocity_square_sum += velocity * velocity;
  }

  computation.quadratic_effort_term = runtime_params.power_loss_k1 * effort_square_sum;
  computation.velocity_loss_term = runtime_params.power_loss_k2 * velocity_square_sum;
  computation.predicted_input_power =
      computation.output_power + computation.quadratic_effort_term +
      computation.velocity_loss_term;
  return computation;
}

template <typename RuntimeParams>
double ComputePowerLimitScale(const RuntimeParams& runtime_params,
                              const PowerLimitComputation& computation)
{
  const double POWER_BUDGET_WITHOUT_VELOCITY_LOSS =
      runtime_params.max_power - computation.velocity_loss_term;
  double scale = runtime_params.min_power_scale;
  if (POWER_BUDGET_WITHOUT_VELOCITY_LOSS <= 0.0)
  {
    return scale;
  }
  if (computation.quadratic_effort_term > MIN_VALID_DT)
  {
    // Solve quadratic_effort_term * s^2 + output_power * s = budget for s in (0, 1].
    const double DISCRIMINANT =
        computation.output_power * computation.output_power +
        4.0 * computation.quadratic_effort_term * POWER_BUDGET_WITHOUT_VELOCITY_LOSS;
    if (DISCRIMINANT > 0.0)
    {
      scale = (-computation.output_power + std::sqrt(DISCRIMINANT)) /
              (2.0 * computation.quadratic_effort_term);
    }
  }
  else if (computation.output_power > MIN_VALID_DT)
  {
    scale = POWER_BUDGET_WITHOUT_VELOCITY_LOSS / computation.output_power;
  }
  return std::max(runtime_params.min_power_scale, std::min(1.0, scale));
}

void ScaleWheelEfforts(double scale,
                       std::array<double, SentryChassisController::WHEEL_COUNT>* efforts)
{
  if (efforts == nullptr)
  {
    return;
  }

  for (double& effort : *efforts)
  {
    effort *= scale;
  }
}

void StorePowerLimitWarningSnapshot(
    std::atomic<uint32_t>* active_count, std::atomic<int32_t>* predicted_milliwatt,
    std::atomic<int32_t>* max_milliwatt, std::atomic<int32_t>* scale_milli,
    const PowerLimitComputation& computation, double max_power, double scale)
{
  if (active_count == nullptr || predicted_milliwatt == nullptr ||
      max_milliwatt == nullptr || scale_milli == nullptr)
  {
    return;
  }

  active_count->fetch_add(1U, std::memory_order_relaxed);
  predicted_milliwatt->store(
      static_cast<int32_t>(std::round(computation.predicted_input_power * 1000.0)),
      std::memory_order_relaxed);
  max_milliwatt->store(static_cast<int32_t>(std::round(max_power * 1000.0)),
                       std::memory_order_relaxed);
  scale_milli->store(static_cast<int32_t>(std::round(scale * 1000.0)),
                     std::memory_order_relaxed);
}

struct WheelMotionState
{
  std::array<double, SentryChassisController::WHEEL_COUNT> steer_positions{};
  std::array<double, SentryChassisController::WHEEL_COUNT> wheel_velocities{};
};

struct WheelEffortState
{
  std::array<double, SentryChassisController::WHEEL_COUNT> signed_wheel_velocities{};
  std::array<double, SentryChassisController::WHEEL_COUNT> signed_wheel_efforts{};
};

std::array<double, SentryChassisController::WHEEL_COUNT> BuildWheelDispatchCommands(
    const std::array<int, SentryChassisController::WHEEL_COUNT>& wheel_rolling_signs,
    const std::array<double, SentryChassisController::WHEEL_COUNT>& signed_wheel_efforts,
    const std::array<bool, SentryChassisController::WHEEL_COUNT>& wheel_pid_reset_flags);

WheelMotionState ReadWheelMotionState(
    const std::array<hardware_interface::JointHandle,
                     SentryChassisController::WHEEL_COUNT>& steer_joints,
    const std::array<hardware_interface::JointHandle,
                     SentryChassisController::WHEEL_COUNT>& wheel_joints)
{
  WheelMotionState state;
  for (std::size_t i = 0; i < SentryChassisController::WHEEL_COUNT; ++i)
  {
    state.steer_positions[i] = steer_joints[i].getPosition();
    state.wheel_velocities[i] = wheel_joints[i].getVelocity();
  }
  return state;
}

void ApplySteerJointCommands(
    const WheelTargetPlan& target_plan, const ros::Duration& period,
    std::array<control_toolbox::Pid, SentryChassisController::WHEEL_COUNT>* steer_pids,
    std::array<hardware_interface::JointHandle,
               SentryChassisController::WHEEL_COUNT>* steer_joints)
{
  if (steer_pids == nullptr || steer_joints == nullptr)
  {
    return;
  }

  for (std::size_t i = 0; i < SentryChassisController::WHEEL_COUNT; ++i)
  {
    const double steer_effort =
        steer_pids->at(i).computeCommand(target_plan.steer_errors[i], period);
    steer_joints->at(i).setCommand(steer_effort);
  }
}

double ClampWheelEffort(double limit, double effort)
{
  return std::max(-limit, std::min(limit, effort));
}

WheelEffortState ComputeWheelEffortState(
    const WheelTargetPlan& target_plan, const WheelMotionState& motion_state,
    const ros::Duration& period, double wheel_effort_limit,
    const std::array<int, SentryChassisController::WHEEL_COUNT>& wheel_rolling_signs,
    std::array<control_toolbox::Pid, SentryChassisController::WHEEL_COUNT>* wheel_pids)
{
  WheelEffortState state;
  if (wheel_pids == nullptr)
  {
    return state;
  }

  for (std::size_t i = 0; i < SentryChassisController::WHEEL_COUNT; ++i)
  {
    if (target_plan.wheel_pid_reset_flags[i])
    {
      wheel_pids->at(i).reset();
      continue;
    }

    const double rolling_sign = static_cast<double>(wheel_rolling_signs[i]);
    const double signed_wheel_velocity = rolling_sign * motion_state.wheel_velocities[i];
    const double wheel_target = target_plan.wheel_targets[i] * target_plan.alignments[i];
    const double wheel_error = wheel_target - signed_wheel_velocity;
    const double signed_wheel_effort =
        wheel_pids->at(i).computeCommand(wheel_error, period);
    state.signed_wheel_velocities[i] = signed_wheel_velocity;
    state.signed_wheel_efforts[i] =
        ClampWheelEffort(wheel_effort_limit, signed_wheel_effort);
  }
  return state;
}

void ApplyWheelDispatchCommands(
    const std::array<int, SentryChassisController::WHEEL_COUNT>& wheel_rolling_signs,
    const WheelEffortState& effort_state,
    const std::array<bool, SentryChassisController::WHEEL_COUNT>& wheel_pid_reset_flags,
    std::array<hardware_interface::JointHandle,
               SentryChassisController::WHEEL_COUNT>* wheel_joints)
{
  if (wheel_joints == nullptr)
  {
    return;
  }

  const std::array<double, SentryChassisController::WHEEL_COUNT> wheel_commands =
      BuildWheelDispatchCommands(wheel_rolling_signs,
                                 effort_state.signed_wheel_efforts,
                                 wheel_pid_reset_flags);
  for (std::size_t i = 0; i < SentryChassisController::WHEEL_COUNT; ++i)
  {
    wheel_joints->at(i).setCommand(wheel_commands[i]);
  }
}

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
  const Eigen::Vector3d command_input_base(limited_command_twist_base.vx,
                                           limited_command_twist_base.vy,
                                           limited_command_twist_base.wz);
  const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>
      command_compensation_base(command_compensation_matrix.data());
  Eigen::Matrix3d primary_compensation = command_compensation_base;
  Eigen::Vector3d command_effective = primary_compensation * command_input_base;
  const controller_math::ReverseCompensationParams reverse_compensation_params{
      reverse_ccw_vy_threshold,
      reverse_ccw_vx_scale,
      reverse_ccw_wz_gain,
      ZERO_CMD_EPS,
      REVERSE_STRAIGHT_VX_BOOST};
  controller_math::ApplyNonLinearReverseCompensation(reverse_compensation_params,
                                                     &primary_compensation,
                                                     &command_effective);
  context.primary_command_twist_base = {
      command_effective.x(), command_effective.y(), command_effective.z()};
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

  Eigen::Matrix3d steer_priority_compensation = primary_compensation;
  steer_priority_compensation.row(0).setZero();
  const Eigen::Vector3d steer_priority_effective =
      steer_priority_compensation * command_input_base;
  context.steer_priority_command_twist_base = {
      steer_priority_effective.x(), steer_priority_effective.y(),
      steer_priority_effective.z()};
  return context;
}

WheelTargetPlan SolveWheelTargets(
    const WheelCommandContext& command_context, const Kinematics& kinematics,
    const std::array<double, SentryChassisController::WHEEL_COUNT>& steer_zero_offsets,
    const std::array<double, SentryChassisController::WHEEL_COUNT>& steer_positions,
    double steer_priority_error_threshold)
{
  WheelTargetPlan target_plan;
  for (std::size_t i = 0; i < SentryChassisController::WHEEL_COUNT; ++i)
  {
    target_plan.steer_errors[i] =
        SentryChassisController::NormalizeAngle(steer_zero_offsets[i] - steer_positions[i]);
    target_plan.wheel_targets[i] = 0.0;
    target_plan.alignments[i] = 1.0;
    target_plan.wheel_pid_reset_flags[i] = command_context.zero_command_requested;
  }
  if (command_context.zero_command_requested)
  {
    return target_plan;
  }

  const Kinematics::WheelTargetSolution primary_targets = kinematics.ComputeWheelTargets(
      command_context.primary_command_twist_base, steer_zero_offsets, steer_positions);
  const Kinematics::WheelTargetSolution steer_priority_targets =
      command_context.use_steer_priority
          ? kinematics.ComputeWheelTargets(command_context.steer_priority_command_twist_base,
                                           steer_zero_offsets, steer_positions)
          : Kinematics::WheelTargetSolution();

  for (std::size_t i = 0; i < SentryChassisController::WHEEL_COUNT; ++i)
  {
    const Kinematics::WheelTarget* selected_target = &primary_targets.modules[i];
    if (selected_target->active && command_context.use_steer_priority &&
        std::fabs(selected_target->steer_error) > steer_priority_error_threshold &&
        steer_priority_targets.modules[i].active)
    {
      selected_target = &steer_priority_targets.modules[i];
    }
    if (selected_target->active)
    {
      target_plan.steer_errors[i] = selected_target->steer_error;
      target_plan.wheel_targets[i] = selected_target->wheel_angular_velocity;
      target_plan.alignments[i] = ComputeWheelAlignment(
          command_context.has_yaw_command, command_context.has_translation_command,
          selected_target->steer_error);
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

struct RuntimeParamSnapshot
{
  std::string odom_frame_id;
  std::string base_frame_id;
  int command_velocity_mode = 0;
  std::string command_frame_id;
  bool publish_tf = true;
  uint64_t odom_frame_config_version = 0;
  uint64_t odom_publish_config_version = 0;
  uint64_t command_transform_config_version = 0;
};

struct RuntimeParamVersionChanges
{
  bool odom_frame_config_changed = false;
  bool odom_publish_config_changed = false;
  bool command_transform_config_changed = false;
};

struct DeferredWarningRule
{
  std::atomic<uint32_t>* counter = nullptr;
  const char* message = nullptr;
};

struct PowerLimitWarningSnapshot
{
  uint32_t count = 0U;
  double last_predicted_power_w = 0.0;
  double last_max_power_w = 0.0;
  double last_scale = 0.0;
};

bool TryBeginDeferredWarningFlush(std::atomic<int64_t>* next_flush_time_ns,
                                  int64_t now_ns)
{
  if (next_flush_time_ns == nullptr)
  {
    return false;
  }

  int64_t next_flush_time = next_flush_time_ns->load(std::memory_order_relaxed);
  if (now_ns < next_flush_time)
  {
    return false;
  }
  const int64_t reserved_next_flush_time =
      now_ns + DEFERRED_RT_WARN_FLUSH_INTERVAL_NS;
  return next_flush_time_ns->compare_exchange_strong(
      next_flush_time, reserved_next_flush_time, std::memory_order_relaxed,
      std::memory_order_relaxed);
}

void FlushDeferredWarningCounters(
    const std::array<DeferredWarningRule, 9>& warning_rules)
{
  for (const auto& rule : warning_rules)
  {
    const uint32_t count = rule.counter->exchange(0U, std::memory_order_relaxed);
    if (count > 0U)
    {
      ROS_WARN("%s (count=%u).", rule.message, count);
    }
  }
}

PowerLimitWarningSnapshot ConsumePowerLimitWarningSnapshot(
    std::atomic<uint32_t>* count_counter,
    std::atomic<int32_t>* last_predicted_milliwatt,
    std::atomic<int32_t>* last_max_milliwatt,
    std::atomic<int32_t>* last_scale_milli)
{
  PowerLimitWarningSnapshot snapshot;
  if (count_counter == nullptr || last_predicted_milliwatt == nullptr ||
      last_max_milliwatt == nullptr || last_scale_milli == nullptr)
  {
    return snapshot;
  }

  snapshot.count = count_counter->exchange(0U, std::memory_order_relaxed);
  if (snapshot.count == 0U)
  {
    return snapshot;
  }

  snapshot.last_predicted_power_w =
      static_cast<double>(last_predicted_milliwatt->load(std::memory_order_relaxed)) /
      1000.0;
  snapshot.last_max_power_w =
      static_cast<double>(last_max_milliwatt->load(std::memory_order_relaxed)) / 1000.0;
  snapshot.last_scale =
      static_cast<double>(last_scale_milli->load(std::memory_order_relaxed)) / 1000.0;
  return snapshot;
}

void FlushPowerLimitWarning(const PowerLimitWarningSnapshot& snapshot)
{
  if (snapshot.count == 0U)
  {
    return;
  }
  ROS_WARN(
      "Power limit was active in realtime path (count=%u, "
      "last_predicted=%.3fW, last_max=%.3fW, last_scale=%.3f).",
      snapshot.count, snapshot.last_predicted_power_w, snapshot.last_max_power_w,
      snapshot.last_scale);
}

template <typename RuntimeParams>
RuntimeParamSnapshot CaptureRuntimeParamSnapshot(const RuntimeParams* previous_runtime_params,
                                                 const RuntimeParams& fallback)
{
  const RuntimeParams& source =
      previous_runtime_params != nullptr ? *previous_runtime_params : fallback;
  RuntimeParamSnapshot snapshot;
  snapshot.odom_frame_id = source.odom_frame_id;
  snapshot.base_frame_id = source.base_frame_id;
  snapshot.command_velocity_mode = static_cast<int>(source.command_velocity_mode);
  snapshot.command_frame_id = source.command_frame_id;
  snapshot.publish_tf = source.publish_tf;
  snapshot.odom_frame_config_version = source.odom_frame_config_version;
  snapshot.odom_publish_config_version = source.odom_publish_config_version;
  snapshot.command_transform_config_version = source.command_transform_config_version;
  return snapshot;
}

template <typename RuntimeParams>
void ClampWheelRadius(RuntimeParams* params)
{
  if (params->geometry.wheel_radius >= MIN_WHEEL_RADIUS)
  {
    return;
  }
  ROS_WARN("Parameter 'geometry/wheel_radius' must be >= %.9f. Clamping to %.9f.",
           MIN_WHEEL_RADIUS, MIN_WHEEL_RADIUS);
  params->geometry.wheel_radius = MIN_WHEEL_RADIUS;
}

template <typename RuntimeParams>
void ApplyNonNegativeZeroRules(RuntimeParams* params)
{
  struct Rule
  {
    const char* name;
    double* value;
  };
  const std::array<Rule, 2> rules = {{
      {"cmd_vel_timeout", &params->cmd_vel_timeout},
      {"odom_startup_hold_sec", &params->odom_startup_hold_sec},
  }};
  for (const auto& rule : rules)
  {
    if (*rule.value < 0.0)
    {
      ROS_WARN("Parameter '%s' is negative. Clamping to 0.0.", rule.name);
      *rule.value = 0.0;
    }
  }
}

template <typename RuntimeParams>
void ApplyPositiveFallbackRules(RuntimeParams* params)
{
  struct Rule
  {
    const char* name;
    double* value;
    double fallback;
  };
  const std::array<Rule, 6> rules = {{
      {"odom_max_linear_speed", &params->odom_max_linear_speed,
       DEFAULT_ODOM_MAX_LINEAR_SPEED},
      {"odom_max_angular_speed", &params->odom_max_angular_speed,
       DEFAULT_ODOM_MAX_ANGULAR_SPEED},
      {"wheel_effort_limit", &params->wheel_effort_limit, DEFAULT_WHEEL_EFFORT_LIMIT},
      {"max_linear_acceleration", &params->max_linear_acceleration,
       DEFAULT_MAX_LINEAR_ACCELERATION},
      {"max_angular_acceleration", &params->max_angular_acceleration,
       DEFAULT_MAX_ANGULAR_ACCELERATION},
      {"max_power", &params->max_power, DEFAULT_MAX_POWER},
  }};
  for (const auto& rule : rules)
  {
    if (*rule.value <= 0.0)
    {
      ROS_WARN("Parameter '%s' must be positive. Clamping to %.3f.", rule.name,
               rule.fallback);
      *rule.value = rule.fallback;
    }
  }
}

template <typename RuntimeParams>
void ApplyNonNegativeFallbackRules(RuntimeParams* params)
{
  struct Rule
  {
    const char* name;
    double* value;
    double fallback;
  };
  const std::array<Rule, 2> rules = {{
      {"power_loss_k1", &params->power_loss_k1, DEFAULT_POWER_LOSS_K1},
      {"power_loss_k2", &params->power_loss_k2, DEFAULT_POWER_LOSS_K2},
  }};
  for (const auto& rule : rules)
  {
    if (*rule.value < 0.0)
    {
      ROS_WARN("Parameter '%s' must be non-negative. Clamping to %.6f.", rule.name,
               rule.fallback);
      *rule.value = rule.fallback;
    }
  }
}

template <typename RuntimeParams>
void ApplyRangeClampRules(RuntimeParams* params)
{
  struct Rule
  {
    const char* name;
    double* value;
    double min;
    double max;
  };
  const std::array<Rule, 5> rules = {{
      {"reverse_ccw_vx_scale", &params->reverse_ccw_vx_scale,
       MIN_REVERSE_CCW_VX_SCALE, MAX_REVERSE_CCW_VX_SCALE},
      {"reverse_ccw_wz_gain", &params->reverse_ccw_wz_gain,
       MIN_REVERSE_CCW_WZ_GAIN, MAX_REVERSE_CCW_WZ_GAIN},
      {"reverse_ccw_vy_threshold", &params->reverse_ccw_vy_threshold,
       MIN_REVERSE_CCW_VY_THRESHOLD, MAX_REVERSE_CCW_VY_THRESHOLD},
      {"reverse_ccw_steer_priority_error", &params->reverse_ccw_steer_priority_error,
       MIN_REVERSE_CCW_STEER_PRIORITY_ERROR, MAX_REVERSE_CCW_STEER_PRIORITY_ERROR},
      {"min_power_scale", &params->min_power_scale, MIN_POWER_SCALE, MAX_POWER_SCALE},
  }};
  for (const auto& rule : rules)
  {
    if (*rule.value < rule.min || *rule.value > rule.max)
    {
      ROS_WARN("Parameter '%s' must be in [%.3f, %.3f]. Clamping.", rule.name,
               rule.min, rule.max);
      *rule.value = std::max(rule.min, std::min(rule.max, *rule.value));
    }
  }
}

template <typename RuntimeParams>
void SanitizeNumericRuntimeParams(RuntimeParams* params)
{
  ClampWheelRadius(params);
  ApplyNonNegativeZeroRules(params);
  ApplyPositiveFallbackRules(params);
  ApplyNonNegativeFallbackRules(params);
  ApplyRangeClampRules(params);
}

template <typename RuntimeParams, typename ParseModeFn, typename ModeToTextFn>
bool NormalizeCommandVelocityMode(RuntimeParams* params,
                                  std::string* command_velocity_mode_text,
                                  bool strict_validation,
                                  ParseModeFn parse_command_velocity_mode,
                                  ModeToTextFn command_velocity_mode_to_text)
{
  auto parsed_mode = params->command_velocity_mode;
  if (parse_command_velocity_mode(*command_velocity_mode_text, &parsed_mode))
  {
    params->command_velocity_mode = parsed_mode;
    return true;
  }
  if (strict_validation)
  {
    ROS_ERROR(
        "Parameter 'command_velocity_mode' must be 'base_link' or 'global', got '%s'.",
        command_velocity_mode_text->c_str());
    return false;
  }
  ROS_WARN_THROTTLE(1.0,
                    "Dynamic command_velocity_mode '%s' is invalid. Keeping previous mode.",
                    command_velocity_mode_text->c_str());
  *command_velocity_mode_text =
      command_velocity_mode_to_text(params->command_velocity_mode);
  return true;
}

template <typename RuntimeParams>
void SanitizeRuntimeFrames(RuntimeParams* params,
                           const std::string& command_velocity_mode_text)
{
  if (params->base_frame_id.empty())
  {
    ROS_WARN("Parameter 'base_frame_id' is empty. Falling back to 'base_link'.");
    params->base_frame_id = "base_link";
  }
  if (params->odom_frame_id.empty())
  {
    ROS_WARN("Parameter 'odom_frame_id' is empty. Falling back to 'odom'.");
    params->odom_frame_id = "odom";
  }
  if (params->command_frame_id.empty())
  {
    const std::string fallback_command_frame =
        command_velocity_mode_text == "base_link" ? params->base_frame_id
                                                   : params->odom_frame_id;
    ROS_WARN("Parameter 'command_frame_id' is empty. Falling back to '%s'.",
             fallback_command_frame.c_str());
    params->command_frame_id = fallback_command_frame;
  }

  if (command_velocity_mode_text == "base_link" &&
      params->command_frame_id != params->base_frame_id)
  {
    ROS_WARN(
        "Parameter 'command_frame_id' is '%s' while command_velocity_mode is "
        "'base_link'. Falling back to '%s'.",
        params->command_frame_id.c_str(), params->base_frame_id.c_str());
    params->command_frame_id = params->base_frame_id;
  }
  if (command_velocity_mode_text == "global" &&
      params->command_frame_id == params->base_frame_id)
  {
    ROS_WARN_THROTTLE(
        1.0,
        "command_velocity_mode is 'global' but command_frame_id equals base frame '%s'. "
        "Global transform will have no effect.",
        params->base_frame_id.c_str());
  }
}

template <typename RuntimeParams, typename ParseModeFn, typename ModeToTextFn>
bool NormalizeRuntimeFrameParams(RuntimeParams* params,
                                 std::string* command_velocity_mode_text,
                                 bool strict_validation,
                                 ParseModeFn parse_command_velocity_mode,
                                 ModeToTextFn command_velocity_mode_to_text)
{
  if (!NormalizeCommandVelocityMode(params, command_velocity_mode_text, strict_validation,
                                    parse_command_velocity_mode,
                                    command_velocity_mode_to_text))
  {
    return false;
  }
  SanitizeRuntimeFrames(params, *command_velocity_mode_text);
  return true;
}

template <typename RuntimeParams>
void WarnOnOdomFrameChange(const RuntimeParamSnapshot& previous_runtime_params,
                           const RuntimeParams& params, bool strict_validation)
{
  if (strict_validation ||
      (previous_runtime_params.odom_frame_id == params.odom_frame_id &&
       previous_runtime_params.base_frame_id == params.base_frame_id))
  {
    return;
  }
  ROS_WARN(
      "Odometry frame parameters changed (odom: '%s' -> '%s', base: '%s' -> '%s'). "
      "Resetting accumulated odometry state.",
      previous_runtime_params.odom_frame_id.c_str(), params.odom_frame_id.c_str(),
      previous_runtime_params.base_frame_id.c_str(), params.base_frame_id.c_str());
}

template <typename RuntimeParams>
RuntimeParamVersionChanges ApplyRuntimeParamVersioning(
    const RuntimeParamSnapshot& previous_runtime_params, RuntimeParams* params)
{
  RuntimeParamVersionChanges changes;
  changes.odom_frame_config_changed =
      previous_runtime_params.odom_frame_id != params->odom_frame_id ||
      previous_runtime_params.base_frame_id != params->base_frame_id;
  params->odom_frame_config_version =
      changes.odom_frame_config_changed
          ? previous_runtime_params.odom_frame_config_version + 1U
          : previous_runtime_params.odom_frame_config_version;

  changes.odom_publish_config_changed =
      changes.odom_frame_config_changed ||
      previous_runtime_params.publish_tf != params->publish_tf;
  params->odom_publish_config_version =
      changes.odom_publish_config_changed
          ? previous_runtime_params.odom_publish_config_version + 1U
          : previous_runtime_params.odom_publish_config_version;

  changes.command_transform_config_changed =
      previous_runtime_params.command_velocity_mode !=
          static_cast<int>(params->command_velocity_mode) ||
      previous_runtime_params.command_frame_id != params->command_frame_id ||
      previous_runtime_params.base_frame_id != params->base_frame_id;
  params->command_transform_config_version =
      changes.command_transform_config_changed
          ? previous_runtime_params.command_transform_config_version + 1U
          : previous_runtime_params.command_transform_config_version;
  return changes;
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
  const RuntimeParamSnapshot previous_runtime_params =
      CaptureRuntimeParamSnapshot(runtime_params_buffer_.readFromNonRT(), params);

  const auto parse_command_velocity_mode =
      [](const std::string& mode_text, auto* parsed_mode) {
        return SentryChassisController::ParseCommandVelocityMode(mode_text, parsed_mode);
      };
  const auto command_velocity_mode_to_text = [](const CommandVelocityMode mode) {
    return mode == CommandVelocityMode::BASE_LINK ? "base_link" : "global";
  };

  SanitizeNumericRuntimeParams(&params);
  if (!NormalizeRuntimeFrameParams(&params, &command_velocity_mode_text_,
                                   strict_validation, parse_command_velocity_mode,
                                   command_velocity_mode_to_text))
  {
    return false;
  }
  WarnOnOdomFrameChange(previous_runtime_params, params, strict_validation);

  const RuntimeParamVersionChanges version_changes =
      ApplyRuntimeParamVersioning(previous_runtime_params, &params);
  runtime_params_buffer_.writeFromNonRT(params);
  if (version_changes.command_transform_config_changed)
  {
    InvalidateCommandTransformCache();
    RefreshCommandTransformCache(ros::TimerEvent());
  }
  return true;
}

void SentryChassisController::InvalidateCommandTransformCache()
{
  command_transform_buffer_.writeFromNonRT(BuildInvalidCommandTransformCache());
}

SentryChassisController::CommandTransformCache
SentryChassisController::BuildInvalidCommandTransformCache() const
{
  CommandTransformCache cache;
  cache.valid = false;
  cache.stamp = ros::Time(0);
  return cache;
}

SentryChassisController::CommandTransformCache
SentryChassisController::BuildIdentityCommandTransformCache(
    const RuntimeParams& runtime_params, const ros::Time& stamp) const
{
  CommandTransformCache cache;
  cache.valid = true;
  cache.stamp = stamp;
  cache.command_transform_config_version =
      runtime_params.command_transform_config_version;
  return cache;
}

bool SentryChassisController::TryLookupCommandTransform(
    const RuntimeParams& runtime_params,
    geometry_msgs::TransformStamped* command_to_base_transform) const
{
  if (command_to_base_transform == nullptr)
  {
    return false;
  }
  if (!tf_buffer_)
  {
    ROS_WARN_THROTTLE(
        1.0,
        "Global command mode requires TF listener, but tf_buffer is not initialized.");
    return false;
  }

  std::string transform_error;
  const bool transform_ready = tf_buffer_->canTransform(
      runtime_params.base_frame_id, runtime_params.command_frame_id, ros::Time(0),
      ros::Duration(0.0), &transform_error);
  if (!transform_ready)
  {
    ROS_WARN_THROTTLE(1.0,
                      "Failed to refresh cmd_vel transform from '%s' to '%s': %s",
                      runtime_params.command_frame_id.c_str(),
                      runtime_params.base_frame_id.c_str(),
                      transform_error.c_str());
    return false;
  }

  try
  {
    *command_to_base_transform = tf_buffer_->lookupTransform(
        runtime_params.base_frame_id, runtime_params.command_frame_id,
        ros::Time(0), ros::Duration(0.0));
  }
  catch (const tf2::TransformException& exception)
  {
    ROS_WARN_THROTTLE(1.0,
                      "Failed to refresh cmd_vel transform from '%s' to '%s': %s",
                      runtime_params.command_frame_id.c_str(),
                      runtime_params.base_frame_id.c_str(),
                      exception.what());
    return false;
  }
  return true;
}

bool SentryChassisController::TryBuildCommandTransformCacheFromTransform(
    const RuntimeParams& runtime_params,
    const geometry_msgs::TransformStamped& command_to_base_transform,
    CommandTransformCache* cache) const
{
  if (cache == nullptr)
  {
    return false;
  }

  Eigen::Matrix3d source_to_target_rotation;
  if (!controller_math::BuildRotationFromQuaternion(
          command_to_base_transform.transform.rotation, MIN_QUATERNION_NORM,
          &source_to_target_rotation))
  {
    ROS_WARN_THROTTLE(
        1.0,
        "Failed to refresh cmd_vel transform from '%s' to '%s': invalid quaternion.",
        runtime_params.command_frame_id.c_str(),
        runtime_params.base_frame_id.c_str());
    return false;
  }

  *cache = BuildIdentityCommandTransformCache(runtime_params,
                                              command_to_base_transform.header.stamp);
  for (int row = 0; row < 3; ++row)
  {
    for (int col = 0; col < 3; ++col)
    {
      cache->rotation_matrix_row_major[static_cast<std::size_t>(row * 3 + col)] =
          source_to_target_rotation(row, col);
    }
  }
  return true;
}

bool SentryChassisController::TryBuildRefreshedCommandTransformCache(
    const RuntimeParams& runtime_params, CommandTransformCache* cache) const
{
  if (cache == nullptr)
  {
    return false;
  }
  if (runtime_params.command_frame_id == runtime_params.base_frame_id)
  {
    *cache = BuildIdentityCommandTransformCache(runtime_params, ros::Time::now());
    return true;
  }

  geometry_msgs::TransformStamped command_to_base_transform;
  if (!TryLookupCommandTransform(runtime_params, &command_to_base_transform))
  {
    return false;
  }
  return TryBuildCommandTransformCacheFromTransform(runtime_params,
                                                    command_to_base_transform, cache);
}

void SentryChassisController::RefreshCommandTransformCache(
    const ros::TimerEvent& event)
{
  (void)event;
  FlushDeferredRealtimeWarnings();
  const RuntimeParams* runtime_params = runtime_params_buffer_.readFromNonRT();
  if (runtime_params == nullptr ||
      runtime_params->command_velocity_mode != CommandVelocityMode::GLOBAL)
  {
    InvalidateCommandTransformCache();
    return;
  }

  CommandTransformCache cache;
  if (!TryBuildRefreshedCommandTransformCache(*runtime_params, &cache))
  {
    InvalidateCommandTransformCache();
    return;
  }
  command_transform_buffer_.writeFromNonRT(cache);
}

void SentryChassisController::FlushDeferredRealtimeWarnings()
{
  const int64_t now_ns = static_cast<int64_t>(ros::WallTime::now().toNSec());
  if (!TryBeginDeferredWarningFlush(&rt_warn_next_flush_time_ns_, now_ns))
  {
    return;
  }

  const std::array<DeferredWarningRule, 9> warning_rules = {{
      {&rt_warn_invalid_period_count_,
       "Realtime update skipped due to non-positive period"},
      {&rt_warn_runtime_params_unready_count_,
       "Realtime update skipped because runtime params are not ready"},
      {&rt_warn_command_buffer_unready_count_,
       "Realtime update skipped because command buffer is not ready"},
      {&rt_warn_prepare_command_failed_count_,
       "Realtime update skipped because command preparation failed"},
      {&rt_warn_transform_cache_unready_count_,
       "Global command transform cache was not ready in realtime path"},
      {&rt_warn_transform_cache_stale_count_,
       "Global command transform cache was stale or mismatched in realtime path"},
      {&rt_warn_odom_singular_count_,
       "Forward kinematics was singular during realtime odometry"},
      {&rt_warn_odom_startup_hold_count_,
       "Odom integration was suppressed by startup hold in realtime path"},
      {&rt_warn_odom_rejected_count_,
       "Abnormal odom twist was rejected in realtime path"},
  }};
  FlushDeferredWarningCounters(warning_rules);

  FlushPowerLimitWarning(ConsumePowerLimitWarningSnapshot(
      &rt_warn_power_limit_active_count_,
      &rt_warn_power_limit_last_predicted_milliwatt_,
      &rt_warn_power_limit_last_max_milliwatt_,
      &rt_warn_power_limit_last_scale_milli_));
}

void SentryChassisController::InvalidateOdomPublishState()
{
  const uint64_t BEGIN_SEQUENCE = odom_publish_state_seq_.load(std::memory_order_relaxed);
  odom_publish_state_seq_.store(BEGIN_SEQUENCE + 1U, std::memory_order_release);
  odom_publish_stamp_ns_.store(0U, std::memory_order_relaxed);
  odom_publish_state_config_version_.store(0U, std::memory_order_relaxed);
  odom_publish_x_.store(0.0, std::memory_order_relaxed);
  odom_publish_y_.store(0.0, std::memory_order_relaxed);
  odom_publish_yaw_.store(0.0, std::memory_order_relaxed);
  odom_publish_vx_.store(0.0, std::memory_order_relaxed);
  odom_publish_vy_.store(0.0, std::memory_order_relaxed);
  odom_publish_wz_.store(0.0, std::memory_order_relaxed);
  odom_publish_valid_.store(false, std::memory_order_relaxed);
  odom_publish_state_seq_.store(BEGIN_SEQUENCE + 2U, std::memory_order_release);
}

void SentryChassisController::StageOdometryPublishState(
    const ros::Time& time, const Kinematics::ChassisTwist& twist,
    const RuntimeParams& runtime_params)
{
  const uint64_t BEGIN_SEQUENCE = odom_publish_state_seq_.load(std::memory_order_relaxed);
  odom_publish_state_seq_.store(BEGIN_SEQUENCE + 1U, std::memory_order_release);
  odom_publish_stamp_ns_.store(time.toNSec(), std::memory_order_relaxed);
  odom_publish_state_config_version_.store(runtime_params.odom_publish_config_version,
                                           std::memory_order_relaxed);
  odom_publish_x_.store(odom_state_.x, std::memory_order_relaxed);
  odom_publish_y_.store(odom_state_.y, std::memory_order_relaxed);
  odom_publish_yaw_.store(odom_state_.yaw, std::memory_order_relaxed);
  odom_publish_vx_.store(twist.vx, std::memory_order_relaxed);
  odom_publish_vy_.store(twist.vy, std::memory_order_relaxed);
  odom_publish_wz_.store(twist.wz, std::memory_order_relaxed);
  odom_publish_valid_.store(true, std::memory_order_relaxed);
  odom_publish_state_seq_.store(BEGIN_SEQUENCE + 2U, std::memory_order_release);
}

bool SentryChassisController::TryReadOdometryPublishState(
    OdomPublishState* snapshot) const
{
  if (snapshot == nullptr)
  {
    return false;
  }

  for (int attempt = 0; attempt < 3; ++attempt)
  {
    const uint64_t BEGIN_SEQUENCE =
        odom_publish_state_seq_.load(std::memory_order_acquire);
    if ((BEGIN_SEQUENCE & 1U) != 0U)
    {
      continue;
    }

    snapshot->stamp.fromNSec(odom_publish_stamp_ns_.load(std::memory_order_relaxed));
    snapshot->publish_config_version =
        odom_publish_state_config_version_.load(std::memory_order_relaxed);
    snapshot->odom_state.x = odom_publish_x_.load(std::memory_order_relaxed);
    snapshot->odom_state.y = odom_publish_y_.load(std::memory_order_relaxed);
    snapshot->odom_state.yaw = odom_publish_yaw_.load(std::memory_order_relaxed);
    snapshot->twist.vx = odom_publish_vx_.load(std::memory_order_relaxed);
    snapshot->twist.vy = odom_publish_vy_.load(std::memory_order_relaxed);
    snapshot->twist.wz = odom_publish_wz_.load(std::memory_order_relaxed);
    snapshot->valid = odom_publish_valid_.load(std::memory_order_relaxed);

    const uint64_t END_SEQUENCE =
        odom_publish_state_seq_.load(std::memory_order_acquire);
    if (BEGIN_SEQUENCE == END_SEQUENCE && (END_SEQUENCE & 1U) == 0U)
    {
      snapshot->sequence = END_SEQUENCE;
      return snapshot->valid;
    }
  }
  return false;
}

void SentryChassisController::FlushOdometryPublishState(const ros::TimerEvent& event)
{
  (void)event;

  OdomPublishState snapshot;
  if (!TryReadOdometryPublishState(&snapshot) ||
      snapshot.sequence == last_published_odom_sequence_)
  {
    return;
  }

  const RuntimeParams* runtime_params = runtime_params_buffer_.readFromNonRT();
  if (runtime_params == nullptr ||
      snapshot.publish_config_version != runtime_params->odom_publish_config_version)
  {
    return;
  }

  nav_msgs::Odometry odometry;
  odometry.header.stamp = snapshot.stamp;
  odometry.header.frame_id = runtime_params->odom_frame_id;
  odometry.child_frame_id = runtime_params->base_frame_id;
  odometry.pose.pose.position.x = snapshot.odom_state.x;
  odometry.pose.pose.position.y = snapshot.odom_state.y;
  odometry.pose.pose.position.z = 0.0;

  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, snapshot.odom_state.yaw);
  quaternion.normalize();

  odometry.pose.pose.orientation.x = quaternion.x();
  odometry.pose.pose.orientation.y = quaternion.y();
  odometry.pose.pose.orientation.z = quaternion.z();
  odometry.pose.pose.orientation.w = quaternion.w();
  odometry.twist.twist.linear.x = snapshot.twist.vx;
  odometry.twist.twist.linear.y = snapshot.twist.vy;
  odometry.twist.twist.angular.z = snapshot.twist.wz;
  if (odom_publisher_)
  {
    odom_publisher_.publish(odometry);
  }

  if (runtime_params->publish_tf && tf_broadcaster_)
  {
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = snapshot.stamp;
    transform.header.frame_id = runtime_params->odom_frame_id;
    transform.child_frame_id = runtime_params->base_frame_id;
    transform.transform.translation.x = snapshot.odom_state.x;
    transform.transform.translation.y = snapshot.odom_state.y;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = odometry.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  last_published_odom_sequence_ = snapshot.sequence;
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

  const bool ODOM_FRAME_CONFIG_CHANGED =
      runtime_params.odom_frame_config_version != applied_odom_frame_config_version_;
  if (ODOM_FRAME_CONFIG_CHANGED)
  {
    odom_state_ = OdomState();
    applied_odom_frame_config_version_ = runtime_params.odom_frame_config_version;
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

  const PowerLimitComputation computation = ComputePowerLimitComputation(
      runtime_params, signed_wheel_velocities, *signed_wheel_efforts);
  if (computation.predicted_input_power <= runtime_params.max_power)
  {
    return;
  }

  const double scale = ComputePowerLimitScale(runtime_params, computation);
  ScaleWheelEfforts(scale, signed_wheel_efforts);

  if (runtime_params.enable_power_limit_logging)
  {
    StorePowerLimitWarningSnapshot(
        &rt_warn_power_limit_active_count_,
        &rt_warn_power_limit_last_predicted_milliwatt_,
        &rt_warn_power_limit_last_max_milliwatt_,
        &rt_warn_power_limit_last_scale_milli_, computation,
        runtime_params.max_power, scale);
  }
}

bool SentryChassisController::ResolveCommandInBaseFrame(const CommandData& command,
                                                        const ros::Time& time,
                                                        const RuntimeParams& runtime_params,
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

  if (runtime_params.command_velocity_mode == CommandVelocityMode::BASE_LINK ||
      runtime_params.command_frame_id == runtime_params.base_frame_id)
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
  if (command_transform->command_transform_config_version !=
      runtime_params.command_transform_config_version)
  {
    rt_warn_transform_cache_stale_count_.fetch_add(1U, std::memory_order_relaxed);
    return false;
  }
  if (command_transform->stamp.isZero() ||
      command_transform->stamp > time ||
      (time - command_transform->stamp).toSec() > COMMAND_TRANSFORM_CACHE_MAX_AGE_SEC)
  {
    rt_warn_transform_cache_stale_count_.fetch_add(1U, std::memory_order_relaxed);
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

bool SentryChassisController::ComputeCommandTimeout(
    const CommandData& command, const ros::Time& time,
    const RuntimeParams& runtime_params) const
{
  const bool command_from_current_session = command.stamp >= controller_start_time_;
  const bool command_valid_for_update = command.valid && command_from_current_session;
  return IsCommandTimedOut(command_valid_for_update, command.stamp, time,
                           runtime_params.cmd_vel_timeout);
}

void SentryChassisController::UpdateCommandTimeoutState(bool timeout)
{
  if (timeout && !last_command_timed_out_)
  {
    for (auto& pid : wheel_pids_)
    {
      pid.reset();
    }
  }
  last_command_timed_out_ = timeout;
}

void SentryChassisController::PrepareBaseCommandTwistForControl(
    const CommandData& command, const ros::Time& time,
    const RuntimeParams& runtime_params, bool timeout,
    Kinematics::ChassisTwist* command_twist_base)
{
  if (command_twist_base == nullptr)
  {
    return;
  }

  *command_twist_base = Kinematics::ChassisTwist();
  if (timeout)
  {
    has_last_limited_command_ = false;
    return;
  }
  if (!ResolveCommandInBaseFrame(command, time, runtime_params, command_twist_base))
  {
    *command_twist_base = Kinematics::ChassisTwist();
  }
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

  const bool command_timed_out =
      ComputeCommandTimeout(command, time, runtime_params);
  UpdateCommandTimeoutState(command_timed_out);

  Kinematics::ChassisTwist command_twist_base;
  PrepareBaseCommandTwistForControl(command, time, runtime_params,
                                    command_timed_out, &command_twist_base);
  *limited_command_twist_base = command_twist_base;
  ApplyAccelerationLimits(command_twist_base, dt, runtime_params,
                          limited_command_twist_base);
  *timeout = command_timed_out;
  return true;
}

void SentryChassisController::ComputeAndApplyWheelControl(
    const Kinematics::ChassisTwist& limited_command_twist_base,
    const ros::Duration& period, const RuntimeParams& runtime_params)
{
  const WheelMotionState motion_state =
      ReadWheelMotionState(steer_joints_, wheel_joints_);

  // Stage 1: constraint/compensation on chassis command (pure).
  const WheelCommandContext command_context = ApplyWheelCommandConstraintsAndCompensation(
      limited_command_twist_base, command_compensation_matrix_,
      runtime_params.reverse_ccw_vy_threshold, runtime_params.reverse_ccw_vx_scale,
      runtime_params.reverse_ccw_wz_gain);
  // Stage 2: per-wheel target solving (pure).
  const WheelTargetPlan target_plan =
      SolveWheelTargets(command_context, kinematics_, steer_zero_offsets_,
                        motion_state.steer_positions,
                        runtime_params.reverse_ccw_steer_priority_error);

  // Stage 3 preparation: run stateful PID loops against solved targets.
  ApplySteerJointCommands(target_plan, period, &steer_pids_, &steer_joints_);

  WheelEffortState effort_state = ComputeWheelEffortState(
      target_plan, motion_state, period, runtime_params.wheel_effort_limit,
      wheel_rolling_signs_, &wheel_pids_);
  ApplyPowerLimiting(runtime_params, effort_state.signed_wheel_velocities,
                     &effort_state.signed_wheel_efforts);

  // Stage 3: build per-wheel dispatch commands (pure), then apply to hardware.
  ApplyWheelDispatchCommands(wheel_rolling_signs_, effort_state,
                             target_plan.wheel_pid_reset_flags, &wheel_joints_);
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
  InvalidateOdomPublishState();
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
  StageOdometryPublishState(time, ODOM_TWIST, *runtime_params);
}

void SentryChassisController::stopping(const ros::Time& time)
{
  (void)time;
  ResetControllerOutputsAndPids();
  ResetControllerTrackingState(ros::Time(0));
  InvalidateOdomPublishState();
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

}  // namespace sentry_chassis_controller
