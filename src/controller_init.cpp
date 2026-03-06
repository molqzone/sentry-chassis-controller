#include "sentry_chassis_controller/sentry_chassis_controller.h"
#include "controller_internal.hpp"

#include <Eigen/Dense>
#include <hardware_interface/internal/hardware_resource_manager.h>

namespace sentry_chassis_controller
{
bool SentryChassisController::init(hardware_interface::EffortJointInterface* hw,
                                   ros::NodeHandle& nh)
{
  std::array<std::string, WHEEL_COUNT> steer_joint_names{};
  std::array<std::string, WHEEL_COUNT> wheel_joint_names{};
  if (!LoadControllerConfig(nh, &steer_joint_names, &wheel_joint_names))
  {
    return false;
  }
  if (!InitJointHandles(hw, steer_joint_names, wheel_joint_names))
  {
    return false;
  }
  if (!InitControllerPids(nh))
  {
    return false;
  }
  InitRealtimeState();
  InitRosInterfaces(nh);
  InitTfResources();
  InitCommandTransformCacheTimer(nh);
  InitOdomPublishTimer(nh);
  if (!InitRuntimeDynamicReconfigure(nh))
  {
    return false;
  }

  ROS_INFO(
      "SentryChassisController initialized with cmd_vel_topic='%s', "
      "command_velocity_mode='%s', command_frame_id='%s', "
      "timeout=%.3fs.",
      cmd_vel_topic_.c_str(),
      runtime_params_shadow_.command_velocity_mode == CommandVelocityMode::BASE_LINK
          ? "base_link"
          : "global",
      runtime_params_shadow_.command_frame_id.c_str(),
      runtime_params_shadow_.cmd_vel_timeout);
  ROS_INFO(
      "odometry output configured with topic='%s', odom_frame='%s', base_frame='%s', "
      "publish_tf=%s.",
      odom_topic_.c_str(), runtime_params_shadow_.odom_frame_id.c_str(),
      runtime_params_shadow_.base_frame_id.c_str(),
      runtime_params_shadow_.publish_tf ? "true" : "false");
  ROS_INFO(
      "odometry stabilization configured with startup_hold=%.3fs, max_linear=%.3fm/s, "
      "max_angular=%.3frad/s, integrate_on_timeout=%s.",
      runtime_params_shadow_.odom_startup_hold_sec,
      runtime_params_shadow_.odom_max_linear_speed,
      runtime_params_shadow_.odom_max_angular_speed,
      runtime_params_shadow_.odom_integrate_on_timeout ? "true" : "false");
  ROS_INFO(
      "acceleration limits configured with enabled=%s, max_linear=%.3fm/s^2, "
      "max_angular=%.3frad/s^2.",
      runtime_params_shadow_.enable_acceleration_limits ? "true" : "false",
      runtime_params_shadow_.max_linear_acceleration,
      runtime_params_shadow_.max_angular_acceleration);
  ROS_INFO(
      "power limiting configured with enabled=%s, logging=%s, max_power=%.3fW, "
      "k1=%.6f, k2=%.6f, min_scale=%.3f.",
      runtime_params_shadow_.enable_power_limit ? "true" : "false",
      runtime_params_shadow_.enable_power_limit_logging ? "true" : "false",
      runtime_params_shadow_.max_power, runtime_params_shadow_.power_loss_k1,
      runtime_params_shadow_.power_loss_k2, runtime_params_shadow_.min_power_scale);
  ROS_INFO("wheel effort command limit configured as %.3f.",
           runtime_params_shadow_.wheel_effort_limit);
  ROS_INFO(
      "wheel_direction_signs loaded: vx=[%d,%d,%d,%d], vy=[%d,%d,%d,%d], "
      "wz=[%d,%d,%d,%d].",
      direction_signs_.vx[0], direction_signs_.vx[1], direction_signs_.vx[2],
      direction_signs_.vx[3], direction_signs_.vy[0], direction_signs_.vy[1],
      direction_signs_.vy[2], direction_signs_.vy[3], direction_signs_.wz[0],
      direction_signs_.wz[1], direction_signs_.wz[2], direction_signs_.wz[3]);
  ROS_INFO("wheel_rolling_signs loaded: [%d,%d,%d,%d].", wheel_rolling_signs_[0],
           wheel_rolling_signs_[1], wheel_rolling_signs_[2], wheel_rolling_signs_[3]);
  return true;
}

bool SentryChassisController::LoadControllerConfig(
    ros::NodeHandle& nh, std::array<std::string, WHEEL_COUNT>* steer_joint_names,
    std::array<std::string, WHEEL_COUNT>* wheel_joint_names)
{
  if (!LoadRequiredFixedArrayParam(nh, "steer_joints", steer_joint_names))
  {
    return false;
  }
  if (!LoadRequiredFixedArrayParam(nh, "wheel_joints", wheel_joint_names))
  {
    return false;
  }

  std::vector<double> steer_zero_offsets_values;
  if (!LoadOptionalVectorParam(nh, "steer_zero_offsets",
                               std::vector<double>(WHEEL_COUNT, 0.0),
                               &steer_zero_offsets_values))
  {
    return false;
  }
  for (std::size_t i = 0; i < WHEEL_COUNT; ++i)
  {
    steer_zero_offsets_[i] = steer_zero_offsets_values[i];
  }

  std::vector<double> command_compensation_values;
  if (!LoadOptionalVectorParam(
          nh, "command_compensation_matrix",
          std::vector<double>(command_compensation_matrix_.begin(),
                              command_compensation_matrix_.end()),
          &command_compensation_values))
  {
    return false;
  }
  std::copy_n(command_compensation_values.begin(), command_compensation_matrix_.size(),
              command_compensation_matrix_.begin());

  runtime_params_shadow_ = RuntimeParams{};
  command_velocity_mode_text_ =
      runtime_params_shadow_.command_velocity_mode == CommandVelocityMode::BASE_LINK
          ? "base_link"
          : "global";

  ddynamic_reconfigure::DDynamicReconfigure parameter_loader(nh);
  parameter_loader.registerVariable<std::string>(
      "cmd_vel_topic", &cmd_vel_topic_, boost::function<void(std::string)>(),
      "cmd_vel topic name.", std::string(), std::string());
  parameter_loader.registerVariable<bool>("enable_dynamic_reconfigure",
                                          &enable_dynamic_reconfigure_,
                                          boost::function<void(bool)>(),
                                          "Enable dynamic reconfigure.", false, true);
  parameter_loader.registerVariable<std::string>("odom_topic", &odom_topic_,
                                                 boost::function<void(std::string)>(),
                                                 "Odometry topic name.",
                                                 std::string(), std::string());
  RegisterSharedRuntimeParameters(&parameter_loader);

  if (!ValidateAndApplyControllerParams(true))
  {
    return false;
  }

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
  applied_geometry_ = runtime_params_shadow_.geometry;
  kinematics_.SetGeometry(applied_geometry_);
  applied_odom_frame_id_ = runtime_params_shadow_.odom_frame_id;
  applied_base_frame_id_ = runtime_params_shadow_.base_frame_id;
  return true;
}

void SentryChassisController::RegisterSharedRuntimeParameters(
    ddynamic_reconfigure::DDynamicReconfigure* parameter_loader)
{
  if (parameter_loader == nullptr)
  {
    return;
  }

  parameter_loader->registerEnumVariable<std::string>(
      "command_velocity_mode", &command_velocity_mode_text_,
      "Velocity command interpretation mode.", COMMAND_VELOCITY_MODE_OPTIONS,
      "base_link/global");
  parameter_loader->registerVariable<std::string>(
      "command_frame_id", &runtime_params_shadow_.command_frame_id,
      boost::function<void(std::string)>(), "Velocity command frame id.",
      std::string(), std::string());
  parameter_loader->registerVariable<std::string>(
      "odom_frame_id", &runtime_params_shadow_.odom_frame_id, boost::function<void(std::string)>(),
      "Odometry frame id.", std::string(), std::string());
  parameter_loader->registerVariable<std::string>(
      "base_frame_id", &runtime_params_shadow_.base_frame_id, boost::function<void(std::string)>(),
      "Base frame id.", std::string(), std::string());
  parameter_loader->registerVariable<double>(
      "cmd_vel_timeout", &runtime_params_shadow_.cmd_vel_timeout, "cmd_vel timeout in seconds.", 0.0, 10.0);
  parameter_loader->registerVariable<double>(
      "odom_startup_hold_sec", &runtime_params_shadow_.odom_startup_hold_sec,
      "Startup hold window for odom integration.", 0.0, 20.0);
  parameter_loader->registerVariable<double>(
      "odom_max_linear_speed", &runtime_params_shadow_.odom_max_linear_speed,
      "Maximum acceptable odom linear speed.", MIN_VALID_DT, 50.0);
  parameter_loader->registerVariable<double>(
      "odom_max_angular_speed", &runtime_params_shadow_.odom_max_angular_speed,
      "Maximum acceptable odom angular speed.", MIN_VALID_DT, 100.0);
  parameter_loader->registerVariable<bool>(
      "odom_integrate_on_timeout", &runtime_params_shadow_.odom_integrate_on_timeout,
      boost::function<void(bool)>(), "Integrate odom when cmd_vel times out.", false,
      true);
  parameter_loader->registerVariable<bool>(
      "publish_tf", &runtime_params_shadow_.publish_tf, boost::function<void(bool)>(),
      "Publish odom to base_link transform.", false, true);
  parameter_loader->registerVariable<double>(
      "wheel_effort_limit", &runtime_params_shadow_.wheel_effort_limit,
      "Absolute wheel effort limit.", MIN_VALID_DT, 100.0);
  parameter_loader->registerVariable<double>(
      "reverse_ccw_vx_scale", &runtime_params_shadow_.reverse_ccw_vx_scale,
      "Reverse-CCW compensation vx scale.", MIN_REVERSE_CCW_VX_SCALE,
      MAX_REVERSE_CCW_VX_SCALE);
  parameter_loader->registerVariable<double>(
      "reverse_ccw_wz_gain", &runtime_params_shadow_.reverse_ccw_wz_gain,
      "Reverse-CCW compensation wz gain.", MIN_REVERSE_CCW_WZ_GAIN,
      MAX_REVERSE_CCW_WZ_GAIN);
  parameter_loader->registerVariable<double>(
      "reverse_ccw_vy_threshold", &runtime_params_shadow_.reverse_ccw_vy_threshold,
      "Reverse-CCW compensation |vy| trigger threshold.",
      MIN_REVERSE_CCW_VY_THRESHOLD, MAX_REVERSE_CCW_VY_THRESHOLD);
  parameter_loader->registerVariable<double>(
      "reverse_ccw_steer_priority_error", &runtime_params_shadow_.reverse_ccw_steer_priority_error,
      "Reverse-CCW steering-priority error threshold.",
      MIN_REVERSE_CCW_STEER_PRIORITY_ERROR, MAX_REVERSE_CCW_STEER_PRIORITY_ERROR);
  parameter_loader->registerVariable<bool>(
      "enable_acceleration_limits", &runtime_params_shadow_.enable_acceleration_limits,
      boost::function<void(bool)>(), "Enable chassis acceleration limits.", false, true);
  parameter_loader->registerVariable<double>(
      "max_linear_acceleration", &runtime_params_shadow_.max_linear_acceleration,
      "Maximum chassis linear acceleration in m/s^2.", MIN_ACCELERATION_LIMIT, 100.0);
  parameter_loader->registerVariable<double>(
      "max_angular_acceleration", &runtime_params_shadow_.max_angular_acceleration,
      "Maximum chassis angular acceleration in rad/s^2.", MIN_ACCELERATION_LIMIT, 200.0);
  parameter_loader->registerVariable<bool>(
      "enable_power_limit", &runtime_params_shadow_.enable_power_limit, boost::function<void(bool)>(),
      "Enable wheel power limiting.", false, true);
  parameter_loader->registerVariable<bool>(
      "enable_power_limit_logging", &runtime_params_shadow_.enable_power_limit_logging,
      boost::function<void(bool)>(), "Enable power limiting status logging.", false, true);
  parameter_loader->registerVariable<double>(
      "max_power", &runtime_params_shadow_.max_power, "Maximum allowed wheel power in watts.",
      MIN_POWER_LIMIT, 2000.0);
  parameter_loader->registerVariable<double>(
      "power_loss_k1", &runtime_params_shadow_.power_loss_k1,
      "Quadratic torque loss coefficient for power model.", 0.0, 1.0);
  parameter_loader->registerVariable<double>(
      "power_loss_k2", &runtime_params_shadow_.power_loss_k2,
      "Quadratic speed loss coefficient for power model.", 0.0, 1.0);
  parameter_loader->registerVariable<double>(
      "min_power_scale", &runtime_params_shadow_.min_power_scale,
      "Lower bound for power limiting scale factor.", MIN_POWER_SCALE, MAX_POWER_SCALE);
  parameter_loader->registerVariable<double>(
      "geometry/wheel_base", &runtime_params_shadow_.geometry.wheel_base, "Wheel base in meters.", 0.0, 5.0);
  parameter_loader->registerVariable<double>(
      "geometry/track_width", &runtime_params_shadow_.geometry.track_width, "Track width in meters.", 0.0,
      5.0);
  parameter_loader->registerVariable<double>(
      "geometry/wheel_radius", &runtime_params_shadow_.geometry.wheel_radius, "Wheel radius in meters.",
      MIN_WHEEL_RADIUS, 1.0);
}

bool SentryChassisController::InitJointHandles(
    hardware_interface::EffortJointInterface* hw,
    const std::array<std::string, WHEEL_COUNT>& steer_joint_names,
    const std::array<std::string, WHEEL_COUNT>& wheel_joint_names)
{
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
  return true;
}

bool SentryChassisController::InitControllerPids(ros::NodeHandle& nh)
{
  return InitPidGroup(nh, "steer", &steer_pids_) &&
         InitPidGroup(nh, "wheel", &wheel_pids_);
}

void SentryChassisController::InitRealtimeState()
{
  CommandData command;
  command.stamp = ros::Time(0);
  command.valid = false;
  command_buffer_.initRT(command);
  CommandTransformCache command_transform_cache;
  command_transform_cache.valid = false;
  command_transform_buffer_.initRT(command_transform_cache);
  odom_state_ = OdomState();
  odom_publish_state_seq_.store(0U, std::memory_order_relaxed);
  odom_publish_stamp_ns_.store(0U, std::memory_order_relaxed);
  odom_publish_x_.store(0.0, std::memory_order_relaxed);
  odom_publish_y_.store(0.0, std::memory_order_relaxed);
  odom_publish_yaw_.store(0.0, std::memory_order_relaxed);
  odom_publish_vx_.store(0.0, std::memory_order_relaxed);
  odom_publish_vy_.store(0.0, std::memory_order_relaxed);
  odom_publish_wz_.store(0.0, std::memory_order_relaxed);
  odom_publish_valid_.store(false, std::memory_order_relaxed);
  last_published_odom_sequence_ = 0U;
  last_limited_command_ = Kinematics::ChassisTwist();
  has_last_limited_command_ = false;
  controller_start_time_ = ros::Time(0);
  last_command_timed_out_ = true;
}

void SentryChassisController::InitRosInterfaces(ros::NodeHandle& nh)
{
  cmd_vel_subscriber_ =
      nh.subscribe(cmd_vel_topic_, 1, &SentryChassisController::CmdVelCallback, this);
  odom_publisher_ = nh.advertise<nav_msgs::Odometry>(odom_topic_, 10);
}

void SentryChassisController::InitTfResources()
{
  // Allocate TF resources in init() to avoid allocations inside realtime update().
  const bool RUNTIME_TF_RECONFIGURE = enable_dynamic_reconfigure_;
  if (runtime_params_shadow_.publish_tf || RUNTIME_TF_RECONFIGURE)
  {
    tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster());
  }
  else
  {
    tf_broadcaster_.reset();
  }
  if (runtime_params_shadow_.command_velocity_mode == CommandVelocityMode::GLOBAL ||
      RUNTIME_TF_RECONFIGURE)
  {
    tf_buffer_.reset(new tf2_ros::Buffer(ros::Duration(10.0)));
    tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));
  }
  else
  {
    tf_listener_.reset();
    tf_buffer_.reset();
  }
}

void SentryChassisController::InitCommandTransformCacheTimer(ros::NodeHandle& nh)
{
  command_transform_cache_timer_.stop();
  command_transform_cache_timer_ = nh.createTimer(
      ros::Duration(COMMAND_TRANSFORM_CACHE_UPDATE_SEC),
      &SentryChassisController::RefreshCommandTransformCache, this);
  RefreshCommandTransformCache(ros::TimerEvent());
}

void SentryChassisController::InitOdomPublishTimer(ros::NodeHandle& nh)
{
  odom_publish_timer_.stop();
  odom_publish_timer_ = nh.createTimer(ros::Duration(ODOM_PUBLISH_FLUSH_SEC),
                                       &SentryChassisController::FlushOdometryPublishState,
                                       this);
}

bool SentryChassisController::InitRuntimeDynamicReconfigure(ros::NodeHandle& nh)
{
  if (!enable_dynamic_reconfigure_)
  {
    controller_params_reconfigure_.reset();
    return true;
  }

  controller_params_reconfigure_.reset(
      new ddynamic_reconfigure::DDynamicReconfigure(nh, true));
  RegisterSharedRuntimeParameters(controller_params_reconfigure_.get());
  controller_params_reconfigure_->setPostUpdateCallback([this]() {
    ValidateAndApplyControllerParams(false);
  });
  controller_params_reconfigure_->publishServicesTopics();
  return ValidateAndApplyControllerParams(true);
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


}  // namespace sentry_chassis_controller
