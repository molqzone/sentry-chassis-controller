#include "sentry_chassis_controller/sentry_chassis_controller.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <geometry_msgs/TransformStamped.h>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <nav_msgs/Odometry.h>
#include <pluginlib/class_list_macros.h>
#include <sophus/se2.hpp>
#include <sophus/so2.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>

#include <array>
#include <algorithm>
#include <cmath>
#include <map>
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
constexpr double STEER_FLIP_EPS = 0.05;
constexpr double DEFAULT_WHEEL_BASE = 0.50;
constexpr double DEFAULT_TRACK_WIDTH = 0.40;
constexpr double DEFAULT_WHEEL_RADIUS = 0.076;
constexpr double DEFAULT_CMD_VEL_TIMEOUT = 0.25;
constexpr double DEFAULT_ODOM_STARTUP_HOLD = 1.0;
constexpr double DEFAULT_ODOM_MAX_LINEAR_SPEED = 8.0;
constexpr double DEFAULT_ODOM_MAX_ANGULAR_SPEED = 16.0;
constexpr double DEFAULT_WHEEL_EFFORT_LIMIT = 12.0;
constexpr double DEFAULT_REVERSE_CCW_VX_SCALE = 1.0;
constexpr double DEFAULT_REVERSE_CCW_WZ_GAIN = 1.0;
constexpr double DEFAULT_REVERSE_CCW_VY_THRESHOLD = 0.03;
constexpr double DEFAULT_REVERSE_CCW_STEER_PRIORITY_ERROR = 0.6;
constexpr double MIN_REVERSE_CCW_VX_SCALE = 0.1;
constexpr double MAX_REVERSE_CCW_VX_SCALE = 1.0;
constexpr double MIN_REVERSE_CCW_WZ_GAIN = 1.0;
constexpr double MAX_REVERSE_CCW_WZ_GAIN = 3.0;
constexpr double MIN_REVERSE_CCW_VY_THRESHOLD = 0.0;
constexpr double MAX_REVERSE_CCW_VY_THRESHOLD = 0.5;
constexpr double MIN_REVERSE_CCW_STEER_PRIORITY_ERROR = 0.0;
constexpr double MAX_REVERSE_CCW_STEER_PRIORITY_ERROR = PI;
constexpr double REVERSE_STRAIGHT_VX_BOOST = 1.10;
constexpr double MIN_QUATERNION_NORM = 1e-12;
// 轮序后缀固定为前左、前右、后左、后右，需与配置文件和运动学保持一致。
// Wheel order suffix must stay aligned with config and kinematics.
const std::array<std::string, SentryChassisController::WHEEL_COUNT> WHEEL_NAME_SUFFIX = {
    "front_left", "front_right", "rear_left", "rear_right"};
const std::map<std::string, std::string> COMMAND_VELOCITY_MODE_OPTIONS = {
    {"base_link", "base_link"},
    {"global", "global"}};

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

bool BuildRotationFromQuaternion(const geometry_msgs::Quaternion& quaternion,
                                 Eigen::Matrix3d* rotation_matrix)
{
  if (rotation_matrix == nullptr)
  {
    return false;
  }

  const Eigen::Quaterniond RAW_QUATERNION(quaternion.w, quaternion.x, quaternion.y,
                                          quaternion.z);
  if (RAW_QUATERNION.norm() < MIN_QUATERNION_NORM)
  {
    return false;
  }

  const Eigen::Quaterniond NORMALIZED_QUATERNION = RAW_QUATERNION.normalized();
  *rotation_matrix = NORMALIZED_QUATERNION.toRotationMatrix();
  return true;
}
}  // namespace

bool SentryChassisController::init(hardware_interface::EffortJointInterface* hw,
                                   ros::NodeHandle& nh)
{
  std::array<std::string, WHEEL_COUNT> steer_joint_names{};
  std::array<std::string, WHEEL_COUNT> wheel_joint_names{};
  const auto LOAD_REQUIRED_STRING_ARRAY =
      [&nh](const std::string& param_name,
            std::array<std::string, WHEEL_COUNT>* output) -> bool {
    std::vector<std::string> raw_values;
    if (output == nullptr || !nh.getParam(param_name, raw_values) ||
        raw_values.size() != WHEEL_COUNT)
    {
      ROS_ERROR("Parameter '%s' must exist and contain exactly %zu items.",
                param_name.c_str(), WHEEL_COUNT);
      return false;
    }
    std::copy_n(raw_values.begin(), WHEEL_COUNT, output->begin());
    return true;
  };
  const auto LOAD_OPTIONAL_DOUBLE_ARRAY =
      [&nh](const std::string& param_name, std::vector<double>* output,
            const std::vector<double>& default_values) -> bool {
    if (output == nullptr)
    {
      return false;
    }
    std::vector<double> raw_values;
    if (!nh.getParam(param_name, raw_values))
    {
      *output = default_values;
      return true;
    }
    if (raw_values.size() != default_values.size())
    {
      ROS_ERROR("Parameter '%s' must contain exactly %zu items.", param_name.c_str(),
                default_values.size());
      return false;
    }
    *output = raw_values;
    return true;
  };

  if (!LOAD_REQUIRED_STRING_ARRAY("steer_joints", &steer_joint_names))
  {
    return false;
  }
  if (!LOAD_REQUIRED_STRING_ARRAY("wheel_joints", &wheel_joint_names))
  {
    return false;
  }

  std::vector<double> steer_zero_offsets_values;
  if (!LOAD_OPTIONAL_DOUBLE_ARRAY("steer_zero_offsets", &steer_zero_offsets_values,
                                  std::vector<double>(WHEEL_COUNT, 0.0)))
  {
    return false;
  }
  for (std::size_t i = 0; i < WHEEL_COUNT; ++i)
  {
    steer_zero_offsets_[i] = steer_zero_offsets_values[i];
  }

  std::vector<double> command_compensation_values;
  if (!LOAD_OPTIONAL_DOUBLE_ARRAY(
          "command_compensation_matrix", &command_compensation_values,
          std::vector<double>(command_compensation_matrix_.begin(),
                              command_compensation_matrix_.end())))
  {
    return false;
  }
  std::copy_n(command_compensation_values.begin(), command_compensation_matrix_.size(),
              command_compensation_matrix_.begin());

  geometry_.wheel_base = DEFAULT_WHEEL_BASE;
  geometry_.track_width = DEFAULT_TRACK_WIDTH;
  geometry_.wheel_radius = DEFAULT_WHEEL_RADIUS;
  cmd_vel_timeout_ = DEFAULT_CMD_VEL_TIMEOUT;
  command_velocity_mode_text_ = "base_link";
  odom_startup_hold_sec_ = DEFAULT_ODOM_STARTUP_HOLD;
  odom_max_linear_speed_ = DEFAULT_ODOM_MAX_LINEAR_SPEED;
  odom_max_angular_speed_ = DEFAULT_ODOM_MAX_ANGULAR_SPEED;
  wheel_effort_limit_ = DEFAULT_WHEEL_EFFORT_LIMIT;
  reverse_ccw_vx_scale_ = DEFAULT_REVERSE_CCW_VX_SCALE;
  reverse_ccw_wz_gain_ = DEFAULT_REVERSE_CCW_WZ_GAIN;
  reverse_ccw_vy_threshold_ = DEFAULT_REVERSE_CCW_VY_THRESHOLD;
  reverse_ccw_steer_priority_error_ = DEFAULT_REVERSE_CCW_STEER_PRIORITY_ERROR;

  ddynamic_reconfigure::DDynamicReconfigure parameter_loader(nh);
  parameter_loader.registerVariable<std::string>(
      "cmd_vel_topic", &cmd_vel_topic_, boost::function<void(std::string)>(),
      "cmd_vel topic name.", std::string(), std::string());
  parameter_loader.registerEnumVariable<std::string>(
      "command_velocity_mode", &command_velocity_mode_text_,
      "Velocity command interpretation mode.", COMMAND_VELOCITY_MODE_OPTIONS,
      "base_link/global");
  parameter_loader.registerVariable<std::string>(
      "command_frame_id", &command_frame_id_,
      boost::function<void(std::string)>(), "Velocity command frame id.",
      std::string(), std::string());
  parameter_loader.registerVariable<double>("cmd_vel_timeout", &cmd_vel_timeout_,
                                            "cmd_vel timeout in seconds.", 0.0, 10.0);
  parameter_loader.registerVariable<bool>("enable_dynamic_reconfigure",
                                          &enable_dynamic_reconfigure_,
                                          boost::function<void(bool)>(),
                                          "Enable dynamic reconfigure.", false, true);
  parameter_loader.registerVariable<std::string>("odom_topic", &odom_topic_,
                                                 boost::function<void(std::string)>(),
                                                 "Odometry topic name.",
                                                 std::string(), std::string());
  parameter_loader.registerVariable<std::string>("odom_frame_id", &odom_frame_id_,
                                                 boost::function<void(std::string)>(),
                                                 "Odometry frame id.",
                                                 std::string(), std::string());
  parameter_loader.registerVariable<std::string>("base_frame_id", &base_frame_id_,
                                                 boost::function<void(std::string)>(),
                                                 "Base frame id.",
                                                 std::string(), std::string());
  parameter_loader.registerVariable<double>(
      "odom_startup_hold_sec", &odom_startup_hold_sec_,
      "Startup hold window for odom integration.", 0.0, 20.0);
  parameter_loader.registerVariable<double>(
      "odom_max_linear_speed", &odom_max_linear_speed_,
      "Maximum acceptable odom linear speed.", MIN_VALID_DT, 50.0);
  parameter_loader.registerVariable<double>(
      "odom_max_angular_speed", &odom_max_angular_speed_,
      "Maximum acceptable odom angular speed.", MIN_VALID_DT, 100.0);
  parameter_loader.registerVariable<bool>(
      "odom_integrate_on_timeout", &odom_integrate_on_timeout_,
      boost::function<void(bool)>(), "Integrate odom when cmd_vel times out.", false,
      true);
  parameter_loader.registerVariable<bool>("publish_tf", &publish_tf_,
                                          boost::function<void(bool)>(),
                                          "Publish odom to base_link transform.", false,
                                          true);
  parameter_loader.registerVariable<double>(
      "wheel_effort_limit", &wheel_effort_limit_,
      "Absolute wheel effort limit.", MIN_VALID_DT, 100.0);
  parameter_loader.registerVariable<double>(
      "reverse_ccw_vx_scale", &reverse_ccw_vx_scale_,
      "Reverse-CCW compensation vx scale.", MIN_REVERSE_CCW_VX_SCALE,
      MAX_REVERSE_CCW_VX_SCALE);
  parameter_loader.registerVariable<double>(
      "reverse_ccw_wz_gain", &reverse_ccw_wz_gain_,
      "Reverse-CCW compensation wz gain.", MIN_REVERSE_CCW_WZ_GAIN,
      MAX_REVERSE_CCW_WZ_GAIN);
  parameter_loader.registerVariable<double>(
      "reverse_ccw_vy_threshold", &reverse_ccw_vy_threshold_,
      "Reverse-CCW compensation |vy| trigger threshold.",
      MIN_REVERSE_CCW_VY_THRESHOLD, MAX_REVERSE_CCW_VY_THRESHOLD);
  parameter_loader.registerVariable<double>(
      "reverse_ccw_steer_priority_error", &reverse_ccw_steer_priority_error_,
      "Reverse-CCW steering-priority error threshold.",
      MIN_REVERSE_CCW_STEER_PRIORITY_ERROR, MAX_REVERSE_CCW_STEER_PRIORITY_ERROR);
  parameter_loader.registerVariable<double>("geometry/wheel_base", &geometry_.wheel_base,
                                            "Wheel base in meters.", 0.0, 5.0);
  parameter_loader.registerVariable<double>("geometry/track_width", &geometry_.track_width,
                                            "Track width in meters.", 0.0, 5.0);
  parameter_loader.registerVariable<double>("geometry/wheel_radius",
                                            &geometry_.wheel_radius,
                                            "Wheel radius in meters.",
                                            MIN_WHEEL_RADIUS, 1.0);

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

  // Allocate TF resources in init() to avoid allocations inside realtime update().
  const bool RUNTIME_TF_RECONFIGURE = enable_dynamic_reconfigure_;
  if (publish_tf_ || RUNTIME_TF_RECONFIGURE)
  {
    tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster());
  }
  else
  {
    tf_broadcaster_.reset();
  }
  if (command_velocity_mode_ == CommandVelocityMode::GLOBAL || RUNTIME_TF_RECONFIGURE)
  {
    tf_buffer_.reset(new tf2_ros::Buffer(ros::Duration(10.0)));
    tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));
  }
  else
  {
    tf_listener_.reset();
    tf_buffer_.reset();
  }

  if (enable_dynamic_reconfigure_)
  {
    controller_params_reconfigure_.reset(
        new ddynamic_reconfigure::DDynamicReconfigure(nh, true));
    controller_params_reconfigure_->registerEnumVariable<std::string>(
        "command_velocity_mode", &command_velocity_mode_text_,
        "Velocity command interpretation mode.", COMMAND_VELOCITY_MODE_OPTIONS,
        "base_link/global");
    controller_params_reconfigure_->registerVariable<std::string>(
        "command_frame_id", &command_frame_id_,
        boost::function<void(std::string)>(), "Velocity command frame id.",
        std::string(), std::string());
    controller_params_reconfigure_->registerVariable<std::string>(
        "odom_frame_id", &odom_frame_id_, boost::function<void(std::string)>(),
        "Odometry frame id.", std::string(), std::string());
    controller_params_reconfigure_->registerVariable<std::string>(
        "base_frame_id", &base_frame_id_, boost::function<void(std::string)>(),
        "Base frame id.", std::string(), std::string());
    controller_params_reconfigure_->registerVariable<double>(
        "cmd_vel_timeout", &cmd_vel_timeout_, "cmd_vel timeout in seconds.", 0.0, 10.0);
    controller_params_reconfigure_->registerVariable<double>(
        "odom_startup_hold_sec", &odom_startup_hold_sec_,
        "Startup hold window for odom integration.", 0.0, 20.0);
    controller_params_reconfigure_->registerVariable<double>(
        "odom_max_linear_speed", &odom_max_linear_speed_,
        "Maximum acceptable odom linear speed.", MIN_VALID_DT, 50.0);
    controller_params_reconfigure_->registerVariable<double>(
        "odom_max_angular_speed", &odom_max_angular_speed_,
        "Maximum acceptable odom angular speed.", MIN_VALID_DT, 100.0);
    controller_params_reconfigure_->registerVariable<bool>(
        "odom_integrate_on_timeout", &odom_integrate_on_timeout_,
        boost::function<void(bool)>(), "Integrate odom when cmd_vel times out.", false,
        true);
    controller_params_reconfigure_->registerVariable<bool>(
        "publish_tf", &publish_tf_, boost::function<void(bool)>(),
        "Publish odom to base_link transform.", false, true);
    controller_params_reconfigure_->registerVariable<double>(
        "wheel_effort_limit", &wheel_effort_limit_,
        "Absolute wheel effort limit.", MIN_VALID_DT, 100.0);
    controller_params_reconfigure_->registerVariable<double>(
        "reverse_ccw_vx_scale", &reverse_ccw_vx_scale_,
        "Reverse-CCW compensation vx scale.", MIN_REVERSE_CCW_VX_SCALE,
        MAX_REVERSE_CCW_VX_SCALE);
    controller_params_reconfigure_->registerVariable<double>(
        "reverse_ccw_wz_gain", &reverse_ccw_wz_gain_,
        "Reverse-CCW compensation wz gain.", MIN_REVERSE_CCW_WZ_GAIN,
        MAX_REVERSE_CCW_WZ_GAIN);
    controller_params_reconfigure_->registerVariable<double>(
        "reverse_ccw_vy_threshold", &reverse_ccw_vy_threshold_,
        "Reverse-CCW compensation |vy| trigger threshold.",
        MIN_REVERSE_CCW_VY_THRESHOLD, MAX_REVERSE_CCW_VY_THRESHOLD);
    controller_params_reconfigure_->registerVariable<double>(
        "reverse_ccw_steer_priority_error", &reverse_ccw_steer_priority_error_,
        "Reverse-CCW steering-priority error threshold.",
        MIN_REVERSE_CCW_STEER_PRIORITY_ERROR, MAX_REVERSE_CCW_STEER_PRIORITY_ERROR);
    controller_params_reconfigure_->registerVariable<double>(
        "geometry/wheel_base", &geometry_.wheel_base, "Wheel base in meters.", 0.0, 5.0);
    controller_params_reconfigure_->registerVariable<double>(
        "geometry/track_width", &geometry_.track_width, "Track width in meters.", 0.0,
        5.0);
    controller_params_reconfigure_->registerVariable<double>(
        "geometry/wheel_radius", &geometry_.wheel_radius, "Wheel radius in meters.",
        MIN_WHEEL_RADIUS, 1.0);
    controller_params_reconfigure_->setPostUpdateCallback([this]() {
      ValidateAndApplyControllerParams(false);
    });
    controller_params_reconfigure_->publishServicesTopics();
    if (!ValidateAndApplyControllerParams(true))
    {
      return false;
    }
  }
  else
  {
    controller_params_reconfigure_.reset();
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
  return Sophus::SO2d::exp(angle).log();
}

SentryChassisController::OdomState SentryChassisController::IntegrateOdom(
    const OdomState& state, const Kinematics::ChassisTwist& twist, double dt)
{
  if (dt <= 0.0)
  {
    return state;
  }

  const Sophus::SE2d WORLD_FROM_BASE(
      Sophus::SO2d::exp(state.yaw), Eigen::Vector2d(state.x, state.y));
  const Eigen::Vector3d BODY_TANGENT(twist.vx * dt, twist.vy * dt, twist.wz * dt);
  const Sophus::SE2d WORLD_FROM_BASE_NEXT = WORLD_FROM_BASE * Sophus::SE2d::exp(BODY_TANGENT);

  OdomState result;
  result.x = WORLD_FROM_BASE_NEXT.translation().x();
  result.y = WORLD_FROM_BASE_NEXT.translation().y();
  result.yaw = NormalizeAngle(WORLD_FROM_BASE_NEXT.so2().log());
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

  Eigen::Matrix3d source_to_target_rotation;
  if (!BuildRotationFromQuaternion(transform.transform.rotation,
                                   &source_to_target_rotation))
  {
    return false;
  }

  const Eigen::Vector3d LINEAR_INPUT(input.vx, input.vy, 0.0);
  const Eigen::Vector3d ANGULAR_INPUT(0.0, 0.0, input.wz);
  const Eigen::Vector3d LINEAR_OUTPUT = source_to_target_rotation * LINEAR_INPUT;
  const Eigen::Vector3d ANGULAR_OUTPUT = source_to_target_rotation * ANGULAR_INPUT;

  output->vx = LINEAR_OUTPUT.x();
  output->vy = LINEAR_OUTPUT.y();
  output->wz = ANGULAR_OUTPUT.z();
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
  runtime_params_shadow_.geometry = geometry_;
  runtime_params_buffer_.writeFromNonRT(runtime_params_shadow_);
  return true;
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

  if (runtime_params.command_velocity_mode == CommandVelocityMode::BASE_LINK)
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
        runtime_params.base_frame_id, runtime_params.command_frame_id, time,
        ros::Duration(TF_LOOKUP_TIMEOUT_SEC));
  }
  catch (const tf2::TransformException& exception)
  {
    ROS_WARN_THROTTLE(1.0,
                      "Failed to transform cmd_vel from '%s' to '%s': %s",
                      runtime_params.command_frame_id.c_str(),
                      runtime_params.base_frame_id.c_str(),
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
  const RuntimeParams* runtime_params = runtime_params_buffer_.readFromRT();
  if (runtime_params == nullptr)
  {
    ROS_WARN_THROTTLE(1.0, "Runtime parameter snapshot is not initialized yet.");
    return;
  }
  ApplyRuntimeParamsInUpdate(*runtime_params);

  CommandData command = *command_buffer_.readFromRT();
  const bool TIMEOUT =
      IsCommandTimedOut(command.valid, command.stamp, time, runtime_params->cmd_vel_timeout);
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
        ResolveCommandInBaseFrame(command, time, *runtime_params, &command_twist_base);
    if (!COMMAND_RESOLVED)
    {
      command_twist_base = Kinematics::ChassisTwist();
    }
  }

  const Kinematics::ChassisTwist COMPENSATED_COMMAND =
      ApplyCommandCompensation(command_compensation_matrix_, command_twist_base);
  double VX = COMPENSATED_COMMAND.vx;
  const double VY = COMPENSATED_COMMAND.vy;
  double WZ = COMPENSATED_COMMAND.wz;
  if (VX < -ZERO_CMD_EPS && std::fabs(VY) <= runtime_params->reverse_ccw_vy_threshold &&
      WZ > ZERO_CMD_EPS)
  {
    VX *= runtime_params->reverse_ccw_vx_scale;
    WZ *= runtime_params->reverse_ccw_wz_gain;
  }
  if (VX < -ZERO_CMD_EPS && std::fabs(VY) <= runtime_params->reverse_ccw_vy_threshold &&
      std::fabs(WZ) <= ZERO_CMD_EPS)
  {
    VX *= REVERSE_STRAIGHT_VX_BOOST;
  }

  const bool ZERO_CMD_REQUESTED =
      std::fabs(VX) < ZERO_CMD_EPS && std::fabs(VY) < ZERO_CMD_EPS &&
      std::fabs(WZ) < ZERO_CMD_EPS;
  const bool HAS_YAW_COMMAND = std::fabs(WZ) >= ZERO_CMD_EPS;
  const bool HAS_TRANSLATION_COMMAND =
      std::fabs(VX) >= ZERO_CMD_EPS || std::fabs(VY) >= ZERO_CMD_EPS;
  const bool REVERSE_CCW_MODE =
      VX < -ZERO_CMD_EPS && std::fabs(VY) <= runtime_params->reverse_ccw_vy_threshold &&
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
      const double SIGNED_VX = static_cast<double>(direction_signs_.vx[i]) * VX;
      const double SIGNED_VY = static_cast<double>(direction_signs_.vy[i]) * VY;
      const double SIGNED_WZ = static_cast<double>(direction_signs_.wz[i]) * WZ;
      const auto SOLVE_MODULE_TARGET =
          [&](double requested_signed_vx, double requested_signed_vy,
              double requested_signed_wz, double* solved_steer_error,
              double* solved_wheel_target) -> bool
      {
        if (solved_steer_error == nullptr || solved_wheel_target == nullptr)
        {
          return false;
        }
        const double MODULE_VX =
            requested_signed_vx - requested_signed_wz * MODULE_Y;
        const double MODULE_VY =
            requested_signed_vy + requested_signed_wz * MODULE_X;
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

      bool has_module_target =
          SOLVE_MODULE_TARGET(SIGNED_VX, SIGNED_VY, SIGNED_WZ, &steer_error, &wheel_target);
      if (has_module_target && REVERSE_CCW_MODE &&
          std::fabs(steer_error) > runtime_params->reverse_ccw_steer_priority_error)
      {
        // Steering-priority mode: when reverse-CCW command arrives while steering
        // still has large error, temporarily solve wheel target without the reverse
        // translation component to avoid opposite-yaw transients.
        has_module_target =
            SOLVE_MODULE_TARGET(0.0, SIGNED_VY, SIGNED_WZ, &steer_error, &wheel_target);
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

  // Use per-wheel alignment scaling for yaw commands while avoiding full-vehicle
  // hard gating. A single module lagging should not stall all wheel targets.
  const double global_alignment = 1.0;

  for (std::size_t i = 0; i < WHEEL_COUNT; ++i)
  {
    if (wheel_pid_reset_flags[i])
    {
      wheel_pids_[i].reset();
      wheel_joints_[i].setCommand(0.0);
      continue;
    }

    // 轮速按速度误差闭环，目标值由逆运动学解算得到。
    // Wheel loop closes on velocity error using IK-computed targets.
    const double ROLLING_SIGN = static_cast<double>(wheel_rolling_signs_[i]);
    const double SIGNED_WHEEL_VELOCITY = ROLLING_SIGN * wheel_joints_[i].getVelocity();
    const double WHEEL_TARGET = wheel_target_values[i] * alignments[i] * global_alignment;
    const double WHEEL_ERROR = WHEEL_TARGET - SIGNED_WHEEL_VELOCITY;
    const double SIGNED_WHEEL_EFFORT = wheel_pids_[i].computeCommand(WHEEL_ERROR, period);
    const double LIMITED_SIGNED_WHEEL_EFFORT =
        std::max(-runtime_params->wheel_effort_limit,
                 std::min(runtime_params->wheel_effort_limit, SIGNED_WHEEL_EFFORT));
    wheel_joints_[i].setCommand(ROLLING_SIGN * LIMITED_SIGNED_WHEEL_EFFORT);
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
    if (TIMEOUT && !runtime_params->odom_integrate_on_timeout)
    {
      odom_twist = Kinematics::ChassisTwist();
    }
    else
    {
      const double STARTUP_AGE = (time - controller_start_time_).toSec();
      const bool IN_STARTUP_HOLD =
          STARTUP_AGE >= 0.0 && STARTUP_AGE < runtime_params->odom_startup_hold_sec;
      const bool SHOULD_SUPPRESS_STARTUP_DRIFT =
          runtime_params->odom_integrate_on_timeout && TIMEOUT && IN_STARTUP_HOLD;

      if (SHOULD_SUPPRESS_STARTUP_DRIFT)
      {
        odom_twist = Kinematics::ChassisTwist();
        ROS_WARN_THROTTLE(
            1.0,
            "Suppress odometry integration during startup hold window (%.3fs remaining).",
            runtime_params->odom_startup_hold_sec - STARTUP_AGE);
      }
      else if (!IsOdomTwistAcceptable(odom_twist, *runtime_params))
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

  PublishOdometry(time, odom_twist, *runtime_params);
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
