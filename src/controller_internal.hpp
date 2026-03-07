#pragma once

#include "sentry_chassis_controller/sentry_chassis_controller.h"

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <map>
#include <string>
#include <vector>

namespace sentry_chassis_controller
{
// Runtime lower bound for wheel radius to avoid division by zero.
constexpr double MIN_WHEEL_RADIUS = 1e-9;
constexpr double MIN_VALID_DT = 1e-9;
constexpr double ZERO_CMD_EPS = 1e-4;
constexpr double GLOBAL_ALIGNMENT_GATE = 0.20;
constexpr double PI = EIGEN_PI;
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
constexpr bool DEFAULT_ENABLE_ACCELERATION_LIMITS = false;
constexpr double DEFAULT_MAX_LINEAR_ACCELERATION = 3.0;
constexpr double DEFAULT_MAX_ANGULAR_ACCELERATION = 5.0;
constexpr bool DEFAULT_ENABLE_POWER_LIMIT = false;
constexpr bool DEFAULT_ENABLE_POWER_LIMIT_LOGGING = false;
constexpr double DEFAULT_MAX_POWER = 360.0;
constexpr double DEFAULT_POWER_LOSS_K1 = 0.001;
constexpr double DEFAULT_POWER_LOSS_K2 = 0.0001;
constexpr double DEFAULT_MIN_POWER_SCALE = 0.3;
constexpr double MIN_REVERSE_CCW_VX_SCALE = 0.1;
constexpr double MAX_REVERSE_CCW_VX_SCALE = 1.0;
constexpr double MIN_REVERSE_CCW_WZ_GAIN = 1.0;
constexpr double MAX_REVERSE_CCW_WZ_GAIN = 3.0;
constexpr double MIN_REVERSE_CCW_VY_THRESHOLD = 0.0;
constexpr double MAX_REVERSE_CCW_VY_THRESHOLD = 0.5;
constexpr double MIN_REVERSE_CCW_STEER_PRIORITY_ERROR = 0.0;
constexpr double MAX_REVERSE_CCW_STEER_PRIORITY_ERROR = PI;
constexpr double MIN_ACCELERATION_LIMIT = MIN_VALID_DT;
constexpr double MIN_POWER_LIMIT = MIN_VALID_DT;
constexpr double MIN_POWER_SCALE = 0.0;
constexpr double MAX_POWER_SCALE = 1.0;
constexpr double REVERSE_STRAIGHT_VX_BOOST = 1.10;
constexpr double MIN_QUATERNION_NORM = 1e-12;
constexpr double COMMAND_TRANSFORM_CACHE_UPDATE_SEC = 0.01;
constexpr double COMMAND_TRANSFORM_CACHE_MAX_AGE_SEC = 0.05;
constexpr double ODOM_PUBLISH_FLUSH_SEC = 0.01;

// Wheel order suffix must stay aligned with config and kinematics.
static const std::array<std::string, SentryChassisController::WHEEL_COUNT>
    WHEEL_NAME_SUFFIX = {"front_left", "front_right", "rear_left", "rear_right"};

static const std::map<std::string, std::string> COMMAND_VELOCITY_MODE_OPTIONS = {
    {"base_link", "base_link"},
    {"global", "global"}};

template <typename T, std::size_t N>
bool LoadRequiredFixedArrayParam(ros::NodeHandle& nh, const std::string& param_name,
                                 std::array<T, N>* output)
{
  if (output == nullptr)
  {
    return false;
  }

  std::vector<T> raw_values;
  if (!nh.getParam(param_name, raw_values) || raw_values.size() != N)
  {
    ROS_ERROR("Parameter '%s' must exist and contain exactly %zu items.",
              param_name.c_str(), N);
    return false;
  }

  std::copy_n(raw_values.begin(), N, output->begin());
  return true;
}

template <typename T>
bool LoadOptionalVectorParam(ros::NodeHandle& nh, const std::string& param_name,
                             const std::vector<T>& default_values,
                             std::vector<T>* output)
{
  if (output == nullptr)
  {
    return false;
  }

  std::vector<T> raw_values;
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
}

template <typename T, std::size_t N>
bool LoadOptionalFixedArrayParam(ros::NodeHandle& nh, const std::string& param_name,
                                 const std::array<T, N>& default_values,
                                 std::array<T, N>* output)
{
  if (output == nullptr)
  {
    return false;
  }

  std::vector<T> raw_values;
  if (!LoadOptionalVectorParam(
          nh, param_name,
          std::vector<T>(default_values.begin(), default_values.end()), &raw_values))
  {
    return false;
  }
  std::copy_n(raw_values.begin(), N, output->begin());
  return true;
}

}  // namespace sentry_chassis_controller
