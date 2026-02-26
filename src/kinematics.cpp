#include "sentry_chassis_controller/kinematics.hpp"

#include <Eigen/Dense>

#include <array>
#include <cmath>
#include <utility>

namespace sentry_chassis_controller
{
namespace
{
constexpr double kMinWheelRadius = 1e-9;
constexpr double kRankTolerance = 1e-9;
}  // namespace

Kinematics::Kinematics() = default;

Kinematics::Kinematics(const Geometry& geometry) : geometry_(geometry) {}

void Kinematics::SetGeometry(const Geometry& geometry) { geometry_ = geometry; }

void Kinematics::SetDirectionSigns(const DirectionSigns& direction_signs)
{
  direction_signs_ = direction_signs;
}

Kinematics::WheelTargets Kinematics::ComputeWheelAngularVelocity(double vx, double vy,
                                                                 double wz) const
{
  // Mecanum inverse kinematics in base_link, represented as:
  // wheel_angular_velocity = (J * [vx, vy, wz]^T) / wheel_radius.
  // Wheel order is fixed: front_left, front_right, rear_left, rear_right.
  WheelTargets targets;
  const double HALF_SUM = (geometry_.wheel_base + geometry_.track_width) * 0.5;
  const double RADIUS = geometry_.wheel_radius > 1e-9 ? geometry_.wheel_radius : 1e-9;
  const Eigen::Matrix<double, 4, 3> jacobian =
      (Eigen::Matrix<double, 4, 3>() << static_cast<double>(direction_signs_.vx[0]),
       -static_cast<double>(direction_signs_.vy[0]),
       -static_cast<double>(direction_signs_.wz[0]) * HALF_SUM,
       static_cast<double>(direction_signs_.vx[1]),
       static_cast<double>(direction_signs_.vy[1]),
       static_cast<double>(direction_signs_.wz[1]) * HALF_SUM,
       static_cast<double>(direction_signs_.vx[2]),
       static_cast<double>(direction_signs_.vy[2]),
       -static_cast<double>(direction_signs_.wz[2]) * HALF_SUM,
       static_cast<double>(direction_signs_.vx[3]),
       -static_cast<double>(direction_signs_.vy[3]),
       static_cast<double>(direction_signs_.wz[3]) * HALF_SUM)
          .finished();
  const Eigen::Vector3d chassis_velocity(vx, vy, wz);
  const Eigen::Matrix<double, 4, 1> wheel_velocity = (jacobian * chassis_velocity) / RADIUS;

  targets.front_left = wheel_velocity(0);
  targets.front_right = wheel_velocity(1);
  targets.rear_left = wheel_velocity(2);
  targets.rear_right = wheel_velocity(3);

  return targets;
}

bool Kinematics::ComputeChassisTwistFromWheelFeedback(
    const WheelFeedback& feedback, const std::array<double, 4>& steer_zero_offsets,
    const std::array<int, 4>& wheel_rolling_signs, ChassisTwist* twist) const
{
  if (twist == nullptr)
  {
    return false;
  }

  const double wheel_radius =
      geometry_.wheel_radius > kMinWheelRadius ? geometry_.wheel_radius : kMinWheelRadius;
  const double half_wheel_base = geometry_.wheel_base * 0.5;
  const double half_track_width = geometry_.track_width * 0.5;

  const std::array<std::pair<double, double>, 4> module_positions = {{
      {half_wheel_base, half_track_width},
      {half_wheel_base, -half_track_width},
      {-half_wheel_base, half_track_width},
      {-half_wheel_base, -half_track_width},
  }};

  Eigen::Matrix<double, 4, 3> a = Eigen::Matrix<double, 4, 3>::Zero();
  Eigen::Matrix<double, 4, 1> b = Eigen::Matrix<double, 4, 1>::Zero();

  for (std::size_t index = 0; index < 4; ++index)
  {
    if (wheel_rolling_signs[index] != -1 && wheel_rolling_signs[index] != 1)
    {
      return false;
    }

    const double steer_angle = feedback.steer_position[index] - steer_zero_offsets[index];
    const double direction_x = std::cos(steer_angle);
    const double direction_y = std::sin(steer_angle);
    const double module_x = module_positions[index].first;
    const double module_y = module_positions[index].second;
    const double wheel_linear_speed = static_cast<double>(wheel_rolling_signs[index]) *
                                      wheel_radius *
                                      feedback.wheel_angular_velocity[index];

    a(index, 0) = direction_x;
    a(index, 1) = direction_y;
    a(index, 2) = -direction_x * module_y + direction_y * module_x;
    b(index) = wheel_linear_speed;
  }

  Eigen::ColPivHouseholderQR<Eigen::Matrix<double, 4, 3>> qr(a);
  qr.setThreshold(kRankTolerance);
  if (qr.rank() < 3)
  {
    return false;
  }
  const Eigen::Vector3d solved = qr.solve(b);

  twist->vx = solved.x();
  twist->vy = solved.y();
  twist->wz = solved.z();
  return true;
}

}  // namespace sentry_chassis_controller
