#include "sentry_chassis_controller/kinematics.hpp"

#include <array>
#include <cmath>
#include <utility>

namespace sentry_chassis_controller
{
namespace
{
constexpr double kMinWheelRadius = 1e-9;
constexpr double kPivotEpsilon = 1e-9;

bool SolveLinearSystem3x3(const std::array<std::array<double, 3>, 3>& matrix,
                          const std::array<double, 3>& vector,
                          std::array<double, 3>* solution)
{
  std::array<std::array<double, 4>, 3> augmented{};
  for (std::size_t row = 0; row < 3; ++row)
  {
    augmented[row][0] = matrix[row][0];
    augmented[row][1] = matrix[row][1];
    augmented[row][2] = matrix[row][2];
    augmented[row][3] = vector[row];
  }

  for (std::size_t pivot_column = 0; pivot_column < 3; ++pivot_column)
  {
    std::size_t pivot_row = pivot_column;
    double pivot_abs = std::fabs(augmented[pivot_row][pivot_column]);
    for (std::size_t row = pivot_column + 1; row < 3; ++row)
    {
      const double candidate = std::fabs(augmented[row][pivot_column]);
      if (candidate > pivot_abs)
      {
        pivot_abs = candidate;
        pivot_row = row;
      }
    }

    if (pivot_abs < kPivotEpsilon)
    {
      return false;
    }

    if (pivot_row != pivot_column)
    {
      std::swap(augmented[pivot_row], augmented[pivot_column]);
    }

    const double pivot_value = augmented[pivot_column][pivot_column];
    for (std::size_t column = pivot_column; column < 4; ++column)
    {
      augmented[pivot_column][column] /= pivot_value;
    }

    for (std::size_t row = 0; row < 3; ++row)
    {
      if (row == pivot_column)
      {
        continue;
      }
      const double factor = augmented[row][pivot_column];
      for (std::size_t column = pivot_column; column < 4; ++column)
      {
        augmented[row][column] -= factor * augmented[pivot_column][column];
      }
    }
  }

  (*solution)[0] = augmented[0][3];
  (*solution)[1] = augmented[1][3];
  (*solution)[2] = augmented[2][3];
  return true;
}
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
  // Mecanum inverse kinematics in base_link.
  // Wheel direction signs are applied per axis to align model-side joint direction.
  // Wheel order is fixed: front_left, front_right, rear_left, rear_right.
  // fl = (sx_fl*vx - sy_fl*vy - sz_fl*((wheel_base + track_width) / 2)*wz) / wheel_radius
  // fr = (sx_fr*vx + sy_fr*vy + sz_fr*((wheel_base + track_width) / 2)*wz) / wheel_radius
  // rl = (sx_rl*vx + sy_rl*vy - sz_rl*((wheel_base + track_width) / 2)*wz) / wheel_radius
  // rr = (sx_rr*vx - sy_rr*vy + sz_rr*((wheel_base + track_width) / 2)*wz) / wheel_radius
  WheelTargets targets;
  const double HALF_SUM = (geometry_.wheel_base + geometry_.track_width) * 0.5;
  const double RADIUS = geometry_.wheel_radius > 1e-9 ? geometry_.wheel_radius : 1e-9;

  targets.front_left = (static_cast<double>(direction_signs_.vx[0]) * vx -
                        static_cast<double>(direction_signs_.vy[0]) * vy -
                        static_cast<double>(direction_signs_.wz[0]) * HALF_SUM * wz) /
                       RADIUS;
  targets.front_right = (static_cast<double>(direction_signs_.vx[1]) * vx +
                         static_cast<double>(direction_signs_.vy[1]) * vy +
                         static_cast<double>(direction_signs_.wz[1]) * HALF_SUM * wz) /
                        RADIUS;
  targets.rear_left = (static_cast<double>(direction_signs_.vx[2]) * vx +
                       static_cast<double>(direction_signs_.vy[2]) * vy -
                       static_cast<double>(direction_signs_.wz[2]) * HALF_SUM * wz) /
                      RADIUS;
  targets.rear_right = (static_cast<double>(direction_signs_.vx[3]) * vx -
                        static_cast<double>(direction_signs_.vy[3]) * vy +
                        static_cast<double>(direction_signs_.wz[3]) * HALF_SUM * wz) /
                       RADIUS;

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

  std::array<std::array<double, 3>, 3> at_a{
      {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}};
  std::array<double, 3> at_b{{0.0, 0.0, 0.0}};

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

    const std::array<double, 3> row = {
        direction_x,
        direction_y,
        -direction_x * module_y + direction_y * module_x,
    };

    for (std::size_t row_index = 0; row_index < 3; ++row_index)
    {
      at_b[row_index] += row[row_index] * wheel_linear_speed;
      for (std::size_t column_index = 0; column_index < 3; ++column_index)
      {
        at_a[row_index][column_index] += row[row_index] * row[column_index];
      }
    }
  }

  std::array<double, 3> solved{{0.0, 0.0, 0.0}};
  if (!SolveLinearSystem3x3(at_a, at_b, &solved))
  {
    return false;
  }

  twist->vx = solved[0];
  twist->vy = solved[1];
  twist->wz = solved[2];
  return true;
}

}  // namespace sentry_chassis_controller
