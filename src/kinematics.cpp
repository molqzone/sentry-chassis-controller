#include "sentry_chassis_controller/kinematics.hpp"

namespace sentry_chassis_controller {

Kinematics::Kinematics() = default;

Kinematics::Kinematics(const Geometry& geometry) : geometry_(geometry) {}

void Kinematics::setGeometry(const Geometry& geometry) {
  geometry_ = geometry;
}

Kinematics::WheelTargets Kinematics::computeWheelAngularVelocity(double vx, double vy, double wz) const {
  WheelTargets targets;
  const double half_sum = (geometry_.wheel_base + geometry_.track_width) * 0.5;
  const double radius = geometry_.wheel_radius > 1e-9 ? geometry_.wheel_radius : 1e-9;

  targets.front_left = (vx - vy - half_sum * wz) / radius;
  targets.front_right = (vx + vy + half_sum * wz) / radius;
  targets.rear_left = (vx + vy - half_sum * wz) / radius;
  targets.rear_right = (vx - vy + half_sum * wz) / radius;

  return targets;
}

}  // namespace sentry_chassis_controller
