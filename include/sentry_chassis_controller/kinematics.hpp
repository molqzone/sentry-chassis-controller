#pragma once

namespace sentry_chassis_controller {

class Kinematics {
 public:
  struct Geometry {
    double wheel_base = 0.50;
    double track_width = 0.40;
    double wheel_radius = 0.076;
  };

  struct WheelTargets {
    double front_left = 0.0;
    double front_right = 0.0;
    double rear_left = 0.0;
    double rear_right = 0.0;
  };

  Kinematics();

  explicit Kinematics(const Geometry& geometry);

  void setGeometry(const Geometry& geometry);

  WheelTargets computeWheelAngularVelocity(double vx, double vy, double wz) const;

 private:
  Geometry geometry_;
};

}  // namespace sentry_chassis_controller
