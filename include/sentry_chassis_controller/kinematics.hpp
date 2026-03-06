#pragma once

#include <array>
#include <cstddef>

/**
 * @file   kinematics.hpp
 * @brief  舵轮底盘运动学定义
 *         Kinematics definitions for the sentry chassis
 */

namespace sentry_chassis_controller
{

/**
 * @brief  舵轮底盘运动学工具类
 *         Kinematics utility for sentry chassis
 *
 * @details
 * 该类统一管理控制器使用的逆运动学与正运动学：
 * - 逆运动学：把 `base_link` 速度解算为四个模块的舵角误差和轮目标角速度；
 * - 正运动学：根据实际舵角和轮速反馈反解 `vx/vy/wz`。
 * 轮序在整个项目内固定为：
 * `front_left, front_right, rear_left, rear_right`。
 */
class Kinematics
{
 public:
  static constexpr std::size_t WHEEL_COUNT = 4;

  /**
   * @brief  底盘几何参数（使用整车尺寸）
   *         Chassis geometry parameters (full dimensions)
   */
  struct Geometry
  {
    double wheel_base = 0.50;     ///< 前后轴距（米） Front-rear wheelbase in meters
    double track_width = 0.40;    ///< 左右轮距（米） Left-right track width in meters
    double wheel_radius = 0.076;  ///< 轮半径（米） Wheel radius in meters
  };

  /**
   * @brief  底盘速度状态结构（base_link）
   *         Chassis twist state in base_link
   */
  struct ChassisTwist
  {
    double vx = 0.0;  ///< x 向线速度 x-axis linear velocity
    double vy = 0.0;  ///< y 向线速度 y-axis linear velocity
    double wz = 0.0;  ///< z 向角速度 z-axis angular velocity
  };

  /**
   * @brief  四轮反馈状态结构
   *         Four-wheel feedback state
   */
  struct WheelFeedback
  {
    std::array<double, WHEEL_COUNT> steer_position{
        {0.0, 0.0, 0.0, 0.0}};  ///< 舵角反馈 Steering angle feedback
    std::array<double, WHEEL_COUNT> wheel_angular_velocity{
        {0.0, 0.0, 0.0, 0.0}};  ///< 轮速反馈 Wheel angular velocity feedback
  };

  /**
   * @brief  三轴轮向符号矩阵
   *         Per-axis wheel direction sign matrix
   */
  struct DirectionSigns
  {
    std::array<int, WHEEL_COUNT> vx{{1, 1, 1, 1}};  ///< x 轴符号 Signs for vx term
    std::array<int, WHEEL_COUNT> vy{{1, 1, 1, 1}};  ///< y 轴符号 Signs for vy term
    std::array<int, WHEEL_COUNT> wz{{1, 1, 1, 1}};  ///< z 轴角速度符号 Signs for wz term
  };

  /**
   * @brief  单个模块的逆运动学目标
   *         Inverse-kinematics target for a single wheel module
   */
  struct WheelTarget
  {
    double steer_error = 0.0;             ///< 舵角误差 Steering error
    double wheel_angular_velocity = 0.0;  ///< 目标轮角速度 Target wheel angular velocity
    bool active = false;                  ///< 是否存在有效目标 Whether the target is active
  };

  /**
   * @brief  四模块逆运动学求解结果
   *         Inverse-kinematics solution for all wheel modules
   */
  struct WheelTargetSolution
  {
    std::array<WheelTarget, WHEEL_COUNT> modules{};
  };

  Kinematics();
  explicit Kinematics(const Geometry& geometry);

  void SetGeometry(const Geometry& geometry);
  void SetDirectionSigns(const DirectionSigns& direction_signs);

  /**
   * @brief  计算四个舵轮模块的控制目标
   *         Computes steer errors and wheel speeds for the four steerable modules
   * @param  twist 经过补偿后的底盘速度命令 Compensated chassis twist command
   * @param  steer_zero_offsets 舵向零位偏置 Steering zero offsets
   * @param  steer_positions 当前舵角位置 Current steering positions
   * @return 四模块求解结果，未激活模块保持 `active=false`
   *         Per-module targets; inactive modules keep `active=false`
   */
  WheelTargetSolution ComputeWheelTargets(
      const ChassisTwist& twist,
      const std::array<double, WHEEL_COUNT>& steer_zero_offsets,
      const std::array<double, WHEEL_COUNT>& steer_positions) const;

  /**
   * @brief  根据舵角和轮速反馈求解底盘速度
   *         Solves chassis twist from steering and wheel feedback
   */
  bool ComputeChassisTwistFromWheelFeedback(
      const WheelFeedback& feedback,
      const std::array<double, WHEEL_COUNT>& steer_zero_offsets,
      const std::array<int, WHEEL_COUNT>& wheel_rolling_signs, ChassisTwist* twist) const;

 private:
  Geometry geometry_{};               ///< 几何参数缓存 Cached geometry
  DirectionSigns direction_signs_{};  ///< 轮向符号矩阵缓存 Cached direction sign matrix
};

}  // namespace sentry_chassis_controller
