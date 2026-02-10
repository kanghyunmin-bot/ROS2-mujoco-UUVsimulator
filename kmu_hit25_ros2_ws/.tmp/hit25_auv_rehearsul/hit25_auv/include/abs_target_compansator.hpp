#pragma once
/**
 * @file abs_target_compansator.hpp
 * @brief Convert absolute target defined in {auv_map} into {map}-compatible values.
 *
 * What this does
 *  - Input  : absolute target (position + yaw[rad]) expressed in {auv_map}
 *  - Uses   : T_map_auv (4×4 homogeneous TF of {auv_map} <- {map})
 *             which is provided by caller (e.g. auv_target_controller_node)
 *  - Output : absolute target re-expressed in {map}, suitable for MAVROS setpoints
 *
 * Math
 *  - Position:
 *      p_map = (T_map_auv)^(-1) * p_auv_map
 *  - Yaw:
 *      R_tgt_auv  = Rz(yaw_auv_map)
 *      R_map_auv  = Rot part of T_map_auv ({map}→{auv_map})
 *      R_auv_map  = R_map_auv^T
 *      R_tgt_map  = R_auv_map * R_tgt_auv
 *      yaw_map    = atan2(R_tgt_map(1,0), R_tgt_map(0,0))  (wrapped to [-pi,pi])
 *
 * Notes
 *  - Eigen is used for all TF computations.
 *  - This header is math-only. It does not depend on ROS messages.
 *  - Typical usage:
 *      1) T_map_auv = auv_tf::MapToAuvMapMatrix(x0,y0,z0)
 *      2) abs_target_comp::compensateAbsTargetToMap(target_in_auv, T_map_auv)
 */

#include <Eigen/Dense>
#include <cmath>

namespace abs_target_comp {

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector3d;
using Eigen::Vector4d;

/**
 * @brief Absolute target (position + yaw).
 */
struct AbsTarget {
  Vector3d pos;  ///< (x,y,z) [m] in {auv_map}
  double   yaw;  ///< yaw [rad], about +Z in {auv_map}
};

inline double wrapPi(double a) {
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

/**
 * @brief Construct Rz(yaw) rotation matrix (yaw about +Z).
 */
inline Matrix3d Rz(double yaw_rad) {
  const double c = std::cos(yaw_rad);
  const double s = std::sin(yaw_rad);
  Matrix3d R;
  R << c,-s,0,
       s, c,0,
       0, 0,1;
  return R;
}

/**
 * @brief Convert an absolute target from {auv_map} to {map}.
 *
 * @param target_in_auv  position + yaw expressed in {auv_map}
 * @param T_map_auv      homogeneous transform of {auv_map} <- {map}
 *                       (4×4 matrix, e.g. auv_tf::MapToAuvMapMatrix(x0,y0,z0))
 * @return AbsTarget     target expressed in {map}
 */
inline AbsTarget compensateAbsTargetToMap(const AbsTarget& target_in_auv,
                                          const Matrix4d& T_map_auv)
{
  // ---- position ----
  // T_map_auv : {auv_map} <- {map}
  // For points: p_auv = T_map_auv * p_map
  // => p_map = T_auv_map * p_auv with T_auv_map = (T_map_auv)^(-1)
  const Matrix4d T_auv_map = T_map_auv.inverse();

  const Vector4d p_a(
      target_in_auv.pos.x(),
      target_in_auv.pos.y(),
      target_in_auv.pos.z(), 1.0);
  const Vector4d p_m = T_auv_map * p_a;

  // ---- yaw ----
  // If T_map_auv carries a pure yaw about Z (as designed), then:
  //   yaw_map = yaw_auv_map - yaw(T_map_auv)
  const Matrix3d R_map_auv = T_map_auv.block<3,3>(0,0);
  const Matrix3d R_auv_map = R_map_auv.transpose();

  const Matrix3d R_tgt_auv = Rz(target_in_auv.yaw);
  const Matrix3d R_tgt_map = R_auv_map * R_tgt_auv;

  const double yaw_map = wrapPi(std::atan2(R_tgt_map(1,0), R_tgt_map(0,0)));

  AbsTarget out;
  out.pos = p_m.head<3>();
  out.yaw = yaw_map;
  return out;
}

/**
 * @brief Helper: make full 4×4 transform from pos+yaw (Z-rot only).
 *        (필요 없으면 사용 안 해도 됨)
 */
inline Matrix4d makeT_from_pos_yaw(const Vector3d& p, double yaw_rad) {
  Matrix4d T = Matrix4d::Identity();
  T.block<3,3>(0,0) = Rz(yaw_rad);
  T.block<3,1>(0,3) = p;
  return T;
}

} // namespace abs_target_comp
