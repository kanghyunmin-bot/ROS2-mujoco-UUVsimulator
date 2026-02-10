#pragma once
/**
 * @file relative_target_compensator.hpp
 * @brief Convert relative commands defined in {auv_base_link} into {base_link}.
 *
 * What this does
 *  - Input  : relative target (Δx, Δy, Δz, Δψ) expressed in {auv_base_link}
 *  - Uses   : T_bl_abl (the homogeneous TF of {base_link}→{auv_base_link})
 *             or yaw_bias (ψ_bias) that defines that TF (yaw-only, zero translation)
 *  - Output : relative target re-expressed in {base_link}
 *
 * Why this is needed
 *  - In RViz/logic, we visualize/command on {auv_base_link} which removes yaw drift.
 *  - MAVROS/flight stack still lives on {base_link}. Before publishing setpoints,
 *    we must convert the relative command back to {base_link}.
 *
 * Math (yaw-only TF between frames)
 *  - Let R_bl_abl = Rz(-ψ_bias) be the rotation mapping a vector in {base_link} to {auv_base_link}.
 *  - Then a vector in {auv_base_link} is mapped to {base_link} by the transpose:
 *        Δp_base = (R_bl_abl)^T · Δp_auv  = Rz(+ψ_bias) · Δp_auv
 *  - Relative yaw is kept as-is (Δψ_base = Δψ_auv). No compensation needed.
 *
 * Notes
 *  - Eigen is used for all TF math.
 *  - The TF matrix itself (for visualization) is defined in baselink2auv_baselink_tf.hpp.
 *  - This header depends only on Eigen; you may pass the rotation from the TF header.
 */

#include <Eigen/Dense>
#include <cmath>

namespace hit25_auv {

struct RelCmd {
  double dx{0.0}, dy{0.0}, dz{0.0};  ///< relative translation
  double dyaw{0.0};                  ///< relative yaw [rad], preserved
};

inline Eigen::Matrix3d Rz(double yaw_rad) {
  const double c = std::cos(yaw_rad);
  const double s = std::sin(yaw_rad);
  Eigen::Matrix3d R;
  R << c, -s, 0,
       s,  c, 0,
       0,  0, 1;
  return R;
}

/**
 * @brief Convert a relative command from {auv_base_link} to {base_link} using a known
 *        {base_link}→{auv_base_link} rotation.
 * @param cmd_abl      Relative command in {auv_base_link}
 * @param R_bl_abl     3×3 rotation that maps {base_link}→{auv_base_link}
 * @return             Relative command re-expressed in {base_link}
 *
 * Usage:
 *   - If you already have R_bl_abl from baselink2auv_baselink_tf.hpp, pass it here.
 *   - Otherwise, call the yaw-bias overload below.
 */
inline RelCmd to_base_from_auv(const RelCmd& cmd_abl,
                               const Eigen::Matrix3d& R_bl_abl)
{
  // Vector rotation: {auv_base_link} → {base_link} uses transpose (inverse).
  const Eigen::Vector3d dpa(cmd_abl.dx, cmd_abl.dy, cmd_abl.dz);
  const Eigen::Vector3d dpb = R_bl_abl.transpose() * dpa;

  RelCmd out;
  out.dx   = dpb.x();
  out.dy   = dpb.y();
  out.dz   = dpb.z();
  out.dyaw = cmd_abl.dyaw; // Relative yaw is unchanged.
  return out;
}

/**
 * @brief Convert using yaw bias directly, assuming TF is yaw-only with zero translation:
 *        R_bl_abl = Rz(-ψ_bias).
 * @param cmd_abl   Relative command in {auv_base_link}
 * @param yaw_bias  ψ_bias [rad]; drift of MAVROS {base_link} relative to drift-free frame
 */
inline RelCmd to_base_from_auv(const RelCmd& cmd_abl, double yaw_bias)
{
  // {base_link}→{auv_base_link} = Rz(-ψ_bias)
  // ⇒ {auv_base_link}→{base_link} = Rz(+ψ_bias)
  const Eigen::Matrix3d R_abl_to_bl = Rz(+yaw_bias);

  const Eigen::Vector3d dpa(cmd_abl.dx, cmd_abl.dy, cmd_abl.dz);
  const Eigen::Vector3d dpb = R_abl_to_bl * dpa;

  RelCmd out;
  out.dx   = dpb.x();
  out.dy   = dpb.y();
  out.dz   = dpb.z();
  out.dyaw = cmd_abl.dyaw; // keep as-is
  return out;
}

} // namespace hit25_auv
