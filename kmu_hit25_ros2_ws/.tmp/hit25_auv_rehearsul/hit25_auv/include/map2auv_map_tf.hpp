#pragma once

// =============================================================================
// map2auv_map_tf.hpp (rev2)
// -----------------------------------------------------------------------------
// Purpose:
//   Header-only helper for constructing the 4x4 homogeneous transform for the
//   frame relation {auv_map} <- {map}.
//
// Definition:
//   We assume a fixed +90 deg yaw rotation about +Z between {map} and {auv_map}.
//   The origin of {auv_map} is pinned to the initial position of {base_link}
//   expressed in MAVROS' world frame (frame_id: "map").
//
//   In matrix form (T = T_{auv_map <- map}):
//
//       [ 0  -1   0    y0 ]
//   T = [ 1   0   0   -x0 ]
//       [ 0   0   1   -z0 ]
//       [ 0   0   0    1  ]
//
//   where (x0, y0, z0) are the initial position offsets (meters) in {map}.
//   The translation is t = -R * p0_map with R = Rz(+90deg) and p0_map=[x0 y0 z0]^T.
//
// Key variables:
//   - x0, y0, z0 : initial position of {base_link} in {map} (meters)
//   - R          : fixed rotation Rz(+90°) = [[0,-1,0],[1,0,0],[0,0,1]]
//   - t          : translation t = -R * [x0, y0, z0]^T = [ y0, -x0, -z0 ]^T
//
// Notes:
//   - Uses Eigen (header-only). Ensure libeigen3-dev is installed and your
//     build finds the include directory (e.g., /usr/include/eigen3).
//   - Provides both Eigen::Isometry3d and Eigen::Matrix4d builders.
// =============================================================================

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace auv_tf {

/// @brief Construct T_{auv_map <- map} with R=Rz(+90deg) and t=-R*p0_map.
/// @param x0  initial X position in {map} (meters)
/// @param y0  initial Y position in {map} (meters)
/// @param z0  initial Z position in {map} (meters)
/// @return Eigen::Isometry3d representing {auv_map} <- {map}
inline Eigen::Isometry3d MapToAuvMap(double x0, double y0, double z0) {
  // Fixed yaw +90 deg about +Z
  Eigen::Matrix3d R;
  R << 0.0, -1.0, 0.0,
       1.0,  0.0, 0.0,
       0.0,  0.0, 1.0;

  const Eigen::Vector3d p0_map(x0, y0, z0);
  // const Eigen::Vector3d p0_map(y0, -x0, -z0);


  Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
  T.linear() = R;
  T.translation() = -R * p0_map;  // [ y0, -x0, -z0 ]^T
  return T;
}

/// @brief Return the raw 4x4 homogeneous matrix of T_{auv_map <- map}.
inline Eigen::Matrix4d MapToAuvMapMatrix(double x0, double y0, double z0) {
  const Eigen::Isometry3d T = MapToAuvMap(x0, y0, z0);
  Eigen::Matrix4d M = Eigen::Matrix4d::Identity();
  M.topLeftCorner<3,3>() = T.linear();
  M.topRightCorner<3,1>() = T.translation();
  return M;
}

} // namespace auv_tf
