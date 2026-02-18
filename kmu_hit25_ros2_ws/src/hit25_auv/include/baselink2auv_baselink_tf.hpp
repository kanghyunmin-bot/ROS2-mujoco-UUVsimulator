#pragma once
/*
 * baselink2auv_baselink_tf.hpp
 *
 * What:  Build a homogeneous transform T_base_link^auv_base_link that only compensates yaw drift.
 * Why:   {auv_base_link} is defined as a child frame of {base_link} with identical position
 *        but with yaw corrected by -psi (psi = yaw_bias).
 *
 * Frames:
 *   parent  : base_link
 *   child   : auv_base_link
 *
 * Key variables:
 *   psi        : yaw_bias (rad), positive in +Z (right-hand rule).
 *   Rz(-psi)   : 3x3 rotation about Z by -psi.
 *   T          : 4x4 homogeneous transform [ R | t ; 0 0 0 1 ] with t = [0,0,0]^T.
 *
 * Notes:
 *   cos(-psi) =  cos(psi)
 *   sin(-psi) = -sin(psi)
 *
 * Dependencies:
 *   - Eigen (matrix build)
 *   - (optional) geometry_msgs / tf2 for broadcasting helper
 */

#include <Eigen/Dense>

// (optional helpers for ROS TF broadcasting)
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

namespace auv_tf
{

// 4x4 homogeneous transform from {base_link} to {auv_base_link} (yaw-only correction).
inline Eigen::Matrix4d makeBaselinkToAuvBaselinkTF(double psi /* yaw_bias [rad] */)
{
    const double c = std::cos(psi);
    const double s = std::sin(psi);

    // Rz(-psi)
    Eigen::Matrix3d R;
    R <<  c,  s, 0,
         -s,  c, 0,
          0,  0, 1;

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.topLeftCorner<3,3>() = R;                // rotation
    T.topRightCorner<3,1>() = Eigen::Vector3d::Zero(); // translation (same position)

    return T;
}

// Build geometry_msgs::TransformStamped for TF broadcaster (translation = 0, rotation = -psi about Z).
inline geometry_msgs::TransformStamped toTransformStamped(double psi,
                                                          const std::string& parent_frame = "base_link",
                                                          const std::string& child_frame  = "auv_base_link",
                                                          const ros::Time& stamp = ros::Time::now())
{
    geometry_msgs::TransformStamped ts;
    ts.header.stamp = stamp;
    ts.header.frame_id = parent_frame;
    ts.child_frame_id  = child_frame;

    // position is identical -> zero translation
    ts.transform.translation.x = 0.0;
    ts.transform.translation.y = 0.0;
    ts.transform.translation.z = 0.0;

    // rotation about +Z by -psi
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, -psi);
    ts.transform.rotation.x = q.x();
    ts.transform.rotation.y = q.y();
    ts.transform.rotation.z = q.z();
    ts.transform.rotation.w = q.w();

    return ts;
}

} // namespace auv_tf
