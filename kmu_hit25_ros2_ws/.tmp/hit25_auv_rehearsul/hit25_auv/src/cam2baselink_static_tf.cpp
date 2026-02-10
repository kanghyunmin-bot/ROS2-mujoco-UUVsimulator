#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <cmath>

int main(int argc, char** argv) {
  ros::init(argc, argv, "cam2baselink_static_tf");
  ros::NodeHandle nh("~");

  // 프레임 이름을 파라미터로 받아서 환경에 맞게 바꿔 쓸 수 있게 함
  std::string parent_frame, child_frame;
  nh.param<std::string>("parent_frame", parent_frame, "camera1"); // ← camera1(optical)
  nh.param<std::string>("child_frame",  child_frame,  "base_link");

  static tf2_ros::StaticTransformBroadcaster br;
  geometry_msgs::TransformStamped t;

  t.header.stamp = ros::Time::now();
  t.header.frame_id = parent_frame;   // camera1(optical)
  t.child_frame_id  = child_frame;    // base_link

  // translation [m] — 제공한 값 그대로
  t.transform.translation.x = -0.0375;
  t.transform.translation.y =  0.0225;
  t.transform.translation.z = -0.07;

  // rotation — R = [[0,0,1],[0,1,0],[-1,0,0]] = +90° about Y
  tf2::Quaternion q;
  q.setRPY(0.0, M_PI_2, 0.0);  // roll, pitch, yaw [rad]
  t.transform.rotation.x = 0.5;
  t.transform.rotation.y = -0.5;
  t.transform.rotation.z = 0.5;
  t.transform.rotation.w = 0.5;

  br.sendTransform(t);
  ros::spin();
  return 0;
}
