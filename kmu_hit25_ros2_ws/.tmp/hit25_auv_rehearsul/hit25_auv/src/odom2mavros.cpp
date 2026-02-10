#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

// TF2/Geometry_msgs 변환을 위한 헤더
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class OdomToVisionPose {
public:
  OdomToVisionPose(ros::NodeHandle& nh) {
    pub_ = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
    sub_ = nh.subscribe("/rovio/odometry", 10, &OdomToVisionPose::cb, this);

    // -----------------------------------------------------------------
    // 1. 변환(Rotation) 정의: (ROVIO Body -> ROS FLU Body)
    // -----------------------------------------------------------------
    // ROVIO (X=Left, Y=Down, Z=Front) -> ROS (X=Front, Y=Left, Z=Up)
    // 매핑: X_ros = Z_rovio, Y_ros = X_rovio, Z_ros = -Y_rovio
    //
    // 해당 변환을 나타내는 회전 행렬:
    // [ 0  0  1 ]
    // [ 1  0  0 ]
    // [ 0 -1  0 ]
    tf2::Matrix3x3 R_rovio_to_base(
      0, 0, 1,  // 첫 번째 행 (ROS X = ROVIO Z)
      1, 0, 0,  // 두 번째 행 (ROS Y = ROVIO X)
      0, -1, 0  // 세 번째 행 (ROS Z = -ROVIO Y)
    );

    // 행렬을 쿼터니언으로 변환하여 저장
    R_rovio_to_base.getRotation(q_rovio_to_base_);
    q_rovio_to_base_.normalize(); // 정규화

    ROS_INFO("Republishing and TRANSFORMING /rovio/odometry to /mavros/vision_pose/pose.");
  }

private:
  void cb(const nav_msgs::Odometry::ConstPtr& msg_in) {
    geometry_msgs::PoseStamped msg_out;
    
    // 헤더는 그대로 복사 (타임스탬프와 frame_id 포함)
    msg_out.header = msg_in->header;

    // -----------------------------------------------------------------
    // 2. 위치(Position) 변환
    // -----------------------------------------------------------------
    // *중요 가정*: ROVIO의 월드 프레임(msg_in->header.frame_id)이
    // 이미 ROS 표준인 ENU라고 가정합니다.
    // 이 가정이 맞다면, 위치 데이터는 변환할 필요가 없습니다.
    msg_out.pose.position = msg_in->pose.pose.position;

    // -----------------------------------------------------------------
    // 3. 방향(Orientation) 변환
    // -----------------------------------------------------------------
    // 입력 쿼터니언 (q_world_to_rovio)
    tf2::Quaternion q_in;
    tf2::fromMsg(msg_in->pose.pose.orientation, q_in);
    
    // 변환 적용: q_out = q_in * q_transform
    // (q_world_to_base = q_world_to_rovio * q_rovio_to_base)
    tf2::Quaternion q_out = q_in * q_rovio_to_base_;
    q_out.normalize(); // 정규화

    // 변환된 쿼터니언을 메시지에 다시 저장
    msg_out.pose.orientation = tf2::toMsg(q_out);

    // -----------------------------------------------------------------
    
    pub_.publish(msg_out);
  }

  ros::Subscriber sub_;
  ros::Publisher  pub_;
  tf2::Quaternion q_rovio_to_base_; // 변환용 쿼터니언
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "odom_to_vision_pose");
  ros::NodeHandle nh;
  OdomToVisionPose node(nh);
  ros::spin();
}