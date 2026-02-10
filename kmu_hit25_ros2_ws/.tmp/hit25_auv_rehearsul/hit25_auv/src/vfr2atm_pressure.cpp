#include "ros/ros.h"
#include "mavros_msgs/VFR_HUD.h"       // 구독할 메시지 (고도)
#include "sensor_msgs/FluidPressure.h" // 발행할 메시지 (압력)

// 퍼블리셔를 전역 변수로 선언
ros::Publisher pressure_pub;

/**
 * /mavros/vfr_hud 토픽을 구독했을 때 호출되는 콜백 함수
 */
void vfrHudCallback(const mavros_msgs::VFR_HUD::ConstPtr& vfr_msg)
{
    // 1. VFR_HUD 메시지에서 '고도(altitude)' 값을 가져옴 (단위: 미터)
    double altitude_value = vfr_msg->altitude;

    // 2. sensor_msgs::FluidPressure 메시지 생성
    sensor_msgs::FluidPressure pressure_msg;

    // 3. 헤더 채우기 (원본 메시지의 타임스탬프 사용)
    pressure_msg.header.stamp = vfr_msg->header.stamp; 
    pressure_msg.header.frame_id = "base_link"; // 적절한 프레임 ID로 변경

    // 4. [요청 사항] 고도 값을 압력 필드에 그대로 복사
    //    (경고: 단위가 맞지 않습니다. '미터' 값을 '파스칼' 필드에 넣는 중입니다.)
    pressure_msg.fluid_pressure = altitude_value;

    // 5. 분산 값 채우기
    pressure_msg.variance = 0.0;

    // 6. /mavros/imu/atm_pressure 토픽으로 발행
    pressure_pub.publish(pressure_msg);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "vfr_passthrough_converter");
    ros::NodeHandle nh;

    // 퍼블리셔 초기화
    // /mavros/imu/atm_pressure 토픽에 sensor_msgs::FluidPressure 메시지 발행
    pressure_pub = nh.advertise<sensor_msgs::FluidPressure>("/mavros/imu/atm_pressure", 10);

    // 서브스크라이버 초기화
    // /mavros/vfr_hud 토픽을 구독하고, 메시지가 오면 vfrHudCallback 함수를 실행
    ros::Subscriber sub = nh.subscribe("/mavros/vfr_hud", 10, vfrHudCallback);

    ROS_INFO("VFR_HUD 'altitude' to Pressure 'fluid_pressure' Passthrough [RUNNING]");
    ROS_INFO("Subscribing to /mavros/vfr_hud...");
    ROS_INFO("Publishing to /mavros/imu/atm_pressure...");

    // 콜백 대기
    ros::spin();

    return 0;
}