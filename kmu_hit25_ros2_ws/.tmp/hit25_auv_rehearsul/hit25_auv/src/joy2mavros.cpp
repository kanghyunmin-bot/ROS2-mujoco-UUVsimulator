#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <vector>
#include <algorithm>

class JoyToMavros
{
public:
    JoyToMavros() : nh_("~"), first_msg_received_(false), led_pwm_(1500)
    {
        // ROS 토픽 및 서비스 클라이언트 초기화
        joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 10, &JoyToMavros::joyCallback, this);
        rc_override_pub_ = nh_.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override", 10);
        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

        ROS_INFO("Joy to MAVROS node started.");

        // 이전 Joy 메시지 초기화 (버튼 엣지 감지를 위해)
        last_joy_msg_.buttons.resize(13, 0);
        last_joy_msg_.axes.resize(8, 0.0);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber joy_sub_;
    ros::Publisher rc_override_pub_;
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;

    sensor_msgs::Joy last_joy_msg_;
    bool first_msg_received_;
    uint16_t led_pwm_;

    // 조이스틱의 axes 값을 PWM 값(1200-1800)으로 변환하는 함수
    uint16_t scale_axis_to_pwm(float axis_val)
    {
        // axis_val은 -1.0 ~ 1.0 범위
        // 결과 PWM은 1200 ~ 1800 범위
        return static_cast<uint16_t>(1500 + axis_val * 300);
    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
    {
        if (!first_msg_received_)
        {
            last_joy_msg_ = *msg;
            first_msg_received_ = true;
            return;
        }

        // --- 서비스 호출 (버튼이 눌리는 순간 한번만 호출) ---

        // Arming
        if (msg->buttons[8] == 1 && last_joy_msg_.buttons[8] == 0)
        {
            // Disarming 조건이 아닐 때만 Arming
            if (msg->buttons[10] != 1) {
                mavros_msgs::CommandBool arm_cmd;
                arm_cmd.request.value = true;
                if (arming_client_.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                else
                {
                    ROS_ERROR("Failed to arm vehicle");
                }
            }
        }
        
        // Disarming
        if (msg->buttons[8] == 1 && msg->buttons[10] == 1 && 
            (last_joy_msg_.buttons[8] == 0 || last_joy_msg_.buttons[10] == 0))
        {
            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = false;
            if (arming_client_.call(arm_cmd) && arm_cmd.response.success)
            {
                ROS_INFO("Vehicle disarmed");
            }
            else
            {
                ROS_ERROR("Failed to disarm vehicle");
            }
        }

        // 모드 변경 (D-Pad 입력)
        setMode(msg);

        // --- RC Override (매번 새로운 값으로 발행) ---

        // LED 밝기 조절
        handleLedControl(msg);

        // OverrideRCIn 메시지 생성
        mavros_msgs::OverrideRCIn rc_override_msg;
        
        // [수정] 18개 채널 모두를 1500(중립)으로 초기화합니다.
        // C++에서 스택에 할당된 배열은 쓰레기 값으로 시작하므로,
        // 사용하지 않는 채널(9-17)도 명시적으로 초기화해야 합니다.
        for(int i = 0; i < 18; i++) {
            rc_override_msg.channels[i] = 1500;
        }

        // 조종 매핑 (요청사항 기반)
        // channels[0] : Roll (병진)
        // channels[1] : Pitch (전후)
        // channels[2] : Throttle (상하)
        // channels[3] : Yaw (회전)
        // APM/ArduSub 표준 채널 순서: 1:Pitch, 2:Roll, 3:Throttle, 4:Yaw, 5:Forward, 6:Lateral
        // 하지만 요청하신 내용에 따라 채널을 매핑합니다.
        
        // [수정] Yaw 조종 방향을 반대로 하기 위해 -1을 곱합니다.
        rc_override_msg.channels[3] = scale_axis_to_pwm(-msg->axes[0]); // 좌/우 회전 (Yaw)
        
        // 상/하 (Throttle)
        rc_override_msg.channels[2] = scale_axis_to_pwm(msg->axes[1]); 
        
        // [수정] Lateral(병진) 조종 방향을 반대로 하기 위해 -1을 곱합니다.
        rc_override_msg.channels[5] = scale_axis_to_pwm(-msg->axes[3]); // 병진 좌/우 (Lateral)

        // 전진/후진 (Forward)
        rc_override_msg.channels[4] = scale_axis_to_pwm(msg->axes[4]); 
        
        // LED 제어 채널 할당 (RC 채널 9번, index 8 사용)
        rc_override_msg.channels[8] = led_pwm_;


        // 토픽 발행
        rc_override_pub_.publish(rc_override_msg);

        // 현재 메시지를 이전 메시지로 저장
        last_joy_msg_ = *msg;
    }

    void setMode(const sensor_msgs::Joy::ConstPtr& msg)
    {
        std::string new_mode = "";
        
        // Manual 모드 (D-Pad Up)
        if (msg->axes[7] == 1.0 && last_joy_msg_.axes[7] != 1.0) new_mode = "MANUAL";
        
        // Stabilize 모드 (D-Pad Down)
        if (msg->axes[7] == -1.0 && last_joy_msg_.axes[7] != -1.0) new_mode = "STABILIZE";
        
        // Depth Hold (Altitude Hold) 모드 (D-Pad Left)
        if (msg->axes[6] == 1.0 && last_joy_msg_.axes[6] != 1.0) new_mode = "ALT_HOLD";

        // [수정] D-Pad Right 입력 (axes[6] == -1.0)
        if (msg->axes[6] == -1.0 && last_joy_msg_.axes[6] != -1.0)
        {
            // Button 10 (왼쪽 트리거)이 함께 눌려있는지 확인
            if (msg->buttons[10] == 1)
            {
                new_mode = "GUIDED"; // Button 10 + D-Pad Right
            }
            else
            {
                new_mode = "POSHOLD"; // D-Pad Right (단독)
            }
        }

        // 모드 변경 서비스 호출
        if (!new_mode.empty())
        {
            mavros_msgs::SetMode set_mode_srv;
            set_mode_srv.request.custom_mode = new_mode;
            if (set_mode_client_.call(set_mode_srv) && set_mode_srv.response.mode_sent)
            {
                ROS_INFO("Set mode to %s", new_mode.c_str());
            }
            else
            {
                ROS_ERROR("Failed to set mode %s", new_mode.c_str());
            }
        }
    }

    void handleLedControl(const sensor_msgs::Joy::ConstPtr& msg)
    {
        // 버튼 9가 눌리는 순간을 감지
        if (msg->buttons[9] == 1 && last_joy_msg_.buttons[9] == 0)
        {
            // 버튼 10이 눌려있으면 (어둡게)
            if (msg->buttons[10] == 1)
            {
                led_pwm_ -= 100;
                if (led_pwm_ < 1100) led_pwm_ = 1100; // 최소값 제한
                ROS_INFO("LED PWM Down: %d", led_pwm_);
            }
            // 버튼 10이 안 눌려있으면 (밝게)
            else
            {
                led_pwm_ += 100;
                if (led_pwm_ > 1800) led_pwm_ = 1800; // 최대값 제한
                ROS_INFO("LED PWM Up: %d", led_pwm_);
            }
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "joy_to_mavros_node");
    JoyToMavros joy_to_mavros;
    ros::spin();
    return 0;
}