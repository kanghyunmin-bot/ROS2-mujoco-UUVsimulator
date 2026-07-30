#ifndef DVL_A50_HPP
#define DVL_A50_HPP

/**
 * dvl-sensor.hpp
 *
 * @author     Pablo Gutiérrez
 * @date       24/11/2021
 */

// ROS 2 Headers
#include <chrono>
#include <memory>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "dvl_a50/tcpsocket.hpp"

#include <string>
#include "auv_dvl_a50_msg/msg/dvl.hpp"
#include "auv_dvl_a50_msg/msg/dvl_beam.hpp"
#include "auv_dvl_a50_msg/msg/dvldr.hpp"
#include "auv_dvl_a50_msg/msg/config_command.hpp"
#include "auv_dvl_a50_msg/msg/command_response.hpp"
#include "auv_dvl_a50_msg/msg/config_status.hpp"

#include <nlohmann/json.hpp>
#include <iomanip>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::string;
using nlohmann::json;


namespace dvl_sensor {

enum DVL_Parameters {
    speed_of_sound,
    acoustic_enabled,
    dark_mode_enabled,
    mounting_rotation_offset,
    range_mode,
    invalid_param
};

class DVL_A50: public rclcpp::Node
{
public:

    uint8_t ready = 0;
    uint8_t error = 0;

    DVL_A50();
    ~DVL_A50();

private:
    int fault = 1; 
    string delimiter = ",";
    std::string ip_address;
    std::string velocity_frame_id;
    std::string position_frame_id;
    bool configure_acoustic_on_startup;
    bool startup_acoustic_enabled;
    bool request_config_on_startup;
    std::string receive_buffer;
    TCPSocket *tcpSocket;
    json json_data;

    DVL_Parameters resolveParameter(std::string param);
    
    std::chrono::steady_clock::time_point first_time;
    std::chrono::steady_clock::time_point first_time_error;
    
    rclcpp::TimerBase::SharedPtr timer_receive;
    rclcpp::TimerBase::SharedPtr timer_send;
    rclcpp::Publisher<auv_dvl_a50_msg::msg::DVL>::SharedPtr dvl_pub_report;
    rclcpp::Publisher<auv_dvl_a50_msg::msg::DVLDR>::SharedPtr dvl_pub_pos;
    rclcpp::Publisher<auv_dvl_a50_msg::msg::CommandResponse>::SharedPtr dvl_pub_command_response;
    rclcpp::Publisher<auv_dvl_a50_msg::msg::ConfigStatus>::SharedPtr dvl_pub_config_status;
    rclcpp::Subscription<auv_dvl_a50_msg::msg::ConfigCommand>::SharedPtr dvl_sub_config_command;


    void handle_receive();
    //Publish velocity and transducer report
    void publish_vel_trans_report();
    void publish_dead_reckoning_report();
    void publish_config_status();
    void publish_command_response();
    void publish_transport_error(const std::string &command);

    void command_subscriber(const auv_dvl_a50_msg::msg::ConfigCommand::SharedPtr msg);
    void set_json_parameter(const std::string name, const std::string value);
    bool send_parameter_to_sensor(const json &message);

};

} // namespace
#endif //DVL_A50_HPP
