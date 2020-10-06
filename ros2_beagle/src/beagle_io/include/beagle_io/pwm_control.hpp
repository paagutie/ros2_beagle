#ifndef PWM_CONTROL_HPP
#define PWM_CONTROL_HPP

/**
 * pwm_control.hpp
 *
 * @author     Pablo Gutiérrez
 * @date       08/07/2020
 */

// ROS 2 Headers
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "beagle_io/servo/servo.hpp"
#include "beagle_io/util_tools.hpp"

#include <sensor_msgs/msg/joy.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;


namespace beagle_io {


class PWM_CONTROL: public rclcpp::Node
{
public:

    uint8_t ready = 0;
    uint8_t error = 0;

    PWM_CONTROL();
    ~PWM_CONTROL();


private:

    int ch_all = 0;
    int pwm_all = 1500;
    esc_struct_t *esc_data;
    joystick_t jstk;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy;

    //void timerCallback();
    void send_pwm(int &channel, int &pwm_us);
    void call_joy(const sensor_msgs::msg::Joy::SharedPtr joy);
    void clamp(int &value, const int MIN, const int MAX);

};

} // namespace
#endif //PWM_CONTROL_HPP
