/**
 * pwm_control.cpp
 *
 * @author     Pablo Gutiérrez
 * @date       08/07/2020
 * Comments:
 * A function (clamp) was added to limit the value of the pwm signal sent to the motors - 02/09/2020 - Pablo Gutiérrez
 */

#include "beagle_io/pwm_control.hpp"

namespace beagle_io {


PWM_CONTROL::PWM_CONTROL():
Node("beagle_io_servos")
{
    sub_joy = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&PWM_CONTROL::call_joy, this, _1));

    esc_data = new esc_struct_t[4];
    for(int i=0; i<4; i++)
    {
        esc_data[i]._channel = i+1;
        esc_data[i]._width_us = 1500;
    }

    rc_servo_init();
    if(rc_servo_init())
        exit (EXIT_FAILURE);
    if(rc_servo_set_esc_range(RC_ESC_DEFAULT_MIN_US, RC_ESC_DEFAULT_MAX_US))
        exit (EXIT_FAILURE);

    this->send_pwm(ch_all, pwm_all);
     
}

PWM_CONTROL::~PWM_CONTROL() 
{
    delete [] esc_data;
    this->send_pwm(ch_all, pwm_all);
    rc_servo_cleanup();
}

void PWM_CONTROL::call_joy(const sensor_msgs::msg::Joy::SharedPtr joy)
{
    jstk.axes[0] = -joy->axes[0];  
    jstk.axes[1] = joy->axes[1];
    jstk.axes[2] = -joy->axes[2];  
    jstk.axes[3] = joy->axes[3];

    for(int i=0; i<4; i++)
    {
        esc_data[i]._channel = i+1;
        esc_data[i]._width_us = (uint16_t)scale_joy_input(jstk.axes[i]);
        this->clamp(esc_data[i]._width_us, RC_ESC_DEFAULT_MIN_US, RC_ESC_DEFAULT_MAX_US);
        this->send_pwm(esc_data[i]._channel, esc_data[i]._width_us);
    }
}  

    
void PWM_CONTROL::send_pwm(int &channel, int &pwm_us)
{
    if(pwm_us<RC_ESC_DEFAULT_MIN_US || pwm_us>RC_ESC_DEFAULT_MAX_US){
        RCLCPP_ERROR(this->get_logger(), "Width in microseconds must be >1100 and <1900");
        exit (EXIT_FAILURE);
    }
    else
    {
        rc_servo_send_pulse_us(channel,pwm_us);
    }
}

void PWM_CONTROL::clamp(int &value, const int MIN, const int MAX)
{
  if(value > MAX)
    value = MAX;
  else if(value < MIN)
    value = MIN;
}


}//end namespace

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<beagle_io::PWM_CONTROL>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
