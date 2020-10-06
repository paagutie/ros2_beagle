#ifndef UTIL_TOOLS_HPP
#define UTIL_TOOLS_HPP

/**
 * util_tools.hpp
 *
 * @author     Pablo Gutiérrez
 * @date       08/07/2020
 */


#include <cstdint>
#include <math.h>

namespace beagle_io {

// IMU data structure
struct imu_t {
  float ax, ay, az, gx, gy, gz;
  void clear()
  {
    ax = 0.0;
    ay = 0.0;
    az = 0.0;
    gx = 0.0;
    gy = 0.0;
    gz = 0.0;
  }
};

// Gyro calibration structure
struct point_t {
  float x, y, z;
  void clear()
  {
     x = 0.0;
     y = 0.0;
     z = 0.0;
  }
};

// Attitude structure
struct attitude_t {
  float roll, pitch, yaw;
};


bool replace(std::string& str, const std::string& from, const std::string& to) {
    size_t start_pos = str.find(from);
    if(start_pos == std::string::npos)
        return false;
    str.replace(start_pos, from.length(), to);
    return true;
}

struct esc_struct_t
{
    int _channel;
    int _width_us;
};

//---------------------------------
// JOYSTICK
//---------------------------------

struct joystick_t
{
    bool A;
    bool B;
    bool X;
    bool Y;
    bool Back;
    bool Start;
    bool LB;
    bool RB;
    bool LT;
    bool RT;
    bool Home;
    int an_horizontal_left;
    int an_horizontal_right;
    int an_vertical_left;
    int an_vertical_right;
    int dig_horizontal;
    int dig_vertical;

    double axes[4];
    int buttons[12];
};

//---------------------------------
// MOTOR
//---------------------------------

// Range with deadzone
constexpr uint16_t MOTOR_FULL_REV_PWM = 1100;
constexpr uint16_t MOTOR_STOP_PWM = 1500;
constexpr uint16_t MOTOR_FULL_FWD_PWM = 1836;
constexpr uint16_t MOTOR_DEADBAND_UPPER_PWM = 1540;
constexpr uint16_t MOTOR_DEADBAND_LOWER_PWM = 1460;
constexpr uint16_t JOYSTICK_DEADBAND = 0.05;

int scale_joy_input(const float axis)
{
    float penPos = 0.0;
    float penNeg = 0.0;
    int pwmOut = 0;
    penPos = (MOTOR_FULL_FWD_PWM - MOTOR_DEADBAND_UPPER_PWM)/(1-JOYSTICK_DEADBAND);
    penNeg = (MOTOR_DEADBAND_LOWER_PWM - MOTOR_FULL_REV_PWM)/(1-JOYSTICK_DEADBAND);

    float b_upper = MOTOR_FULL_FWD_PWM - penPos;
    float b_lower = MOTOR_FULL_REV_PWM + penNeg;

    if(axis > JOYSTICK_DEADBAND)
        pwmOut = (int)(axis*penPos + b_upper);
    else if(axis < -JOYSTICK_DEADBAND)
        pwmOut = (int)(axis*penNeg + b_lower);
    else
        pwmOut = MOTOR_STOP_PWM;

    return pwmOut;
}

} //end_namespace

#endif //UTIL_TOOLS_HPP
