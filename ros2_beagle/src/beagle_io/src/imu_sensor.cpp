/**
 * imu_sensor.cpp
 *
 * @author     Pablo Gutiérrez
 * @date       08/07/2020
 */

#include "beagle_io/MPU9250.hpp"
#include "beagle_io/AK8963.hpp"


#include <chrono>
#include <memory>
#include <iomanip>  

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "beagle_io/Madgwick.hpp"


namespace beagle_io {

class IMU : public rclcpp::Node
{
public:
    
    MPU9250 *mpu9250;
    AK8963 *ak8963;
    Madgwick madgwick;

    IMU():
    Node("beagle_io_imu")  
    {

        const char *filename = "/dev/i2c-2";
        //Connect to sensor using file identifier
        mpu9250 = new MPU9250(filename);

        // Initialize the IMU and set the senstivity values
        std::cout << "IMU initialize. Pass/Fail: ";
     
        std::cout << mpu9250->initIMU() << std::endl;
        mpu9250->getAres(AFS_4G);
        mpu9250->getGres(GFS_500DPS);
        sleep(1);

        // Calibrate the gyroscope
        gyro_cal_t gyro_cal;
        std::cout << "Calibrating gyroscope, hold IMU stationary." << std::endl;
        sleep(2);
        mpu9250->gyroCalibration(1000);
        // Display calibration values to user
        gyro_cal = mpu9250->getGyroCalibration();

        sleep(2);

        if(!mpu9250->load_acc_calibration())
        {
            std::cout << "Failed to read accelerometer's calibration data!" << std::endl;
            exit(1);
        }

        ak8963 = new AK8963(filename, AK8963_MODE_C100HZ);
        std::cout <<  ak8963->getMres(AK8963_BIT_16) << std::endl;

        if(!ak8963->load_mag_calibration())
        {
            std::cout << "Failed to read magnetometer's calibration data!" << std::endl;
            exit(1);
        }

        timer_ = this->create_wall_timer(std::chrono::milliseconds(10),std::bind(&IMU::timerCallback, this));
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

    }

private:

    int sampleRate = 4000; // Microseconds (~250 Hz)
    int time2Delay;
    int fd;
    float dt;


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;


    void timerCallback()
    {  
        sensor_msgs::msg::Imu imu_msg;

        mpu9250->readCalData();
        imu_msg.linear_acceleration.x = mpu9250->imu_cal.ax;
        imu_msg.linear_acceleration.y = mpu9250->imu_cal.ay;
        imu_msg.linear_acceleration.z = mpu9250->imu_cal.az;

        imu_msg.angular_velocity.x =  mpu9250->imu_cal.gx;
        imu_msg.angular_velocity.y =  mpu9250->imu_cal.gy;
        imu_msg.angular_velocity.z =  mpu9250->imu_cal.gz;

        //ak8963->readMagnetometer();


        // Update the filter
        madgwick.updateIMU(mpu9250->imu_cal.gx, mpu9250->imu_cal.gy, mpu9250->imu_cal.gz, 
			            mpu9250->imu_cal.ax, mpu9250->imu_cal.ay, mpu9250->imu_cal.az); 


	float roll = madgwick.getRoll();
  	float pitch = madgwick.getPitch();
  	float heading = madgwick.getYaw();

	std::cout << "roll: " << roll << " pitch: " << pitch << " heading: " << heading << std::endl;
        imu_pub_->publish(imu_msg);

    }


};

} // namespace rov_io

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<beagle_io::IMU>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
