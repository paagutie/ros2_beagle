#ifndef MPU9250_H
#define MPU9250_H

/**
 * MPU9250.hpp
 *
 * @author     Pablo Gutiérrez
 * @date       08/07/2020
 */

// IMU configuration and check registries
#define WHO_AM_I          0x75
#define DEVICE_ID         0x71
#define MPU9250_ADDRESS   0x68
#define GYRO_CONFIG       0x1B
#define CONFIG            0x1A
#define ACCEL_CONFIG      0x1C
#define ACCEL_CONFIG2     0x1D

// Other IMU registeries
#define PWR_MGMT_1   0x6B
#define INT_STATUS   0x3A
#define INT_PIN_CFG  0x37
#define INT_ENABLE   0x38
#define SMPLRT_DIV   0x19

// Accelerometer, temperature, and gyroscope data out registries
#define ACCEL_XOUT_H  0x3B
#define ACCEL_XOUT_L  0x3C
#define ACCEL_YOUT_H  0x3D
#define ACCEL_YOUT_L  0x3E
#define ACCEL_ZOUT_H  0x3F
#define ACCEL_ZOUT_L  0x40

#define TEMP_OUT_H    0x41
#define TEMP_OUT_L    0x42

#define GYRO_XOUT_H   0x43
#define GYRO_XOUT_L   0x44
#define GYRO_YOUT_H   0x45
#define GYRO_YOUT_L   0x46
#define GYRO_ZOUT_H   0x47
#define GYRO_ZOUT_L   0x48

// Full scale range
#define AFS_2G  0
#define AFS_4G  1
#define AFS_8G  2
#define AFS_16G 3

#define GFS_250DPS  0
#define GFS_500DPS  1
#define GFS_1000DPS 2
#define GFS_2000DPS 3



#include <iostream>
#include <time.h>
#include <stdint.h>
#include <string.h>

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <cstdint>
#include <fstream>

#include "I2CBus.hpp"

// Type def for function pointer
//typedef int (*i2c_read_write_t)(unsigned char addr, unsigned char *data, unsigned char len);

// Read and write struct
//struct i2c_device_t {
//  i2c_read_write_t i2c_write;
//  i2c_read_write_t i2c_read;
//};

// IMU data structure
struct imu_t {
  float ax, ay, az, gx, gy, gz;
};

// Gyro calibration structure
struct gyro_cal_t {
  float x, y, z;
};

struct point_t {
  float x, y, z;
};

// Attitude structure
struct attitude_t {
  float roll, pitch, yaw;
};

class MPU9250 {
private:
  unsigned char _addr;
  int fd;
  //i2c_device_t _i2c_dev;

  unsigned char data[14];
  float acc_cal_data[6] = {0,0,0,0,0,0};

  float accelPitch, accelRoll;
  beagle_io::I2CBus i2cObject;


public:
  MPU9250(const char * i2cDeviceFilePath) : i2cObject(i2cDeviceFilePath)
  {
      i2cObject.addressSet(MPU9250_ADDRESS);
      gyro_raw_cal.x = 0.0;
      gyro_raw_cal.y = 0.0;
      gyro_raw_cal.z = 0.0;
  }


  bool initIMU()
  {
    // Check if a valid connection has been established
    data[0] = WHO_AM_I;
    i2cObject.i2c_write(data, 1);
    i2cObject.i2c_read(data, 1);
    unsigned char whoAmI = data[0];

    if (whoAmI == DEVICE_ID){
        // Activate/reset the IMU
        i2cObject.write2bytes(PWR_MGMT_1, 0x00);
        // Auto select clock source to be PLL gyroscope reference if ready else
        i2cObject.write2bytes(PWR_MGMT_1, 0x01);

        // Configure Gyro and Thermometer
        // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
        // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
        // be higher than 1 / 0.0059 = 170 Hz
        // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
        // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
        i2cObject.write2bytes(CONFIG, 0x03);

        // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
        i2cObject.write2bytes(SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate

        // Set gyroscope full scale range
        data[0] = GYRO_CONFIG;
        i2cObject.i2c_write(data, 1);
        i2cObject.i2c_read(data, 1); // get current ACCEL_CONFIG register value
        uint8_t c = data[0];
        c = c & ~0x03; // Clear Fchoice bits [1:0]
        c = c & ~0x18; // Clear GFS bits [4:3]
        c = c | GFS_2000DPS << 3; // Set full scale range for the gyro
        i2cObject.write2bytes(GYRO_CONFIG, c); // Write new GYRO_CONFIG value to register

        // Set accelerometer full-scale range configuration
        data[0] = ACCEL_CONFIG;
        i2cObject.i2c_write(data, 1);
        i2cObject.i2c_read(data, 1); // get current ACCEL_CONFIG register value
        c = data[0];
        c = c & ~0x18;  // Clear AFS bits [4:3]
        c = c | AFS_16G << 3; // Set full scale range for the accelerometer
        i2cObject.write2bytes(ACCEL_CONFIG, c); // Write new ACCEL_CONFIG register value

        // Set accelerometer sample rate configuration
        // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
        // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
        data[0] = ACCEL_CONFIG2;
        i2cObject.i2c_write(data, 1);
        i2cObject.i2c_read(data, 1); // get current ACCEL_CONFIG register value
        c = data[0];
        c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
        c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
        i2cObject.write2bytes(ACCEL_CONFIG2, c); // Write new ACCEL_CONFIG2 register value

        // Configure Interrupts and Bypass Enable
        // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
        // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
        // can join the I2C bus and all can be controlled by the Arduino as master
        i2cObject.write2bytes(INT_PIN_CFG, 0x22);
        i2cObject.write2bytes(INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
        std::cout << "IMU Pass" << std::endl;
        return true;
    }

    return false;
    exit(1);
  }


  void readCalData()
  {
    // Get new data
    //data[0] = INT_STATUS;
    //i2cObject.i2c_write(data, 1);
     // wait for magnetometer data ready bit to be set
     //if(i2cObject.i2c_read(data, 1) & 0x01) 
     //{ 
      readRawData();

      // Convert accelerometer values to g's
      imu_cal.ax = imu_raw.ax * accel_to_ms2;
      imu_cal.ay = imu_raw.ay * accel_to_ms2;
      imu_cal.az = imu_raw.az * accel_to_ms2;

      imu_cal.ax = (imu_cal.ax - acc_bias.x)/acc_scale.x;
      imu_cal.ay = (imu_cal.ay - acc_bias.y)/acc_scale.y;
      imu_cal.az = (imu_cal.az - acc_bias.z)/acc_scale.z;

      // Remove gyro offset
      imu_cal.gx = imu_raw.gx - gyro_raw_cal.x;
      imu_cal.gy = imu_raw.gy - gyro_raw_cal.y;
      imu_cal.gz = imu_raw.gz - gyro_raw_cal.z;

      // Convert gyro values to degrees per second
      imu_cal.gx = imu_cal.gx * gyro_to_degs;
      imu_cal.gy = imu_cal.gy * gyro_to_degs;
      imu_cal.gz = imu_cal.gz * gyro_to_degs;
     //}
  }

  void readRawData()
  {
    // Subroutine for reading the raw data
    data[0] = ACCEL_XOUT_H;
    i2cObject.i2c_write(data, 1);
    i2cObject.i2c_read(data, 14);

    // Read data - Temperature falls between accel and gyro registers
    imu_raw.ax = (short) (data[0]  << 8 | data[1]);
    imu_raw.ay = (short) (data[2]  << 8 | data[3]);
    imu_raw.az = (short) (data[4]  << 8 | data[5]);

    imu_raw.gx = (short) (data[8]  << 8 | data[9]);
    imu_raw.gy = (short) (data[10] << 8 | data[11]);
    imu_raw.gz = (short) (data[12] << 8 | data[13]);

    temperature = (short) (data[6] << 8 | data[7]);
  }

  void compFilter(float dt, float tau)
  {
    // Read calibrated data
    readCalData();

    // Complementary filter
    accelPitch = atan2(imu_cal.ay, imu_cal.az) * (180 / M_PI);
    accelRoll = atan2(imu_cal.ax, imu_cal.az) * (180 / M_PI);

    attitude.roll = (tau)*(attitude.roll - imu_cal.gy*dt) + (1-tau)*(accelRoll);
    attitude.pitch = (tau)*(attitude.pitch + imu_cal.gx*dt) + (1-tau)*(accelPitch);
    attitude.yaw += imu_cal.gz*dt;
  }

  float getAres(int Ascale)
  {
    // Set the full scale range for the accelerometer
    switch (Ascale){
      case AFS_2G:
        accel_to_ms2 = 2.0/32768.0;  //9.80665
        i2cObject.write2bytes(ACCEL_CONFIG, 0x00);
        return accel_to_ms2;
      case AFS_4G:
        accel_to_ms2 = 4.0/32768.0;
        i2cObject.write2bytes(ACCEL_CONFIG, 0x08);
        return accel_to_ms2;
      case AFS_8G:
        accel_to_ms2 = 8.0/32768.0;
        i2cObject.write2bytes(ACCEL_CONFIG, 0x10);
        return accel_to_ms2;
      case AFS_16G:
        accel_to_ms2 = 16.0/32768.0;
        i2cObject.write2bytes(ACCEL_CONFIG, 0x18);
        return accel_to_ms2;
      default:
        return 0;
    }    
  }

  float getGres(int Gscale)
  {
    // Set the full scale range for the gyroscope
    switch (Gscale){
      case GFS_250DPS:
        gyro_to_degs = 250.0/32768.0;
        i2cObject.write2bytes(GYRO_CONFIG, 0x00);
        return gyro_to_degs;
      case GFS_500DPS:
        gyro_to_degs = 500.0/32768.0;
        i2cObject.write2bytes(GYRO_CONFIG, 0x08);
        return gyro_to_degs;
      case GFS_1000DPS:
        gyro_to_degs = 1000.0/32768.0;
        i2cObject.write2bytes(GYRO_CONFIG, 0x10);
        return gyro_to_degs;
      case GFS_2000DPS:
        gyro_to_degs = 2000.0/32768.0;
        i2cObject.write2bytes(GYRO_CONFIG, 0x18);
        return gyro_to_degs;
      default:
        return 0;
    }
  }

  void gyroCalibration(int numCalPoints = 1000)
  {
    // Run calibration for given number of points
    for (int ii = 0; ii < numCalPoints; ii++){
        readRawData();
        gyro_raw_cal.x += imu_raw.gx;
        gyro_raw_cal.y += imu_raw.gy;
        gyro_raw_cal.z += imu_raw.gz;
    }

    // Find the averge offset values
    gyro_raw_cal.x = gyro_raw_cal.x / (float)numCalPoints;
    gyro_raw_cal.y = gyro_raw_cal.y / (float)numCalPoints;
    gyro_raw_cal.z = gyro_raw_cal.z / (float)numCalPoints;   

    std::cout << "---------------------------------------" << std::endl;
    std::cout << "Gyroscope raw bias values:" << std::endl;
    std::cout << "\t X: " << gyro_raw_cal.x << std::endl;
    std::cout << "\t Y: " << gyro_raw_cal.y << std::endl;
    std::cout << "\t Z: " << gyro_raw_cal.z << std::endl;
    std::cout << "---------------------------------------" << std::endl;  
  }

  bool load_acc_calibration(void)
  {
    std::string line;
    std::ifstream myfile ("/var/lib/robotcontrol/accel.cal", std::ios_base::in);
    if (myfile.is_open())
    {
      for(int i=0; i<6; i++)
        myfile >> acc_cal_data[i];

      myfile.close();
      acc_bias.x = acc_cal_data[0];
      acc_bias.y = acc_cal_data[1];
      acc_bias.z = acc_cal_data[2];
      acc_scale.x = acc_cal_data[3];
      acc_scale.y = acc_cal_data[4];
      acc_scale.z = acc_cal_data[5];

      std::cout << "---------------------------------------" << std::endl;
      std::cout << "Accelerometer's bias and scales data:" << std::endl;
      std::cout << "\t X: " << acc_bias.x << std::endl;
      std::cout << "\t Y: " << acc_bias.y << std::endl;
      std::cout << "\t Z: " << acc_bias.z << std::endl;
      std::cout << "\t X: " << acc_scale.x << std::endl;
      std::cout << "\t Y: " << acc_scale.y << std::endl;
      std::cout << "\t Z: " << acc_scale.z << std::endl;
      std::cout << "---------------------------------------" << std::endl;

      return true;
    }
    else return false;

  }



  void setGyroCalibration(gyro_cal_t gyro)
  {
    gyro_raw_cal = gyro;
  }

  gyro_cal_t getGyroCalibration()
  {
    return gyro_raw_cal;
  }

  // Variables
  float accel_to_ms2, gyro_to_degs;

  imu_t imu_raw;
  imu_t imu_cal;
  point_t acc_bias;
  point_t acc_scale;
  point_t acc_;
  attitude_t attitude;

  gyro_cal_t gyro_raw_cal;

  int temperature;
};

#endif //MPU9250_H
