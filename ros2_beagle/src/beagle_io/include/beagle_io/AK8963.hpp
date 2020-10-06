#ifndef AK8963_H
#define AK8963_H

/**
 * AK8963.hpp
 *
 * @author     Pablo Gutiérrez
 * @date       08/07/2020
 */

// Calibration File Locations
#define ACCEL_CAL_FILE	"accel.cal"
#define GYRO_CAL_FILE		"gyro.cal"
#define MAG_CAL_FILE		"mag.cal"

// Magnetometer configuration and check registries
#define AK8963_ADDRESS  0x0C
#define AK8963_WHO_AM_I        0x00
#define AK8963_DEVICE_ID       0x48

//AK8963 Register Addresses
#define AK8963_ST1           0x02
#define AK8963_MAGNET_OUT    0x03
#define AK8963_CNTL1         0x0A
#define AK8963_CNTL2         0x0B
#define AK8963_ASAX          0x10

#define USER_CTRL		  0x6A
#define I2C_MST_EN		0x01<<5

// CNTL1 Mode select
// Power down mode
#define AK8963_MODE_DOWN  0x00
// One shot data output
#define AK8963_MODE_ONE   0x01

// Continous data output 8Hz
#define AK8963_MODE_C8HZ    0x02
// Continous data output 100Hz
#define AK8963_MODE_C100HZ  0x06

// Magneto Scale Select
// 14bit output
#define AK8963_BIT_14  0x00
// 16bit output
#define AK8963_BIT_16  0x01

//Magnetometer AK8963_ST2 register definitions
#define MAGNETOMETER_SATURATION	0x01<<3
//Magnetometer AK8963_ST1 register definitions
#define MAG_DATA_READY		0x01

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

// IMU data structure
struct mag_t {
  float mx, my, mz;
};

class AK8963 {
private:
  unsigned char _mode;
  int fd;
  
  unsigned char data[14];
  float mag_temp[3] = {0,0,0};
  float _magBias[3] = {0,0,0};
  float _magScale[3] = {1,1,1};
  float mag_cal_data[6] = {0,0,0,0,0,0};

  float accelPitch, accelRoll;
  beagle_io::I2CBus i2cObject;

public:
  AK8963(const char * i2cDeviceFilePath, unsigned char mode) : i2cObject(i2cDeviceFilePath)
  { 
      i2cObject.addressSet(AK8963_ADDRESS);
      _mode = mode;
      if(!initMAG())
        exit(1);
  }


  bool initMAG()
  {
    // Check if a valid connection has been established
    data[0] = AK8963_WHO_AM_I;
    i2cObject.i2c_write(data, 1);
    i2cObject.i2c_read(data, 1);
    unsigned char whoAmI = data[0];

    if (whoAmI == AK8963_DEVICE_ID){
        // Power down
        i2cObject.write2bytes(AK8963_CNTL1, 0x00);
        usleep(1000); //0.01 sec 
        // Set read FuseROM mode
        i2cObject.write2bytes(AK8963_CNTL1, 0x0F);
        usleep(1000); //0.01 sec 
        readFactoryCalData();
        // Power down magnetometer  
        i2cObject.write2bytes(AK8963_CNTL1, 0x00);
        usleep(100); //0.01 sec

        // Configure the magnetometer for 16 bit resolution
	      // and continuous sampling mode 2 (100hz)
        unsigned char mfs = AK8963_MODE_C100HZ;
        i2cObject.write2bytes(AK8963_CNTL1, (mfs << 4 | AK8963_MODE_C100HZ)); 
        usleep(100); //0.01 sec 
        return true;
    }

    return false;
  }

  void readFactoryCalData()
  {
    data[0] = AK8963_ASAX;
    i2cObject.i2c_write(data, 1);
    i2cObject.i2c_read(data, 3);
    // Read the xyz sensitivity adjustment values
    mag_factory_adjust.mx = (float)(data[0] - 128)/256.0f + 1.0f;;
    mag_factory_adjust.my = (float)(data[1] - 128)/256.0f + 1.0f;  
    mag_factory_adjust.mz = (float)(data[2] - 128)/256.0f + 1.0f; 

    std::cout << "---------------------------------------" << std::endl;
    std::cout << "Magnetometer factory adjust:" << std::endl;
    std::cout << "\t X: " << mag_factory_adjust.mx << std::endl;
    std::cout << "\t Y: " << mag_factory_adjust.my << std::endl;
    std::cout << "\t Z: " << mag_factory_adjust.mz << std::endl;
    std::cout << "---------------------------------------" << std::endl;

  }

  void readRawData()
  {
    // Subroutine for reading the raw data
    data[0] = AK8963_MAGNET_OUT;
    i2cObject.i2c_write(data, 1);
    i2cObject.i2c_read(data, 7);
    uint8_t c = data[6]; // End data read by reading ST2 register

    if((c & 0x08) != 0x08 ) { // Check if magnetic sensor overflow set, if not then report data
        mag_raw.mx = (int16_t)(((int16_t)data[1] << 8) | data[0]);  // Turn the MSB and LSB into a signed 16-bit value
        mag_raw.my = (int16_t)(((int16_t)data[3] << 8) | data[2]);  // Data stored as little Endian
        mag_raw.mz = (int16_t)(((int16_t)data[5] << 8) | data[4]); 
        mag_temp[0] = mag_raw.mx;
        mag_temp[1] = mag_raw.my;
        mag_temp[2] = mag_raw.mz;

        // multiply by the sensitivity adjustment and convert to units of uT micro
	      // Teslas. Also correct the coordinate system as someone in invensense
	      // thought it would be bright idea to have the magnetometer coordinate
	      // system aligned differently than the accelerometer and gyro.... -__-

        factory_cal_data.mx = mag_raw.my*mag_factory_adjust.my*_mRes;
        factory_cal_data.my = mag_raw.mx*mag_factory_adjust.mx*_mRes;
        factory_cal_data.mz = -mag_raw.mz*mag_factory_adjust.mz*_mRes;

        mag_.mx = (factory_cal_data.mx-mag_bias.mx)*mag_scale.mx;
	      mag_.my = (factory_cal_data.my-mag_bias.my)*mag_scale.my;
	      mag_.mz = (factory_cal_data.mz-mag_bias.mz)*mag_scale.mz;

        /*
        std::cout << "---------------------------------------" << std::endl;
        std::cout << "Magnetometer:" << std::endl;
        std::cout << "\t X: " << mag_raw.mx << std::endl;
        std::cout << "\t Y: " << mag_raw.my << std::endl;
        std::cout << "\t Z: " << mag_raw.mz << std::endl;
        std::cout << "---------------------------------------" << std::endl;
        */
        

    }
    else
      std::cout << "Magnetometer overflow set" << std::endl;
  }

  void readMagnetometer()
  {
    uint8_t st1;
    data[0] = AK8963_ST1;
    i2cObject.i2c_write(data, 1);
    i2cObject.i2c_read(data, 1);

    st1 = data[0];
    if(!(st1&MAG_DATA_READY)){
      std::cout << "No new magnetometer data ready, skipping read" << std::endl;
		}
    else
      readRawData();
  }


  void setMagBias(mag_t bias)
  {
    mag_bias = bias;
  }

  //Raw to uT
  float getMres(int Mscale)
  {
    // Set the full scale range for the gyroscope
    switch (Mscale){
      case AK8963_BIT_14:
        _mRes = 4912.0 / 8190.0; 
        return _mRes;
      case AK8963_BIT_16:
        _mRes = 4912.0 / 32760.0;
        return _mRes;
      default:
        return 0;
    }
  }

  bool load_mag_calibration(void)
  {
    std::string line;
    std::ifstream myfile ("/var/lib/robotcontrol/mag.cal", std::ios_base::in);
    if (myfile.is_open())
    {
      for(int i=0; i<6; i++)
        myfile >> mag_cal_data[i];

      myfile.close();
      mag_bias.mx = mag_cal_data[0];
      mag_bias.my = mag_cal_data[1];
      mag_bias.mz = mag_cal_data[2];
      mag_scale.mx = mag_cal_data[3];
      mag_scale.my = mag_cal_data[4];
      mag_scale.mz = mag_cal_data[5];

      std::cout << "---------------------------------------" << std::endl;
      std::cout << "Magnetometer bias and scale data:" << std::endl;
      std::cout << "\t X: " << mag_bias.mx << std::endl;
      std::cout << "\t Y: " << mag_bias.my << std::endl;
      std::cout << "\t Z: " << mag_bias.mz << std::endl;
      std::cout << "\t X: " << mag_scale.mx << std::endl;
      std::cout << "\t Y: " << mag_scale.my << std::endl;
      std::cout << "\t Z: " << mag_scale.mz << std::endl;
      std::cout << "---------------------------------------" << std::endl;

      return true;
    }
    else return false;

  }


  // Variables
  float _mRes;

  mag_t mag_raw;
  mag_t mag_factory_adjust;
  mag_t factory_cal_data;
  mag_t mag_bias;
  mag_t mag_scale;
  mag_t mag_;


};

#endif //AK8963_H



  /*
  void calibrateMagnetometer(void)
  {
    uint16_t ii = 0, sample_count = 0;
    int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
    int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

    if(_mode == AK8963_MODE_C8HZ) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
    if(_mode == AK8963_MODE_C100HZ) sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms

    for(ii = 0; ii < sample_count; ii++) {

        readRawData();  // Read the mag data   

        for (int jj = 0; jj < 3; jj++) {
            if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
            if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
        }

        if(_mode == AK8963_MODE_C8HZ) usleep(135000);  // at 8 Hz ODR, new mag data is available every 125 ms
        if(_mode == AK8963_MODE_C100HZ) usleep(12000);  // at 100 Hz ODR, new mag data is available every 10 ms
    }

    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

    _magBias[0] = (float) mag_bias[0]*_mRes*mag_cal.mx;  // save mag biases in G for main program
    _magBias[1] = (float) mag_bias[1]*_mRes*mag_cal.my;   
    _magBias[2] = (float) mag_bias[2]*_mRes*mag_cal.mz;  

    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    _magScale[0] = avg_rad/((float)mag_scale[0]);
    _magScale[1] = avg_rad/((float)mag_scale[1]);
    _magScale[2] = avg_rad/((float)mag_scale[2]);
 }
 */
