#ifndef _I2CBUS_HPP
#define _I2CBUS_HPP

/**
 * I2CBus.hpp
 *
 * @author     Pablo Gutiérrez
 * @date       08/07/2020
 */

#include <stdint.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

//#include "exceptions.h"

namespace beagle_io {

class I2CBus
{
public:
    I2CBus(const char * deviceName)
    {
        fd = open(deviceName, O_RDWR);
        if (fd == -1)
        {
            //throw posix_error(std::string("Failed to open I2C device ") + deviceName);
            std::cout << "Failed to open the i2c bus" << std::endl;
            exit(1);
        }
    }
    ~I2CBus()
    {
        close(fd);
    }

    void addressSet(uint8_t address)
    {
        int result = ioctl(fd, I2C_SLAVE, address);
        if (result == -1)
        {
            //throw posix_error("Failed to set address");
            std::cout << "Failed to set address" << std::endl;
            exit(1);
        }
    }

    int write2bytes(unsigned char byte0, unsigned char byte1)
    {
        unsigned char data[2];
        data[0] = byte0;
        data[1] = byte1;
        return i2c_write(data, 2);
    }

    int i2c_read(unsigned char *data, char len){
        if (read(fd, data, len) != len){
            return -1;
        } else {
            return 0;
        }
    }

    int i2c_write(unsigned char *data, char len){
        if (write(fd, data, len) != len){
            return -1;
        } else {
            return 0;
        }
    }

private:
    int fd;

};

} // namespace rov_io

#endif
