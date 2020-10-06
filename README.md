# ros2_beagle
Beaglebone Blue under ROS2

# Description

The project includes the possibility of using the 8 servo outputs provided by the beaglebone blue board, as well as reading data from the MEMS MPU9250 under ROS2 Dashing and Ubuntu 18.04. The ROS2 driver for a generic Linux joystick is used to test the motors.
Due to the architecture of the processor on the beaglebone board, the installation of ROS2 was carried out using a script hosted in the project (ros2_dashing_ubuntu_arm32v7).

# Requirements

- ROS2 Dashing Diademata or Eloquent Elusor.
- Ubuntu 18.04

# Installation
```
$ git clone https://github.com/paagutie/ros2_beagle.git
$ cd ros2_beagle
$ source /home/beagle/ros2_dashing/install/local_setup.bash
$ colcon build
```
# Usage

## On the main Computer ##
```
& source /opt/ros/dashing/setup.bash
$ ros2 run joy joy_node
```
## On the Beaglebone Blue ##
To launch the nodes independently you can use the following commands for each node respectively.

###### Terminal 1 ######
```
$ source install/local_setup.bash
$ ros2 run beagle_io beagle_io_servos
```
###### Terminal 2 ######
```
$ source install/local_setup.bash
$ ros2 run beagle_io beagle_io_imu
```

To launch the nodes using a ros2 launch file
```
$ source install/local_setup.bash
$ ros2 launch beagle_launch beagle_launch.py
```

# License
- MIT
