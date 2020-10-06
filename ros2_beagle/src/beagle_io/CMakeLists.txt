cmake_minimum_required(VERSION 3.5)
project(beagle_io)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


include_directories(include
  ${rclcpp_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_imu src/imu_sensor.cpp)
ament_target_dependencies(${PROJECT_NAME}_imu rclcpp sensor_msgs geometry_msgs)

add_executable(${PROJECT_NAME}_servos src/pwm_control.cpp)
ament_target_dependencies(${PROJECT_NAME}_servos rclcpp sensor_msgs)


#Install C++ targets
install(TARGETS
  ${PROJECT_NAME}_imu
  ${PROJECT_NAME}_servos
  DESTINATION lib/${PROJECT_NAME}
)

#Install Launch file
#install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
