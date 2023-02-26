[![Build Status](https://github.com/wuehr1999/lois_ecu_ros/actions/workflows/ros2.yml/badge.svg)](https://github.com/wuehr1999/lois_ecu_ros/actions/workflows/ros2.yml)

# lois_ecu_ros
ROS2 driver for [lois_ecu](https://github.com/wuehr1999/lois_ecu)

ROS Node for interfacing ECU PCB via Serial communication (termios character device driver).

## Features

- Realtime RPM measurement
- Realtime Torque measurement
- Realtime Heading measurement
- Control via Twist messages
- Odometry calculation
- Manipulation and storage of ECU runtime parameters (see config folder)

## 3rd-party dependencies
 - inipp: https://github.com/mcmtroffaes/inipp

## Installation

Clone this repository into Your ROS2 workspace and simply run ```colcon build```.

## Subscribed Topics
  **cmd_vel** (geometry_msgs::Twist)

  linear.x and angular.z are supported

  **enable_rpmctrl** (std_msgs::Bool)

  Enables rpm controller. If disabled, cmd_vel linear.x is dutycycle and cmd_vel angular.z ist dutycycle delta.

  **rpm_request** (std_msgs::UInt16MultiArray)

  Requests rpm record in format: [samples left, samples right]. Maximum sample number is 1024. Publishes record to rpm_record topic. All running measurements are aborted.

  **heading_request** (std_msgs::UInt32)

  Requests heading record. Maximum sample number is 2056. Publishes record to heading_record topic. All running measurements are aborted.

  **ecu_params_request** (std_msgs::Bool)

  Triggers publishing of current ecu runtime parameters on ecu_params.

  **ecu_params** (ecu::runtimeparameters)

  ECU runtime parameters. See ini-File in config folder for format.

  **torque_request** (std_msgs::UInt16MultiArray)

  Requests torque record in format: [samples left, samples right]. Maximum samples are 1024. Publishes record to torque_record topic. All running measurements are aborted.

## Published Topics
  **rpm_record** (std_msgs::UInt16MultiArray)

  Durations of hal encoder HIGH and LOW times, with 100us resolution. Format: [left_0, ..., left_n, right_0, ..., right_m]
. The sample counts n and m are known from the request.

  **torque_record** (std_msgs::UInt16MultiArray)

  12-bit current measurements, accquired by averaging 100 samples. Period is 100ms. Format: [left_0, ..., left_n, right_0, ..., right_m]
. The sample counts n and m are known from the request.

  **heading_record** (std_msgs::Int16MultiArray)

  Compass realtime samples from -180° to 180°. Format: [heading0, ..., heading_n]
. The sample counts n is known from the request.

  **emergency_stop** (diagnostic_msgs::DiagnosticStatus)

  Emergency stop event. Release: level OK, Push: level ERROR

  **kvh_heading** (std_msgs::Int32)

  Perdiodic KVH-C100 compass heading.

  **gps_data** (sensor_msgs::NavSatFix)

  Perdiodic UBLOX GPS data

  **odom_wheel** (nav_msgs::Odometry)

  Periodic wheel odomery data, tf is also published if enabled.

  **ecu_params** (ecu::runtimeparameters)

  Current runtime parameters.

## Parameters
  **wheel_width_m** (float, default: 0.5)

  Robot width between wheel centers in m

  **wheel_radius_m** (float, default: 0.13)

  Wheel radius in m

  **encoder_steps** (float, default: 66)

  Encoder steps per revolution

  **port** (string, default: "/dev/ttyUSB0")

  **baud** (int, default: 115200)

  **odom_parent** (string, default: "odom")

  **odom_child** (string, default: "base_link")
  
  **configfile** (string, default: "/home/jonas/lois/colcon_ws/lois_ecu/config/runtimeparameters.ini")

