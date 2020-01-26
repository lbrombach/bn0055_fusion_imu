ROS package for bn0055 absolute orientation sensor -
written for using the bn0055 in uart serial mode with a
Raspberry pi UART pins 14 and 15 enabled - using the pigpiod
C-interface. The Pigpio daemon must be running (sudo pigpiod)
See http://abyz.me.uk/rpi/pigpio/pigpiod.html for more info.
** Much thanks to Joan for her work on the Pigpio libraries and
support on various forums.
**
Should work via USB serial (with FTDI USB-TTL or similar converter)
by simply changing the const std::string SERIAL_PORT in bn0055.h
Currently, this works in accel/gyro only fusion mode but plans are to add
9DOF mode that fuses the magnetometer data as well.
**
This is an early alpha version, written to be relatively simple to follow
as an accompaniment to the book Practical Robotics in C++ by myself, but
further development and config options are planned.
Follow facebook.com/practicalrobotics for the book release and extra robotics content.
**
1111 is used as an error code in several places
**
Author: Lloyd Brombach
lbrombach2@gmail.com
github.com/lbrombach
Date 12/8/2019
***



Nodes:

bn0055_setup_util_node:
Run as a node with rosrun bn0055_fusion_imu bn0055_setup_util_node.
Has command line interface that displays a number of configuration, mode, and
calibration values. Options to run calibration routine and load calibration
data from default or user-specified file. No publishers or subscribers.



bn0055_accel_gyro_fusion_pub
This is the publisher node. If unit requires calibration, attempts to read from default
calibration file location. If different calibration is desired, load it with the
setup util node before running this node. Magnetometer not active at this time
(to be added as option) in the future. Runs unit in accel-gyro fusion mode
(the most appropriate mode for indoor robots. OK for outdoor robots).
***MUST RUN CALIBRATION ROUTINE IN SETUP UTIL NODE AT LEAST ONCE BEFORE RUNNING
THIS NODE OR YOUR UNIT WILL USE WHATEVER MY LAST CALIBRATION DATA IS. CALIBRATION
DATA IS INDIVIDUAL FOR EVERY UNIT***

**Default port is /dev/ttyAMA0 for Raspberry Pi GPIO UART. Until I get a chance to
parameterize, change the port as necessary in bn0055.h line 44.

Subscribers: none

Publishers:
sensor_msgs::Imu   topic=/imu  frame=imu
