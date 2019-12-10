/*************************************************************
** bn0055.cpp
** header file for both bn0055_base_functions.cpp and
** bn0055_util_functions.cpp in bn0055_fusion_imu package
**
** written for using the bn0055 in uart serial mode with a
** Raspberry pi UART pins 14 and 15 enabled - using the pigpiod
** C-interface. The Pigpio daemon must be running (sudo pigpiod)
** See http://abyz.me.uk/rpi/pigpio/pigpiod.html for more info
** Much thanks to Joan for her work on the Pigpio libraries and
** support on various forums.
**
** Should work via USB serial (with FTDI USB-TTL or similar converter)
** by simply changing the const std::string SERIAL_PORT in bn0055.h
** Currently, this works in accel/gyro only fusion mode but plans are to add
** 9DOF mode that fuses the magnetometer data as well.
**
** This is an early alpha version, written to be relatively simple to follow
** as an accompaniment to the book Practical Robotics in C++ by myself, but
** further development and config options are planned.
** Follow facebook.com/practicalrobotics for the book release and extra robotics content.
**
** 1111 is used as an error code in several places
**
** Author: Lloyd Brombach
** lbrombach2@gmail.com
** github.com/lbrombach
** Date 12/8/2019
**
*************************************************************/

#ifndef BN0055_H_INCLUDED
#define BN0055_H_INCLUDED

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include <string>




// ttyAMA0 works with a PI3 running lubuntu, One might also try ttySerial0 or ttyS0 for the uart
// Of course, if using with an ftdi via USB, try ttyUSB0
const std::string SERIAL_PORT = "/dev/ttyAMA0";

//Recommend you set this to the config folder in your bn0055_fusion_imu package.
const std::string DEFAULT_FILE_PATH = "/home/lloyd/catkin_ws/src/bn0055_fusion_imu/config/bn0055_config_data.dat";



const double PI = 3.141592;
const int START = 0xAA; //start everry msg with this byte
const int WRITE = 0x00; //write cmd
const int READ  = 0x01; //read cmd
const int CONFIG_MODE = 0;  //the opr_code for config mode
const int IMU_MODE = 0x08; //the opr_code for accel/gyro fusion mode
const int MY_UNITS = 134; // 10000110 - defines unit preferences. read/write at UNIT_SEL reg
const int CALIB_REQUIRED_FOR_IMU_MODE = 60; //60 = 00111100
const int SUCCESS_CODE = 0xBB; // 187 signals successful write, 0xEE (238) is a fail code

const int AXIS_MAP_SIGN = 0X42;
const int AXIS_MAP_CONFIG = 0X41;
const int AXIS_REMAP_CONFIG_VAL_DEFAULT = 0X24;
const int AXIS_REMAP_CONFIG_VAL_90_CW = 0X21;
const int AXIS_REMAP_CONFIG_VAL_180 = 0X24;
const int AXIS_REMAP_CONFIG_VAL_270_CW = 0X21;
const int AXIS_REMAP_SIGN_VAL_DEFAULT = 0X00;
const int AXIS_REMAP_SIGN_VAL_90_CW = 0X02;
const int AXIS_REMAP_SIGN_VAL_180 = 0X06;
const int AXIS_REMAP_SIGN_VAL_270_CW = 0X04;



//addresses of calibration offset and radius data registers
const int MAG_RADIUS_MSB    = 0x6a;
const int MAG_RADIUS_LSB    = 0x69;
const int ACC_RADIUS_MSB    = 0x68;
const int ACC_RADIUS_LSB    = 0x67;
const int GYR_OFFSET_Z_MSB  = 0x66;
const int GYR_OFFSET_Z_LSB  = 0x65;
const int GYR_OFFSET_Y_MSB  = 0x64;
const int GYR_OFFSET_Y_LSB  = 0x63;
const int GYR_OFFSET_X_MSB  = 0x62;
const int GYR_OFFSET_X_LSB  = 0x61;
const int MAG_OFFSET_Z_MSB  = 0x60;
const int MAG_OFFSET_Z_LSB  = 0x5F;
const int MAG_OFFSET_Y_MSB  = 0x5E;
const int MAG_OFFSET_Y_LSB  = 0x5D;
const int MAG_OFFSET_X_MSB  = 0x5C;
const int MAG_OFFSET_X_LSB  = 0x5B;
const int ACC_OFFSET_Z_MSB  = 0x5A;
const int ACC_OFFSET_Z_LSB  = 0x59;
const int ACC_OFFSET_Y_MSB  = 0x58;
const int ACC_OFFSET_Y_LSB  = 0x57;
const int ACC_OFFSET_X_MSB  = 0x56;
const int ACC_OFFSET_X_LSB  = 0x55;




//this node reads these registers for fusion-algorithm data
//LIA are linear accel data with gravity vectors removed
//the node publishes the quaternion data, but if just heading is needed
//the EUL_Heading (and others) are regular euler angles
const int LIA_Data_Z_MSB    = 0x2D;
const int LIA_Data_Z_LSB    = 0x2C;
const int LIA_Data_Y_MSB    = 0x2B;
const int LIA_Data_Y_LSB    = 0x2A;
const int LIA_Data_X_MSB    = 0x29;
const int LIA_Data_X_LSB    = 0x28;
const int QUA_Data_z_MSB    = 0x27;
const int QUA_Data_z_LSB    = 0x26;
const int QUA_Data_y_MSB    = 0x25;
const int QUA_Data_y_LSB    = 0x24;
const int QUA_Data_x_MSB    = 0x23;
const int QUA_Data_x_LSB    = 0x22;
const int QUA_Data_w_MSB    = 0x21;
const int QUA_Data_w_LSB    = 0x20;
const int EUL_Pitch_MSB     = 0x1F;
const int EUL_Pitch_LSB     = 0x1E;
const int EUL_Roll_MSB      = 0x1D;
const int EUL_Roll_LSB      = 0x1C;
const int EUL_Heading_MSB   = 0x1B;
const int EUL_Heading_LSB   = 0x1A;
const int GYR_DATA_Z_MSB    = 0x19;
const int GYR_DATA_Z_LSB    = 0x18;
const int GYR_DATA_Y_MSB    = 0x17;
const int GYR_DATA_Y_LSB    = 0x16;
const int GYR_DATA_X_MSB    = 0x15;
const int GYR_DATA_X_LSB    = 0x14;


//currently not reading from these registers
//these accel data are raw - without gravity vectors removed
const int MAG_DATA_Z_MSB    = 0x13;
const int MAG_DATA_Z_LSB    = 0x12;
const int MAG_DATA_Y_MSB    = 0x11;
const int MAG_DATA_Y_LSB    = 0x10;
const int MAG_DATA_X_MSB    = 0x0F;
const int MAG_DATA_X_LSB    = 0x0E;
const int ACC_DATA_Z_MSB    = 0x0D;
const int ACC_DATA_Z_LSB    = 0x0C;
const int ACC_DATA_Y_MSB    = 0x0B;
const int ACC_DATA_Y_LSB    = 0x0A;
const int ACC_DATA_X_MSB    = 0x09;
const int ACC_DATA_X_LSB    = 0x08;

//some mode and utility registers.
const int SYS_TRIGGER       = 0x3F;
const int PWR_MODE          = 0x3E;
const int OPR_MODE          = 0x3D;
const int UNIT_SEL          = 0x3B;
const int SYS_ERR           = 0x3A;
const int SYS_STATUS        = 0x39;
const int INTR_STAT         = 0X37;
const int ST_RESULT         = 0X36;
const int CALIB_STAT        = 0X35;
const int TEMPERATURE       = 0x34;



/******************FUNCTIONS FOUND IN BN0055_UTIL_FUNCTIONS.H*********************/

//checkes current status and saves calibration and config data
bool save_config(int pi, int serhandle);

//after reset or power on, run this to set mode, unit preferences, and calibration data from file
bool initialize_bn0055(int pi, int serHandle);

//More of a visual tool... once the unit is in IMU mode, it is always trying to calibrate if it
//is out of calibration. This keeps checking calibration status and lets you know when its good
bool calibrate_bn0055(int pi, int serHandle);

//just peeks to see if file exists in default location
void check_for_file();

//reads registers and returns true if valid read or false if unable
bool get_imu_data(int pi, int serHandle, sensor_msgs::Imu &imu);

//prints current config and claibration data to screen
void show_current_config(int pi, int serHandle);

//checks current mode, unit setting, and calibration for compatibility with publisher node and functions
bool verify_accel_gyro_fusion_config(int pi, int serHandle);

//returns heading or 1111 if failed read attempt
double get_heading(int pi, int serHandle);

//prints raw accel data to screen.
void get_accel(int pi, int serHandle);

//allows user to compensate for sensor mounted in orientaion other than default
//90 degree incrememts about the z axis only)
bool rotate_axis_map(int pi, int serHandle);




/******************FUNCTIONS FOUND IN BN0055_BASE_FUNCTIONS.H*********************/

//helper functions for combining bytes in a single value
int UnsignedToInt(uint8_t low, uint8_t high);
int SignedToInt(uint8_t low, int8_t high);

//just asks for a y or n input and returns true or false
bool confirm();

//issues reset command
bool reset_bn0055(int pi, int serHandle);

//sometimes unit doesn't seem to respond and I wonder if I havea  bug where I'm leaving an
//unread byte. This requests a one byte value then tries to read 128 bytes (the max) to
//catch anything that might have been hanging. Outputs to screen so I can see any extra data
void clear_buffer(int pi, int serHandle);

//returns a single byte. Usually to be broken down and individual buts read
int get_byte(int pi, int serHandle, int reg);

//tries to read numBytes consecutive bytes starting at regLSB and combine them
// into numBytes/2 values placed into data[]. returns true of false based on success
bool get_bytes(int pi, int serHandle, int regLSB, int numBytes, int data[]);

//writes a byte to regLSB
bool set_byte(int pi, int serHandle, int regLSB, int val);

//bool set_bytes(int pi, int serHandle, int regLSB, int numBytes, int val);

//checks the response code (and displays result) after a read/write command sent
bool write_success(int response);

//reads axis map config byte and axis map sign byte.
//returns 0-3 code (or 1111 for failed read)
// 0 = set for sensor mounted 270 degrees CW from default
// 1 = set for default mounting
// 2 = set for sensor mounted 180 degrees from default
// 3 = set for sensor mounted 90 degrees CW from default
int get_axis_map(int pi, int serHandle);

//sets axis map config data
bool set_axis_remap(int pi, int serHandle, int axisMapConfig);


//reads system status
int get_system_status(int pi, int serHandle);

//checks calibrations status. Must not be in config mode
int get_calibration_status(int pi, int serHandle);

//reads the unit preferences (degrees vs radians, m/s^2 vs milliG, etc
int get_units(int pi, int serHandle);

//reads the operation mode. 0 = config, 8 = accel/gyro IMU fusion
int get_opr_mode(int pi, int serHandle);

//reads power mode. normal, low, suspend
int get_pwr_mode(int pi, int serHandle);

//self test result
bool get_st_result(int pi, int serHandle);

//writes config mode command to opr_mode register.
bool set_config_mode(int pi, int serHandle);

//writes "8" (accel/gyro fusion mode) to opr_mode register
bool set_opr_mode_8(int pi, int serHandle);

//writes unit preferences to unts register. Currently hard-coded for
//m/s^2, rads/sec, and radians per standard ROS units.
bool set_my_units(int pi, int serHandle);





#endif // BN0055_H_INCLUDED
