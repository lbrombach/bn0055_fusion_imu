/*************************************************************
** bn0055_accel_gyro_fusion_pub.cpp
** ROS publisher node for bn0055 absolute orientation sensor
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
#include "ros/ros.h"
#include <iostream>
#include <pigpiod_if2.h>
#include <fstream>
#include "bn0055_fusion_imu/bn0055.h"

using namespace std;

int PigpioSetup()
{
  char *addrStr = NULL;
  char *portStr = NULL;
  return pigpio_start(addrStr, portStr);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bn0055_publisher");
    ros::NodeHandle node;
    ros::Publisher pub = node.advertise<sensor_msgs::Imu>("imu", 0);
    sensor_msgs::Imu imu;

    //initialize pipiod interface
    int pi = PigpioSetup();
    if(pi>=0)
    {
        cout<<"daemon interface started ok at "<<pi<<endl<<endl;
    }
    else
    {
        cout<<"************************************************************"<<endl;
        cout<<"*   Failed to connect to PIGPIO Daemon - is it running?    *"<<endl;
        cout<<"*Try running   sudo pigpiod   and try again. ABORTING NODE *"<<endl;
        cout<<"************************************************************"<<endl;
        return -1;
    }

    int serHandle = serial_open(pi, (char*)SERIAL_PORT.c_str(), 115200, 0);
    if(serHandle < 0)
    {
        cout<<"*************************************************************"<<endl;
        cout<<"*Unable to open serial device. Check port Defined in bn0055.h*"<<endl;
        cout<<"*for correctness and permission.       ABORTING NODE         *"<<endl;
        cout<<"*************************************************************"<<endl;
        return -1;
    }

    if(!verify_accel_gyro_fusion_config(pi, serHandle) && !initialize_bn0055(pi, serHandle))
    {

        cout<<"************************************************************"<<endl;
        cout<<"* Unable to verify or set BN0055 IMU config. ABORTING NODE *"<<endl;
        cout<<"************************************************************"<<endl;
        return -1;
    }
    cout<<"IMU Config ok. IMU Ready. "<<endl;


    //set our range message fixed data
    imu.header.frame_id = "imu";

    //set absolute orientaion and angular velocity to "do not use"
    //imu.angular_velocity_covariance[0] = -1;

    //set accel covariance to unknown
    for(int i = 0; i<9; i++)
    {
        if(i==0 || i==4 || i==8)
        {
            imu.linear_acceleration_covariance[i]=.0001;
            imu.orientation_covariance[i] = .0001;
            imu.angular_velocity_covariance[i] = .0001;
        }
        else
        {
            imu.linear_acceleration_covariance[i]=0;
            imu.orientation_covariance[i] = 0;
            imu.angular_velocity_covariance[i] = 0;
        }
    }
    int good_reads = 0;
    int total_errors = 0;
    int consecutive_errors = 0;
    ros::Rate loop_rate(30);
    while(ros::ok && consecutive_errors < 200)
    {
        if(get_imu_data(pi, serHandle, imu) == true)
        {

            imu.linear_acceleration_covariance[0]=.0001;
            imu.orientation_covariance[0] = .0001;
            imu.angular_velocity_covariance[0] = .0001;
            pub.publish(imu);
            consecutive_errors = 0;
            good_reads++;
        }
        else
        {
            total_errors++;
            cout<<"Error count = "<<consecutive_errors++<<endl;
            imu.linear_acceleration_covariance[0]=-1;
            imu.orientation_covariance[0] = -1;
            imu.angular_velocity_covariance[0] = -1;
            pub.publish(imu);
        }

        if(good_reads%100 == 0)
        {
            cout<<"good .. errors ... success % .... "<<good_reads <<" ... "<< total_errors <<" ... "<<(double)(good_reads / (total_errors+good_reads))*100<<endl;
        }

        loop_rate.sleep();
    }

    serial_close(pi, serHandle);
    pigpio_stop(pi);

    if(consecutive_errors >=200)
    {
        cout<<"************************************************************"<<endl;
        cout<<"*     TOO MANY CONSECUTIVE FAILED READS. ABORTING NODE     *"<<endl;
        cout<<"************************************************************"<<endl;
        return -1;
    }

    return 0;
}
