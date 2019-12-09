/*************************************************************
** bn0055_setup_util_node.cpp
** ROS node that acts as a setup utility to calibrate, save,
** and reload calibration data from file.
** Set file path in const std::string DEFAULT_FILE_PATH in bn0055.h
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


int get_menu_choice()
{
    int choice = -1;
    while(choice < 0 || choice > 8)
    {
        if(cin.fail())
        {
            cin.clear();
            cin.ignore(200, '\n');
        }

        cout<<endl<<"BN0055 SETUP UTILITY MENU:"<<endl
        <<"1. Command Reset BN0055"<<endl
        <<"2. Command to accel/gyro fusion (IMU) mode"<<endl
        <<"3. Save current profile"<<endl
        <<"4. Load last saved profile"<<endl
        <<"5. Run calibration routine"<<endl
        <<"6. Print raw accel data"<<endl
        <<"7. Print fused euler heading"<<endl
        <<"8. Clear Buffer (try this if unit acting wonky. must be not be in config mode)"<<endl
        <<"0. Exit"<<endl;

        cin>>choice;
    }

    return choice;
}



int PigpioSetup()
{
  char *addrStr = NULL;
  char *portStr = NULL;
  return pigpio_start(addrStr, portStr);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bn0055_setup_util");
    ros::NodeHandle node;

    //initialize pipiod interface
    int pi = PigpioSetup();
    if(pi>=0)
    {
        cout<<"daemon interface started ok at "<<pi<<endl;
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

    check_for_file();

    if(get_opr_mode(pi, serHandle) != IMU_MODE)
    {
        set_opr_mode_8(pi, serHandle);
    }

    int choice = -1;
    while(choice != 0)
    {
        show_current_config(pi, serHandle);
        choice = get_menu_choice();

        switch(choice)
        {
            case 0:
            break;

            case 1:
            reset_bn0055(pi, serHandle);
            cout<<"Reset command sent. "<<endl
                <<"When reset complete, unit will be in config mode and won't be able to verify calibration"<<endl
                <<"Set to accel/gyro fusion mode before trying to calibrate or get readings"<<endl
                <<"Please wait a few seconds for reset to complete..."<<endl;
            time_sleep(3);
            cout<<"OK"<<endl;

            case 2:
            if(!set_opr_mode_8(pi, serHandle) && get_opr_mode(pi, serHandle != IMU_MODE))
            {
                cout<<"Unable to set opr mode"<<endl;
            }

            break;

            case 3:
            if(!save_config(pi, serHandle))
            {
                cout<<"WARNING - SAVE NOT SUCCESSFUL"<<endl;
            }
            break;

            case 4:
            if(!initialize_bn0055(pi, serHandle))
            {
                cout<<"LOADING PREVIOUS DATA NOT SUCCESSFUL"<<endl;
            }
            break;

            case 5:
            if(!calibrate_bn0055(pi, serHandle))
            {
                cout<<"CALIBRATION NOT SUCCESSFUL"<<endl;
            }
            break;

            case 6:
                 get_accel(pi, serHandle);
                 break;

            case 7:
                cout<<"heading = "<<get_heading(pi, serHandle);
                break;

            case 8:
                 clear_buffer(pi, serHandle);
                 break;

            default:
            cout<<"Invalid option"<<endl;
            choice = -1;
        }
    }




    serial_close(pi, serHandle);
    pigpio_stop(pi);
    return 0;
}
