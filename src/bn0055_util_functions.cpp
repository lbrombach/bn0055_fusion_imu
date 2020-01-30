/*************************************************************
** bn0055_util_functions.cpp
** support functions for bn0055_fusion_imu package,
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

#include <iostream>
#include <pigpiod_if2.h>
#include <fstream>
#include "bn0055_fusion_imu/bn0055.h"


using namespace std;


bool get_imu_data(int pi, int serHandle, sensor_msgs::Imu &imu)
{
    //requesting 13 values
    int data[13] = {0};

    //two bytes per data[] element = request 26 bytes,
    //and try again if error
    int calStat = get_calibration_status(pi, serHandle, true);
    if(get_bytes(pi, serHandle, GYR_DATA_X_LSB, 26, data) == false
        || calStat != CALIB_REQUIRED_FOR_IMU_MODE)
    {
        cout<<"********ERROR**RETRYING********  "<<endl;
        time_sleep(.025);
        if(calStat != CALIB_REQUIRED_FOR_IMU_MODE)
        {
            initialize_bn0055(pi, serHandle);
            calStat = get_calibration_status(pi, serHandle, true);
        }
        if(get_bytes(pi, serHandle, GYR_DATA_X_LSB, 2, data) == false
        || calStat != CALIB_REQUIRED_FOR_IMU_MODE)
        {
            cout<<"*******FAILURE THIS ROUND ***********"<<endl;
            imu.orientation.w = -1;
            imu.orientation.x = -1;
            imu.orientation.y = -1;
            imu.orientation.z = -1;
            imu.linear_acceleration_covariance[0]=-1;
            imu.orientation_covariance[0] = -1;
            imu.angular_velocity_covariance[0] = -1;
        }
        return false;
    }
    else
    {
        // divide angular data by 900 for rads/sec
        imu.angular_velocity.x = (double)data[0]/900;
        imu.angular_velocity.y = (double)data[1]/900;
        imu.angular_velocity.z = (double)data[2]/900;
        //divide quaternion data by 2^14
        imu.orientation.w = (double)data[6]/16384;
        imu.orientation.x = (double)data[7]/16384;
        imu.orientation.y = (double)data[8]/16384;
        imu.orientation.z = (double)data[9]/16384;
        //divide acceleration data by 100 for m/s^2
        imu.linear_acceleration.x = (double)data[10]/100;
        imu.linear_acceleration.y = (double)data[11]/100;
        imu.linear_acceleration.y = (double)data[12]/100;

        imu.linear_acceleration_covariance[0]=.0001;
        imu.orientation_covariance[0] = .000001;
        imu.angular_velocity_covariance[0] = .0001;

        imu.header.stamp = ros::Time::now();
        return true;
    }
}


double get_heading(int pi, int serHandle)
{
    int data[3];

    //try again if error
    if(!get_bytes(pi, serHandle, EUL_Heading_LSB, 2, data))
    {
        time_sleep(.05);
        if(!get_bytes(pi, serHandle, EUL_Heading_LSB, 2, data))
        {   //still error, return error code
            return 1111;
        }
    }
    else
    {
        // divide val by 900 for rads mode
        double rads=(double)(data[0])/900;
        //convert from 0 to 6.28 rad to -3.14 to +3.14
        rads = (rads > PI) ?  (2*PI)-rads : (0-rads);
        return rads;
    }
}

void get_accel(int pi, int serHandle)
{
    int data[3]={0};
    get_bytes(pi, serHandle, ACC_DATA_X_LSB, 6, data);
    cout<<endl<< "Accel x, y, z = "<<(double)data[0] /100<<" "<<(double)data[1]/100<<" "<<(double)data[2]/100<<" m/s^2 = "<<endl;
}

bool rotate_axis_map(int pi, int serHandle)
{
    set_config_mode(pi, serHandle);
    int choice = -1;
    while(choice < 0 || choice > 3)
    {
        if(cin.fail())
        {
            cin.clear();
            cin.ignore(200, '\n');
        }

        cout<<endl<<"Choose sensor orientation:"<<endl
        <<"0. Sensor mounted 270 degrees CW from default"<<endl
        <<"1. Default mounting"<<endl
        <<"2. Sensor mounted 180 degrees from default"<<endl
        <<"3. Sensor mounted 90 degrees CW from default"<<endl;
        cin>>choice;
    }

    if (!set_axis_remap(pi, serHandle, choice) && get_axis_map(pi, serHandle != choice))
    {
        time_sleep(.25);
        if(!set_axis_remap(pi, serHandle, choice))
        {
            cout<<"***WARNING: UNABLE TO SET OR VERIFY AXIS MAP CONFIG!***"<<endl;
            return false;
        }
    }

    cout<<"Axis successfully remapped"<<endl;
    time_sleep(.25);
    set_opr_mode_8(pi, serHandle);
    return true;
}



bool verify_accel_gyro_fusion_config(int pi, int serHandle)
{
    //int pwr = get_pwr_mode(pi, serHandle);

    int opr = get_opr_mode(pi, serHandle);
    if(opr != IMU_MODE)
    {
        set_opr_mode_8(pi, serHandle);
        opr = get_opr_mode(pi, serHandle);
    }

    int units = get_units(pi, serHandle);
    if(units != MY_UNITS)
    {
        set_my_units(pi, serHandle);
        units = get_units(pi, serHandle);
    }

    if(opr != IMU_MODE || units != MY_UNITS)
    {
        cout<<"unable to set mode to accel/gyro fusion or unable to set units."<<endl;
        return false;
    }

    int calStat = get_calibration_status(pi, serHandle);
    if(calStat != CALIB_REQUIRED_FOR_IMU_MODE)
    {
        cout<<"accel/gyro fusion mode requires calibration status of 60. Please calibrate and try again"<<endl;
        return false;
    }

    cout<<"opr mode, units, and calibration ok. IMU Ready."<<endl;
    return true;
}

void show_current_config(int pi, int serHandle)
{
    cout<<endl<<"#########################################################"<<endl
    <<"CURRENT CONFIG DATA AS FOLLOWS:"<<endl<<endl;
    get_opr_mode(pi, serHandle);
    cout<<endl;
    get_units(pi, serHandle);
    cout<<endl;
    get_calibration_status(pi, serHandle);
    cout<<endl;
    get_pwr_mode(pi, serHandle);
    cout<<endl;

    switch (get_axis_map(pi, serHandle))
    {
        case 0:
        cout<<endl<<"Axis orientation map is 270 CW from default"<<endl;
        break;

        case 1:
        cout<<endl<<"Axis orientation map is default"<<endl;
        break;

        case 2:
        cout<<endl<<"Axis orientation map is 180 degrees from default"<<endl;
        break;

        case 3:
        cout<<endl<<"Axis orientation map is 90 degrees CW from default"<<endl;
        break;

        default:
        cout<<endl<<"****UNABLE TO DETERMINE AXIS MAP CONFIGURATION****"<<endl;
    }


    cout<<"#########################################################"<<endl<<endl;
}

void check_for_file()
{
    ifstream inFile;
    inFile.open(DEFAULT_FILE_PATH.c_str());
    if(inFile.is_open() == false )
    {
        cout<<"************************************************************"<<endl
            <<"WARNING: CONFIG FILE NOT FOUND AT DEFAULT FILE PATH         "<<endl
            <<"Highly recommend checking DEFAULT_FILE_PATH defined in bn0055.h"<<endl
            <<"When correct, run the calibration and save routines. Got it? "<<endl;
        confirm();
    }
    inFile.close();
}

bool calibrate_bn0055(int pi, int serHandle)
{
    if(get_opr_mode(pi, serHandle) != IMU_MODE)
    {
        cout<<"opr mode must be in imu fusion mode (mode 8) to calibrate"<<endl
            <<"trying to set mode..."<<endl;
        if(!set_opr_mode_8(pi, serHandle) && get_opr_mode(pi, serHandle) != IMU_MODE)
        {
            cout<<"unable to set mode. Aborting."<<endl;
            return false;
        }
    }

    cout<<"."<<endl<<"."<<endl<<"."<<endl
        <<"Place the sensor in at least 6 different orientations"<<endl
        <<"Move slowly between positions and pause at each for a few seconds"<<endl
        <<"Make sure device is perpendicular to each axis at least once"<<endl
        <<"The following loop will stop when calibration successful OR timeout"<<endl
        <<"You have 120 seconds. confirm when ready (y to continue or n to cancel): "<<endl;

    if(!confirm())
    {
        cout<<"cancelled by request"<<endl;
        return -1;
    }

    cout<<endl<<"GO!"<<endl;
    time_sleep(2);


    int status = -1;
    double startTime = ros::Time::now().toSec();

    while(status != CALIB_REQUIRED_FOR_IMU_MODE && ros::Time::now().toSec() - startTime < 120)
    {
        time_sleep(.25);
        cout<<"***** Current status *****"<<endl;
        status = get_calibration_status(pi, serHandle);
        cout<<endl<<endl;

    }
    return true;
}

bool initialize_bn0055(int pi, int serHandle, bool autoMode)
{
    string path = DEFAULT_FILE_PATH;

    if(!autoMode)
    {
        cout<<endl<<"Current path and file to read from: "<<path
            <<"\n is this the correct?  ";

        while(confirm()==false)
        {
            if(cin.fail())
            {
                cin.clear();
                cin.ignore(50, '\n');
            }
            cout<<endl<<"enter path (without filename) to save to: (please include leading and trialing '/'"<<endl;
            cin>>path;
            cout<<endl<<"Current path and file to retrieve calibration data: "<<path
                <<"\nIs this ok?  ";
        }
    }
    ifstream inFile;
    inFile.open(path.c_str());
    int opr, units, calStat, axisMapConfig;
    char calData[27]={0};

    if(inFile.is_open() )
    {
        //disregard header - ignore all text until '#' flag, then ignore that line too
        while(inFile.peek()!='#')
        {
            inFile.ignore();
        }
        inFile.ignore(50, '\n');

        //read current opr mode, units, and calibration status at time of last save
        inFile>>opr>>units>>calStat>>axisMapConfig;
        cout<<endl<<"opr mode = "<<opr<<"  units = "<<units<< "  calStat = "<<calStat<<"  axis map config = "<<axisMapConfig<<endl;

        //read calibration offsets and radii into buf to write, leaving room for header bytes
        for(int i=0; i<22; i++)
        {
            inFile>>calData[i+4];
            cout<<(int)calData[i]<<" ";
        }

        inFile.close();
    }
    else
    {
        cout<<"Unable to open "<<path<<" ***INITIALIZE FAILED***"<<endl;
        return false;
    }

    cout<<endl<<"File read complete, trying to write data"<<endl;
    set_config_mode(pi, serHandle);


    if (!set_axis_remap(pi, serHandle, axisMapConfig) && get_axis_map(pi, serHandle != axisMapConfig))
    {
        time_sleep(.25);
        if(!set_axis_remap(pi, serHandle, axisMapConfig))
        {
            cout<<"***WARNING: UNABLE TO SET OR VERIFY AXIS MAP CONFIG!***"<<endl;
        }
    }

    //set write header bytes
    calData[0]=0xAA;
    calData[1]=0x00;
    calData[2]=ACC_OFFSET_X_LSB;
    calData[3]= 22;
    char buf[2] = {0};

    //try twice to write the offset data
    serial_write(pi, serHandle, calData, 26);
    time_sleep(.5);
    (serial_read(pi, serHandle, buf, 2));
    int response = buf[1];
    if (!write_success(response))
    {
        serial_write(pi, serHandle, calData, 26);
        time_sleep(.5);
        (serial_read(pi, serHandle, buf, 2));
        int response = buf[1];
        if(!write_success(response))
        {
            cout<<"**UNABLE TO WRITE TO CALIBRATION REGISTERS**"<<endl;
            return false;
        }
    }
    cout<<"Writing to calibration registers successful"<<endl;

    if(!verify_accel_gyro_fusion_config(pi, serHandle))
    {
        cout<<"************************************************************"<<endl;
        cout<<"*       Unable to verify or set BN0055 IMU config.         *"<<endl;
        cout<<"*       Verify config before trying to run node            *"<<endl;
        cout<<"************************************************************"<<endl;
        if(autoMode)
        {
            return false;
        }
        else
        {
            confirm();
        }
    }

    cout<<endl<<"Initialization successful"<<endl<<endl;
    return true;
}

//
//bool auto_initialize_bn0055(int pi, int serHandle)
//{
//    string path = DEFAULT_FILE_PATH;
//
//    ifstream inFile;
//    inFile.open(path.c_str());
//    int opr, units, calStat, axisMapConfig;
//    char calData[27]={0};
//
//    if(inFile.is_open() )
//    {
//        //disregard header - ignore all text until '#' flag, then ignore that line too
//        while(inFile.peek()!='#')
//        {
//            inFile.ignore();
//        }
//        inFile.ignore(50, '\n');
//
//        //read current opr mode, units, and calibration status at time of last save
//        inFile>>opr>>units>>calStat>>axisMapConfig;
//        cout<<endl<<"opr mode = "<<opr<<"  units = "<<units<< "  calStat = "<<calStat<<"  axis map config = "<<axisMapConfig<<endl;
//
//        //read calibration offsets and radii into buf to write, leaving room for header bytes
//        for(int i=0; i<22; i++)
//        {
//            inFile>>calData[i+4];
//            cout<<(int)calData[i]<<" ";
//        }
//
//        inFile.close();
//    }
//    else
//    {
//        cout<<"Unable to open "<<path<<" ***INITIALIZE FAILED***"<<endl;
//        return false;
//    }
//
//    cout<<endl<<"File read complete, trying to write data"<<endl;
//    set_config_mode(pi, serHandle);
//
//
//    if (!set_axis_remap(pi, serHandle, axisMapConfig) && get_axis_map(pi, serHandle != axisMapConfig))
//    {
//        time_sleep(.25);
//        if(!set_axis_remap(pi, serHandle, axisMapConfig))
//        {
//            cout<<"***WARNING: UNABLE TO SET OR VERIFY AXIS MAP CONFIG!***"<<endl;
//        }
//    }
//
//    //set write header bytes
//    calData[0]=0xAA;
//    calData[1]=0x00;
//    calData[2]=ACC_OFFSET_X_LSB;
//    calData[3]= 22;
//    char buf[2] = {0};
//
//    //try twice to write the offset data
//    serial_write(pi, serHandle, calData, 26);
//    time_sleep(.5);
//    (serial_read(pi, serHandle, buf, 2));
//    int response = buf[1];
//    if (!write_success(response))
//    {
//        serial_write(pi, serHandle, calData, 26);
//        time_sleep(.5);
//        (serial_read(pi, serHandle, buf, 2));
//        int response = buf[1];
//        if(!write_success(response))
//        {
//            cout<<"**UNABLE TO WRITE TO CALIBRATION REGISTERS**"<<endl;
//            return false;
//        }
//    }
//    cout<<"Writing to calibration registers successful"<<endl;
//
//    if(!verify_accel_gyro_fusion_config(pi, serHandle))
//    {
//        cout<<"************************************************************"<<endl;
//        cout<<"*       Unable to verify or set BN0055 IMU config.         *"<<endl;
//        cout<<"*       Verify config before trying to run node            *"<<endl;
//        cout<<"************************************************************"<<endl;
//        confirm();
//    }
//
//    cout<<endl<<"Initialization successful"<<endl<<endl;
//    return true;
//}



bool save_config(int pi, int serHandle)
{
    if(!verify_accel_gyro_fusion_config(pi, serHandle))
    {
        cout<<"************************************************************"<<endl;
        cout<<"*  Unable to verify or set BN0055 IMU config.  Aborting.    *"<<endl;
        cout<<"************************************************************"<<endl;
        return -1;
    }

    cout<<"opr mode, units, and calibration ok to proceed. Getting sensor offsets and radii"<<endl;
    set_config_mode(pi, serHandle);

    //get all 22 sensor offsets and radii values starting at reg 0x55
    int numBytes = 22;
    char outBuf[4]={START, READ, ACC_OFFSET_X_LSB, (char)numBytes};
    char inBuf[numBytes+2] = {0};
    serial_write(pi, serHandle, outBuf, 4);
    time_sleep(.5);
    serial_read(pi, serHandle, inBuf, numBytes+2);
    //successful response should be [responseByte] [msgLength] [data]
    //read success response byte = 0xBB (187)
    //error response will be [0xEE] [error code] (0xEE = 238)
    if(inBuf[0] == SUCCESS_CODE)
    {
        for(int i=0; i<numBytes; i++)
        {
          cout<<(int)inBuf[i]<<"  ";
        }
        cout<<endl<<endl;
    }
    else //error
    {
        cout<<"bad read trying again"<<endl;
        time_sleep(.5);
        serial_write(pi, serHandle, outBuf, 4);
        time_sleep(.5);
        serial_read(pi, serHandle, inBuf, numBytes+2);

        if(inBuf[0] == SUCCESS_CODE)
        {
            for(int i=0; i<numBytes; i++)
            {
                cout<<(int)inBuf[i]<<"  ";
            }
            cout<<endl<<endl;
        }
    }
    if(inBuf[0] != SUCCESS_CODE)
    {
        cout<<"cannot read offset registers. Aborting."<<endl;
        return false;
    }

    set_opr_mode_8(pi, serHandle);
    if(get_opr_mode(pi, serHandle) != IMU_MODE)
    {
        set_opr_mode_8(pi, serHandle);
    }
    if(get_opr_mode(pi, serHandle) != IMU_MODE)
    {
        cout<<"***WARNING - UNABLE TO RESET OPR MODE. "
            <<"\nCONTINUING DATA SAVE BUT DEVICE NOT OPERATING PROPERLY"<<endl
            <<"press y to continue"<<endl;
            confirm();
    }

    int axisMapConfig = get_axis_map(pi, serHandle);
    if(axisMapConfig < 0 || axisMapConfig > 3)
    {
        time_sleep(.2);
        axisMapConfig = get_axis_map(pi, serHandle);

        if(axisMapConfig < 0 || axisMapConfig > 3)
        {
            cout<<"************************************************************"<<endl;
            cout<<"*     Unable to retrieve axis map config.  Aborting.       *"<<endl;
            cout<<"************************************************************"<<endl;
            return -1;
        }
    }

    string path = DEFAULT_FILE_PATH;

    cout<<endl<<"Current path and file to save to: "<<path
        <<"\nIs this ok?  ";

    while(confirm()==false)
    {
        if(cin.fail())
        {
            cin.clear();
            cin.ignore(50, '\n');
        }
        cout<<endl<<"enter path and file to save to"<<endl;
        cin>>path;
        cout<<endl<<"Current path and file to save to: "<<path
            <<"\nIs this ok?  ";
    }
    ofstream outFile;
    outFile.open(path.c_str());
    if(outFile.is_open() )
    {
        outFile<<"bn0055 absolute orientation imu sensor calibration offset file"<<endl
                <<"Author: Lloyd Brombach 12/6/2019   github.com/lbrombach"<<endl
                <<"#####"<<endl
                <<IMU_MODE<<" "<<MY_UNITS<<" "<<CALIB_REQUIRED_FOR_IMU_MODE<<" "<<axisMapConfig<<" ";
                for(int i=0; i<numBytes; i++)
                {
                    outFile<<(int)inBuf[i]<<" ";
                }
        outFile.close();
        cout<<"config and calibration data saved"<<endl;
        return true;
    }
    else
    {
        cout<<endl<<"***SAVE FAILED***"<<endl;
        return false;
    }
}





















