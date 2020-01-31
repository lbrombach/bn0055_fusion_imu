/*************************************************************
** bn0055_base_functions.cpp
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
#include "bn0055_fusion_imu/bn0055.h"

using namespace std;

void clear_buffer(int pi, int serHandle)
{
    char outBuf[4]={START, READ, OPR_MODE, 1};
    char inBuf[130] = {0};
    serial_write(pi, serHandle, outBuf, sizeof(outBuf));
    time_sleep(.2);
    serial_read(pi, serHandle, inBuf, 130);
    time_sleep(.2);

    cout<<endl<<"**********clearing buffer**********"<<endl;
    for (int i=0; i<130; i++)
    {
        cout<<(int)inBuf[i]<<" . ";
        if(i%10 == 0)
            cout<<endl;
    }
    cout<<endl<<"*********fin***********"<<endl;

}

//returns requested registers, concatenates bytes,
//but does not process to signed int or units
int get_byte(int pi, int serHandle, int regLSB)
{
    //slow down 2nd requests
    time_sleep(.05);
    char outBuf[4]={START, READ, (char)regLSB, 1};
    char inBuf[4] = {0};

    serial_write(pi, serHandle, outBuf, sizeof(outBuf));
    time_sleep(.05);
    serial_read(pi, serHandle, inBuf, 4);
    time_sleep(.05);

    //successful response should be [responseByte] [msgLength] [data]
    //read success response byte = 0xBB (187)
    //error response will be [0xEE] [error code] (0xEE = 238)
    if(inBuf[0] == SUCCESS_CODE)
    {
        return inBuf[2];
    }
    //it seems that at times, the response packet ends up shifted one byte
    else if(inBuf[1] == SUCCESS_CODE)
    {
        return inBuf[3];
    }
    else //error -check code and return 1111 as error code
    {
        write_success(inBuf[1]);
        return 1111;
    }
}

//retrieves AND combines two bytes to single SIGNED int values
bool get_bytes(int pi, int serHandle, int regLSB, int numBytes, int data[])
{
    char outBuf[4]={START, READ, (char)regLSB, (char)numBytes};
    char inBuf[numBytes+6] = {0};
    serial_write(pi, serHandle, outBuf, 4);
    time_sleep(.02);
    serial_read(pi, serHandle, inBuf, numBytes+2);
    time_sleep(.02);

    //successful response should be [responseByte] [msgLength] [data]
    //read success response byte = 0xBB (187)
    //error response will be [0xEE] [error code] (0xEE = 238)
    if(inBuf[0] == SUCCESS_CODE)
    {
        for(int i=0; i<numBytes/2; i++)
        {
          data[i] = SignedToInt(inBuf[i*2+2], inBuf[i*2+3]);
        }
        return true;
    }
    else //error -check code and return 1111 as error code
    {
        write_success(inBuf[1]);
        return false;
    }
}

bool set_byte(int pi, int serHandle, int regLSB, int val)
{
    char buf[5] = {0};
    char cmdBuf[5]={0xAA, 0x00, (char)regLSB, 1, (char)val};

    //write cmdBuf to bn0055
    serial_write(pi, serHandle, cmdBuf, 5);
    time_sleep(.2);

    //read response. should be [startByte] [response code]
    (serial_read(pi, serHandle, buf, 5));
    time_sleep(.2);
    int response = buf[1];

    //check respose code
    if (write_success(response))
    {
        return true;
    }
    else
    {
        cout<<"setbyte() failed. here are the 5 values in the return buf: ";
        for(int i = 0; i<5; i++)
        {
            cout<<(int)buf[i]<<" ";
        }
        cout<<endl;
        return false;
    }
}


bool write_success(int response)
{
    switch (response)
    {
      case 1: //write success
      return true;
      break;

      case 2:
      cout<<"Error 2 - Read Fail"<<endl;
      break;

      case 3:
      cout<<"Error 3 - Write Fail"<<endl;
      break;

      case 4:
      cout<<"Error 4 - Regmap invalid address"<<endl;

      break;

      case 5:
      cout<<"Error 5 - Regmap write disabled"<<endl;
      break;

      case 6:
      cout<<"Error 6 - Wrong start byte"<<endl;
      break;

      case 7:
      cout<<"Error 7 - Bus overrun error"<<endl;
      break;

      case 8:
      cout<<"Error 8 - Max length error"<<endl;
      break;

      case 9:
      cout<<"Error 9 - Min length error"<<endl;
      break;

      case 10:
      cout<<"Error 10 - Receive character timeout"<<endl;
      break;

      default:
      cout<<"Default case - I don't know what the following error is : "<<response<<endl;
    }
    return false;
}

bool reset_bn0055(int pi, int serHandle)
{
    set_config_mode(pi, serHandle);
    time_sleep(1);
    return set_byte(pi, serHandle, SYS_TRIGGER, 0x20);
}

bool set_config_mode(int pi, int serHandle)
{
    if(!set_byte(pi, serHandle, OPR_MODE, CONFIG_MODE))
    {
        return set_byte(pi, serHandle, OPR_MODE, CONFIG_MODE);
    }
    return true;
}


bool set_opr_mode_8(int pi, int serHandle)
{
    if(!set_byte(pi, serHandle, OPR_MODE, IMU_MODE) )
    {
    return set_byte(pi, serHandle, OPR_MODE, IMU_MODE);
    }
    return true;
}


bool set_my_units(int pi, int serHandle)
{
    set_config_mode(pi, serHandle);
    bool changed = set_byte(pi, serHandle, UNIT_SEL, MY_UNITS);
    if(changed == false){changed = set_byte(pi, serHandle, UNIT_SEL, MY_UNITS);}
    set_opr_mode_8(pi, serHandle);
    return changed;
}


int get_pwr_mode(int pi, int serHandle)
{
    int mode = get_byte(pi, serHandle, PWR_MODE);
    if (mode==1111)  {mode = get_byte(pi, serHandle, PWR_MODE);}
    if (mode == 0)   {cout<<"Power Mode = Normal"<<endl;}
    if (mode == 1)   {cout<<"Power Mode = Low Power"<<endl;}
    if (mode == 2)   {cout<<"Power Mode = Supend"<<endl;}
    if (mode < 0 || mode >2)  {cout<<"UNKNOWN MODE"<<endl;}
    return mode;
}

int get_axis_map(int pi, int serHandle)
{
    int mapConfig = get_byte(pi, serHandle, AXIS_MAP_CONFIG);
    if (mapConfig == 1111){mapConfig = get_byte(pi, serHandle, AXIS_MAP_CONFIG);}
    int mapSign = get_byte(pi, serHandle, AXIS_MAP_SIGN);
    if (mapSign == 1111){mapSign = get_byte(pi, serHandle, AXIS_MAP_SIGN);}


    if (mapConfig == AXIS_REMAP_CONFIG_VAL_270_CW && mapSign == AXIS_REMAP_SIGN_VAL_270_CW)
    { return 0; }

    if (mapConfig == AXIS_REMAP_CONFIG_VAL_DEFAULT && mapSign == AXIS_REMAP_SIGN_VAL_DEFAULT)
    { return 1; }

    if (mapConfig == AXIS_REMAP_CONFIG_VAL_180 && mapSign == AXIS_REMAP_SIGN_VAL_180)
    { return 2; }

    if (mapConfig == AXIS_REMAP_CONFIG_VAL_90_CW && mapSign == AXIS_REMAP_SIGN_VAL_90_CW)
    { return 3; }

    cout<<endl<<"**** Unable to read axis orientation map ****"<<endl;
    cout<<"raw bytes out = "<<(int)mapConfig <<" .. "<< (int)mapSign<<endl;
    return 1111;
}

bool set_axis_remap(int pi, int serHandle, int axisMapConfig)
{
    int mapConfig, mapSign;
    switch(axisMapConfig)
    {
        case 0:
        mapConfig = AXIS_REMAP_CONFIG_VAL_270_CW;
        mapSign = AXIS_REMAP_SIGN_VAL_270_CW;
        break;

        case 1:
        mapConfig = AXIS_REMAP_CONFIG_VAL_DEFAULT;
        mapSign = AXIS_REMAP_SIGN_VAL_DEFAULT;
        break;

        case 2:
        mapConfig = AXIS_REMAP_CONFIG_VAL_180;
        mapSign = AXIS_REMAP_SIGN_VAL_180;
        break;

        case 3:
        mapConfig = AXIS_REMAP_CONFIG_VAL_90_CW;
        mapSign = AXIS_REMAP_SIGN_VAL_90_CW;
        break;

        default:
        cout<<"**INVALID AXIS MAP CONFIG. VERIFY OK AXIS MAP BEFORE USING IMU**"<<endl;
        return false;
    }

    if(!set_byte(pi, serHandle, AXIS_MAP_CONFIG, mapConfig) )
    {
        if(!set_byte(pi, serHandle, AXIS_MAP_CONFIG, mapConfig))
        {
            return false;
        }
    }

    if(!set_byte(pi, serHandle, AXIS_MAP_SIGN, mapSign) )
    {
        if(!set_byte(pi, serHandle, AXIS_MAP_SIGN, mapSign) )
        {
            return false;
        }
    }

    return true;
}

int get_opr_mode(int pi, int serHandle)
{
    int mode = get_byte(pi, serHandle, OPR_MODE);
    if (mode==1111)  {mode = get_byte(pi, serHandle, OPR_MODE);}
    if (mode == 0)   {cout<<"Mode = Config mode"<<endl;}
    if (mode == 1)   {cout<<"Mode = Accel only"<<endl;}
    if (mode == 2)   {cout<<"Mode = Mag only  "<<endl;}
    if (mode == 3)   {cout<<"Mode = Gyro only"<<endl;}
    if (mode == 4)   {cout<<"Mode = AccelMag"<<endl;}
    if (mode == 5)   {cout<<"Mode = AccelGyro"<<endl;}
    if (mode == 6)   {cout<<"Mode = MagGyro"<<endl;}
    if (mode == 7)   {cout<<"Mode = AccelMagGyro"<<endl;}
    if (mode == 8)   {cout<<"Mode = AccelGyro Fusion"<<endl;}
    if (mode == 9)   {cout<<"Mode = Compass Fusion"<<endl;}
    if (mode == 10)  {cout<<"Mode = M4G Fusion"<<endl;}
    if (mode == 11)  {cout<<"Mode = NDOF FMC OFF Fusion"<<endl;}
    if (mode == 12)  {cout<<"Mode = NDOF Fusion"<<endl;}
    if (mode < 0 || mode >12)  {cout<<"Mode = Error checking mode - UNKNOWN MODE or bad read"<<endl;}
    return mode;
}

int get_units(int pi, int serHandle)
{
    int units = get_byte(pi, serHandle, UNIT_SEL);
    if(units == 1111)
    {
        units = get_byte(pi, serHandle, UNIT_SEL);
        if(units == 1111)
        {
        cout<<"bad read"<<endl;
        return units;
        }
    }
    cout<<"Unit selection bits 0-1 values :"<<endl<<" orient xxx xxx temps xxx angUnit angRate accel"<<endl
    <<((units & 128) == 128)<<((units & 64) == 64)<<((units & 32) == 32)<<((units & 16) == 16)
    <<((units & 0x08) == 8)<<((units & 0x04) == 4)<<((units & 0x02) == 2)<<((units & 0x01) == 1)<<endl;

    if ((units & 128) == 128)  {cout<<"Orientation mode   = Android"<<endl;}
    else                       {cout<<"Orientation mode   = Windows"<<endl;}

    if ((units & 16) == 16)    {cout<<"Temp units         = F"<<endl;}
    else                       {cout<<"Temp units         = C"<<endl;}

    if ((units & 4) == 4)      {cout<<"Euler angle Units  = Rad"<<endl;}
    else                       {cout<<"Euler angle Units  = Deg"<<endl;}

    if ((units & 2) == 2)      {cout<<"Angular rate units = Rad/sec"<<endl;}
    else                       {cout<<"Angular rate units = Deg/sec"<<endl;}

    if ((units & 1) == 1)      {cout<<"Acceleration units = mg   "<<endl;}
    else                       {cout<<"Acceleration units = m/s^2"<<endl;}

    return units;
}

int get_system_status(int pi, int serHandle)
{
    int status = get_byte(pi, serHandle, SYS_STATUS);
    switch (status)
    {
        case 0:
        cout<<"System status 0 - System idle"<<endl;
        return status;
        break;

        case 1:
        {
        cout<<"System status 1 - System Error. Reading error code now."<<endl;
        int error = get_byte(pi, serHandle, SYS_STATUS);
        switch (error)
        {
            case 0:
            cout<<"No error"<<endl;
            break;

            case 1:
            cout<<"Peripheral initialization error"<<endl;
            break;

            case 2:
            cout<<"Error 2 - System initialization error"<<endl;
            break;

            case 3:
            cout<<"Error 3 - Self test result failed"<<endl;
            break;

            case 4:
            cout<<"Error 4 - Register map value out of range"<<endl;
            break;

            case 5:
            cout<<"Error 5 - Register map address out of range"<<endl;
            break;

            case 6:
            cout<<"Error 6 - Register map write error"<<endl;
            break;

            case 7:
            cout<<"Error 7 - BNO low power mode not available for selected operation mode"<<endl;
            break;

            case 8:
            cout<<"Error 8 - Accelerometer power mode not available"<<endl;
            break;

            case 9:
            cout<<"Error 9 - Fusion algorithm configuration error"<<endl;
            break;

            case 10:
            cout<<"Error 10 - Sensor configuration error"<<endl;
            break;

            default:
            cout<<"Default case - I don't know what the following error is : "<<error<<endl;
        }
        return status;
        break;
        }

        case 2:
        cout<<"System status 2 - Initializing peripherals"<<endl;
        return status;
        break;

        case 3:
        cout<<"System status 3 - System Initialization"<<endl;
        return status;
        break;

        case 4:
        cout<<"System status 4 - Executing selftest"<<endl;
        return status;
        break;

        case 5:
        cout<<"System status 5 - Sensor fusion algorithm running"<<endl;
        return status;
        break;

        case 6:
        cout<<"System status 6 - Sensor fusion algorithm running"<<endl;
        return status;
        break;

        default:
        cout<<"Default switch - something weird with unknown status "<<status<<endl;
        return status;
    }
}

bool get_st_result(int pi, int serHandle)
{
    int result = get_byte(pi, serHandle, ST_RESULT);
    if(result == 1111){result = get_byte(pi, serHandle, ST_RESULT);}
    if(result==15)
    {
        cout<<"self test - all units passed "<<endl;
        return true;
    }
    else
    {
        if ((result & 8) != 8){cout<<"MCU self test failed"<<endl;}
        if ((result & 4) != 4){cout<<"Gyro self test failed"<<endl;}
        if ((result & 2) != 2){cout<<"Magnetometer self test failed"<<endl;}
        if ((result & 1) != 1){cout<<"Accelerometer self test failed"<<endl;}
        return false;
    }
}


int get_calibration_status(int pi, int serHandle, bool autoMode)
{
/*
4 statuses: 0-3 where 0 not calibrated and 3 fully calibrated
bits 6&7 = sys cal (MSb first)
bits 4&5 = gyro cal
bits 2&3 = accelerometr cal
bits 0&1 = magnetometer cal
for mode 8 (basic fusion imu mode) a value of 60 (00111100) is required
and indicates that gyro and accel are fully calibrated
*/
    int status = get_byte(pi, serHandle, CALIB_STAT);
    if(status == 1111){status = get_byte(pi, serHandle, CALIB_STAT);}
    if(!autoMode)
    {
        cout<<"calibration bits 0-3 values : sys sys gyr gyr acc acc mag mag"<<endl
            <<((status & 128) == 128)<<((status & 64) == 64)<<((status & 32) == 32)<<((status & 16) == 16)
            <<((status & 0x08) == 8)<<((status & 0x04) == 4)<<((status & 0x02) == 2)<<((status & 0x01) == 1)<<endl;
    }

    if(status == CALIB_REQUIRED_FOR_IMU_MODE)
    {
        if(!autoMode)
        {
            cout<<"gyro and accel fully calibrated, good to go for IMU fusion mode "<<endl;
        }
        return status;
    }
    else
    {
        if(!autoMode)
        {
            cout<<"calibration required. current calibration status "<<status<<endl
            <<"(00111100 aka 60 means gyro and accel are fuly calibrated"<<endl;
        }
        return status;
    }
}


bool confirm()
{
    char n=-1;
    while(n!='Y' && n != 'N')
    {
    if(cin.fail())
    {
        cin.clear();
        cin.ignore(50, '\n');
    }
    cout<<"Enter y to continue or n for no"<<endl;
    cin >> n;
    n = toupper(n);
    }
    return n == 'Y';
}


// Helper function for converting 1 or 2 chars into int (unsigned).
int UnsignedToInt(uint8_t low, uint8_t high = 0)
{
return ((int)high << 8) + (int)low;
}
// Helper function for converting 2 chars into int (signed).
int SignedToInt(uint8_t low, int8_t high)
{
return ((int)high << 8) + (int)low;
}
