#include <stdlib.h>
#include <ros/ros.h>

#include <robocontroller/SetRegister.h>
#include <robocontroller/Debug_00_23.h>
#include <robocontroller/Debug_50_53.h>
#include <robocontroller/Debug_200_220.h>
#include <robocontroller/Debug_250_259.h>
#include <robocontroller/Debug_60000_60019.h>

#include "rbctrliface.h"
#include "robotctrl.h"
#include "modbus_registers.h"

using namespace std;
using namespace robocontroller;

// >>>>> Global params
double debug_pub_freq = 10;
// <<<<< Global params

RbCtrlIface* rbCtrlIface= NULL; // RoboController Interface
RobotCtrl* rbCtrl = NULL; // Robot control functions

// >>>>> Serial port parameters
int boardIdx = 1;
string serialPort = "/dev/ttyUSB0";
int serialbaudrate = 57600;
string parity = "N";
int data_bit = 8;
int stop_bit = 1;
bool simulMode = false;
// <<<<< Serial port parameters

bool setRegister_callback( robocontroller::SetRegisterRequest& req,
                      robocontroller::SetRegisterResponse& resp )
{
    resp.ok = rbCtrl->setRegister( req.resister_idx, req.register_value );

    return resp.ok;
}

int main( int argc, char **argv)
{
    ros::init( argc, argv, "robocontroller_debug_node" );
    ros::NodeHandle nh;

    ROS_INFO_STREAM("RoboController debugger node");
    ROS_INFO_STREAM("============================");
    ROS_INFO_STREAM("- Use 'rqt'->'PlugIns'->'Topics'->'Topic Monitor' ");
    ROS_INFO_STREAM("  to visualize the values of the register of the RoboController");
    ROS_INFO_STREAM("- Use 'rqt'->'PlugIns'->'Services'->'Service Caller' ");
    ROS_INFO_STREAM("  to set the value of a register");

    // Parameters initialization
    //init_system( nh );

    // >>>>> Publishers for Debug messages
    ros::Publisher debug_00_23_pub = nh.advertise<robocontroller::Debug_00_23>( "/robocontroller/Debug_00_23", 100 );
    robocontroller::Debug_00_23 debug_msg_00_23;

    ros::Publisher debug_50_53_pub = nh.advertise<robocontroller::Debug_50_53>( "/robocontroller/Debug_50_53", 100 );
    robocontroller::Debug_50_53 debug_msg_50_53;

    ros::Publisher debug_200_220_pub = nh.advertise<robocontroller::Debug_200_220>( "/robocontroller/Debug_200_220", 100 );
    robocontroller::Debug_200_220 debug_msg_200_220;

    ros::Publisher debug_250_259_pub = nh.advertise<robocontroller::Debug_250_259>( "/robocontroller/Debug_250_259", 100 );
    robocontroller::Debug_250_259 debug_msg_250_259;

    ros::Publisher debug_60000_60019_pub = nh.advertise<robocontroller::Debug_60000_60019>( "/robocontroller/Debug_60000_60019", 100 );
    robocontroller::Debug_60000_60019 debug_msg_60000_60019;

    // Server Modbus registers
    ros::ServiceServer setRegister = nh.advertiseService( "/robocontroller/SetRegister", setRegister_callback );

    // >>>>> Interface to RoboController board
    rbCtrlIface = new RbCtrlIface( boardIdx, serialPort, serialbaudrate, parity.c_str()[0], data_bit, stop_bit, simulMode );

    if(!rbCtrlIface->isConnected())
    {
        ROS_FATAL_STREAM( "RoboController not found on port " << serialPort );
        return -1;
    }
    // <<<<< Interface to RoboController board

    // Robot Control functions
    rbCtrl = new RobotCtrl( &nh, rbCtrlIface );

    // RoboController publishes telemetry at 30hz
    ros::Rate rate( debug_pub_freq );

    int phase = 0;

    while( ros::ok() )
    {
        ros::spinOnce(); // Process pending callback

        vector< u_int16_t > data;

        if( phase==0 )
        {
            data = rbCtrl->getRegisters( 0, 24 );

            if(data.size()!= 24)
                continue;

            debug_msg_00_23.register_00 = data[0];
            debug_msg_00_23.register_01 = data[1];
            debug_msg_00_23.register_02 = data[2];
            debug_msg_00_23.register_03 = data[3];
            debug_msg_00_23.register_04 = data[4];
            debug_msg_00_23.register_05 = data[5];
            debug_msg_00_23.register_06 = data[6];
            debug_msg_00_23.register_07 = data[7];
            debug_msg_00_23.register_08 = data[8];
            debug_msg_00_23.register_09 = data[9];
            debug_msg_00_23.register_10 = data[10];
            debug_msg_00_23.register_11 = data[11];
            debug_msg_00_23.register_12 = data[12];
            debug_msg_00_23.register_13 = data[13];
            debug_msg_00_23.register_14 = data[14];
            debug_msg_00_23.register_15 = data[15];
            debug_msg_00_23.register_16 = data[16];
            debug_msg_00_23.register_17 = data[17];
            debug_msg_00_23.register_18 = data[18];
            debug_msg_00_23.register_19 = data[19];
            debug_msg_00_23.register_20 = data[20];
            debug_msg_00_23.register_21 = data[21];
            debug_msg_00_23.register_22 = data[22];
            debug_msg_00_23.register_23 = data[23];

            debug_00_23_pub.publish( debug_msg_00_23 );
        }
        else if( phase==1 )
        {
            data = rbCtrl->getRegisters( 50, 4 );

            if(data.size()!= 4)
                continue;

            debug_msg_50_53.register_50 = data[0];
            debug_msg_50_53.register_51 = data[1];
            debug_msg_50_53.register_52 = data[2];
            debug_msg_50_53.register_53 = data[3];

            debug_50_53_pub.publish( debug_msg_50_53 );
        }
        else if( phase==2 )
        {
            data = rbCtrl->getRegisters( 200, 21 );

            if(data.size()!= 21)
                continue;

            debug_msg_200_220.register_200 = data[0];
            debug_msg_200_220.register_201 = data[1];
            debug_msg_200_220.register_202 = data[2];
            debug_msg_200_220.register_203 = data[3];
            debug_msg_200_220.register_204 = data[4];
            debug_msg_200_220.register_205 = data[5];
            debug_msg_200_220.register_206 = data[6];
            debug_msg_200_220.register_207 = data[7];
            debug_msg_200_220.register_208 = data[8];
            debug_msg_200_220.register_209 = data[9];
            debug_msg_200_220.register_210 = data[10];
            debug_msg_200_220.register_211 = data[11];
            debug_msg_200_220.register_212 = data[12];
            debug_msg_200_220.register_213 = data[13];
            debug_msg_200_220.register_214 = data[14];
            debug_msg_200_220.register_215 = data[15];
            debug_msg_200_220.register_216 = data[16];
            debug_msg_200_220.register_217 = data[17];
            debug_msg_200_220.register_218 = data[18];
            debug_msg_200_220.register_219 = data[19];
            debug_msg_200_220.register_220 = data[20];

            debug_200_220_pub.publish( debug_msg_200_220 );
        }
        else if( phase==3 )
        {
            data = rbCtrl->getRegisters( 250, 10 );

            if(data.size()!= 10)
                continue;

            debug_msg_250_259.register_250 = data[0];
            debug_msg_250_259.register_251 = data[1];
            debug_msg_250_259.register_252 = data[2];
            debug_msg_250_259.register_253 = data[3];
            debug_msg_250_259.register_254 = data[4];
            debug_msg_250_259.register_255 = data[5];
            debug_msg_250_259.register_256 = data[6];
            debug_msg_250_259.register_257 = data[7];
            debug_msg_250_259.register_258 = data[8];
            debug_msg_250_259.register_259 = data[9];

            debug_250_259_pub.publish( debug_msg_250_259 );
        }
        else if( phase==4 )
        {
            data = rbCtrl->getRegisters( 60000, 20 );

            if(data.size()!= 20)
                continue;

            debug_msg_60000_60019.register_60000 = data[0];
            debug_msg_60000_60019.register_60001 = data[1];
            debug_msg_60000_60019.register_60002 = data[2];
            debug_msg_60000_60019.register_60003 = data[3];
            debug_msg_60000_60019.register_60004 = data[4];
            debug_msg_60000_60019.register_60005 = data[5];
            debug_msg_60000_60019.register_60006 = data[6];
            debug_msg_60000_60019.register_60007 = data[7];
            debug_msg_60000_60019.register_60008 = data[8];
            debug_msg_60000_60019.register_60009 = data[9];
            debug_msg_60000_60019.register_60010 = data[10];
            debug_msg_60000_60019.register_60011 = data[11];
            debug_msg_60000_60019.register_60012 = data[12];
            debug_msg_60000_60019.register_60013 = data[13];
            debug_msg_60000_60019.register_60014 = data[14];
            debug_msg_60000_60019.register_60015 = data[15];
            debug_msg_60000_60019.register_60016 = data[16];
            debug_msg_60000_60019.register_60017 = data[17];
            debug_msg_60000_60019.register_60018 = data[18];
            debug_msg_60000_60019.register_60019 = data[19];

            debug_60000_60019_pub.publish( debug_msg_60000_60019 );
        }

        phase = (++phase)%5;
        rate.sleep();
    }

    if(rbCtrlIface)
        delete rbCtrlIface;

    if(rbCtrl)
        delete rbCtrl;

    return 0;
}
