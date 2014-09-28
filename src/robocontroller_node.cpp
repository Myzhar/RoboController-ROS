// This node handle the communication with RoboController board

#include <stdlib.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <robocontroller/Pose.h>
#include <robocontroller/Debug.h>
#include <robocontroller/Telemetry.h>
#include <robocontroller/SetPID.h>
#include <robocontroller/GetPID.h>
#include <robocontroller/EnablePID.h>
#include <robocontroller/EnableCommWD.h>
#include <robocontroller/GetBoardStatus.h>
#include <robocontroller/SetBatteryCalib.h>
#include <robocontroller/EnableSaveToEeprom.h>

#include "rbctrliface.h"
#include "robotctrl.h"
#include "modbus_registers.h"

using namespace std;
using namespace robocontroller;

ros::Time last_vel_cmd_time; // Time of the last velocity comand received

// >>>>> Global params
double telem_freq = 30;
double vel_cmd_timeout_sec = 0.5; // Timeout for motor stop if non velocity command is received
bool speed_filter_enabled = false;
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

// >>>>> Robot Params
RobotConfiguration rbConf;
// <<<<< Robot Params

// >>>>> Functions
void test_connection();
void init_system( ros::NodeHandle& nh );
// <<<<< Functions

// >>>>> Callbacks
// Messages
void vel_cmd_callback( const geometry_msgs::Twist& msg );

// Services
bool setPid_callback( robocontroller::SetPIDRequest &req, robocontroller::SetPIDResponse &resp );
bool getPid_callback( robocontroller::GetPIDRequest &req, robocontroller::GetPIDResponse &resp );
bool enablePID_callback( robocontroller::EnablePIDRequest& req, robocontroller::EnablePIDResponse& resp );
bool enableCommWD_callback( robocontroller::EnableCommWDRequest& req, robocontroller::EnableCommWDResponse& resp );
bool getBoardStatus_callback( robocontroller::GetBoardStatusRequest& req, robocontroller::GetBoardStatusResponse&resp );
bool setBattCalib_callback( robocontroller::SetBatteryCalibRequest& req, robocontroller::SetBatteryCalibResponse& resp );
bool enableSaveToEeprom_callback( robocontroller::EnableSaveToEepromRequest& req, robocontroller::EnableSaveToEepromResponse& resp );
// <<<<< Callbacks

void vel_cmd_callback( const geometry_msgs::Twist& msg )
{
    last_vel_cmd_time = ros::Time::now();

    double v = msg.linear.x;
    double omega = msg.angular.z;

    //ROS_INFO_STREAM( "Received Twist. [v: " << v << "; omega: " << omega <<"]" );

    if( !rbCtrl->setRobotSpeed( v, omega ) )
    {
        ROS_ERROR_STREAM( "Robot Speed not set!" );
    }
}

bool setPid_callback( robocontroller::SetPIDRequest& req, robocontroller::SetPIDResponse& resp )
{
    MotorPos mot;
    mot = req.motorIdx==0?motLeft:motRight;

    resp.ok = rbCtrl->setPidValues( mot, req.K_P, req.K_I, req.K_D );

    return resp.ok;
}

bool getPid_callback( robocontroller::GetPIDRequest &req, robocontroller::GetPIDResponse &resp )
{
    MotorPos mot;
    mot = req.motorIdx==0?motLeft:motRight;

    bool ok = rbCtrl->getPidValues( mot, resp.K_P, resp.K_I, resp.K_D );

    return ok;
}

bool getBoardStatus_callback( robocontroller::GetBoardStatusRequest& req, robocontroller::GetBoardStatusResponse& resp )
{
    BoardStatus status;

    if( !rbCtrl->getBoardStatus( status ) )
        return false;

    resp.pid_enable = status.pidEnable;
    resp.ramps_enable = status.accelRampEnable;
    resp.watchdog_enable = status.wdEnable;
    resp.saveToEeprom_enable = status.saveToEeprom;

    return true;

}

bool setBattCalib_callback( robocontroller::SetBatteryCalibRequest& req, robocontroller::SetBatteryCalibResponse& resp )
{
    AnalogCalibValue par_type;
    par_type = req.paramType==0?CalLow:CalHigh;

    resp.ok = rbCtrl->setBattCalibValue( par_type, ((double)req.batValue_mV)/1000.0 );

    return resp.ok;
}

bool enablePID_callback( robocontroller::EnablePIDRequest& req, robocontroller::EnablePIDResponse& resp )
{    
    resp.ok = rbCtrl->enablePID( req.pidEnabled, req.rampsEnabled );

    return resp.ok;
}

bool enableCommWD_callback( robocontroller::EnableCommWDRequest& req, robocontroller::EnableCommWDResponse& resp )
{
    resp.ok = rbCtrl->enableWD( req.wdEnabled, req.wdTime_msec );

    return resp.ok;
}

bool enableSaveToEeprom_callback( robocontroller::EnableSaveToEepromRequest& req, robocontroller::EnableSaveToEepromResponse& resp )
{
    resp.ok = rbCtrl->enableSaveToEeprom( req.enableSaveToEeprom );

    return resp.ok;
}

void test_connection()
{
    ROS_INFO_STREAM("Testing RoboController connection");

    if( !rbCtrlIface->testBoardConnection() )
    {
        ROS_ERROR_STREAM("Robocontroller is not replying. Trying reconnection...");
        rbCtrlIface->connectModbus( -1 );
    }
}

void init_system( ros::NodeHandle& nh )
{
    string nodeName = ros::this_node::getName();
    string nameSpace = ros::this_node::getNamespace();

    string paramStr;

    // >>>>> Global
    paramStr = ( nameSpace + nodeName + "/general/Telemetry_freq");
    if(nh.hasParam( paramStr ))
    {
        nh.getParam(paramStr, telem_freq);
        ROS_INFO_STREAM( boardIdx );
    }
    else
        nh.setParam(paramStr, telem_freq );

    paramStr = ( nameSpace + nodeName + "/general/Command_timeout");
    if(nh.hasParam( paramStr ))
    {
        nh.getParam(paramStr, vel_cmd_timeout_sec);
        ROS_INFO_STREAM( boardIdx );
    }
    else
        nh.setParam(paramStr, vel_cmd_timeout_sec );

    paramStr = ( nameSpace + nodeName + "/general/Speed_filter_enabled");
    if(nh.hasParam( paramStr ))
    {
        nh.getParam(paramStr, speed_filter_enabled);
        ROS_INFO_STREAM( boardIdx );
    }
    else
        nh.setParam(paramStr, speed_filter_enabled );
    // <<<<< Global

    // >>>>> Serial Port
    paramStr = ( nameSpace + nodeName + "/rc_serial/Board_Idx");
    if(nh.hasParam( paramStr ))
        nh.getParam(paramStr, boardIdx);
    else
        nh.setParam(paramStr, boardIdx );

    paramStr = ( nameSpace + nodeName + "/rc_serial/Serial_Port");
    if(nh.hasParam( paramStr ))    
        nh.getParam(paramStr, serialPort);
    else
        nh.setParam(paramStr, serialPort );

    paramStr = ( nameSpace + nodeName + "/rc_serial/Baud_Rate");
    if(nh.hasParam( paramStr ))
        nh.getParam(paramStr, serialbaudrate);
    else
        nh.setParam(paramStr, serialbaudrate );

    paramStr = ( nameSpace + nodeName + "/rc_serial/Parity");
    if(nh.hasParam( paramStr ))
        nh.getParam(paramStr, parity);
    else
        nh.setParam(paramStr, parity );

    paramStr = ( nameSpace + nodeName + "/rc_serial/Data_Bit");
    if(nh.hasParam( paramStr ))
        nh.getParam(paramStr, data_bit);
    else
        nh.setParam(paramStr, data_bit );

    paramStr = ( nameSpace + nodeName + "/rc_serial/Stop_Bits");
    if(nh.hasParam( paramStr ))
        nh.getParam(paramStr, stop_bit);
    else
        nh.setParam(paramStr, stop_bit );

    paramStr = ( nameSpace + nodeName + "/rc_serial/Simulation_Active");
    if(nh.hasParam( paramStr ))
        nh.getParam(paramStr, simulMode);
    else
        nh.setParam(paramStr, simulMode );

    // <<<<< Serial Port

    // >>>>> Robot Params to be saved on RoboController's EEPROM
    paramStr = ( nameSpace + nodeName + "/robot_param/Weight_g");
    if(nh.hasParam( paramStr ))
        nh.getParam(paramStr, rbConf.Weight);
    else
        nh.setParam(paramStr, 5000 );

    paramStr = ( nameSpace + nodeName + "/robot_param/Width_mm");
    if(nh.hasParam( paramStr ))
        nh.getParam(paramStr, rbConf.Width);
    else
        nh.setParam(paramStr, 420 );

    paramStr = ( nameSpace + nodeName + "/robot_param/Height_mm");
    if(nh.hasParam( paramStr ))
        nh.getParam(paramStr, rbConf.Height);
    else
        nh.setParam(paramStr, 200 );

    paramStr = ( nameSpace + nodeName + "/robot_param/Lenght_mm");
    if(nh.hasParam( paramStr ))
        nh.getParam(paramStr, rbConf.Lenght);
    else
        nh.setParam(paramStr, 365 );

    paramStr = ( nameSpace + nodeName + "/robot_param/WheelBase_mm");
    if(nh.hasParam( paramStr ))
        nh.getParam(paramStr, rbConf.WheelBase);
    else
        nh.setParam(paramStr, 340 );

    paramStr = ( nameSpace + nodeName + "/robot_param/WheelRadiusLeft_cent_mm");
    if(nh.hasParam( paramStr ))
        nh.getParam(paramStr, rbConf.WheelRadiusLeft);
    else
        nh.setParam(paramStr, 3500 );

    paramStr = ( nameSpace + nodeName + "/robot_param/WheelRadiusRight_cent_mm");
    if(nh.hasParam( paramStr ))
        nh.getParam(paramStr, rbConf.WheelRadiusRight);
    else
        nh.setParam(paramStr, 3500 );

    paramStr = ( nameSpace + nodeName + "/robot_param/EncoderCprLeft");
    if(nh.hasParam( paramStr ))
        nh.getParam(paramStr, rbConf.EncoderCprLeft);
    else
        nh.setParam(paramStr, 400 );

    paramStr = ( nameSpace + nodeName + "/robot_param/EncoderCprRight");
    if(nh.hasParam( paramStr ))
        nh.getParam(paramStr, rbConf.EncoderCprRight);
    else
        nh.setParam(paramStr, 400 );

    paramStr = ( nameSpace + nodeName + "/robot_param/MaxRpmMotorLeft");
    if(nh.hasParam( paramStr ))
        nh.getParam(paramStr, rbConf.MaxRpmMotorLeft);
    else
        nh.setParam(paramStr, 218 );

    paramStr = ( nameSpace + nodeName + "/robot_param/MaxRpmMotorRight");
    if(nh.hasParam( paramStr ))
        nh.getParam(paramStr, rbConf.MaxRpmMotorRight);
    else
        nh.setParam(paramStr, 218 );

    paramStr = ( nameSpace + nodeName + "/robot_param/MaxAmpereMotorLeft_mA");
    if(nh.hasParam( paramStr ))
        nh.getParam(paramStr, rbConf.MaxAmpereMotorLeft);
    else
        nh.setParam(paramStr, 1650 );

    paramStr = ( nameSpace + nodeName + "/robot_param/MaxAmpereMotorRight_mA");
    if(nh.hasParam( paramStr ))
        nh.getParam(paramStr, rbConf.MaxAmpereMotorRight);
    else
        nh.setParam(paramStr, 1650 );

    paramStr = ( nameSpace + nodeName + "/robot_param/MaxTorqueMotorLeft_Ncm");
    if(nh.hasParam( paramStr ))
        nh.getParam(paramStr, rbConf.MaxTorqueMotorLeft);
    else
        nh.setParam(paramStr, 60 );

    paramStr = ( nameSpace + nodeName + "/robot_param/MaxTorqueMotorRight_Ncm");
    if(nh.hasParam( paramStr ))
        nh.getParam(paramStr, rbConf.MaxTorqueMotorRight);
    else
        nh.setParam(paramStr, 60 );

    paramStr = ( nameSpace + nodeName + "/robot_param/RatioShaftLeft");
    if(nh.hasParam( paramStr ))
        nh.getParam(paramStr, rbConf.RatioShaftLeft);
    else
        nh.setParam(paramStr, 3 );

    paramStr = ( nameSpace + nodeName + "/robot_param/RatioShaftRight");
    if(nh.hasParam( paramStr ))
        nh.getParam(paramStr, rbConf.RatioShaftRight);
    else
        nh.setParam(paramStr, 3 );

    paramStr = ( nameSpace + nodeName + "/robot_param/RatioMotorLeft");
    if(nh.hasParam( paramStr ))
        nh.getParam(paramStr, rbConf.RatioMotorLeft);
    else
        nh.setParam(paramStr, 55 );

    paramStr = ( nameSpace + nodeName + "/robot_param/RatioMotorRight");
    if(nh.hasParam( paramStr ))
        nh.getParam(paramStr, rbConf.RatioMotorRight);
    else
        nh.setParam(paramStr, 55 );

    int val;
    paramStr = ( nameSpace + nodeName + "/robot_param/MotorEnableLevel");
    if(nh.hasParam( paramStr ))
    {
        nh.getParam(paramStr, val);
        rbConf.MotorEnableLevel = static_cast<PinLevel>(val);
    }
    else
        nh.setParam(paramStr, 1 );


    paramStr = ( nameSpace + nodeName + "/robot_param/EncoderPosition");
    if(nh.hasParam( paramStr ))
    {
        nh.getParam(paramStr, val/*rbConf.EncoderPosition*/);
        rbConf.EncoderPosition = static_cast<EncoderPos>(val);
    }
    else
        nh.setParam(paramStr, 0 );

    paramStr = ( nameSpace + nodeName + "/robot_param/MaxChargedBatteryLevel_mV");
    if(nh.hasParam( paramStr ))
        nh.getParam(paramStr, rbConf.MaxChargedBatteryLevel);
    else
        nh.setParam(paramStr, 16800 );

    paramStr = ( nameSpace + nodeName + "/robot_param/MinChargedBatteryLevel_mV");
    if(nh.hasParam( paramStr ))
        nh.getParam(paramStr, rbConf.MinChargedBatteryLevel);
    else
        nh.setParam(paramStr, 12000 );
    // <<<<< Robot Params to be saved on RoboController's EEPROM
}

int main( int argc, char **argv)
{
    ros::init( argc, argv, "robocontroller_node" );
    ros::NodeHandle nh;

    // Parameters initialization
    init_system( nh );

    // Subscription to use standard "cmd_vel" speed commands
    ros::Subscriber cmd_vel_sub = nh.subscribe( "/cmd_vel", 3, &vel_cmd_callback );

    // Publisher for the Telemetry of the robot
    ros::Publisher telem_pub = nh.advertise<robocontroller::Telemetry>( "/robocontroller/Telemetry", 100 );
    robocontroller::Telemetry telemetry_msg;

    // Publisher for the Pose of the robot
    ros::Publisher pose_pub = nh.advertise<robocontroller::Pose>( "/robocontroller/Pose", 100 );
    robocontroller::Pose pose_msg;

    // Publisher for Debug message
    ros::Publisher debug_pub = nh.advertise<robocontroller::Debug>( "/robocontroller/Debug", 100 );
    robocontroller::Debug debug_msg;

    // Server PID
    ros::ServiceServer setPidServer = nh.advertiseService( "/robocontroller/SetPid", setPid_callback );
    ros::ServiceServer getPidServer = nh.advertiseService( "/robocontroller/GetPid", getPid_callback );
    ros::ServiceServer enablePidServer = nh.advertiseService( "/robocontroller/EnablePid", enablePID_callback );

    // Server Battery Calibration
    ros::ServiceServer battCalibServer = nh.advertiseService( "/robocontroller/SetBattCalib", setBattCalib_callback );

    // Server WatchDog
    ros::ServiceServer enableWdServer = nh.advertiseService( "/robocontroller/EnableCommWD", enableCommWD_callback );

    // Server enable EEPROM saving
    ros::ServiceServer enableEepromServer = nh.advertiseService( "/robocontroller/EnableSaveToEeprom", enableSaveToEeprom_callback );

    // Server Robocontroller Board Status
    ros::ServiceServer getStatusServer = nh.advertiseService( "robocontroller/GetBoardStatus", getBoardStatus_callback);

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

    // Set Robot params
    rbCtrl->setRobotConfig( rbConf );
    rbCtrl->enableSpeedFilter( speed_filter_enabled );

    // RoboController publishes telemetry at 30hz
    ros::Rate rate( telem_freq );

    last_vel_cmd_time = ros::Time::now();

    while( ros::ok() )
    {
        ros::spinOnce(); // Process pending callback

        // >>>>> Debug message
        if( debug_pub.getNumSubscribers()>0 )
        {
            RcDebug debugInfo;
            if( rbCtrl->getDebugInfo( debugInfo ) )
            {
                debug_msg.enc1_period = debugInfo.enc1_period;
                debug_msg.enc2_period = debugInfo.enc2_period;

                for( int i=0; i<debug_msg.debug.size(); i++ )
                    debug_msg.debug[i]=debugInfo.debug_reg[i];

                debug_pub.publish( debug_msg );

                ROS_INFO_STREAM( "Published Debug information");
            }
        }
        // <<<<< Debug message

        // >>>>> Publishing Telemetry at 30Hz
        RobotTelemetry roboTelemetry;
        RobotPose roboPose;
        if( rbCtrl->getTelemetry( roboTelemetry ) )
        {
            pose_msg.header.stamp = telemetry_msg.header.stamp = ros::Time::now();
            telemetry_msg.mot_pwm_left = roboTelemetry.PwmLeft;
            telemetry_msg.mot_pwm_right = roboTelemetry.PwmRight;
            telemetry_msg.mot_rpm_left = roboTelemetry.RpmLeft;
            telemetry_msg.mot_rpm_right = roboTelemetry.RpmRight;
            telemetry_msg.mot_speed_left = roboTelemetry.LinSpeedLeft;
            telemetry_msg.mot_speed_right = roboTelemetry.LinSpeedRight;
            telemetry_msg.battery = roboTelemetry.Battery;

            // Telemetry publishing
            telem_pub.publish( telemetry_msg );

            rbCtrl->getPose( roboPose );

            pose_msg.X = roboPose.x;
            pose_msg.Y = roboPose.y;
            pose_msg.Theta = roboPose.theta;

            // Pose publishing
            pose_pub.publish( pose_msg );

            ROS_DEBUG_STREAM( "Telemetry ID " << telemetry_msg.header.seq << "published" );
            ROS_DEBUG_STREAM( "Pose ID " << pose_msg.header.seq << "published" );
        }
        else
        {
            ROS_WARN_STREAM( "No telemetry received" );
        }
        // <<<<< Publishing Telemetry at 30Hz

        // >>>>> if not received a movement message for 1 sec then stop motors!
        double time_since_last_cmd = (ros::Time::now() - last_vel_cmd_time).toSec();
        if( !rbCtrl->isMotorStopped() &&
                time_since_last_cmd > vel_cmd_timeout_sec )
        {
            rbCtrl->stopMotors();

            ROS_INFO_STREAM( "No velocity command received since " << time_since_last_cmd << "seconds. Motor Stopped!" );

            test_connection(); // Let's send a test message to board to verify that it is connected
        }
        // <<<<< if not received a movement message for N msec then stop motors!

        rate.sleep();
    }

    if(rbCtrlIface)
        delete rbCtrlIface;

    if(rbCtrl)
        delete rbCtrl;

    return 0;
}
