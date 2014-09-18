// This node handle the communication with RoboController board

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <robocontroller/Telemetry.h>
#include <robocontroller/Pose.h>
#include <stdlib.h>

#include "rbctrliface.h"
#include "robotctrl.h"
#include "modbus_registers.h"

using namespace std;
using namespace robocontroller;

ros::Time last_vel_cmd_time; // Time of the last velocity comand received
double vel_cmd_timeout_sec; // Timeout for motor stop if non velocity command is received

RbCtrlIface* rbCtrlIface= NULL; // RoboController Interface
RobotCtrl* rbCtrl = NULL; // Robot control functions


// >>>>> Functions
void vel_cmd_callback( const geometry_msgs::Twist& msg );
void test_connection();
// <<<<< Functions

void vel_cmd_callback( const geometry_msgs::Twist& msg )
{
    last_vel_cmd_time = ros::Time::now();

    double v = msg.linear.x;
    double omega = msg.angular.z;

    ROS_INFO_STREAM( "Received Twist. [v: " << v << "; omega: " << omega <<"]" );

    if( !rbCtrl->setRobotSpeed( v, omega ) )
    {
        ROS_ERROR_STREAM( "Robot Speed not set!" );
    }
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

int main( int argc, char **argv) 
{
    ros::init( argc, argv, "robocontroller_node" );
    ros::NodeHandle nh;

    // Subscription to use standard "cmd_vel" speed commands
    ros::Subscriber cmd_vel_sub = nh.subscribe( "/cmd_vel", 3, &vel_cmd_callback );

    // Publisher for the Telemetry of the robot
    ros::Publisher telem_pub = nh.advertise<robocontroller::Telemetry>( "/robocontroller/Telemetry", 100 );
    robocontroller::Telemetry telemetry_msg;

    // Publisher for the Pose of the robot
    ros::Publisher pose_pub = nh.advertise<robocontroller::Pose>( "/robocontroller/Pose", 100 );
    robocontroller::Pose pose_msg;

    // >>>>> Interface to RoboController board

    // TODO load params using ROS parameters
    vel_cmd_timeout_sec = 1;
    int boardIdx = 1;
    string serialPort = "/dev/ttyUSB0";
    int serialbaudrate = 57600;
    char parity = 'N';
    int data_bit = 8;
    int stop_bit = 1;
    bool simulMode = false;

    rbCtrlIface = new RbCtrlIface( boardIdx, serialPort, serialbaudrate, parity, data_bit, stop_bit, simulMode );

    if(!rbCtrlIface->isConnected())
    {
        ROS_FATAL_STREAM( "RoboController not found on port " << serialPort );
        return -1;
    }
    // <<<<< Interface to RoboController board

    // Robot Control functions
    rbCtrl = new RobotCtrl( &nh, rbCtrlIface );

    // RoboController publishes telemetry at 30hz
    ros::Rate rate( 30 );

    last_vel_cmd_time = ros::Time::now();

    while( ros::ok() )
    {
        ros::spinOnce(); // Process pending callback

        // >>>>> Publishing Telemetry at 30Hz
        RobotTelemetry roboTelemetry;
        RobotPose roboPose;
        if( rbCtrl->getTelemetry( roboTelemetry ) )
        {
            pose_msg.header.stamp = telemetry_msg.header.stamp = ros::Time::now();
            telemetry_msg.mot_pwm_left = roboTelemetry.PwmLeft;
            telemetry_msg.mot_pwm_right = roboTelemetry.PwmRight;
            telemetry_msg.mot_rpm_left = roboTelemetry.RpmLeft;
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
