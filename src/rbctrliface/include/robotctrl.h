#ifndef ROBOTCTRL_H
#define ROBOTCTRL_H

#include <ros/ros.h>
#include "rbctrliface.h"
#include "rc_msg_srv.h"

class RobotCtrl
{
public:
    RobotCtrl(ros::NodeHandle* nh,RbCtrlIface* rbCtrl);

    bool setMotorSpeeds(double speedL, double speedR );
    bool stopMotors();

    bool getMotorSpeeds(double &speedL, double &speedR );

    bool getTelemetry( RobotTelemetry& telemetry);
    void getPose( RobotPose& pose); // Get the latest pose updated by Telemetry

    inline bool isMotorStopped(){return mMotStopped;}

private:
    ros::NodeHandle* mNodeH;
    RbCtrlIface* mRbCtrl;

    bool mMotStopped;

    RobotTelemetry mTelemetry;
    RobotPose mPose;

    RobotConfiguration mConfig;

    ros::Time mLastTelemTime;
};

#endif // ROBOTCTRL_H
