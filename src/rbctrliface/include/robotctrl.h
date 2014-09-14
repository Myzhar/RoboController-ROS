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

    inline bool isMotorStopped(){return mMotStopped;}

private:
    ros::NodeHandle* mNodeH;
    RbCtrlIface* mRbCtrl;

    bool mMotStopped;

    RobotTelemetry mTelemetry;
};

#endif // ROBOTCTRL_H
