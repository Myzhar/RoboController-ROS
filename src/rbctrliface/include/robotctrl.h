#ifndef ROBOTCTRL_H
#define ROBOTCTRL_H

#include <ros/ros.h>
#include "rbctrliface.h"
#include "rc_msg_srv.h"

namespace robocontroller
{
class RobotCtrl
{
public:
    RobotCtrl(ros::NodeHandle* nh,RbCtrlIface* rbCtrl);

    bool setRobotSpeed( double fwSpeed, double rotSpeed );
    bool setMotorSpeeds(double speedL, double speedR );
    bool stopMotors();

    bool getMotorSpeeds(double &speedL, double &speedR );

    bool getTelemetry( RobotTelemetry& telemetry);
    void getPose( RobotPose& pose); // Get the latest pose updated by Telemetry

    inline bool isMotorStopped(){return mMotStopped;}

    void enableSpeedFilter(bool enable){mSpeedFilterActive=true;}
    bool isSpeedFilterEnabled(){return mSpeedFilterActive;}

private:
    void initSpeedFilter();
    void updateMeanVar();

private:
    ros::NodeHandle* mNodeH;
    RbCtrlIface* mRbCtrl;

    bool mMotStopped;

    RobotTelemetry mTelemetry;
    RobotPose mPose;

    RobotConfiguration mConfig;

    ros::Time mLastTelemTime;

    // >>>>> Speed Filter
    bool mSpeedFilterActive;
    vector<double> mMotorSpeedVecLeft;
    vector<double> mMotorSpeedVecRight;
    double mSpeedMeanLeft;
    double mSpeedVarLeft;
    double mSpeedMeanRight;
    double mSpeedVarRight;
    int mSpeedRightCount;
    int mSpeedLeftCount;
    uint8_t mSpeedLeftVecIdx;
    uint8_t mSpeedRightVecIdx;
    // <<<<< Speed Filter
};

}

#endif // ROBOTCTRL_H
