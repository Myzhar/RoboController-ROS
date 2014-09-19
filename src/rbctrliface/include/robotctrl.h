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

    bool getDebugInfo( RcDebug& debug);

    inline bool isMotorStopped(){return mMotStopped;}

    void enableSpeedFilter(bool enable){mSpeedFilterActive=true;}
    bool isSpeedFilterEnabled(){return mSpeedFilterActive;}

    bool setRobotConfig( RobotConfiguration& config );

private:
    void initSpeedFilter();
    void updateMeanVar();
    void applySpeedFilter(RobotTelemetry& telemetry);

private:
    ros::NodeHandle* mNodeH;
    RbCtrlIface* mRbCtrl;

    bool mMotStopped;

    RobotTelemetry mTelemetry;
    RobotPose mPose;

    RobotConfiguration mRobotConfig;

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
    int mSpeedLeftVecIdx;
    int mSpeedRightVecIdx;
    // <<<<< Speed Filter
};

}

#endif // ROBOTCTRL_H
