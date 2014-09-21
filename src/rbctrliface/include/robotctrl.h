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
    bool getMotorSpeeds(double &speedL, double &speedR );

    bool stopMotors();


    bool getTelemetry( RobotTelemetry& telemetry);
    void getPose( RobotPose& pose); // Get the latest pose updated by Telemetry
    bool getDebugInfo( RcDebug& debug);

    inline bool isMotorStopped(){return mMotStopped;}

    void enableSpeedFilter(bool enable){ROS_INFO_STREAM( "Speed filter enabled: " << enable);mSpeedFilterActive=true;}
    bool isSpeedFilterEnabled(){return mSpeedFilterActive;}

    bool setRobotConfig( RobotConfiguration& config );

    bool setPidValues( MotorPos mot, u_int16_t Kp, u_int16_t Ki, u_int16_t Kd );
    bool getPidValues( MotorPos mot, u_int16_t& Kp, u_int16_t& Ki, u_int16_t& Kd );

    bool getBoardStatus( BoardStatus& status);

    bool setBattCalibValue(AnalogCalibValue valueType, double curChargeVal_V );

private:
    void initSpeedFilter();
    void updateMeanVar();
    void applySpeedFilter(RobotTelemetry& telemetry);

    bool updateBoardStatus();

private:
    ros::NodeHandle* mNodeH;
    RbCtrlIface* mRbCtrl;

    bool mMotStopped;
    bool mBoardStatusUpdated;

    BoardStatus mBoardStatus;

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
