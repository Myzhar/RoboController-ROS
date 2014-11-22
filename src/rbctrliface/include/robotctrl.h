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

    /*!
     * \brief setRobotSpeed set forward and rotation speed
     *
     * \param fwSpeed forward speed in m/sec
     * \param rotSpeed rotation speed in rad/sec
     *
     * \return true if no error
     */
    bool setRobotSpeed( double fwSpeed, double rotSpeed );

    bool setWheelLinSpeeds(double speedL, double speedR );
    bool getWheelLinSpeeds(double &speedL, double &speedR );

    bool setWheelRPM(double wheelRpmL, double wheelRpmR );
    bool getWheelRPM(double &wheelRpmL, double &wheelRpmR );

    /*!
     * \brief setWheelRotSpeed sets the speed of the wheels in rad/sec
     *
     * \param wheelRotSpeedL speed of the left wheel in rad/sec
     * \param wheelRotSpeedR speed of the right wheel in rad/sec
     *
     * \return true if no error
     */
    bool setWheelRotSpeed(double wheelRotSpeedL, double wheelRotSpeedR );

    /*!
     * \brief getWheelRotSpeed returns the rotation speed of the wheels
     *
     * \param wheelRotSpeedL the current rotation speed of the left wheel in rad/sec
     * \param wheelRotSpeedR the current rotation speed of the right wheel in rad/sec
     *
     * \return true if no error
     */
    bool getWheelRotSpeed(double &wheelRotSpeedL, double &wheelRotSpeedR );

    /*!
     * \brief setEncoderRotSpeed set the rotation speed of the motor referred to encoder.
     *
     * \param encoderRotSpeedL rotation speed of the left encoder measured by encoder in rad/sec
     * \param encoderRotSpeedR rotation speed of the right encoder measured by encoder in rad/sec
     * \return true if no error
     */
    bool setEncoderRotSpeed(double encoderRotSpeedL, double encoderRotSpeedR );

    /*!
     * \brief getEncoderRotSpeed returns the rotation speed measured by the encoders
     *
     * \param encoderRotSpeedL the current rotation speed of the left encoder in rad/sec
     * \param encoderRotSpeedR the current rotation speed of the right encoder in rad/sec
     *
     * \return true if no error
     */
    bool getEncoderRotSpeed(double &encoderRotSpeedL, double &encoderRotSpeedR );

    /*!
     * \brief stopMotors force the motors to stop
     *
     * \return true if no error
     */
    bool stopMotors();

    bool getTelemetry( RobotTelemetry& telemetry);
    void getPose( RobotPose& pose); // Get the latest pose updated by Telemetry
    bool getDebugInfo( RcDebug& debug);

    inline bool isMotorStopped(){return mMotStopped;}

    void enableSpeedFilter(bool enable){ROS_INFO_STREAM( "Speed filter enabled: " << enable);mSpeedFilterActive=enable;}
    bool isSpeedFilterEnabled(){return mSpeedFilterActive;}

    bool setRobotConfig( RobotConfiguration& config );
    bool getRobotConfig( RobotConfiguration& config );

    bool setPidValues( MotorPos mot, u_int16_t Kp, u_int16_t Ki, u_int16_t Kd );
    bool getPidValues( MotorPos mot, u_int16_t& Kp, u_int16_t& Ki, u_int16_t& Kd );

    bool getBoardStatus( BoardStatus& status);
    bool setBoardStatus( BoardStatus& status);

    bool enablePID( bool pidEnable, bool rampsEnable  );
    bool enableWD( bool enable, u_int16_t wdTime_msec );
    bool enableSaveToEeprom( bool enable );

    bool setWdTimeoutTime( u_int16_t wdTimeout_msec );
    u_int16_t getWdTimeoutTime();

    bool setBattCalibValue(AnalogCalibValue valueType, double curChargeVal_V );

    bool setRegister(u_int16_t regIdx, u_int16_t value );
    vector<u_int16_t>  getRegisters( u_int16_t startAddr, u_int16_t nReg );

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

    MotorCtrlMode mMotorCtrlMode; /**< Current motor control mode */

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
