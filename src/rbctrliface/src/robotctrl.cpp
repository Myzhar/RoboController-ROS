#include "robotctrl.h"
#include "modbus_registers.h"

#define RADPS2RPM 9.549296585514
#define RPM2RADPS 0.104719755119

namespace robocontroller
{

#define SPEED_FILTER_SIZE 10

#define _SIGN(X) (((X) >= 0) - ((X) < 0))

RobotCtrl::RobotCtrl(ros::NodeHandle* nh, RbCtrlIface *rbCtrl)
{
    mNodeH = nh;
    mMotStopped = true;
    mBoardStatusUpdated = false;

    mRbCtrl = rbCtrl;

    mPose.x = 0.0;
    mPose.y = 0.0;
    mPose.theta = 0.0;

    mSpeedFilterActive = true;

    initSpeedFilter();
    updateBoardStatus();
}

bool RobotCtrl::getDebugInfo( RcDebug& debug )
{
    uint16_t startAddr = WORD_ENC1_PERIOD;
    //uint16_t nReg = 2;
    uint16_t nReg = 3; // TODO remember to change this to 2 when the firmware changes!

    vector<uint16_t> reply = mRbCtrl->readMultiReg( startAddr, nReg );

    if( reply.size() != nReg )
    {
        ROS_WARN_STREAM( "RC reply for debug is incorrect in size, expected " << nReg << ", received " << reply.size() );
        return false;
    }

    debug.enc1_period = reply[0];
    //debug.enc2_period = reply[1];
    debug.enc2_period = reply[2]; // TODO remember to change this to 1 when the firmware changes!

    // >>>>> Debug registers
    startAddr = WORD_DEBUG_00;
    nReg = 20;

    vector<uint16_t> reply2 = mRbCtrl->readMultiReg( startAddr, nReg );

    if( reply2.size() != nReg )
    {
        ROS_WARN_STREAM( "RC reply for debug registers is incorrect in size, expected " << nReg << ", received " << reply2.size() );
        return false;
    }
    // <<<<< Debug registers

    memcpy( debug.debug_reg, reply2.data(), nReg );

    return true;
}

bool RobotCtrl::getTelemetry( RobotTelemetry& telemetry)
{
    // TODO Everything in getTelemetry function must be changed
    //      when RoboController will emit RPM instead of
    //      linear speeds

    // >>>>> Telemetry update
    // WORD_TENSIONE_ALIM 8
    // WORD_ENC1_SPEED 20
    // WORD_ENC2_SPEED 21
    // WORD_RD_PWM_CH1 22
    // WORD_RD_PWM_CH2 23

    uint16_t startAddr = WORD_ENC1_SPEED;
    uint16_t nReg = 4;

    vector<uint16_t> reply = mRbCtrl->readMultiReg( startAddr, nReg );

    if( reply.size() != nReg )
    {
        ROS_WARN_STREAM( "RC reply for motors is incorrect in size, expected " << nReg << ", received " << reply.size() );
        return false;
    }

    double speed0;
    if(reply[0] < 32767)  // Speed is integer 2-complement!
        speed0 = ((double)reply[0])/1000.0;
    else
        speed0 = ((double)(reply[0]-65536))/1000.0;
    telemetry.LinSpeedLeft = speed0;

    double speed1;
    if(reply[1] < 32767)  // Speed is integer 2-complement!
        speed1 = ((double)reply[1])/1000.0;
    else
        speed1 = ((double)(reply[1]-65536))/1000.0;
    telemetry.LinSpeedRight = speed1;

    telemetry.PwmLeft = reply[2];
    telemetry.PwmRight = reply[3];

    // rpm = (v_lin/r)*RAD2RPM
    telemetry.RpmLeft  = RADPS2RPM*(telemetry.LinSpeedLeft/(((double)(mRobotConfig.WheelRadiusLeft))/100000.0)); // Remember that wheel radius is in 0.01mm
    telemetry.RpmRight = RADPS2RPM*(telemetry.LinSpeedRight/(((double)(mRobotConfig.WheelRadiusRight))/100000.0)); // Remember that wheel radius is in 0.01mm

    startAddr = WORD_TENSIONE_ALIM;
    nReg = 1;
    reply = mRbCtrl->readMultiReg( startAddr, nReg );

    if( reply.size() != nReg )
    {
        ROS_WARN_STREAM( "RC reply for battery is incorrect in size, expected " << nReg << ", received " << reply.size() );
        return false;
    }

    telemetry.Battery = ((double)reply[0])/1000.0;

    // >>>>> Speed Filter
    // Eliminates glitches due to uncorrect speed reading on the RoboController
    if( mSpeedFilterActive)
        applySpeedFilter(telemetry);
    // <<<<< Speed Filter

    memcpy( &mTelemetry, &telemetry, sizeof(RobotTelemetry) );

    double v = (telemetry.LinSpeedLeft + telemetry.LinSpeedRight)/2.0;
    double omega = (telemetry.LinSpeedLeft + telemetry.LinSpeedRight)/((double)mRobotConfig.WheelBase/1000.0);

    ros::Time now = ros::Time::now();

    double dt = (now - mLastTelemTime).toSec();
    mLastTelemTime = now;

    // >>>>> Odometry update
    // TODO this elaboration must be replaced by QEI data from
    //      RoboController, when QEI will be updated!
    mPose.theta += omega * dt;
    mPose.x += v * cos( mPose.theta ) * dt;
    mPose.y += v * sin( mPose.theta ) * dt;
    // <<<<< Odometry update

    return true;
}

void RobotCtrl::getPose( RobotPose& pose)
{
    memcpy( &pose, &mPose, sizeof(RobotPose) );
}

bool RobotCtrl::getWheelLinSpeeds(double& speedL, double& speedR )
{


    return true;
}

void RobotCtrl::initSpeedFilter()
{
    // >>>>> Speed Filter Initialization
    mMotorSpeedVecLeft.resize( SPEED_FILTER_SIZE );
    mMotorSpeedVecRight.resize( SPEED_FILTER_SIZE );
    mMotorSpeedVecLeft.clear();
    mMotorSpeedVecRight.clear();
    mSpeedLeftCount = 0;
    mSpeedLeftVecIdx = 0;
    mSpeedRightCount = 0;
    mSpeedRightVecIdx = 0;
    mSpeedMeanLeft = 0.0;
    mSpeedVarLeft = 0.0;
    mSpeedMeanRight = 0.0;
    mSpeedVarRight = 0.0;
    // <<<<< Speed Filter Initialization
}

void RobotCtrl::applySpeedFilter(RobotTelemetry &telemetry)
{
    bool replaceLeft = false;
    bool replaceRight = false;

    if( fabs(mSpeedMeanLeft) < 0.1 )
        mSpeedMeanLeft = telemetry.LinSpeedLeft;

    if( fabs(mSpeedMeanRight) < 0.1 )
        mSpeedMeanLeft = telemetry.LinSpeedRight;
    else
    {
        if( /*((mSpeedMeanLeft != 0) && _SIGN(telemetry.LinSpeedLeft) != _SIGN(mSpeedMeanLeft)) ||*/
                fabs(telemetry.LinSpeedLeft)<fabs(mSpeedMeanLeft)/2 )
        {
            mMotorSpeedVecLeft[mSpeedLeftVecIdx] = mSpeedMeanLeft;
            replaceLeft = true;
        }
        else
            mMotorSpeedVecLeft[mSpeedLeftVecIdx] = telemetry.LinSpeedLeft;

        if( /*((mSpeedMeanRight != 0) &&_SIGN(telemetry.LinSpeedRight)!=_SIGN(mSpeedMeanRight)) ||*/
                fabs(telemetry.LinSpeedRight)<fabs(mSpeedMeanRight)/2 )
        {
            mMotorSpeedVecRight[mSpeedRightVecIdx] = mSpeedMeanRight;
            replaceRight = true;
        }
        else
            mMotorSpeedVecRight[mSpeedRightVecIdx] = telemetry.LinSpeedRight;
    }

    mSpeedLeftVecIdx++;
    mSpeedLeftVecIdx %= SPEED_FILTER_SIZE;
    mSpeedRightVecIdx++;
    mSpeedRightVecIdx %= SPEED_FILTER_SIZE;

    if(mSpeedLeftCount<SPEED_FILTER_SIZE) // Used for incomplete vector
        mSpeedLeftCount++;
    if(mSpeedRightCount<SPEED_FILTER_SIZE) // Used for incomplete vector
        mSpeedRightCount++;

    updateMeanVar();

    if(replaceLeft)
        telemetry.LinSpeedLeft = mSpeedMeanLeft;
    else
        telemetry.LinSpeedLeft = telemetry.LinSpeedLeft;

    if(replaceRight)
        telemetry.LinSpeedRight = mSpeedMeanRight;
    else
        telemetry.LinSpeedRight = telemetry.LinSpeedRight;
}

void RobotCtrl::updateMeanVar()
{
    /* Algorithm:
     * ==========
     * def online_variance(data):
     * n = 0
     * mean = 0
     * M2 = 0
     *
     * for x in data:
     *     n = n + 1
     *     delta = x - mean
     *     mean = mean + delta/n
     *     M2 = M2 + delta*(x - mean)
     *
     * if (n < 2):
     *     return 0
     *
     * variance = M2/(n - 1)
     * return variance
     */

    int n;
    double M2;

    // >>>>> Left Motor
    n=0;
    M2=0.0;
    mSpeedMeanLeft=0.0;

    for( int i=0; i<mSpeedLeftCount; i++ )
    {
        n++;
        double delta = mMotorSpeedVecLeft[i] - mSpeedMeanLeft;
        mSpeedMeanLeft += (delta/(double)n);

        M2 += delta*(mMotorSpeedVecLeft[i] - mSpeedMeanLeft);
    }

    if(n<2)
        mSpeedVarLeft = 0.0;
    else
        mSpeedVarLeft = M2/(double)(n - 1);
    // <<<<< Left Motor

    // >>>>> Right Motor
    n=0;
    M2=0.0;
    mSpeedMeanRight=0.0;

    for( int i=0; i<mSpeedRightCount; i++ )
    {
        n++;
        double delta = mMotorSpeedVecRight[i] - mSpeedMeanRight;
        mSpeedMeanRight += (delta/(double)n);

        M2 += delta*(mMotorSpeedVecRight[i] - mSpeedMeanRight);
    }

    if(n<2)
        mSpeedVarRight = 0.0;
    else
        mSpeedVarRight = M2/(n - 1);
    // <<<<< Right Motor

}

bool RobotCtrl::setRobotSpeed( double fwSpeed, double rotSpeed )
{
    double speedL = fwSpeed + 0.5 * rotSpeed * ((double)mRobotConfig.WheelBase)/1000.0;
    double speedR = fwSpeed - 0.5 * rotSpeed * ((double)mRobotConfig.WheelBase)/1000.0;

    ROS_INFO_STREAM( "fwSpeed: " << fwSpeed << " ; rotSpeed: " << rotSpeed << " m/sec" );
    ROS_INFO_STREAM( "speedL: " << speedL << " ; speedR: " << speedR << " m/sec" );

    if( fabs(speedL) < 0.01 )
    {
        mSpeedLeftCount = 0;
        mSpeedLeftVecIdx = 0;
        mSpeedMeanLeft = 0.0;
        mSpeedVarLeft = 0.0;
    }

    if( fabs(speedR) < 0.01 )
    {
        mSpeedRightCount = 0;
        mSpeedRightVecIdx = 0;
        mSpeedMeanRight = 0.0;
        mSpeedVarRight = 0.0;
    }

    return setWheelLinSpeeds( speedL, speedR );
}

bool RobotCtrl::setWheelLinSpeeds( double speedL, double speedR )
{
    if( mMotorCtrlMode != mcPID )
    {
        ROS_ERROR_STREAM( "It is not possible to set the speeds of the motors. PID in not enabled");
        return false;
    }

    if( mMotStopped )
        mLastTelemTime = ros::Time::now();

    // >>>>> 16 bit saturation
    if( speedL > 32.767)
        speedL = 32.767;

    if( speedL < -32.768 )
        speedL = -32.768;

    if( speedR > 32.767)
        speedR = 32.767;

    if( speedR < -32.768 )
        speedR = -32.768;
    // <<<<< 16 bit saturation

    // TODO convert in rad/sec, and call setWheelRotSpeed

}

bool RobotCtrl::setWheelRPM(double wheelRpmL, double wheelRpmR )
{
    // TODO convert in rad/sec, apply ratios and call setEncoderRotSpeed
}

bool RobotCtrl::getWheelRPM(double &wheelRpmL, double &wheelRpmR )
{}

bool RobotCtrl::setWheelRotSpeed(double wheelRotSpeedL, double wheelRotSpeedR )
{
    double ratioL = 1.0; // Ratio is 1.0 when encoder is mounted on the wheel shaft
    double ratioR = 1.0; // Ratio is 1.0 when encoder is mounted on the wheel shaft

    if( mRobotConfig.EncoderPosition == Motor )
    {
        ratioL = mRobotConfig.RatioMotorLeft/mRobotConfig.RatioShaftLeft;
        ratioR = mRobotConfig.RatioMotorRight/mRobotConfig.RatioShaftRight;
    }

    double encSpeedL = wheelRotSpeedL*ratioL;
    double encSpeedR = wheelRotSpeedR*ratioR;

    return setEncoderRotSpeed( encSpeedL, encSpeedR );
}

bool RobotCtrl::getWheelRotSpeed(double &wheelRotSpeedL, double &wheelRotSpeedR )
{
    double encSpeedR;
    double encSpeedL;

    if( !getEncoderRotSpeed( encSpeedL, encSpeedR) )
        return false;

    double ratioL = 1.0; // Ratio is 1.0 when encoder is mounted on the wheel shaft
    double ratioR = 1.0; // Ratio is 1.0 when encoder is mounted on the wheel shaft

    if( mRobotConfig.EncoderPosition == Motor )
    {
        ratioL = mRobotConfig.RatioMotorLeft/mRobotConfig.RatioShaftLeft;
        ratioR = mRobotConfig.RatioMotorRight/mRobotConfig.RatioShaftRight;
    }

    wheelRotSpeedL = encSpeedL/ratioL;
    wheelRotSpeedR = encSpeedR/ratioR;

    return true;
}

bool RobotCtrl::setEncoderRotSpeed(double encoderRotSpeedL, double encoderRotSpeedR )
{
    if( mMotorCtrlMode != mcPID )
    {
        ROS_ERROR_STREAM( "It is not possible to set the speeds of the motors. PID in not enabled");
        return false;
    }

    /*! \note: rotation speed must be sent to RoboController in 0.1 rad/sec
     * so we must convert RPM in rad/sec before sending
     */

    double scaledRotL =  encoderRotSpeedL * 10.0;
    double scaledRotR =  encoderRotSpeedR * 10.0;

    // >>>>> 16 bit saturation
    if(scaledRotL > 32767.0)
        scaledRotL = 32767.0;
    if(scaledRotL < -32768.0)
        scaledRotL = -32768.0;

    if(scaledRotR > 32767.0)
        scaledRotR = 32767.0;
    if(scaledRotR < -32768.0)
        scaledRotR = -32768.0;
    // <<<<< 16 bit saturation

    // >>>>> New SetPoint to RoboController
    uint16_t address = WORD_PWM_CH1;

    vector<uint16_t> data;
    data.resize(2);

    uint16_t sp; // Speed is integer 2-complement!
    if(scaledRotL >= 0)
        sp = (uint16_t)(scaledRotL);
    else
        sp = (uint16_t)(scaledRotL+65536.0);

    //uint16_t sp = (uint16_t)(speed*1000.0+32767.5);
    data[0] = sp;

    if(scaledRotR >= 0)
        sp = (uint16_t)(scaledRotR);
    else
        sp = (uint16_t)(scaledRotR+65536.0);

    //uint16_t sp = (uint16_t)(speed*1000.0+32767.5);
    data[1] = sp;

    bool commOk = mRbCtrl->writeMultiReg( address, 2, data );
    // <<<<< New SetPoint to RoboController

    if( commOk )
    {
        if( scaledRotL!=0.0 || scaledRotR!=0.0 )
            mMotStopped = false;
        else
            mMotStopped = true;
    }

    return commOk;
}

bool RobotCtrl::getEncoderRotSpeed(double &encoderRotSpeedL, double &encoderRotSpeedR )
{
    uint16_t startAddr = WORD_ENC1_SPEED;
    uint16_t nReg = 2;

    vector<uint16_t> reply = mRbCtrl->readMultiReg( startAddr, nReg );

    if( reply.size() != nReg )
    {
        ROS_WARN_STREAM( "RC reply for motor speeds is incorrect in size, expected " << nReg << ", received " << reply.size() );
        return false;
    }

    if(reply[0] < 32767)  // Speed is integer 2-complement!
        encoderRotSpeedL = ((double)reply[0])/10.0;
    else
        encoderRotSpeedL = ((double)(reply[0]-65536))/10.0;

    if(reply[1] < 32767)  // Speed is integer 2-complement!
        encoderRotSpeedR = ((double)reply[1])/10.0;
    else
        encoderRotSpeedR = ((double)(reply[1]-65536))/10.0;

    mTelemetry.LinSpeedLeft = encoderRotSpeedL;
    mTelemetry.LinSpeedRight = encoderRotSpeedR;
}

bool RobotCtrl::stopMotors()
{
    int count = 0;

    ros::Rate rate( 30 );
    while( 1 ) // Try to stop the motors 5 times for security!
    {
        if( setWheelLinSpeeds( 0.0, 0.0) )
        {
            mMotStopped = true;
            return true;
        }
        else
            count++;

        if(count==5)
            return false;

        rate.sleep();
    }
}

bool RobotCtrl::setRobotConfig( RobotConfiguration& config )
{
    if( !mBoardStatus.saveToEeprom )
        ROS_WARN_STREAM( "'Save to EEPROM' is not enable. Parameter changing will not be permanent!!!");

    memcpy( &mRobotConfig, &config, sizeof(RobotConfiguration) );

    // >>>>> Robot Configuration Data (19 consequtive registers)
    vector<uint16_t> data;
    int nReg = 19;
    data.resize(nReg);
    uint16_t startAddr =  WORD_ROBOT_DIMENSION_WEIGHT;

    data[0]  = mRobotConfig.Weight;
    data[1]  = mRobotConfig.Width;
    data[2]  = mRobotConfig.Height;
    data[3]  = mRobotConfig.Lenght;
    data[4]  = mRobotConfig.WheelBase;
    data[5]  = mRobotConfig.WheelRadiusLeft;
    data[6]  = mRobotConfig.WheelRadiusRight;
    data[7]  = mRobotConfig.EncoderCprLeft;
    data[8]  = mRobotConfig.EncoderCprRight;
    data[9]  = mRobotConfig.MaxRpmMotorLeft;
    data[10] = mRobotConfig.MaxRpmMotorRight;
    data[11] = mRobotConfig.MaxAmpereMotorLeft;
    data[12] = mRobotConfig.MaxAmpereMotorRight;
    data[13] = mRobotConfig.MaxTorqueMotorLeft;
    data[14] = mRobotConfig.MaxTorqueMotorRight;
    data[15] = mRobotConfig.RatioShaftLeft;
    data[16] = mRobotConfig.RatioShaftRight;
    data[17] = mRobotConfig.RatioMotorLeft;
    data[18] = mRobotConfig.RatioMotorRight;

    if( !mRbCtrl->writeMultiReg( startAddr, nReg, data ) )
    {
        ROS_ERROR_STREAM( "Error sendng first part of configuration to RoboController");
        return false;
    }
    // <<<<< Robot Configuration Data (19 consequtive registers)

    // >>>>> Status Register 2
    nReg = 1;
    data.resize(nReg);
    startAddr = WORD_STATUSBIT2;

    uint16_t statusVal = 0;
    if(mRobotConfig.EncoderPosition)
        statusVal |= FLG_STATUSBI2_EEPROM_ENCODER_POSITION;
    if(mRobotConfig.MotorEnableLevel)
        statusVal |= FLG_EEPROM_OUTPUT_DRIVER_ENABLE_POLARITY;

    data[0] = statusVal;

    if( !mRbCtrl->writeMultiReg( startAddr, nReg, data ) )
    {
        ROS_ERROR_STREAM( "Error sending second part of configuration to RoboController");
        return false;
    }
    // <<<<< Status Register 2

    ROS_INFO_STREAM( "Robot parameters sent to RoboController");

    return true;
}

bool RobotCtrl::getRobotConfig( RobotConfiguration& config )
{
    // >>>>> Config Registers
    uint16_t startAddr = WORD_ROBOT_DIMENSION_WEIGHT;
    uint16_t nReg = 19;

    vector<uint16_t> data = mRbCtrl->readMultiReg( startAddr, nReg );

    if( data.size() != nReg )
    {
        ROS_WARN_STREAM( "RC reply for Robot Configuration is incorrect in size, expected " << nReg << ", received " << data.size() );
        return false;
    }
    // <<<<< Config Registers

    config.Weight                 = data[0];
    config.Width                  = data[1];
    config.Height                 = data[2];
    config.Lenght                 = data[3];
    config.WheelBase              = data[4];
    config.WheelRadiusLeft        = data[5];
    config.WheelRadiusRight       = data[6];
    config.EncoderCprLeft         = data[7];
    config.EncoderCprRight        = data[8];
    config.MaxRpmMotorLeft        = data[9];
    config.MaxRpmMotorRight       = data[10];
    config.MaxAmpereMotorLeft     = data[11];
    config.MaxAmpereMotorRight    = data[12];
    config.MaxTorqueMotorLeft     = data[13];
    config.MaxTorqueMotorRight    = data[14];
    config.RatioShaftLeft         = data[15];
    config.RatioShaftRight        = data[16];
    config.RatioMotorLeft         = data[17];
    config.RatioMotorRight        = data[18];
}

bool RobotCtrl::getBoardStatus( BoardStatus& status)
{
    //if( !mBoardStatusUpdated )
    {
        if( !updateBoardStatus() )
            return false;
    }

    memcpy( &status, &mBoardStatus, sizeof(BoardStatus) );

    return true;
}

bool RobotCtrl::setBoardStatus( BoardStatus &status )
{
    uint16_t statusVal = 0x0000;
    if(status.accelRampEnable)
        statusVal |= FLG_STATUSBI1_EEPROM_RAMP_EN;
    if(status.pidEnable)
    {
        mMotorCtrlMode = mcPID;
        statusVal |= FLG_STATUSBI1_PID_EN;
    }
    else
        mMotorCtrlMode = mcDirectPWM;
    if(status.saveToEeprom)
        statusVal |= FLG_STATUSBI1_EEPROM_SAVE_EN;
    if(status.wdEnable)
        statusVal |= FLG_STATUSBI1_COMWATCHDOG;

    vector<uint16_t> data;
    int nReg = 1;
    data.resize(nReg);
    data[0] = statusVal;
    uint16_t startAddr = WORD_STATUSBIT1;

    if( !mRbCtrl->writeMultiReg( startAddr, nReg, data ) )
    {
        ROS_ERROR_STREAM( "Error sending board status to RoboController");

        return false;
    }

    mBoardStatusUpdated = true;

    if( mBoardStatus.pidEnable )
        mMotorCtrlMode = mcPID;
    else
        mMotorCtrlMode = mcDirectPWM;

    ROS_INFO_STREAM( "Board parameters sent to RoboController");

    return true;
}

bool RobotCtrl::enablePID( bool pidEnable, bool rampsEnable  )
{
    if( !mBoardStatus.saveToEeprom )
        ROS_WARN_STREAM( "'Save to EEPROM' is not enable. Parameter changing will not be permanent!!!");

    if( !mBoardStatusUpdated )
    {
        if( !updateBoardStatus() )
            return false;
    }

    mBoardStatus.pidEnable = pidEnable;
    mBoardStatus.accelRampEnable = rampsEnable;

    if( !setBoardStatus( mBoardStatus ) )
    {
        ROS_ERROR_STREAM( "Error setting PID status");
        mBoardStatusUpdated = false;
        return false;
    }

    mBoardStatusUpdated = true;

    ROS_INFO_STREAM( "PID and Ramps status sent to RoboController");
    return true;
}

bool RobotCtrl::enableWD( bool enable, uint16_t wdTime_msec )
{
    if( !mBoardStatus.saveToEeprom )
        ROS_WARN_STREAM( "'Save to EEPROM' is not enable. Parameter changing will not be permanent!!!");

    if( !mBoardStatusUpdated )
    {
        if( !updateBoardStatus() )
            return false;
    }

    mBoardStatus.wdEnable = enable;

    if( !setBoardStatus( mBoardStatus ) )
    {
        ROS_ERROR_STREAM( "Error setting WatchDog status");
        mBoardStatusUpdated = false;
        return false;
    }

    ROS_INFO_STREAM( "Watchdog status sent to RoboController");

    return setWdTimeoutTime( wdTime_msec);
}

bool RobotCtrl::setWdTimeoutTime( uint16_t wdTimeout_msec )
{
    if( !mBoardStatus.saveToEeprom )
        ROS_WARN_STREAM( "'Save to EEPROM' is not enable. Parameter changing will not be permanent!!!");

    vector<uint16_t> data;
    int nReg = 1;
    data.resize(nReg);
    data[0] = wdTimeout_msec;
    uint16_t startAddr = WORD_COMWATCHDOG_TIME;

    if( !mRbCtrl->writeMultiReg( startAddr, nReg, data ) )
    {
        ROS_ERROR_STREAM( "Error setting watchdog communication timeout to RoboController");

        return false;
    }

    ROS_INFO_STREAM( "WatchDog parameters sent to RoboController");

    return true;
}

uint16_t RobotCtrl::getWdTimeoutTime()
{
    // >>>>> Status Register
    uint16_t startAddr = WORD_COMWATCHDOG_TIME;
    uint16_t nReg = 1;

    vector<uint16_t> reply = mRbCtrl->readMultiReg( startAddr, nReg );

    if( reply.size() != nReg )
    {
        ROS_WARN_STREAM( "RC reply for WORD_COMWATCHDOG_TIME is incorrect in size, expected " << nReg << ", received " << reply.size() );
        return false;
    }
    // <<<<< Status Register

    return reply[0];
}

bool RobotCtrl::enableSaveToEeprom( bool enable )
{
    if( !mBoardStatusUpdated )
    {
        if( !updateBoardStatus() )
            return false;
    }

    mBoardStatus.saveToEeprom = enable;

    if( !setBoardStatus( mBoardStatus ) )
    {
        ROS_ERROR_STREAM( "Error setting SaveToEeprom status");
        mBoardStatusUpdated = false;
        return false;
    }

    mBoardStatusUpdated = true;
    return true;
}

bool RobotCtrl::updateBoardStatus()
{
    // >>>>> Status Register
    uint16_t startAddr = WORD_STATUSBIT1;
    uint16_t nReg = 1;

    vector<uint16_t> reply1 = mRbCtrl->readMultiReg( startAddr, nReg );

    if( reply1.size() != nReg )
    {
        ROS_WARN_STREAM( "RC reply for WORD_STATUSBIT1 is incorrect in size, expected " << nReg << ", received " << reply1.size() );
        return false;
    }
    // <<<<< Status Register

    uint16_t value = reply1[0];

    mBoardStatus.pidEnable = value & FLG_STATUSBI1_PID_EN;
    mBoardStatus.wdEnable = value & FLG_STATUSBI1_COMWATCHDOG;
    mBoardStatus.saveToEeprom = value & FLG_STATUSBI1_EEPROM_SAVE_EN;
    mBoardStatus.accelRampEnable = value & FLG_STATUSBI1_EEPROM_RAMP_EN;

    mBoardStatusUpdated = true;

    if( mBoardStatus.pidEnable )
        mMotorCtrlMode = mcPID;
    else
        mMotorCtrlMode = mcDirectPWM;

    return true;
}

bool RobotCtrl::setPidValues( MotorPos mot, uint16_t Kp, uint16_t Ki, uint16_t Kd )
{
    if( !mBoardStatus.saveToEeprom )
        ROS_WARN_STREAM( "'Save to EEPROM' is not enable. Parameter changing will not be permanent!!!");

    vector<uint16_t> data;
    int nReg = 3;
    data.resize(nReg);

    uint16_t startAddr;

    if( mot == motLeft )
        startAddr = WORD_PID_P_LEFT;
    else
        startAddr = WORD_PID_P_RIGHT;

    data[0] = Kp;
    data[1] = Ki;
    data[2] = Kd;

    // >>>>> Status Register 1
    if( !mRbCtrl->writeMultiReg( startAddr, nReg, data ) )
    {
        ROS_ERROR_STREAM( "Error sending PID parameters to RoboController");
        return false;
    }
    // <<<<< Status Register

    ROS_INFO_STREAM( "PID parameters sent to RoboController");
}

bool RobotCtrl::getPidValues( MotorPos mot, uint16_t& Kp, uint16_t& Ki, uint16_t& Kd )
{
    uint16_t startAddr;

    if( mot == motLeft )
        startAddr = WORD_PID_P_LEFT;
    else
        startAddr = WORD_PID_P_RIGHT;

    // >>>>> PID registers
    uint16_t nReg = 3;

    vector<uint16_t> reply = mRbCtrl->readMultiReg( startAddr, nReg );

    if( reply.size() != nReg )
    {
        ROS_WARN_STREAM( "RC reply for PID registers is incorrect in size, expected " << nReg << ", received " << reply.size() );
        return false;
    }
    // <<<<< PID Registers

    Kp = reply[0];
    Ki = reply[1];
    Kd = reply[2];

    return true;
}

bool RobotCtrl::setBattCalibValue( AnalogCalibValue valueType, double curChargeVal_V )
{
    if( !mBoardStatus.saveToEeprom )
        ROS_WARN_STREAM( "'Save to EEPROM' is not enable. Parameter changing will not be permanent!!!");

    vector<uint16_t> data;
    int nReg = 1;
    data.resize(nReg);

    // >>>>> First phase: setting value
    uint16_t charVal = (uint16_t)(curChargeVal_V*1000.0);
    uint16_t startAddr = WORD_VAL_TAR_FS;

    data[0] = charVal;

    if( !mRbCtrl->writeMultiReg( startAddr, nReg, data ) )
    {
        ROS_ERROR_STREAM( "Error setting battery calibration value" );
        return false;
    }
    // <<<<< First phase: setting value

    ros::Duration(0.010).sleep(); // sleep for 10 msec

    // >>>>> Second phase: value imposition
    uint16_t flag = (valueType==CalLow)?0x00001:0x0020;
    startAddr = WORD_FLAG_TARATURA;

    data[0] = flag;

    if( !mRbCtrl->writeMultiReg( startAddr, nReg, data ) )
    {
        ROS_ERROR_STREAM( "Error imposing battery calibration value" );
        return false;
    }
    // <<<<< Second phase: value imposition

    ros::Duration(0.010).sleep(); // sleep for 10 msec

    ROS_INFO_STREAM( "Battery calibration parameters sent to RoboController");

    return true;
}

bool RobotCtrl::setRegister( uint16_t regIdx, uint16_t value )
{
    vector<uint16_t> data;
    int nReg = 1;
    data.resize(nReg);
    data[0] = value;
    uint16_t startAddr = regIdx;

    if( !mRbCtrl->writeMultiReg( startAddr, nReg, data ) )
    {
        ROS_ERROR_STREAM( "Error setting value (%" << value << ") to register #" << regIdx << " on RoboController");

        return false;
    }

    ROS_INFO_STREAM( "WatchDog parameters sent to RoboController");

    return true;
}

vector<uint16_t>  RobotCtrl::getRegisters( uint16_t startAddr, uint16_t nReg )
{
    return mRbCtrl->readMultiReg( startAddr, nReg );
}

}
