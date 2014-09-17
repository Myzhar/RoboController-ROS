#include "robotctrl.h"
#include "modbus_registers.h"

#define SPEED_FILTER_SIZE 10

RobotCtrl::RobotCtrl(ros::NodeHandle* nh, RbCtrlIface *rbCtrl)
{
    mNodeH = nh;
    mMotStopped = true;

    mRbCtrl = rbCtrl;

    mPose.x = 0.0;
    mPose.y = 0.0;
    mPose.theta = 0.0;

    mSpeedFilterActive = true;

    initSpeedFilter();
}

bool RobotCtrl::getTelemetry( RobotTelemetry& telemetry)
{
    // >>>>> Telemetry update
    // WORD_TENSIONE_ALIM 8
    // WORD_ENC1_SPEED 20
    // WORD_ENC2_SPEED 21
    // WORD_RD_PWM_CH1 22
    // WORD_RD_PWM_CH2 23

    uint16_t startAddr = WORD_ENC1_SPEED;
    uint16_t nReg = 4;

    vector<uint16_t> reply = mRbCtrl->readMultiReg( startAddr, nReg );

    if( reply.size() != nReg+2 )
    {
        ROS_WARN_STREAM( "RC reply for motors is incorrect in size, expected " << nReg+2 << ", received " << reply.size() );
        return false;
    }

    double speed0;
    if(reply[2] < 32767)  // Speed is integer 2-complement!
        speed0 = ((double)reply[2])/1000.0;
    else
        speed0 = ((double)(reply[2]-65536))/1000.0;
    telemetry.LinSpeedLeft = speed0;

    double speed1;
    if(reply[3] < 32767)  // Speed is integer 2-complement!
        speed1 = ((double)reply[3])/1000.0;
    else
        speed1 = ((double)(reply[3]-65536))/1000.0;
    telemetry.LinSpeedRight = speed1;

    telemetry.PwmLeft = reply[4];
    telemetry.PwmRight = reply[5];

    // TODO mTelemetry.RpmLeft = // CALCULATE!!!
    // TODO mTelemetry.RpmRight = // CALCULATE!!!

    startAddr = WORD_TENSIONE_ALIM;
    nReg = 1;
    reply = mRbCtrl->readMultiReg( startAddr, nReg );

    if( reply.size() != nReg+2 )
    {
        ROS_WARN_STREAM( "RC reply for battery is incorrect in size, expected " << nReg+2 << ", received " << reply.size() );
        return false;
    }

    telemetry.Battery = ((double)reply[2])/1000.0;

    // >>>>> Speed Filter
    if( mSpeedFilterActive)
    {
        double speedLeftMean, speedRightMean;

        speedLeftMean = mMotorSpeedLeftSum/mSpeedCount;
        speedRightMean = mMotorSpeedRightSum/mSpeedCount;

        // >>>>> Evaluate Thresholds
        double threshLeft, threshRight;

        threshLeft = fabs( speedLeftMean/10.0 );
        threshRight = fabs( speedRightMean/10.0 );
        // <<<< Evaluate Thresholds

        if( fabs(telemetry.LinSpeedLeft-speedLeftMean)>threshLeft )
        {
            telemetry.LinSpeedLeft = speedLeftMean;
        }

        if( fabs(telemetry.LinSpeedRight-speedRightMean)>threshRight )
        {
            telemetry.LinSpeedRight = speedRightMean;
        }

        if( mSpeedCount==SPEED_FILTER_SIZE )
        {
            mMotorSpeedLeftSum -= mMotorSpeedVecLeft[speedVecIdx];
            mMotorSpeedRightSum -= mMotorSpeedVecRight[speedVecIdx];
        }
        else
            mSpeedCount++;

        mMotorSpeedLeftSum += telemetry.LinSpeedLeft;
        mMotorSpeedRightSum += telemetry.LinSpeedRight;

        mMotorSpeedVecLeft[speedVecIdx] = telemetry.LinSpeedLeft;
        mMotorSpeedVecRight[speedVecIdx] = telemetry.LinSpeedRight;

        speedVecIdx++;
        speedVecIdx %= SPEED_FILTER_SIZE;
    }
    // <<<<< Speed Filter

    memcpy( &mTelemetry, &telemetry, sizeof(RobotTelemetry) );

    double v = (telemetry.LinSpeedLeft + telemetry.LinSpeedRight)/2.0;
    double omega = (telemetry.LinSpeedLeft + telemetry.LinSpeedRight)/((double)mConfig.WheelBase/1000.0);

    ros::Time now = ros::Time::now();

    double dt = (now - mLastTelemTime).toSec();
    mLastTelemTime = now;

    mPose.theta += omega * dt;
    mPose.x += v * cos( mPose.theta ) * dt;
    mPose.y += v * sin( mPose.theta ) * dt;

    return true;
}

void RobotCtrl::getPose( RobotPose& pose)
{
    memcpy( &pose, &mPose, sizeof(RobotPose) );
}

bool RobotCtrl::getMotorSpeeds(double& speedL, double& speedR )
{
    uint16_t startAddr = WORD_ENC1_SPEED;
    uint16_t nReg = 2;

    vector<uint16_t> reply = mRbCtrl->readMultiReg( startAddr, nReg );

    if( reply.size() != nReg+2 )
    {
        ROS_WARN_STREAM( "RC reply for motor speeds is incorrect in size, expected " << nReg+2 << ", received " << reply.size() );
        return false;
    }


    if(reply[2] < 32767)  // Speed is integer 2-complement!
        speedL = ((double)reply[2])/1000.0;
    else
        speedL = ((double)(reply[2]-65536))/1000.0;

    if(reply[3] < 32767)  // Speed is integer 2-complement!
        speedR = ((double)reply[3])/1000.0;
    else
        speedR = ((double)(reply[3]-65536))/1000.0;

    mTelemetry.LinSpeedLeft = speedL;
    mTelemetry.LinSpeedRight = speedR;

    return true;
}

void RobotCtrl::initSpeedFilter()
{
    // >>>>> Speed Filter Initialization
    mMotorSpeedVecLeft.resize( SPEED_FILTER_SIZE );
    mMotorSpeedVecRight.resize( SPEED_FILTER_SIZE );
    mMotorSpeedLeftSum = 0.0;
    mMotorSpeedRightSum = 0.0;
    mSpeedCount = 0;
    speedVecIdx = 0;
    // <<<<< Speed Filter Initialization
}

bool RobotCtrl::setRobotSpeed( double fwSpeed, double rotSpeed )
{
    double speedL = fwSpeed + 0.5 * rotSpeed * mConfig.WheelBase/1000.0;
    double speedR = fwSpeed - 0.5 * rotSpeed * mConfig.WheelBase/1000.0;

    ROS_DEBUG_STREAM( "fwSpeed: " << fwSpeed << " ; rotSpeed: " << rotSpeed << " m/sec" );
    ROS_DEBUG_STREAM( "speedL: " << speedL << " ; speedR: " << speedR << " m/sec" );

    return setMotorSpeeds( speedL, speedR );
}

bool RobotCtrl::setMotorSpeeds( double speedL, double speedR )
{
    // TODO Verify that RoboController is in PID mode (make a service for the status)

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

    // >>>>> New SetPoint to RoboController
    uint16_t address = WORD_PWM_CH1;

    vector<uint16_t> data;
    data.resize(2);

    uint16_t sp; // Speed is integer 2-complement!
    if(speedL >= 0)
        sp = (uint16_t)(speedL*1000.0);
    else
        sp = (uint16_t)(speedL*1000.0+65536.0);

    //uint16_t sp = (uint16_t)(speed*1000.0+32767.5);
    data[0] = sp;

    if(speedR >= 0)
        sp = (uint16_t)(speedR*1000.0);
    else
        sp = (uint16_t)(speedR*1000.0+65536.0);

    //uint16_t sp = (uint16_t)(speed*1000.0+32767.5);
    data[1] = sp;

    bool commOk = mRbCtrl->writeMultiReg( address, 2, data );
    // <<<<< New SetPoint to RoboController

    if( commOk )
    {
        if( speedL!=0.0 && speedL!=0 )
            mMotStopped = false;
        else
            mMotStopped = true;
    }

    return commOk;
}

bool RobotCtrl::stopMotors()
{
    int count = 0;

    initSpeedFilter();

    ros::Rate rate( 30 );
    while( 1 ) // Try to stop the motors 5 times for security!
    {
        if( setMotorSpeeds( 0.0, 0.0) )
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
