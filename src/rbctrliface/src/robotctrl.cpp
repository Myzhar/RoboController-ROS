#include "robotctrl.h"
#include "modbus_registers.h"

namespace robocontroller
{

#define SPEED_FILTER_SIZE 10

#define _SIGN(X) (((X) >= 0) - ((X) < 0))

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

bool RobotCtrl::getDebugInfo( RcDebug& debug )
{
    uint16_t startAddr = WORD_ENC1_PERIOD;
    uint16_t nReg = 2;

    vector<uint16_t> reply = mRbCtrl->readMultiReg( startAddr, nReg );

    if( reply.size() != nReg+2 )
    {
        ROS_WARN_STREAM( "RC reply for debug is incorrect in size, expected " << nReg+2 << ", received " << reply.size() );
        return false;
    }

    debug.enc1_period = reply[2];
    debug.enc2_period = reply[3];
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
    // Eliminates glitches due to uncorrect speed reading on the RoboController
    if( mSpeedFilterActive)
        applySpeedFilter(telemetry);
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

    if( ((mSpeedMeanLeft != 0) && _SIGN(telemetry.LinSpeedLeft) != _SIGN(mSpeedMeanLeft)) ||
            fabs(telemetry.LinSpeedLeft)<fabs(mSpeedMeanLeft)/2 )
    {
        mMotorSpeedVecLeft[mSpeedLeftVecIdx] = mSpeedMeanLeft;
        replaceLeft = true;
    }
    else
        mMotorSpeedVecLeft[mSpeedLeftVecIdx] = telemetry.LinSpeedLeft;

    if( ((mSpeedMeanRight != 0) &&_SIGN(telemetry.LinSpeedRight)!=_SIGN(mSpeedMeanRight)) ||
            fabs(telemetry.LinSpeedRight)<fabs(mSpeedMeanRight)/2 )
    {
        mMotorSpeedVecRight[mSpeedRightVecIdx] = mSpeedMeanRight;
        replaceRight = true;
    }
    else
        mMotorSpeedVecRight[mSpeedRightVecIdx] = telemetry.LinSpeedRight;

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
    /*
    def online_variance(data):
    n = 0
    mean = 0
    M2 = 0

    for x in data:
        n = n + 1
        delta = x - mean
        mean = mean + delta/n
        M2 = M2 + delta*(x - mean)

    if (n < 2):
        return 0

    variance = M2/(n - 1)
    return variance
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
    double speedL = fwSpeed + 0.5 * rotSpeed * mConfig.WheelBase/1000.0;
    double speedR = fwSpeed - 0.5 * rotSpeed * mConfig.WheelBase/1000.0;

    ROS_DEBUG_STREAM( "fwSpeed: " << fwSpeed << " ; rotSpeed: " << rotSpeed << " m/sec" );
    ROS_DEBUG_STREAM( "speedL: " << speedL << " ; speedR: " << speedR << " m/sec" );

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

}
