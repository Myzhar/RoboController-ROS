#include "robotctrl.h"
#include "modbus_registers.h"

namespace robocontroller
{

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
    ROS_WARN_STREAM( "Before filtering: "<< mSpeedLeftCount << " "  << mSpeedRightCount << " "  << telemetry.LinSpeedLeft << " " << telemetry.LinSpeedRight );

    if( mSpeedFilterActive)
    {
        // >>>>> Evaluate Thresholds
        double threshLeft, threshRight;

        threshLeft = mSpeedVarLeft*5.0;
        threshRight = mSpeedVarRight*5.0;
        // <<<< Evaluate Thresholds

        if( telemetry.LinSpeedLeft > 0.01)
        {
            if( mSpeedLeftCount > 3 && fabs(telemetry.LinSpeedLeft-mSpeedMeanLeft)>threshLeft )
            {
                telemetry.LinSpeedLeft = mSpeedMeanLeft;
            }

            if( mSpeedLeftCount<SPEED_FILTER_SIZE )
                mSpeedLeftCount++;

            mMotorSpeedVecLeft[mSpeedLeftVecIdx] = telemetry.LinSpeedLeft;

            mSpeedLeftVecIdx++;
            mSpeedLeftVecIdx %= SPEED_FILTER_SIZE;
        }

        if( telemetry.LinSpeedRight > 0.01)
        {
            if( mSpeedRightCount > 3 && fabs(telemetry.LinSpeedRight-mSpeedMeanRight)>threshRight )
            {
                telemetry.LinSpeedRight = mSpeedMeanRight;
            }

            if( mSpeedRightCount<SPEED_FILTER_SIZE )
                mSpeedRightCount++;

            mMotorSpeedVecRight[mSpeedRightVecIdx] = telemetry.LinSpeedRight;

            mSpeedRightVecIdx++;
            mSpeedRightVecIdx %= SPEED_FILTER_SIZE;
        }
    }

    ROS_WARN_STREAM( "After filtering: " << mSpeedLeftCount << " "  << mSpeedRightCount << " "  << telemetry.LinSpeedLeft << " " << telemetry.LinSpeedRight );
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

    for( int i=mSpeedLeftVecIdx; i<mSpeedLeftVecIdx+mSpeedLeftCount; i++ )
    {
        int idx = i%SPEED_FILTER_SIZE;
        n++;
        double delta = mMotorSpeedVecLeft[idx] - mSpeedMeanLeft;
        mSpeedMeanLeft += delta/n;

        M2 += delta*(mMotorSpeedVecLeft[idx] - mSpeedMeanLeft);
    }

    if(n<2)
        mSpeedVarLeft = 0.0;
    else
        mSpeedVarLeft = M2/(n - 1);
    // <<<<< Left Motor

    // >>>>> Right Motor
    n=0;
    M2=0.0;
    mSpeedMeanRight=0.0;

    for( int i=mSpeedRightVecIdx; i<mSpeedRightVecIdx+mSpeedRightCount; i++ )
    {
        int idx = i%SPEED_FILTER_SIZE;
        n++;
        double delta = mMotorSpeedVecRight[idx] - mSpeedMeanRight;
        mSpeedMeanRight += delta/n;

        M2 += delta*(mMotorSpeedVecRight[idx] - mSpeedMeanRight);
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

}
