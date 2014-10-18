#include <rc_teleop_key.h>
#include <geometry_msgs/Twist.h>
#include <ncurses.h>
#include <math.h>
#include <string>

using namespace std;

TeleopRcKey::TeleopRcKey():
    mLinear(0.0),
    mAngular(0.0),
    mMaxLin(1.0),
    mMaxAng(3.14),
    mLinStep(0.1),
    mAngStep(0.314),
    mSpeedRatio(1.0),
    mKeyTimeout(100)
{
    mVelPub = m_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // >>>>> Parameters
    string nodeName = ros::this_node::getName();
    string nameSpace = ros::this_node::getNamespace();

    string paramStr;

    paramStr = ( nameSpace + nodeName + "/teleop_key/Max_linear");
    if(m_nh.hasParam( paramStr ))
        m_nh.getParam(paramStr, mMaxLin);
    else
        m_nh.setParam(paramStr, mMaxLin );

    paramStr = ( nameSpace + nodeName + "/teleop_key/Max_angular");
    if(m_nh.hasParam( paramStr ))
        m_nh.getParam(paramStr, mMaxAng);
    else
        m_nh.setParam(paramStr, mMaxAng );

    paramStr = ( nameSpace + nodeName + "/teleop_key/Lin_step");
    if(m_nh.hasParam( paramStr ))
        m_nh.getParam(paramStr, mMaxAng);
    else
        m_nh.setParam(paramStr, mMaxAng );

    paramStr = ( nameSpace + nodeName + "/teleop_key/Ang_step");
    if(m_nh.hasParam( paramStr ))
        m_nh.getParam(paramStr, mAngStep);
    else
        m_nh.setParam(paramStr, mAngStep );

    paramStr = ( nameSpace + nodeName + "/teleop_key/Key_timeout");
    if(m_nh.hasParam( paramStr ))
        m_nh.getParam(paramStr, mKeyTimeout);
    else
        m_nh.setParam(paramStr, mKeyTimeout );
    // <<<<< Parameters

}

void TeleopRcKey::keyLoop()
{
    // >>>>> nCurses initization
    initscr();
    keypad(stdscr, TRUE);
    cbreak();
    noecho();
    timeout(mKeyTimeout);
    // <<<<< nCurses initization

    int c;
    bool dirty=false;

    c = getch();

    ROS_INFO_STREAM("-----------------------------------\r");
    ROS_INFO_STREAM("      Keyboard teleoperation       \r");
    ROS_INFO_STREAM("-----------------------------------\r");
    ROS_INFO_STREAM("- Use arrow keys to move the robot.\r");
    ROS_INFO_STREAM("- Press SPACEBAR to stop the robot.\r");
    ROS_INFO_STREAM("- Press Q to exit.\r");
    ROS_INFO_STREAM("- Press 1 for Max_speed.\r");
    ROS_INFO_STREAM("- Press 2 for Max speed/2.\r");
    ROS_INFO_STREAM("- Press 3 for Max speed/3.\r");
    ROS_INFO_STREAM("-----------------------------------\r");

    bool stop = false;

    while(!stop)
    {
        dirty = false;

        c = getch();

        double linStep = mLinStep*mSpeedRatio;
        double angStep = mAngStep*mSpeedRatio;

        ROS_DEBUG_STREAM("Key pressed: " << c << "\r");

        switch(c)
        {
        case KEY_LEFT:
        {
            ROS_DEBUG_STREAM("LEFT\r");
            mAngular -= angStep;
            dirty = true;
            break;
        }
        case KEY_RIGHT:
        {
            ROS_DEBUG_STREAM("RIGHT\r");
            mAngular += angStep;
            dirty = true;
            break;
        }
        case KEY_UP:
        {
            ROS_DEBUG_STREAM("UP\r");
            mLinear += linStep;
            dirty = true;
            break;
        }
        case KEY_DOWN:
        {
            ROS_DEBUG_STREAM("DOWN\r");
            mLinear -= linStep;
            dirty = true;
            break;
        }
        case ' ':
        {
            ROS_DEBUG_STREAM("STOP\r");
            mLinear = 0.0;
            mAngular = 0.0;
            dirty = true;
            break;
        }
        case '1':
        {
            mSpeedRatio = 1.0;
            dirty = true;
            break;
        }
        case '2':
        {
            mSpeedRatio = 0.5;
            dirty = true;
            break;
        }
        case '3':
        {
            mSpeedRatio = 0.333333;
            dirty = true;
            break;
        }
        case 'q':
        case 'Q':
        {
            ROS_DEBUG_STREAM("EXIT\r");
            stop = true;
        }
        default:
        {
            if( mLinear > 0 )
            {
                mLinear -= linStep;
                dirty = true;
            }
            else if( mLinear < 0  )
            {
                mLinear += linStep;
                dirty = true;
            }


            if( mAngular > 0 )
            {
                mAngular -= angStep;
                dirty = true;
            }
            else if( mAngular < 0  )
            {
                mAngular += angStep;

                dirty = true;
            }
        }
        }

        geometry_msgs::Twist vel;

        // >>>>> Saturations
        if( mLinear > mMaxLin*mSpeedRatio )
            mLinear = mMaxLin*mSpeedRatio;
        else if( mLinear < -mMaxLin*mSpeedRatio )
            mLinear = -mMaxLin*mSpeedRatio;

        if( mAngular > mMaxAng*mSpeedRatio )
            mAngular = mMaxAng*mSpeedRatio;
        else if( mAngular < -mMaxAng*mSpeedRatio )
            mAngular = -mMaxAng*mSpeedRatio;

        if( fabs(mLinear) < 0.01 )
            mLinear = 0.0;

        if( fabs(mAngular) < 0.01 )
            mAngular = 0.0;

        vel.linear.x = mLinear;
        vel.angular.z = mAngular;
        // <<<<< Saturations

        if(dirty==true)
        {
            ROS_INFO_STREAM( "Robot speed - Linear: " << mLinear << " - Angular: " << mAngular << "\r");
            mVelPub.publish(vel);
            dirty=false;
        }
    }

    endwin();

    ROS_INFO_STREAM("---------------------------\r");
    ROS_INFO_STREAM("STOPPED BY USER\r");
    ROS_INFO_STREAM("---------------------------\r");

    return;
}




