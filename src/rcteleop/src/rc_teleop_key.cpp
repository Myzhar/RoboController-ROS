#include <rc_teleop_key.h>
#include <geometry_msgs/Twist.h>
#include <ncurses.h>

TeleopRcKey::TeleopRcKey():
    mLinear(0.0),
    mAngular(0.0),
    mMaxLin(1.0),
    mMaxAng(3.14),
    mLinStep(0.1),
    mAngStep(0.314)
{
    mVelPub = m_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // TODO set teleoperation variables with parameters
}

void TeleopRcKey::keyLoop()
{
    int c;
    bool dirty=false;

    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    timeout(100);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the robot.");

    while(1)
    {
        dirty = false;

        c = getch();

        ROS_INFO_STREAM("value: " << c << "\r");

        switch(c)
        {
        case KEY_LEFT:
        {
            ROS_DEBUG("LEFT");
            mAngular += mAngStep;
            dirty = true;
            break;
        }
        case KEY_RIGHT:
        {
            ROS_DEBUG("RIGHT");
            mAngular -= mAngStep;
            dirty = true;
            break;
        }
        case KEY_UP:
        {
            ROS_DEBUG("UP");
            mLinear += mLinStep;
            dirty = true;
            break;
        }
        case KEY_DOWN:
        {
            ROS_DEBUG("DOWN");
            mLinear -= mLinStep;
            dirty = true;
            break;
        }
        case 'q':
        {
            endwin();
            exit(0);
        }
        default:
        {
            if( mLinear > 0 )
            {
                mLinear -= mLinStep;
                dirty = true;
            }
            else if( mLinear < 0  )
            {
                mLinear += mLinStep;
                dirty = true;
            }


            if( mAngular > 0 )
            {
                mAngular -= mAngStep;
                dirty = true;
            }
            else if( mAngular < 0  )
            {
                mAngular += mAngStep;

                dirty = true;
            }
        }
        }

        ROS_INFO_STREAM( "mLinear: " << mLinear << " - mAngular: " << mAngular << "\r");

        geometry_msgs::Twist vel;

        vel.linear.x = mLinear;
        vel.angular.z = mAngular;

        if(dirty==true)
        {
            mVelPub.publish(vel);
            dirty=false;
        }
    }


    return;
}




