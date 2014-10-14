#include <rc_teleop_key.h>
#include <geometry_msgs/Twist.h>

extern int kfd;
extern struct termios cooked, raw;

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
    char c;
    bool dirty=false;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);

    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the turtle.");

    for(;;)
    {
        // get the next event from the keyboard
        if(read(kfd, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }

        ROS_DEBUG("value: 0x%02X\n", c);

        switch(c)
        {
        case KEYCODE_L:
        {
            ROS_DEBUG("LEFT");
            mAngular += mAngStep;
            dirty = true;
            break;
        }
        case KEYCODE_R:
        {
            ROS_DEBUG("RIGHT");
            mAngular -= mAngStep;
            dirty = true;
            break;
        }
        case KEYCODE_U:
        {
            ROS_DEBUG("UP");
            mLinear += mLinStep;
            dirty = true;
            break;
        }
        case KEYCODE_D:
        {
            ROS_DEBUG("DOWN");
            mLinear -= mLinStep;
            dirty = true;
            break;
        }
        default:
        {
            if( mLinear > 0 )
                mLinear -= mLinStep;
            else if( mLinear < 0  )
                mLinear += mLinStep;

            if( mAngular > 0 )
                mAngular -= mAngStep;
            else if( mAngular < 0  )
                mAngular += mAngStep;
        }
        }



        geometry_msgs::Twist vel;

        vel.linear.x = mLinear;
        vel.angular.z = mAngular;

        if(dirty ==true)
        {
            mVelPub.publish(vel);
            dirty=false;
        }
    }


    return;
}




