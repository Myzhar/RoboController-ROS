#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

//#define KEYCODE_R 0x43
//#define KEYCODE_L 0x44
//#define KEYCODE_U 0x41
//#define KEYCODE_D 0x42
#define KEYCODE_R 0x6a
#define KEYCODE_L 0x69
#define KEYCODE_U 0x67
#define KEYCODE_D 0x6c
#define KEYCODE_Q 0x71

class TeleopRcKey
{
public:
  TeleopRcKey();
  void keyLoop();

private:
  ros::NodeHandle m_nh;

  double mLinear;
  double mAngular;
  double mMaxLin;
  double mMaxAng;

  double mLinStep;
  double mAngStep;

  ros::Publisher mVelPub;
};


