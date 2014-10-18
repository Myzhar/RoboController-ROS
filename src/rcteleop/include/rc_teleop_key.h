#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

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

  int mKeyTimeout;

  ros::Publisher mVelPub;
};


