#include <rc_teleop_key.h>

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_keyboard");

  TeleopRcKey teleop_turtle;

  signal(SIGINT,quit);

  teleop_turtle.keyLoop();

  return(0);
}
