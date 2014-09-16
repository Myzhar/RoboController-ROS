RoboController-ROS
==================

RoboController-ROS allows to the robots based on RoboController board to be controlled by ROS framework

One of the robot based on RoboController board is [MyzharBot](http://myzharbot.robot-home.it)

Installation
------------
* Install **ROS** on a Linux machine (follow the [tutorial](http://wiki.ros.org/ROS/Installation)) or download the latest VM ready for ROS from [Nootrix](http://nootrix.com/downloads/)
* Open a new terminal (*Ctrl + Alt + t*)
* Go to your **ROS workspace** (e.g. *cd ~/catkin_ws*) or create one if you never did it (e.g. *mkdir ~/catkin_ws*)
* Go to **src** folder (i.e. *cd src*)
* Get **RoboController-ROS** code (i.e. *git clone https://github.com/Myzhar/RoboController-ROS.git*)
* Go back to your workspace root (e.g. *cd ~/catkin_ws*)
* Compile the new package (i.e. *catkin_make*)
* Source the new package (i.e. *source devel/setup.bash*)

You are now ready to start *robocontroller_node*:
* *rosrun robocontroller robocontroller_node*

