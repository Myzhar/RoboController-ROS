RoboController-ROS
==================

**RoboController-ROS** allows to the robots based on **RoboController board** to be controlled by ROS framework.
RoboController-ROS on ROS Indigo.

There are two robots controlled by RoboController board:
* **MyzharBot** [Official Blog](http://myzharbot.robot-home.it) - [Officine Robotiche](http://2014.officinerobotiche.it/myzharbot/)
* **WoodyBot** [Official Blog](http://tuttoelettronica.net/archives/755) - [Officine Robotiche](http://2014.officinerobotiche.it/woodybot/)

Installation
------------
* Install **ROS** on a Linux machine (follow the [tutorial](http://wiki.ros.org/ROS/Installation)) or download the latest VM ready for ROS from [Nootrix](http://nootrix.com/downloads/)
* Open a new terminal (*Ctrl + Alt + t*)
* Install **libModbus** (i.e. *sudo apt-get install libmodbus-dev*)
* Go to your **ROS workspace** (e.g. *cd ~/catkin_ws* on the Virtual Machine) or create one if you never did it before entering (e.g. *mkdir ~/catkin_ws* then *cd ~/catkin_ws*)
* Create **src** directory in the workspace if it does not exist (i.e. *mkdir src*)
* Go to **src** folder (i.e. *cd src*)
* Get **RoboController-ROS** code (i.e. *git clone https://github.com/Myzhar/RoboController-ROS.git*)
* Go back to your workspace root (e.g. *cd ~/catkin_ws*)
* Compile the new package (i.e. *catkin_make*) [Note: *catkin_make* command must be launched always in the root of your workspace!]
* Source the new package (i.e. *source devel/setup.bash*) [Note: you can add *source ~<your_ws>/devel/setup.bash* at the end of your *~/.bashrc* file to avoid to type the command each time you open a new terminal]

Usage
-----
You are now ready to start *robocontroller_node*:
* *rosrun robocontroller robocontroller_node* to run the node with default params
* *roslaunch robocontroller robocontroller_simul.launch* to run the node in simulation mode
* *roslaunch robocontroller robocontroller_myzharbot.launch* to run the node with parameters for MyzharBot
* copy and modify *yaml* files in *config* folder to configure your robot
* you can control the robot using Qt widget provided by **rqt** package: (i.e. launch *rqt*, add a *Robot Steering" widget, the message should be * /cmd_vel *)
* you can plot the Telemetry and the Pose of the robot adding one or more **plot** widgets and subscribing to the messages * /robocontroller/Telemetry * and * /robocontroller/Pose *

