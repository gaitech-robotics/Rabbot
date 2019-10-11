# Rabbot #
## rabbot\_sim Package ##

rabbot_sim is a gazebo simulator. It provides the robot “rabbot”  model  with simulated sensors(such as the IMU, odometry sensor, and the rplidar, rgbd camra, other sensor  which can be mounted on the robot.), arm and mobile base.
This package contains some controllers, like teleop in an indoor worlds, a joystick controller. Below we provide the instructions necessary for getting started with the navigation in the simulation world.


Installation

Note: Following steps are based on UBUNTU 16.04 with ROS-kinetic

This step assumes that your PC is running with Ubuntu 16 and configured with ROS KINETIC and all required libraries

Install dependencies

      sudo apt install ros-kinetic-controller-manager ros-kinetic-gazebo-ros-control ros-kinetic-teleop-twist-keyboard ros-kinetic-teleop-twist-joy ros-kinetic-joy ros-kinetic-slam-gmapping ros-kinetic-hector-mapping ros-kinetic-hector-trajectory-server ros-kinetic-move-base ros-kinetic-map-server ros-kinetic-amcl ros-kinetic-teb-local-planner ros-kinetic-moveit*

Creating Work Space and cloning Rabbot package

     mkdir –p ~/catkin_ws/src
     cd ~/catkin_ws/src
     git clone https://github.com/gaitech-robotics/Rabbot.git
     cd ~/catkin_ws && catkin_make



Add env variable

       sudo echo "export USE_EKF='true'" >> ~/.bashrc


Basic Usage

First, launch the simulation by use the following:

    $roslaunch rabbot_sim gazebo.launch

Note The first run of gazebo might take considerably long, as it will download some models from an online database.

Getting the robot to move

To let the robot move you need to send velocity command,  There are currently a few ways to send commands to the robot, we will show them here.

• Send direct velocities commands

We will for now just send some constant command velocities to the robot by:

     $rostopic pub /rabbot_base_controller/cmd_vel geometry_msgs/Twist "linear: x: 1.0 y: 0.0 z: 0.0 angular: x: 0.0 y: 0.0 z: 0.0"  



•  Teleop using keyboard

Now, we are goining to send the command to the robot via keypoard by typing the following:

        $ roslaunch rabbot_sim rabbot_keyboard.launch



•  Teleop using joystick

Connect a USB joystick to your computer and launch the file(Please check the js port number,default is js1. You can change in the launch file.)   
Now, we are goining to send the command to the robot via joystick by typing the following:

        $ roslaunch rabbot_sim rabbot_joystick.launch




Navigation

Create a map using slam_gmapping:

First launch the robot in the  simulation world by:

       $ roslaunch rabbot_sim gazebo.launch

Now launch the keyboard control node:

       $ roslaunch rabbot_sim rabbot_keyboard.launch

Further, we need to launch the gmapping slam by:

       $roslaunch rabbot_navigation rabbot_gmapping.launch

And start move around in the whole environment, open rviz and visualize.







The last step you have to save the map using the following:

      $ rosrun map_server map_saver -f office_map


It is recommended to save the map in the path:

        ~/catkin_ws/src/rabbot/rabbot_navigation/maps


The last step is to launch the navigation file by:

      $ roslaunch rabbot_navigation rabbot_navigation.launch  


## rabbot\_moveit\_config\_gazebo Package ##




Note: Gazebo Version,
It is recommended to install at least Gazebo v7.x for full functionlity, (which is installed by default with ROS Kinetic).

For more demos, please visit the official wiki here
