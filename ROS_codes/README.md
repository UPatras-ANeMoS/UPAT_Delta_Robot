UPatras - Delta robot control
=============

## Basic Usage

1.If not already, install  and initialize ROS indigo desktop full (http://wiki.ros.org/indigo/Installation/Ubuntu) and phidget libraries (http://www.phidgets.com/docs/OS_-_Linux#Installing)

2.Copy the folders into your workspace.

3.Build your workspace:

    $ cd ∼/delta    
    $ catkin_make    

4a.Run the launch file (a new terminal will be opened where you can interact and control the movement of the robot):

    $ roslaunch phidget delta.launch
    
    
4b.Alternatively, run each line in a new terminal

    $ roscore    
    $ rosrun arduino arduino
    $ rosrun phidget phidget
