# infield_robotics_workshop
This repository contains material for the Infield Robotics Workshop of the VDI-Land.Technik / EurAgEng pre-conference.

content: 
- Requirements
- Installation
  - installation in a new catkin workspace
  - installation in an existing catkin
- Running the Excercises


-------------------------------------------------------------------------------------

Requirements: A system with setup ROS 1 Melodic or higher

-------------------------------------------------------------------------------------

INSTALLATION

installation in a new catkin workspace using catkin_make

source your ROS installation (replace ROS_DISTRO with the ROS_DISTRO you are using (e.g. 'noetic')

$ source /opt/ros/ROS_DISTRO/setup.bash

create new catkin workspace

$ mkdir -p ~/infield_robotics_ws/src

swithc into that workspace

$ cd ~/infield_robotics_ws/src

clone the repository with the tasks

$ git clone https://github.com/ATB-potsdam-automation/infield_robotics_workshop.git

(optional) clone the repository with the solutions

$ git clone https://github.com/ATB-potsdam-automation/infield_robotics_workshop_solutions.git

switch to workspace root folder

$ cd ~/infield_robotics_ws

build the workspace

$ catkin_make

--------------------

installation in an existing catkin

1. source your catkin workspace
2. change directory into your workspaces src folder
3. clone the reposistories
4. build using catkin_make / catkin build

-------------------------------------------------------------------------------------

RUNNING THE EXCERCISES

open a terminal and source your catkin_ws and ROS installation 

$ ~/infield_robotics_ws/devel/setup.bash

launch the data playback

$ roslaunch infield_robotics_workshop workshop.launch

open a second terminal and source your catkin_ws and ROS installation 

$ ~/infield_robotics_ws/devel/setup.bash

run the tasks (here the task1.py)

$ rosrun infield_robotics_workshop task1.py

