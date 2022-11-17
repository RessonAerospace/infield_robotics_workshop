# infield_robotics_workshop
## About
This repository contains material for the Infield Robotics Workshop of the VDI-Land.Technik / EurAgEng pre-conference.

<details open="open">
<summary>Table of Contents</summary>

- [About](#about)
- [Requirements](#requirements)
- [Installation](#installation)
    - [installation in a new catkin workspace](#installation-in-a-new-catkin-workspace)
    - [installation in an existing catkin workspace](#installation-in-an-existing-catkin-workspace)
- [Running the Excercises](#running-the-excercises)
</details>

-------------------------------------------------------------------------------------
## Requirements

A system with setup ROS 1 Melodic or higher (tested for Melodic and Noetic)

-------------------------------------------------------------------------------------

## INSTALLATION

### installation in a new catkin workspace 

exemplary installation using using catkin_make

source your ROS installation (replace ROS_DISTRO with the ROS_DISTRO you are using (e.g. 'noetic')
```sh
source /opt/ros/ROS_DISTRO/setup.bash
```

create new catkin workspace
```sh
mkdir -p ~/infield_robotics_ws/src
```

swithc into that workspace
```sh
cd ~/infield_robotics_ws/src
```

clone the repository with the tasks
```sh
git clone https://github.com/ATB-potsdam-automation/infield_robotics_workshop.git
```

(optional) clone the repository with the solutions
```sh
git clone https://github.com/ATB-potsdam-automation/infield_robotics_workshop_solutions.git
```

switch to workspace root folder
```sh
cd ~/infield_robotics_ws
```

build the workspace
```sh
catkin_make
```

--------------------

### installation in an existing catkin workspace

1. source your catkin workspace
2. change directory into your workspaces src folder
3. clone the reposistories
4. build using catkin_make / catkin build

-------------------------------------------------------------------------------------

## RUNNING THE EXCERCISES

open a terminal and source your catkin_ws and ROS installation
```sh
~/infield_robotics_ws/devel/setup.bash
```

launch the data playback
```sh
roslaunch infield_robotics_workshop workshop.launch
```

open a second terminal and source your catkin_ws and ROS installation 
```sh
~/infield_robotics_ws/devel/setup.bash
```

run the tasks contained in the 'scripts' foulder (here the task1.py)
```sh
rosrun infield_robotics_workshop task1.py
```
