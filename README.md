# Team 39 Submission

## Video
video/Team39_TortoiseBot.mp4

## How to run
- Clone the repository
- Run the following commands
```
$ sudo apt install ros-humble-joint-state-publisher ros-humble-robot-state-publisher ros-humble-cartographer ros-humble-cartographer-ros ros-humble-gazebo-plugins ros-humble-teleop-twist-keyboard  ros-humble-teleop-twist-joy ros-humble-xacro ros-humble-nav2* ros-humble-urdf

$ cd tortoisebot

$ colcon build

$ source install/setup.bash

# Run the next command in all terminals
$ export NUM_ROBOTS=3

# Run the following in different terminals
$ ros2 launch tortoisebot_description display.launch.py use_sim_time:=true

$ ros2 launch tortoisebot_slam cartographer.launch.py use_sim_time:=true

$ ros2 run tortoisebot_server map_merge
```

## Modifications to the existing Tortoisebot code

1. **Modification to Robot xacro  and Gazebo files:**
    - Addition of namespaces to all the links and joints
    <!-- - Remapping of topics to include  -->
2. **Modification to Launch files:**
    - `display.launch.py`
    - `state_publisher.launch.py`
    - `Gazebo.launch.py`
    - To generalize the application for any given number of robots, all the deployments are put in a for loop that runs for `NUM_ROBOTS` times
    - Remapped all the sensor ooutputs to comply with the namespaces

3. **Created a `tortoise_server` package:**
    - Created a `map_merge` node that takes individual submaps as input and merges them to create a more accurate estimation of the complete map



## Approach

1. **Spawing multiple robots in the environment:**
    - Environment variable `NUM_ROBOTS` is used to set the number of robots in the environment
    - Every bot has it's own cartographer node.
    - The maps published by each cartographer are available at `/tortoisebot_simple_i/map` where `i` denotes the index of the bot (0,1 or 2)
    - Status - Completed in ROS

2. **Map Merger:**
    - **Approach 1:** 
        - Directly overlay all the three maps after converting them to the world frame.
        - Status: Implemented in ROS
    - **Approach 2:**
        - The Occupancy grids generated by individual SLAM nodes are converted into images such that the obstacle boundaries are shown in black, unoccupied cells in white while unknown/ unexplored cells are shown in grey (here, the shade of grey used for unknown cells is closer to white for better feature extraction)
        - SIFT based image sticher is used to extract and match the features in various images that cover overlapping areas for better estimation of the final map.
        - Status: yet to be integrated with ROS
    
3. **Navigation:**
    - This task is currently replaced with manual control for testing
    - The robots are assigned navigation tasks based on the positions of other robots and explored map. The mentioned data is used to calculate a probabilty of the area being explored by other robots later in time. The tasks are reassigned when map is updated upon encounter.
    
4. **Robot Detection:**
    - In order to collaboratively manuever and map the given space, it is crucial that the robots can detect and communicate with one another
    - The detection can be done by using Computer Vision and visual markers like color or AruCo markers.

5. **Collaborative relative pose Estimation:**
    - Since we are not using any predefined location marker, the pose estimation using odometry will have a deviation increasing with time. In such a case, the relative position of a detected robot can be used to refine the localization estimations. 
    - One way to do this Estimation is **Collaborative Multi-Robot Monte Carlo Localization**. When the bots detect each other, they share the sensor data i.e. the map and the _beleif of_ other bot's relative postion. This information is used to correct the possible errors in both the systems.


# Tortoisebot ROS2 Humble Release

# ![TortoiseBot Banner](https://github.com/rigbetellabs/tortoisebot_docs/raw/master/imgs/packaging/pack_front.png)

![stars](https://img.shields.io/github/stars/rigbetellabs/tortoisebot?style=for-the-badge)
![forks](https://img.shields.io/github/forks/rigbetellabs/tortoisebot?style=for-the-badge)
![watchers](https://img.shields.io/github/watchers/rigbetellabs/tortoisebot?style=for-the-badge)
![repo-size](https://img.shields.io/github/repo-size/rigbetellabs/tortoisebot?style=for-the-badge)
![contributors](https://img.shields.io/github/contributors/rigbetellabs/tortoisebot?style=for-the-badge)

---
<p align="center"><a href="#connect-with-us-">Connect with Us</a> • <a href="#1-installation">Installation</a> • <a href="#2-setup">Setup</a> • <a href="#3-demos">Demos</a>

<h1 align="center"> TortoiseBot </h1>

# Connect with us ![some-changes](https://img.shields.io/badge/some_changes-yellow)

<a href="https://rigbetellabs.com/">![Website](https://img.shields.io/website?down_color=lightgrey&down_message=offline&label=Rigbetellabs%20Website&style=for-the-badge&up_color=green&up_message=online&url=https%3A%2F%2Frigbetellabs.com%2F)</a>
<a href="https://rigbetellabs.com/discord">![Discord Channel](https://img.shields.io/discord/890669104330063903?logo=Discord&style=for-the-badge)</a>
<a href="https://www.youtube.com/channel/UCfIX89y8OvDIbEFZAAciHEA">![Youtube Subscribers](https://img.shields.io/youtube/channel/subscribers/UCfIX89y8OvDIbEFZAAciHEA?label=YT%20Subscribers&style=for-the-badge)</a>
<a href="https://www.instagram.com/rigbetellabs/">![Instagram](https://img.shields.io/badge/Follow_on-Instagram-pink?style=for-the-badge&logo=appveyor?label=Instagram)</a>
# 1. Installation
## 1.1 Required Dependences: 
```
sudo apt install ros-humble-joint-state-publisher ros-humble-robot-state-publisher ros-humble-cartographer ros-humble-cartographer-ros ros-humble-gazebo-plugins ros-humble-teleop-twist-keyboard  ros-humble-teleop-twist-joy ros-humble-xacro ros-humble-nav2* ros-humble-urdf 

```
```
cd ~/your workscpace
colcon build
```
## 1.2 Clone this repo 
Make sure you clone the repo in your robot and your remote PC 
```
git clone -b ros2-humble https://github.com/rigbetellabs/tortoisebot.git
```
```
cd ~/your workscpace
colcon build
```
# 2. Setup

- Run bringup.launch.py to only spawn the robot
- Run autobringup.launch.py to spawn the robot with navigation and slam/localization
- Launch the files with use_sim_time:=False when working on real robot

### 2.1 Launching the robot

```
ros2 launch tortoisebot_bringup autobringup.launch.py use_sim_time:=True exploration:=True
```
- exploration:=False for passed a saved map to navigation

### 2.2 Launch files for reference
#### SLAM
- cartographer.launch.py
#### Navigation
- navigation.launch.py
#### Rviz
- rviz.launch.py
#### Gazebo
- gazebo.launch.py

### 2.3 Remote PC

While performing colcon build on remote-pc please add the below to ignore `ydlidar_sdk, ydlidar_ros2_driver, v4l2_camera` since lidar will not be connected to remote-pc.

```
colcon build --packages-ignore ydlidar_sdk ydlidar_ros2_driver v4l2_camera
```

# 3. Demos

<!-- Simulation | Visualisation of Sensors (Lidar, Odometery, Camera) 
:-------------------------:|:-------------------------:
![](https://raw.githubusercontent.com/rigbetellabs/tortoisebot_docs/master/imgs/tortoiseBot_demo/002.png) |![](https://raw.githubusercontent.com/rigbetellabs/tortoisebot_docs/master/imgs/tortoiseBot_demo/005.png) 

Teleop |  Mapping | Navigation 
:-------------------------:|:-------------------------:|:-------------------------: 
![](https://raw.githubusercontent.com/rigbetellabs/tortoisebot_docs/master/imgs/tortoiseBot_demo/007.png) |  ![](https://raw.githubusercontent.com/rigbetellabs/tortoisebot_docs/master/imgs/tortoiseBot_demo/006.png) | ![](https://raw.githubusercontent.com/rigbetellabs/tortoisebot_docs/master/imgs/tortoiseBot_demo/010.png) -->

# The TortoiseBot 🐢🤖

The ReadMe is divided into several sections as per different topics and is constantly been updated and maintained with new updates by our talented and dedicated 👥 Team at RigBetel Labs LLP. So don't forget to often come here and check on it for the latest and greatest software updates, projects & skills for your TortoiseBot. Also don't forget to 🌟 Star this repository on top-right corner of the screen to show your 💖 Love and Support 🤗 for our Team. 🤩 It will make us happy and encourage us to make and bring more such projects for you. 😍 Click [here](https://github.com/rigbetellabs/tortoisebot/wiki/1.-Getting-Started) to get started.

1. [Getting Started](https://github.com/rigbetellabs/tortoisebot/wiki/1.-Getting-Started)
2. [Hardware Assembly](https://github.com/rigbetellabs/tortoisebot/wiki/2.-Hardware-Assembly)
3. [TortoiseBot Setup](https://github.com/rigbetellabs/tortoisebot/wiki/3.-TortoiseBot-Setup)
4. [Server PC Setup](https://github.com/rigbetellabs/tortoisebot/wiki/4.-Server-PC-Setup)
5. [Running Demos](https://github.com/rigbetellabs/tortoisebot/wiki/5.-Running-Demos)

[Join](https://discord.gg/qDuCSMTjvN) our community for Free. Post your projects or ask questions if you need any help.


## TortosieBot is sourced, assembled, made & maintained by our team 🧑🏻‍🤝‍🧑🏻 at<br>

RigBetel Labs LLP®, Charholi Bk., via. Loheagaon, Pune - 412105, MH, India 🇮🇳<br>
🌐 [RigBetelLabs.com](https://rigbetellabs.com) 📞 [+91-8432152998](https://wa.me/918432152998) 📨 getintouch.rbl@gmail.com , info@rigbetellabs.com <br>
[LinkedIn](http://linkedin.com/company/rigbetellabs/) | [Instagram](http://instagram.com/rigbetellabs/) | [Facebook](http://facebook.com/rigbetellabs) | [Twitter](http://twitter.com/rigbetellabs) | [YouTube](https://www.youtube.com/channel/UCfIX89y8OvDIbEFZAAciHEA) | [Discord Community](https://discord.gg/qDuCSMTjvN)
