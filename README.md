# Cuscobot Packages
This repository contains the ros packages used in Cuscobot, a research mobile robot for guiding blind people on indoor and outdoor environments.

## 1.1 Bibliography
For this project, we use RPLidar and Teleop_twist_keyboard algorithms:

Visit the following websites for more details.

rplidar repository: https://github.com/Slamtec/rplidar_ros

rplidar roswiki: http://wiki.ros.org/rplidar

rplidar HomePage: http://www.slamtec.com/en/Lidar

rplidar SDK: https://github.com/Slamtec/rplidar_sdk

teleop_twist_keygoard repository:   https://github.com/ros-teleop/teleop_twist_keyboard

## 1.2 Install dependencies installation
### ROS noetic
First of all, make sure your Debian package index is up-to-date:

```bash
sudo apt update
```
For this project, we use ROS Noetic, so, first check the ROS installation.

```bash
rosversion -d
```

For more details about ROS noetic setup, check http://wiki.ros.org/noetic/Installation/Ubuntu

### Rosserial
Then, check the Rosserial installation

```bash
rosversion rosserial
```
For more details about rosserial setup, check http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

### RPLidar
This project uses a Slamtec 2D Laser Scanner RPLIDAR A1. Install the packages for running RPLidar.

```bash
sudo apt install ros-noetic-rplidar-ros
```

For more details about RPLidar setup, check https://wiki.ros.org/rplidar

### AMCL dependencies
The navigation algorithm used is the AMCL. Istall the packages for this ROS library:

```bash
sudo apt-get install ros-noetic-navigation
```

```bash
sudo apt-get install ros-noetic-map-server
```
```bash

sudo apt-get install ros-noetic-move-base
```

```bash
sudo apt-get install ros-noetic-amcl
```

For more details about AMCL setup, check http://wiki.ros.org/amcl

### Teleop Twist Keyboard
To control Cuscobot, we use keyboard commands, handle by the Teleop Twist Keyboard package for ROS. Install this package.

```bash
sudo apt-get install ros-noetic-teleop-twist-keyboard
```

For more details about Teleop_Twist_Keyboard setup, check https://wiki.ros.org/teleop_twist_keyboard

## 1.3 Create the catkin workspace

```bash
cd ~
```

```bash
mkdir -p cuscobot_ws/src
```

```bash
cd ~/cuscobot_ws/
```

```bash
catkin_make
```

## 1.4 Clone this repository

Clone this repository into your src folder inside your workspace

```bash
cd ~/cuscobot_ws/src
```

```bash
git clone https://github.com/cardonixjr/cuscobot.git
```

## 1.5 Enviroment setup
### Source bash
You must source this script in every ~bash~ terminal you use ROS in. 

```bash
source /opt/ros/noetic/setup.bash
```

## 1.6 Built enviroment
### Run catkin_make 

```bash
cd ~/cuscobot_ws
catkin_make
```

### Source packages
*You must source this script in every bash terminal you use ROS.*

```bash
cd ~/cuscobot_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

## 1.8 Run packages

### Check serial-ports authority
We'll use two serial ports in this project. One for the RPlidar, and one for Arduino MEGA.

With Arduino and Lidar connected to USB ports of your computer, check their permissions:

```bash
ls -l /dev |grep ttyUSB
```

```bash
ls -l /dev |grep ttyACM
```

Add the authority of write for rplidar port: (such as /dev/ttyUSB0)

```bash
sudo chmod 666 /dev/ttyUSB0
```

Add the authority of write for Arduino port: (such as /dev/ttyACM0)

```bash
sudo chmod 666 /dev/ttyACM0
```

### Run Cuscobot

First, we initialize roscore.

```bash
roscore
```

In a separated terminal, we initialize the cuscobot core algorith. Run this with Cuscobot and Lidar connected to your USB ports.


```bash
roslaunch cuscobot cuscobot_launch.launch
```

You should run each node in a separated terminal. In all of them, you must Source bash and the package.

### Run navigation stacks and rviz:

```bash
roslaunch cuscobot nav_stacks.launch
```
