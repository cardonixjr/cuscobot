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
### Rosserial
For this project, we use ROS Noetic, so, first check the ROS installation.

```bash
rosversion -d
```

For more details about ROS noetic setup, check http://wiki.ros.org/noetic/Installation/Ubuntu

Then, check the Rosserial installation

```bash
rosversion rosserial
```
For more details about rosserial setup, check http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

### Python plot libraries
Finally, pip install the python plotting libraries

```bash
pip install matplotlib
pip install plotly
```

## 1.3 Create the catkin workspace

```bash
cd ~
mkdir cuscobot_ws
cd ~/cuscobot_ws
```

## 1.4 Clone this repository

With ros and rosserial installed you should clone this repository.

```bash
git clone https://github.com/cardonixjr/CuscoBot
```

And, then, you must rename the folder to match the catkin pattern.

```bash
mv CuscoBot src
```

## 1.5 Enviroment setup
### Source bash
You must source this script in every ~bash~ terminal you use ROS in. 

```bash
source /opt/ros/noetic/setup.bash
```

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

## 1.6 Built enviroment
### Run catkin_make 

```bash
cd ~/cuscobot_ws
catkin_make
```

### Source packages
You must source this script in every ~bash~ terminal you use ROS in. 

```bash
cd ~/cuscobot_ws
source devel/setup.bash
. ~/cuscobot_ws/devel/setup.bash
```

## 1.8 Run packages
First, we initialize roscore and rosserial communication. Run this with Cuscobot and Lidar connected to your USB ports

```bash
roscore
```

In a separated terminal, run:

```bash
rosrun rosserial_python serial_node.py /dev/ttyACM0
```

You should run each node in a separated terminal. In all of them, enter the cuscobot_ws folder.

```bash
cd ~/cuscobot_ws/
```

### Run rplidar node

```bash
roslaunch rplidar_ros rplidar_a1.launch
```

### Run teleop_twist_keyboard node

```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### Run MD49 Odometry node

```bash
rosrun MD49 deadReckoning.py
```

### Run rviz

```bash
rosrun rviz rviz
```

### Check running topics
```bash
rostopic list
```

Check odometry communication

```bash
rostopic echo /odom
```
