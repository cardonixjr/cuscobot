# Cuscobot systems
This repository contains Cuscobot, a research mobile robot for guiding blind people on indoor and outdoor environments.

The "Old_researches" folder contain obsolete or unused codes developed during the first stages of Cuscobot development.
Inside each subproject, there is a "requirements.txt" that shoud be installed for this project to work.
If you'll execute one of those old algorythms, execute first: `pip install -r requirements.txt`

## 1.1 Bibliography
For this project, we use RPLidar and Teleop_twist_keyboard algorithms:

Visit the following websites for more details.

rplidar repository: https://github.com/Slamtec/rplidar_ros

rplidar roswiki: http://wiki.ros.org/rplidar

rplidar HomePage: http://www.slamtec.com/en/Lidar

rplidar SDK: https://github.com/Slamtec/rplidar_sdk

teleop_twist_keygoard repository:   https://github.com/ros-teleop/teleop_twist_keyboard

## 1.2 ROS Noetic installation
For this project, we use ROS Noetic.

To install this ros distro, first, make sure your Debian package index is up-to-date: 

```bash
sudo apt update
```

Then, install ROS.

```bash
sudo apt install ros-noetic-desktop
```

For more details about ROS installation, check http://wiki.ros.org/noetic/Installation/Ubuntu

## 1.3 Rosserial installarion
Then, install rosserial library

```bash
sudo apt-get install ros-noetic-rosserial-arduino
sudo apt-get install ros-noetic-rosserial
```

For more details about rosserial setup, check http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

## 1.4 Clone this repository

With ros and rosserial installed you should clone this repository.

```bash
git clone https://github.com/cardonixjr/CuscoBot
```

And, then, use catkin_make to build the workspace.

```bash
cd ~/CuscoBot/catkin_ws/
catkin_make
```

## 1.5 Enviroment setup
First, enter catkin_ws.

```bash
cd ~/CuscoBot/catkin_ws/
```

### Source bash
You must source this script in every ~bash~ terminal you use ROS in. 

```bash
source /opt/ros/noetic/setup.bash
```

### Check serial-ports authority
We'll use two serial ports in this project. One for the RPlidar, and one for Arduino MEGA.

Check if those ports have authority:

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

### Run roscore
In a separated terminal, run the core.

```bash
roscore
```

### Enable Rosserial communication
Open the rosserial communication with Arduino port.

```bash
rosrun rosserial_python serial_node.py /dev/ttyACM0
```

## 1.6 Run packages

Enter the catkin_ws folder.

```bash
cd ~/CuscoBot/catkin_ws/
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
