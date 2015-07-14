Gazebo Basic
===
### Install Gazebo
```python
# This installs the ROS package that integrates Gazebo
$ sudo apt-get install ros-indigo-gazebo-ros-pkgs ros-indigo-gazebo-ros-control
```

### Testing Gazebo with ROS Interface
```python
# Test One

$ roscore

# run Gazebo with ROS
$ rosrun gazebo_ros gazebo

# potential warnings/errors and solutions:
#   1. "waited for namespace":
#       a. source /usr/share/gazebo-2.2/setup.sh
#       b. or restart the computer if nothing works. Remember to source the file again
#   2. "Unable to start server ... There is probably another Gazebo process running"
#       a. killall gzserver

# list out all the topics generated
$ rostopic list
```

```python
# Test Two

$ roscore

# run gazebo server
$ rosrun gazebo_ros gzserver

# run gazebo client
$ rosrun gazebo_ros gzclient
```

### Installing TurtleBot Robot Packages on ROS Indigo
```python
# let's install the package manager first to make our lives easier
$ sudo apt-get install synaptic

# open synaptic
$ sudo synaptic

Follow the following steps:
1. # search "ros-indigo-rocon" keyword in synaptic

2. # select all items and mark all for install, and click the apply button above

Next, install ros-indigo-kobuki just like installing ros-indigo-rocon
```
#### Build and Install Turtlebot's latest ROS Package
```python
$ mkdir ~/turtlebot

$ cd ~/turtlebot

# downlaod source files for turtlebot package
$ wstool init src -j5
https://raw.github.com/yujinrobot/yujin_tools/master/rosinstalls/indigo/turtlebot.rosinstall

# install dependencies
$ rosdep install --from-paths src -i -y

# build the package
$ catkin_make

# To access TurtleBot packages from all terminals, add source to .bashrc
$ echo "source  ~/turtlebot/devel/setup.bash" >> ~/.bashrc

$ source ~/.bashrc

# install turtlebot ROS packages
$ sudo apt-get install ros-indigo-turtlebot ros-indigo-turtlebot-apps ros-indigo-turtlebot-interactions ros-indigo-turtlebot-simulator ros-indigo-kobuki-ftdi ros-indigo-rocon-remocon

```

```python
$ source /usr/share/gazebo-2.2/setup.sh
$ source ~/catkin_ws/devel/setup.bash
```
