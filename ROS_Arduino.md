ROS with Arduino
=================

### Installation on Ubuntu
```python
# get the ros serial arduino package
$ sudo apt-get install ros-indigo-rosserial-arduino

# clone the rosserial library into src directory in the catkin workspace
$ cd catkin_ws/src
$ git clone https://github.com/ros-drivers/rosserial.git
$ cd catkin_ws

# compile the file and install.
$ catkin_make
$ catkin_make install

# source the environment
$ source catkin_ws/install/setup.bash

# install ros_lib into Arduino
# sketchbook is the directory to store Arduino's files/libraries
$ cd <sketchbook>/libraries

# remove the ros_lib if it exist in the libraries directory
$ rm -rf ros_lib

# import the ros_lib
$ rosrun rosserial_arduino make_libraries.py .

# if the above command required some additional packages
# call this $ sudo apt-get install ros-<distro>-<package_name>
$ sudo apt-get install ros-indigo-convex-decomposition
```
### Serial Communication
#### Setting up Arduino Serial Port
```python
# since we don't know which port the arduino is using
# this will give permission for arduino to access the serial port
$ sudo arduino

# go to arduino menu -> tools -> serial port
# see what options are available for the arduino and remember the name

# permanently give permission to the port
$ sudo usermod -a -G dialout <username>
```
#### Publishing to A Topic
```c
#include <ros.h>
ros::NodeHandle nh;

// before your setup() function
std_msgs::String str_msg;
ros::Publisher pub("foo", &str_msg);

void setup() {
    ...
    nh.advertise(pub);
    ...
}

void loop() {
   pub.publish( &str_msg );
   nh.spinOnce();
}

```

####Example Publisher
```python
# Tutorial: http://wiki.ros.org/rosserial_arduino/Tutorials/Hello%20World

# open up HelloWorld program from ros_lib
# check out the sample code in the tutorial

# run roscore to activate ros
$ roscore

# run the node for serial via port of the USB. ex. /dev/ttyACM0
$ rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0

# catch the serial information from the arduino
$ rostopic echo chatter
```

#### Example Subscriber
```python
# Tutorial: http://wiki.ros.org/rosserial_arduino/Tutorials/Blink

# open up Blink program from ros_lib
# check out the sample code in the tutorial

# run roscore to activate ros
$ roscore

# run the node for serial via port of the USB. ex. /dev/ttyACM0
$ rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0

# publish a message "toggle_led" to the Arduino
$ rostopic pub toggle_led std_msgs/Empty --once
```
