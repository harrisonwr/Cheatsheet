ROS Cheatsheet
===============
### How to Get Started
#### System Requirement
```
Ubuntu: 14.04.02, 64-bit,
You can download here: http://www.ubuntu.com/download/desktop

ROS: Indigo
Installation Guide: http://wiki.ros.org/indigo/Installation/Ubuntu
```

#### ROS Indigo Installation on Ubuntu
```python
# These steps are meant to be run one at a time

$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

$ sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116

# Always remember to update the Debian package
$ sudo apt-get update

$ sudo apt-get install ros-indigo-desktop-full

$ sudo rosdep init

$ rosdep update

# These two steps is for setting up the environment
# It's convenient if the ROS environment variables are automatically
# added to your bash session every time a new shell is launched:
$ echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc

$ source ~/.bashrc

# Install rosinstall
$ sudo apt-get install python-rosinstall

# Recommend to install
# install this if you want to go through the tutorial
$ sudo apt-get install ros-indigo-ros-tutorials

# install rqt
$ sudo apt-get install ros-indigo-rqt
$ sudo apt-get install ros-indigo-rqt-common-plugins

# we want to install ros-ingigo-turtlesim.
# If you have not install rqt yet, put ros-indigo-rqt and ros-indigo-rqt-common-plugins
$ sudo apt-get install ros-indigo-rqt ros-indigo-rqt-common-plugins ros-<distro>-turtlesim

```
#### Run Turtle Simulation
##### you should be able to see a blue screen with a turtle in the middle
```python
# In one terminal tab
$ roscore

# open another terminal tab and run
$ rosmake turtlesim

$ rosrun turtlesim turtlesim_node
```

#### Useful Command
```python
# we recommend you to UPDATE THE PACKAGE every few days
$ sudo apt-get update

# install the update
$ sudo apt-get upgrade

# search for packages.
# Ex. apt-cache search turtlesim
$ apt-cache search <topic>

# install package
$sudo apt-get install <package>

# process status
$ ps -elf | grep ros

# environment
$ env | grep ROS
```


### Ros Basics
##### wiki.ros.org/ROS/Tutorials
#### Creating Catkin Workspace
```python
# go to your home directory
$ cd

# sourced your environment
$ source /opt/ros/indigo/setup.bash

# create a directory named src inside catkin_ws directory.
# If catkin_ws doesn't exist,
# computer will also create another directory named catkin_ws
$ mkdir -p catkin_ws/src

# go to src directory
$ cd ~/catkin_ws/src

# initialize workspace
$ catkin_init_workspace

# go into directory catkin_ws
$ cd ~/catkin_ws

# create build and devel
$ catkin_make

# environment variable. This will set some environment variable that make ROS works.
# call this whenever you open a new terminal tab
$ source devel/setup.bash
```

#### Create Catkin Package
```python
# Tutorials: http://wiki.ros.org/ROS/Tutorials/CreatingPackage

# go to src directory inside the catkin workspace
$ cd ~/catkin_ws/src

# create package with dependencies
# catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
# sample:
$ catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
```
#### Building a catkin workspace and sourcing the setup file
```python
# Tutorials: http://wiki.ros.org/ROS/Tutorials/CreatingPackage

# run this whenever you create a package or modifying the file
$ cd ~/catkin_ws
$ catkin_make

# to add workspace to your ROS environment
$ . ~/catkin_ws/devel/setup.bash
```

#### Package Dependencies
```python
# Tutorials: http://wiki.ros.org/ROS/Tutorials/CreatingPackage

# First-order dependencies
$ rospack depends1 beginner_tutorials

# indirect dependencies
$ rospack depends1 rospy

# get all nested dependencies
$ rospack depends beginner_tutorials
```

#### Customizing Your Package
```xml
<!-- http://wiki.ros.org/catkin/package.xml#Build.2C_Run.2C_and_Test_Dependencies -->

<!--
# Package contains four main parts
# 1. <description>
# 2. <maintainer>
# 3. <license>
# 4. <dependencies>
-->

<!-- update the description tag -->
<description>The beginner_tutorials package</description>

<!-- maintainer: let others know who to contact about the package -->
<maintainer email="you@yourdomain.tld">Your Name</maintainer>

<!-- license: include license such as some open source license -->
<license>BSD</license>

<!-- dependencies:
# 1. build_depend: specify which packages are needed to build this package
# 2. buildtool_depend: specify build system tool which this package need to build itself. We mostly use catkin
# 3. run_depend: specify which packages are needed to run code in this package
# 4. test_depend: specify only additional dependencies for unit tests
-->
<buildtool_depend>catkin</buildtool_depend>

<build_depend>message_generation</build_depend>
<build_depend>roscpp</build_depend>
<build_depend>std_msgs</build_depend>

<run_depend>message_runtime</run_depend>
<run_depend>roscpp</run_depend>
<run_depend>rospy</run_depend>
<run_depend>std_msgs</run_depend>

<test_depend>python-mock</test_depend>
```

#### Building a ROS Package
```python
# remember to source your environment before building the package
# indigo can be replace by other ros distro such as jade or groovy
$ source /opt/ros/indigo/setup.bash

# go to catkin workspace directory
$ cd ~/catkin_ws

# take a look what is inside the src directory
$ ls src

# build the catkin package and you should see three directories afterwards: build, devel, src
$ catkin_make
```

#### ROS Services and Parameters
##### Services
```python
# get all the services that we can call in the application that we are running
$ rosservice list

# get the type of a specific service
# template: $ rosservice type [service]
$ rosservice type clear

# call service
# template: $ rosservice call [service] [args]
$ rosservice call /clear


# demonstration of call the service and getting the type with turtlesim

# check out the type of the service spawn
# it will give you information about values that it needs to call the service
$ rosservice type spawn | rossrv show

# call the service spawn
# (float32 x, float32 y, float32 theta, string name)
$ rosservice call spawn 2 2 0.2 ""
```

##### Parameters
```python

# list out all the parameters that the node that are running
$ rosparam list

# set the parameter with the value that come after
$ rosparam set [param_name]

# get the value of the parameter
$ rosparam get [param_name]

# -------------------------------------------------------------------------
# this is a demonstration of setting and getting the parameter of turtlesim

# this set the red background color to 150
$ rosparam set background_r 150

# this will clear the screen so that the screen got update with new value
$ rosservice call clear

# get the green value of the background color
$ rosparam get background_g

# or you can just get all the values of parameters
$ rosparam get /
#
#---------------------------------------------------------------------------


# write information into a file
$ rosparam dump [file_name] [namespace]

# load information into a new namespace
$ rosparam load [file_name] [namespace]

# -----------------------------------------------------
# sample code for using rosparam dump and rosparam load

# write all the parameters to the file params.yaml
$ rosparam dump params.yaml

# load these yaml files into new namespaces, e.g. copy
$ rosparam load params.yaml copy
$ rosparam get copy/background_r
#
# ------------------------------------------------------
```

#### rqt_console and roslaunch
```python

```


#### Sample Package
```python
# go to src directory, and you should see CMakeLists.txt
$ cd src

# This is sample code that we are going to run
# go to github.com/hcrlab, find randomwalker and clone it
$ git clone https://github.com/hcrlab/randomwalker.git

# go back to catkin_ws directory
$ cd ..

# call catkin_make to make sure you got the file
# it looks for your work space and found a new package
$ catkin_make

# how to launch the application
$ roslaunch launch/walk.launch

# display all topics of the master/application
$ rostopic list

# echo the message publish by the node.
# topic_name should be replace by the different topic name
$ rostopic echo /topic_name

# how to make a file become a node
rospy.init_node('teleop')

# spin() simply keeps python from exiting until this node is stopped
$ rospy.spin()

# to get the message
$ rosmsg

# add something new here

```
