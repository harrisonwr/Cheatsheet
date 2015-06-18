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

# install rqt individually
# $ sudo apt-get install ros-indigo-rqt
# $ sudo apt-get install ros-indigo-rqt-common-plugins

# since we want to install ros-ingigo-turtlesim for going through the tutorial,
# we can install all the rqt related plugins all at once
$ sudo apt-get install ros-indigo-rqt ros-indigo-rqt-common-plugins ros-indigo-turtlesim

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

# these command will show you the dependenciese inside the package
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
##### Using rqt console
```python
# Tutorial: http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch

# check which message
$ rosrun rqt_console rqt_console

$ rosrun rqt_logger_level rqt_logger_level

```
##### Roslaunch
```python
# Tutorial: http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch

# starts nodes as define in a launch file
$ roslaunch [package] [filename.launch]

#------------------

# go to the package
$ roscd beginner_tutorials

# if "No such package/stack 'beginner_tutorials'",
# you will need to source the environment first
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roscd beginner_tutorials

# create a launch folder and go to the launch folder
$ mkdir launch
$ cd launch

# use rqt_graph to see the relationship between each node
$ rqt_graph
```

```xml
 <!-- create a launch file named turtlemimic.launch -->

<!-- launch tag let the computer to identify the file as launch file-->
<launch>
  <!--
    start two groups with namespace of turtlesim1 and turtlesim2
    that use turtlesim node so that we can use the turtle simulators
    without having name conflicts
  -->
  <group ns="turtlesim1">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>

  <group ns="turtlesim2">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>

  <!--
    user input -> turtlesim1 -> mimic -> turtlesim2

    turtlesim1 listen to user input
    mimic's input is given by turtlesim1
    mimic's output to turtlesim2
    therefore, turtlesim2 mimic turtlesim1
  -->
  <node pkg="turtlesim" name="mimic" type="mimic">
    <remap from="input" to="turtlesim1/turtle1"/>
    <remap from="output" to="turtlesim2/turtle1"/>
  </node>
</launch>
```

#### Edit File in ROS with rosed
```python
# Tutorial: http://wiki.ros.org/ROS/Tutorials/UsingRosEd

# directly edit a file within a package
# template: $ rosed [package_name] [filename]
$ rosed roscpp Logger.msg

# see and optionally edit all files from a package without knowing its exact name
# template: $ rosed [package_name] <tab><tab>
$ rosed roscpp <tab><tab>
```

#### Using Msg and Srv
```python

# Tutorial: http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv

# msg files are text file that describes the field of a ROS message

# srv files describes a service. It composed with two parts: request and response

# Sample msg file:
#   Header header
#   string child_frame_id
#   geometry_msgs/PoseWithCovariance pose
#   geometry_msgs/TwistWithCovariance twist

# Sample srv file: '---' distinguishes request and response
# int64 A and int64 B are requests, int64 Sum is response
#   int64 A
#   int64 B
#   ---
#   int64 Sum
```
##### Msg
```python
# Creating a msg

# create a msg directory in beginner_tutorials
$ cd ~/catkin_ws/src/beginner_tutorials
$ mkdir msg

# create a msg file named Num and insert it with int64 Num
$ echo "int64 num" > /msg/Num.msg

# put this in package.xml file inside beginner_tutorials so that
# the msg files are turned into source code for C++, python, and other languages
<build_depend>message_generation</build_depend>
<run_depend>message_runtime</run_depend>

# add this inside CMakeLists.txt under find_package(catkin REQUIRED COMPONENTS)
message_generation

add lots of stuff, please visit the tutorial link

# before accessing the msg file
# remember to source the package first
$ cd ~/catkin_ws
$ source devel/setup.bash

# template: rosmsg show [message type]
$ rosmsg show beginner_tutorials/Num

$ rosmsg show Num

```

##### Srv
```python
# Create a srv

$ roscd beginner_tutorials
$ mkdir srv

# copy a srv file from rospy package to beginner_tutorial package
# template: roscp [package_name] [file_to_copy_path] [copy_path]
$ roscp rospy_tutorials AddTwoInts.srv srv/AddTwoInts.srv

add lots of stuff, please visit the tutorial link

$ rossrv show AddTwoInts

```
#### Publisher and Subscriber
##### Publisher
```c
  #include "ros/ros.h"
  #include "std_msgs/String.h"

  #include <sstream>

  /**
   * This tutorial demonstrates simple sending of messages over the ROS system.
   */
  int main(int argc, char **argv)
  {
    /**
     * The ros::init() function needs to see argc and argv so that it can perform
     * any ROS arguments and name remapping that were provided at the command line.
     * For programmatic remappings you can use a different version of init() which takes
     * remappings directly, but for most command-line programs, passing argc and argv is
     * the easiest way to do it.  The third argument to init() is the name of the node.
     *
     * You must call one of the versions of ros::init() before using any other
     * part of the ROS system.
     */
    ros::init(argc, argv, "talker");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;

    /**
     * The advertise() function is how you tell ROS that you want to
     * publish on a given topic name. This invokes a call to the ROS
     * master node, which keeps a registry of who is publishing and who
     * is subscribing. After this advertise() call is made, the master
     * node will notify anyone who is trying to subscribe to this topic name,
     * and they will in turn negotiate a peer-to-peer connection with this
     * node.  advertise() returns a Publisher object which allows you to
     * publish messages on that topic through a call to publish().  Once
     * all copies of the returned Publisher object are destroyed, the topic
     * will be automatically unadvertised.
     *
     * The second parameter to advertise() is the size of the message queue
     * used for publishing messages.  If messages are published more quickly
     * than we can send them, the number here specifies how many messages to
     * buffer up before throwing some away.
     */
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    ros::Rate loop_rate(10);

    /**
     * A count of how many messages we have sent. This is used to create
     * a unique string for each message.
     */
    int count = 0;
    while (ros::ok())
    // remember to add the bracket. I comment it because my editor is acting
    // weird for markdown file
    // {
      /**
       * This is a message object. You stuff it with data, and then publish it.
       */
      std_msgs::String msg;

      std::stringstream ss;
      ss << "hello world " << count;
      msg.data = ss.str();

      ROS_INFO("%s", msg.data.c_str());

      /**
       * The publish() function is how you send messages. The parameter
       * is the message object. The type of this object must agree with the type
       * given as a template parameter to the advertise<>() call, as was done
       * in the constructor above.
       */
      chatter_pub.publish(msg);

      ros::spinOnce();

      loop_rate.sleep();
      ++count;
    }


    return 0;
  }
```

##### Subscriber
```c++
#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
```
##### Building Your Node for Publishers and Subscribers
```python
# add this at the bottom of CMakeList.txt of the beginner_tutorials package

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)
```

##### Test out your Publisher and Subscriber
```python
# run roscore first
$ roscore

# source the environment
$ cd ~/catkin_ws
$ source ./devel/setup.bash

# if either publisher or subscriber can't not be run, source the environment
# run the publisher
$ rosrun beginner_tutorials talker

# run the subscriber
$ rosrun beginner_tutorials listener
```

#### Service and Client
```python
# go to beginner_tutorials package directory
$ cd ~/catkin_ws/src/beginner_tutorials

# remember to create AddTwoInts.srv first
```

##### Service
###### create add_two_ints_server.cpp under src directory
```python
#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"

bool add(beginner_tutorials::AddTwoInts::Request  &req,
         beginner_tutorials::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}
```

##### Client
###### create add_two_ints_client.cpp under src directory
```python
#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");
  if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
  beginner_tutorials::AddTwoInts srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}
```
##### Build your Node for Services and Clients
```python
# add this into CMakeLists.txt

add_executable(add_two_ints_server src/add_two_ints_server.cpp)
target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})
add_dependencies(add_two_ints_server beginner_tutorials_gencpp)

add_executable(add_two_ints_client src/add_two_ints_client.cpp)
target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})
add_dependencies(add_two_ints_client beginner_tutorials_gencpp)

# remember to build
$ cd ~/catkin_ws
$ catkin_make
```

##### Test Services and Clients
```python
# remember to source the environment first
$ cd ~/catkin_ws
$ catkin_make

# open the server to listen to the message
$ rosrun beginner_tutorials add_two_ints_server

# send the request with two arguments 1 3,
# the server should receive 1 for x and 3 for y and add them together to 4
$ rosrun beginner_tutorials add_two_ints_client 1 3
```

#### Recording and Playing Back Data

##### Recording data
```python
# let's run the essential simulator to test out the command
# this will start two nodes
$ roscore
$ rosrun turtlesim turtlesim_node
$ rosrun turtlesim turtle_teleop_key


# list out all the topic with detail info about publishers and subscribers
$ rostopic list -v

# record publishers data

# create a directory call bagfiles in home directory and go into it
$ mkdir ~/bagfiles
$ cd ~/bagfiles

# running rosbag record that record all published topics
$ rosbag record -a

# go back to turtle_teleop_key to move the turtle around
# quit the rosbag record by typing ctrl-c in the window
```

##### Examing Data
```python
# after we quit the rosbag record, we should have a .bag file save
# in bagfiles directory specified with year, data, and time

# get information about the recorded data
$ rosbag info <your bag_file>

# play back the recorded data in the simulator,
# the turtle should move around with the similar pattern as before
$ rosbag play <your bag_file>

# play back the data twise faster
$ rosbag play -r 2 <your bag_file>

```
##### Recording a Subset of Data
```python
# -O tells rosbag record to log a file named subset.bag
# and only subscribes to topics that specified. In this case: cmd_vel and pose
$ rosbag record -O subset /turtle1/cmd_vel /turtle1/pose
```

#### ROSWTF for Debugging
```python
# roswtf can give you some feedback such as warning and errors
# we usually call roscd first then call roswtf

$ roscd
$ roswtf
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
