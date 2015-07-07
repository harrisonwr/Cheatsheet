ROS + Gazebo
===
###Using roslaunch to start Gazebo
```python
# These launch files locate at /opt/ros/indigo/share/gazebo_ros/launch
# start Gazebo with a empty world
$ roslaunch gazebo_ros empty_world.launch

# start Gazebo with willowgarage world
$ roslaunch gazebo_ros willowgarage_world.launch
```
##### You can change the behavior when launching Gazebo with there parameters
```
1. paused:
      Start Gazebo in a paused state (default false)
2. use_sim_time:
      Tells ROS nodes asking for time to get the Gazebo-published simulation
      time, published over the ROS topic /clock (default true)
3. gui:
      Launch the user interface window of Gazebo (default true)
4. headless:
      Disable any function calls to simulator rendering (Ogre) components.
      Does not work with gui:=true (default false)
5. debug:
      Start gzserver (Gazebo Server) in debug mode using gdb (default false)
```
##### Sample Launch File( mud_world.launch )
```xml
<!--
  This file is locate at /opt/ros/indigo/share/gazebo_ros/launch
-->
<launch>
  <!-- We resume the logic in empty_world.launch, changing only the name of the world to be launched -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="worlds/mud.world"/> <!-- Note: the world_name is with respect to GAZEBO_RESOURCE_PATH environmental variable -->
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>
</launch>
```
##### Sample World File( mud.world )
```xml
 <!-- This file is locate at /usr/share/gazebo-5.1/worlds  -->
<sdf version="1.4">
  <world name="default">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://double_pendulum_with_base</uri>
      <name>pendulum_thick_mud</name>
      <pose>-2.0 0 0 0 0 0</pose>
    </include>
    ...
  </world>
</sdf>
```

### Creating Gazebo ROS Package
```python
# Tutorial: http://gazebosim.org/tutorials?tut=ros_roslaunch&ver=1.9%2B&cat=connect_ros

# PATH: /home/paralles/catkin_ws/src (parallels can be replaced with user_name)

/MYROBOT_description: a package that contains robot model and description

/MYROBOT_gazebo: a package that conatins world files and launch files

# This is the hierarchy for ROS Package.
# Remember to replace MYROBOT with the name you want and put it in lowercase.
# You will create those directories by yourself.  
../catkin_ws/src
    /MYROBOT_description
        package.xml
        CMakeLists.txt
        /urdf
            MYROBOT.urdf
        /meshes
            mesh1.dae
            mesh2.dae
            ...
        /materials
        /cad
    /MYROBOT_gazebo
        /launch
            MYROBOT.launch
        /worlds
            MYROBOT.world
        /models
            world_object1.dae
            world_object2.stl
            world_object3.urdf
        /materials
        /plugins
```

### Creating a Custom World File
```xml
Step1: Create a ROS package with the convention MYROBOT_gazebo
Step2: Within MYROBOT_gazebo package, create a launch folder
Step3: Within the launch folder, create a YOURBOT.launch file
Step4: Within MYROBOT_gazebo package, create a worlds folder
Step5: Within the worlds folder, create a MYROBOT.world file

<!-- Step1  -->
$ cd ~/catkin_ws/src
$ mkdir MYROBOT
$ cd MYROBOT
$ catkin_create_pkg MYROBOT_gazebo
$ cd MYROBOT_gazebo

<!-- Step2 -->
$ mkdir launch
$ atom YOURBOT.launch

<!-- Step3: YOURBOT.launch -->
<!-- Remember to replace MYROBOT with the name of the robot or directory's name -->
<launch>
  <!-- We resume the logic in empty_world.launch, changing only the name of the world to be launched -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find MYROBOT_gazebo)/worlds/MYROBOT.world"/>
    <!-- more default parameters can be changed here -->
  </include>
</launch>

<!-- Step4 -->
$ cd ~/catkin_ws/src/MYROBOT/MYROBOT_gazebo
$ mkdir world
$ atom MYROBOT.world

<!-- Step5: MYROBOT.world -->
<?xml version="1.0" ?>
<sdf version="1.4">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://gas_station</uri>
      <name>gas_station</name>
      <pose>-2.0 7.0 0 0 0 0</pose>
    </include>

    <!-- You can add a robot in your world by including a model
    <include>
      <uri>model://turtlebot</uri>
      <name>Turtle Bot</name>
      <pose> 0 0 0 0 0 0 </pose>
    </include>
    -->
  </world>
</sdf>
```
#### Testing your custom world
```python
# Before you launch your custom world, remember to source the environment
$ ~/catkin_ws/devel/setup.bach

# launch your custom world
$ roslaunch MYROBOT_gazebo MYROBOT.launch
```

### URDF in Gazebo
#### Backgound Information
```xml
<!-- Tutorial: http://wiki.ros.org/urdf/XML/link -->
<!-- Tutorial: http://gazebosim.org/tutorials/?tut=ros_urdf -->

<!-- Required Tag -->
An <inertia> element within each <link> element must be properly
specified and configured.


<!-- Optional Tags -->
Add a <gazebo> element for every <link>:
  1. Convert visual colors to Gazebo format
  2. Convert STL files to DAE files for better texture
  3. Add sensors plugins
Add a <gazebo> element for every <joint>:
  1. Set proper damping dynamics
  2. Add actuator control plugins
Add a <gazebo> element for the <robot> element:
Add a <link name="world"/> link if the robot should be rigidly
attached to the world/base_link
```
#### Get Started with RRBot
```python
# We need a working URDF file for the robot to make it work in Gazebo
# Test your URDF by viewing it in Rviz before proceeding to configure
# your robot with Gazebo

$ cd ~/catkin_ws/src/
$ git clone https://github.com/ros-simulation/gazebo_ros_demos.git
$ cd ..
$ catkin_make

# View in Rviz
$ roslaunch rrbot_description rrbot_rviz.launch

# If encounter this error:
# Failed to load plugin libgazebo_ros_control.so: libgazebo_ros_control.so:
# cannot open shared object file: No such file or directory
# install this
$ sudo apt-get install ros-indigo-gazebo-ros-control
```
