ROS TurtleBot Simulator
===
## Setup
### Install the software
```python
# This will install the necessary software to run turtlebot on Gazebo
$ sudo apt-get install ros-indigo-turtlebot-simulator

# if you encounter any error like this,
# The following packages have unmet dependencies:
#  ros-indigo-turtlebot-simulator : Depends: ros-indigo-turtlebot-gazebo but
#  it is not going to be installed
# E: Unable to correct problems, you have held broken packages.

# if missing dependency, install the dependency by
$ sudo apt-get install <dependency name>

# you can replace dependency's name with any name of the dependency required
$ sudo apt-get install ros-indigo-turtlebot-gazebo
```

### Build the Catkin Package for TurtleBot Simulation
```python
# since we don't have the package for turtlebot simulation yet, we want
# to clone the files from Github to our catkin workspace:
$ cd ~/catkin_ws/src
$ git clone https://github.com/turtlebot/turtlebot_simulator.git

# build the package
$ cd ~/catkin_ws
$ catkin_make

# source the environment
$ source devel/setup.bash
```

## Customizing the Stage Simulator
### Package Layout
```python
# Tutorial: http://wiki.ros.org/turtlebot_stage/Tutorials/indigo/Customizing%20the%20Stage%20Simulator

# this is how the turtlebot_stage folder looks like
  launch/turtlebot_in_stage.launch
  maps/maze.png
  maps/maze.yaml
  maps/stage/maze.world
  maps/stage/turtlebot.inc
  rviz/robot_navigation.rviz

# rviz file contains the rviz settings
# turtlebot.inc defines the layout of the turtlebot and its sensor
```
## Duplicate Files that we need to go through the Tutorial
```python
# to go through the tutorial, we'll use the preset
# config files and modify these.


# make a copy of the files "world", "yaml" and "png" file to a folder
# of your choice. In the example below we use "~/stageTutorial/"

# create directory named stageTutorial
$ cd
$ mkdir stageTutorial

# go to turtlebot_stage directory
$ cd ~/catkin_ws/src/turtlebot_simulator/turtlebot_stage/

# copy the file
$ cp maps/maze.png ~/stageTutorial/tutorial.png
$ cp maps/maze.yaml ~/stageTutorial/tutorial.yaml
$ cp maps/stage/maze.world ~/stageTutorial/tutorial.world
```

### Modifing a Map
```python
# tutorial.yaml

# change in the the default map(first line)from "maze.png" to
# "tutorial.png" or the path to the file if it is not in the same folder

# change image: maze.png to
image: tutorial.png
```
```python
# tutorial.world

# In the section of floorplan, edit the following two lines where
# the first is just an abitrary name and the second the path to the
# "tutorial.png" file
name "tutorial"world
bitmap "../tutorial.png"
```
