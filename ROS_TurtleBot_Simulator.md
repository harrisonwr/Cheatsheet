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
### Package Setup
