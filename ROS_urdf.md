ROS URDF
===
###Creating your own URDF(Robot Description) file
```xml
<!--
  Create my_robot.urdf in urdf directory in your MYROBOT_description package.
  This is a simplest robot that has 4 arms that connect by 3 joints
-->

<robot name="test_robot">
  <link name="link1" />
  <link name="link2" />
  <link name="link3" />
  <link name="link4" />

  <joint name="joint1" type="continuous">
    <parent link="link1"/>
    <child link="link2"/>
  </joint>

  <joint name="joint2" type="continuous">
    <parent link="link1"/>
    <child link="link3"/>
  </joint>

  <joint name="joint3" type="continuous">
    <parent link="link3"/>
    <child link="link4"/>
  </joint>
</robot>
```
#### Check if we can get this urdf file parsed
``` python
# install urdfdom as an upstream first if not yet installed
$ sudo apt-get install liburdfdom-tools

# run check command in the directory that contains my_robot.urdf
$ check_urdf my_robot.urdf
```

### Add the Dimensions
The reference frame of each link is locate at the bottom of the link,
and is identical to the reference frame of the joint(they are the same).
Therefore, we only need to specify the offset from a link to a joint of
it's children by adding filed <origin> to each joint

#### Origin Tag
```xml
<!--
  In this example, Joint2 is offset in the Y-direction from link1, a little
  offset in the negative X-direction from link1, and it is rotated 90 degrees
  around the Z-axis
-->
<joint>
  ...
  <origin xyz="-2 5 0" rpy="0 0 1.57" />
  ...
</joint>
```

#### Adding Dimensions to my_robot.urdf
```xml
<robot name="test_robot">
  <link name="link1" />
  <link name="link2" />
  <link name="link3" />
  <link name="link4" />


  <joint name="joint1" type="continuous">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="5 3 0" rpy="0 0 0" />
  </joint>

  <joint name="joint2" type="continuous">
    <parent link="link1"/>
    <child link="link3"/>
    <origin xyz="-2 5 0" rpy="0 0 1.57" />
  </joint>

  <joint name="joint3" type="continuous">
    <parent link="link3"/>
    <child link="link4"/>
    <origin xyz="5 0 0" rpy="0 0 -1.57" />
  </joint>
</robot>
```
```python
# parsed my_robot.urdf again. If everything looks fine, continue to next step.
$ check_urdf my_robot.urdf
```

### Completing the Kinematics
We need to specify which axis the joint rotate by adding <axis> element to each joint
```xml
<!--
  For joint 2, it rotate diagonally. You can think of the value in axis as vector
  This axis is based on joint's frame, not the parent frame.
-->
<joint>
  ...
  <origin />
  <axis xyz = "-0.9 0.15 0"/>
  ...
</joint>
```
#### Adding Kinematics to my_robot.urdf
```xml
<robot name="test_robot">
  <link name="link1" />
  <link name="link2" />
  <link name="link3" />
  <link name="link4" />

  <joint name="joint1" type="continuous">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="5 3 0" rpy="0 0 0" />
    <axis xyz="-0.9 0.15 0" />
  </joint>

  <joint name="joint2" type="continuous">
    <parent link="link1"/>
    <child link="link3"/>
    <origin xyz="-2 5 0" rpy="0 0 1.57" />
    <axis xyz="-0.707 0.707 0" />
  </joint>

  <joint name="joint3" type="continuous">
    <parent link="link3"/>
    <child link="link4"/>
    <origin xyz="5 0 0" rpy="0 0 -1.57" />
    <axis xyz="0.707 -0.707 0" />
  </joint>
</robot>
```

### Reading a URDF File
Previously, we create my_robot.urdf inside the package MYROBOT_description package.
Now, we want to show you how the package of robot description should look like
```python
# go to src directory inside carking workspace
$ cd ~/catkin_ws/src

# let's create a package with dependency on the urdf parser
$ catkin_create_pkg testbot_description urdf

# go to the package we just create
$ cd testbot_description

# create some standard subfolders for robot description package
# /urdf, /meshed, /media, /cad
$ mkdir meshes
$ mkdir media
$ mkdir cad
$ mkdir urdf
$ mkdir materials

# copy my_robot.urdf we created previously to src folder inside the package
# cp ~/catkin_ws/src/myrobot_description/urdf/my_robot.urdf urdf/
$ cp /path/to/.../testbot_description/urdf/my_robot.urdf
```

#### Create parser.cpp
```python
# in the testbot_description directory, create a src directory
# PATH: ~/catkin_ws/src/testbot_description
$ mkdir src

# create parser.cpp in the src folder
$ cd src
$ atom parser.cpp
```
```c
// parser.cpp
#include <urdf/model.h>
#include "ros/ros.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "my_parser");
  if (argc != 2){
    ROS_ERROR("Need a urdf file as argument");
    return -1;
  }
  std::string urdf_file = argv[1];

  urdf::Model model;
  if (!model.initFile(urdf_file)){
    ROS_ERROR("Failed to parse urdf file");
    return -1;
  }
  ROS_INFO("Successfully parsed urdf file");
  return 0;
}
```

#### Modify CMakeList.txt
```python
# in CMakeList.txt
# add these lines
add_executable(parser src/parser.cpp)
target_link_libraries(parser ${catkin_LIBRARIES})
```
```python
# build the package
$ cd ~/catkin_ws
$ catkin_make

# run the package
# template: $ .<path>/parser <path>my_robot.urdf
$ ./devel/lib/testbot_description/parser /src/testbot_description/urdf/my_robot.urdf

# you shoud see something like this
# [ INFO] 1254520129.560927000: Successfully parsed urdf file
```




### Using a robot state publisher on your own robot
```xml

```
