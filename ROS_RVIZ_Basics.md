ROS RVIZ Basics
===
### Create the Model
#### Setup
```python
# to go through this tutorial, we need to get essential files
# we will clone the files into src directory of catkin workspace
$ cd ~/catkin_ws/src
$ git clone https://github.com/ros/urdf_tutorial.git

# you might find some value for the robot model in the tutorial different from
# code explanation.
```

#### One Shape
```python
# test out the 01-myfirst.urdf in urdf_tutorial package
$ roslaunch urdf_tutorial display.launch model:='$(find urdf_tutorial)/urdf/01-myfirst.urdf'

# the above command take in the parameter and use it in display.launch
# <param name="robot_description" command="$(find xacro)/xacro.py $(arg model)"/>
```
##### Code Explain
```xml
<?xml version="1.0"?>
<!-- setup robot's name -->
<robot name="myfirst">
  <!-- this is the only link for the model -->
  <link name="base_link">
    <!-- this model only have a visual, one cylinder -->
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
  </link>
</robot>
```

#### Multiple Shapes
```python
# launch the 02-multipleshapes.urdf
$ roslaunch urdf_tutorial display.launch model:='$(find urdf_tutorial)/urdf/02-multipleshapes.urdf'
```
##### Code Explain
```xml
<?xml version="1.0"?>
<!-- robot's name -->
<robot name="multipleshapes">

  <!-- the base link for the robot, usually fixed frame will be set to this -->
  <link name="base_link">
    <!-- base link is a cylinder -->
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
  </link>

  <!-- the second shapes of the model as the right leg -->
  <link name="right_leg">
    <!-- right leg will be a 0.6 * 0.2 * 0.1 box -->
    <visual>
      <geometry>
        <box size="0.6 .2 .1"/>
      </geometry>
    </visual>
  </link>

  <!-- set up the joint(connection) between base link and right leg -->
  <joint name="base_to_right_leg" type="fixed">
    <parent link="base_link"/>
    <child link="right_leg"/>
  </joint>

</robot>
```

#### Origins
```python
# launch 03-origins.urdf
$ roslaunch urdf_tutorial display.launch model:='$(find urdf_tutorial)/urdf/03-origins.urdf'
```

##### Code Explain
```xml
<?xml version="1.0"?>
<robot name="origins">
  <!-- base link is the same as previous multiple shapes model -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
  </link>

  <!-- we move the origin of the box to some where else -->
  <link name="right_leg">
    <visual>
      <geometry>
        <box size="0.6 .2 .1"/>
      </geometry>
      <!--
        we move the right leg down by 0.3 meter and
        rotate around y-direction by 90 degree(roll: 0, pitch: 90, yaw: 0)
        Remember, the rpy is based on the coordinate of the link.
        This coordinate will move around based on how we set xyz value
      -->
      <origin rpy="0 1.57075 0" xyz="0 0 -0.3"/>
    </visual>
  </link>

  <!-- set up the relationship between base link and right leg -->
  <joint name="base_to_right_leg" type="fixed">
    <parent link="base_link"/>
    <child link="right_leg"/>
    <!--
      0.22 meters away from frame of base link in x direction and
      0.25 meters away in z-direction
    -->
    <origin xyz="0.22 0 .25"/>
  </joint>

</robot>
```

#### Materials
```python
# launch 04-materials.urdf
$ roslaunch urdf_tutorial display.launch model:='$(find urdf_tutorial)/urdf/04-materials.urdf'
```
##### Code Explain
```xml
<?xml version="1.0"?>
<robot name="materials">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <!-- set the color of base link to blue -->
      <material name="blue">
        <color rgba="0 0 .8 1"/>
      </material>
    </visual>
  </link>

  <link name="right_leg">
    <visual>
      <geometry>
        <box size="0.6 .2 .1"/>
      </geometry>
      <origin rpy="0 1.57075 0" xyz="0 0 -0.3"/>
      <!-- set material color of right leg to white -->
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
  </link>

  <!-- relationship of base link and right leg -->
  <joint name="base_to_right_leg" type="fixed">
    <parent link="base_link"/>
    <child link="right_leg"/>
    <origin xyz="0.22 0 .25"/>
  </joint>

  <!-- create the left leg that is just like right leg -->
  <link name="left_leg">
    <visual>
      <geometry>
        <box size="0.6 .2 .1"/>
      </geometry>
      <origin rpy="0 1.57075 0" xyz="0 0 -0.3"/>
      <material name="white"/>
    </visual>
  </link>
  <!-- set up the position of left leg relative to base link -->
  <joint name="base_to_left_leg" type="fixed">
    <parent link="base_link"/>
    <child link="left_leg"/>
    <origin xyz="-0.22 0 .25"/>
  </joint>

</robot>
```
#### Finishing the R2D2 Model
```python
# launch 05-visual.urdf
$ roslaunch urdf_tutorial display.launch model:='$(find urdf_tutorial)/urdf/05-visual.urdf'
```
##### Explain Code
```xml
<!--
  This is just part of the explanation for the robot model.
  The code should be fairly self explanatory
-->

<!-- We create a sphere and aline with the base properly to make it intersect-->
<link name="head">
  <visual>
    <geometry>
      <sphere radius="0.2"/>
    </geometry>
    <material name="white"/>
  </visual>
</link>
<joint name="head_swivel" type="fixed">
  <parent link="base_link"/>
  <child link="head"/>
  <origin xyz="0 0 0.3"/>
</joint>

<!-- we create a cylinder as the pole that can control how far the gripper want to go -->
<link name="gripper_pole">
  <visual>
    <geometry>
      <cylinder length="0.2" radius=".01"/>
    </geometry>
    <origin rpy="0 1.57075 0 " xyz="0.1 0 0"/>
  </visual>
</link>

<!-- the relationship between the gripper pole and the gripper we create below -->
<joint name="left_gripper_joint" type="fixed">
  <origin rpy="0 0 0" xyz="0.2 0.01 0"/>
  <parent link="gripper_pole"/>
  <child link="left_gripper"/>
</joint>

<!--
  we borrow the model of the gripper from other package.
  it's okay to put this link below the joint that declare this link as child
-->
<link name="left_gripper">
  <visual>
    <geometry>
      <mesh filename="package://pr2_description/meshes/gripper_v0/l_finger.dae"/>
    </geometry>
  </visual>
</link>
```

### Make the Model Move
```python
# launch 06-flexible.urdf
$ roslaunch urdf_tutorial display.launch model:='$(find urdf_tutorial)/urdf/06-flexible.urdf'
```
#### Code Explain: The Head
```xml
<!-- File to modify: 06-flexible.urdf -->

<!--
  Modify the joint tag for base link and head:
  1. modify type from fixed to continuous, meaning that it can take on any
     angle from negative infinity to positive infinity
  2. add the axis of rotation
 -->

<joint name="head_swivel" type="continuous">
  <parent link="base_link"/>
  <child link="head"/>
  <!--
    since we want it to rotate according to z-axis, we set xyz to 0 0 1
    value for axis will only be 0 or 1 to indicate true or false
  -->
  <axis xyz="0 0 1"/>
  <origin xyz="0 0 0.3"/>
</joint>
```

#### Code Explain: The Gripper
```xml

<joint name="left_gripper_joint" type="revolute">
  <axis xyz="0 0 1"/>
  <limit effort="1000.0" lower="0.0" upper="0.548" velocity="0.5"/>
  <origin rpy="0 0 0" xyz="0.2 0.01 0"/>
  <parent link="gripper_pole"/>
  <child link="left_gripper"/>
</joint>

```

#### Code Explain: The Gripper Arm
```xml
<!--
  prismatic joint will move alone the axis of the shape extrude, not around it
  the axis of the gripper pole
 -->
 <!--
  since gripper pole is extrude alone the z-axis in the beginning,
  the cylinder will move along that axis. However, we rotate the shape
  around the y-axis by 90 degree. The cylinder will move along x-axis
-->
<link name="gripper_pole">
  <visual>
    <geometry>
      <cylinder length="0.2" radius=".01"/>
    </geometry>
    <origin rpy="0 1.57075 0 " xyz="0.1 0 0"/>
  </visual>
</link>
<joint name="gripper_extension" type="prismatic">
  <parent link="base_link"/>
  <child link="gripper_pole"/>
  <limit effort="1000.0" lower="-0.38" upper="0" velocity="0.5"/>
  <origin rpy="0 0 1.57075" xyz="0 0.19 .2"/>
</joint>
```

### Adding Physical and Collision Properties to a URDF Model
#### Collision
```xml
<!--
  Tutorial: http://wiki.ros.org/urdf/Tutorials/Adding%20Physical%20and%20Collision%20Properties%20to%20a%20URDF%20Model
 -->
<!--
  To run our model in Gazebo,
  we need to enable the physics engine by adding collision tag
-->
<link name="base_link">
  <visual>
    <geometry>
      <cylinder length="0.6" radius="0.2"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 .8 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.6" radius="0.2"/>
    </geometry>
  </collision>
</link>
```
#### Inertial
```xml
<link name="base_link">
  <visual>
    <geometry>
      <cylinder length="0.6" radius="0.2"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 .8 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.6" radius="0.2"/>
    </geometry>
  </collision>
  <inertial>
    <!-- the unit for mass is kilogram -->
    <mass value="10"/>
    <!-- this is a 3x3 matrix. Since it is symmetric, we only need 6 elements -->
    <inertia ixx="0.4" ixy="0.0" ixz="0.0" iyy="0.4" iyz="0.0" izz="0.2"/>
  </inertial>
</link>
```

### Use XACRO to clean up URDF
```python
# typical way to convert xacro to urdf
$ rosrun xacro xacro model.xacro > model.urdf
```

```xml
<!-- automatically generate urdf file from xacro in launch file -->
<param name="robot_description"
  command="$(find xacro)/xacro '$(find pr2_description)/robots/pr2.urdf.xacro'" />
<!-- to use(spawn) this model in gazebo-->
<node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen"
	args="-urdf -model pr2 -param robot_description"/>
```

```xml
<!--
At the top of the URDF file, you must specify a namespace
in order for the file to parse properly.
For example, these are the first two lines of a valid xacro file:
 -->
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="firefighter">
```
#### Use It as Constant
##### URDF
```xml
<link name="base_link">
   <visual>
     <geometry>
       <cylinder length="0.6" radius="0.2"/>
     </geometry>
     <material name="blue">
       <color rgba="0 0 .8 1"/>
     </material>
   </visual>
   <collision>
     <geometry>
       <cylinder length="0.6" radius="0.2"/>
     </geometry>
   </collision>
 </link>
```
##### to XACRO
```xml
<!-- we use xacro to create constant that we use multiple times -->
<xacro:property name="width" value=".2" />
<xacro:property name="bodylen" value=".6" />
<link name="base_link">
    <visual>
        <geometry>
            <cylinder radius="${width}" length="${bodylen}"/>
        </geometry>
        <material name="blue">
            <color rgba="0 0 .8 1"/>
        </material>
    </visual>
    <collision>
        <geometry>
            <cylinder radius="${width}" length="${bodylen}"/>
        </geometry>
    </collision>
</link>
```
##### Additional Explanation
```xml
<!--
  The value of the contents of the ${} construct are then used to replace the ${}.
  This means you can combine it with other text in the attribute.
 -->
 <xacro:property name=”robotname” value=”marvin” />
 <link name=”${robotname}s_leg” />

 <!-- this will become -->
 <link name=”marvins_leg” />
```

#### Use Math with XACRO
```xml
<!--
  we can use basic calculation(+,-,*,/) with these constants/macros we create
  Exponentiation and modulus are not supported currently
  remember to put the calculation inside ${}
-->
<cylinder radius="${wheeldiam/2}" length=".1"/>
<origin xyz="${reflect*(width+.02)} 0 .25" />

<!-- example -->
<link name="${5/6}"/>
<!-- become -->
<link name="0.833333333333"/>
```

#### Macro
##### Simple Macro
```xml
<!-- create a macro "degauly_origin" so that we can use this macro multiple times -->
<xacro:macro name="default_origin">
    <origin xyz="0 0 0" rpy="0 0 0"/>
</xacro:macro>
<xacro:default_origin />

<!-- with above code, it becomes -->
<origin rpy="0 0 0" xyz="0 0 0"/>

<!--
1. Every instance of the <xacro:$NAME /> is replaced
   with the contents of the xacro:macro tag.
2. If the xacro with a specified name is not found,
   it will not be expanded and will NOT generate an error.
-->
```
##### Parametrized Macro
```xml
<!-- we can parametrize the macro to make it flexible-->
<xacro:macro name="default_inertial" params="mass">
    <inertial>
            <mass value="${mass}" />
            <inertia ixx="1.0" ixy="0.0" ixz="0.0"
                 iyy="1.0" iyz="0.0"
                 izz="1.0" />
    </inertial>
</xacro:macro>

<!-- we will use thsi macro by assign value to params we create -->
<xacro:default_inertial mass="10"/>
```

```xml
<!-- use entire block as parameter -->
<!--
1. To specify a block parameter, include an asterisk before its parameter name.
2. A block can be inserted using the insert_block command
3. Insert the block as many times as you wish.
 -->
<xacro:macro name="blue_shape" params="name *shape">
    <link name="${name}">
        <visual>
            <geometry>
                <xacro:insert_block name="shape" />
            </geometry>
            <material name="blue">
                <color rgba="0 0 .8 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <xacro:insert_block name="shape" />
            </geometry>
        </collision>
    </link>
</xacro:macro>

<xacro:blue_shape name="base_link">
    <cylinder radius=".42" length=".01" />
</xacro:blue_shape>
```
##### Sample
```xml

<xacro:property name="length" value=".6" />
<xacro:property name="width" value=".2" />
<xacro:property name="pi" value="3.14159" />

<xacro:macro name="leg" params="prefix reflect">
    <link name="${prefix}_leg">
        <visual>
            <geometry>
                <box size="${leglen} .2 .1"/>
            </geometry>
            <origin xyz="0 0 -${leglen/2}" rpy="0 ${pi/2} 0"/>
            <material name="white">
                <color rgba="1 1 1 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <box size="${leglen} .2 .1"/>
            </geometry>
            <origin xyz="0 0 -${leglen/2}" rpy="0 ${pi/2} 0"/>
        </collision>
        <xacro:default_inertial mass="10"/>
    </link>

    <joint name="base_to_${prefix}_leg" type="fixed">
        <parent link="base_link"/>
        <child link="${prefix}_leg"/>
        <origin xyz="${reflect*(width+.02)} 0 .25" />
    </joint>
    <!-- A bunch of stuff cut -->
</xacro:macro>
<xacro:leg prefix="right" reflect="1" />
<xacro:leg prefix="left" reflect="-1" />
```
