Using URDF in Gazebo
===
### Convert URDF file to Gazebo
#### Required
```xml
An <inertia> element within each <link> element must be properly specified
and configured
```
#### Optional
```xml
Add a <gazebo> element for every <link>
  * Convert visual colors to Gazebo format
  * Convert stl files to dae files for better textures
  * Add sensor plugins

Add a <gazebo> element for every <joint>
  * Set proper damping dynamics
  * Add actuator control plugins

Add a <gazebo> element for the <robot> element

Add a <link name="world"/> link if the robot should be rigidly attached to
the world/base_link
```

### Get Started with RRBot
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

# View in Gazebo
$ roslaunch rrbot_gazebo rrbot_world.launch

# If encounter this error:
# Failed to load plugin libgazebo_ros_control.so: libgazebo_ros_control.so:
# cannot open shared object file: No such file or directory
# install this
$ sudo apt-get install ros-indigo-gazebo-ros-control
```

### <inertial> Element
```xml
<!--
  For the Gazebo physics engine to work properly, the <inertial> element must
  be provided as documented on the URDF link element page. For links to not
  be ignored in Gazebo, their mass must be greater than zero. Additionally,
  links with zero principal moment of inertia (ixx, iyy, izz) could lead
  to infinite acceleration under any finite torque application.
 -->

<inertial>
  <origin xyz="0 0 ${height1/2}" rpy="0 0 0"/>
  <mass value="1"/>
  <inertia
    ixx="1.0" ixy="0.0" ixz="0.0"
    iyy="1.0" iyz="0.0"
    izz="1.0"/>
</inertial>
```

### "gazebo" Element for Tag
```xml
Name	           Type	    Description
material	       value	  Material of visual element
gravity	         bool	    Use gravity
dampingFactor    double	  Exponential velocity decay of the link velocity
                          - takes the value and multiplies the previous link
                          velocity by (1-dampingFactor).
maxVel	         double	  maximum contact correction velocity truncation term.
minDepth	       double	  minimum allowable depth before contact correction
                          impulse is applied
mu1/mu2	         double	  Friction coefficients μ for the principle contact
                          directions along the contact surface as defined
                          by the [ODE](http://www.ros.org/wiki/opende)
fdir1	           string	  3-tuple specifying direction of mu1 in the collision
                          local reference frame.
kp	             double	  Contact stiffness k_p for rigid body contacts as
                          defined by [ODE](http://opende.sourceforge.net/)
                          (ODE uses cfm)
kd	             double	  Contact damping k_d for rigid body contacts as
                          defined by [ODE](http://opende.sourceforge.net/)
                          (ODE uses erp)
selfCollide	     bool	    If true, the link can collide with other links in
                          the model.
maxContacts	     int	    Maximum number of contacts allowed between two
                          entities. This value overrides the max_contacts
                          element defined in physics.
laserRetro	     double	  intensity value returned by laser sensor.
```

### Verifying the Gazebo Model Works
```python
$ gz sdf -p MODEL.urdf
```

### How Gazebo ROS Package Looks Like
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
### Convert STL to URDF (use it as mesh)
```xml
<!--
  File: MYROBOT.stl
  PATH: ~/catkin_ws/src/MYROBOT_description/meshes/MYROBOT.stl
-->
<robot name="test_model">
  <link name="test_body">
    <inertial>
      <mass value="0.1" />
      <origin xyz="0 0 0" />
      <inertia  ixx="1.0" ixy="0.0"  ixz="0.0"  iyy="1.0"  iyz="0.0"  izz="1.0" />
    </inertial>
    <visual>
      <!-- visual origin is defined w.r.t. link local coordinate system -->
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <!-- Import the STL file in the package testbot_description -->
        <mesh filename="package://testbot_description/meshes/test.STL" scale="0.01 0.01 0.01"/>
      </geometry>
    </visual>
    <collision>
      <!-- collision origin is defined w.r.t. link local coordinate system -->
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <!-- Import the STL file in the package testbot_description -->
        <mesh filename="package://testbot_description/meshes/test.STL"/>
      </geometry>
    </collision>
  </link>
  <gazebo reference="test_body">
    <material>Gazebo/LightWood</material>
    <turnGravityOff>true</turnGravityOff>
  </gazebo>
</robot>
```

```xml
<!--
  File: MYROBOT.launch
  PATH: ~/catkin_ws/src/testbot_gazebo/launch/MYROBOT.launch
 -->
<!-- Spawn a robot into Gazebo -->
  <node name="testSpawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-file $(find MYROBOT_description)/urdf/MYROBOT.urdf -urdf -z 1 -model MYROOBOT" />
```
### Some Helpful Tutorial:
```python
#Tutorial: Using Gazebo plugins with ROS:
http://gazebosim.org/tutorials?tut=ros_gzplugins

```
