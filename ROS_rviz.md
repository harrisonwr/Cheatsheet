ROS rviz
===
### User Guide
#### Built-in Display Types
##### Axis
```python
# Tutorial: http://wiki.ros.org/rviz/UserGuide
The Axes display shows a set of axes, located at the origin
of the target frame.
```
##### Effort
```python
The Effort display shows a sensor_msgs/JointState/effort message
as circled arrows around each revolute joint in the robot.
```
##### Camera
```python
The Camera display creates a new rendering window from the
perspective of a camera, and overlays the image from the camera on
top of it. For this display to work properly, the sensor_msgs/Image
topic subscribed to must be part of a camera, and must have a
sensor_msgs/CameraInfo topic named camera_info alongside it.

For example:
/wide_stereo/left/image_rect
/wide_stereo/left/camera_info

The Camera display assumes a z-forward x-left frame, rather than the
normal x-forward y-left robot frame.
```

##### Grid
```python
The Grid display shows a grid of lines, centered at the origin of
the target frame
```

##### Grid Cells
```python
The GridCells display shows a nav_msgs::GridCells message, as upwards-facing billboards.

You need to specified which topic the grid cells subscribe to
```


### Creating a Scratch Package
```python
# Tutorial: http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes

# let's create a catkin package first, go to ~/catkin_ws/src
$ catkin_create_pkg using_markers roscpp visualization_msgs
```

### Sending Markers
```c
// Tutorial: http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes
// Create a file "basic_shapes.cpp" and paste the code
// (PATH: /catkin_ws/src/using_markers/src/)

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);

    // Cycle between different shapes
    switch (shape)
    {
    case visualization_msgs::Marker::CUBE:
      shape = visualization_msgs::Marker::SPHERE;
      break;
    case visualization_msgs::Marker::SPHERE:
      shape = visualization_msgs::Marker::ARROW;
      break;
    case visualization_msgs::Marker::ARROW:
      shape = visualization_msgs::Marker::CYLINDER;
      break;
    case visualization_msgs::Marker::CYLINDER:
      shape = visualization_msgs::Marker::CUBE;
      break;
    }

    r.sleep();
  }
}
```

```python
# Add these lines at the bottom of CMakeList.txt in your catkin package
# This enable the program to run basic_shapes.cpp
add_executable(basic_shapes src/basic_shapes.cpp)
target_link_libraries(basic_shapes ${catkin_LIBRARIES})
```
