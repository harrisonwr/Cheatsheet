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

### Sending Basic Shapes
#### Sample Code
```c
// Tutorial: http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes
// Create a file "basic_shapes.cpp" and paste the code
// (PATH: /catkin_ws/src/using_markers/src/)

// We include ROS and visualization_msgs/Marker messages definition
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  // Initialize ROS and create a publisher for topic "visualization_marker"
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set our initial shape type to be a cube
  // The variable shape will be use to track what shape we're going to publish
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp. See the TF tutorials for
    // information on these.
    // In a running system this should be the frame relative to which
    // you want the marker's pose to be interpreted.
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

#### Build and Run the Code
```python
# Build
$ cd ~/catkin_ws
$ catkin_make

# Run
$ rosrun using_markers basic_shapes

# or you can view the marker by
# make sure rviz is built
$ rosmake rviz

$ rosrun using_markers basic_shapes
$ rosrun rviz rviz

# to visualize it
#   1. add a marker in rviz
#   2. put "my_frame" for fixed frame
```


### Points and Lines
#### Code
```c
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::Rate r(30);

  float f = 0.0;
  while (ros::ok())
  {

    visualization_msgs::Marker points, line_strip, line_list;
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/my_frame";
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = "points_and_lines";
    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;



    points.id = 0;
    line_strip.id = 1;
    line_list.id = 2;



    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;



    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;
    line_list.scale.x = 0.1;



    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // Line list is red
    line_list.color.r = 1.0;
    line_list.color.a = 1.0;



    // Create the vertices for the points and lines
    for (uint32_t i = 0; i < 100; ++i)
    {
      float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
      float z = 5 * cos(f + i / 100.0f * 2 * M_PI);

      geometry_msgs::Point p;
      p.x = (int32_t)i - 50;
      p.y = y;
      p.z = z;

      points.points.push_back(p);
      line_strip.points.push_back(p);

      // The line list needs two points for each line
      line_list.points.push_back(p);
      p.z += 1.0;
      line_list.points.push_back(p);
    }


    marker_pub.publish(points);
    marker_pub.publish(line_strip);
    marker_pub.publish(line_list);

    r.sleep();

    f += 0.04;
  }
}
```

#### Bulid and Run Code
```python
$ cd ~/catkin_ws
$ catkin_make

$ rosrun rviz rviz
$ rosrun using_marker points_and_lines

# to visualize it
#   1. add a marker in rviz
#   2. put "my_frame" for fixed frame
```
