# A Python Helper Module for the Waffles  

The `tuos_examples/tb3.py` module [as introduced in COM2009 Assignment #1 Part 5](https://tom-howard.github.io/ros/com2009/assignment1/part5/#cam_swp_act_srv), is a *very minimal* working example of how to modularise your Python code and wrap some key operations (publishing to `/cmd_vel`; subscribing to `/odom`, for example) inside some external Python Classes.

This is by no means optimal though, and it also isn't particularly robust when it comes to working with the real robots, which have [some key differences to a simulated Waffle](https://tom-howard.github.io/ros/waffles/fact-finding/).

Building on this then, the `tuos_examples/waffle.py` module works similarly to `tb3.py`, but is more suited to real world applications (it works pretty well in simulation as well though).

## Installation and Usage

[Install or update the `tuos_ros` Course Repo](https://tom-howard.github.io/ros/extras/tuos-ros/) to get the most up-to-date version of `waffle.py`. You are welcome to use/adapt this as you wish for the COM2009 Assignment work.

The `waffle.py` module should be copied into your own package's `src` directory, so that all the nodes here can access it.

```py
import waffle
```

There are then three classes that can be used:

1. `Motion()`: Creates a `/cmd_vel` publisher and has callable methods to control velocity
1. `Lidar()`: Creates a `/scan` subscriber and handles `LaserScan` data from the Waffle's LiDAR sensor
1. `Pose()`: Creates an `/odom` subscriber and handles the robot's `Odometry` data

```py
motion = waffle.Motion()
lidar = waffle.Lidar()
odom = waffle.Pose()
```

## Usage Example

[See `usage_example.py` for an example of how to use `waffle.py`](./usage_example.py).

## Classes

### `Motion(debug = True)`

A `/cmd_vel` publisher.

Optional:
* Set `debug=True/False` to enable/disable ROS log/warn messages (default = `True`)

### Methods

`set_velocity(linear = 0.0, angular = 0.0)`

* Set up a `Twist()` message with appropriate `linear` (m/s) and `angular` (rad/s) velocities.

`publish_velocity()`

* Publishes the `Twist()` message as set by `set_velocity()`.  

`stop()`

* Publishes an appropriate velocity command to make the robot stop moving.

### `Pose(debug = True)`

An `/odom` subscriber.

Optional:
* Set `debug=True/False` to enable/disable ROS log/warn messages (default = `True`)

#### Methods

`show()`

* Prints the robot's current odometry to the terminal at a rate of 1Hz (regardless of the rate at which the method is actually called).

#### Attributes 

`posx`

* Returns the robot's current position in the X-axis (meters).

`posy`

* Returns the robot's current position in the Y-axis (meters).

`yaw`

* Returns the robot's current *absolute* orientation about the Z-axis (degrees).

`yaw_direction`

* Returns +1 or -1 to indicate whether the robot is oriented to the left or right of it's principal X-axis.

``

### `Lidar()`

A `/scan` subscriber.

Optional:
* Set `debug=True/False` to enable/disable ROS log/warn messages (default = `True`)

#### `subsets` (Sub-class)

Distance (in meters) from 9 subsets of the `LaserScan.ranges` array:

![](https://tom-howard.github.io/ros/others/amr31001/lab2/lidar_segments.png?width=500px)

##### Attributes

For each of the 9 subsets (as shown in the figure) a **single distance value** can be obtained from the `subsets` class, which represents the **average** distance to any object(s) within that particular zone:

* `subsets.front`: returns the average distance to any object(s) in front of the robot (within the frontal zone).
* `subsets.l1`: returns the average distance to any object(s) located within LiDAR zone L1.
* `subsets.r1`: returns the average distance to any object(s) located within LiDAR zone R1.

    and so on...

Out of range LiDAR readings (both `0.0` and `inf`) are removed before obtaining the average, if no data remains within the segment then the average distance value is reported as `nan`.

For each of the 9 subsets (as shown in the figure) the **full array of distance readings** can be obtained, returned as Python Lists: 

* `subsets.frontArray`: all distance readings from the front LiDAR subset.
* `subsets.l1Array`: all distance readings from LiDAR zone L1.
* `subsets.r1Array`: all distance readings from LiDAR zone R1.

    and so on...

Out of range LiDAR readings (both `0.0` and `inf`) are removed, if no data remains then an empty list will be returned `[]`.

##### Methods

`subsets.show()`

* Prints the distance values from all 9 scan subsets to the terminal at a rate of 1Hz (regardless of the rate at which the method is actually called).

