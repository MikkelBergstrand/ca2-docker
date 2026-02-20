# Intro

Install tutorial program

`$ sudo apt-get install ros-<distro>-ros-tutorials`

Spawn turtle window

`rosrun turtlesim turtlesim_node`

Spawn teleop_key

`rosrun turtlesim turtle_teleop_key`


# rqt

Launch with

`rqt`

# Nodes

Run a node

`rosrun <package_name> <node_name>`

List active nodes

`rosnode list`

Info about an acive node

`rosnode info <node_name>`

# Topics

List active topics. `-v` shows more information.
`rostopic list`
`rostopic list -v`

See message type of topic:

`rostopic type <topic>`

Echo a topic (print all traffic to topic)

`rostopic echo <topic>`

Detailed information about topic types (in ROS lingo, topic types are called messages)

`rosmsg show <type>`

Manually publish to a topic. Use TAB complete here to make your life easier.

`rostopic pub <topic> <type> <data>`

Example: 

`rostopic pub /turtle1/cmd_vel geometry_msgs/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"`

# SERVICES

These commands (and more) are similar to interfacing with rostopic:

`rosservice list`
`rosservice type <service>`

Request a service:

`rosservice call <service> <data>`

Example:

`rosservice call /spawn 2 2 0.2 "Jimmy"`

Service message type: (note `rossrv` for server types, not `rosmsg` which are topic types)

`rossrv type <service>`

# Parameters 

Self-explanatory commands:

`rosparam list`
`rosparam get <node> <param_name>`
`rosparam set <node> <param_name> <value>`

Dump current parameters to file:

`rosparam dump > params.yaml`

Load parameters from file into active nodes: 

`rosparam load params.yaml`

# Launch files

Launch .launch files. Note that the .launch file must be located in the root of the
package directory, e.g. `catkin_ws/src/<pkg_name>/<launch_file_name>`
`roslaunch <pkg_name> <launch_file_name>`

# Bagging

To record data from selected topics:

`rosbag record -o <output_file_name> <topic1> <topic2> ...`

Record data from all topics:

`rosbag record -o <output_file_name> -a`

Play back a recording:

`rosbag play <bag_file>`

# Writing your own packages

## Workspace

A workspace contains multiple packages. It can be any folder, but here I will
call the folder `catkin_ws`. Create a folder inside this called `src`. Here,
all packages will be located.

## Packages

To create a new package. You should run this in the `src` folder.

`catkin_create_pkg <package_name> <dependency1> <dependency2> ...`

To set up nodes, it is recommended to create a folder called `scripts` inside the 
package. Inside this, you will create the Python files that are your nodes.

You must edit `CMakeLists.txt` to register the nodes:

```catkin_install_python(PROGRAMS scripts/node1.py scripts/node2.py ...
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)```

To build a package run in `catkin_ws`:

`catkin_make`

You should now have two additional folders alongside `src` called `build` and
`devel`. 

In order to use the nodes in your custom packages, you must source the
workspace. This is done with 

`source devel/source.bash`
