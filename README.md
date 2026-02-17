# Assignment 2 using Docker

This lets you use Docker to launch a ROS Noetic environment.
It is used as a base for Assignment 2.

## Prerequisites

Installation is so far only supported on __Linux__ and __Windows__.
It is __not supported on macOS__

First, make sure you have
[Docker](https://docs.docker.com/get-started/get-docker/) installed on your
system. If you are on Windows, make sure you have the [Windows Subsystem for
Linux](https://learn.microsoft.com/en-us/windows/wsl/install) installed, and that you have installed [Docker in WSL](https://docs.docker.com/desktop/features/wsl/).

## Preparing the environment

Just as with the virtualbox install, you will need to clone dependencies through git.
Run

```
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b noetic https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b noetic https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b noetic https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```

## Building and running 

Importantly, in order for your graphical environment (X11) to trust Docker, you
must explicitly tell it to do so using this command:

`xhost +local:docker`

Enter the root directory of this project (where this file exists).
To build the Docker image, run:

`docker compose build`

To start the Docker container, run:

`docker compose run main`

Once inside the container, you have a `bash` terminal. You should already be at the ROS workspace
root folder, which is `/root/catkin_ws`. To build the ROS code, run (in this folder):

`catkin_make`

Then, you can source the ROS workspace using

`source devel/setup.bash`

You can then run a `roslaunch` file to launch a ROS application .
You can test that it works by running

```roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch```

## Making changes

The `catkin_ws` folder is mounted as a [bind
mount](https://docs.docker.com/engine/storage/bind-mounts/) in Docker.  This
means that any changes made to this folder will immediatley be reflected in the
Docker container, without the need to rebuild the container. It is thus
recommended to make changes directly in the folder outside the container, and
then use `catkin_make` and `roslaunch` inside the container to test the changes.
