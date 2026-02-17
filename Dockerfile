FROM ubuntu:focal

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update &&  apt-get install -y git iproute2 curl sudo \
ca-certificates wget

# Hack so that tzdata does not request stdin input during install.
RUN ln -fs /usr/share/zoneinfo/Europe/Oslo /etc/localtime

ARG name_os_version="focal"
ARG name_ros_version="noetic"
ARG name_catkin_workspace="catkin_ws"

RUN apt install -y chrony ntpdate curl build-essential
#RUN ntpdate ntp.ubuntu.com

RUN echo "deb http://packages.ros.org/ros/ubuntu ${name_os_version} main" > /etc/apt/sources.list.d/ros-latest.list
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -


RUN apt-get update
RUN apt install -y ros-${name_ros_version}-desktop-full
RUN apt install -y ros-${name_ros_version}-rqt-* ros-${name_ros_version}-gazebo-*
RUN apt install -y python3-rosinstall python3-rosinstall-generator python3-wstool build-essential git

RUN apt install python3-rosdep
RUN sh -c "rosdep init"
RUN rosdep update

RUN apt-get update && apt-get install -y \
ros-noetic-joy ros-noetic-teleop-twist-joy \
ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
ros-noetic-rosserial-python ros-noetic-rosserial-client \
ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers

RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws/src

ENV TURTLEBOT3_MODEL=waffle_pi

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
#RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

RUN echo "alias eb='nano ~/.bashrc'" >> ~/.bashrc
RUN echo "alias sb='source ~/.bashrc'" >> ~/.bashrc
RUN echo "alias gs='git status'" >> ~/.bashrc
RUN echo "alias gp='git pull'" >> ~/.bashrc
RUN echo "alias cw='cd ~/${name_catkin_workspace}'" >> ~/.bashrc
RUN echo "alias cs='cd ~/${name_catkin_workspace}/src'" >> ~/.bashrc
RUN echo "alias cm='cd ~/${name_catkin_workspace} && catkin_make'" >> ~/.bashrc

RUN echo "source /opt/ros/${name_ros_version}/setup.bash" >> ~/.bashrc
RUN echo "source ~/${name_catkin_workspace}/devel/setup.bash" >> ~/.bashrc

RUN echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc
RUN echo "export ROS_HOSTNAME=localhost" >> ~/.bashrc

WORKDIR /root/catkin_ws

ENTRYPOINT [ "/bin/bash" ]
