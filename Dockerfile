# Pulling ubuntu 20.04 image, setting shell to bash.
FROM ubuntu:20.04
SHELL ["/bin/bash", "-c"]
LABEL version="0.1"
LABEL description="Docker Image for Ardupilot Simulations with Gazebo"

# Set variables username and timezone. Default: user = user and empty password
ENV USER="user"
ENV TZ="America/Mexico_City"
ENV ROSDISTRO="noetic"

# Following section creates the user and sets the time zone.
RUN DEBIAN_FRONTEND=noninteractive && apt-get update && apt-get install -y sudo \
	&& adduser --disabled-password --gecos "" --shell /bin/bash $USER \
 	&& usermod -g sudo $USER \
 	&& passwd -d $USER \
	&& apt-get install -yq tzdata \
    && ln -fs /usr/share/zoneinfo/$TZ /etc/localtime \
    && dpkg-reconfigure -f noninteractive tzdata 

# ROS Noetic installation.
RUN sudo apt-get update && sudo apt-get install -y lsb-release \
    && sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && sudo apt-get install -y curl && sudo apt-get install -y gnupg \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - \
    && sudo apt-get update && sudo apt-get install -y ros-$ROSDISTRO-ros-base \
    && source /opt/ros/$ROSDISTRO/setup.bash \
    && echo 'source /opt/ros/$ROSDISTRO/setup.bash' >> ~/.bashrc \
    && source ~/.bashrc \
    && sudo apt-get install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool python3-catkin-tools build-essential \
    && sudo rosdep init \
    && rosdep update

# Login into user and moves to work directory.
USER $USER
WORKDIR /home/$USER/

# Git and Ardupilot installation.
RUN sudo apt-get update \
    && sudo apt-get install -y git \
    && sudo apt-get install -y gitk git-gui \
    && git clone https://github.com/ArduPilot/ardupilot.git && cd ardupilot \
    && git submodule update --init --recursive \
    && Tools/environment_install/install-prereqs-ubuntu.sh -y \
    && . ~/.profile \
    && ./waf configure --board sitl && ./waf copter

# MAVROS dependencies installation.
RUN sudo apt-get install -y ros-$ROSDISTRO-mavros ros-$ROSDISTRO-mavros-extras \
    && cd /opt/ros/$ROSDISTRO/lib/mavros && sudo bash install_geographiclib_datasets.sh

# GAZEBO 11 and dependencies installation.
RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
	&& wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - \
	&& sudo apt-get update \
	&& sudo apt-get install -y gazebo11 libgazebo11-dev \
	&& sudo apt-get install -y ros-$ROSDISTRO-gazebo-ros-pkgs ros-$ROSDISTRO-gazebo-ros-control
