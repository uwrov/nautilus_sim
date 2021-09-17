# syntax=docker/dockerfile:1
FROM ros:noetic-ros-core-focal

ENV ROS_DISTRO noetic
ENV NODE_VERSION 11

# Install general utilities
RUN apt-get update && apt-get install -y \
    imagemagick \
    cmake \
    build-essential \
    curl \
    git \
    xvfb

# Install gazebo and dependencies
RUN apt-get update && apt-get install -y \
    gazebo11 \
    libgazebo11-dev \
    libjansson-dev \
    libboost-dev \ 
    libtinyxml-dev \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-ros-control

# Install nvm
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.35.3/install.sh | bash
RUN . ~/.bashrc \
  && nvm install $NODE_VERSION \
  && nvm alias default $NODE_VERSION \
  && nvm use default

# switch to /root
WORKDIR /root

# install and build gzweb
RUN git clone https://github.com/osrf/gzweb
RUN rm -rf /root/gzweb/http/client/assets

COPY models/ /root/gzweb/http/client/assets/

RUN . ~/.bashrc && . /usr/share/gazebo/setup.sh \
   && cd gzweb \
   && ./deploy.sh -m local

EXPOSE 8080


