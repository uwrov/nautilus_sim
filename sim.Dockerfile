# syntax=docker/dockerfile:1
FROM ros:melodic-ros-core-bionic

ENV ROS_DISTRO melodic
ENV NODE_VERSION 11

# Install dependencies
RUN apt-get update && apt-get install -y \
    gazebo9 \
    libgazebo9-dev \
    libjansson-dev \
    libboost-dev \ 
    imagemagick \
    libtinyxml-dev \
    cmake \
    build-essential \
    curl wget vim git \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-gazebo-ros-control \
    xvfb

# Install nvm
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.35.3/install.sh | bash
RUN . ~/.bashrc \
  && nvm install $NODE_VERSION \
  && nvm alias default $NODE_VERSION \
  && nvm use default

WORKDIR /~

# install and build gzweb
RUN git clone https://github.com/osrf/gzweb
RUN . ~/.bashrc && . /usr/share/gazebo/setup.sh \
   && cd gzweb \
   && ./deploy.sh -m -t

EXPOSE 8080


