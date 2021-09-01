# syntax=docker/dockerfile:1
FROM ros:melodic-ros-core-bionic

ENV NODE_VERSION 11

# Install dependencies
RUN apt-get update && apt-get install -y \
    gazebo9 \
    libgazebo9-dev \
    libjansson-dev \
    libboost-dev \ 
    imagemagick \
    libtinyxml-dev \
    mercurial \
    cmake \
    build-essential \
    curl \
    git \
    ros-melodic-gazebo-ros-pkgs \
    ros-melodic-gazebo-ros-control

# Install gzweb
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.35.3/install.sh | bash
RUN . ~/.bashrc \
  && nvm install $NODE_VERSION \
  && nvm alias default $NODE_VERSION \
  && nvm use default

# build gzweb
RUN git clone https://github.com/osrf/gzweb
RUN . ~/.bashrc && . /usr/share/gazebo/setup.sh \
   && cd /gzweb \
   && npm run deploy

EXPOSE 8080