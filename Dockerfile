# syntax=docker/dockerfile:1
FROM nautilus_sim:latest

WORKDIR /~

# install custom packages
COPY nautilus_* ~/catkin_ws/src
RUN cd ~/catkin_ws/src && catkin_make