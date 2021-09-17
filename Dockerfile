# syntax=docker/dockerfile:1
FROM uwrov/sim_base:latest

# switch to /root
ENV DIRPATH /root
WORKDIR $DIRPATH

ENV GZWEB_PATH ${DIRPATH}/gzweb
ENV GZWEB_ASSETS ${DIRPATH}/gzweb/http/client/assets

# COPY models models
COPY models/* ${GZWEB_ASSETS}

COPY setup.sh /usr/share/gazebo/

# Build ros packages
RUN mkdir -p /root/catkin_ws/src
COPY nautilus_worlds catkin_ws/src/nautilus_worlds
RUN . /opt/ros/noetic/setup.sh && cd /root/catkin_ws && catkin_make

RUN echo ". /usr/share/gazebo/setup.sh" >> /root/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

EXPOSE 8080