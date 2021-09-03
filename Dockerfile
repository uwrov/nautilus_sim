# syntax=docker/dockerfile:1
FROM nautilus_sim_base:latest

# switch to /root
ENV DIRPATH /root
WORKDIR $DIRPATH

ENV GZWEB_PATH ${DIRPATH}/gzweb
ENV GZWEB_ASSETS ${DIRPATH}/gzweb/http/client/assets
ENV MODELDIR ${DIRPATH}/models

COPY nautilus_worlds nautilus_worlds
COPY models models

COPY setup.sh /usr/share/gazebo/

EXPOSE 8080

# SHELL /bin/bash -c "source /root/.bashrc && /usr/share/gazebo/setup.sh"
SHELL ["/bin/bash", "-c", "source /root/.bashrc && /usr/share/gazebo/setup.sh"]