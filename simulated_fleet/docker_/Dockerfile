# Use the official image for ROS2 Jazzy under Ubuntu 24.04
FROM osrf/ros:jazzy-desktop-full-noble

ARG ROS_DOMAIN_ID
ARG DEBIAN_FRONTEND=noninteractive
ARG TZ=Europe/Berlin
ARG PYTHON_VENV_PATH=/opt/ros_venv

##############################################################################
# 1) Base environment
##############################################################################
ENV TZ=${TZ}
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV DEBIAN_FRONTEND=${DEBIAN_FRONTEND}

RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

##############################################################################
# 2) Install apt dependencies
##############################################################################
# - Combine apt-get update & install
RUN apt-get update && apt-get install -y \
    tmux \
    python3-pip \
    python3-venv \
    gosu \
    ssh \
    curl \
    sudo \
    iputils-ping \
    libyaml-cpp-dev \
    ros-$ROS_DISTRO-foxglove-bridge \
    ros-${ROS_DISTRO}-topic-tools \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
 && rm -rf /var/lib/apt/lists/*

##############################################################################
# 3) Install pip dependencies in virtual environment
##############################################################################
RUN python3 -m venv ${PYTHON_VENV_PATH} --system-site-packages && \
    ${PYTHON_VENV_PATH}/bin/pip install --no-cache-dir --upgrade pip && \
    . ${PYTHON_VENV_PATH}/bin/activate && \
    pip install --no-cache-dir \
      PyYAML \
      catmux

##############################################################################
# 4) Environment settings
##############################################################################
# We'll store a script that sources the venv + /ros1_ws/devel + jazzy
RUN echo "source ${PYTHON_VENV_PATH}/bin/activate" >> /root/.bashrc
RUN echo 'source /opt/ros/jazzy/setup.bash' >> /root/.bashrc
RUN echo "export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST" >> /root/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc
RUN echo "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" >> /root/.bashrc
RUN echo 'export PYTHONPATH="'"${PYTHON_VENV_PATH}"'/lib/python3.12/site-packages:$PYTHONPATH"' >> /root/.bashrc
RUN echo 'alias catmux="tmux -L catmux"' >> /root/.bashrc

##############################################################################
# 5) Clean up
##############################################################################
# Delete unused user which is created by default in the ubuntu image
RUN userdel -r $(id -un 1000)

##############################################################################
# 6) EntryPoint
##############################################################################
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]