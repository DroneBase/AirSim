FROM ubuntu:22.04

ARG ROS_UID=1000
ENV ROS_UID=${ROS_UID}
ARG ROS_GID=1000
ENV ROS_GID=${ROS_GID}

SHELL ["/bin/bash", "-c"]

ENV ROS_USER=ros
ENV ROS_USER_HOME=/home/ros

RUN groupadd -g ${ROS_UID} ${ROS_USER} && \
    useradd ${ROS_USER} -u ${ROS_UID} -g ${ROS_GID} -r -m -d ${ROS_USER_HOME} -s /bin/bash

RUN apt-get update && \
    apt-get install -y python3-pip && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean

USER ${ROS_USER}

RUN python3 -m pip install \
        rosbags \
        rosbags-image \
        pyexiv2

ADD --chown=${ROS_USER} ./extract_images.py ${ROS_USER_HOME}/extract_images.py

ENTRYPOINT ["/home/ros/extract_images.py"]

