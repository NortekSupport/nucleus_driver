ARG ROS_DISTRO=humble
ARG WORKSPACE=/ros2


FROM osrf/ros:${ROS_DISTRO}-desktop as base
ARG WORKSPACE
WORKDIR ${WORKSPACE}
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

COPY requirements.txt .
RUN apt-get update -y \
 && apt-get upgrade -y \
 && apt-get install python3-pip -y \
 && python3 -m pip install -r requirements.txt


FROM base as overlay
ARG WORKSPACE
WORKDIR ${WORKSPACE}

COPY ./src/ ./src/
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && colcon build


FROM overlay as run_time
ARG WORKSPACE
WORKDIR ${WORKSPACE}

COPY ./config/fastrtps-profile.xml ${WORKSPACE}/config/fastrtps-profile.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=${WORKSPACE}/config/fastrtps-profile.xml

COPY ./entrypoints/entrypoint.sh /
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]
