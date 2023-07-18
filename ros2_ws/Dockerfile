ARG ROS_DISTRO=humble
ARG WORKSPACE=/ros2_ws

FROM osrf/ros:${ROS_DISTRO}-desktop as base
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

RUN mkdir -p ${WORKSPACE}/src
#COPY requirements.txt ${WORKSPACE}/

ARG WORKSPACE
WORKDIR ${WORKSPACE}
COPY requirements.txt .
RUN apt-get update -y \
 && apt-get upgrade -y \
 #&& apt-get install ros-${ROS_DISTRO}-rmw-cyclonedds-cpp -y --no-install-recommends \
 && apt-get install python3-pip -y \
 && python3 -m pip install -r requirements.txt

#ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

FROM base as overlay
ARG WORKSPACE
WORKDIR ${WORKSPACE}
COPY ./src/ ./src/


#CMD ["ros2", "node", "list"]

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && colcon build


FROM overlay as run_time
ARG WORKSPACE
WORKDIR ${WORKSPACE}
#RUN source install/setup.bash
COPY ./config/fastrtps-profile.xml ${WORKSPACE}/config/fastrtps-profile.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=${WORKSPACE}/config/fastrtps-profile.xml

COPY ./entrypoints/entrypoint.sh /
RUN chmod +x /entrypoint.sh

ENTRYPOINT [ "/entrypoint.sh" ]
#CMD ["ros2", "run", "nucleus_node", "nucleus_node"]
