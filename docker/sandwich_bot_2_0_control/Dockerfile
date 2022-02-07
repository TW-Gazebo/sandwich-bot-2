FROM ubuntu:focal

ARG ROS_DISTRO=foxy
ENV ROS_DISTRO ${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive

ENV TZ=Etc/UTC
RUN echo $TZ > /etc/timezone && \
    ln -s /usr/share/zoneinfo/$TZ /etc/localtime

RUN apt-get update \
   && apt-get install -y --no-install-recommends \
        build-essential \
        lsb-release \
        sudo \
        gpg \
        wget \
        gnupg2 \
        curl \
        git \
   && apt-get clean

RUN echo "deb http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2-latest.list \
    && apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN sudo apt-get update \
    && sudo apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-ros-base \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        ros-${ROS_DISTRO}-xacro \
    && sudo apt-get clean

RUN apt-get update \
   && apt-get install -y --no-install-recommends \
      python3-rosdep \
      python3-colcon-ros \
      python3-colcon-common-extensions \
   && rosdep init \
   && rosdep update 
   
ENV ROS2_WS /app/ros2_ws
RUN mkdir -p $ROS2_WS/src
WORKDIR $ROS2_WS

COPY sandwich_bot_2_0_control $ROS2_WS/src/sandwich_bot_2_0_control
COPY sandwich_bot_2_0_description $ROS2_WS/src/sandwich_bot_2_0_description
COPY sandwich_bot_2_0_interfaces $ROS2_WS/src/sandwich_bot_2_0_interfaces

RUN apt-get update \
    && rosdep update \
    && rosdep install -y -i --from-paths src --rosdistro ${ROS_DISTRO}
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --merge-install

COPY docker/sandwich_bot_2_0_control/entrypoint.sh /app/

ENTRYPOINT ["/app/entrypoint.sh"]
CMD ros2 launch sandwich_bot_2_0_control box_picker_start.py use_sim_time:=True