#FROM arm64v8/ubuntu:20.04 
FROM ubuntu:20.04
USER root

WORKDIR /usr/src/app

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update -yq 
RUN apt-get install -yq apt-utils
RUN apt-get install -yq locales
RUN apt-get install -yq tzdata
RUN echo "Europe/Berlin" > /etc/timezone && \
    touch /etc/locale.gen && \
    dpkg-reconfigure -f noninteractive tzdata && \
    sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen && \
    sed -i -e 's/# de_DE.UTF-8 UTF-8/de_DE.UTF-8 UTF-8/' /etc/locale.gen && \
    echo 'LANG="de_DE.UTF-8"'>/etc/default/locale && \
    dpkg-reconfigure --frontend=noninteractive locales && \
    update-locale LANG=de_DE.UTF-8

ENV LANG de_DE.UTF-8
ENV LANGUAGE de_DE.UTF-8
ENV LC_ALL de_DE.UTF-8

RUN apt-get install -yq software-properties-common
RUN add-apt-repository universe

RUN apt-get update -yq && apt-get install -yq curl
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update -yq
RUN apt-get install -yq ros-galactic-desktop
RUN apt-get install -yq ros-dev-tools

RUN apt-get install -yq bash
RUN apt-get install -yq git
RUN apt-get -yq update --fix-missing
RUN apt-get -yq install python3 python3-pip
RUN apt-get -yq install python3-opencv
RUN apt-get -yq install ros-galactic-rviz2
RUN apt-get -yq install ros-galactic-rosbridge-server
RUN apt-get -yq install tmux
RUN pip3 install pyserial
RUN apt-get -yq install ros-galactic-realsense*
RUN apt-get -yq install ros-galactic-librealsense*
RUN apt-get -yq install ros-galactic-rtabmap*
RUN apt-get -yq install ros-galactic-compressed-image-transport

RUN apt-get -yq install chrony
COPY ./ws/chrony/chrony.conf /etc/chrony
