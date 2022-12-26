FROM osrf/ros:galactic-desktop 

USER root

RUN apt-get -y update --fix-missing
RUN apt-get -y install python3 python3-pip
RUN apt-get -y install python3-opencv
RUN apt-get -y install ros-galactic-rviz2
RUN apt-get -y install ros-galactic-rosbridge-server
RUN apt-get -y install tmux
RUN pip3 install pyserial
