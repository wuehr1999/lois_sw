FROM arm64v8/ubuntu:20.04 

USER root

RUN apt-get update -y
RUN apt-get install -y apt-utils
RUN apt-get install -y locales
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

RUN apt-get install -y software-properties-common
RUN add-apt-repository universe

RUN apt-get update -y && apt-get install -y curl
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update -y
RUN apt-get install -y ros-galactic-desktop
RUN apt-get install -y ros-dev-tools

RUN apt-get install -y bash
RUN apt-get install -y git
RUN apt-get -y update --fix-missing
RUN apt-get -y install python3 python3-pip
RUN apt-get -y install python3-opencv
RUN apt-get -y install ros-galactic-rviz2
RUN apt-get -y install ros-galactic-rosbridge-server
RUN apt-get -y install tmux
RUN pip3 install pyserial
