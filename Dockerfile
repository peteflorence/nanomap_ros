FROM ros:kinetic-ros-base-xenial

RUN  apt-get update -y && apt-get install -y --no-install-recommends wget

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
RUN wget http://packages.ros.org/ros.key -O - | apt-key add -

RUN apt-get update -y && apt-get install -y --no-install-recommends \
python-catkin-tools \
ros-kinetic-tf \
ros-kinetic-tf2 \
ros-kinetic-cv-bridge \
ros-kinetic-image-transport \
ros-kinetic-orocos-kdl

WORKDIR /catkin_workspace

ADD . /catkin_workspace/src/nanomap_ros