#!/bin/bash

#reason of this script is that dockerfile only execute one command at the time but we need sometimes at the moment we create 
#the docker image to run more that one software for expecified configuration like when you need mysql running to chnage or create
#database for the container ...

  rosdep init
  echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  apt-get update
  apt-get install python-rosinstall
  source /opt/ros/indigo/setup.bash
  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws/src/
  catkin init
  cd ~/catkin_ws/
  catkin build
  
  
  apt-get clean
  rm -rf /tmp/* /var/tmp/*
  rm -rf /var/lib/apt/lists/*
