#Should set up a system to use GRL
#Includes dependencies for GrlInverseKinematics, PivotCalibration & HandEyeCalibration
#NB - This is a complicated docker file and might need to be __run twice__!

#Install on ubuntu 16.04

#Comment the following if you wish to use ROS 
#--------------Start--------------------

FROM ubuntu:16.04
CMD ["bash"]
#---------------End-----------------

#Uncomment the following if you wish to use ROS 
#------------Start---------------------
#FROM osrf/ros:kinetic-desktop

#RUN apt-get update && apt-get install -y \
#    ros-kinetic-desktop-full=1.3.0-0* \
#    && rm -rf /var/lib/apt/lists/*

#RUN mkdir -p /etc/my_init.d
#COPY startup.sh /etc/my_init.d/startup.sh
#RUN chmod +x /etc/my_init.d/startup.sh

#COPY pre-conf.sh /sbin/pre-conf
#RUN chmod +x /sbin/pre-conf \
#    && /bin/bash -c /sbin/pre-conf \
#    && rm /sbin/pre-conf
#CMD bash -c "/etc/my_init.d/startup.sh"; bash

#-------------End-----------------------------

LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

ENV HOME /root
WORKDIR /root

#Essentials
RUN apt-get clean && apt-get update \
    && rm -rf /tmp/* /var/tmp/* \
    && rm -rf /var/lib/apt/lists/*
      
#Compiling
RUN apt-get clean && apt-get update  && apt-get install -y \
     libtool \ 
     bash-completion \
     pkg-config \ 
     build-essential \ 
     autoconf \ 
     automake \ 
     cmake \ 
     cmake-curses-gui 
RUN apt-get clean && apt-get update  && apt-get install -y \
     git \ 
     vim \ 
     sudo \ 
     unzip \ 
     curl \ 
     ctags \ 
     git \ 
     tmux  
#LLVM
RUN apt-get clean && apt-get update  && apt-get install -y \
    clang \ 
    lldb 
#apt-repository-add scripts
RUN apt-get clean && apt-get update  && apt-get install -y \
    software-properties-common \ 
    python-software-properties 
#GRL dependencies
RUN apt-get clean && apt-get update  && apt-get install -y \
    libboost-all-dev \
    libeigen3-dev \
    gfortran 
#OpenCV
RUN apt-get clean && add-apt-repository --yes ppa:xqms/opencv-nonfree \
    && apt-get update \
    && apt-get -y install \
    libopencv-nonfree-dev \
    libopencv-dev


RUN cd ~\ 
    && git clone https://github.com/ahundt/robotics_setup.git  

RUN ["/bin/bash","-c","cd ~/robotics_setup && ~/robotics_setup/vrep.sh"]
RUN ["/bin/bash","-c","cd ~/robotics_setup && ~/robotics_setup/spdlog.sh"]
#two scripts below called from grl_kuka.sh
#RUN ["/bin/bash","-c","./flatbuffers.sh"]
#RUN ["/bin/bash","-c","./cmake-basis.sh"]
RUN ["/bin/bash","-c","cd ~/robotics_setup && ~/robotics_setup/robotics_tasks.sh"]
#NB - Ceres must run before Camodocal - should run ceres first in camodocal script
#Better to isolate each script from it's dependencies?
RUN ["/bin/bash","-c","cd ~/robotics_setup && ~/robotics_setup/ceres.sh"]
RUN ["/bin/bash","-c","cd ~/robotics_setup && ~/robotics_setup/camodocal.sh"]
RUN ["/bin/bash","-c","cd ~/robotics_setup && ~/robotics_setup/trtk.sh"]

#Should be separate nonopb script
RUN curl -sSL https://koti.kapsi.fi/~jpa/nanopb/download/nanopb-0.3.7-linux-x86.tar.gz | tar -xvz \
    && cd nanopb-0.3.7-linux-x86 \
    && cmake . \
    && make && make install

RUN ["/bin/bash","-c","cd ~/robotics_setup && ~/robotics_setup/grl_kuka.sh"]

