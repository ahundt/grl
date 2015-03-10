
##################################################
# Ubuntu Instructions to setup robone with Docker
##################################################


curl -sSL https://get.docker.com/ubuntu/ | sudo sh


# source: https://www.docker.com/

####################
# give docker permission to run
####################

sudo gpasswd -a ${USER} docker

# source : http://dev-maziarz.blogspot.com/2015/01/running-docker-sock-permission-denied.html


####################
# log out then log in
####################

###########################################
# clone the docker setup script I modified
##########################################

git clone https://github.com/ahundt/install-clang.git

# source : https://registry.hub.docker.com/u/rsmmr/clang/

####################
# setup c++ environment in a protected box so it doesn't mess with your OS
####################


# optional step that may speed things up:
#   docker pull rsmmr/clang

cd install-clang


####################
# coplete setup c++ environment in a protected box so it doesn't mess with your OS
####################
make docker-build

####################
# run c++ environment as if it was a separate OS.
####################

make docker-run


# build files should be in /root

#########################################
# how to cleanup if something goes wrong:
#########################################
# http://jimhoskins.com/2013/07/27/remove-untagged-docker-images.html
# http://docs.docker.com/reference/commandline/cli/
# bash script: https://github.com/blueyed/dotfiles/blob/master/usr/bin/docker-cleanup
# github issue requesing cleanup: https://github.com/docker/docker/issues/928











###############################################################
alternative, ignore below here
###############################################################


# install docker
curl -sSL https://get.docker.com/ubuntu/ | sudo sh

####################
# give docker permission to run
####################

sudo gpasswd -a ${USER} docker

# source : http://dev-maziarz.blogspot.com/2015/01/running-docker-sock-permission-denied.html


####################
# log out then log in
####################


####################
# setup c++ environment in a protected box so it doesn't mess with your OS
####################

docker pull sango/cpp-clang

# source : https://registry.hub.docker.com/u/sango/cpp-clang/


# update
docker run --user="root" sango/cpp-clang apt-get update


docker run --user="root" sango/cpp-clang apt-get install -y cmake libtool autoconf automake uuid-dev build-essential libboost-all-dev libeigen3-dev wget curl


####################
# zmq
####################

# source : https://maddigitiser.wordpress.com/2013/05/02/installing-zeromq-on-ubuntu-13-04/

########
# robone
########

docker run --user="root" sango/cpp-clang /bin/sh -c 'git clone https://github.com/zeromq/azmq.git /root/azmq/ && cd azmq && mkdir build && cd build && cmake .. && make && make test && make install"
docker run  --user="root" sango/cpp-clang git clone https://github.com/ahundt/robone.git  /root/robone
