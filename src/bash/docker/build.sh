#! /bin/bash
#
# build.sh
#
# Build this docker image.
#
sudo apt-get -y install nvidia-docker
nvidia-docker build -t xenial-grl:nvidia .
