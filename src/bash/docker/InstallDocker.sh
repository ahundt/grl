
##################################################
# Ubuntu Instructions to setup grl with Docker
##################################################

# Install docker with the line below

curl -sSL https://get.docker.com/ | sudo sh


# source: https://www.docker.com/

####################
# give docker permission to run
####################

sudo gpasswd -a ${USER} docker

# source : http://dev-maziarz.blogspot.com/2015/01/running-docker-sock-permission-denied.html

cd docker
#Edit Dockerfile if ROS is required
sh build.sh

