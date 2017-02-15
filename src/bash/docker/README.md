##For basic Ubuntu 16.04 build:

InstallDocker.sh - InstallsDocker on a base system without it
*Build.sh* - Builds the docker image using the Dockerfile
*Run.sh* - runs a container using the built image with access to X11 for V-REP.

The run.sh file could be more advanced as it starts a new container everytime it is run and doesn't restart and attach to an existing container. Knowledge of docker is an advantage here. If you wish to restart and attach to an existing container you also need to set xhost +local:root.

'xhost +local:root && docker start 'name of container' && docker attach 'name of container''

An alternative is to commit the container with modifications to the base image:

'docker commit 'name of container' 'name of image e.g. xenial-grl:nvidia''

then simply use the ./run.sh script

##ROS-kinetic base image from OSRF (See dockerfile for details):

*pre-conf.sh* is used to create and build the catkin workspaces
*startup.sh* is run everytime the system is run. 

Both these files are copied into the docker image while it is being built and shouldn't be run by the user. 
