#-i allows interactive --env set env variables --volume allows container to access system X11 socket
#-t allows a pseudo TTY to enter commands
xhost +local:root
nvidia-docker run -it \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    xenial-grl:nvidia 
    #bash -c "roscore & rosrun rviz rviz"
xhost -local:root
