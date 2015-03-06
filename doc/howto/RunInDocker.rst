

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

docker pull rsmmr/clang

# source : https://registry.hub.docker.com/u/rsmmr/clang/


# update
docker run rsmmr/clang apt-get update

# boost
docker run rsmmr/clang apt-get install -y libboost-all-dev                       

# eigen                
docker run rsmmr/clang apt-get install -y libeigen3-dev

# useful command line utilities
docker run rsmmr/clang apt-get install -y wget curl

####################
# zmq
####################


docker run rsmmr/clang apt-get install -y libtool autoconf automake uuid-dev build-essential



# note: the directory ~ in this case is /root
docker run rsmmr/clang /bin/sh -c 'cd ~ && \
                       wget http://download.zeromq.org/zeromq-4.0.5.tar.gz && \
                       tar zxvf zeromq-4.0.5.tar.gz && cd zeromq-4.0.5 && \
                       ./configure && make && make install'

# source : https://maddigitiser.wordpress.com/2013/05/02/installing-zeromq-on-ubuntu-13-04/







###############################################################
alternative






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

docker run --user="root" sango/cpp-clang /bin/sh -c 'cd /home/sango && \
                       curl  --remote-name http://download.zeromq.org/zeromq-4.0.5.tar.gz && \
                       tar zxvf zeromq-4.0.5.tar.gz && cd zeromq-4.0.5 && \
                       ./configure && make && make install'

# source : https://maddigitiser.wordpress.com/2013/05/02/installing-zeromq-on-ubuntu-13-04/

#####
# robone
#####

docker run  --user="root" sango/cpp-clang git clone https://github.com/ahundt/robone.git  /home/sango/robone
