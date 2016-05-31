Introduction
============

This document contains the build and installation instructions for the Generic Robotics Library.


Easy Installation for OS X and Linux
====================================

The easy installation instructions work on OS X via homebrew and Ubuntu 14.04 Linux via Linuxbrew.

**ROS users:**

Install ROS before following these instructions.

**Linux users:**

Please note that on Linux these instructions will build many dependencies from source
and install them in `~/.linuxbrew`, so if you need to use specific versions that are
already installed these instructions may not work for you! ROS users, this may mean you!

Also on linux you may want to use `brew install libname --env=inherit`
so it uses your native environment variable configuration.

**Setup Instructions:**

1. Setup brew for OS X or Linux
    - [Homebrew OS X setup instructions](http://brew.sh/) or paste `ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"` into the terminal
    - [Linuxbrew setup instructions](http://linuxbrew.sh/), or paste `ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Linuxbrew/linuxbrew/go/install)"` into the terminal
2. Linux only - add linuxbrew to your `~/.bashrc` or ~/.zshrc:
    export PKG_CONFIG_PATH="/usr/bin/pkg-config:$HOME/.linuxbrew/bin/pkg-config"
    export PKG_CONFIG_LIBDIR="/usr/lib/pkgconfig:$HOME/.linuxbrew/lib/pkgconfig"
    export PATH="$HOME/.linuxbrew/bin:$PATH"                                    
    export MANPATH="$HOME/.linuxbrew/share/man:$MANPATH"                        
    export INFOPATH="$HOME/.linuxbrew/share/info:$INFOPATH"
3. Check that it is setup correctly `brew help` should output the homebrew help.
4. OS X only - install [Homebrew Cask](http://caskroom.io/) `brew install caskroom/cask/brew-cask`
5.

```
brew tap homebrew/science # [homebrew-science](http://brew.sh/homebrew-science/) contains many scientific libraries, such as OpenCV and pcl
brew tap homebrew/robotics # [homebrew-robotics](https://github.com/ahundt/homebrew-robotics) contains software packages needed by grl.
brew install grl
```

**Done!** (unless you need kuka drivers)


#### Extra steps for KUKA iiwa drivers

There are a few additional steps to get the KUKA iiwa drivers up and running. First run the following:

```
brew uninstall grl # we need to do some manual steps so remove the auto installed version (dependencies stay)
mkdir -p ~/local/src
cd ~/local/src
git clone https://github.com/ahundt/grl.git
cd grl
```

The grl code will now be in `~/local/src/grl`.

Now follow the instructions in:

[KUKA LBR iiwa Java Setup](https://ahundt.github.io/grl/howto/iiwaKukaRobotSetup.html)

**ROS Users:**

We recommend installing [iiwa_stack](https://github.com/SalvoVirga/iiwa_stack)
in addition to grl for full ROS integration.

Also take a look at [costar_stack](https://github.com/cpaxton/costar_stack)
which is integrated with grl and provides full blown system integration tools
for many typical robot arm tasks such as pick and place operations.




Runtime Requirements
====================

This software has an optional runtime dependency on [V-REP][10] and [ROS][22], independently or together.



Installing on Ubuntu 14.04 Manually
===================================

These instructions are for manually installing the ubuntu 14.04 package dependencies for grl.

```
sudo apt-get install libtool pkg-config build-essential autoconf automake
sudo apt-get install libboost-all-dev libeigen3-dev libopencv-dev
```

#### [ceres-solver](http://ceres-solver.org/building.html#linux) setup

# Boost
You need to install and compile the latest boost version (currently is 1.61)

```
# CMake
sudo apt-get install cmake
```
*** for using the default basis you need to have cmake 2.8 version.
Cmake >=3.0 causes issues with the cmake-basis-3.1.0 compilation.
You need to compile the newest version of [cmake-basis](https://cmake-basis.github.io/download.html) and add to the build directory.***

```
# google-glog + gflags
sudo apt-get install libgoogle-glog-dev
# BLAS & LAPACK
sudo apt-get install libatlas-base-dev
# Eigen3
sudo apt-get install libeigen3-dev

# SuiteSparse and CXSparse (needed for grl, optional for ceres)
# - However, if you want to build Ceres as a *shared* library (recommended for grl), you must
#   add the following PPA:
sudo add-apt-repository ppa:bzindovic/suitesparse-bugfix-1319687
sudo apt-get update
sudo apt-get install libsuitesparse-dev

# - If you want to build Ceres as a *static* library
#   you can use the SuiteSparse package in the main Ubuntu package
#   repository:
sudo apt-get install libsuitesparse-dev
```
clone, build and install ceres:

```
git clone https://ceres-solver.googlesource.com/ceres-solver
cd ceres-solver
mkdir build
cd build
cmake ..
sudo make install
cd ../..
```

#### [ZeroMQ](http://ceres-solver.org/building.html#linux) setup


```
sudo apt-get install libzmq3-dev
```

for 14.04 don't use libzmq-dev, it is too out of date

```
git clone https://github.com/zeromq/azmq.git
cd azmq
mkdir build
cd build
cmake ..
sudo make install
cd ../..
```

#### [flatbuffers](https://github.com/google/flatbuffers) setup


```
git clone https://github.com/google/flatbuffers.git
cd flatbuffers
mkdir build
cd build
cmake ..
sudo make install
cd ../..
```


optional:
```
sudo apt-get install libpcl-1.7-all-dev
````


#### [OpenCV](http://ceres-solver.org/building.html#linux) setup

```
sudo apt-get install libopencv-dev
sudo add-apt-repository --yes ppa:xqms/opencv-nonfree
sudo apt-get update
sudo apt-get install libopencv-nonfree-dev
```


#### [Camodocal](https://github.com/hengli/camodocal)

```
git clone https://ceres-solver.googlesource.com/ceres-solver
cd ceres-solver
mkdir build
cd build
cmake ..
sudo make install
cd ../..
```

#### [grl](https://github.com/ahundt/grl)



```
git clone https://github.com/ahundt/grl.git
cd grl
mkdir build
cd build
cmake ..
sudo make install
cd ../..
```

Build Dependencies
==================

Many dependencies are optional based on the functionality you wish to use.
Details about what dependencies are for what functionality are below this table.


Package                       | Version    | Language |     Description
----------------------------- | ---------- | -------- | ----------------------------------------------
Sunrise Connectivity Suite    | >= 3.0     | Java     | Provided by KUKA, Eclipse based Java platform
Fast Robot Interface SDK      | >= 3.0     | C++      | Provided by KUKA, drivers for fast robot control
[CMake BASIS][1]              | >= 3.2     | C++      | Meta-project which makes it easy to create sharable software and libraries that work together.
[Boost][4]                    | >= 1.61.0  | C++      | Collection of general use C++ libraries.
[Eigen][14]                   | >= 3.2     | C++      | C++ template library for linear algebra.
[ZeroMQ][6]                   | >= 4.0     | C++      | High Performance Network Communication Library
[libsodium][11]               | >= 1.0.4   | C++      | A modern and easy-to-use crypto library used by ZeroMQ.
[AZMQ][7]                     | >= 1.0     | C++      | [Boost.asio][5] based official ZeroMQ C++ Networking Library Interface
[JeroMQ][8]                   | >= 3.0     | Java     | Official Pure Java implementation of ZeroMQ
[Google Flatbuffers][9]       | >= 1.0     | Java/C++ | High Performance Message Serialization Library
[V-REP][10]                   | >= 3.2     | C++      | Recommended robot simulation library (Optional)
[CAMODOCAL][13]               | ?          | C++      | CamOdoCal: Automatic Intrinsic and Extrinsic Calibration
[ceres-solver][12]            | >= 1.11    | C++      | Nonlinear Optimization library developed by google
BLAS                          | ?          | ?        | Standard linear algebra APIs, see ceres-solver install instructions
[CUDA][15]                    | >= 7.5     | C++      | NVIDIA GPU C/C++ Development Environment used by CAMODOCAL
[gtest][16]                   | >= 1.7.0   | C++      | Google's C++ test framework used by ceres-solver
[glog][17]                    | >= 0.3.4   | C++      | Google's C++ logging framework used by ceres-solver
[cisst-netlib][18]            | >= 0.3.4   | C++      | Binary distribution of [netlib][21] numerical routines
[cisst][19]                   | >= 0.3.4   | C++      | Google's C++ logging framework used by ceres-solver
[sawConstraintController][20] | >= 1.0.2   | C++      | cisst component for constrained optimization and arm path planning

### KUKA Driver Dependencies

Standalone driver C++ dependencies:

```
    Threads            # pthreads, see CMake documentation
    ZeroMQ      # zeromq.org https://github.com/zeromq/libzmq
	AZMQ        # https://github.com/zeromq/azmq
	FlatBuffers # google flatbuffers https://github.com/google/flatbuffers
	Boost{program_options,filesystem,unit_test_framework,system,regex,coroutine,log,chrono}
    Nanopb             # Used in Kuka Fast Robot Interface (comes in FRI zip file)
```

**KUKA software**

You must follow the instructions at https://ahundt.github.io/grl/howto/iiwaKukaRobotSetup.html to correctly setup the software KUKA provides.

**Optional**

    ROS    # for ROS driver http://www.ros.org/
    V-REP  # for V-REP driver http://coppeliarobotics.com


### Hand Eye Calibration Dependencies

```
	Eigen3             # Linear Algebra eigen.tuxfamily.com
    CAMODOCAL          # used for hand eye calibration plugin https://github.com/hengli/camodocal
    Ceres              # http://ceres-solver.org/ used in arm hand eye calibration
```

### Pivot Calibration Dependencies

    TRTK               # Transform Registration Toolkit, for pivot calibration https://github.com/RWTHmediTEC/TRTK

### Arm Path Planning

Constrained Optimization based arm path planning dependencies:

```

    CisstNetlib        # https://github.com/jhu-cisst/cisstNetlib  used for inverse kinematics plugin
    cisst{cisstNumerical,cisstOSAbstraction,cisstVector,cisstCommon} # https://github.com/jhu-cisst/cisst used for inverse kinematics plugin
    sawConstraintController #  used for inverse kinematics plugin https://github.com/jhu-saw/sawConstraintController
```


### Vision

```
    PCL                # point cloud library, for vision components >=1.7 recommended https://github.com/PointCloudLibrary/pcl
    freenect2          # kinect v2 drivers, for vision components https://github.com/OpenKinect/libfreenect2
```


### UR5 Robot arm

Not yet fully implemented/tested.

```
    ur_modern_driver   # Universal Robots Arm Device Driver https://github.com/ThomasTimm/ur_modern_driver/
```

### Extra Notes on dependencies

Here are some extra notes if you're planning to install
dependencies manually.



libsodium (for zeromq)

```
git clone git@github.com:jedisct1/libsodium.git
cd libsodium/
sudo make install
```

  zeromq >=4.0

  [azmq](https://github.com/zeromq/azmq)

```
git clone git@github.com:zeromq/azmq.git
cd azmq
mkdir build
cd build
cmake ..
sudo make -j8 install
```

Required dependencies (outdated)

BLAS (Ubuntu package: libblas-dev)
Boost >= 1.4.0 (Ubuntu package: libboost-all-dev)
CUDA >= 4.2
Eigen3 (Ubuntu package: libeigen3-dev)
glog
OpenCV >= 2.4.6
SuiteSparse >= 4.2.1 # note CERES dependencies above
Optional dependencies

GTest
OpenMP




Standard cmake-basis build steps
--------------------------------

**you probably don't want to follow these steps**
These are generic instructions for libraries that use cmake-basis.
I suggest following the easy install instructions instead!

For general build and installation instructions which apply to any software
developed on top of the [CMake Build system And Software Implementation
Standard (BASIS)][1], please refer to the respective [BASIS Installation Guide][2]
which is part of the CMake BASIS documentation.

The common steps to build, test, and install software based on CMake,
including this software, are as follows:

1. Extract source files.
2. Create build directory and change to it.
3. Run CMake to configure the build tree.
4. Build the software using selected build tool.
5. Test the built software.
6. Install the built files.

On Unix-like systems with GNU Make as build tool, these build steps can be
summarized by the following sequence of commands executed in a shell,
where $package and $version are shell variables which represent the name
of this package and the obtained version of the software.

    $ tar xzf $package-$version-source.tar.gz
    $ mkdir $package-$version-build
    $ cd $package-$version-build
    $ ccmake -DBASIS_DIR:PATH=/path/to/basis ../$package-$version-source

    | Press 'c' to configure the build system and 'e' to ignore warnings.
    | Set CMAKE_INSTALL_PREFIX and other CMake variables and options.
    | Continue pressing 'c' until the option 'g' is available.
    | Then press 'g' to generate the configuration files for GNU Make.

    $ make
    $ make test    (optional)
    $ make install (optional)

An exhaustive list of minimum build dependencies, including the build tools
along detailed step-by-step build, test, and installation instructions can
be found in the corresponding "Building the Software from Sources" section
of the [BASIS how-to guide on software installation][2].

Please refer to this guide first if you are uncertain about above steps or
have problems to build, test, or install the software on your system.
If this guide does not help you resolve the issue, please contact Andrew Hundt <ATHundt@gmail.com>.
In case of failing tests, please attach the output of the following command:

    $ ctest -V >& test.log

In the following, only package-specific CMake settings available to
configure the build and installation of this software are documented.



Binary Distribution Package
===========================

Please see the corresponding section of the [BASIS Installation Guide][3].

CMake Options
-------------

(no additional CMake options considered by this package)


Advanced CMake Options
----------------------

(no additional advanced CMake options considered by this package)



<!-- REFERENCES -->
[1]:  http://opensource.andreasschuh.com/cmake-basis/
[2]:  http://opensource.andreasschuh.com/cmake-basis/howto/install.html
[3]:  http://opensource.andreasschuh.com/cmake-basis/howto/install.html#binary-distribution-package
[4]:  http://www.boost.org
[5]:  http://www.boost.org/libs/asio
[6]:  http://www.zeromq.org/
[7]:  https://github.com/zeromq/azmq
[8]:  https://github.com/zeromq/jeromq
[9]:  https://github.com/google/flatbuffers
[10]: http://www.coppeliarobotics.com/index.html
[11]: https://github.com/jedisct1/libsodium
[12]: http://ceres-solver.org/
[13]: https://github.com/hengli/camodocal
[14]: http://eigen.tuxfamily.org/
[15]: https://developer.nvidia.com/cuda-toolkit
[16]: https://github.com/google/googletest
[17]: https://github.com/google/glog
[18]: https://github.com/jhu-cisst/cisstNetlib
[19]: https://github.com/jhu-cisst/cisst
[20]: https://github.com/jhu-saw/sawConstraintController
[21]: http://www.netlib.org
[22]: http://www.ros.org
