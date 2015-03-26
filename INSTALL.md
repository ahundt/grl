Introduction
============

This document contains the build and installation instructions for Robone.

For general build and installation instructions which apply to any software
developed on top of the [CMake Build system And Software Implementation
Standard (BASIS)][1], please refer to the respective [BASIS Installation Guide][2]
which is part of the CMake BASIS documentation.



Binary Distribution Package
===========================

Please see the corresponding section of the [BASIS Installation Guide][3].



Runtime Requirements
====================

This software has no runtime dependencies.



Building the Software
=====================

Build Dependencies
------------------

The following software has to be installed (if not optional).

Package                    | Version    | Language |     Description
-------------------------- | ---------- | -------- - ----------------------------------------------
Sunrise Connectivity Suite | >= 3.0     | Java     | Provided by KUKA, Eclipse based Java platform
-------------------------- | ---------- | -------- - ----------------------------------------------
Fast Robot Interface SDK   | >= 3.0     | C++      | Provided by KUKA, drivers for fast robot control
-------------------------- | ---------- | -------- - ----------------------------------------------
[CMake BASIS][1]           | >= 3.0     | C++      | Meta-project which makes it easy to create sharable software and libraries that work together.
-------------------------- | ---------- | -------- - ----------------------------------------------
[Boost][4]                 | >= 1.54.0  | C++      | Collection of general use C++ libraries.
-------------------------- | ---------- | -------- - ----------------------------------------------
[ZeroMQ][6]                | >= 4.0     | C++      | High Performance Network Communication Library
-------------------------- | ---------- | -------- - ----------------------------------------------
[AZMQ][7]                  | >= 1.0     | C++      | [Boost.asio][5] based official ZeroMQ C++ Networking Library Interface
-------------------------- | ---------- | -------- - ----------------------------------------------
[JeroMQ][8]                | >= 3.0     | Java     | Official Pure Java implementation of ZeroMQ
-------------------------- | ---------- | -------- - ----------------------------------------------
[Google Flatbuffers][9]    | >= 1.0     | Java/C++ | High Performance Message Serialization Library
-------------------------- | ---------- | -------- - ----------------------------------------------
[V-REP][10]                | >= 3.2     | C++      | Recommended robot simulation library (Optional)



Build Steps
-----------

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

    - Press 'c' to configure the build system and 'e' to ignore warnings.
    - Set CMAKE_INSTALL_PREFIX and other CMake variables and options.
    - Continue pressing 'c' until the option 'g' is available.
    - Then press 'g' to generate the configuration files for GNU Make.

    $ make
    $ make test    (optional)
    $ make install (optional)

An exhaustive list of minimum build dependencies, including the build tools
along detailed step-by-step build, test, and installation instructions can
be found in the corresponding "Building the Software from Sources" section
of the [BASIS how-to guide on software installation][2].

Please refer to this guide first if you are uncertain about above steps or
have problems to build, test, or install the software on your system.
If this guide does not help you resolve the issue, please contact <provider-name> <Robone@<vendor>.com>.
In case of failing tests, please attach the output of the following command:

    $ ctest -V >& test.log

In the following, only package-specific CMake settings available to
configure the build and installation of this software are documented.


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