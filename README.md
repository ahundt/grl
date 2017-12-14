Generic Robotics Library
=========================

[![Build Status](https://travis-ci.org/ahundt/grl.svg?branch=master)](https://travis-ci.org/ahundt/grl)

The Generic Robotics Library (GRL) has a long term goal of implementing robotics algorithms using generic programming in C++11.

Currently GRL implements C++11 drivers for the new Kuka LBR iiwa arm and hardware integration with [ROS](ros.org) and the [V-REP](http://http://www.coppeliarobotics.com/index.html) robotics simulation software.


[![grl kuka control from linux over Java API demo](https://img.youtube.com/vi/pvs-lG2_K3g/0.jpg)](https://youtu.be/pvs-lG2_K3g)

If you use GRL in research please consider providing a citation:
[![DOI](https://zenodo.org/badge/33050653.svg)](https://zenodo.org/badge/latestdoi/33050653)



License
=======

Copyright (c) 2015-2016 Andrew Hundt

See COPYING file for license information.



Installation
============

See build and installation instructions given in the [INSTALL](/INSTALL.md) file.



Documentation
=============

[Documentation Website](https://ahundt.github.io/grl/index.html)

See the software manual for details on the software including a demonstration
of how to apply the software tools provided by this package.



Package Content
===============

Path                    | Content description
----------------------- | ----------------------------------------------------------
[BasisProject.cmake][1] | Meta-data used for the build configuration.
[CMakeLists.txt]    [2] | Root CMake configuration file.
[config/]           [3] | Package configuration files.
[data/]             [4] | Data files required by this software.
[doc/]              [5] | Documentation source files.
[example/]          [6] | Example files used for demonstration.
[include/]          [7] | Public header files.
[src/]              [8] | Source code files.
[test/]             [9] | Regression and unit tests.






<!-- --------------------------------------------------------------------------------- -->

<!-- Links to GitHub, see the local directory if you have downloaded the files already -->
[1]: /BasisProject.cmake
[2]: /CMakeLists.txt
[3]: /config
[4]: /data
[5]: /doc
[6]: /example
[7]: /include
[8]: /src
[9]: /test
