==========
VREP Setup
==========

.. note:: You will need to understand V-REP and grl thoroughly if you plan to modify this substantially. Work is also needed to generalize it to any robot and create more examples. If interested create a github issue to discuss, pull requests will be appreciated!

First install `VREP <http://coppeliarobotics.com/>`__ and grl.

Find the location of the actual vrep executable, for which there are example paths below

Create symlinks to the grl libraries that V-REP should load.

There is a `SymbolicLinksRoboneSimulation.sh <https://github.com/ahundt/robone/blob/master/data/SymbolicLinksRoboneSimulation.sh>`__ script to assist with this, which you can open and edit for your particular system.

.. code-block:: bash

	GDIR="/path/to/grl/"
	# Mac example directory:   VDIR="/Applications/V-REP_PRO_EDU_V3_3_2_Mac/vrep.app/Contents/MacOS/"
	# Linux example directory: VDIR="~/V-REP_PRO_EDU_V3_3_2_Linux/"
	#
	# cd into the appropriate directory, then create the following symlinks
	ln -s src/lua/grl.lua ${VDIR}/
	ln -s build/libv_repExtKukaLBRiiwa ${VDIR}/
	# ... continue for all libraries created for grl


An example simulation can be found in the `Robone/data <https://github.com/ahundt/robone/tree/master/data>`__ project folder.


Then simply open the project with V-REP and you can be on your way!

Please note that this project is currently set up in a fairly specific way,
so you will most 