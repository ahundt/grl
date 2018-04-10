==========
Data Setup
==========

.. note:: Here I will interpret the data file, including the relative location and the main content.

Flatbuffer fbs file
==================================

First install `VREP <http://coppeliarobotics.com/>`__ and grl.

The location of the 'fbs file <https://github.com/ahundt/robonetracker/tree/master/modules/grl/include/grl/flatbuffer/>'__.
They define the data we collect from Kuka and Atracsys.

The binary files are put in VREP folder (i.e. ~/src/V-REP_PRO_EDU_V3_4_0_Linux), and are named by the time stamp (i.e. 2018_03_26_19_06_21_FusionTrack.flik, 2018_03_26_19_06_21_Kukaiiwa.iiwa).

The command to generate json file, the binary file and the fbs file should be put in the same folder:
flatc -I . --json LogKUKAiiwaFusionTrack.fbs -- 2018_03_26_19_06_21_FusionTrack.flik
flatc -I . --json KUKAiiwa.fbs -- 2018_03_26_19_06_21_Kukaiiwa.iiwa

CSV file
==================================

When Parsing the flatbuffer file to CSV, you need to follow the instructions below:

1. Copy the binary files to the location of the actual 'readFlatbufferTest <https://github.com/ahundt/robonetracker/tree/master/modules/grl/test>'__executable;

2. Run 'readFlatbufferTest <https://github.com/ahundt/robonetracker/tree/master/modules/grl/test>'__ with the arguments of the name of binary file.
   .. code-block:: bash
        ./readFlatbufferTest 2018_03_26_19_06_21_Kukaiiwa.iiwa 2018_03_26_19_06_21_FusionTrack.flik
	   # You can also pass the arguments manually to the main() function in readFlatbufferTest.cpp.
3. The generated csv files will be put into a folder in the name of time stamp.
    Label explanation:
	.. code-block:: bash
        local_request_time_offset: PC time of sending a requesting command to devices;
	    local_receive_time_offset: PC time of receiving measured data from devices;
	    device_time_offset: the time from device;
	    time_Y: time driftting, device_time_offset - local_request_time_offset;
	    counter: the identifier of message, defined by devices;
	    X	Y	Z	A	B	C: the cartesian postion and oritation in Plucker coordinate system;
	    M_Pos_Ji: measured joint position of joint i from kuka;
	    C_Pos_J*: command joint position of joint i to kuka;
	    M_Tor_J*: measured joint torque of joint i form kuka;
	    C_Tor_J*: comand joint torque of joint i to kuka;
	    E_Tor_J*: external torque of joint i exerted on kuka;

		# on the first message save local_request_time as the initial_local_request_time. Both kuka and Atracsys share the same initial_local_request_time, which means they have the same time axis.
        # on the first message save device_time as the initial_device_time
        X = local_request_time
        local_request_offset = (local_request_time - initial_local_request_time)
        device_offset = (device_time - initial_device_time)
        time_Y = device_offset - local_request_offset
	CSV file explanation:
	   FTKUKA_TimeEvent.csv has the information from both Atracsys and kuka. It can help analysis time event.
	   FT_Pose_Marker*.csv have the time event and pose (in Plucker coordinate) of the specific marker in Atracsys space;
	   FT_TimeEvent.csv gives more detail information about time event from Atracsys, such as the time step;
	   KUKA_FRIMessage.csv includes all the FRI message from robot;
	   KUKA_Command_Joint.csv has the commanding joint angle sent to robot, which should use local_request_time_offset as time axis when plotting;
	   KUKA_Measured_Joint.csv has the measured joint angles received from robt, which should use local_receive_time_offset as time axis when plotting. 
	   KUKA_TimeEvent.csv gives more detail information about time event from kuka, such as the time step;
	 
	   All the CSV files above are generated from binary files. To make it convenient, all the files have time event information, which can be used as X-axis when plotting. 

	   ForwardKinematics_Pose.csv is created by the replay process(at this moment, it's in the InverseKinematicsVrepPlug.cpp. 
	   In future, It's better to creat a replay plugin to handl this process). It will read the measured joint angles from KUKA_Measured_Joint.csv, excute forward kinematics based on vrep model,
	   then write the cartesian pose (in Atracsys space) into ForwardKinematics_Pose.csv, which should be in ~/src/V-REP_PRO_EDU_V3_4_0_Linux/.
