====================
Data Analysis 
====================

.. note:: I will interpret the data files, including the relative location and the main content.

How to collect data
==================================

In this project, the messages are communicated by `FlatBuffers <https://google.github.io/flatbuffers/>`__  between PC and Kuka workstation, which makes reading and writing data efficient. 


1. Install `VREP <http://coppeliarobotics.com/>`__ and `grl <https://github.com/ahundt/robotics_setup>`__.

2. Start VREP and load the scene `RoboneSimulation_private.ttt <https://github.com/ahundt/robonetracker/blob/master/modules/roboneprivate/data/RoboneSimulation_private.ttt>`__.

3. Set flatbuffer limit in `robone.lua <https://github.com/ahundt/robonetracker/blob/master/modules/grl/src/lua/robone.lua>`__. 
   The hard limit for flatbuffer is 2 GB, but you can customize it based on your requirement in this project. 
   When flatbuffer size hits this limit, the data will be written to disk. Or when you click on the STOP button in VREP, the data will also be written to disk automaticly, regardless of the limit.
   .. code-block:: bash
   KUKA_single_buffer_limit_bytes = 256    -- MB
   FB_single_buffer_limit_bytes = 1024     -- MB

4. Start to record data while simulation is running in `robone.lua <https://github.com/ahundt/robonetracker/blob/master/modules/grl/src/lua/robone.lua>`__.
   By defaut, program starts to collect data automatically. You can change it by following functions::
   .. code-block:: bash
      simExtKukaLBRiiwaRecordWhileSimulationIsRunning(true, KUKA_single_buffer_limit_bytes)
	  simExtAtracsysFusionTrackRecordWhileSimulationIsRunning(true, FB_single_buffer_limit_bytes)

5. Enable lua scripts StartRealArmDriverScript and Tracker in VREP, then program can communicate with Atracsys and Kuka.
   Only after connecting the devices, it can start to record data correctly.

6. Stop recording data.
   There are two ways to stop the data collection process, one is the buffer hits the limit in Step 3, the other is to click on the STOP button in VREP, then it will stop and write the data to disk automatically.





The location of the `fbs file <https://github.com/ahundt/robonetracker/tree/master/modules/grl/include/grl/flatbuffer/>`__.
fbs files define the binary format we use for high performance data collection from devices, such as the Kuka LBR iiwa 14kg and Atracsys FusionTrack.

The binary files are put in VREP data folder (i.e. ~/src/V-REP_PRO_EDU_V3_4_0_Linux/data/), 
and are named by the time stamp (i.e. 2018_03_26_19_06_21_FusionTrack.flik, 2018_03_26_19_06_21_Kukaiiwa.iiwa).

The command to generate json file, the binary file and the fbs file should be put in the same folder:
flatc -I . --json LogKUKAiiwaFusionTrack.fbs -- 2018_03_26_19_06_21_FusionTrack.flik
flatc -I . --json KUKAiiwa.fbs -- 2018_03_26_19_06_21_Kukaiiwa.iiwa

CSV file
==================================

When exporting the flatbuffer file to CSV, you need to follow the instructions below:

1. Keep the binary files in the VREP data folder (i.e. ~/src/V-REP_PRO_EDU_V3_4_0_Linux/data/);

2. Run `readFlatbufferTest <https://github.com/ahundt/robonetracker/tree/master/modules/grl/test>`__ with the arguments of the name of binary file.
   .. code-block:: bash
        ./readFlatbufferTest 2018_03_26_19_06_21_Kukaiiwa.iiwa 2018_03_26_19_06_21_FusionTrack.flik
	   # You can also pass the arguments manually to the main() function in readFlatbufferTest.cpp.
3. The generated csv files will be put into the VREP data folder  in which the subfolder is named by time stamp.
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


Replay Process
==================================
At this moment, the replay process is set and called in InverseKinematicsVrepPlug.cpp.
In future, It`s better to creat a replay plugin to handl this process.

Before run it, you should copy the KUKA_Measured_Joint.csv and FT_Pose_Marker22.csv to the  ~/src/V-REP_PRO_EDU_V3_4_0_Linux/data/data_in/.
The result will be writen in ForwardKinematics_Pose.csv.
