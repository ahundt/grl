// Copyright 2006-2014 Coppelia Robotics GmbH. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// -------------------------------------------------------------------
// THIS FILE IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
// AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
// DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
// MISUSING THIS SOFTWARE.
// 
// You are free to use/modify/distribute this file for whatever purpose!
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.2.0 on Feb. 3rd 2015

#include <memory>
#include "robone/KukaVrepInterface.hpp"

#include "v_repExtRobonePlugin.h"
#include "v_repLib.h"
#include <iostream>

#ifdef _WIN32
	#include <shlwapi.h>
	#pragma comment(lib, "Shlwapi.lib")
#endif /* _WIN32 */

#if defined (__linux) || defined (__APPLE__)
	#include <unistd.h>
	#include <string.h>
	#define _stricmp(x,y) strcasecmp(x,y)
#endif

#define PLUGIN_VERSION 1

LIBRARY vrepLib; // the V-REP library that we will dynamically load and bind
std::unique_ptr<KukaVrepInterface> kukaVrepInterfacePG;


#define CONCAT(x,y,z) x y z
#define strConCat(x,y,z)	CONCAT(x,y,z)
#define LUA_GET_SENSOR_DATA_COMMAND "simExtSkeleton_getSensorData"


void LUA_GET_REAL_KUKA_STATE_CALLBACK(SLuaCallBack* p)
{ // the callback function of the new Lua command ("simExtSkeleton_getSensorData")
  // return Lua Table or arrays containing position, torque, torque minus motor force, timestamp, FRI state
}

void LUA_SET_SIM_KUKA_IDENTIFIERS_CALLBACK(SLuaCallBack* p)
{
	// Here we need the ineger "Handle" identifiers for the arm itself, and for each joint
}


void LUA_GET_SENSOR_DATA_CALLBACK(SLuaCallBack* p)
{ // the callback function of the new Lua command ("simExtSkeleton_getSensorData")

	// Check the function v_repStart to see how this callback is registered

	bool commandWasSuccessful=false;

	if (p->inputArgCount>=3)
	{ // Ok, we have at least 2 input argument. We are expecting an integer, a table with at least 3 floats, and a table with at least 2 ints:
		if ( (p->inputArgTypeAndSize[0*2+0]==sim_lua_arg_int) && // the integer value
			(p->inputArgTypeAndSize[1*2+0]==(sim_lua_arg_float|sim_lua_arg_table))&&(p->inputArgTypeAndSize[1*2+1]>=3) && // the table value with at least 3 floats
			(p->inputArgTypeAndSize[2*2+0]==(sim_lua_arg_int|sim_lua_arg_table))&&(p->inputArgTypeAndSize[2*2+1]>=2) ) // the table value with at least 2 ints
		{ // Ok, we have all required arguments
			int intArgInd=0;
			int floatArgInd=0;

			// the first argument:
			int sensorIndex=p->inputInt[intArgInd++]; 

			// the second argument:
			float floatParams[3];
			floatParams[0]=p->inputFloat[floatArgInd++];
			floatParams[1]=p->inputFloat[floatArgInd++];
			floatParams[2]=p->inputFloat[floatArgInd++];

			// the third argument:
			int intParams[2];
			intParams[0]=p->inputInt[intArgInd++];
			intParams[1]=p->inputInt[intArgInd++];

			// Now do something with above's arguments!!
			commandWasSuccessful=true;
		}
		else
			simSetLastError(LUA_GET_SENSOR_DATA_COMMAND,"Wrong argument type/size."); // output an error
	}
	else
		simSetLastError(LUA_GET_SENSOR_DATA_COMMAND,"Not enough arguments."); // output an error


	// Now prepare the return values:
	if (!commandWasSuccessful)
		p->outputArgCount=0; // Command failed, we do not return anything!
	else
	{ // Command succeeded, we return 3 values: result (integer), data (float table of size 10), distance (float)
		int result=1; // an integer
		float data[10]; // a float table
		for (int i=0;i<10;i++)
			data[i]=float(i);
		float distance=0.0; // a float

		p->outputArgCount=3; // 3 return values
		p->outputArgTypeAndSize=(simInt*)simCreateBuffer(p->outputArgCount*2*sizeof(simInt)); // x return values takes x*2 simInt for the type and size buffer

		p->outputArgTypeAndSize[0*2+0]=sim_lua_arg_int;	// The first return value is an int
		p->outputArgTypeAndSize[0*2+1]=1;				// Not used (table size if the return value was a table)

		p->outputArgTypeAndSize[1*2+0]=sim_lua_arg_float|sim_lua_arg_table;	// The second return value is a float table
		p->outputArgTypeAndSize[1*2+1]=10;				// The table size is 10

		p->outputArgTypeAndSize[2*2+0]=sim_lua_arg_float;	// The third return value is a float
		p->outputArgTypeAndSize[2*2+1]=1;				// Not used (table size if the return value was a table)

		// Now create the int buffer and populate it:
		p->outputInt=(simInt*)simCreateBuffer(1*sizeof(int)); // We have a total of 1 int return value
		p->outputInt[0]=result; // This is the int value we want to return

		// Now create the float buffer and populate it:
		p->outputFloat=(simFloat*)simCreateBuffer(11*sizeof(float)); // We have a total of 11 float return values
		int floatInd=0;
		for (int i=0;i<10;i++)
			p->outputFloat[floatInd++]=data[i]; 
		p->outputFloat[floatInd++]=distance;
	}
}


// This is the plugin start routine (called just once, just after the plugin was loaded):
VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer,int reservedInt)
{
	// Dynamically load and bind V-REP functions:
	// ******************************************
	// 1. Figure out this plugin's directory:
	char curDirAndFile[1024];
#ifdef _WIN32
	GetModuleFileName(NULL,curDirAndFile,1023);
	PathRemoveFileSpec(curDirAndFile);
#elif defined (__linux) || defined (__APPLE__)
	getcwd(curDirAndFile, sizeof(curDirAndFile));
#endif
	std::string currentDirAndPath(curDirAndFile);
	// 2. Append the V-REP library's name:
	std::string temp(currentDirAndPath);
#ifdef _WIN32
	temp+="\\v_rep.dll";
#elif defined (__linux)
	temp+="/libv_rep.so";
#elif defined (__APPLE__)
	temp+="/libv_rep.dylib";
#endif /* __linux || __APPLE__ */
	// 3. Load the V-REP library:
	vrepLib=loadVrepLibrary(temp.c_str());
	if (vrepLib==NULL)
	{
		std::cout << "Error, could not find or correctly load the V-REP library. Cannot start 'PluginSkeleton' plugin.\n";
		return(0); // Means error, V-REP will unload this plugin
	}
	if (getVrepProcAddresses(vrepLib)==0)
	{
		std::cout << "Error, could not find all required functions in the V-REP library. Cannot start 'PluginSkeleton' plugin.\n";
		unloadVrepLibrary(vrepLib);
		return(0); // Means error, V-REP will unload this plugin
	}
	// ******************************************

	// Check the version of V-REP:
	// ******************************************
	int vrepVer;
	simGetIntegerParameter(sim_intparam_program_version,&vrepVer);
	if (vrepVer<20604) // if V-REP version is smaller than 2.06.04
	{
		std::cout << "Sorry, your V-REP copy is somewhat old. Cannot start 'PluginSkeleton' plugin.\n";
		unloadVrepLibrary(vrepLib);
		return(0); // Means error, V-REP will unload this plugin
	}
	// ******************************************


	// Register the new Lua command "simExtSkeleton_getSensorData":
	// ******************************************
	// Expected input arguments are: int sensorIndex, float floatParameters[3], int intParameters[2]
	int inArgs_getSensorData[]={3,sim_lua_arg_int,sim_lua_arg_float|sim_lua_arg_table,sim_lua_arg_int|sim_lua_arg_table}; // this says we expect 3 arguments (1 integer, a table of floats, and a table of ints)
	// Return value can change on the fly, so no need to specify them here, except for the calltip.
	// Now register the callback:
	simRegisterCustomLuaFunction(LUA_GET_SENSOR_DATA_COMMAND,strConCat("number result,table data,number distance=",LUA_GET_SENSOR_DATA_COMMAND,"(number sensorIndex,table_3 floatParams,table_2 intParams)"),inArgs_getSensorData,LUA_GET_SENSOR_DATA_CALLBACK);
	// ******************************************

	return(PLUGIN_VERSION); // initialization went fine, we return the version number of this plugin (can be queried with simGetModuleName)
}

// This is the plugin end routine (called just once, when V-REP is ending, i.e. releasing this plugin):
VREP_DLLEXPORT void v_repEnd()
{
	// Here you could handle various clean-up tasks

		/////////////////////////
		// PUT OBJECT RESET CODE HERE
		// close out as necessary
		////////////////////
	
	unloadVrepLibrary(vrepLib); // release the library
}

// This is the plugin messaging routine (i.e. V-REP calls this function very often, with various messages):
VREP_DLLEXPORT void* v_repMessage(int message,int* auxiliaryData,void* customData,int* replyData)
{ // This is called quite often. Just watch out for messages/events you want to handle
	// Keep following 5 lines at the beginning and unchanged:
	static bool refreshDlgFlag=true;
	int errorModeSaved;
	simGetIntegerParameter(sim_intparam_error_report_mode,&errorModeSaved);
	simSetIntegerParameter(sim_intparam_error_report_mode,sim_api_errormessage_ignore);
	void* retVal=NULL;

	// Here we can intercept many messages from V-REP (actually callbacks). Only the most important messages are listed here.
	// For a complete list of messages that you can intercept/react with, search for "sim_message_eventcallback"-type constants
	// in the V-REP user manual.

	if (message==sim_message_eventcallback_refreshdialogs)
		refreshDlgFlag=true; // V-REP dialogs were refreshed. Maybe a good idea to refresh this plugin's dialog too

	if (message==sim_message_eventcallback_menuitemselected)
	{ // A custom menu bar entry was selected..
		// here you could make a plugin's main dialog visible/invisible
	}

	if (message==sim_message_eventcallback_instancepass)
	{	// This message is sent each time the scene was rendered (well, shortly after) (very often)
		// It is important to always correctly react to events in V-REP. This message is the most convenient way to do so:

		int flags=auxiliaryData[0];
		bool sceneContentChanged=((flags&(1+2+4+8+16+32+64+256))!=0); // object erased, created, model or scene loaded, und/redo called, instance switched, or object scaled since last sim_message_eventcallback_instancepass message 
		bool instanceSwitched=((flags&64)!=0);

		if (instanceSwitched)
		{
			// React to an instance switch here!!
		}

		if (sceneContentChanged)
		{ // we actualize plugin objects for changes in the scene

			//...
			//////////////
			// PUT MAIN CODE HERE
			
			/////////////
			if (simGetSimulationState() != sim_simulation_advancing_abouttostop)	//checks if the simulation is still running
			{	
				std::cout << simGetSimulationTime() << std::endl;					// gets simulation time point
			}
			// make sure it is "right" (what does that mean?)
			

			// find the v-rep C functions to do the following:
			////////////////////////////////////////////////////
			// Use handles that were found at the "start" of this simulation running

			// next few Lines get the joint angles, torque, etc from the simulation

			for (i=0 ; i<=6 ; i++)
			{
				float* simJointPosition[7];
				simGetJointPosition(jointHandle[i],&simJointPosition[i]);  //retrieves the intrinsic position of a joint (Angle for revolute joint)
			}

			for (i=0 ; i<=6 ; i++)
			{
				float* simJointForce[7];
				simGetJointForce(jointHandle[i],&simJointForce[i]);	//retrieves the force or torque applied to a joint along/about its active axis. This function retrieves meaningful information only if the joint is prismatic or revolute, and is dynamically enabled. 
			}

			for (i=0 ; i<=6 ; i++)
			{
				float* simJointTargetPosition[7];
				simGetJointTargetPosition(jointHandle[i],&simJointTargetPosition[i]);  //retrieves the target position of a joint
			}

			for (i=0 ; i<=6 ; i++)
			{
				float* simJointTrasformationMatrix[7];	
				simGetJointMatrix(jointHandle[i],&simJointTransformationMatrix[i]);   //retrieves the intrinsic transformation matrix of a joint (the transformation caused by the joint movement)
			}

			// Send updated position to the real arm based on simulation
			
            
			// Step 1
			////////////////////////////////////////////////////
			// call the functions here and just print joint angles out
			// or display something on the screens

			std::cout << *jointAngle << std::endl << *jointForce << std::endl << *jointTargetPosition << std::endl << *jointTransformationMatrix << std::endl ;
			

			///////////////////
			// call our object to get the latest real kuka state
			// then use the functions below to set the simulation state
			// to match

			/////////////////// assuming given real joint position (angles), forces, target position and target velocity 

			float realJointPosition[7] = { 0, 0, 0, 0, 0, 0, 0 };
			float realJointTargetPosition[7] = { 0, 0, 0, 0, 0, 0, 0 };
			float realJointForce[7] = { 0, 0, 0, 0, 0, 0, 0 };
			float realJointTargetVelocity[7] = { 0, 0, 0, 0, 0, 0, 0 };

			// setting the simulation variables to data from real robot (here they have been assumed given)

			for (i=0 ; i <=6) ; i++)
			{
				simSetJointPosition(jointHandle[i],realJointPosition[i]); //Sets the intrinsic position of a joint. May have no effect depending on the joint mode
			}
			
			for (i=0 ; i <=6) ; i++)
			{
				simSetJointTargetPosition(jointHandle[i],realJointTargetPosition[i]);  //Sets the target position of a joint if the joint is in torque/force mode (also make sure that the joint's motor and position control are enabled
			}

			for (i=0 ; i <=6) ; i++)
			{
				simSetJointForce(jointHandle[i],realJointForce[i]);  //Sets the maximum force or torque that a joint can exert. This function has no effect when the joint is not dynamically enabled
			}
			
			for (i=0 ; i <=6) ; i++)
			{
				simSetJointTargetVelocity(jointHandle[i],realJointTargetVelocity[i]);  //Sets the intrinsic target velocity of a non-spherical joint. This command makes only sense when the joint mode is: (a) motion mode: the joint's motion handling feature must be enabled (simHandleJoint must be called (is called by default in the main script), and the joint motion properties must be set in the joint settings dialog), (b) torque/force mode: the dynamics functionality and the joint motor have to be enabled (position control should however be disabled)
			}
			  
			//simSetJointInterval(simInt objectHandle,simBool cyclic,const simFloat* interval); //Sets the interval parameters of a joint (i.e. range values)
			//simSetJointMode(simInt jointHandle,simInt jointMode,simInt options); //Sets the operation mode of a joint. Might have as side-effect the change of additional properties of the joint

			refreshDlgFlag=true; // always a good idea to trigger a refresh of this plugin's dialog here
		}
	}

	if (message==sim_message_eventcallback_mainscriptabouttobecalled)
	{ // The main script is about to be run (only called while a simulation is running (and not paused!))
		
	}

	if (message==sim_message_eventcallback_simulationabouttostart)
	{ // Simulation is about to start

			jointHandle = {-1,-1,-1,-1,-1,-1,-1};

			jointHandle[0] = simGetObjectHandle("LBR_iiwa_14_R820_joint1");	//Obtain Joint Handles
			jointHandle[1] = simGetObjectHandle("LBR_iiwa_14_R820_joint2");
			jointHandle[2] = simGetObjectHandle("LBR_iiwa_14_R820_joint3");
			jointHandle[3] = simGetObjectHandle("LBR_iiwa_14_R820_joint4");
			jointHandle[4] = simGetObjectHandle("LBR_iiwa_14_R820_joint5");
			jointHandle[5] = simGetObjectHandle("LBR_iiwa_14_R820_joint6");
			jointHandle[6] = simGetObjectHandle("LBR_iiwa_14_R820_joint7");

			robotTip = simGetObjectHandle("RobotTip#0");					//Obtain RobotTip handle
			robotTarget = simGetObjectHandle("RobotTarget#0");
			implantCutPath = simGetObjectHandle("ImplantCutPath");
			removeBallJoit = simGetObjectHandle("RemoveBallJoint");
			bone = simGetObjectHandle("FemurBone");



		/////////////////////////
		// PUT OBJECT STARTUP CODE HERE
		////////////////////
		// get the handles to all the objects, joints, etc that we need
		/////////////////////
		// simGetObjectHandle
	}

	if (message==sim_message_eventcallback_simulationended)
	{ // Simulation just ended

		/////////////////////////
		// PUT OBJECT RESET CODE HERE
		// close out as necessary
		////////////////////
		

	}

	if (message==sim_message_eventcallback_moduleopen)
	{ // A script called simOpenModule (by default the main script). Is only called during simulation.
		if ( (customData==NULL)||(_stricmp("PluginSkeleton",(char*)customData)==0) ) // is the command also meant for this plugin?
		{
			// we arrive here only at the beginning of a simulation
		}
	}

	if (message==sim_message_eventcallback_modulehandle)
	{ // A script called simHandleModule (by default the main script). Is only called during simulation.
		if ( (customData==NULL)||(_stricmp("PluginSkeleton",(char*)customData)==0) ) // is the command also meant for this plugin?
		{
			// we arrive here only while a simulation is running
		}
	}

	if (message==sim_message_eventcallback_moduleclose)
	{ // A script called simCloseModule (by default the main script). Is only called during simulation.
		if ( (customData==NULL)||(_stricmp("PluginSkeleton",(char*)customData)==0) ) // is the command also meant for this plugin?
		{
			// we arrive here only at the end of a simulation
		}
	}

	if (message==sim_message_eventcallback_instanceswitch)
	{ // Here the user switched the scene. React to this message in a similar way as you would react to a full
	  // scene content change. In this plugin example, we react to an instance switch by reacting to the
	  // sim_message_eventcallback_instancepass message and checking the bit 6 (64) of the auxiliaryData[0]
	  // (see here above)

	}

	if (message==sim_message_eventcallback_broadcast)
	{ // Here we have a plugin that is broadcasting data (the broadcaster will also receive this data!)

	}

	if (message==sim_message_eventcallback_scenesave)
	{ // The scene is about to be saved. If required do some processing here (e.g. add custom scene data to be serialized with the scene)

	}

	// You can add many more messages to handle here

	if ((message==sim_message_eventcallback_guipass)&&refreshDlgFlag)
	{ // handle refresh of the plugin's dialogs
		// ...
		refreshDlgFlag=false;
	}

	// Keep following unchanged:
	simSetIntegerParameter(sim_intparam_error_report_mode,errorModeSaved); // restore previous settings
	return(retVal);
}

