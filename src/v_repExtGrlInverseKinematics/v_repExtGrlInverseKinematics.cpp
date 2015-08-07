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
#include <boost/log/trivial.hpp>
#include <boost/exception/diagnostic_information.hpp> 
#include "v_repExtGrlInverseKinematics.h"
#include "grl/cisst/VrepVFController.hpp"

#include "v_repLib.h"

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


#define CONCAT(x,y,z) x y z
#define strConCat(x,y,z)	CONCAT(x,y,z)
#define LUA_GET_SENSOR_DATA_COMMAND "simExtSkeleton_getSensorData"



std::shared_ptr<grl::VrepInverseKinematicsController> InverseKinematicsControllerPG;

/*int getPathPosVectorFromObjectPose(int objectHandle, float relativeDistance) //This might be helpful if we decide to implement all the code in the plugin (instead of the lua script
{
	float positionOnPath[3];
	float eulerAngles[3];

	simGetPositionOnPath(objectHandle, relativeDistance, positionOnPath);
	simGetOrientationOnPath(objectHandle, eulerAngles);
	int o = getQuaternionFromEulerAngle(positionOnPath, eulerAngles);
	return (p,o);
}
*/

void LUA_SIM_EXT_GRL_IK_START(SLuaCallBack* p)
{
  if (!InverseKinematicsControllerPG) {
  
    BOOST_LOG_TRIVIAL(info) << "v_repExtInverseKinematicsController Starting Inverse Kinematics Plugin\n";
    InverseKinematicsControllerPG=std::make_shared<grl::VrepInverseKinematicsController>();
    InverseKinematicsControllerPG->construct();
  }
}

void LUA_SIM_EXT_GRL_IK_RESET(SLuaCallBack* p)
{
    BOOST_LOG_TRIVIAL(info) << "v_repExtInverseKinematicsController Starting Inverse Kinematics Plugin Data Collection\n";
    InverseKinematicsControllerPG=std::make_shared<grl::VrepInverseKinematicsController>();
    InverseKinematicsControllerPG->construct();
}

void LUA_SIM_EXT_GRL_IK_STOP(SLuaCallBack* p)
{
    
    BOOST_LOG_TRIVIAL(info) << "Ending v_repExtInverseKinematicsController plugin\n";
	InverseKinematicsControllerPG.reset();
}

void LUA_SIM_EXT_GRL_IK_ADD_FRAME(SLuaCallBack* p)
{
  if (InverseKinematicsControllerPG) {
    //InverseKinematicsControllerPG->addFrame();
  }
}

void LUA_SIM_EXT_GRL_IK_FIND_TRANSFORM(SLuaCallBack* p)
{
  if (InverseKinematicsControllerPG) {
    //InverseKinematicsControllerPG->estimateHandEyeScrew();
  }
}


void LUA_SIM_EXT_GRL_IK_APPLY_TRANSFORM(SLuaCallBack* p)
{
  if (InverseKinematicsControllerPG) {
    //InverseKinematicsControllerPG->applyEstimate();
  }
}


void LUA_SIM_EXT_GRL_IK_RESTORE_SENSOR_POSITION(SLuaCallBack* p)
{
  if (InverseKinematicsControllerPG) {
    //InverseKinematicsControllerPG->restoreSensorPosition();
  }
}

/// @todo implement and connect up this function
/// Returns the current transform estimate in a format that vrep understands
void LUA_SIM_EXT_GRL_IK_GET_TRANSFORM(SLuaCallBack* p)
{
  if (InverseKinematicsControllerPG) {
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
		BOOST_LOG_TRIVIAL(error) << "Error, could not find or correctly load the V-REP library. Cannot start 'PluginSkeleton' plugin.\n";
		return(0); // Means error, V-REP will unload this plugin
	}
	if (getVrepProcAddresses(vrepLib)==0)
	{
		BOOST_LOG_TRIVIAL(error) << "Error, could not find all required functions in the V-REP library. Cannot start 'PluginSkeleton' plugin.\n";
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
		BOOST_LOG_TRIVIAL(error) << "Sorry, your V-REP copy is somewhat old. Cannot start 'PluginSkeleton' plugin.\n";
		unloadVrepLibrary(vrepLib);
		return(0); // Means error, V-REP will unload this plugin
	}
	// ******************************************
    
    
	int noArgs[]={0}; // no input arguments
	simRegisterCustomLuaFunction("simExtGrlInverseKinematicsStart","number result=simExtGrlInverseKinematicsStart()",noArgs,LUA_SIM_EXT_GRL_IK_START);
	simRegisterCustomLuaFunction("simExtGrlInverseKinematicsStop","number result=simExtGrlInverseKinematicsStop()",noArgs,LUA_SIM_EXT_GRL_IK_STOP);
	simRegisterCustomLuaFunction("simExtGrlInverseKinematicsReset","number result=simExtGrlInverseKinematicsReset()",noArgs,LUA_SIM_EXT_GRL_IK_RESET);
	simRegisterCustomLuaFunction("simExtGrlInverseKinematicsAddFrame","number result=simExtGrlInverseKinematicsAddFrame()",noArgs,LUA_SIM_EXT_GRL_IK_ADD_FRAME);
	simRegisterCustomLuaFunction("simExtGrlInverseKinematicsFindTransform","number result=simExtGrlInverseKinematicsFindTransform()",noArgs,LUA_SIM_EXT_GRL_IK_FIND_TRANSFORM);
	simRegisterCustomLuaFunction("simExtGrlInverseKinematicsApplyTransform","number result=simExtGrlInverseKinematicsApplyTransform()",noArgs,LUA_SIM_EXT_GRL_IK_APPLY_TRANSFORM);
	simRegisterCustomLuaFunction("simExtGrlInverseKinematicsRestoreSensorPosition","number result=simExtGrlInverseKinematicsRestoreSensorPosition()",noArgs,LUA_SIM_EXT_GRL_IK_RESTORE_SENSOR_POSITION);
    
    
	// ******************************************

    BOOST_LOG_TRIVIAL(info) << "Inverse Kinematics plugin initialized. Build date/time: " << __DATE__ << " " << __TIME__ <<"\n";

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
    
    InverseKinematicsControllerPG.reset();

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
			refreshDlgFlag=true; // always a good idea to trigger a refresh of this plugin's dialog here
		}
        


		//...
		//////////////
		// PUT MAIN CODE HERE
		
		/////////////
		if (simGetSimulationState() != sim_simulation_advancing_abouttostop)	//checks if the simulation is still running
		{	
			//if(InverseKinematicsControllerPG) BOOST_LOG_TRIVIAL(info) << "current simulation time:" << simGetSimulationTime() << std::endl;					// gets simulation time point
		}
		// make sure it is "right" (what does that mean?)
		
			
		// find the v-rep C functions to do the following:
		////////////////////////////////////////////////////
		// Use handles that were found at the "start" of this simulation running

		// next few Lines get the joint angles, torque, etc from the simulation
		if (InverseKinematicsControllerPG)// && InverseKinematicsControllerPG->allHandlesSet == true // allHandlesSet now handled internally
		{
		
          // run one loop synchronizing the arm and plugin
          InverseKinematicsControllerPG->run_one();
		  
		}
	}

	if (message==sim_message_eventcallback_mainscriptabouttobecalled)
	{ // The main script is about to be run (only called while a simulation is running (and not paused!))
		
	}

	if (message==sim_message_eventcallback_simulationabouttostart)
	{ // Simulation is about to start

		/////////////////////////
		// PUT OBJECT STARTUP CODE HERE
		////////////////////
		// get the handles to all the objects, joints, etc that we need
		/////////////////////
		// simGetObjectHandle
        
        try {
            //InverseKinematicsControllerPG = std::make_shared<grl::VrepInverseKinematicsController>();
            //InverseKinematicsControllerPG->construct();
            //InverseKinematicsControllerPG->run_one();  // for debugging purposes only
            //InverseKinematicsControllerPG.reset();     // for debugging purposes only
        } catch (boost::exception& e){
            // log the error and print it to the screen, don't release the exception
            std::string initerr("v_repExtInverseKinematicsController plugin initialization error:\n" + boost::diagnostic_information(e));
            simAddStatusbarMessage( initerr.c_str());
            BOOST_LOG_TRIVIAL(error) <<  initerr;
        }
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

