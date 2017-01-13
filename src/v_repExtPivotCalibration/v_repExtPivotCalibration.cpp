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


#include "luaFunctionData.h"
#include "v_repExtPivotCalibration.h"
#include "grl/vrep/PivotCalibrationVrepPlugin.hpp"

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



std::shared_ptr<grl::PivotCalibrationVrepPlugin> pivotCalibrationPG;
std::shared_ptr<spdlog::logger>                  loggerPG;


const int inArgs_PIVOT_CALIB_START[]={
 4,                   //   Example Value              // Parameter name
 sim_lua_arg_string,0, //  "RobotMillTip"            , // RobotBaseName,
 sim_lua_arg_string,0, //  "RobotMillTipTarget"      , // RobotTipName,
 sim_lua_arg_string,0, //  "RobotMillTip"            , // RobotBaseName,
 sim_lua_arg_string,0, //  "RobotMillTipTarget"      , // RobotTipName,
};

std::string LUA_SIM_EXT_PIVOT_CALIB_START_CALL_TIP("number result=simExtPivotCalibStart(string ToolTipToModifyName, string ToolTipToMeasureName, string ToolBaseToModifyName, string ToolBaseToMeasureName)");


void LUA_SIM_EXT_PIVOT_CALIB_START(SLuaCallBack* p)
{
  if (!pivotCalibrationPG) {
  
    loggerPG->info( "v_repExtPivotCalibration Starting Pivot Calibration Plugin Data Collection\n");
    
    	CLuaFunctionData data;

    	if (data.readDataFromLua(p,inArgs_PIVOT_CALIB_START,inArgs_PIVOT_CALIB_START[0],"simExtPivotCalibStart"))
        {
    		std::vector<CLuaFunctionDataItem>* inData=data.getInDataPtr();
            std::string ToolTipModifyName((inData->at(0 ).stringData[0]));
            std::string ToolTipMeasureName((inData->at(1 ).stringData[0]));
            std::string ToolBaseModifyName((inData->at(0 ).stringData[0]));
            std::string ToolBaseMeasureName((inData->at(2 ).stringData[0]));
            pivotCalibrationPG=std::make_shared<grl::PivotCalibrationVrepPlugin>(
                std::make_tuple(ToolTipModifyName , ToolTipMeasureName, ToolBaseModifyName, ToolBaseMeasureName)
        
            );
            pivotCalibrationPG->construct();

        }
        else
        {
            pivotCalibrationPG=std::make_shared<grl::PivotCalibrationVrepPlugin>();
            pivotCalibrationPG->construct();
        }
  }
}

const int inArgs_PIVOT_CALIB_ALGORITHM[]={
 1,                   //   Example Value              // Parameter name
 sim_lua_arg_string,0, //  "RobotMillTip"            , // RobotBaseName,
};

std::string LUA_SIM_EXT_PIVOT_CALIB_ALGORITHM_CALL_TIP("number result=simExtPivotCalibAlgorithm(string AlgorithmName) -- Algorithm options are TWO_STEP_PROCEDURE and COMBINATORICAL_APPROACH");


void LUA_SIM_EXT_PIVOT_CALIB_ALGORITHM(SLuaCallBack* p)
{
  if (!pivotCalibrationPG) {
  
    loggerPG->info( "v_repExtPivotCalibration Setting Algorithm\n");
    
    	CLuaFunctionData data;

    	if (pivotCalibrationPG.get()!=nullptr && data.readDataFromLua(p,inArgs_PIVOT_CALIB_ALGORITHM,inArgs_PIVOT_CALIB_ALGORITHM[0],"simExtPivotCalibAlgorithm"))
        {
    		std::vector<CLuaFunctionDataItem>* inData=data.getInDataPtr();
            std::string AlgorithmName((inData->at(0 ).stringData[0]));
            pivotCalibrationPG->setAlgorithm(AlgorithmName);

        }
        else
        {
        }
  }
}

void LUA_SIM_EXT_PIVOT_CALIB_RESET(SLuaCallBack* p)
{
    loggerPG->info( "v_repExtPivotCalibration Starting Pivot Calibration Plugin Data Collection\n");
    pivotCalibrationPG=std::make_shared<grl::PivotCalibrationVrepPlugin>();
    pivotCalibrationPG->construct();
}

void LUA_SIM_EXT_PIVOT_CALIB_STOP(SLuaCallBack* p)
{
    
    loggerPG->info( "Ending v_repExtPivotCalibration plugin\n");
	pivotCalibrationPG.reset();
}

void LUA_SIM_EXT_PIVOT_CALIB_ADD_FRAME(SLuaCallBack* p)
{
  if (pivotCalibrationPG) {
    pivotCalibrationPG->addFrame();
  }
}

void LUA_SIM_EXT_PIVOT_CALIB_FIND_TRANSFORM(SLuaCallBack* p)
{
  if (pivotCalibrationPG) {
    pivotCalibrationPG->estimatePivotOffset();
  }
}


void LUA_SIM_EXT_PIVOT_CALIB_APPLY_TRANSFORM(SLuaCallBack* p)
{
  if (pivotCalibrationPG) {
    pivotCalibrationPG->applyEstimate();
  }
}


void LUA_SIM_EXT_PIVOT_CALIB_RESTORE_SENSOR_POSITION(SLuaCallBack* p)
{
  if (pivotCalibrationPG) {
    pivotCalibrationPG->restoreSensorPosition();
  }
}

/// @todo implement and connect up this function
/// Returns the current transform estimate in a format that vrep understands
void LUA_SIM_EXT_PIVOT_CALIB_GET_TRANSFORM(SLuaCallBack* p)
{
  if (pivotCalibrationPG) {
  }
}



// This is the plugin start routine (called just once, just after the plugin was loaded):
VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer,int reservedInt)
{
    loggerPG = spdlog::stdout_logger_mt("console");
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
		loggerPG->error("Error, could not find or correctly load the V-REP library. Cannot start 'PluginSkeleton' plugin.\n");
		return(0); // Means error, V-REP will unload this plugin
	}
	if (getVrepProcAddresses(vrepLib)==0)
	{
		loggerPG->error("Error, could not find all required functions in the V-REP library. Cannot start 'PluginSkeleton' plugin.\n");
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
		loggerPG->error("Sorry, your V-REP copy is somewhat old. Cannot start 'PluginSkeleton' plugin.\n");
		unloadVrepLibrary(vrepLib);
		return(0); // Means error, V-REP will unload this plugin
	}
	// ******************************************
    
    
	int noArgs[]={0}; // no input arguments
	simRegisterCustomLuaFunction("simExtPivotCalibStart",LUA_SIM_EXT_PIVOT_CALIB_START_CALL_TIP.c_str(),inArgs_PIVOT_CALIB_START,LUA_SIM_EXT_PIVOT_CALIB_START);
	simRegisterCustomLuaFunction("simExtPivotCalibAlgorithm",LUA_SIM_EXT_PIVOT_CALIB_ALGORITHM_CALL_TIP.c_str(),inArgs_PIVOT_CALIB_ALGORITHM,LUA_SIM_EXT_PIVOT_CALIB_ALGORITHM);
	simRegisterCustomLuaFunction("simExtPivotCalibStop","number result=simExtPivotCalibStop()",noArgs,LUA_SIM_EXT_PIVOT_CALIB_STOP);
	simRegisterCustomLuaFunction("simExtPivotCalibReset","number result=simExtPivotCalibReset()",noArgs,LUA_SIM_EXT_PIVOT_CALIB_RESET);
	simRegisterCustomLuaFunction("simExtPivotCalibAddFrame","number result=simExtPivotCalibAddFrame()",noArgs,LUA_SIM_EXT_PIVOT_CALIB_ADD_FRAME);
	simRegisterCustomLuaFunction("simExtPivotCalibFindTransform","number result=simExtPivotCalibFindTransform()",noArgs,LUA_SIM_EXT_PIVOT_CALIB_FIND_TRANSFORM);
	simRegisterCustomLuaFunction("simExtPivotCalibApplyTransform","number result=simExtPivotCalibApplyTransform()",noArgs,LUA_SIM_EXT_PIVOT_CALIB_APPLY_TRANSFORM);
	simRegisterCustomLuaFunction("simExtPivotCalibRestoreSensorPosition","number result=simExtPivotCalibRestoreSensorPosition()",noArgs,LUA_SIM_EXT_PIVOT_CALIB_RESTORE_SENSOR_POSITION);
    
    
	// ******************************************

    loggerPG->info( "Pivot Calibration plugin initialized. Build date/time: ", __DATE__, " ", __TIME__);

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
    
    pivotCalibrationPG.reset();

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
			//if(pivotCalibrationPG) loggerPG->info( "current simulation time:" << simGetSimulationTime() << std::endl);					// gets simulation time point
		}
		// make sure it is "right" (what does that mean?)
		
			
		// find the v-rep C functions to do the following:
		////////////////////////////////////////////////////
		// Use handles that were found at the "start" of this simulation running

		// next few Lines get the joint angles, torque, etc from the simulation
		if (pivotCalibrationPG)// && PivotCalibrationPG->allHandlesSet == true // allHandlesSet now handled internally
		{
		
          // run one loop synchronizing the arm and plugin
          // pivotCalibrationPG->run_one();
		  
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
            //pivotCalibrationPG = std::make_shared<grl::PivotCalibrationVrepPlugin>();
            //pivotCalibrationPG->construct();
            //PivotCalibrationPG->run_one();  // for debugging purposes only
            //PivotCalibrationPG.reset();     // for debugging purposes only
        } catch (boost::exception& e){
            // log the error and print it to the screen, don't release the exception
            std::string initerr("v_repExtPivotCalibration plugin initialization error:\n" + boost::diagnostic_information(e));
            simAddStatusbarMessage( initerr.c_str());
            loggerPG->error( initerr);
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

