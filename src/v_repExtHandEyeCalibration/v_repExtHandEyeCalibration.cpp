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
#include "v_repExtHandEyeCalibration.h"
#include "grl/vrep/HandEyeCalibrationVrepPlugin.hpp"

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
#define LUA_SIM_EXT_HAND_EYE_CALIB_START_COMMAND "simExtHandEyeCalibStart"
#define LUA_SIM_EXT_HAND_EYE_CALIB_APPLY_TRANSFORM_COMMAND "simExtHandEyeCalibApplyTransform"



std::shared_ptr<grl::HandEyeCalibrationVrepPlugin> handEyeCalibrationPG;
std::shared_ptr<spdlog::logger>                    loggerPG;

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


const int inArgs_HAND_EYE_CALIB_START[]={
 4,                   //   Example Value              // Parameter name
 sim_lua_arg_string,0, //  "RobotMillTip"            , // RobotBaseName,
 sim_lua_arg_string,0, //  "RobotMillTipTarget"      , // RobotTipName,
 sim_lua_arg_string,0, //  "Robotiiwa"               , // OpticalTrackerBaseName,
 sim_lua_arg_string,0, //  "tcp://0.0.0.0:30010"     , // OpticalTrackerTipName
};

const int inXArgs_HAND_EYE_CALIB_SCENE_NAME[]={
 1,                   //   Example Value              // Parameter name
 sim_lua_arg_string,0, //  "RobotMillTip"            , // RobotBaseName,
};
// -- KukaCommandMode options are JAVA and FRI
std::string LUA_SIM_EXT_HAND_EYE_CALIB_START_CALL_TIP("number result=simExtHandEyeCalibStart(string RobotBaseName , string RobotTipName, string OpticalTrackerBaseName, string OpticalTrackerDetectedObjectName)");
std::string LUA_SIM_EXT_HAND_EYE_CALIB_APPLY_TRANSFORM_CALL_TIP("number result=simExtHandEyeCalibApplyTransform(string sceneName)");


void LUA_SIM_EXT_HAND_EYE_CALIB_START(SLuaCallBack* p)
{
  if (!handEyeCalibrationPG) {

    loggerPG->info( "v_repExtHandEyeCalibration Starting Hand Eye Calibration Plugin Data Collection\n");

    	CLuaFunctionData data;

    	if (data.readDataFromLua(p,inArgs_HAND_EYE_CALIB_START,inArgs_HAND_EYE_CALIB_START[0], LUA_SIM_EXT_HAND_EYE_CALIB_START_COMMAND))
        {
    		std::vector<CLuaFunctionDataItem>* inData=data.getInDataPtr();
            std::string RobotBaseName((inData->at(0 ).stringData[0]));
            std::string RobotTipName((inData->at(1 ).stringData[0]));
            std::string OpticalTrackerBaseName((inData->at(2 ).stringData[0]));
            std::string OpticalTrackerDetectedObjectName(inData->at(3 ).stringData[0]);
            handEyeCalibrationPG=std::make_shared<grl::HandEyeCalibrationVrepPlugin>(
               // std::make_tuple(RobotBaseName , RobotTipName, OpticalTrackerBaseName, OpticalTrackerDetectedObjectName)

            );
            handEyeCalibrationPG->construct();

        }
        else
        {
            handEyeCalibrationPG=std::make_shared<grl::HandEyeCalibrationVrepPlugin>();
            handEyeCalibrationPG->construct();
        }
  }
}

void LUA_SIM_EXT_HAND_EYE_CALIB_RESET(SLuaCallBack* p)
{
    loggerPG->info("v_repExtHandEyeCalibration Starting Hand Eye Calibration Plugin Data Collection\n");
    handEyeCalibrationPG=std::make_shared<grl::HandEyeCalibrationVrepPlugin>();
    handEyeCalibrationPG->construct();
}

void LUA_SIM_EXT_HAND_EYE_CALIB_STOP(SLuaCallBack* p)
{

    loggerPG->info("Ending v_repExtHandEyeCalibration plugin\n");
	handEyeCalibrationPG.reset();
}

void LUA_SIM_EXT_HAND_EYE_CALIB_ADD_FRAME(SLuaCallBack* p)
{
  if (handEyeCalibrationPG) {
    handEyeCalibrationPG->addFrame();
  }
}

void LUA_SIM_EXT_HAND_EYE_CALIB_FIND_TRANSFORM(SLuaCallBack* p)
{
  if (handEyeCalibrationPG) {
    handEyeCalibrationPG->estimateHandEyeScrew();
  }
}


void LUA_SIM_EXT_HAND_EYE_CALIB_APPLY_TRANSFORM(SLuaCallBack* p)
{
  if (handEyeCalibrationPG) {
	  CLuaFunctionData data;

    if (data.readDataFromLua(p,inXArgs_HAND_EYE_CALIB_SCENE_NAME,inXArgs_HAND_EYE_CALIB_SCENE_NAME[0], LUA_SIM_EXT_HAND_EYE_CALIB_APPLY_TRANSFORM_COMMAND))
    {
    	std::vector<CLuaFunctionDataItem>* inData=data.getInDataPtr();
        std::string sceneName((inData->at(0 ).stringData[0]));
		handEyeCalibrationPG->applyEstimate(sceneName);
    }
   
   
  }
}


void LUA_SIM_EXT_HAND_EYE_CALIB_RESTORE_SENSOR_POSITION(SLuaCallBack* p)
{
  if (handEyeCalibrationPG) {
    handEyeCalibrationPG->restoreSensorPosition();
  }
}

/// @todo implement and connect up this function
/// Returns the current transform estimate in a format that vrep understands
void LUA_SIM_EXT_HAND_EYE_CALIB_GET_TRANSFORM(SLuaCallBack* p)
{
  if (handEyeCalibrationPG) {
  }
}



// This is the plugin start routine (called just once, just after the plugin was loaded):
VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer,int reservedInt)
{
    try 	{ 		 loggerPG = spdlog::stdout_logger_mt("console"); 	} 	catch (spdlog::spdlog_ex ex) 	{ 		loggerPG = spdlog::get("console"); 	}
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
		loggerPG->error("Sorry, your V-REP copy is somewhat old. Cannot start 'HandEyeCalib' plugin.\n");
		unloadVrepLibrary(vrepLib);
		return(0); // Means error, V-REP will unload this plugin
	}
	// ******************************************

	std::vector<int> inArgs;

    CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_HAND_EYE_CALIB_START,inArgs);
	simRegisterCustomLuaFunction
    (
        LUA_SIM_EXT_HAND_EYE_CALIB_START_COMMAND,
        LUA_SIM_EXT_HAND_EYE_CALIB_START_CALL_TIP.c_str(),
        &inArgs[0],
        LUA_SIM_EXT_HAND_EYE_CALIB_START
    );

	CLuaFunctionData::getInputDataForFunctionRegistration(inXArgs_HAND_EYE_CALIB_SCENE_NAME,inArgs);
	simRegisterCustomLuaFunction
    (
        LUA_SIM_EXT_HAND_EYE_CALIB_APPLY_TRANSFORM_COMMAND,
        LUA_SIM_EXT_HAND_EYE_CALIB_APPLY_TRANSFORM_CALL_TIP.c_str(),
        &inArgs[0],
        LUA_SIM_EXT_HAND_EYE_CALIB_APPLY_TRANSFORM
    );


	int noArgs[]={0}; // no input arguments
	//simRegisterCustomLuaFunction("simExtHandEyeCalibStart","number result=simExtHandEyeCalibStart()",noArgs,LUA_SIM_EXT_HAND_EYE_CALIB_START);
	//simRegisterCustomLuaFunction("simExtHandEyeCalibStart","number result=simExtHandEyeCalibStart()",inArgs_HAND_EYE_CALIB_START,LUA_SIM_EXT_HAND_EYE_CALIB_START);
	simRegisterCustomLuaFunction("simExtHandEyeCalibStop","number result=simExtHandEyeCalibStop()",noArgs,LUA_SIM_EXT_HAND_EYE_CALIB_STOP);
	simRegisterCustomLuaFunction("simExtHandEyeCalibReset","number result=simExtHandEyeCalibReset()",noArgs,LUA_SIM_EXT_HAND_EYE_CALIB_RESET);
	simRegisterCustomLuaFunction("simExtHandEyeCalibAddFrame","number result=simExtHandEyeCalibAddFrame()",noArgs,LUA_SIM_EXT_HAND_EYE_CALIB_ADD_FRAME);
	simRegisterCustomLuaFunction("simExtHandEyeCalibFindTransform","number result=simExtHandEyeCalibFindTransform()",noArgs,LUA_SIM_EXT_HAND_EYE_CALIB_FIND_TRANSFORM);
	simRegisterCustomLuaFunction("simExtHandEyeCalibRestoreSensorPosition","number result=simExtHandEyeCalibRestoreSensorPosition()",noArgs,LUA_SIM_EXT_HAND_EYE_CALIB_RESTORE_SENSOR_POSITION);


	// ******************************************

    loggerPG->info("Hand Eye Calibration plugin initialized. Build date/time: ", __DATE__, " ", __TIME__,"\n");

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

    handEyeCalibrationPG.reset();

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
			//if(handEyeCalibrationPG) BOOST_LOG_TRIVIAL(info) << "current simulation time:" << simGetSimulationTime() << std::endl;					// gets simulation time point
		}
		// make sure it is "right" (what does that mean?)


		// find the v-rep C functions to do the following:
		////////////////////////////////////////////////////
		// Use handles that were found at the "start" of this simulation running

		// next few Lines get the joint angles, torque, etc from the simulation
		if (handEyeCalibrationPG)// && HandEyeCalibrationPG->allHandlesSet == true // allHandlesSet now handled internally
		{

          // run one loop synchronizing the arm and plugin
          // handEyeCalibrationPG->run_one();

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
            //handEyeCalibrationPG = std::make_shared<grl::HandEyeCalibrationVrepPlugin>();
            //handEyeCalibrationPG->construct();
            //HandEyeCalibrationPG->run_one();  // for debugging purposes only
            //HandEyeCalibrationPG.reset();     // for debugging purposes only
        } catch (boost::exception& e){
            // log the error and print it to the screen, don't release the exception
            std::string initerr("v_repExtHandEyeCalibration plugin initialization error:\n" + boost::diagnostic_information(e));
            simAddStatusbarMessage( initerr.c_str());
            loggerPG->error(initerr);
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

