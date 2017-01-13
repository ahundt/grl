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
#include "v_repExtKukaLBRiiwa.h"
#include "grl/vrep/KukaLBRiiwaVrepPlugin.hpp"

#include <spdlog/spdlog.h>

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
#define LUA_KUKA_LBR_IIWA_START_COMMAND "simExtKukaLBRiiwaStart"



std::shared_ptr<grl::vrep::KukaVrepPlugin> kukaPluginPG;
std::shared_ptr<spdlog::logger>            loggerPG;

const int inArgs_KUKA_LBR_IIWA_START[]={
 16,                   //   Example Value              // Parameter name
 sim_lua_arg_string|sim_lua_arg_table,0, // joint handle table with 0 or more strings
 sim_lua_arg_string,0, //  "RobotFlangeTip"          , // RobotFlangeTipHandle,
 sim_lua_arg_string,0, //  "RobotMillTip"            , // RobotTipHandle,
 sim_lua_arg_string,0, //  "RobotMillTipTarget"      , // RobotTargetHandle,
 sim_lua_arg_string,0, //  "Robotiiwa"               , // RobotTargetBaseHandle, aka RobotName
 sim_lua_arg_string,0, //  "KUKA_LBR_IIWA_14_R820"   , // RobotModel (options are KUKA_LBR_IIWA_14_R820, KUKA_LBR_IIWA_7_R800)
 sim_lua_arg_string,0, //  "tcp://0.0.0.0:30010"     , // LocalUDPAddress
 sim_lua_arg_string,0, //  "30010"                   , // LocalUDPPort
 sim_lua_arg_string,0, //  "tcp://172.31.1.147:30010", // RemoteUDPAddress
 sim_lua_arg_string,0, //  "192.170.10.100"          , // LocalHostKukaKoniUDPAddress,
 sim_lua_arg_string,0, //  "30200"                   , // LocalHostKukaKoniUDPPort,
 sim_lua_arg_string,0, //  "192.170.10.2"            , // RemoteHostKukaKoniUDPAddress,
 sim_lua_arg_string,0, //  "30200"                   , // RemoteHostKukaKoniUDPPort
 sim_lua_arg_string,0, //  "JAVA"                    , // KukaCommandMode (options are "JAVA", "FRI")
 sim_lua_arg_string,0, //  "FRI"                     , // KukaMonitorMode (options are "JAVA", "FRI")
 sim_lua_arg_string,0, //  "IK_Group1_iiwa"            // IKGroupName (VREP built in inverse kinematics group)
};

std::string LUA_KUKA_LBR_IIWA_START_CALL_TIP("number result=simExtKukaLBRiiwaStart(string_table JointHandles , string RobotTipHandle, string RobotFlangeTipHandle, string RobotTargetHandle, string RobotTargetBaseHandle, string RobotModel, string LocalUDPAddress, string LocalUDPPort, string RemoteUDPAddress, string LocalHostKukaKoniUDPAddress, string LocalHostKukaKoniUDPPort, string RemoteHostKukaKoniUDPAddress, string RemoteHostKukaKoniUDPPort, string KukaCommandMode, string KukaMonitorMode, string IKGroupName) -- KukaCommandMode options are JAVA and FRI");

void LUA_SIM_EXT_KUKA_LBR_IIWA_START(SLuaCallBack* p)
{ // the callback function of the new Lua command ("simExtSkeleton_getSensorData")
  // return Lua Table or arrays containing position, torque, torque minus motor force, timestamp, FRI state
  
  try {
      if (!kukaPluginPG) {
        loggerPG->error("Starting KUKA LBR iiwa plugin connection to Kuka iiwa\n" );

    	CLuaFunctionData data;

    	if (data.readDataFromLua(p,inArgs_KUKA_LBR_IIWA_START,inArgs_KUKA_LBR_IIWA_START[0],LUA_KUKA_LBR_IIWA_START_COMMAND))
        {

    		std::vector<CLuaFunctionDataItem>* inData=data.getInDataPtr();
            std::vector<std::string> JointHandles;
            for (size_t i=0;i<inData->at(0).stringData.size();i++)
            {
              JointHandles.push_back(std::string(inData->at(0).stringData[i].c_str()));
            }
            std::string RobotFlangeTipHandle                (inData->at(1 ).stringData[0]);
            std::string RobotTipHandle                      (inData->at(2 ).stringData[0]);
            std::string RobotTargetHandle                   (inData->at(3 ).stringData[0]);
            std::string RobotTargetBaseHandle               (inData->at(4 ).stringData[0]);
            std::string RobotModel                          (inData->at(5 ).stringData[0]);
            std::string LocalUDPAddress                     (inData->at(6 ).stringData[0]);
            std::string LocalUDPPort                        (inData->at(7 ).stringData[0]);
            std::string RemoteUDPAddress                    (inData->at(8 ).stringData[0]);
            std::string LocalHostKukaKoniUDPAddress         (inData->at(9 ).stringData[0]);
            std::string LocalHostKukaKoniUDPPort            (inData->at(10).stringData[0]);
            std::string RemoteHostKukaKoniUDPAddress        (inData->at(11).stringData[0]);
            std::string RemoteHostKukaKoniUDPPort           (inData->at(12).stringData[0]);
            std::string KukaCommandMode                     (inData->at(13).stringData[0]);
            std::string KukaMonitorMode                     (inData->at(14).stringData[0]);
            std::string IKGroupName                         (inData->at(15).stringData[0]);
            
        
            kukaPluginPG=std::make_shared<grl::vrep::KukaVrepPlugin>(
                std::make_tuple(
                    JointHandles                  ,
                    RobotFlangeTipHandle          ,
                    RobotTipHandle                ,
                    RobotTargetHandle             ,
                    RobotTargetBaseHandle         ,
                    RobotModel                    ,
                    LocalUDPAddress               ,
                    LocalUDPPort                  ,
                    RemoteUDPAddress              ,
                    LocalHostKukaKoniUDPAddress   ,
                    LocalHostKukaKoniUDPPort      ,
                    RemoteHostKukaKoniUDPAddress  ,
                    RemoteHostKukaKoniUDPPort     ,
                    KukaCommandMode               ,
                    KukaMonitorMode               ,
                    IKGroupName
                )
            );
            kukaPluginPG->construct();
        } else {
            /// @todo report an error?
            // use default params
            kukaPluginPG=std::make_shared<grl::vrep::KukaVrepPlugin>();
            kukaPluginPG->construct();
        }
        
        
        
        
      }
  
  } catch (const boost::exception& e){
      // log the error and print it to the screen, don't release the exception
      std::string initerr("v_repExtKukaLBRiiwa plugin encountered the following error and will disable itself:\n" + boost::diagnostic_information(e));
      simAddStatusbarMessage( initerr.c_str());
      loggerPG->error( initerr );
      kukaPluginPG.reset();
  } catch (const std::exception& e){
      // log the error and print it to the screen, don't release the exception
      std::string initerr("v_repExtKukaLBRiiwa plugin encountered the following error and will disable itself:\n" + boost::diagnostic_information(e));
      simAddStatusbarMessage( initerr.c_str());
      loggerPG->error( initerr );
      kukaPluginPG.reset();
  } catch (...){
      // log the error and print it to the screen, don't release the exception
      std::string initerr("v_repExtKukaLBRiiwa plugin encountered an unknown error and will disable itself. Please debug this issue! file and line:" + std::string(__FILE__) + " " + boost::lexical_cast<std::string>(__LINE__) + "\n");
      simAddStatusbarMessage( initerr.c_str());
      loggerPG->error( initerr );
      kukaPluginPG.reset();
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
		loggerPG->error("Error, could not find or correctly load the V-REP library. Cannot start 'v_repExtKukaLBRiiwa' plugin.\n" );
		return(0); // Means error, V-REP will unload this plugin
	}
	if (getVrepProcAddresses(vrepLib)==0)
	{
		loggerPG->error("Error, could not find all required functions in the V-REP library. Cannot start 'v_repExtKukaLBRiiwa' plugin.\n" );
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
		loggerPG->error("Sorry, your V-REP copy is somewhat old. Cannot start 'v_repExtKukaLBRiiwa' plugin.\n" );
		unloadVrepLibrary(vrepLib);
		return(0); // Means error, V-REP will unload this plugin
	}
	// ******************************************


	// Register the new Lua command "simExtSkeleton_getSensorData":
	// ******************************************
    
    std::vector<int> inArgs;
    
    CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_KUKA_LBR_IIWA_START,inArgs);
	simRegisterCustomLuaFunction
    (
        LUA_KUKA_LBR_IIWA_START_COMMAND,
        LUA_KUKA_LBR_IIWA_START_CALL_TIP.c_str(),
        &inArgs[0],
        LUA_SIM_EXT_KUKA_LBR_IIWA_START
    );
    
    
    
	// Expected input arguments are: int sensorIndex, float floatParameters[3], int intParameters[2]
	//int inArgs_getSensorData[]={3,sim_lua_arg_int,sim_lua_arg_float|sim_lua_arg_table,sim_lua_arg_int|sim_lua_arg_table}; // this says we expect 3 arguments (1 integer, a table of floats, and a table of ints)
	// Return value can change on the fly, so no need to specify them here, except for the calltip.
	// Now register the callback:
	//simRegisterCustomLuaFunction(LUA_GET_SENSOR_DATA_COMMAND,strConCat("number result,table data,number distance=",LUA_GET_SENSOR_DATA_COMMAND,"(number sensorIndex,table_3 floatParams,table_2 intParams)"),inArgs_getSensorData,LUA_GET_SENSOR_DATA_CALLBACK);
	// ******************************************

    loggerPG->error("KUKA LBR iiwa plugin initialized. Build date/time: ", __DATE__, " ", __TIME__ );

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
    
    kukaPluginPG.reset();

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
			//if(kukaPluginPG) loggerPG->error("current simulation time:" << simGetSimulationTime() << std::endl );					// gets simulation time point
		}
		// make sure it is "right" (what does that mean?)
		
			
		// find the v-rep C functions to do the following:
		////////////////////////////////////////////////////
		// Use handles that were found at the "start" of this simulation running

		// next few Lines get the joint angles, torque, etc from the simulation
		if (kukaPluginPG)// && kukaPluginPG->allHandlesSet == true // allHandlesSet now handled internally
		{
              try
              {
                  // run one loop synchronizing the arm and plugin
                  kukaPluginPG->run_one();
          
              } catch (const boost::exception& e){
                  // log the error and print it to the screen, don't release the exception
                  std::string initerr("v_repExtKukaLBRiiwa plugin encountered the following error and will disable itself:\n" + boost::diagnostic_information(e));
                  simAddStatusbarMessage( initerr.c_str());
                  loggerPG->error( initerr );
                  kukaPluginPG.reset();
              } catch (const std::exception& e){
                  // log the error and print it to the screen, don't release the exception
                  std::string initerr("v_repExtKukaLBRiiwa plugin encountered the following error and will disable itself:\n" + boost::diagnostic_information(e));
                  simAddStatusbarMessage( initerr.c_str());
                  loggerPG->error( initerr );
                  kukaPluginPG.reset();
              } catch (...){
                  // log the error and print it to the screen, don't release the exception
                  std::string initerr("v_repExtKukaLBRiiwa plugin encountered an unknown error and will disable itself. Please debug this issue! file and line:" + std::string(__FILE__) + " " + boost::lexical_cast<std::string>(__LINE__) + "\n");
                  simAddStatusbarMessage( initerr.c_str());
                  loggerPG->error( initerr );
                  kukaPluginPG.reset();
              }
                      
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
        
//        try {
//            loggerPG->error("Starting KUKA LBR iiwa plugin connection to Kuka iiwa\n" );
//            kukaPluginPG = std::make_shared<grl::KukaVrepPlugin>();
//            kukaPluginPG->construct();
//            //kukaPluginPG->run_one();  // for debugging purposes only
//            //kukaPluginPG.reset();     // for debugging purposes only
//        } catch (boost::exception& e){
//            // log the error and print it to the screen, don't release the exception
//            std::string initerr("v_repExtKukaLBRiiwa plugin initialization error:\n" + boost::diagnostic_information(e));
//            simAddStatusbarMessage( initerr.c_str());
//            loggerPG->error( initerr );
//        }
	}

	if (message==sim_message_eventcallback_simulationended)
	{ // Simulation just ended

		/////////////////////////
		// PUT OBJECT RESET CODE HERE
		// close out as necessary
		////////////////////
        loggerPG->error("Ending KUKA LBR iiwa plugin connection to Kuka iiwa\n" );
		kukaPluginPG.reset();

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

