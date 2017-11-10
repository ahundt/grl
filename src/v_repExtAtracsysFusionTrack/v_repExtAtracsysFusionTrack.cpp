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

#include <boost/range/algorithm/remove_if.hpp>

#include "luaFunctionData.h"
#include "v_repExtAtracsysFusionTrack.h"
#include "grl/vrep/AtracsysFusionTrackVrepPlugin.hpp"
#include "grl/time.hpp"

#include "v_repLib.h"

#ifdef _WIN32
#include <shlwapi.h>
#pragma comment(lib, "Shlwapi.lib")
#endif /* _WIN32 */

#if defined(__linux) || defined(__APPLE__)
#include <unistd.h>
#include <string.h>
#define _stricmp(x, y) strcasecmp(x, y)
#endif

#define PLUGIN_VERSION 1

LIBRARY vrepLib; // the V-REP library that we will dynamically load and bind

#define CONCAT(x, y, z) x y z
#define strConCat(x, y, z) CONCAT(x, y, z)

/// @todo consider parameterizing this or placing it in a class so multiple trackers can exist.
grl::FusionTrackLogAndTrack::Params fusionTrackParamsG = grl::FusionTrackLogAndTrack::emptyDefaultParams();
std::shared_ptr<grl::FusionTrackLogAndTrack> fusionTrackPG;
/// spdlog is a header only library. Just copy the files under include to your build tree and use a C++11 compiler.
std::shared_ptr<spdlog::logger> loggerPG;
/// Recording will begin when the simulation starts running, and log files will be saved every time it stops running.
bool recordWhileSimulationIsRunningG = false;


void removeGeometryID(std::string geometryID_lua_param, grl::FusionTrackLogAndTrack::Params &params)
{
    auto &currentObjects = params.MotionConfigParamsVector; // get the motion configuration params
    // convert the handle string to an integer
    int geometryID = boost::lexical_cast<int>(geometryID_lua_param);
    // remove the id from the global params object
    // @todo this is redundant when the object is active, should we retain config between resets?
    boost::range::remove_if(currentObjects,
                            [geometryID](grl::FusionTrackLogAndTrack::MotionConfigParams &currentObject) {
                                //     remove if this geometry id equals the one passed to lua
                                return std::get<grl::FusionTrackLogAndTrack::GeometryID>(currentObject) == geometryID;

                            });
}

///////////////////////////////////////////////
//   LUA function to Add Geometry INI files  //
///////////////////////////////////////////////

#define LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_ADD_GEOMETRY_COMMAND "simExtAtracsysFusionTrackAddGeometry"

/*
inputArgumentTypes: array indicating the desired input arguments.
This is important so that the simulator knows how to convert between Lua types (e.g. Lua value"4.2" can be converted to int 4, to float 4.2, to string "4.2" or to Boolean 1).
Can be NULL, in that case no input argument is forwarded to the callback address.
inputArgumentTypes[0] represents the number of input arguments we wish to be forwarded,
inputArgumentTypes[1] is the type of the first argument,
inputArgumentTypes[2] is the type of the second argument, etc.
An input argument type can be sim_lua_arg_bool, sim_lua_arg_int, sim_lua_arg_float, sim_lua_arg_string or sim_lua_arg_charbuff, that can be combined (|) with sim_lua_arg_table if table values are desired.
And exception to this is sim_lua_arg_charbuff, which cannot be combined with sim_lua_arg_table.
*/
const int inArgs_LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_ADD_GEOMETRY[] = {
    1,
    sim_lua_arg_string, 0 // geometry file
};

std::string LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_ADD_GEOMETRY_CALL_TIP("number result=simExtAtracsysFusionTrackAddGeometry(string filename)");

void LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_ADD_GEOMETRY(SLuaCallBack *p)
{
    /// https://github.com/jhu-lcsr/vrep_ros_packages/blob/master/vrep_plugin_skeleton/include/luaFunctionData.h
    CLuaFunctionData data;
    /// https://github.com/jhu-lcsr/vrep_ros_packages/blob/master/vrep_plugin_skeleton/src/luaFunctionData.cpp
    /// bool CLuaFunctionData::readDataFromLua(
    /// 	const SLuaCallBack* p,
    ///		const int* expectedArguments,
    ///		int requiredArgumentCount,
    ///		const char* functionName)
    if (data.readDataFromLua(p, inArgs_LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_ADD_GEOMETRY, inArgs_LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_ADD_GEOMETRY[0], LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_ADD_GEOMETRY_COMMAND))
    {
        /// use this when reading data from Lua from inside of a custom Lua function call
        std::vector<CLuaFunctionDataItem> *inData = data.getInDataPtr();
        std::string markerINIpath(inData->at(0).stringData[0]);
        // insert a new ini file path
        fusionTrackParamsG.FusionTrackParams.geometryFilenames.push_back(markerINIpath);

        std::string log_message = std::string("simExtAtracsysFusionTrackAddGeometry: ") + markerINIpath;
        simAddStatusbarMessage(log_message.c_str());
        loggerPG->info(log_message);
    }
}

/////////////////////////////////////////////////////////////////////////////
//   LUA function to Set the object representing the optical tracker base  //
/////////////////////////////////////////////////////////////////////////////

#define LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_SET_OPTICAL_TRACKER_BASE_COMMAND "simExtAtracsysFusionTrackSetOpticalTrackerBase"

const int inArgs_LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_SET_OPTICAL_TRACKER_BASE[] = {
    1,
    sim_lua_arg_string, 0 // string identifying the optical tracker base
};

std::string LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_SET_OPTICAL_TRACKER_BASE_CALL_TIP("number result=simExtAtracsysFusionTrackSetOpticalTrackerBase(string baseObject)");

void LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_SET_OPTICAL_TRACKER_BASE(SLuaCallBack *p)
{

    CLuaFunctionData data;

    if (data.readDataFromLua(p, inArgs_LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_SET_OPTICAL_TRACKER_BASE, inArgs_LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_SET_OPTICAL_TRACKER_BASE[0], LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_SET_OPTICAL_TRACKER_BASE_COMMAND))
    {
        std::vector<CLuaFunctionDataItem> *inData = data.getInDataPtr();
        std::string opticalTrackerBase(inData->at(0).stringData[0]);
        int base_handle = grl::vrep::getHandle(opticalTrackerBase);
        fusionTrackParamsG.OpticalTrackerBase = base_handle; // set the optical tracker base
        if(fusionTrackPG)
        {
            fusionTrackPG->setOpticalTrackerBase(base_handle);
        }
        std::string log_message = std::string("simExtAtracsysFusionTrackSetOpticalTrackerBase base object name: ") + opticalTrackerBase + " handle: " + boost::lexical_cast<std::string>(base_handle);
        simAddStatusbarMessage(log_message.c_str());
        loggerPG->info(log_message);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
//  LUA function to Define an object that will be controlled by the optical tracker and how it will move
/////////////////////////////////////////////////////////////////////////////////////////

#define LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_OBJECT_COMMAND "simExtAtracsysFusionTrackAddObject"

const int inArgs_LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_OBJECT[] = {
    4,
    sim_lua_arg_string, 0, // ObjectToMove
    sim_lua_arg_string, 0, // FrameInWhichToMoveObject
    sim_lua_arg_string, 0, // ObjectBeingMeasured
    sim_lua_arg_string, 0  // GeometryID
};

std::string LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_OBJECT_CALL_TIP("number result=simExtAtracsysFusionTrackAddObject( string ObjectToMove, string FrameInWhichToMoveObject, string ObjectBeingMeasured, string GeometryID)");

/// Add a motion config param, defining the object that should be moved in VREP and the frame in which it should be moved
/// @todo if simExtAtracsysFusionTrackAddObject is called twice, the config will contain duplicates. This may be ok for now because duplicates will be overwritten.
void LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_OBJECT(SLuaCallBack *p)
{

    CLuaFunctionData data;

    if (data.readDataFromLua(p, inArgs_LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_OBJECT, inArgs_LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_OBJECT[0], LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_OBJECT_COMMAND))
    {
        std::vector<CLuaFunctionDataItem> *inData = data.getInDataPtr();

        grl::VrepStringMotionConfigParams newObjectToTrack = std::make_tuple( // insert the new parameters
            inData->at(0).stringData[0],		 // ObjectToMove
            inData->at(1).stringData[0],		 // FrameInWhichToMoveObject
            inData->at(2).stringData[0],		 // ObjectBeingMeasured
            inData->at(3).stringData[0]			 // GeometryID
            );

            std::string output = std::string("simExtAtracsysFusionTrackAddObject: ObjectToMove: ") + inData->at(0).stringData[0] +
            std::string(" FrameInWhichToMoveObject: ") + inData->at(1).stringData[0] +		 // FrameInWhichToMoveObject
            std::string(" ObjectBeingMeasured: ") + inData->at(2).stringData[0] +		 // ObjectBeingMeasured
            std::string(" GeometryID: ") + inData->at(3).stringData[0];			 // GeometryID
            loggerPG->info(output);
        // convert the tuple of strings to a tuple of ints
        auto newObjectToTrackIDs = grl::VrepStringToLogAndTrackMotionConfigParams(newObjectToTrack);
        fusionTrackParamsG.MotionConfigParamsVector.push_back(newObjectToTrackIDs);

        if (fusionTrackPG)
        {

            fusionTrackPG->add_object(newObjectToTrackIDs);
        }
    }
}

/////////////////////////////////////////////////////////////////////////////
//   LUA function to stop tracking an object with the specified GeometryID //
/////////////////////////////////////////////////////////////////////////////

#define LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_STOP_TRACKING_OBJECT_WITH_GEOMETRY_ID_COMMAND "simExtAtracsysFusionTrackStopTrackingObjectWithGeometryID"

const int inArgs_LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_STOP_TRACKING_OBJECT_WITH_GEOMETRY_ID[] = {
    1,
    sim_lua_arg_string, 0 // string identifying the optical tracker base
};

std::string LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_STOP_TRACKING_OBJECT_WITH_GEOMETRY_ID_CALL_TIP("number result=simExtAtracsysFusionTrackStopTrackingObjectWithGeometryID(string GeometryID)");

void LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_STOP_TRACKING_OBJECT_WITH_GEOMETRY_ID(SLuaCallBack *p)
{

    CLuaFunctionData data;

    if (data.readDataFromLua(p, inArgs_LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_SET_OPTICAL_TRACKER_BASE, inArgs_LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_SET_OPTICAL_TRACKER_BASE[0], LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_SET_OPTICAL_TRACKER_BASE_COMMAND))
    {
        std::vector<CLuaFunctionDataItem> *inData = data.getInDataPtr();
        std::string geometryID_lua_param(inData->at(0).stringData[0]);

        removeGeometryID(geometryID_lua_param, fusionTrackParamsG);

        if (fusionTrackPG)
        {
            fusionTrackPG->remove_geometry(geometryID_lua_param);
        }
    }
}

/////////////////////////////////////////////////////////////////////////////
//   LUA function to clear the set of objects being updated                //
/////////////////////////////////////////////////////////////////////////////
void LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_CLEAR_OBJECTS(SLuaCallBack *p)
{
    if (fusionTrackPG)
    {
        fusionTrackPG->clear_objects();
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
///  LUA function to actually start the optical tracker running.
/////////////////////////////////////////////////////////////////////////////////////////

// simExtAtracsysFusionTrackStart
void LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_START(SLuaCallBack *p)
{ // the callback function of the new Lua command ("simExtSkeleton_getSensorData")
    // return Lua Table or arrays containing position, torque, torque minus motor force, timestamp, FRI state

    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //       enum trackerOrBone {
    //         moveTracker = 0,
    //         moveBone = 1
    //       };
    //       static const trackerOrBone TruePositionsTrackerFalsePositionsBone = moveBone;
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    /// std::vector<std::string> geometryFilenames in FusionTrack::Params
    // bool areMarkerINIpathsEmpty = fusionTrackParamsG.FusionTrackParams.geometryFilenames.empty();

    // bool areMotionConfigParamsEmpty = fusionTrackParamsG.MotionConfigParamsVector.empty();

    if (!fusionTrackPG)
    {
        /// @todo currently we are using the "empty" defaults for some parameters, such as the allowed timeout for reading data
        fusionTrackPG = std::make_shared<grl::FusionTrackLogAndTrack>(fusionTrackParamsG);
        fusionTrackPG->construct();

        std::string log_message = std::string("simExtAtracsysFusionTrackStart with base object handle: ") + boost::lexical_cast<std::string>(fusionTrackParamsG.OpticalTrackerBase);
        simAddStatusbarMessage(log_message.c_str());
        loggerPG->info(log_message);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
///  LUA function to actually STOP the optical tracker running.
/////////////////////////////////////////////////////////////////////////////////////////

void LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_STOP(SLuaCallBack *p)
{
    loggerPG->info("Ending Atracsys Fusion Track Plugin connection to Optical Tracker\n");
    fusionTrackPG.reset();
}

/////////////////////////////////////////////////////////////////////////////////////////
///  LUA function to actually STOP the optical tracker running.
/////////////////////////////////////////////////////////////////////////////////////////

void LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_RESET(SLuaCallBack *p)
{
    fusionTrackPG.reset();
    LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_START(p);
}

/////////////////////////////////////////////////////////////////////////////////////////
///  LUA function to check if the fusiontrack is actively running without errors.
/////////////////////////////////////////////////////////////////////////////////////////

void LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_IS_ACTIVE(SLuaCallBack *p)
{
    CLuaFunctionData D;
    bool success = false;
    if (fusionTrackPG)
    {
        success = fusionTrackPG->is_active();
    }
    /// void CLuaFunctionData::pushOutData(const CLuaFunctionDataItem& dataItem): use this when returning data from inside of a custom Lua function call
    D.pushOutData(CLuaFunctionDataItem(success));
    D.writeDataToLua(p);
}

/////////////////////////////////////////////////////////////////////////////////////////
///  LUA function to actually start recording the fusiontrack frame data in memory.
/////////////////////////////////////////////////////////////////////////////////////////

// simExtAtracsysFusionTrackStartRecording
void LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_START_RECORDING(SLuaCallBack *p)
{
    CLuaFunctionData D;
    bool success = false;
    if (fusionTrackPG)
    {
        std::string log_message("Starting the recording of fusiontrack frame data in memory.\n");
        simAddStatusbarMessage(log_message.c_str());
        loggerPG->info(log_message);
        success = fusionTrackPG->start_recording();
    }
    D.pushOutData(CLuaFunctionDataItem(success));
    D.writeDataToLua(p);
}
/////////////////////////////////////////////////////////////////////////////////////////
///  LUA function to actually stop recording the fusiontrack frame data in memory.
/////////////////////////////////////////////////////////////////////////////////////////

void LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_STOP_RECORDING(SLuaCallBack *p)
{
    CLuaFunctionData D;
    bool success = false;
    if (fusionTrackPG)
    {
        std::string log_message("Stopping the recording of fusiontrack frame data in memory.\n");
        simAddStatusbarMessage(log_message.c_str());
        loggerPG->info(log_message);
        success = fusionTrackPG->stop_recording();
    }
    D.pushOutData(CLuaFunctionDataItem(success));
    D.writeDataToLua(p);
}
/////////////////////////////////////////////////////////////////////////////////////////
///  LUA function to actually clear recording the fusiontrack frame data in memory.
/////////////////////////////////////////////////////////////////////////////////////////

void LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_CLEAR_RECORDING(SLuaCallBack *p)
{
    CLuaFunctionData D;
    bool success = false;
    if (fusionTrackPG)
    {
        std::string log_message("Clearing the recorded fusiontrack frame data from memory.\n");
        simAddStatusbarMessage(log_message.c_str());
        loggerPG->info(log_message);
        fusionTrackPG->clear_recording();
        success = true;
    }
    D.pushOutData(CLuaFunctionDataItem(success));
    D.writeDataToLua(p);
}
/////////////////////////////////////////////////////////////////////////////
//   Save the currently recorded fusiontrack frame data, this also clears the recording.
/////////////////////////////////////////////////////////////////////////////

#define LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_SAVE_RECORDING_COMMAND "simExtAtracsysFusionTrackSaveRecording"

const int inArgs_LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_SAVE_RECORDING[] = {
    1,
    sim_lua_arg_string, 0 // string file name
};

std::string LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_SAVE_RECORDING_CALL_TIP("number result=simExtAtracsysFusionTrackSaveRecording(string filename)");

void LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_SAVE_RECORDING(SLuaCallBack *p)
{

    CLuaFunctionData D;
    bool success = false;
    if (D.readDataFromLua(p, inArgs_LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_SAVE_RECORDING, inArgs_LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_SAVE_RECORDING[0], LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_SAVE_RECORDING_COMMAND))
    {
        std::vector<CLuaFunctionDataItem> *inData = D.getInDataPtr();
        std::string filename_lua_param(inData->at(0).stringData[0]);

        if (fusionTrackPG)
        {
            std::string log_message("Saving the currently recorded fusiontrack frame data to a file.\n");
            simAddStatusbarMessage(log_message.c_str());
            loggerPG->info(log_message);
            success = fusionTrackPG->save_recording(filename_lua_param);
        }
        D.pushOutData(CLuaFunctionDataItem(success));
        D.writeDataToLua(p);
    }
}


/////////////////////////////////////////////////////////////////////////////
//   Set recordWhileSimulationIsRunningG
/////////////////////////////////////////////////////////////////////////////

#define LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_RECORD_WHILE_SIMULATION_IS_RUNNING_COMMAND "simExtAtracsysFusionTrackRecordWhileSimulationIsRunning"

const int inArgs_LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_RECORD_WHILE_SIMULATION_IS_RUNNING[] = {
    1,
    sim_lua_arg_bool, 1 // string file name
};

std::string LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_RECORD_WHILE_SIMULATION_IS_RUNNING_CALL_TIP("number result=simExtAtracsysFusionTrackRecordWhileSimulationIsRunning(bool recording)");

void LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_RECORD_WHILE_SIMULATION_IS_RUNNING(SLuaCallBack *p)
{

    CLuaFunctionData D;
    bool success = false;
    if (D.readDataFromLua(p,
        inArgs_LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_RECORD_WHILE_SIMULATION_IS_RUNNING,
        inArgs_LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_RECORD_WHILE_SIMULATION_IS_RUNNING[0],
        LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_RECORD_WHILE_SIMULATION_IS_RUNNING_COMMAND))
    {
        std::vector<CLuaFunctionDataItem> *inData = D.getInDataPtr();
        recordWhileSimulationIsRunningG = inData->at(0).boolData[0];

        std::string log_message = std::string("simExtAtracsysFusionTrackRecordWhileSimulationIsRunning: ") + boost::lexical_cast<std::string>(recordWhileSimulationIsRunningG);
        simAddStatusbarMessage(log_message.c_str());
        loggerPG->info(log_message);

        D.pushOutData(CLuaFunctionDataItem(success));
        D.writeDataToLua(p);
    }
}


/// v-rep plugin: http://www.coppeliarobotics.com/helpFiles/en/plugins.htm
/// This is the plugin start routine (called just once, just after the plugin was loaded):
VREP_DLLEXPORT unsigned char v_repStart(void *reservedPointer, int reservedInt)
{
    try
    {
        loggerPG = spdlog::stdout_logger_mt("console");
    }
    catch (spdlog::spdlog_ex ex)
    {
        loggerPG = spdlog::get("console");
    }
    // Dynamically load and bind V-REP functions:
    // ******************************************
    // 1. Figure out this plugin's directory:
    char curDirAndFile[1024];
#ifdef _WIN32
    GetModuleFileName(NULL, curDirAndFile, 1023);
    PathRemoveFileSpec(curDirAndFile);
#elif defined(__linux) || defined(__APPLE__)
    getcwd(curDirAndFile, sizeof(curDirAndFile));
#endif
    std::string currentDirAndPath(curDirAndFile);
    // 2. Append the V-REP library's name:
    std::string temp(currentDirAndPath);
#ifdef _WIN32
    temp += "\\v_rep.dll";
#elif defined(__linux)
    temp += "/libv_rep.so";
#elif defined(__APPLE__)
    temp += "/libv_rep.dylib";
#endif /* __linux || __APPLE__ */
    // 3. Load the V-REP library:
    vrepLib = loadVrepLibrary(temp.c_str());
    if (vrepLib == NULL)
    {
        loggerPG->error("Error, could not find or correctly load the V-REP library. Cannot start 'AtracsysFusionTrack' plugin.\n");
        return (0); // Means error, V-REP will unload this plugin
    }
    if (getVrepProcAddresses(vrepLib) == 0)
    {
        loggerPG->error("Error, could not find all required functions in the V-REP library. Cannot start 'AtracsysFusionTrack' plugin.\n");
        unloadVrepLibrary(vrepLib);
        return (0); // Means error, V-REP will unload this plugin
    }
    // ******************************************

    // Check the version of V-REP:
    // ******************************************
    int vrepVer;
    simGetIntegerParameter(sim_intparam_program_version, &vrepVer);
    if (vrepVer < 20604) // if V-REP version is smaller than 2.06.04
    {
        loggerPG->error("Sorry, your V-REP copy is somewhat old. Cannot start 'AtracsysFusionTrack' plugin.\n");
        unloadVrepLibrary(vrepLib);
        return (0); // Means error, V-REP will unload this plugin
    }
    // ******************************************

    // Register the new Lua command "simExtSkeleton_getSensorData":
    // ******************************************
    // Expected input arguments are: int sensorIndex, float floatParameters[3], int intParameters[2]
    // int inArgs_getSensorData[]={3,sim_lua_arg_int,sim_lua_arg_float|sim_lua_arg_table,sim_lua_arg_int|sim_lua_arg_table}; // this says we expect 3 arguments (1 integer, a table of floats, and a table of ints)
    // Return value can change on the fly, so no need to specify them here, except for the calltip.
    // Now register the callback:
    // simRegisterCustomLuaFunction(LUA_GET_SENSOR_DATA_COMMAND,strConCat("number result,table data,number distance=",LUA_GET_SENSOR_DATA_COMMAND,"(number sensorIndex,table_3 floatParams,table_2 intParams)"),inArgs_getSensorData,LUA_GET_SENSOR_DATA_CALLBACK);

    std::vector<int> inArgs;
    /// Register lua callbacks
    CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_ADD_GEOMETRY, inArgs);
    simRegisterCustomLuaFunction(LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_ADD_GEOMETRY_COMMAND,
                                 LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_ADD_GEOMETRY_CALL_TIP.c_str(),
                                 &inArgs[0],
                                 LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_ADD_GEOMETRY);

    CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_SET_OPTICAL_TRACKER_BASE, inArgs);
    simRegisterCustomLuaFunction(LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_SET_OPTICAL_TRACKER_BASE_COMMAND,
                                 LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_SET_OPTICAL_TRACKER_BASE_CALL_TIP.c_str(),
                                 &inArgs[0],
                                 LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_SET_OPTICAL_TRACKER_BASE);

    CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_OBJECT, inArgs);
    simRegisterCustomLuaFunction(LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_OBJECT_COMMAND,
                                 LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_OBJECT_CALL_TIP.c_str(),
                                 &inArgs[0],
                                 LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_OBJECT);

    CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_STOP_TRACKING_OBJECT_WITH_GEOMETRY_ID, inArgs);
    simRegisterCustomLuaFunction(LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_STOP_TRACKING_OBJECT_WITH_GEOMETRY_ID_COMMAND,
                                 LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_STOP_TRACKING_OBJECT_WITH_GEOMETRY_ID_CALL_TIP.c_str(),
                                 &inArgs[0],
                                 LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_STOP_TRACKING_OBJECT_WITH_GEOMETRY_ID);
    /// Register functions to control the status of FusionTrack
    int inArgs1[] = {0}; // no input arguments
    simRegisterCustomLuaFunction("simExtAtracsysFusionTrackStart", "number result=simExtAtracsysFusionTrackStart()", inArgs1, LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_START);
    simRegisterCustomLuaFunction("simExtAtracsysFusionTrackStop", "number result=simExtAtracsysFusionTrackStop()", inArgs1, LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_STOP);
    simRegisterCustomLuaFunction("simExtAtracsysFusionTrackReset", "number result=simExtAtracsysFusionTrackReset()", inArgs1, LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_RESET);
    simRegisterCustomLuaFunction("simExtAtracsysFusionTrackIsActive", "number result=simExtAtracsysFusionTrackIsActive()", inArgs1, LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_IS_ACTIVE);
    simRegisterCustomLuaFunction("simExtAtracsysFusionTrackClearObjects", "number result=simExtAtracsysFusionTrackClearObjects()", inArgs1, LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_CLEAR_OBJECTS);

    /// Register functions to control the recording procedure of the fusiontrack frame data in memory
    simRegisterCustomLuaFunction("simExtAtracsysFusionTrackStartRecording", "number result=simExtAtracsysFusionTrackStartRecording()", inArgs1, LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_START_RECORDING);
    simRegisterCustomLuaFunction("simExtAtracsysFusionTrackStopRecording", "number result=simExtAtracsysFusionTrackStopRecording()", inArgs1, LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_STOP_RECORDING);
    simRegisterCustomLuaFunction("simExtAtracsysFusionTrackClearRecording", "number result=simExtAtracsysFusionTrackClearRecording()", inArgs1, LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_CLEAR_RECORDING);

    CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_SAVE_RECORDING, inArgs);
    simRegisterCustomLuaFunction(LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_SAVE_RECORDING_COMMAND,
                                 LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_SAVE_RECORDING_CALL_TIP.c_str(),
                                 &inArgs[0],
                                 LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_SAVE_RECORDING);


    CLuaFunctionData::getInputDataForFunctionRegistration(inArgs_LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_RECORD_WHILE_SIMULATION_IS_RUNNING, inArgs);
    simRegisterCustomLuaFunction(LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_RECORD_WHILE_SIMULATION_IS_RUNNING_COMMAND,
        LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_RECORD_WHILE_SIMULATION_IS_RUNNING_CALL_TIP.c_str(),
                              &inArgs[0],
                              LUA_SIM_EXT_ATRACSYS_FUSION_TRACK_RECORD_WHILE_SIMULATION_IS_RUNNING);

    // ******************************************
    /// Print to terminal
    /// __DATE__ and __TIME__ haven't been defined yet
    loggerPG->info("Atracsys Fusion Track plugin initialized. Build date/time: ", __DATE__, " ", __TIME__);

    return (PLUGIN_VERSION); // initialization went fine, we return the version number of this plugin (can be queried with simGetModuleName)
}

// This is the plugin end routine (called just once, when V-REP is ending, i.e. releasing this plugin):
VREP_DLLEXPORT void v_repEnd()
{
    // Here you could handle various clean-up tasks

    /////////////////////////
    // PUT OBJECT RESET CODE HERE
    // close out as necessary
    ////////////////////
    loggerPG->info("Ending Atracsys Fusion Track Plugin connection to Optical Tracker\n");
    fusionTrackPG.reset();

    unloadVrepLibrary(vrepLib); // release the library
}

// This is the plugin messaging routine (i.e. V-REP calls this function very often, with various messages):
// This is called quite often. Just watch out for messages/events you want to handle
VREP_DLLEXPORT void *v_repMessage(int message, int *auxiliaryData, void *customData, int *replyData)
{
    // Keep following 5 lines at the beginning and unchanged:
    static bool refreshDlgFlag = true;
    int errorModeSaved;
    simGetIntegerParameter(sim_intparam_error_report_mode, &errorModeSaved);
    simSetIntegerParameter(sim_intparam_error_report_mode, sim_api_errormessage_ignore);
    void *retVal = NULL;

    // Here we can intercept many messages from V-REP (actually callbacks). Only the most important messages are listed here.
    // For a complete list of messages that you can intercept/react with, search for "sim_message_eventcallback"-type constants
    // in the V-REP user manual.
    // http://www.coppeliarobotics.com/helpFiles/en/apiConstants.htm#simulatorMessages

    if (message == sim_message_eventcallback_refreshdialogs)
        refreshDlgFlag = true; // V-REP dialogs were refreshed. Maybe a good idea to refresh this plugin's dialog too

    if (message == sim_message_eventcallback_menuitemselected)
    {   // A custom menu bar entry was selected..
        // here you could make a plugin's main dialog visible/invisible
    }

    if (message == sim_message_eventcallback_instancepass)
    { // This message is sent each time the scene was rendered (well, shortly after) (very often)
        // It is important to always correctly react to events in V-REP. This message is the most convenient way to do so:

        int flags = auxiliaryData[0];
        bool sceneContentChanged = ((flags & (1 + 2 + 4 + 8 + 16 + 32 + 64 + 256)) != 0); // object erased, created, model or scene loaded, und/redo called, instance switched, or object scaled since last sim_message_eventcallback_instancepass message
        bool instanceSwitched = ((flags & 64) != 0);

        if (instanceSwitched)
        {
            // React to an instance switch here!!
        }

        if (sceneContentChanged)
        {						   // we actualize plugin objects for changes in the scene
            refreshDlgFlag = true; // always a good idea to trigger a refresh of this plugin's dialog here
        }

        //...
        //////////////
        // PUT MAIN CODE HERE

        /////////////
        if (simGetSimulationState() != sim_simulation_advancing_abouttostop) //checks if the simulation is still running
        {
            //if(fusionTrackPG) BOOST_LOG_TRIVIAL(info) << "current simulation time:" << simGetSimulationTime() << std::endl;					// gets simulation time point
        }
        // make sure it is "right" (what does that mean?)

        // find the v-rep C functions to do the following:
        ////////////////////////////////////////////////////
        // Use handles that were found at the "start" of this simulation running

        // next few Lines get the joint angles, torque, etc from the simulation
        if (fusionTrackPG && fusionTrackPG->is_active()) // && fusionTrackPG->allHandlesSet == true // allHandlesSet now handled internally
        {

            // run one loop synchronizing the tracker, plugin, and simulation
            try
            {
                // run_one
                std::vector<std::tuple<int, int, Eigen::Affine3f>> transforms = fusionTrackPG->get_poses();
                // loggerPG->info("Count of poses to update: " + boost::lexical_cast<std::string>(transforms.size()));
                for(std::tuple<int, int, Eigen::Affine3f>& transform : transforms)
                {
                    // 2 is the index in the tuple of the transform between the frames
                    Eigen::Affine3f pose = std::get<2>(transform);
                    // grl::FusionTrackLogAndTrack::MotionConfigParamsIndex::ObjectToMove
                    int objectToMove = std::get<grl::FusionTrackLogAndTrack::MotionConfigParamsIndex::ObjectToMove>(transform);
                    int frameInWhichToMoveObject = std::get<grl::FusionTrackLogAndTrack::MotionConfigParamsIndex::FrameInWhichToMoveObject>(transform);
                    // setObjectTransform sets vrep object poses from eigen transformsin grl/vrep/Eigen.hpp
                    setObjectTransform(objectToMove,
                                       frameInWhichToMoveObject,
                                       pose);
                }
            }
            catch (const boost::exception &e)
            {
                std::string initerr("v_repExtAtracsysFusionTrack plugin encountered the following error and will disable itself:\n" + boost::diagnostic_information(e));
                simAddStatusbarMessage(initerr.c_str());
                loggerPG->error(initerr);
                fusionTrackPG.reset();
            }
        }
    }

    if (message == sim_message_eventcallback_mainscriptabouttobecalled)
    { // The main script is about to be run (only called while a simulation is running (and not paused!))
    }

    if (message == sim_message_eventcallback_simulationabouttostart)
    { // Simulation is about to start

        /////////////////////////
        // PUT OBJECT STARTUP CODE HERE
        ////////////////////
        // get the handles to all the objects, joints, etc that we need
        /////////////////////
        // simGetObjectHandle

        //        try {
        //            //BOOST_LOG_TRIVIAL(info) << "Starting KUKA LBR iiwa plugin connection to Kuka iiwa\n";
        //            //fusionTrackPG = std::make_shared<grl::HandEyeCalibrationVrepPlugin>();
        //            //fusionTrackPG->construct();
        //            //fusionTrackPG->run_one();  // for debugging purposes only
        //            //fusionTrackPG.reset();     // for debugging purposes only
        //        } catch (boost::exception& e){
        //            // log the error and print it to the screen, don't release the exception
        //            std::string initerr("v_repExtKukaLBRiiwa plugin initialization error:\n" + boost::diagnostic_information(e));
        //            simAddStatusbarMessage( initerr.c_str());
        //            LoggerPG->error( initerr);
        //        }

        if(fusionTrackPG && recordWhileSimulationIsRunningG) {
            fusionTrackPG->start_recording();
        }
    }

    if (message == sim_message_eventcallback_simulationended)
    { // Simulation just ended

        /////////////////////////
        // SIMULATION STOPS RUNNING HERE
        // close out as necessary
        ////////////////////
        if(fusionTrackPG && recordWhileSimulationIsRunningG && fusionTrackPG->is_recording()) {
            fusionTrackPG->save_recording();
            fusionTrackPG->stop_recording();
        }
    }

    if (message == sim_message_eventcallback_moduleopen)
    {																							// A script called simOpenModule (by default the main script). Is only called during simulation.
        if ((customData == NULL) || (_stricmp("AtracsysFusionTrack", (char *)customData) == 0)) // is the command also meant for this plugin?
        {
            // we arrive here only at the beginning of a simulation
        }
    }

    if (message == sim_message_eventcallback_modulehandle)
    {																							// A script called simHandleModule (by default the main script). Is only called during simulation.
        if ((customData == NULL) || (_stricmp("AtracsysFusionTrack", (char *)customData) == 0)) // is the command also meant for this plugin?
        {
            // we arrive here only while a simulation is running
        }
    }

    if (message == sim_message_eventcallback_moduleclose)
    {																							// A script called simCloseModule (by default the main script). Is only called during simulation.
        if ((customData == NULL) || (_stricmp("AtracsysFusionTrack", (char *)customData) == 0)) // is the command also meant for this plugin?
        {
            // we arrive here only at the end of a simulation
        }
    }

    if (message == sim_message_eventcallback_instanceswitch)
    {   // Here the user switched the scene. React to this message in a similar way as you would react to a full
        // scene content change. In this plugin example, we react to an instance switch by reacting to the
        // sim_message_eventcallback_instancepass message and checking the bit 6 (64) of the auxiliaryData[0]
        // (see here above)
    }

    if (message == sim_message_eventcallback_broadcast)
    { // Here we have a plugin that is broadcasting data (the broadcaster will also receive this data!)
    }

    if (message == sim_message_eventcallback_scenesave)
    { // The scene is about to be saved. If required do some processing here (e.g. add custom scene data to be serialized with the scene)
    }

    // You can add many more messages to handle here

    if ((message == sim_message_eventcallback_guipass) && refreshDlgFlag)
    { // handle refresh of the plugin's dialogs
        // ...
        refreshDlgFlag = false;
    }

    // Keep following unchanged:
    simSetIntegerParameter(sim_intparam_error_report_mode, errorModeSaved); // restore previous settings
    return (retVal);
}
