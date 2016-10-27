// Copyright 2006-2016 Coppelia Robotics GmbH. All rights reserved. 
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
// This file was automatically created for V-REP release V3.3.2 on August 29th 2016

#pragma once

#include "scriptFunctionDataItem.h"
#include "v_repLib.h"

#define SIM_SCRIPT_ARG_NULL_ALLOWED (65536)

class CScriptFunctionData  
{
public:
    CScriptFunctionData();
    virtual ~CScriptFunctionData();

    //------------------------
    // Use following to read args coming from a script call to this plugin, and to return args back to a script
    bool readDataFromStack(int stackHandle,const int* expectedArguments,int requiredArgumentCount,const char* functionName);
    std::vector<CScriptFunctionDataItem>* getInDataPtr();
    void pushOutData(const CScriptFunctionDataItem& dataItem);
    void writeDataToStack(int stackHandle);
    //------------------------


    //------------------------
    // Use following to write args for a script function call, and to read the return values from that script function call
    void pushOutData_scriptFunctionCall(const CScriptFunctionDataItem& dataItem);
    void writeDataToStack_scriptFunctionCall(int stackHandle);
    bool readDataFromStack_scriptFunctionCall(int stackHandle,const int* expectedArguments,int requiredArgumentCount,const char* functionName);
    std::vector<CScriptFunctionDataItem>* getOutDataPtr_scriptFunctionCall();
    //------------------------

protected:
    bool _readData(int stack,const int* expectedArguments,int requiredArgumentCount,const char* functionName,const char* argumentText1,const char* argumentText2,std::vector<CScriptFunctionDataItem>& inOutData);
    void _writeData(int stack,std::vector<CScriptFunctionDataItem>& inOutData);

    std::vector<CScriptFunctionDataItem> _inData;
    std::vector<CScriptFunctionDataItem> _outData;
};
