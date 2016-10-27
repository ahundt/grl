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

#include "scriptFunctionData.h"
#include <sstream>
#include <cstring>

CScriptFunctionData::CScriptFunctionData()
{
}

CScriptFunctionData::~CScriptFunctionData()
{
}

std::vector<CScriptFunctionDataItem>* CScriptFunctionData::getInDataPtr()
{ // use this when reading data from a script from inside of a custom script function call
    return(&_inData);
}

std::vector<CScriptFunctionDataItem>* CScriptFunctionData::getOutDataPtr_scriptFunctionCall()
{ // use this when reading data returned from a script function call from a plugin
    return(&_outData);
}

bool CScriptFunctionData::readDataFromStack(int stackHandle,const int* expectedArguments,int requiredArgumentCount,const char* functionName)
{  // use this when reading data from a script from inside of a custom script function call
    bool retVal=_readData(stackHandle,expectedArguments,requiredArgumentCount,functionName,"arguments.","Argument ",_inData);
    simPopStackItem(stackHandle,0); // clears the stack (which will serve as return value container from now)
    return(retVal);
}

bool CScriptFunctionData::readDataFromStack_scriptFunctionCall(int stackHandle,const int* expectedArguments,int requiredArgumentCount,const char* functionName)
{   // use this when reading data returned from a Lua function call from a plugin
    bool retVal=_readData(stackHandle,expectedArguments,requiredArgumentCount,functionName,"return arguments.","Return argument ",_outData);
    simPopStackItem(stackHandle,0); // clears the stack
    return(retVal);
}

void CScriptFunctionData::pushOutData(const CScriptFunctionDataItem& dataItem)
{ // use this when returning data from inside of a custom Lua function call
    _outData.push_back(dataItem);
}

void CScriptFunctionData::pushOutData_scriptFunctionCall(const CScriptFunctionDataItem& dataItem)
{ // use this when doing a Lua function call from a plugin
    _inData.push_back(dataItem);
}

void CScriptFunctionData::writeDataToStack(int stackHandle)
{   // use this when returning data from inside of a custom Lua function call
    _writeData(stackHandle,_outData);
}

void CScriptFunctionData::writeDataToStack_scriptFunctionCall(int stackHandle)
{ // use this when doing a Lua function call from a plugin
    _writeData(stackHandle,_inData);
}

bool CScriptFunctionData::_readData(int stack,const int* expectedArguments,int requiredArgumentCount,const char* functionName,const char* argumentText1,const char* argumentText2,std::vector<CScriptFunctionDataItem>& inOutData)
{  // use this when reading data from a script from inside of a custom script function call
    inOutData.clear();
    int argCnt=simGetStackSize(stack);
    if (argCnt<requiredArgumentCount)
    {
        std::ostringstream str;
        str << "Not enough " << argumentText1;
        simSetLastError(functionName,str.str().c_str());
        return(false);
    }

    for (int i=0;i<argCnt;i++)
    {
        if (i>=expectedArguments[0])
            break;
        bool done=false;
        simMoveStackItemToTop(stack,0);
        if (simIsStackValueNull(stack)==1)
        {
            // is nil explicitely allowed?
            if (expectedArguments[1+i*2+0]&SIM_SCRIPT_ARG_NULL_ALLOWED)
            { // yes. This is for an argument that can optionally also be nil.
                CScriptFunctionDataItem dat;
                inOutData.push_back(dat);
                done=true;
            }
            else
            { // no
                if (int(inOutData.size())<requiredArgumentCount)
                {
                    std::ostringstream str;
                    str << argumentText2 << i+1 << " is not correct.";
                    simSetLastError(functionName,str.str().c_str());
                    return(false);
                }
                break; // this argument is nil, so it is like inexistant. But we also won't explore any more arguments, we have enough.
            }
        }
        if (!done)
        {
            int tableSize=simGetStackTableInfo(stack,0);
            bool expectingATable=(((expectedArguments[1+i*2+0]|SIM_SCRIPT_ARG_NULL_ALLOWED)-SIM_SCRIPT_ARG_NULL_ALLOWED)&sim_script_arg_table)!=0;
            if ( (tableSize>=0)!=expectingATable )
            {
                std::ostringstream str;
                str << argumentText2 << i+1 << " is not correct.";
                simSetLastError(functionName,str.str().c_str());
                return(false);
            }
            int expectingType=(((expectedArguments[1+i*2+0]|SIM_SCRIPT_ARG_NULL_ALLOWED)-SIM_SCRIPT_ARG_NULL_ALLOWED)|sim_script_arg_table)-sim_script_arg_table;
            if (expectingATable)
            { // we have a table
                int infoType=0;
                if (expectingType==sim_script_arg_null)
                    infoType=1;
                if (expectingType==sim_script_arg_bool)
                    infoType=3;
                if (expectingType==sim_script_arg_float)
                    infoType=2;
                if (expectingType==sim_script_arg_int32)
                    infoType=2;
                if (expectingType==sim_script_arg_string)
                    infoType=4;
                if (expectingType==sim_script_arg_charbuff)
                    infoType=4;
                if (expectingType==sim_script_arg_double)
                    infoType=2;
                if (simGetStackTableInfo(stack,infoType)<1)
                { // table content cannot be converted
                    std::ostringstream str;
                    str << argumentText2 << i+1 << " is not correct.";
                    simSetLastError(functionName,str.str().c_str());
                    return(false);
                }

                if ( (tableSize<expectedArguments[1+i*2+1])&&(expectedArguments[1+i*2+1]!=0) )
                {
                    std::ostringstream str;
                    str << argumentText2 << i+1 << " is not correct (wrong table size).";
                    simSetLastError(functionName,str.str().c_str());
                    return(false);
                }
                else
                {
                    int t=expectingType;
                    int itemCnt=tableSize;

                    if (t==sim_script_arg_null)
                    {
                        CScriptFunctionDataItem* a=new CScriptFunctionDataItem();
                        a->setNilTable(itemCnt);
                        CScriptFunctionDataItem dat;
                        dat.setNilTable(itemCnt);
                        inOutData.push_back(dat);
                    }
                    if (t==sim_script_arg_bool)
                    {
                        std::vector<bool> vect;
                        simUnfoldStackTable(stack); // this removes the table and exposes the inside
                        for (int j=0;j<itemCnt;j++)
                        {
                            simBool val;
                            simGetStackBoolValue(stack,&val);
                            vect.insert(vect.begin(),val!=0);
                            simPopStackItem(stack,2);
                        }
                        simPushTableOntoStack(stack); // empty table, will be removed at the end of the loop
                        CScriptFunctionDataItem dat(vect);
                        inOutData.push_back(dat);
                    }
                    if (t==sim_script_arg_int32)
                    {
                        std::vector<int> vect;
                        if (itemCnt>0)
                        {
                            vect.resize(itemCnt);
                            simGetStackInt32Table(stack,&vect[0],itemCnt);
                        }
                        CScriptFunctionDataItem dat(vect);
                        inOutData.push_back(dat);
                    }
                    if (t==sim_script_arg_float)
                    {
                        std::vector<float> vect;
                        if (itemCnt>0)
                        {
                            vect.resize(itemCnt);
                            simGetStackFloatTable(stack,&vect[0],itemCnt);
                        }
                        CScriptFunctionDataItem dat(vect);
                        inOutData.push_back(dat);
                    }
                    if (t==sim_script_arg_double)
                    {
                        std::vector<double> vect;
                        if (itemCnt>0)
                        {
                            vect.resize(itemCnt);
                            simGetStackDoubleTable(stack,&vect[0],itemCnt);
                        }
                        CScriptFunctionDataItem dat(vect);
                        inOutData.push_back(dat);
                    }
                    if (t==sim_script_arg_string)
                    {
                        std::vector<std::string> vect;
                        simUnfoldStackTable(stack); // this removes the table and exposes the inside
                        for (int j=0;j<itemCnt;j++)
                        {
                            int l;
                            char* str=simGetStackStringValue(stack,&l);
                            std::string str2(str); // treat it as a char string, not buffer
                            simReleaseBuffer(str);
                            vect.insert(vect.begin(),str2);
                            simPopStackItem(stack,2);
                        }
                        simPushTableOntoStack(stack); // empty table, will be removed at the end of the loop
                        CScriptFunctionDataItem dat(vect);
                        inOutData.push_back(dat);
                    }
                    if (t==sim_script_arg_charbuff)
                    {
                        std::ostringstream str;
                        str << argumentText2 << i+1 << " cannot be a table.";
                        simSetLastError(functionName,str.str().c_str());
                        return(false);
                    }
                }
            }
            else
            { // we do not have a table
                int t=expectingType;
                bool failedMsgAndLeave=false;
                if (t==sim_script_arg_null)
                {
                    if (simIsStackValueNull(stack)>0)
                    {
                        CScriptFunctionDataItem dat;
                        inOutData.push_back(dat);
                    }
                    else
                        failedMsgAndLeave=true;
                }
                if (t==sim_script_arg_bool)
                {
                    simBool val=0;
                    if (simGetStackBoolValue(stack,&val)==1)
                    {
                        CScriptFunctionDataItem dat(val!=0);
                        inOutData.push_back(dat);
                    }
                    else
                        failedMsgAndLeave=true;
                }
                if (t==sim_script_arg_int32)
                {
                    int val=0;
                    if (simGetStackInt32Value(stack,&val)==1)
                    {
                        CScriptFunctionDataItem dat(val);
                        inOutData.push_back(dat);
                    }
                    else
                        failedMsgAndLeave=true;
                }
                if (t==sim_script_arg_float)
                {
                    float val=0.0;
                    if (simGetStackFloatValue(stack,&val)==1)
                    {
                        CScriptFunctionDataItem dat(val);
                        inOutData.push_back(dat);
                    }
                    else
                        failedMsgAndLeave=true;
                }
                if (t==sim_script_arg_double)
                {
                    double val=0.0;
                    if (simGetStackDoubleValue(stack,&val)==1)
                    {
                        CScriptFunctionDataItem dat(val);
                        inOutData.push_back(dat);
                    }
                    else
                        failedMsgAndLeave=true;
                }
                if (t==sim_script_arg_string)
                {
                    int l;
                    char* str=simGetStackStringValue(stack,&l);
                    if (str!=NULL)
                    {
                        std::string str2(str);
                        simReleaseBuffer(str);
                        CScriptFunctionDataItem dat(str2); // treat it as a char string, not buffer
                        inOutData.push_back(dat);
                    }
                    else
                        failedMsgAndLeave=true;
                }
                if (t==sim_script_arg_charbuff)
                {
                    int l;
                    char* str=simGetStackStringValue(stack,&l);
                    if (str!=NULL)
                    {
                        if ( (l<expectedArguments[1+i*2+1])&&(expectedArguments[1+i*2+1]!=0) )
                        {
                            simReleaseBuffer(str);
                            std::ostringstream str;
                            str << argumentText2 << i+1 << " is not correct (wrong buffer size).";
                            simSetLastError(functionName,str.str().c_str());
                            return(false);
                        }
                        else
                        {
                            CScriptFunctionDataItem dat(str,l);
                            inOutData.push_back(dat);
                            simReleaseBuffer(str);
                        }
                    }
                    else
                        failedMsgAndLeave=true;
                }
                if (failedMsgAndLeave)
                {
                    std::ostringstream str;
                    str << argumentText2 << i+1 << " is not correct.";
                    simSetLastError(functionName,str.str().c_str());
                    return(false);
                }
            }
        }
        simPopStackItem(stack,1);
    }
    return(true);
}

void CScriptFunctionData::_writeData(int stack,std::vector<CScriptFunctionDataItem>& inOutData)
{
    simPopStackItem(stack,0); // Clear the stack

    int itemCnt=int(inOutData.size());

    for (int i=0;i<itemCnt;i++)
    {
        if (inOutData[i].isTable())
        { // table
            if (inOutData[i].getType()==-1)
            { // nil table
                simPushTableOntoStack(stack);
                for (int j=0;j<inOutData[i].getNilTableSize();j++)
                {
                    simPushInt32OntoStack(stack,j+1); // the key
                    simPushNullOntoStack(stack); // the value
                    simInsertDataIntoStackTable(stack);
                }
            }
            if (inOutData[i].getType()==0)
            { // bool table
                simPushTableOntoStack(stack);
                for (size_t j=0;j<inOutData[i].boolData.size();j++)
                {
                    simPushInt32OntoStack(stack,(int)j+1); // the key
                    simPushBoolOntoStack(stack,inOutData[i].boolData[j]); // the value
                    simInsertDataIntoStackTable(stack);
                }
            }
            if (inOutData[i].getType()==1)
            { // int table
                if (inOutData[i].int32Data.size()>0)
                    simPushInt32TableOntoStack(stack,&inOutData[i].int32Data[0],int(inOutData[i].int32Data.size()));
                else
                    simPushTableOntoStack(stack);
            }
            if (inOutData[i].getType()==2)
            { // float table
                if (inOutData[i].floatData.size()>0)
                    simPushFloatTableOntoStack(stack,&inOutData[i].floatData[0],int(inOutData[i].floatData.size()));
                else
                    simPushTableOntoStack(stack);
            }
            if (inOutData[i].getType()==5)
            { // double table
                if (inOutData[i].doubleData.size()>0)
                    simPushDoubleTableOntoStack(stack,&inOutData[i].doubleData[0],int(inOutData[i].doubleData.size()));
                else
                    simPushTableOntoStack(stack);
            }
            if (inOutData[i].getType()==3)
            { // string table
                simPushTableOntoStack(stack);
                for (size_t j=0;j<inOutData[i].stringData.size();j++)
                {
                    simPushInt32OntoStack(stack,(int)j+1); // the key
                    simPushStringOntoStack(stack,inOutData[i].stringData[j].c_str(),(int)inOutData[i].stringData[j].length()); // the value
                    simInsertDataIntoStackTable(stack);
                }
            }
        }
        else
        { // non-table values:
            if (inOutData[i].getType()==-1)
                simPushNullOntoStack(stack);
            if (inOutData[i].getType()==0)
                simPushBoolOntoStack(stack,inOutData[i].boolData[0]);
            if (inOutData[i].getType()==1)
                simPushInt32OntoStack(stack,inOutData[i].int32Data[0]);
            if (inOutData[i].getType()==2)
                simPushFloatOntoStack(stack,inOutData[i].floatData[0]);
            if (inOutData[i].getType()==5)
                simPushDoubleOntoStack(stack,inOutData[i].doubleData[0]);
            if ( (inOutData[i].getType()==3)||(inOutData[i].getType()==4) )
                simPushStringOntoStack(stack,inOutData[i].stringData[0].c_str(),(int)inOutData[i].stringData[0].length());
        }
    }
}
