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

#include <vector>
#include <string>

class CScriptFunctionDataItem
{
public:
    CScriptFunctionDataItem();
    CScriptFunctionDataItem(bool v);
    CScriptFunctionDataItem(int v);
    CScriptFunctionDataItem(float v);
    CScriptFunctionDataItem(double v);
    CScriptFunctionDataItem(const std::string& str);
    CScriptFunctionDataItem(const char* str);
    CScriptFunctionDataItem(const char* bufferPtr,unsigned int bufferLength);

    CScriptFunctionDataItem(const std::vector<bool>& v);
    CScriptFunctionDataItem(const std::vector<int>& v);
    CScriptFunctionDataItem(const std::vector<float>& v);
    CScriptFunctionDataItem(const std::vector<double>& v);
    CScriptFunctionDataItem(const std::vector<std::string>& v);

    virtual ~CScriptFunctionDataItem();

    bool isTable();
    int getType();
    void setNilTable(int size);
    int getNilTableSize();

    std::vector<bool> boolData;
    std::vector<int> int32Data;
    std::vector<float> floatData;
    std::vector<double> doubleData;
    std::vector<std::string> stringData;

protected:
    int _nilTableSize;
    bool _isTable;
    int _type; // -1=nil,0=bool,1=int32,2=float,3=string,4=buffer,5=double
};
