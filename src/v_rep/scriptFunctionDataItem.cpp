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

#include "scriptFunctionDataItem.h"

CScriptFunctionDataItem::CScriptFunctionDataItem()
{
    _nilTableSize=0;
    _isTable=false;
    _type=-1; // nil
}

CScriptFunctionDataItem::CScriptFunctionDataItem(bool v)
{
    _nilTableSize=0;
    _isTable=false;
    _type=0;
    boolData.push_back(v);
}

CScriptFunctionDataItem::CScriptFunctionDataItem(int v)
{
    _nilTableSize=0;
    _isTable=false;
    _type=1;
    int32Data.push_back(v);
}

CScriptFunctionDataItem::CScriptFunctionDataItem(float v)
{
    _nilTableSize=0;
    _isTable=false;
    _type=2;
    floatData.push_back(v);
}

CScriptFunctionDataItem::CScriptFunctionDataItem(double v)
{
    _nilTableSize=0;
    _isTable=false;
    _type=5;
    doubleData.push_back(v);
}

CScriptFunctionDataItem::CScriptFunctionDataItem(const std::string& str)
{
    _nilTableSize=0;
    _isTable=false;
    _type=3;
    stringData.push_back(str);
}

CScriptFunctionDataItem::CScriptFunctionDataItem(const char* str)
{
    _nilTableSize=0;
    _isTable=false;
    _type=3;
    stringData.push_back(str);
}

CScriptFunctionDataItem::CScriptFunctionDataItem(const char* bufferPtr,unsigned int bufferLength)
{
    _nilTableSize=0;
    _isTable=false;
    _type=4;
    std::string v(bufferPtr,bufferLength);
    stringData.push_back(v);
}

CScriptFunctionDataItem::CScriptFunctionDataItem(const std::vector<bool>& v)
{
    _nilTableSize=0;
    _isTable=true;
    _type=0;
    boolData.assign(v.begin(),v.end());
}

CScriptFunctionDataItem::CScriptFunctionDataItem(const std::vector<int>& v)
{
    _nilTableSize=0;
    _isTable=true;
    _type=1;
    int32Data.assign(v.begin(),v.end());
}

CScriptFunctionDataItem::CScriptFunctionDataItem(const std::vector<float>& v)
{
    _nilTableSize=0;
    _isTable=true;
    _type=2;
    floatData.assign(v.begin(),v.end());
}

CScriptFunctionDataItem::CScriptFunctionDataItem(const std::vector<double>& v)
{
    _nilTableSize=0;
    _isTable=true;
    _type=5;
    doubleData.assign(v.begin(),v.end());
}

CScriptFunctionDataItem::CScriptFunctionDataItem(const std::vector<std::string>& v)
{
    _nilTableSize=0;
    _isTable=true;
    _type=3;
    stringData.assign(v.begin(),v.end());
}

CScriptFunctionDataItem::~CScriptFunctionDataItem()
{
}

bool CScriptFunctionDataItem::isTable()
{
    return(_isTable);
}

int CScriptFunctionDataItem::getType()
{
    return(_type);
}

void CScriptFunctionDataItem::setNilTable(int size)
{
    if (_type==-1)
    {
        _isTable=true;
        _nilTableSize=size;
    }
}

int CScriptFunctionDataItem::getNilTableSize()
{
    return(_nilTableSize);
}
