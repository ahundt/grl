// Copyright 2006-2015 Coppelia Robotics GmbH. All rights reserved. 
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
// This file was automatically created for V-REP release V3.2.3 rev4 on December 21st 2015

#include "luaFunctionData.h"
#include <sstream>
#include <cstring>

CLuaFunctionData::CLuaFunctionData()
{
}

CLuaFunctionData::~CLuaFunctionData()
{

}

void CLuaFunctionData::getInputDataForFunctionRegistration(const int* dat,std::vector<int>& outDat)
{
	outDat.clear();
	outDat.push_back(dat[0]);
	for (int i=0;i<dat[0];i++)
		outDat.push_back((dat[1+2*i+0]|SIM_LUA_ARG_NIL_ALLOWED)-SIM_LUA_ARG_NIL_ALLOWED);
}

std::vector<CLuaFunctionDataItem>* CLuaFunctionData::getInDataPtr()
{ // use this when reading data from Lua from inside of a custom Lua function call
	return(&_inData);
}

std::vector<CLuaFunctionDataItem>* CLuaFunctionData::getOutDataPtr_luaFunctionCall()
{ // use this when reading data returned from a Lua function call from a plugin
	return(&_outData);
}

bool CLuaFunctionData::readDataFromLua(const SLuaCallBack* p,const int* expectedArguments,int requiredArgumentCount,const char* functionName)
{  // use this when reading data from Lua from inside of a custom Lua function call
	_inData.clear();
	int argCnt=p->inputArgCount;
	if (argCnt<requiredArgumentCount)
	{
		simSetLastError(functionName,"Not enough arguments.");
		return(false);
	}

	int boolArgInd=0;
	int intArgInd=0;
	int floatArgInd=0;
    int doubleArgInd=0;
    int charArgInd=0;
	int charBuffArgInd=0;

	for (int i=0;i<argCnt;i++)
	{
		if (i>=expectedArguments[0])
			break;
		bool done=false;
		if (p->inputArgTypeAndSize[i*2+0]==sim_lua_arg_nil)
		{
			// is nil explicitely allowed?
			if (expectedArguments[1+i*2+0]&SIM_LUA_ARG_NIL_ALLOWED)
			{ // yes. This is for an argument that can optionally also be nil.
				CLuaFunctionDataItem dat;
				_inData.push_back(dat);
				done=true;
			}
			else
			{ // no
				if (int(_inData.size())<requiredArgumentCount)
				{
					std::ostringstream str;
					str << "Argument " << i+1 << " is not correct.";
					simSetLastError(functionName,str.str().c_str());
					return(false);
				}
				break; // this argument is nil, so it is like inexistant. But we also won't explore any more arguments, we have enough.
			}
		}
		if (!done)
		{
			if (p->inputArgTypeAndSize[i*2+0]!=((expectedArguments[1+i*2+0]|SIM_LUA_ARG_NIL_ALLOWED)-SIM_LUA_ARG_NIL_ALLOWED))
			{
				std::ostringstream str;
				str << "Argument " << i+1 << " is not correct.";
				simSetLastError(functionName,str.str().c_str());
				return(false);
			}
			if (p->inputArgTypeAndSize[i*2+0]&sim_lua_arg_table)
			{ // we have a table
				if ( (p->inputArgTypeAndSize[i*2+1]<expectedArguments[1+i*2+1])&&(expectedArguments[1+i*2+1]!=0) )
				{
					std::ostringstream str;
					str << "Argument " << i+1 << " is not correct (wrong table size).";
					simSetLastError(functionName,str.str().c_str());
					return(false);
				}
				else
				{
					int t=p->inputArgTypeAndSize[i*2+0]-sim_lua_arg_table;
					int itemCnt=p->inputArgTypeAndSize[i*2+1];

					if (t==sim_lua_arg_nil)
					{
						CLuaFunctionDataItem* a=new CLuaFunctionDataItem();
						a->setNilTable(itemCnt);
						CLuaFunctionDataItem dat;
						dat.setNilTable(itemCnt);
						_inData.push_back(dat);
					}
					if (t==sim_lua_arg_bool)
					{
						std::vector<bool> vect;
						for (int j=0;j<itemCnt;j++)
							vect.push_back(p->inputBool[boolArgInd++]!=0);
						CLuaFunctionDataItem dat(vect);
						_inData.push_back(dat);
					}
					if (t==sim_lua_arg_int)
					{
						std::vector<int> vect;
						for (int j=0;j<itemCnt;j++)
							vect.push_back(p->inputInt[intArgInd++]);
						CLuaFunctionDataItem dat(vect);
						_inData.push_back(dat);
					}
					if (t==sim_lua_arg_float)
					{
						std::vector<float> vect;
						for (int j=0;j<itemCnt;j++)
							vect.push_back(p->inputFloat[floatArgInd++]);
						CLuaFunctionDataItem dat(vect);
						_inData.push_back(dat);
					}
					if (t==sim_lua_arg_double)
					{
						std::vector<double> vect;
						for (int j=0;j<itemCnt;j++)
							vect.push_back(p->inputDouble[doubleArgInd++]);
						CLuaFunctionDataItem dat(vect);
						_inData.push_back(dat);
					}
					if (t==sim_lua_arg_string)
					{
						std::vector<std::string> vect;
						for (int j=0;j<itemCnt;j++)
						{
							std::string str(p->inputChar+charArgInd);
							vect.push_back(str);
							charArgInd+=int(strlen(p->inputChar+charArgInd))+1;
						}
						CLuaFunctionDataItem dat(vect);
						_inData.push_back(dat);
					}
					if (t==sim_lua_arg_charbuff)
					{
						std::ostringstream str;
						str << "Argument " << i+1 << " cannot be a table.";
						simSetLastError(functionName,str.str().c_str());
						return(false);
					}
				}
			}
			else
			{ // we do not have a table
				int t=p->inputArgTypeAndSize[i*2+0];
				if (t==sim_lua_arg_nil)
				{
					CLuaFunctionDataItem dat;
					_inData.push_back(dat);
				}
				if (t==sim_lua_arg_bool)
				{
					CLuaFunctionDataItem dat(p->inputBool[boolArgInd++]!=0);
					_inData.push_back(dat);
				}
				if (t==sim_lua_arg_int)
				{
					CLuaFunctionDataItem dat(p->inputInt[intArgInd++]);
					_inData.push_back(dat);
				}
				if (t==sim_lua_arg_float)
				{
					CLuaFunctionDataItem dat(p->inputFloat[floatArgInd++]);
					_inData.push_back(dat);
				}
				if (t==sim_lua_arg_double)
				{
					CLuaFunctionDataItem dat(p->inputDouble[doubleArgInd++]);
					_inData.push_back(dat);
				}
				if (t==sim_lua_arg_string)
				{
					CLuaFunctionDataItem dat(std::string(p->inputChar+charArgInd));
					charArgInd+=int(strlen(p->inputChar+charArgInd))+1;
					_inData.push_back(dat);
				}
				if (t==sim_lua_arg_charbuff)
				{
					if ( (p->inputArgTypeAndSize[i*2+1]<expectedArguments[1+i*2+1])&&(expectedArguments[1+i*2+1]!=0) )
					{
						std::ostringstream str;
						str << "Argument " << i+1 << " is not correct (wrong buffer size).";
						simSetLastError(functionName,str.str().c_str());
						return(false);
					}
					else
					{
						CLuaFunctionDataItem dat(p->inputCharBuff+charBuffArgInd,p->inputArgTypeAndSize[i*2+1]);
						charBuffArgInd+=p->inputArgTypeAndSize[i*2+1];
						_inData.push_back(dat);
					}
				}
			}
		}
	}
	return(true);
}

bool CLuaFunctionData::readDataFromLua_luaFunctionCall(const SLuaCallBack* p,const int* expectedArguments,int requiredArgumentCount,const char* functionName)
{  // use this when reading data returned from a Lua function call from a plugin
	_outData.clear();
	int argCnt=p->outputArgCount;
	if (argCnt<requiredArgumentCount)
	{
		simSetLastError(functionName,"Not enough return arguments.");
		return(false);
	}

	int boolArgInd=0;
	int intArgInd=0;
	int floatArgInd=0;
    int doubleArgInd=0;
    int charArgInd=0;
	int charBuffArgInd=0;

	for (int i=0;i<argCnt;i++)
	{
		if (i>=expectedArguments[0])
			break;
		bool done=false;
		if (p->outputArgTypeAndSize[i*2+0]==sim_lua_arg_nil)
		{
			// is nil explicitely allowed?
			if (expectedArguments[1+i*2+0]&SIM_LUA_ARG_NIL_ALLOWED)
			{ // yes. This is for an argument that can optionally also be nil.
				CLuaFunctionDataItem dat;
				_outData.push_back(dat);
				done=true;
			}
			else
			{ // no
				if (int(_outData.size())<requiredArgumentCount)
				{
					std::ostringstream str;
					str << "Return argument " << i+1 << " is not correct.";
					simSetLastError(functionName,str.str().c_str());
					return(false);
				}
				break; // this argument is nil, so it is like inexistant. But we also won't explore any more arguments, we have enough.
			}
		}
		if (!done)
		{
			if (p->outputArgTypeAndSize[i*2+0]!=((expectedArguments[1+i*2+0]|SIM_LUA_ARG_NIL_ALLOWED)-SIM_LUA_ARG_NIL_ALLOWED))
			{
				std::ostringstream str;
				str << "Return argument " << i+1 << " is not correct.";
				simSetLastError(functionName,str.str().c_str());
				return(false);
			}
			if (p->outputArgTypeAndSize[i*2+0]&sim_lua_arg_table)
			{ // we have a table
				if ( (p->outputArgTypeAndSize[i*2+1]<expectedArguments[1+i*2+1])&&(expectedArguments[1+i*2+1]!=0) )
				{
					std::ostringstream str;
					str << "Return argument " << i+1 << " is not correct (wrong table size).";
					simSetLastError(functionName,str.str().c_str());
					return(false);
				}
				else
				{
					int t=p->outputArgTypeAndSize[i*2+0]-sim_lua_arg_table;
					int itemCnt=p->outputArgTypeAndSize[i*2+1];

					if (t==sim_lua_arg_nil)
					{
						CLuaFunctionDataItem* a=new CLuaFunctionDataItem();
						a->setNilTable(itemCnt);
						CLuaFunctionDataItem dat;
						dat.setNilTable(itemCnt);
						_outData.push_back(dat);
					}
					if (t==sim_lua_arg_bool)
					{
						std::vector<bool> vect;
						for (int j=0;j<itemCnt;j++)
							vect.push_back(p->outputBool[boolArgInd++]!=0);
						CLuaFunctionDataItem dat(vect);
						_outData.push_back(dat);
					}
					if (t==sim_lua_arg_int)
					{
						std::vector<int> vect;
						for (int j=0;j<itemCnt;j++)
							vect.push_back(p->outputInt[intArgInd++]);
						CLuaFunctionDataItem dat(vect);
						_outData.push_back(dat);
					}
					if (t==sim_lua_arg_float)
					{
						std::vector<float> vect;
						for (int j=0;j<itemCnt;j++)
							vect.push_back(p->outputFloat[floatArgInd++]);
						CLuaFunctionDataItem dat(vect);
						_outData.push_back(dat);
					}
					if (t==sim_lua_arg_double)
					{
						std::vector<double> vect;
						for (int j=0;j<itemCnt;j++)
							vect.push_back(p->outputDouble[doubleArgInd++]);
						CLuaFunctionDataItem dat(vect);
						_outData.push_back(dat);
					}
					if (t==sim_lua_arg_string)
					{
						std::vector<std::string> vect;
						for (int j=0;j<itemCnt;j++)
						{
							std::string str(p->outputChar+charArgInd);
							vect.push_back(str);
							charArgInd+=int(strlen(p->outputChar+charArgInd))+1;
						}
						CLuaFunctionDataItem dat(vect);
						_outData.push_back(dat);
					}
					if (t==sim_lua_arg_charbuff)
					{
						std::ostringstream str;
						str << "Return argument " << i+1 << " cannot be a table.";
						simSetLastError(functionName,str.str().c_str());
						return(false);
					}
				}
			}
			else
			{ // we do not have a table
				int t=p->outputArgTypeAndSize[i*2+0];
				if (t==sim_lua_arg_nil)
				{
					CLuaFunctionDataItem dat;
					_outData.push_back(dat);
				}
				if (t==sim_lua_arg_bool)
				{
					CLuaFunctionDataItem dat(p->outputBool[boolArgInd++]!=0);
					_outData.push_back(dat);
				}
				if (t==sim_lua_arg_int)
				{
					CLuaFunctionDataItem dat(p->outputInt[intArgInd++]);
					_outData.push_back(dat);
				}
				if (t==sim_lua_arg_float)
				{
					CLuaFunctionDataItem dat(p->outputFloat[floatArgInd++]);
					_outData.push_back(dat);
				}
				if (t==sim_lua_arg_double)
				{
					CLuaFunctionDataItem dat(p->outputDouble[doubleArgInd++]);
					_outData.push_back(dat);
				}
				if (t==sim_lua_arg_string)
				{
					CLuaFunctionDataItem dat(std::string(p->outputChar+charArgInd));
					charArgInd+=int(strlen(p->outputChar+charArgInd))+1;
					_outData.push_back(dat);
				}
				if (t==sim_lua_arg_charbuff)
				{
					if ( (p->outputArgTypeAndSize[i*2+1]<expectedArguments[1+i*2+1])&&(expectedArguments[1+i*2+1]!=0) )
					{
						std::ostringstream str;
						str << "Return argument " << i+1 << " is not correct (wrong buffer size).";
						simSetLastError(functionName,str.str().c_str());
						return(false);
					}
					else
					{
						CLuaFunctionDataItem dat(p->outputCharBuff+charBuffArgInd,p->outputArgTypeAndSize[i*2+1]);
						charBuffArgInd+=p->outputArgTypeAndSize[i*2+1];
						_outData.push_back(dat);
					}
				}
			}
		}
	}
	return(true);
}

void CLuaFunctionData::pushOutData(const CLuaFunctionDataItem& dataItem)
{ // use this when returning data from inside of a custom Lua function call
	_outData.push_back(dataItem);
}

void CLuaFunctionData::pushOutData_luaFunctionCall(const CLuaFunctionDataItem& dataItem)
{ // use this when doing a Lua function call from a plugin
	_inData.push_back(dataItem);
}


void CLuaFunctionData::writeDataToLua(SLuaCallBack* p)
{ // use this when returning data from inside of a custom Lua function call
	p->outputArgCount=0;
	int itemCnt=int(_outData.size());
	if (itemCnt>0)
	{
		p->outputArgCount=itemCnt;
		p->outputArgTypeAndSize=(simInt*)simCreateBuffer(p->outputArgCount*2*sizeof(simInt));

		int boolDataCnt=0;
		int intDataCnt=0;
		int floatDataCnt=0;
        int doubleDataCnt=0;
        int charDataCnt=0;
		int charBuffDataCnt=0;

		for (int i=0;i<itemCnt;i++)
		{
			if (_outData[i].isTable())
			{ // table
				if (_outData[i].getType()==-1)
				{
					p->outputArgTypeAndSize[i*2+0]=sim_lua_arg_nil|sim_lua_arg_table;
					p->outputArgTypeAndSize[i*2+1]=_outData[i].getNilTableSize();
				}
				if (_outData[i].getType()==0)
				{
					p->outputArgTypeAndSize[i*2+0]=sim_lua_arg_bool|sim_lua_arg_table;
					p->outputArgTypeAndSize[i*2+1]=int(_outData[i].boolData.size());
					boolDataCnt+=p->outputArgTypeAndSize[i*2+1];
				}
				if (_outData[i].getType()==1)
				{
					p->outputArgTypeAndSize[i*2+0]=sim_lua_arg_int|sim_lua_arg_table;
					p->outputArgTypeAndSize[i*2+1]=int(_outData[i].intData.size());
					intDataCnt+=p->outputArgTypeAndSize[i*2+1];
				}
				if (_outData[i].getType()==2)
				{
					p->outputArgTypeAndSize[i*2+0]=sim_lua_arg_float|sim_lua_arg_table;
					p->outputArgTypeAndSize[i*2+1]=int(_outData[i].floatData.size());
					floatDataCnt+=p->outputArgTypeAndSize[i*2+1];
				}
                if (_outData[i].getType()==5)
                {
                    p->outputArgTypeAndSize[i*2+0]=sim_lua_arg_double|sim_lua_arg_table;
                    p->outputArgTypeAndSize[i*2+1]=int(_outData[i].doubleData.size());
                    doubleDataCnt+=p->outputArgTypeAndSize[i*2+1];
                }
                if (_outData[i].getType()==3)
				{
					p->outputArgTypeAndSize[i*2+0]=sim_lua_arg_string|sim_lua_arg_table;
					p->outputArgTypeAndSize[i*2+1]=int(_outData[i].stringData.size());
					for (int j=0;j<int(_outData[i].stringData.size());j++)
						charDataCnt+=int(_outData[i].stringData[j].length())+1;
				}
			}
			else
			{
				if (_outData[i].getType()==-1)
				{
					p->outputArgTypeAndSize[i*2+1]=0;
					p->outputArgTypeAndSize[i*2+0]=sim_lua_arg_nil;
				}
				if (_outData[i].getType()==0)
				{
					p->outputArgTypeAndSize[i*2+1]=0;
					p->outputArgTypeAndSize[i*2+0]=sim_lua_arg_bool;
					boolDataCnt++;
				}
				if (_outData[i].getType()==1)
				{
					p->outputArgTypeAndSize[i*2+1]=0;
					p->outputArgTypeAndSize[i*2+0]=sim_lua_arg_int;
					intDataCnt++;
				}
				if (_outData[i].getType()==2)
				{
					p->outputArgTypeAndSize[i*2+1]=0;
					p->outputArgTypeAndSize[i*2+0]=sim_lua_arg_float;
					floatDataCnt++;
				}
                if (_outData[i].getType()==5)
                {
                    p->outputArgTypeAndSize[i*2+1]=0;
                    p->outputArgTypeAndSize[i*2+0]=sim_lua_arg_double;
                    doubleDataCnt++;
                }
                if (_outData[i].getType()==3)
				{
					p->outputArgTypeAndSize[i*2+1]=0;
					p->outputArgTypeAndSize[i*2+0]=sim_lua_arg_string;
					charDataCnt+=int(_outData[i].stringData[0].length())+1;
				}
				if (_outData[i].getType()==4)
				{
					p->outputArgTypeAndSize[i*2+1]=int(_outData[i].stringData[0].length());
					p->outputArgTypeAndSize[i*2+0]=sim_lua_arg_charbuff;
					charBuffDataCnt+=int(_outData[i].stringData[0].length());
				}
			}
		}

		// Now create the buffers:
		p->outputBool=(simBool*)simCreateBuffer(boolDataCnt*sizeof(simBool));
		p->outputInt=(simInt*)simCreateBuffer(intDataCnt*sizeof(simInt));
		p->outputFloat=(simFloat*)simCreateBuffer(floatDataCnt*sizeof(simFloat));
        p->outputDouble=(simDouble*)simCreateBuffer(doubleDataCnt*sizeof(simDouble));
        p->outputChar=(simChar*)simCreateBuffer(charDataCnt*sizeof(simChar));
		p->outputCharBuff=(simChar*)simCreateBuffer(charBuffDataCnt*sizeof(simChar));
		
		// Now populate the buffers:
		int boolDataOff=0;
		int intDataOff=0;
		int floatDataOff=0;
        int doubleDataOff=0;
        int charDataOff=0;
		int charBuffDataOff=0;

		for (int i=0;i<itemCnt;i++)
		{
			if (_outData[i].isTable())
			{ // table
				if (_outData[i].getType()==0)
				{
					for (int j=0;j<int(_outData[i].boolData.size());j++)
						p->outputBool[boolDataOff++]=_outData[i].boolData[j];
				}
				if (_outData[i].getType()==1)
				{
					for (int j=0;j<int(_outData[i].intData.size());j++)
						p->outputInt[intDataOff++]=_outData[i].intData[j];
				}
				if (_outData[i].getType()==2)
				{
					for (int j=0;j<int(_outData[i].floatData.size());j++)
						p->outputFloat[floatDataOff++]=_outData[i].floatData[j];
				}
                if (_outData[i].getType()==5)
                {
                    for (int j=0;j<int(_outData[i].doubleData.size());j++)
                        p->outputDouble[doubleDataOff++]=_outData[i].doubleData[j];
                }
                if (_outData[i].getType()==3)
				{
					for (int j=0;j<int(_outData[i].stringData.size());j++)
					{
						for (int k=0;k<int(_outData[i].stringData[j].length());k++)
							p->outputChar[charDataOff++]=_outData[i].stringData[j][k];
						p->outputChar[charDataOff++]=0;
					}
				}
			}
			else
			{
				if (_outData[i].getType()==0)
					p->outputBool[boolDataOff++]=_outData[i].boolData[0];
				if (_outData[i].getType()==1)
					p->outputInt[intDataOff++]=_outData[i].intData[0];
				if (_outData[i].getType()==2)
					p->outputFloat[floatDataOff++]=_outData[i].floatData[0];
                if (_outData[i].getType()==5)
                    p->outputDouble[doubleDataOff++]=_outData[i].doubleData[0];
                if (_outData[i].getType()==3)
				{
					for (int j=0;j<int(_outData[i].stringData[0].length());j++)
						p->outputChar[charDataOff++]=_outData[i].stringData[0][j];
					p->outputChar[charDataOff++]=0;
				}
				if (_outData[i].getType()==4)
				{
					for (int j=0;j<int(_outData[i].stringData[0].length());j++)
						p->outputCharBuff[charBuffDataOff++]=_outData[i].stringData[0][j];
				}
			}
		}
	}
}

void CLuaFunctionData::writeDataToLua_luaFunctionCall(SLuaCallBack* p,const int* expectedArguments)
{ // use this when doing a Lua function call from a plugin
	p->inputArgCount=0;
	p->inputBool=NULL;
	p->inputInt=NULL;
	p->inputFloat=NULL;
	p->inputDouble=NULL;
	p->inputChar=NULL;
	p->inputCharBuff=NULL;
	p->inputArgTypeAndSize=NULL;

	p->outputArgCount=expectedArguments[0];
	p->outputBool=NULL;
	p->outputInt=NULL;
	p->outputFloat=NULL;
	p->outputDouble=NULL;
	p->outputChar=NULL;
	p->outputCharBuff=NULL;
	p->outputArgTypeAndSize=(simInt*)simCreateBuffer(p->outputArgCount*2*sizeof(simInt));
	for (int i=0;i<p->outputArgCount*2;i++)
		p->outputArgTypeAndSize[i]=expectedArguments[1+i];

	int itemCnt=int(_inData.size());
	if (itemCnt>0)
	{
		p->inputArgCount=itemCnt;
		p->inputArgTypeAndSize=(simInt*)simCreateBuffer(p->inputArgCount*2*sizeof(simInt));

		int boolDataCnt=0;
		int intDataCnt=0;
		int floatDataCnt=0;
        int doubleDataCnt=0;
        int charDataCnt=0;
		int charBuffDataCnt=0;

		for (int i=0;i<itemCnt;i++)
		{
			if (_inData[i].isTable())
			{ // table
				if (_inData[i].getType()==-1)
				{
					p->inputArgTypeAndSize[i*2+0]=sim_lua_arg_nil|sim_lua_arg_table;
					p->inputArgTypeAndSize[i*2+1]=_inData[i].getNilTableSize();
				}
				if (_inData[i].getType()==0)
				{
					p->inputArgTypeAndSize[i*2+0]=sim_lua_arg_bool|sim_lua_arg_table;
					p->inputArgTypeAndSize[i*2+1]=int(_inData[i].boolData.size());
					boolDataCnt+=p->inputArgTypeAndSize[i*2+1];
				}
				if (_inData[i].getType()==1)
				{
					p->inputArgTypeAndSize[i*2+0]=sim_lua_arg_int|sim_lua_arg_table;
					p->inputArgTypeAndSize[i*2+1]=int(_inData[i].intData.size());
					intDataCnt+=p->inputArgTypeAndSize[i*2+1];
				}
				if (_inData[i].getType()==2)
				{
					p->inputArgTypeAndSize[i*2+0]=sim_lua_arg_float|sim_lua_arg_table;
					p->inputArgTypeAndSize[i*2+1]=int(_inData[i].floatData.size());
					floatDataCnt+=p->inputArgTypeAndSize[i*2+1];
				}
                if (_inData[i].getType()==5)
                {
                    p->inputArgTypeAndSize[i*2+0]=sim_lua_arg_double|sim_lua_arg_table;
                    p->inputArgTypeAndSize[i*2+1]=int(_inData[i].doubleData.size());
                    doubleDataCnt+=p->inputArgTypeAndSize[i*2+1];
                }
                if (_inData[i].getType()==3)
				{
					p->inputArgTypeAndSize[i*2+0]=sim_lua_arg_string|sim_lua_arg_table;
					p->inputArgTypeAndSize[i*2+1]=int(_inData[i].stringData.size());
					for (int j=0;j<int(_inData[i].stringData.size());j++)
						charDataCnt+=int(_inData[i].stringData[j].length())+1;
				}
			}
			else
			{
				if (_inData[i].getType()==-1)
				{
					p->inputArgTypeAndSize[i*2+1]=0;
					p->inputArgTypeAndSize[i*2+0]=sim_lua_arg_nil;
				}
				if (_inData[i].getType()==0)
				{
					p->inputArgTypeAndSize[i*2+1]=0;
					p->inputArgTypeAndSize[i*2+0]=sim_lua_arg_bool;
					boolDataCnt++;
				}
				if (_inData[i].getType()==1)
				{
					p->inputArgTypeAndSize[i*2+1]=0;
					p->inputArgTypeAndSize[i*2+0]=sim_lua_arg_int;
					intDataCnt++;
				}
				if (_inData[i].getType()==2)
				{
					p->inputArgTypeAndSize[i*2+1]=0;
					p->inputArgTypeAndSize[i*2+0]=sim_lua_arg_float;
					floatDataCnt++;
				}
                if (_inData[i].getType()==5)
                {
                    p->inputArgTypeAndSize[i*2+1]=0;
                    p->inputArgTypeAndSize[i*2+0]=sim_lua_arg_double;
                    doubleDataCnt++;
                }
                if (_inData[i].getType()==3)
				{
					p->inputArgTypeAndSize[i*2+1]=0;
					p->inputArgTypeAndSize[i*2+0]=sim_lua_arg_string;
					charDataCnt+=int(_inData[i].stringData[0].length())+1;
				}
				if (_inData[i].getType()==4)
				{
					p->inputArgTypeAndSize[i*2+1]=int(_inData[i].stringData[0].length());
					p->inputArgTypeAndSize[i*2+0]=sim_lua_arg_charbuff;
					charBuffDataCnt+=int(_inData[i].stringData[0].length());
				}
			}
		}

		// Now create the buffers:
		p->inputBool=(simBool*)simCreateBuffer(boolDataCnt*sizeof(simBool));
		p->inputInt=(simInt*)simCreateBuffer(intDataCnt*sizeof(simInt));
		p->inputFloat=(simFloat*)simCreateBuffer(floatDataCnt*sizeof(simFloat));
        p->inputDouble=(simDouble*)simCreateBuffer(doubleDataCnt*sizeof(simDouble));
        p->inputChar=(simChar*)simCreateBuffer(charDataCnt*sizeof(simChar));
		p->inputCharBuff=(simChar*)simCreateBuffer(charBuffDataCnt*sizeof(simChar));
		
		// Now populate the buffers:
		int boolDataOff=0;
		int intDataOff=0;
		int floatDataOff=0;
        int doubleDataOff=0;
        int charDataOff=0;
		int charBuffDataOff=0;

		for (int i=0;i<itemCnt;i++)
		{
			if (_inData[i].isTable())
			{ // table
				if (_inData[i].getType()==0)
				{
					for (int j=0;j<int(_inData[i].boolData.size());j++)
						p->inputBool[boolDataOff++]=_inData[i].boolData[j];
				}
				if (_inData[i].getType()==1)
				{
					for (int j=0;j<int(_inData[i].intData.size());j++)
						p->inputInt[intDataOff++]=_inData[i].intData[j];
				}
				if (_inData[i].getType()==2)
				{
					for (int j=0;j<int(_inData[i].floatData.size());j++)
						p->inputFloat[floatDataOff++]=_inData[i].floatData[j];
				}
                if (_inData[i].getType()==5)
                {
                    for (int j=0;j<int(_inData[i].doubleData.size());j++)
                        p->inputDouble[doubleDataOff++]=_inData[i].doubleData[j];
                }
                if (_inData[i].getType()==3)
				{
					for (int j=0;j<int(_inData[i].stringData.size());j++)
					{
						for (int k=0;k<int(_inData[i].stringData[j].length());k++)
							p->inputChar[charDataOff++]=_inData[i].stringData[j][k];
						p->inputChar[charDataOff++]=0;
					}
				}
			}
			else
			{
				if (_inData[i].getType()==0)
					p->inputBool[boolDataOff++]=_inData[i].boolData[0];
				if (_inData[i].getType()==1)
					p->inputInt[intDataOff++]=_inData[i].intData[0];
				if (_inData[i].getType()==2)
					p->inputFloat[floatDataOff++]=_inData[i].floatData[0];
                if (_inData[i].getType()==5)
                    p->inputDouble[doubleDataOff++]=_inData[i].doubleData[0];
                if (_inData[i].getType()==3)
				{
					for (int j=0;j<int(_inData[i].stringData[0].length());j++)
						p->inputChar[charDataOff++]=_inData[i].stringData[0][j];
					p->inputChar[charDataOff++]=0;
				}
				if (_inData[i].getType()==4)
				{
					for (int j=0;j<int(_inData[i].stringData[0].length());j++)
						p->inputCharBuff[charBuffDataOff++]=_inData[i].stringData[0][j];
				}
			}
		}
	}
}

void CLuaFunctionData::releaseBuffers_luaFunctionCall(SLuaCallBack* p)
{ // use this you finished with a Lua function call from a plugin
	simReleaseBuffer((char*)p->inputBool);
	p->inputBool=NULL;
	simReleaseBuffer((char*)p->inputInt);
	p->inputInt=NULL;
	simReleaseBuffer((char*)p->inputFloat);
	p->inputFloat=NULL;
	simReleaseBuffer((char*)p->inputDouble);
	p->inputDouble=NULL;
	simReleaseBuffer((char*)p->inputChar);
	p->inputChar=NULL;
	simReleaseBuffer((char*)p->inputCharBuff);
	p->inputCharBuff=NULL;
	simReleaseBuffer((char*)p->inputArgTypeAndSize);
	p->inputArgTypeAndSize=NULL;

	simReleaseBuffer((char*)p->outputBool);
	p->outputBool=NULL;
	simReleaseBuffer((char*)p->outputInt);
	p->outputInt=NULL;
	simReleaseBuffer((char*)p->outputFloat);
	p->outputFloat=NULL;
	simReleaseBuffer((char*)p->outputDouble);
	p->outputDouble=NULL;
	simReleaseBuffer((char*)p->outputChar);
	p->outputChar=NULL;
	simReleaseBuffer((char*)p->outputCharBuff);
	p->outputCharBuff=NULL;
	simReleaseBuffer((char*)p->outputArgTypeAndSize);
	p->outputArgTypeAndSize=NULL;
}
