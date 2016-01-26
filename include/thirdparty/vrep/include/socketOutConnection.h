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

#pragma once

#include <vector>
#include <string>

#ifdef _WIN32
	#include <Windows.h>
	#include <process.h>
	#ifndef QT_COMPIL
		#pragma message("Adding library: Winmm.lib")
		#pragma comment(lib,"Winmm.lib")
		#pragma message("Adding library: Ws2_32.lib")
		#pragma comment(lib,"Ws2_32.lib")
	#endif
#elif defined (__linux) || defined (__APPLE__)
	#include <sys/time.h>
	#include <unistd.h>
	#include <sys/socket.h>
	#include <arpa/inet.h>
	#include <netdb.h>
	#define SOCKET int
	#define INVALID_SOCKET (-1)
#endif

class CSocketOutConnection  
{
public:
	CSocketOutConnection(const char* theConnectionAddress,int theConnectionPort,unsigned short maxPacketSize=250,char headerID1=59,char headerID2=57);
	virtual ~CSocketOutConnection();

	int connectToServer();
	bool sendData(char* data,int dataSize);
	char* receiveReplyData(int& dataSize);

protected:
	bool _sendSimplePacket(char* packet,int packetLength,unsigned short packetsLeft);
	int _receiveSimplePacket(std::vector<char>& packet);

	int _getTimeInMs();
	int _getTimeDiffInMs(int lastTime);

	std::string			_socketConnectionAddress;
	int					_socketConnectionPort;

#ifdef _WIN32
	WSADATA	_socketWsaData;
#else

#endif
	SOCKET _socketConn;
	struct sockaddr_in _socketServer;

	char _headerByte1;
	char _headerByte2;
	unsigned short _maxPacketSize;
};
