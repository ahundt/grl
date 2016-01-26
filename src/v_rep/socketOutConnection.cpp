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

#include "socketOutConnection.h"

#define HEADER_LENGTH 6 // byte1=id1, byte2=id2, byte3+byte4=packetSize, byte5+byte6=packetsLeftToRead

CSocketOutConnection::CSocketOutConnection(const char* theConnectionAddress,int theConnectionPort,unsigned short maxPacketSize/*=250*/,char headerID1/*=59*/,char headerID2/*=57*/)
{
	_socketConnectionAddress=theConnectionAddress;
	_socketConnectionPort=theConnectionPort;
	_socketConn=-1;
	_headerByte1=headerID1;
	_headerByte2=headerID2;
	_maxPacketSize=maxPacketSize;
}

CSocketOutConnection::~CSocketOutConnection()
{
	if (_socketConn!=(SOCKET)-1)
	{
	#ifdef _WIN32
		closesocket(_socketConn);
		WSACleanup();
	#elif defined (__linux) || defined (__APPLE__)
		close(_socketConn);
	#endif
	}
}

int CSocketOutConnection::connectToServer()
{ // return 1: success
#ifdef _WIN32
	if (WSAStartup(0x101,&_socketWsaData)!=0)
		return(0);
#endif
	_socketConn=socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
	if(_socketConn==INVALID_SOCKET)
	{
#ifdef _WIN32
		WSACleanup();
#endif
		return(0);
	}
	_socketServer.sin_addr.s_addr=inet_addr(_socketConnectionAddress.c_str());

	_socketServer.sin_family=AF_INET;
	_socketServer.sin_port=htons(_socketConnectionPort);
	if(connect(_socketConn,(struct sockaddr*)&_socketServer,sizeof(_socketServer)))
	{
#ifdef _WIN32
		closesocket(_socketConn);
		WSACleanup();
#elif defined (__linux) || defined (__APPLE__)
		close(_socketConn);
#endif
		return(0);
	}
	return(1);
}

bool CSocketOutConnection::sendData(char* data,int dataSize)
{
	if (_socketConn==(SOCKET)-1)
		return(false);

	if (dataSize==0)
		return(false);
	
	// In Following we make sure we don't send too big packets (we might send the data in several packets)
	int packetCount=0;
	int s=dataSize;
	while (s!=0)
	{
		packetCount++;
		if (s>_maxPacketSize-HEADER_LENGTH)
			s-=_maxPacketSize-HEADER_LENGTH;
		else
			s=0;
	}

	s=dataSize;
	int ptr=0;
	while (s!=0)
	{
		packetCount--;
		int sizeToSend=s;
		if (s>_maxPacketSize-HEADER_LENGTH)
			sizeToSend=_maxPacketSize-HEADER_LENGTH;
		s-=sizeToSend;
		if (!_sendSimplePacket(data+ptr,sizeToSend,packetCount))
			return(false);
		ptr+=sizeToSend;
	}
	return(true);
}

char* CSocketOutConnection::receiveReplyData(int& dataSize)
{ // Returns the data size if >0, otherwise error
	if (_socketConn==(SOCKET)-1)
	{
		dataSize=-2; // error
		return(NULL);
	}

	std::vector<char> receivedData;
	while (true)
	{
		std::vector<char> inDat;
		int result=_receiveSimplePacket(inDat);
		if (result<0)
		{
			dataSize=0; // error
			return(NULL);
		}
		receivedData.insert(receivedData.end(),inDat.begin(),inDat.end());
		if (result==0)
		{ // success
			dataSize=int(receivedData.size());
			char* retBuff=new char[dataSize];
			for (int i=0;i<dataSize;i++)
				retBuff[i]=receivedData[i];
			return(retBuff);
		}
	}
}

bool CSocketOutConnection::_sendSimplePacket(char* packet,int packetLength,unsigned short packetsLeft)
{
	if (packetLength==0)
		return(false);
	// Insert the header:
	unsigned short s=(unsigned short)packetLength;
	char header[HEADER_LENGTH];
	header[0]=_headerByte1;
	header[1]=_headerByte2;
	((unsigned short*)(header+2))[0]=s;
	((unsigned short*)(header+2))[1]=packetsLeft;

	std::vector<char> toSend;
	for (int i=0;i<HEADER_LENGTH;i++)
		toSend.push_back(header[i]);
	for (int i=0;i<packetLength;i++)
		toSend.push_back(packet[i]);
	// Send the packet:
	int dl=send(_socketConn,&toSend[0],packetLength+HEADER_LENGTH,0);
	if (dl==packetLength+HEADER_LENGTH)
		return(true);
	return(false);
}

int CSocketOutConnection::_receiveSimplePacket(std::vector<char>& packet)
{ // Returns the number of packets left to read if >=0, otherwise error
	//1. Read the header and packet size:
	char headerAndSize[HEADER_LENGTH];
	int totalReceived=0;
	unsigned short startT=_getTimeInMs();
	while(totalReceived!=HEADER_LENGTH)
	{
		int nb=recv(_socketConn,headerAndSize+totalReceived,HEADER_LENGTH-totalReceived,0);
		if (nb<1)
			break;
		totalReceived+=nb;
		if (_getTimeDiffInMs(startT)>3000)
			break;
	}
	// 2. Check if the header is consistent:
	if (totalReceived!=HEADER_LENGTH)
		return(-1); // Error reading
	if ( (headerAndSize[0]!=_headerByte1)||(headerAndSize[1]!=_headerByte2) )
		return(-1); // Error, wrong header
	unsigned short dataLength=((unsigned short*)(headerAndSize+2))[0];
	// 3. Read the data with correct length:
	packet.clear();
	packet.resize(dataLength,0);
	totalReceived=0;
	startT=_getTimeInMs();
	while(totalReceived!=dataLength)
	{
		int nb=recv(_socketConn,&packet[0]+totalReceived,dataLength-totalReceived,0);
		if (nb<1)
			break;
		totalReceived+=nb;
		if (_getTimeDiffInMs(startT)>3000)
			break;
	}
	if (totalReceived!=dataLength)
		return(-1); // wrong size or nothing received
	return(int(((unsigned short*)(headerAndSize+2))[1]));
}

int CSocketOutConnection::_getTimeInMs()
{
#ifdef _WIN32
	return(timeGetTime()&0x03ffffff);
#elif defined (__linux) || defined (__APPLE__)
	struct timeval tv;
	unsigned int result=0;
	if (gettimeofday(&tv,NULL)==0)
		result=(tv.tv_sec*1000+tv.tv_usec/1000)&0x03ffffff;
	return(result);
#endif
}

int CSocketOutConnection::_getTimeDiffInMs(int lastTime)
{
	int currentTime=_getTimeInMs();
	if (currentTime<lastTime)
		return(currentTime+0x03ffffff-lastTime);
	return(currentTime-lastTime);
}
