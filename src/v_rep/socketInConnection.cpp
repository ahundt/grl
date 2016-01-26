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

#include "socketInConnection.h"
#define HEADER_LENGTH 6 // byte1=id1, byte2=id2, byte3+byte4=packetSize, byte5+byte6=packetsLeftToRead

CSocketInConnection::CSocketInConnection(int theConnectionPort,unsigned short maxPacketSize/*=250*/,char headerID1/*=59*/,char headerID2/*=57*/)
{
	_socketConnectionPort=theConnectionPort;
	_socketConnectWasOk=false;
	_headerByte1=headerID1;
	_headerByte2=headerID2;
	_maxPacketSize=maxPacketSize;

	memset(&_socketLocal,0,sizeof(struct sockaddr_in));
#if defined (__linux) || defined (__APPLE__)
	_socketServer=-1;
	_socketClient=-1;
#endif /* __linux || __APPLE__ */
}

CSocketInConnection::~CSocketInConnection()
{
	if (_socketConnectWasOk)
	{
#ifdef _WIN32
		shutdown(_socketClient,SD_BOTH);
		closesocket(_socketServer);
		WSACleanup();
#elif defined (__linux) || defined (__APPLE__)
		if (_socketServer!=-1)
		{
			close(_socketServer);
		}
		if (_socketClient!=-1)
		{
			close(_socketClient);
		}
#endif
	}
}

bool CSocketInConnection::connectToClient()
{
#ifdef _WIN32
	// 1. connect to port:
	if (WSAStartup(0x101,&_socketWsaData)!=0)
		return(false);	 // WSAStartup failed.
#endif /* _WIN32 */

	_socketLocal.sin_family=AF_INET;
	_socketLocal.sin_addr.s_addr=INADDR_ANY;
	_socketLocal.sin_port=htons((u_short)_socketConnectionPort);
	_socketServer=socket(AF_INET,SOCK_STREAM,0);
	if (_socketServer==INVALID_SOCKET)
		return(false); // socket failed.

	if (bind(_socketServer,(struct sockaddr*)&_socketLocal,sizeof(_socketLocal))!=0)
		return(false); // bind failed.

	if (listen(_socketServer,10)!=0)
		return(false); // listen failed.

	// 2. accept client:
	struct sockaddr_in from;
	int fromlen=sizeof(from);

	_socketClient=accept(_socketServer,(struct sockaddr*) &from, (_socklen *) &fromlen);
	_socketConnectedMachineIP=inet_ntoa(from.sin_addr);
	FD_ZERO(&_socketTheSet);
	FD_SET(_socketClient,&_socketTheSet);
	_socketConnectWasOk=true;
	return(true);
}

char* CSocketInConnection::receiveData(int& dataSize)
{ // Returns the data size if >0, 0=we had a read time out, -1=we have an error
	if (!_socketConnectWasOk)
	{
		dataSize=-1; // error
		return(NULL);
	}
	std::vector<char> receivedData;
	while (true)
	{
		std::vector<char> inDat;
		int result=_receiveSimplePacket(inDat);
		if (result<0)
		{
			dataSize=result+1; // error or read time-out
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

bool CSocketInConnection::replyToReceivedData(char* data,int dataSize)
{
	if (!_socketConnectWasOk)
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

std::string CSocketInConnection::getConnectedMachineIP()
{
	if (!_socketConnectWasOk)
		return("NONE (reception line is not open)");
	return(_socketConnectedMachineIP);
}

bool CSocketInConnection::_sendSimplePacket(char* packet,int packetLength,unsigned short packetsLeft)
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
	return(send(_socketClient,&toSend[0],packetLength+HEADER_LENGTH,0)==packetLength+HEADER_LENGTH);
}

int CSocketInConnection::_receiveSimplePacket(std::vector<char>& packet)
{
	// Returns the number of packets left to read if >=0, -2=error, -1=select time out
	FD_ZERO(&_socketTheSet);
	FD_SET(_socketClient,&_socketTheSet);
#ifdef _WIN32
	int selectResult=select(0,&_socketTheSet,NULL,NULL,NULL); // No timeout!
#endif /* _WIN32 */
#if defined (__linux) || defined (__APPLE__)
	int selectResult=select(_socketClient+1, &_socketTheSet,NULL,NULL,NULL); // No timeout!
#endif /* __linux || __APPLE__ */
	if (selectResult==1)
	{
		//1. Read the header and packet size:
		char headerAndSize[HEADER_LENGTH];
		int totalReceived=0;
		unsigned int startT=_getTimeInMs();
		while(totalReceived!=HEADER_LENGTH)
		{
			int nb=recv(_socketClient,headerAndSize+totalReceived,HEADER_LENGTH-totalReceived,0);
			if (nb<1)
				break;
			totalReceived+=nb;
			if (_getTimeDiffInMs(startT)>3000)
				break;
		}
		// 2. Check if the header is consistent:
		if (totalReceived!=HEADER_LENGTH)
			return(-2); // Error reading
		if ( (headerAndSize[0]!=_headerByte1)||(headerAndSize[1]!=_headerByte2) )
			return(-2); // Error, wrong header
		unsigned short dataLength=((unsigned short*)(headerAndSize+2))[0];
		// 3. Read the data with correct length:
		packet.clear();
		packet.resize(dataLength,0);
		totalReceived=0;
		startT=_getTimeInMs();
		while(totalReceived!=dataLength)
		{
			int nb=recv(_socketClient,&packet[0]+totalReceived,dataLength-totalReceived,0);
			if (nb<1)
				break;
			totalReceived+=nb;
			if (_getTimeDiffInMs(startT)>3000)
				break;
		}
		if (totalReceived!=dataLength)
			return(-2); // wrong size or nothing received
		return(int(((unsigned short*)(headerAndSize+2))[1]));
	}
	if (selectResult==0)
		return(-1);
	return(-2);
}

#ifdef _WIN32
unsigned int CSocketInConnection::_getTimeInMs(void)
{
	return(timeGetTime()&0x03ffffff);
}
#endif /* _WIN32 */

#if defined (__linux) || defined (__APPLE__)
unsigned int CSocketInConnection::_getTimeInMs(void)
{
	struct timeval tv;
	unsigned int result=0;
	if (gettimeofday(&tv,NULL)==0)
		result=(tv.tv_sec*1000+tv.tv_usec/1000)&0x03ffffff;
	return(result);
}
#endif /* __linux || __APPLE__ */

unsigned int CSocketInConnection::_getTimeDiffInMs(unsigned int lastTime)
{
	unsigned int currentTime=_getTimeInMs();
	if (currentTime<lastTime)
		return(currentTime+0x03ffffff-lastTime);
	return(currentTime-lastTime);
}
