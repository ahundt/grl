// This file is part of the REMOTE API
// 
// Copyright 2006-2016 Coppelia Robotics GmbH. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// The REMOTE API is licensed under the terms of GNU GPL:
// 
// -------------------------------------------------------------------
// The REMOTE API is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// THE REMOTE API IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
// AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
// DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
// MISUSING THIS SOFTWARE.
// 
// See the GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with the REMOTE API.  If not, see <http://www.gnu.org/licenses/>.
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.3.2 on August 29th 2016

#include "extApiPlatform.h"
#include <stdio.h>

#ifdef _WIN32
    #include <Windows.h>
    #include <process.h>
    #ifndef QT_COMPIL
        #pragma message("Adding library: Winmm.lib")
        #pragma comment(lib,"Winmm.lib")
        #pragma message("Adding library: Ws2_32.lib")
        #pragma comment(lib,"Ws2_32.lib")
    #endif
    #define MUTEX_HANDLE HANDLE
    #define MUTEX_HANDLE_X MUTEX_HANDLE
    #define THREAD_ID DWORD
    WSADATA _socketWsaData;
    #ifdef USE_ALSO_SHARED_MEMORY
        HANDLE _mmfConn[MAX_EXT_API_CONNECTIONS];
        simxInt _mmfSize[MAX_EXT_API_CONNECTIONS];
    #endif
#elif defined (__linux) || defined (__APPLE__)
    #include <pthread.h>
    #include <stdlib.h>
    #include <unistd.h>
    #include <string.h>
    #include <netinet/in.h>
    #include <sys/time.h>
    #include <sys/socket.h>
    #include <arpa/inet.h>
    #include <netdb.h>
    #define MUTEX_HANDLE pthread_mutex_t
    #define MUTEX_HANDLE_X MUTEX_HANDLE*
    #define THREAD_ID pthread_t
    #define SOCKET int
    #define DWORD unsigned long
    #define INVALID_SOCKET (-1)
#endif

MUTEX_HANDLE _globalMutex;

MUTEX_HANDLE _mutex1[MAX_EXT_API_CONNECTIONS];
MUTEX_HANDLE _mutex1Aux[MAX_EXT_API_CONNECTIONS];
simxInt _mutex1LockLevel[MAX_EXT_API_CONNECTIONS];
THREAD_ID _lock1ThreadId[MAX_EXT_API_CONNECTIONS];

MUTEX_HANDLE _mutex2[MAX_EXT_API_CONNECTIONS];
MUTEX_HANDLE _mutex2Aux[MAX_EXT_API_CONNECTIONS];
simxInt _mutex2LockLevel[MAX_EXT_API_CONNECTIONS];
THREAD_ID _lock2ThreadId[MAX_EXT_API_CONNECTIONS];

SOCKET _socketConn[MAX_EXT_API_CONNECTIONS];
struct sockaddr_in _socketServer[MAX_EXT_API_CONNECTIONS];

#ifdef USE_ALSO_SHARED_MEMORY
    #ifdef _WIN32

    #elif defined (__linux) || defined (__APPLE__)

    #endif
#endif

simxShort extApi_endianConversionShort(simxShort shortValue)
{ /* just used for testing purposes. Endianness is detected on the server side */
#ifdef ENDIAN_TEST
    simxShort retV;
    ((char*)&retV)[0]=((char*)&shortValue)[1];
    ((char*)&retV)[1]=((char*)&shortValue)[0];
    return(retV);
#else
    return(shortValue);
#endif
}

simxUShort extApi_endianConversionUShort(simxUShort shortValue)
{ /* just used for testing purposes. Endianness is detected on the server side */
#ifdef ENDIAN_TEST
    simxUShort retV;
    ((char*)&retV)[0]=((char*)&shortValue)[1];
    ((char*)&retV)[1]=((char*)&shortValue)[0];
    return(retV);
#else
    return(shortValue);
#endif
}

simxInt extApi_endianConversionInt(simxInt intValue)
{ /* just used for testing purposes. Endianness is detected on the server side */
#ifdef ENDIAN_TEST
    simxInt retV;
    ((char*)&retV)[0]=((char*)&intValue)[3];
    ((char*)&retV)[1]=((char*)&intValue)[2];
    ((char*)&retV)[2]=((char*)&intValue)[1];
    ((char*)&retV)[3]=((char*)&intValue)[0];
    return(retV);
#else
    return(intValue);
#endif
}

simxFloat extApi_endianConversionFloat(simxFloat floatValue)
{ /* just used for testing purposes. Endianness is detected on the server side */
#ifdef ENDIAN_TEST  
    simxFloat retV;
    ((char*)&retV)[0]=((char*)&floatValue)[3];
    ((char*)&retV)[1]=((char*)&floatValue)[2];
    ((char*)&retV)[2]=((char*)&floatValue)[1];
    ((char*)&retV)[3]=((char*)&floatValue)[0];
    return(retV);
#else
    return(floatValue);
#endif
}

simxDouble extApi_endianConversionDouble(simxDouble doubleValue)
{ /* just used for testing purposes. Endianness is detected on the server side */
#ifdef ENDIAN_TEST  
    simxDouble retV;
    ((char*)&retV)[0]=((char*)&doubleValue)[7];
    ((char*)&retV)[1]=((char*)&doubleValue)[6];
    ((char*)&retV)[2]=((char*)&doubleValue)[5];
    ((char*)&retV)[3]=((char*)&doubleValue)[4];
    ((char*)&retV)[4]=((char*)&doubleValue)[3];
    ((char*)&retV)[5]=((char*)&doubleValue)[2];
    ((char*)&retV)[6]=((char*)&doubleValue)[1];
    ((char*)&retV)[7]=((char*)&doubleValue)[0];
    return(retV);
#else
    return(doubleValue);
#endif
}

simxInt extApi_getTimeInMs()
{
#ifdef _WIN32
    return(timeGetTime()&0x03ffffff);
#elif defined (__linux) || defined (__APPLE__)
    struct timeval tv;
    DWORD result=0;
    if (gettimeofday(&tv,NULL)==0)
        result=(tv.tv_sec*1000+tv.tv_usec/1000)&0x03ffffff;
    return(result);
#endif
}

simxInt extApi_getTimeDiffInMs(simxInt lastTime)
{
    simxInt currentTime=extApi_getTimeInMs();
    if (currentTime<lastTime)
        return(currentTime+0x03ffffff-lastTime);
    return(currentTime-lastTime);
}

simxVoid extApi_initRand()
{
    srand(extApi_getTimeInMs());
}

simxFloat extApi_rand()
{
    return(((float)rand())/((float)RAND_MAX));
}

simxVoid extApi_sleepMs(simxInt ms)
{
#ifdef _WIN32
    Sleep(ms);
#elif defined (__linux) || defined (__APPLE__)
    usleep(ms*1000);
#endif
}

simxVoid extApi_switchThread()
{ /* or just use a extApi_sleepMs(1) here */
    extApi_sleepMs(1);
/*
#ifdef _WIN32
    extApi_sleepMs(1);
#elif defined (__APPLE__)
    pthread_yield_np();
#elif defined (__linux)
    pthread_yield();
#endif
*/
}

simxUChar extApi_areStringsSame(const simxChar* str1,const simxChar* str2)
{
    if (strcmp(str1,str2)==0)
        return(1);
    return(0);
}

simxInt extApi_getStringLength(const simxChar* str)
{
    return((simxInt)strlen(str));
}

simxUChar* extApi_readFile(const simxChar* fileName,simxInt* len)
{
    FILE *file;
    unsigned long fileLength;
    simxUChar* retVal=0;
    file=fopen(fileName,"rb");
    len[0]=0;
    if (file)
    {
        fseek(file,0,SEEK_END);
        fileLength=ftell(file);
        fseek(file,0,SEEK_SET);
        retVal=extApi_allocateBuffer(fileLength);
        fread(retVal,fileLength,1,file);
        fclose(file);
        len[0]=fileLength;
    }
    return(retVal);
}

simxFloat extApi_getFloatFromPtr(const simxUChar* ptr)
{ /* To avoid BUS ERROR on some processors, when the data is not properly aligned (thanks to Scott Hissam) */
    simxFloat retVal;
    memcpy (&retVal,ptr,sizeof(simxFloat));
    return(retVal);
}

simxInt extApi_getIntFromPtr(const simxUChar* ptr)
{ /* To avoid BUS ERROR on some processors, when the data is not properly aligned (thanks to Scott Hissam) */
    simxInt retVal;
    memcpy (&retVal,ptr,sizeof(simxInt));
    return(retVal);
}

simxUChar* extApi_allocateBuffer(simxInt bufferSize)
{
    return ((simxUChar*) (malloc(bufferSize)));
}

simxVoid extApi_releaseBuffer(simxUChar* buffer)
{
    free(buffer);
}

simxVoid extApi_createMutexes(simxInt clientID)
{
#ifdef _WIN32
    _mutex1[clientID]=CreateMutex(0,FALSE,0);
    _mutex1Aux[clientID]=CreateMutex(0,FALSE,0);
    _mutex2[clientID]=CreateMutex(0,FALSE,0);
    _mutex2Aux[clientID]=CreateMutex(0,FALSE,0);
#elif defined (__linux) || defined (__APPLE__)
    pthread_mutex_init(&_mutex1[clientID],0);
    pthread_mutex_init(&_mutex1Aux[clientID],0);
    pthread_mutex_init(&_mutex2[clientID],0);
    pthread_mutex_init(&_mutex2Aux[clientID],0);
#endif
    _mutex1LockLevel[clientID]=0;
    _mutex2LockLevel[clientID]=0;
}

simxVoid extApi_deleteMutexes(simxInt clientID)
{
#ifdef _WIN32
    CloseHandle(_mutex2Aux[clientID]);
    CloseHandle(_mutex2[clientID]);
    CloseHandle(_mutex1Aux[clientID]);
    CloseHandle(_mutex1[clientID]);
#elif defined (__linux) || defined (__APPLE__)
    pthread_mutex_destroy(&_mutex2Aux[clientID]);
    pthread_mutex_destroy(&_mutex2[clientID]);
    pthread_mutex_destroy(&_mutex1Aux[clientID]);
    pthread_mutex_destroy(&_mutex1[clientID]);
#endif
}

simxVoid _simpleLock(MUTEX_HANDLE_X mutex)
{
#ifdef _WIN32
    while (WaitForSingleObject(mutex,INFINITE)!=WAIT_OBJECT_0)
        extApi_switchThread();
#elif defined (__linux) || defined (__APPLE__)
    while (pthread_mutex_lock(mutex)==-1)
        extApi_switchThread();
#endif
}

simxVoid _simpleUnlock(MUTEX_HANDLE_X mutex)
{
#ifdef _WIN32
    ReleaseMutex(mutex);
#elif defined (__linux) || defined (__APPLE__)
    pthread_mutex_unlock(mutex);
#endif
}

simxVoid extApi_createGlobalMutex()
{
#ifdef _WIN32
    _globalMutex=CreateMutex(0,FALSE,0);
#elif defined (__linux) || defined (__APPLE__)
    pthread_mutex_init(&_globalMutex,0);
#endif
}

simxVoid extApi_deleteGlobalMutex()
{
#ifdef _WIN32
    CloseHandle(_globalMutex);
#elif defined (__linux) || defined (__APPLE__)
    pthread_mutex_destroy(&_globalMutex);
#endif
}

simxVoid extApi_globalSimpleLock()
{
#ifdef _WIN32
    _simpleLock(_globalMutex);
#elif defined (__linux) || defined (__APPLE__)
    _simpleLock(&_globalMutex);
#endif
}

simxVoid extApi_globalSimpleUnlock()
{
#ifdef _WIN32
    _simpleUnlock(_globalMutex);
#elif defined (__linux) || defined (__APPLE__)
    _simpleUnlock(&_globalMutex);
#endif
}

simxVoid extApi_lockResources(simxInt clientID)
{
#ifdef _WIN32
    _simpleLock(_mutex1Aux[clientID]);
    if ( (GetCurrentThreadId()==_lock1ThreadId[clientID]) && (_mutex1LockLevel[clientID]>0) )
    { // Already locked by this thread
        _mutex1LockLevel[clientID]++;
        _simpleUnlock(_mutex1Aux[clientID]);
        return;
    }
    // First level lock
    _simpleUnlock(_mutex1Aux[clientID]);
    _simpleLock(_mutex1[clientID]);
    _simpleLock(_mutex1Aux[clientID]);
    _lock1ThreadId[clientID]=GetCurrentThreadId();
    _mutex1LockLevel[clientID]=1;
    _simpleUnlock(_mutex1Aux[clientID]);
#elif defined (__linux) || defined (__APPLE__)
    _simpleLock(&_mutex1Aux[clientID]);
    if ( (pthread_self()==_lock1ThreadId[clientID]) && (_mutex1LockLevel[clientID]>0) )
    { // Already locked by this thread
        _mutex1LockLevel[clientID]++;
        _simpleUnlock(&_mutex1Aux[clientID]);
        return;
    }
    // First level lock
    _simpleUnlock(&_mutex1Aux[clientID]);
    _simpleLock(&_mutex1[clientID]);
    _simpleLock(&_mutex1Aux[clientID]);
    _lock1ThreadId[clientID]=pthread_self();
    _mutex1LockLevel[clientID]=1;
    _simpleUnlock(&_mutex1Aux[clientID]);
#endif
}

simxVoid extApi_unlockResources(simxInt clientID)
{
#ifdef _WIN32
    _simpleLock(_mutex1Aux[clientID]);
    _mutex1LockLevel[clientID]--;
    if (_mutex1LockLevel[clientID]==0)
    {
        _simpleUnlock(_mutex1Aux[clientID]);
        _simpleUnlock(_mutex1[clientID]);
    }
    else
        _simpleUnlock(_mutex1Aux[clientID]);
#elif defined (__linux) || defined (__APPLE__)
    _simpleLock(&_mutex1Aux[clientID]);
    _mutex1LockLevel[clientID]--;
    if (_mutex1LockLevel[clientID]==0)
    {
        _simpleUnlock(&_mutex1Aux[clientID]);
        _simpleUnlock(&_mutex1[clientID]);
    }
    else
        _simpleUnlock(&_mutex1Aux[clientID]);
#endif
}

simxVoid extApi_lockSendStart(simxInt clientID)
{
#ifdef _WIN32
    _simpleLock(_mutex2Aux[clientID]);
    if ( (GetCurrentThreadId()==_lock2ThreadId[clientID]) && (_mutex2LockLevel[clientID]>0) )
    { // Already locked by this thread
        _mutex2LockLevel[clientID]++;
        _simpleUnlock(_mutex2Aux[clientID]);
        return;
    }
    // First level lock
    _simpleUnlock(_mutex2Aux[clientID]);
    _simpleLock(_mutex2[clientID]);
    _simpleLock(_mutex2Aux[clientID]);
    _lock2ThreadId[clientID]=GetCurrentThreadId();
    _mutex2LockLevel[clientID]=1;
    _simpleUnlock(_mutex2Aux[clientID]);
#elif defined (__linux) || defined (__APPLE__)
    _simpleLock(&_mutex2Aux[clientID]);
    if ( (pthread_self()==_lock2ThreadId[clientID]) && (_mutex2LockLevel[clientID]>0) )
    { // Already locked by this thread
        _mutex2LockLevel[clientID]++;
        _simpleUnlock(&_mutex2Aux[clientID]);
        return;
    }
    // First level lock
    _simpleUnlock(&_mutex2Aux[clientID]);
    _simpleLock(&_mutex2[clientID]);
    _simpleLock(&_mutex2Aux[clientID]);
    _lock2ThreadId[clientID]=pthread_self();
    _mutex2LockLevel[clientID]=1;
    _simpleUnlock(&_mutex2Aux[clientID]);
#endif
}

simxVoid extApi_unlockSendStart(simxInt clientID)
{
#ifdef _WIN32
    _simpleLock(_mutex2Aux[clientID]);
    _mutex2LockLevel[clientID]--;
    if (_mutex2LockLevel[clientID]==0)
    {
        _simpleUnlock(_mutex2Aux[clientID]);
        _simpleUnlock(_mutex2[clientID]);
    }
    else
        _simpleUnlock(_mutex2Aux[clientID]);
#elif defined (__linux) || defined (__APPLE__)
    _simpleLock(&_mutex2Aux[clientID]);
    _mutex2LockLevel[clientID]--;
    if (_mutex2LockLevel[clientID]==0)
    {
        _simpleUnlock(&_mutex2Aux[clientID]);
        _simpleUnlock(&_mutex2[clientID]);
    }
    else
        _simpleUnlock(&_mutex2Aux[clientID]);
#endif
}

simxUChar extApi_launchThread(SIMX_THREAD_RET_TYPE(*startAddress)(simxVoid*))
{
#ifdef _WIN32
    return(_beginthread(startAddress,0,0)!=0);
#elif defined (__linux) || defined (__APPLE__)
    pthread_t th;
    return (pthread_create(&th,NULL,startAddress,NULL) == 0);
#endif
}

simxUChar extApi_connectToServer_socket(simxInt clientID,const simxChar* theConnectionAddress,simxInt theConnectionPort)
{ /* return 1: success */
    /* struct hostent *hp;
    simxUInt addr; */
#ifdef _WIN32
    if (WSAStartup(0x101,&_socketWsaData)!=0)
        return(0);
#endif
    _socketConn[clientID]=socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
    if(_socketConn[clientID]==INVALID_SOCKET)
    {
#ifdef _WIN32
        WSACleanup();
#endif
        return(0);
    }
    /*
    Following code can be problematic since some IP Addresses can't be resolved:
    if (inet_addr(theConnectionAddress)==INADDR_NONE)
        hp=gethostbyname(theConnectionAddress);
    else
    {
        addr=inet_addr(theConnectionAddress);
        hp=gethostbyaddr((char*)&addr,sizeof(addr),AF_INET);
    }
    if(hp==NULL)
    {
#ifdef _WIN32
        closesocket(_socketConn[clientID]);
        WSACleanup();
#elif defined (__linux) || defined (__APPLE__)
        close(_socketConn[clientID]);
#endif
        return(0);
    }
    _socketServer[clientID].sin_addr.s_addr=*((unsigned long*)hp->h_addr);
    */

    /* Above code replaced with: */
    _socketServer[clientID].sin_addr.s_addr=inet_addr(theConnectionAddress);



    _socketServer[clientID].sin_family=AF_INET;
    _socketServer[clientID].sin_port=htons(theConnectionPort);
    if(connect(_socketConn[clientID],(struct sockaddr*)&_socketServer[clientID],sizeof(_socketServer[clientID])))
    {
#ifdef _WIN32
        closesocket(_socketConn[clientID]);
        WSACleanup();
#elif defined (__linux) || defined (__APPLE__)
        close(_socketConn[clientID]);
#endif
        return(0);
    }
    return(1);
}

simxVoid extApi_cleanUp_socket(simxInt clientID)
{
#ifdef _WIN32
    closesocket(_socketConn[clientID]);
    WSACleanup();
#elif defined (__linux) || defined (__APPLE__)
    close(_socketConn[clientID]);
#endif
}

simxInt extApi_send_socket(simxInt clientID,const simxUChar* data,simxInt dataLength)
{
    return(send(_socketConn[clientID],(char*)data,dataLength,0));
}

simxInt extApi_recv_socket(simxInt clientID,simxUChar* data,simxInt maxDataLength)
{
    return(recv(_socketConn[clientID],(char*)data,maxDataLength,0));
}



#ifdef USE_ALSO_SHARED_MEMORY
simxUChar extApi_connectToServer_sharedMem(simxInt clientID,simxInt theConnectionPort)
{ /* return 1: success */
#ifdef _WIN32
    HANDLE memoryMapedFile;
    simxUChar* buff;
    simxChar theName[27];
    simxChar* _theName="Local\\VREP_REMOTE_API00000";
    theConnectionPort=-theConnectionPort;
    memcpy(theName,_theName,27);
    theName[21]=(simxChar)(48+(theConnectionPort/10000));
    theConnectionPort=theConnectionPort-(theConnectionPort/10000)*10000;
    theName[22]=(simxChar)(48+(theConnectionPort/1000));
    theConnectionPort=theConnectionPort-(theConnectionPort/1000)*1000;
    theName[23]=(simxChar)(48+(theConnectionPort/100));
    theConnectionPort=theConnectionPort-(theConnectionPort/100)*100;
    theName[24]=(simxChar)(48+(theConnectionPort/10));
    theConnectionPort=theConnectionPort-(theConnectionPort/10)*10;
    theName[25]=(simxChar)(48+theConnectionPort);
    memoryMapedFile=OpenFileMapping(FILE_MAP_ALL_ACCESS,FALSE,theName);   
    if (memoryMapedFile!=NULL)
    {
        buff=(simxUChar*)MapViewOfFile(memoryMapedFile,FILE_MAP_ALL_ACCESS,0,0,5);
        if (buff!=NULL)
        {
            if (buff[0]==0)
            {
                _mmfConn[clientID]=memoryMapedFile;
                _mmfSize[clientID]=((simxInt*)(buff+1))[0];
                buff[5]=0; /* client has nothing to send */
                buff[0]=1; /* connected */
                UnmapViewOfFile(buff);
                return(1);
            }
            UnmapViewOfFile(buff);
            CloseHandle(memoryMapedFile);
            return(0);
        }
        else
            CloseHandle(memoryMapedFile);
        return(0);
    }
    return(0);
#elif defined (__linux) || defined (__APPLE__)
    return(0);
#endif
}

simxVoid extApi_cleanUp_sharedMem(simxInt clientID)
{
#ifdef _WIN32
    simxUChar* buff;
    buff=(simxUChar*)MapViewOfFile(_mmfConn[clientID],FILE_MAP_ALL_ACCESS,0,0,_mmfSize[clientID]+20);
    if (buff!=0)
    {
        buff[0]=0;
        UnmapViewOfFile(buff);
    }
    CloseHandle(_mmfConn[clientID]);
#elif defined (__linux) || defined (__APPLE__)

#endif
}

simxInt extApi_send_sharedMem(simxInt clientID,const simxUChar* data,simxInt dataLength)
{
    simxUChar* buff;
    simxInt startTime;
    simxInt off=0;
    simxInt initDataLength=dataLength;
    if (dataLength==0)
        return(0);
#ifdef _WIN32
        startTime=extApi_getTimeInMs();
    buff=(simxUChar*)MapViewOfFile(_mmfConn[clientID],FILE_MAP_ALL_ACCESS,0,0,_mmfSize[clientID]+20);
    if (buff!=0)
    {
        if (buff[0]!=1)
        {
            UnmapViewOfFile(buff);
            return(0);
        }

        while (dataLength>0)
        {
            /* Wait for previous data to be gone: */
            while (buff[5]!=0)
            {
                if (extApi_getTimeDiffInMs(startTime)>1000)
                {
                    UnmapViewOfFile(buff);
                    return(0);
                }
            }
            /* ok, we can send the data: */
            if (dataLength<=_mmfSize[clientID])
            { /* we can send the data in one shot: */
                memcpy(buff+20,data+off,dataLength);
                ((int*)(buff+6))[0]=dataLength;
                ((int*)(buff+6))[1]=20;
                ((int*)(buff+6))[2]=initDataLength;
                dataLength=0;
            }
            else
            { /* just send a smaller part first: */
                memcpy(buff+20,data+off,_mmfSize[clientID]);
                ((int*)(buff+6))[0]=_mmfSize[clientID];
                ((int*)(buff+6))[1]=20;
                ((int*)(buff+6))[2]=initDataLength;
                dataLength-=(_mmfSize[clientID]);
                off+=(_mmfSize[clientID]);
            }
            buff[5]=1; /* client has something to send! */
        }
        UnmapViewOfFile(buff);
        return(initDataLength);
    }
    return(0);
#elif defined (__linux) || defined (__APPLE__)
    return(0);
#endif
}

simxUChar* extApi_recv_sharedMem(simxInt clientID,simxInt* dataLength)
{
    simxUChar* buff;
    simxInt startT;
    simxInt l=0;
    simxInt off=0;
    simxInt retDataOff=0;
    simxUChar* retData=0;
    simxInt totalLength=-1;
#ifdef _WIN32
    buff=(simxUChar*)MapViewOfFile(_mmfConn[clientID],FILE_MAP_ALL_ACCESS,0,0,_mmfSize[clientID]+20);
    if (buff!=0)
    {
        if (buff[0]!=1)
        { /* we are not connected anymore */
            UnmapViewOfFile(buff);
            return(0);
        }

        startT=extApi_getTimeInMs();
        while (retDataOff!=totalLength)
        {
            /* Wait for data: */
            while (buff[5]!=2)
            {
                if (extApi_getTimeDiffInMs(startT)>1000)
                {
                    UnmapViewOfFile(buff);
                    return(0);
                }
            }
            /* ok, data is there! */
            /* Read the data with correct length: */
            l=((int*)(buff+6))[0];
            off=((int*)(buff+6))[1];
            totalLength=((int*)(buff+6))[2];
            if (retData==0)
                retData=extApi_allocateBuffer(totalLength);
            memcpy(retData+retDataOff,buff+off,l);
            retDataOff=retDataOff+l;
            /* Tell the other side we have read that part and additional parts could be sent (if present): */
            buff[5]=0;
        }
        UnmapViewOfFile(buff);
        dataLength[0]=retDataOff;
        return(retData);
    }
    return(0);
#elif defined (__linux) || defined (__APPLE__)
    return(0);
#endif
}
#endif
