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

#include "extApi.h"
#include "extApiInternal.h"
#include <stdio.h>

#ifdef _Included_extApiJava
    #include "extApiJava.h"
#endif

simxUChar _wholeThingInitialized=0;
simxInt _clientsCount=0;
simxInt _clientIDForThread;

simxUChar _softLock_=0;
simxVoid _softLock()
{ /* a bit delicate, since we can't use mutexes yet... very simple non fail-safe lock! */
    while (1)
    {
        if (_softLock_==0)
        {
            _softLock_=1;
            return;
        }
        extApi_switchThread();
    }
}

simxVoid _softUnlock()
{ /* a bit delicate, since we can't use mutexes yet... very simple non fail-safe unlock! */
    _softLock_=0;
}

simxVoid _increaseClientCount()
{ 
    simxInt i;
    _softLock(); /* simple and not fail-safe. Init/deinit routines would probably be better... */
    if (_wholeThingInitialized==0)
    { 
        _wholeThingInitialized=1;
        for (i=0;i<MAX_EXT_API_CONNECTIONS;i++)
        {
            _nextConnectionID[i]=0;
            _replyWaitTimeoutInMs[i]=_REPLY_WAIT_TIMEOUT_IN_MS;
        }
        extApi_createGlobalMutex();
    }
    _softUnlock();
    extApi_globalSimpleLock();
    _clientsCount=_clientsCount+1;
    extApi_globalSimpleUnlock();
}

simxVoid _decreaseClientCount(simxUChar useSoftLock)
{
    if (useSoftLock!=0)
        _softLock(); /* simple and not fail-safe. Init/deinit routines would probably be better... */
    if (_clientsCount>0)
    {
        extApi_globalSimpleLock();
        _clientsCount=_clientsCount-1;
        extApi_globalSimpleUnlock();
        if (_clientsCount==0)
        {
            extApi_deleteGlobalMutex();
            _wholeThingInitialized=0;
        }
    }
    if (useSoftLock!=0)
        _softUnlock();
}

EXTAPI_DLLEXPORT simxInt simxStart(const simxChar* connectionAddress,simxInt connectionPort,simxUChar waitUntilConnected,simxUChar doNotReconnectOnceDisconnected,simxInt timeOutInMs,simxInt commThreadCycleInMs)
{
    simxInt startTime,i,clientID;

#ifndef USE_ALSO_SHARED_MEMORY
    if (connectionPort<0)
        return(-1);
#endif

    _increaseClientCount();
    extApi_initRand();
    clientID=-1;
    extApi_globalSimpleLock();
    for (i=0;i<MAX_EXT_API_CONNECTIONS;i++)
    {
        if (_nextConnectionID[i]==0)
        {
            clientID=i;
            break;
        }
    }

    if (clientID!=-1)
    {
        for (i=0;i<MAX_EXT_API_CONNECTIONS;i++)
        {
            if (_nextConnectionID[i]!=0)
            {
                if (connectionPort<0)
                { /* using shared memory */
                    if (connectionPort==_connectionPort[i])
                    { /* that 'shared memory number' was already used */
                        clientID=-1;
                        break;
                    }
                }
                else
                { /* using sockets */
                    if ( (connectionPort==_connectionPort[i])&&(extApi_areStringsSame(_connectionIP[i],connectionAddress)!=0) )
                    { /* that IP/port was already used */
                        clientID=-1;
                        break;
                    }
                }
            }
        }
    }
    extApi_globalSimpleUnlock();
    if (clientID==-1)
    {
        _decreaseClientCount(1);
        return(-1);
    }
    _nextConnectionID[clientID]=1;
    _connectionPort[clientID]=connectionPort;

    if (connectionPort<0)
    { /* using shared memory */
        _connectionIP[clientID]=(simxChar*)extApi_allocateBuffer(2+1);
        _connectionIP[clientID][0]='@';
        _connectionIP[clientID][1]='P';
        _connectionIP[clientID][2]=0;
    }
    else
    { /* using sockets */
        _connectionIP[clientID]=(simxChar*)extApi_allocateBuffer(extApi_getStringLength(connectionAddress)+1);
        for (i=0;i<extApi_getStringLength(connectionAddress)+1;i++)
            _connectionIP[clientID][i]=connectionAddress[i];
    }

    /* Prepare various buffers */
    _messageToSend[clientID]=extApi_allocateBuffer(SIMX_INIT_BUFF_SIZE);
    _messageToSend_bufferSize[clientID]=SIMX_INIT_BUFF_SIZE;
    _messageToSend_dataSize[clientID]=SIMX_HEADER_SIZE;

    _splitCommandsToSend[clientID]=extApi_allocateBuffer(SIMX_INIT_BUFF_SIZE);
    _splitCommandsToSend_bufferSize[clientID]=SIMX_INIT_BUFF_SIZE;
    _splitCommandsToSend_dataSize[clientID]=0;

    _messageReceived[clientID]=extApi_allocateBuffer(SIMX_INIT_BUFF_SIZE);
    _messageReceived_bufferSize[clientID]=SIMX_INIT_BUFF_SIZE;
    _messageReceived_dataSize[clientID]=0;

    _splitCommandsReceived[clientID]=extApi_allocateBuffer(SIMX_INIT_BUFF_SIZE);
    _splitCommandsReceived_bufferSize[clientID]=SIMX_INIT_BUFF_SIZE;
    _splitCommandsReceived_dataSize[clientID]=0;

    _commandReceived[clientID]=extApi_allocateBuffer(SIMX_INIT_BUFF_SIZE);
    _commandReceived_bufferSize[clientID]=SIMX_INIT_BUFF_SIZE;
    _commandReceived_simulationTime[clientID]=0;

    _nextMessageIDToSend[clientID]=0;
    _lastReceivedMessageID[clientID]=-1;
    _waitBeforeSendingAgainWhenMessageIDArrived[clientID]=-1; /* do not wait */
    _minCommunicationDelay[clientID]=commThreadCycleInMs;

    if (timeOutInMs<0)
    { /* a positive time out is for the first connection. A negative, for all blocking commands */
        _replyWaitTimeoutInMs[clientID]=-timeOutInMs;
        timeOutInMs=5000;
    }

    extApi_createMutexes(clientID);

    /* Launch the socket/shared memory communication thread */
    _communicationThreadRunning[clientID]=1;
    _connectionID[clientID]=-1;
    _tempConnectionAddress[clientID]=connectionAddress;
    _tempConnectionPort[clientID]=connectionPort;
    _tempDoNotReconnectOnceDisconnected[clientID]=doNotReconnectOnceDisconnected;

    extApi_globalSimpleLock();
    _clientIDForThread=clientID;
    extApi_launchThread(_communicationThread);
    while (_clientIDForThread!=-1)
        extApi_switchThread(); /* wait until the thread is set */
    extApi_globalSimpleUnlock();

    if (waitUntilConnected==0)
        return(clientID); /* we do not wait until connected */

    startTime=extApi_getTimeInMs();
    while ( (extApi_getTimeDiffInMs(startTime)<timeOutInMs)&&(_connectionID[clientID]==-1) )
        extApi_switchThread();
    if (_connectionID[clientID]==-1)
    { /* we failed connecting */
        simxFinish(clientID);
        return(-1);
    }
    return(clientID);
}

EXTAPI_DLLEXPORT simxVoid simxFinish(simxInt clientID)
{
    simxInt returnValue,i;
    if ((clientID<-1)||(clientID>=MAX_EXT_API_CONNECTIONS))
        return;

    _softLock();
    if (_clientsCount<=0)
    {
        _softUnlock();
        return;
    }

    if (clientID>=0)
    { /* shut down a specific client */
        if (_communicationThreadRunning[clientID])
        {

            if (_connectionID[clientID]!=-1)
            { /* we are still connected */
                /* send the kill connection command */
                _exec_int(clientID,simx_cmd_kill_connection,simx_opmode_oneshot,0,0,&returnValue);
                /* wait 0.5 seconds */
                extApi_sleepMs(500);
            }

            /* now tell the communication thread the end and wait until it's done: */
            _communicationThreadRunning[clientID]=0;
            while (_communicationThreadRunning[clientID]==0)
                extApi_switchThread();
            _communicationThreadRunning[clientID]=0;

            /* do some clean-up: */
            extApi_deleteMutexes(clientID);
            extApi_releaseBuffer(_commandReceived[clientID]);
            extApi_releaseBuffer(_splitCommandsReceived[clientID]);
            extApi_releaseBuffer(_messageToSend[clientID]);
            extApi_releaseBuffer(_splitCommandsToSend[clientID]);
            extApi_releaseBuffer(_messageReceived[clientID]);
            extApi_releaseBuffer((simxUChar*)_connectionIP[clientID]);
            _nextConnectionID[clientID]=0; /* slot becomes free */

            _decreaseClientCount(0);
        }
    }
    else
    { /* shut down all opened clients */
        for (i=0;i<MAX_EXT_API_CONNECTIONS;i++)
        {
            if (_communicationThreadRunning[i])
            {
                if (_connectionID[i]!=-1)
                { /* we are still connected */
                    /* send the kill connection command */
                    _exec_int(i,simx_cmd_kill_connection,simx_opmode_oneshot,0,0,&returnValue);
                }
            }
        }

        /* wait 0.5 seconds */
        extApi_sleepMs(500);    

        for (i=0;i<MAX_EXT_API_CONNECTIONS;i++)
        {
            if (_communicationThreadRunning[i])
            {

                /* now tell the communication thread to end and wait until it's done: */
                _communicationThreadRunning[i]=0;
                while (_communicationThreadRunning[i]==0)
                    extApi_switchThread();
                _communicationThreadRunning[i]=0;

                /* do some clean-up: */
                extApi_deleteMutexes(i);
                extApi_releaseBuffer(_commandReceived[i]);
                extApi_releaseBuffer(_splitCommandsReceived[i]);
                extApi_releaseBuffer(_messageToSend[i]);
                extApi_releaseBuffer(_splitCommandsToSend[i]);
                extApi_releaseBuffer(_messageReceived[i]);
                extApi_releaseBuffer((simxUChar*)_connectionIP[i]);
                _nextConnectionID[i]=0; /* slot becomes free */

                _decreaseClientCount(0);
            }
        }
    }
    _softUnlock();
}

simxVoid _waitUntilMessageArrived(simxInt clientID,simxInt* error)
{
    simxInt startTime;
    simxInt lastReceivedMessageIDCopy;
    if (_waitBeforeSendingAgainWhenMessageIDArrived[clientID]!=-1)
    { /* wait until we received a reply, or a timeout (if we wanna wait for the reply) */
        startTime=extApi_getTimeInMs();
        while (1)
        {
            extApi_lockResources(clientID);
            lastReceivedMessageIDCopy=_lastReceivedMessageID[clientID];
            extApi_unlockResources(clientID);
            if ((extApi_getTimeDiffInMs(startTime)>=_replyWaitTimeoutInMs[clientID])||(lastReceivedMessageIDCopy>=_waitBeforeSendingAgainWhenMessageIDArrived[clientID]))
                break;
            extApi_switchThread();
        }
        if (lastReceivedMessageIDCopy<_waitBeforeSendingAgainWhenMessageIDArrived[clientID])
            error[0]|=simx_return_timeout_flag;
    }
}

simxUChar* _setLastFetchedCmd(simxInt clientID,simxUChar* cmdPtr,simxInt* error)
{
    simxInt blockSize,incr,i,status;
    simxUChar* newCommand;
    if (cmdPtr!=0)
    {
        blockSize=extApi_endianConversionInt(((simxInt*)(cmdPtr+simx_cmdheaderoffset_mem_size))[0]);
        if (blockSize>_commandReceived_bufferSize[clientID])
        { /* we need more memory here for the fetched command buffer */
            incr=blockSize-_commandReceived_bufferSize[clientID];
            if (incr<SIMX_MIN_BUFF_INCR)
                incr=SIMX_MIN_BUFF_INCR;
            newCommand=(simxUChar*)extApi_allocateBuffer(_commandReceived_bufferSize[clientID]+incr);
            extApi_releaseBuffer(_commandReceived[clientID]);
            _commandReceived[clientID]=newCommand;
            _commandReceived_bufferSize[clientID]+=incr;
        }
        for (i=0;i<blockSize;i++)
            _commandReceived[clientID][i]=cmdPtr[i];
        cmdPtr=_commandReceived[clientID];
        status=cmdPtr[simx_cmdheaderoffset_status];
        _commandReceived_simulationTime[clientID]=extApi_endianConversionInt(((simxInt*)(cmdPtr+simx_cmdheaderoffset_sim_time))[0]);
        if (status&1)
            error[0]|=simx_return_remote_error_flag; /* command caused an error on the server side */
    }
    else
        error[0]|=simx_return_novalue_flag;
    return(cmdPtr);
}


simxUChar* _exec_null(simxInt clientID,simxInt cmdRaw,simxInt opMode,simxUChar options,simxInt* error)
{
    simxUShort delayOrSplit;
    simxUChar* cmdPtr=0;

    error[0]=simx_return_ok;
    delayOrSplit=opMode&simx_cmdmask;
    opMode-=(opMode&simx_cmdmask);

    if (opMode!=simx_opmode_buffer)
    {
        /* Check if the command is already present in the request list */
        extApi_lockResources(clientID);
        if (opMode==simx_opmode_oneshot_split)
        {
            if (delayOrSplit<_MIN_SPLIT_AMOUNT_IN_BYTES)
                delayOrSplit=_MIN_SPLIT_AMOUNT_IN_BYTES;
            cmdPtr=_getCommandPointer_(cmdRaw,_splitCommandsToSend[clientID],_splitCommandsToSend_dataSize[clientID]);
            if (cmdPtr!=0)
                error[0]|=simx_return_split_progress_flag; /* Command already there */
            else
            { /* Command not there. Add it */
                _splitCommandsToSend[clientID]=_appendCommand_(cmdRaw+opMode,options,delayOrSplit,_splitCommandsToSend[clientID],&_splitCommandsToSend_bufferSize[clientID],&_splitCommandsToSend_dataSize[clientID]);
            }
        }
        else
        {
            cmdPtr=_getCommandPointer_(cmdRaw,_messageToSend[clientID]+SIMX_HEADER_SIZE,_messageToSend_dataSize[clientID]-SIMX_HEADER_SIZE);
            if ((cmdPtr==0)||(options&1))
            { /* Command not there (or cmd cannot be overwritten). Add it */
                _messageToSend[clientID]=_appendCommand_(cmdRaw+opMode,options,delayOrSplit,_messageToSend[clientID],&_messageToSend_bufferSize[clientID],&_messageToSend_dataSize[clientID]);
            }
        }

        if (opMode==simx_opmode_blocking)
            _waitBeforeSendingAgainWhenMessageIDArrived[clientID]=_nextMessageIDToSend[clientID];
        extApi_unlockResources(clientID);

        /* wait until we received a reply, or a timeout (if we wanna wait for the reply) */
        if (_waitBeforeSendingAgainWhenMessageIDArrived[clientID]!=-1)
            _waitUntilMessageArrived(clientID,error); 
    }

    /* Check if the command is present in the input list */
    extApi_lockResources(clientID);
    cmdPtr=_getCommandPointer_(cmdRaw,_messageReceived[clientID]+SIMX_HEADER_SIZE,_messageReceived_dataSize[clientID]-SIMX_HEADER_SIZE);
    cmdPtr=_setLastFetchedCmd(clientID,cmdPtr,error);
    _waitBeforeSendingAgainWhenMessageIDArrived[clientID]=-1; /* make sure to enable the communication thread again! */
    extApi_unlockResources(clientID);
    if (opMode==simx_opmode_blocking) /* A cmd reply stays in the inbox always.. except when the mode is simx_opmode_blocking (to avoid polluting the inbox) */
        _removeCommandReply_null(clientID,cmdRaw);
    return(cmdPtr);
}


simxUChar* _exec_null_buffer(simxInt clientID,simxInt cmdRaw,simxInt opMode,simxUChar options,simxUChar* buffer,simxInt bufferSize,simxInt* error)
{
    simxUShort delayOrSplit;
    simxUChar* cmdPtr=0;

    error[0]=simx_return_ok;

    delayOrSplit=opMode&simx_cmdmask;
    opMode-=(opMode&simx_cmdmask);

    if (opMode!=simx_opmode_buffer)
    {
        /* Check if the command is already present in the request list */
        extApi_lockResources(clientID);
        if (opMode==simx_opmode_oneshot_split)
        {
            if (delayOrSplit<_MIN_SPLIT_AMOUNT_IN_BYTES)
                delayOrSplit=_MIN_SPLIT_AMOUNT_IN_BYTES;
            cmdPtr=_getCommandPointer_(cmdRaw,_splitCommandsToSend[clientID],_splitCommandsToSend_dataSize[clientID]);
            if (cmdPtr!=0)
            { /* Command already there */
                /* Now make sure we have the same command size, otherwise we have to remove the old cmd and add freshly the new */
                if (extApi_endianConversionInt(((simxInt*)(cmdPtr+simx_cmdheaderoffset_mem_size))[0])==SIMX_SUBHEADER_SIZE+4+bufferSize)
                    error[0]|=simx_return_split_progress_flag; /* ok, we have the same size */
                else
                { /* we don't have the same size! Remove the old command */
                    _removeChunkFromBuffer(_splitCommandsToSend[clientID],cmdPtr,extApi_endianConversionInt(((simxInt*)(cmdPtr+simx_cmdheaderoffset_mem_size))[0]),&_splitCommandsToSend_dataSize[clientID]);
                    cmdPtr=0; /* so that we will add the new command in next section */
                }
            }
            if (cmdPtr==0)
                _splitCommandsToSend[clientID]=_appendCommand_null_buff(cmdRaw+opMode,options,buffer,bufferSize,delayOrSplit,_splitCommandsToSend[clientID],&_splitCommandsToSend_bufferSize[clientID],&_splitCommandsToSend_dataSize[clientID]);
        }
        else
        {
            cmdPtr=_getCommandPointer_(cmdRaw,_messageToSend[clientID]+SIMX_HEADER_SIZE,_messageToSend_dataSize[clientID]-SIMX_HEADER_SIZE);

            if ((cmdPtr!=0)&&((options&1)==0)) /* Command already there, and we can overwrite it. We remove it and add it again */
                _removeChunkFromBuffer(_messageToSend[clientID],cmdPtr,extApi_endianConversionInt(((simxInt*)(cmdPtr+simx_cmdheaderoffset_mem_size))[0]),&_messageToSend_dataSize[clientID]); /* we remove then add the command again (b/c no guarantee the buffer has the same size) */
            _messageToSend[clientID]=_appendCommand_null_buff(cmdRaw+opMode,options,buffer,bufferSize,delayOrSplit,_messageToSend[clientID],&_messageToSend_bufferSize[clientID],&_messageToSend_dataSize[clientID]);
        }

        if (opMode==simx_opmode_blocking)
            _waitBeforeSendingAgainWhenMessageIDArrived[clientID]=_nextMessageIDToSend[clientID];
        extApi_unlockResources(clientID);

        /* wait until we received a reply, or a timeout (if we wanna wait for the reply) */
        if (_waitBeforeSendingAgainWhenMessageIDArrived[clientID]!=-1)
            _waitUntilMessageArrived(clientID,error); 
    }

    /* Check if the command is present in the input list (we might have this situation when we want to check if there was an error on the server side) */
    extApi_lockResources(clientID);
    cmdPtr=_getCommandPointer_(cmdRaw,_messageReceived[clientID]+SIMX_HEADER_SIZE,_messageReceived_dataSize[clientID]-SIMX_HEADER_SIZE);
    cmdPtr=_setLastFetchedCmd(clientID,cmdPtr,error);
    _waitBeforeSendingAgainWhenMessageIDArrived[clientID]=-1; /* make sure to enable the communication thread again! */
    extApi_unlockResources(clientID);
    if (opMode==simx_opmode_blocking) /* A cmd reply stays in the inbox always.. except when the mode is simx_opmode_blocking (to avoid polluting the inbox) */
        _removeCommandReply_null(clientID,cmdRaw);
    return(cmdPtr);
}



simxUChar* _exec_int(simxInt clientID,simxInt cmdRaw,simxInt opMode,simxUChar options,simxInt intValue,simxInt* error)
{
    simxUShort delayOrSplit;
    simxUChar* cmdPtr=0;

    error[0]=simx_return_ok;
    delayOrSplit=opMode&simx_cmdmask;
    opMode-=(opMode&simx_cmdmask);

    if (opMode!=simx_opmode_buffer)
    {
        /* Check if the command is already present in the request list */
        extApi_lockResources(clientID);
        if (opMode==simx_opmode_oneshot_split)
        {
            if (delayOrSplit<_MIN_SPLIT_AMOUNT_IN_BYTES)
                delayOrSplit=_MIN_SPLIT_AMOUNT_IN_BYTES;
            cmdPtr=_getCommandPointer_i(cmdRaw,intValue,_splitCommandsToSend[clientID],_splitCommandsToSend_dataSize[clientID]);
            if (cmdPtr!=0)
                error[0]|=simx_return_split_progress_flag; /* Command already there */
            else
            { /* Command not there. Add it */
                _splitCommandsToSend[clientID]=_appendCommand_i(cmdRaw+opMode,options,intValue,delayOrSplit,_splitCommandsToSend[clientID],&_splitCommandsToSend_bufferSize[clientID],&_splitCommandsToSend_dataSize[clientID]);
            }
        }
        else
        {
            cmdPtr=_getCommandPointer_i(cmdRaw,intValue,_messageToSend[clientID]+SIMX_HEADER_SIZE,_messageToSend_dataSize[clientID]-SIMX_HEADER_SIZE);
            if ((cmdPtr!=0)&&((options&1)==0))
            { /* Command already there, and we can overwrite it. Update it */
                ((simxInt*)(cmdPtr+simx_cmdheaderoffset_cmd))[0]=extApi_endianConversionInt(cmdRaw+opMode);
            }
            else
            { /* Command not there. Add it */
                _messageToSend[clientID]=_appendCommand_i(cmdRaw+opMode,options,intValue,delayOrSplit,_messageToSend[clientID],&_messageToSend_bufferSize[clientID],&_messageToSend_dataSize[clientID]);
            }
        }

        if (opMode==simx_opmode_blocking)
            _waitBeforeSendingAgainWhenMessageIDArrived[clientID]=_nextMessageIDToSend[clientID];
        extApi_unlockResources(clientID);

        /* wait until we received a reply, or a timeout (if we wanna wait for the reply) */
        if (_waitBeforeSendingAgainWhenMessageIDArrived[clientID]!=-1)
            _waitUntilMessageArrived(clientID,error); 
    }

    /* Check if the command is present in the input list */
    extApi_lockResources(clientID);
    cmdPtr=_getCommandPointer_i(cmdRaw,intValue,_messageReceived[clientID]+SIMX_HEADER_SIZE,_messageReceived_dataSize[clientID]-SIMX_HEADER_SIZE);
    cmdPtr=_setLastFetchedCmd(clientID,cmdPtr,error);
    _waitBeforeSendingAgainWhenMessageIDArrived[clientID]=-1; /* make sure to enable the communication thread again! */
    extApi_unlockResources(clientID);
    if (opMode==simx_opmode_blocking) /* A cmd reply stays in the inbox always.. except when the mode is simx_opmode_blocking (to avoid polluting the inbox) */
        _removeCommandReply_int(clientID,cmdRaw,intValue);
    return(cmdPtr);
}

simxUChar* _exec_intint(simxInt clientID,simxInt cmdRaw,simxInt opMode,simxUChar options,simxInt intValue1,simxInt intValue2,simxInt* error)
{
    simxUShort delayOrSplit;
    simxUChar* cmdPtr=0;

    error[0]=simx_return_ok;
    delayOrSplit=opMode&simx_cmdmask;
    opMode-=(opMode&simx_cmdmask);

    if (opMode!=simx_opmode_buffer)
    {
        /* Check if the command is already present in the request list */
        extApi_lockResources(clientID);
        if (opMode==simx_opmode_oneshot_split)
        {
            if (delayOrSplit<_MIN_SPLIT_AMOUNT_IN_BYTES)
                delayOrSplit=_MIN_SPLIT_AMOUNT_IN_BYTES;
            cmdPtr=_getCommandPointer_ii(cmdRaw,intValue1,intValue2,_splitCommandsToSend[clientID],_splitCommandsToSend_dataSize[clientID]);
            if (cmdPtr!=0)
                error[0]|=simx_return_split_progress_flag; /* Command already there */
            else
            { /* Command not there. Add it */
                _splitCommandsToSend[clientID]=_appendCommand_ii(cmdRaw+opMode,options,intValue1,intValue2,delayOrSplit,_splitCommandsToSend[clientID],&_splitCommandsToSend_bufferSize[clientID],&_splitCommandsToSend_dataSize[clientID]);
            }
        }
        else
        {
            cmdPtr=_getCommandPointer_ii(cmdRaw,intValue1,intValue2,_messageToSend[clientID]+SIMX_HEADER_SIZE,_messageToSend_dataSize[clientID]-SIMX_HEADER_SIZE);
            if ((cmdPtr!=0)&&((options&1)==0))
            { /* Command already there, and we can overwrite it. Update it */
                ((simxInt*)(cmdPtr+simx_cmdheaderoffset_cmd))[0]=extApi_endianConversionInt(cmdRaw+opMode);
            }
            else
            { /* Command not there. Add it */
                _messageToSend[clientID]=_appendCommand_ii(cmdRaw+opMode,options,intValue1,intValue2,delayOrSplit,_messageToSend[clientID],&_messageToSend_bufferSize[clientID],&_messageToSend_dataSize[clientID]);
            }
        }

        if (opMode==simx_opmode_blocking)
            _waitBeforeSendingAgainWhenMessageIDArrived[clientID]=_nextMessageIDToSend[clientID];
        extApi_unlockResources(clientID);

        /* wait until we received a reply, or a timeout (if we wanna wait for the reply) */
        if (_waitBeforeSendingAgainWhenMessageIDArrived[clientID]!=-1)
            _waitUntilMessageArrived(clientID,error); 
    }

    /* Check if the command is present in the input list */
    extApi_lockResources(clientID);
    cmdPtr=_getCommandPointer_ii(cmdRaw,intValue1,intValue2,_messageReceived[clientID]+SIMX_HEADER_SIZE,_messageReceived_dataSize[clientID]-SIMX_HEADER_SIZE);
    cmdPtr=_setLastFetchedCmd(clientID,cmdPtr,error);
    _waitBeforeSendingAgainWhenMessageIDArrived[clientID]=-1; /* make sure to enable the communication thread again! */
    extApi_unlockResources(clientID);
    if (opMode==simx_opmode_blocking) /* A cmd reply stays in the inbox always.. except when the mode is simx_opmode_blocking (to avoid polluting the inbox) */
        _removeCommandReply_intint(clientID,cmdRaw,intValue1,intValue2);
    return(cmdPtr);
}

simxUChar* _exec_string(simxInt clientID,simxInt cmdRaw,simxInt opMode,simxUChar options,const simxUChar* stringValue,simxInt* error)
{
    simxUShort delayOrSplit;
    simxUChar* cmdPtr=0;

    error[0]=simx_return_ok;

    delayOrSplit=opMode&simx_cmdmask;
    opMode-=(opMode&simx_cmdmask);

    if (opMode!=simx_opmode_buffer)
    {
        /* Check if the command is already present in the request list */
        extApi_lockResources(clientID);
        if (opMode==simx_opmode_oneshot_split)
        {
            if (delayOrSplit<_MIN_SPLIT_AMOUNT_IN_BYTES)
                delayOrSplit=_MIN_SPLIT_AMOUNT_IN_BYTES;
            cmdPtr=_getCommandPointer_s(cmdRaw,stringValue,_splitCommandsToSend[clientID],_splitCommandsToSend_dataSize[clientID]);
            if (cmdPtr!=0)
            { /* Command already there */
                error[0]|=simx_return_split_progress_flag;
            }
            else
            { /* Command not there. Add it */
                _splitCommandsToSend[clientID]=_appendCommand_s(cmdRaw+opMode,options,stringValue,delayOrSplit,_splitCommandsToSend[clientID],&_splitCommandsToSend_bufferSize[clientID],&_splitCommandsToSend_dataSize[clientID]);
            }
        }
        else
        {
            cmdPtr=_getCommandPointer_s(cmdRaw,stringValue,_messageToSend[clientID]+SIMX_HEADER_SIZE,_messageToSend_dataSize[clientID]-SIMX_HEADER_SIZE);
            if ((cmdPtr!=0)&&((options&1)==0))
            { /* Command already there, and we can overwrite it. Update it */
                ((simxInt*)(cmdPtr+simx_cmdheaderoffset_cmd))[0]=extApi_endianConversionInt(cmdRaw+opMode);
            }
            else
            { /* Command not there. Add it */
                _messageToSend[clientID]=_appendCommand_s(cmdRaw+opMode,options,stringValue,delayOrSplit,_messageToSend[clientID],&_messageToSend_bufferSize[clientID],&_messageToSend_dataSize[clientID]);
            }
        }

        if (opMode==simx_opmode_blocking)
            _waitBeforeSendingAgainWhenMessageIDArrived[clientID]=_nextMessageIDToSend[clientID];
        extApi_unlockResources(clientID);

        /* wait until we received a reply, or a timeout (if we wanna wait for the reply) */
        if (_waitBeforeSendingAgainWhenMessageIDArrived[clientID]!=-1)
            _waitUntilMessageArrived(clientID,error); 
    }

    /* Check if the command is present in the input list */
    extApi_lockResources(clientID);
    cmdPtr=_getCommandPointer_s(cmdRaw,stringValue,_messageReceived[clientID]+SIMX_HEADER_SIZE,_messageReceived_dataSize[clientID]-SIMX_HEADER_SIZE);
    cmdPtr=_setLastFetchedCmd(clientID,cmdPtr,error);
    _waitBeforeSendingAgainWhenMessageIDArrived[clientID]=-1; /* make sure to enable the communication thread again! */
    extApi_unlockResources(clientID);
    if (opMode==simx_opmode_blocking) /* A cmd reply stays in the inbox always.. except when the mode is simx_opmode_blocking (to avoid polluting the inbox) */
        _removeCommandReply_string(clientID,cmdRaw,stringValue);
    return(cmdPtr);
}

simxUChar* _exec_int_int(simxInt clientID,simxInt cmdRaw,simxInt opMode,simxUChar options,simxInt intValue,simxInt intValue2,simxInt* error)
{
    simxUChar* cmdPtr;
    simxUShort delayOrSplit;

    error[0]=simx_return_ok;

    delayOrSplit=opMode&simx_cmdmask;
    opMode-=(opMode&simx_cmdmask);

    if (opMode!=simx_opmode_buffer)
    {
        /* Check if the command is already present in the request list */
        extApi_lockResources(clientID);
        if (opMode==simx_opmode_oneshot_split)
        {
            if (delayOrSplit<_MIN_SPLIT_AMOUNT_IN_BYTES)
                delayOrSplit=_MIN_SPLIT_AMOUNT_IN_BYTES;
            cmdPtr=_getCommandPointer_i(cmdRaw,intValue,_splitCommandsToSend[clientID],_splitCommandsToSend_dataSize[clientID]);
            if ((cmdPtr!=0)&&((options&1)==0))
            { /* Command already there, and we can overwrite it. Update it */
                error[0]|=simx_return_split_progress_flag;
            }
            else
            { /* Command not there. Add it */
                _splitCommandsToSend[clientID]=_appendCommand_i_i(cmdRaw+opMode,options,intValue,intValue2,delayOrSplit,_splitCommandsToSend[clientID],&_splitCommandsToSend_bufferSize[clientID],&_splitCommandsToSend_dataSize[clientID]);
            }
        }
        else
        {
            cmdPtr=_getCommandPointer_i(cmdRaw,intValue,_messageToSend[clientID]+SIMX_HEADER_SIZE,_messageToSend_dataSize[clientID]-SIMX_HEADER_SIZE);
            if (cmdPtr!=0)
            { /* Command already there. Update it */
                ((simxInt*)(cmdPtr+simx_cmdheaderoffset_cmd))[0]=extApi_endianConversionInt(cmdRaw+opMode);
                ((simxInt*)(cmdPtr+SIMX_SUBHEADER_SIZE+4))[0]=extApi_endianConversionInt(intValue2);
            }
            else
            { /* Command not there. Add it */
                _messageToSend[clientID]=_appendCommand_i_i(cmdRaw+opMode,options,intValue,intValue2,delayOrSplit,_messageToSend[clientID],&_messageToSend_bufferSize[clientID],&_messageToSend_dataSize[clientID]);
            }
        }

        if (opMode==simx_opmode_blocking)
            _waitBeforeSendingAgainWhenMessageIDArrived[clientID]=_nextMessageIDToSend[clientID];
        extApi_unlockResources(clientID);

        /* wait until we received a reply, or a timeout (if we wanna wait for the reply) */
        if (_waitBeforeSendingAgainWhenMessageIDArrived[clientID]!=-1)
            _waitUntilMessageArrived(clientID,error); 
    }

    /* Check if the command is present in the input list (we might have this situation when we want to check if there was an error on the server side) */
    extApi_lockResources(clientID);
    cmdPtr=_getCommandPointer_i(cmdRaw,intValue,_messageReceived[clientID]+SIMX_HEADER_SIZE,_messageReceived_dataSize[clientID]-SIMX_HEADER_SIZE);
    cmdPtr=_setLastFetchedCmd(clientID,cmdPtr,error);
    _waitBeforeSendingAgainWhenMessageIDArrived[clientID]=-1; /* make sure to enable the communication thread again! */
    extApi_unlockResources(clientID);
    if (opMode==simx_opmode_blocking) /* A cmd reply stays in the inbox always.. except when the mode is simx_opmode_blocking (to avoid polluting the inbox) */
        _removeCommandReply_int(clientID,cmdRaw,intValue);
    return(cmdPtr);
}

simxUChar* _exec_intint_int(simxInt clientID,simxInt cmdRaw,simxInt opMode,simxUChar options,simxInt intValue1,simxInt intValue2,simxInt intValue3,simxInt* error)
{
    simxUChar* cmdPtr;
    simxUShort delayOrSplit;

    error[0]=simx_return_ok;

    delayOrSplit=opMode&simx_cmdmask;
    opMode-=(opMode&simx_cmdmask);

    if (opMode!=simx_opmode_buffer)
    {
        /* Check if the command is already present in the request list */
        extApi_lockResources(clientID);
        if (opMode==simx_opmode_oneshot_split)
        {
            if (delayOrSplit<_MIN_SPLIT_AMOUNT_IN_BYTES)
                delayOrSplit=_MIN_SPLIT_AMOUNT_IN_BYTES;
            cmdPtr=_getCommandPointer_ii(cmdRaw,intValue1,intValue2,_splitCommandsToSend[clientID],_splitCommandsToSend_dataSize[clientID]);
            if (cmdPtr!=0)
            { /* Command already there */
                error[0]|=simx_return_split_progress_flag;
            }
            else
            { /* Command not there. Add it */
                _splitCommandsToSend[clientID]=_appendCommand_ii_i(cmdRaw+opMode,options,intValue1,intValue2,intValue3,delayOrSplit,_splitCommandsToSend[clientID],&_splitCommandsToSend_bufferSize[clientID],&_splitCommandsToSend_dataSize[clientID]);
            }
        }
        else
        {
            cmdPtr=_getCommandPointer_ii(cmdRaw,intValue1,intValue2,_messageToSend[clientID]+SIMX_HEADER_SIZE,_messageToSend_dataSize[clientID]-SIMX_HEADER_SIZE);
            if ((cmdPtr!=0)&&((options&1)==0))
            { /* Command already there, and we can overwrite it. Update it */
                ((simxInt*)(cmdPtr+simx_cmdheaderoffset_cmd))[0]=extApi_endianConversionInt(cmdRaw+opMode);
                ((simxInt*)(cmdPtr+SIMX_SUBHEADER_SIZE+8))[0]=extApi_endianConversionInt(intValue3);
            }
            else
            { /* Command not there. Add it */
                _messageToSend[clientID]=_appendCommand_ii_i(cmdRaw+opMode,options,intValue1,intValue2,intValue3,delayOrSplit,_messageToSend[clientID],&_messageToSend_bufferSize[clientID],&_messageToSend_dataSize[clientID]);
            }
        }

        if (opMode==simx_opmode_blocking)
            _waitBeforeSendingAgainWhenMessageIDArrived[clientID]=_nextMessageIDToSend[clientID];
        extApi_unlockResources(clientID);

        /* wait until we received a reply, or a timeout (if we wanna wait for the reply) */
        if (_waitBeforeSendingAgainWhenMessageIDArrived[clientID]!=-1)
            _waitUntilMessageArrived(clientID,error); 
    }

    /* Check if the command is present in the input list (we might have this situation when we want to check if there was an error on the server side) */
    extApi_lockResources(clientID);
    cmdPtr=_getCommandPointer_ii(cmdRaw,intValue1,intValue2,_messageReceived[clientID]+SIMX_HEADER_SIZE,_messageReceived_dataSize[clientID]-SIMX_HEADER_SIZE);
    cmdPtr=_setLastFetchedCmd(clientID,cmdPtr,error);
    _waitBeforeSendingAgainWhenMessageIDArrived[clientID]=-1; /* make sure to enable the communication thread again! */
    extApi_unlockResources(clientID);
    if (opMode==simx_opmode_blocking) /* A cmd reply stays in the inbox always.. except when the mode is simx_opmode_blocking (to avoid polluting the inbox) */
        _removeCommandReply_intint(clientID,cmdRaw,intValue1,intValue2);
    return(cmdPtr);
}

simxUChar* _exec_intint_buffer(simxInt clientID,simxInt cmdRaw,simxInt opMode,simxUChar options,simxInt intValue1,simxInt intValue2,simxUChar* buffer,simxInt bufferSize,simxInt* error)
{
    simxUChar* cmdPtr;
    simxUShort delayOrSplit;

    error[0]=simx_return_ok;

    delayOrSplit=opMode&simx_cmdmask;
    opMode-=(opMode&simx_cmdmask);

    if (opMode!=simx_opmode_buffer)
    {
        /* Check if the command is already present in the request list */
        extApi_lockResources(clientID);
        if (opMode==simx_opmode_oneshot_split)
        {
            if (delayOrSplit<_MIN_SPLIT_AMOUNT_IN_BYTES)
                delayOrSplit=_MIN_SPLIT_AMOUNT_IN_BYTES;
            cmdPtr=_getCommandPointer_ii(cmdRaw,intValue1,intValue2,_splitCommandsToSend[clientID],_splitCommandsToSend_dataSize[clientID]);
            if (cmdPtr!=0)
            { /* Command already there */
                error[0]|=simx_return_split_progress_flag;
            }
            else
            { /* Command not there. Add it */
                _splitCommandsToSend[clientID]=_appendCommand_ii_buff(cmdRaw+opMode,options,intValue1,intValue2,buffer,bufferSize,delayOrSplit,_splitCommandsToSend[clientID],&_splitCommandsToSend_bufferSize[clientID],&_splitCommandsToSend_dataSize[clientID]);
            }
        }
        else
        {
            cmdPtr=_getCommandPointer_ii(cmdRaw,intValue1,intValue2,_messageToSend[clientID]+SIMX_HEADER_SIZE,_messageToSend_dataSize[clientID]-SIMX_HEADER_SIZE);
            if ((cmdPtr!=0)&&((options&1)==0))
            { /* Command already there, and we can overwrite it. Remove it, we'll add it again just after */
                _removeChunkFromBuffer(_messageToSend[clientID],cmdPtr,extApi_endianConversionInt(((simxInt*)(cmdPtr+simx_cmdheaderoffset_mem_size))[0]),&_messageToSend_dataSize[clientID]); /* we remove then add the command again (b/c no guarantee the buffer has the same size) */
            }
            /* Add it: */
            _messageToSend[clientID]=_appendCommand_ii_buff(cmdRaw+opMode,options,intValue1,intValue2,buffer,bufferSize,delayOrSplit,_messageToSend[clientID],&_messageToSend_bufferSize[clientID],&_messageToSend_dataSize[clientID]);
        }

        if (opMode==simx_opmode_blocking)
            _waitBeforeSendingAgainWhenMessageIDArrived[clientID]=_nextMessageIDToSend[clientID];
        extApi_unlockResources(clientID);

        /* wait until we received a reply, or a timeout (if we wanna wait for the reply) */
        if (_waitBeforeSendingAgainWhenMessageIDArrived[clientID]!=-1)
            _waitUntilMessageArrived(clientID,error); 
    }

    /* Check if the command is present in the input list (we might have this situation when we want to check if there was an error on the server side) */
    extApi_lockResources(clientID);
    cmdPtr=_getCommandPointer_ii(cmdRaw,intValue1,intValue2,_messageReceived[clientID]+SIMX_HEADER_SIZE,_messageReceived_dataSize[clientID]-SIMX_HEADER_SIZE);
    cmdPtr=_setLastFetchedCmd(clientID,cmdPtr,error);
    _waitBeforeSendingAgainWhenMessageIDArrived[clientID]=-1; /* make sure to enable the communication thread again! */
    extApi_unlockResources(clientID);
    if (opMode==simx_opmode_blocking) /* A cmd reply stays in the inbox always.. except when the mode is simx_opmode_blocking (to avoid polluting the inbox) */
        _removeCommandReply_intint(clientID,cmdRaw,intValue1,intValue2);
    return(cmdPtr);
}

simxUChar* _exec_int_float(simxInt clientID,simxInt cmdRaw,simxInt opMode,simxUChar options,simxInt intValue,simxFloat floatValue,simxInt* error)
{
    simxUChar* cmdPtr;
    simxUShort delayOrSplit;

    error[0]=simx_return_ok;

    delayOrSplit=opMode&simx_cmdmask;
    opMode-=(opMode&simx_cmdmask);

    if (opMode!=simx_opmode_buffer)
    {
        /* Check if the command is already present in the request list */
        extApi_lockResources(clientID);
        if (opMode==simx_opmode_oneshot_split)
        {
            if (delayOrSplit<_MIN_SPLIT_AMOUNT_IN_BYTES)
                delayOrSplit=_MIN_SPLIT_AMOUNT_IN_BYTES;
            cmdPtr=_getCommandPointer_i(cmdRaw,intValue,_splitCommandsToSend[clientID],_splitCommandsToSend_dataSize[clientID]);
            if (cmdPtr!=0)
            { /* Command already there */
                error[0]|=simx_return_split_progress_flag;
            }
            else
            { /* Command not there. Add it */
                _splitCommandsToSend[clientID]=_appendCommand_i_f(cmdRaw+opMode,options,intValue,floatValue,delayOrSplit,_splitCommandsToSend[clientID],&_splitCommandsToSend_bufferSize[clientID],&_splitCommandsToSend_dataSize[clientID]);
            }
        }
        else
        {
            cmdPtr=_getCommandPointer_i(cmdRaw,intValue,_messageToSend[clientID]+SIMX_HEADER_SIZE,_messageToSend_dataSize[clientID]-SIMX_HEADER_SIZE);
            if ((cmdPtr!=0)&&((options&1)==0))
            { /* Command already there, and we can overwrite it. Update it */
                ((simxInt*)(cmdPtr+simx_cmdheaderoffset_cmd))[0]=extApi_endianConversionInt(cmdRaw+opMode);
                ((simxFloat*)(cmdPtr+SIMX_SUBHEADER_SIZE+4))[0]=extApi_endianConversionFloat(floatValue);
            }
            else
            { /* Command not there. Add it */
                _messageToSend[clientID]=_appendCommand_i_f(cmdRaw+opMode,options,intValue,floatValue,delayOrSplit,_messageToSend[clientID],&_messageToSend_bufferSize[clientID],&_messageToSend_dataSize[clientID]);
            }
        }

        if (opMode==simx_opmode_blocking)
            _waitBeforeSendingAgainWhenMessageIDArrived[clientID]=_nextMessageIDToSend[clientID];
        extApi_unlockResources(clientID);

        /* wait until we received a reply, or a timeout (if we wanna wait for the reply) */
        if (_waitBeforeSendingAgainWhenMessageIDArrived[clientID]!=-1)
            _waitUntilMessageArrived(clientID,error); 
    }

    /* Check if the command is present in the input list (we might have this situation when we want to check if there was an error on the server side) */
    extApi_lockResources(clientID);
    cmdPtr=_getCommandPointer_i(cmdRaw,intValue,_messageReceived[clientID]+SIMX_HEADER_SIZE,_messageReceived_dataSize[clientID]-SIMX_HEADER_SIZE);
    cmdPtr=_setLastFetchedCmd(clientID,cmdPtr,error);
    _waitBeforeSendingAgainWhenMessageIDArrived[clientID]=-1; /* make sure to enable the communication thread again! */
    extApi_unlockResources(clientID);
    if (opMode==simx_opmode_blocking) /* A cmd reply stays in the inbox always.. except when the mode is simx_opmode_blocking (to avoid polluting the inbox) */
        _removeCommandReply_int(clientID,cmdRaw,intValue);
    return(cmdPtr);
}

simxUChar* _exec_int_buffer(simxInt clientID,simxInt cmdRaw,simxInt opMode,simxUChar options,simxInt intValue,simxUChar* buffer,simxInt bufferSize,simxInt* error)
{
    simxUChar* cmdPtr;
    simxUShort delayOrSplit;

    error[0]=simx_return_ok;

    delayOrSplit=opMode&simx_cmdmask;
    opMode-=(opMode&simx_cmdmask);

    if (opMode!=simx_opmode_buffer)
    {
        /* Check if the command is already present in the request list */
        extApi_lockResources(clientID);
        if (opMode==simx_opmode_oneshot_split)
        {
            if (delayOrSplit<_MIN_SPLIT_AMOUNT_IN_BYTES)
                delayOrSplit=_MIN_SPLIT_AMOUNT_IN_BYTES;
            cmdPtr=_getCommandPointer_i(cmdRaw,intValue,_splitCommandsToSend[clientID],_splitCommandsToSend_dataSize[clientID]);
            if (cmdPtr!=0)
            { /* Command already there */
                /* Now make sure we have the same command size, otherwise we have to remove the old cmd and add freshly the new */
                if (extApi_endianConversionInt(((simxInt*)(cmdPtr+simx_cmdheaderoffset_mem_size))[0])==SIMX_SUBHEADER_SIZE+4+bufferSize)
                    error[0]|=simx_return_split_progress_flag; /* ok, we have the same size */
                else
                { /* we don't have the same size! Remove the old command */
                    _removeChunkFromBuffer(_splitCommandsToSend[clientID],cmdPtr,extApi_endianConversionInt(((simxInt*)(cmdPtr+simx_cmdheaderoffset_mem_size))[0]),&_splitCommandsToSend_dataSize[clientID]);
                    cmdPtr=0; /* so that we will add the new command in next section */
                }
            }
            if (cmdPtr==0)
                _splitCommandsToSend[clientID]=_appendCommand_i_buff(cmdRaw+opMode,options,intValue,buffer,bufferSize,delayOrSplit,_splitCommandsToSend[clientID],&_splitCommandsToSend_bufferSize[clientID],&_splitCommandsToSend_dataSize[clientID]);
        }
        else
        {
            cmdPtr=_getCommandPointer_i(cmdRaw,intValue,_messageToSend[clientID]+SIMX_HEADER_SIZE,_messageToSend_dataSize[clientID]-SIMX_HEADER_SIZE);

            if ((cmdPtr!=0)&&((options&1)==0)) /* Command already there, and we can overwrite it. We remove it and add it again */
                _removeChunkFromBuffer(_messageToSend[clientID],cmdPtr,extApi_endianConversionInt(((simxInt*)(cmdPtr+simx_cmdheaderoffset_mem_size))[0]),&_messageToSend_dataSize[clientID]); /* we remove then add the command again (b/c no guarantee the buffer has the same size) */
            _messageToSend[clientID]=_appendCommand_i_buff(cmdRaw+opMode,options,intValue,buffer,bufferSize,delayOrSplit,_messageToSend[clientID],&_messageToSend_bufferSize[clientID],&_messageToSend_dataSize[clientID]);
        }

        if (opMode==simx_opmode_blocking)
            _waitBeforeSendingAgainWhenMessageIDArrived[clientID]=_nextMessageIDToSend[clientID];
        extApi_unlockResources(clientID);

        /* wait until we received a reply, or a timeout (if we wanna wait for the reply) */
        if (_waitBeforeSendingAgainWhenMessageIDArrived[clientID]!=-1)
            _waitUntilMessageArrived(clientID,error); 
    }

    /* Check if the command is present in the input list (we might have this situation when we want to check if there was an error on the server side) */
    extApi_lockResources(clientID);
    cmdPtr=_getCommandPointer_i(cmdRaw,intValue,_messageReceived[clientID]+SIMX_HEADER_SIZE,_messageReceived_dataSize[clientID]-SIMX_HEADER_SIZE);
    cmdPtr=_setLastFetchedCmd(clientID,cmdPtr,error);
    _waitBeforeSendingAgainWhenMessageIDArrived[clientID]=-1; /* make sure to enable the communication thread again! */
    extApi_unlockResources(clientID);
    if (opMode==simx_opmode_blocking) /* A cmd reply stays in the inbox always.. except when the mode is simx_opmode_blocking (to avoid polluting the inbox) */
        _removeCommandReply_int(clientID,cmdRaw,intValue);
    return(cmdPtr);
}


simxUChar* _exec_string_buffer(simxInt clientID,simxInt cmdRaw,simxInt opMode,simxUChar options,const simxUChar* stringValue,simxUChar* buffer,simxInt bufferSize,simxInt* error)
{
    simxUChar* cmdPtr;
    simxUShort delayOrSplit;

    error[0]=simx_return_ok;

    delayOrSplit=opMode&simx_cmdmask;
    opMode-=(opMode&simx_cmdmask);

    if (opMode!=simx_opmode_buffer)
    {
        /* Check if the command is already present in the request list */
        extApi_lockResources(clientID);
        if (opMode==simx_opmode_oneshot_split)
        {
            if (delayOrSplit<_MIN_SPLIT_AMOUNT_IN_BYTES)
                delayOrSplit=_MIN_SPLIT_AMOUNT_IN_BYTES;
            cmdPtr=_getCommandPointer_s(cmdRaw,stringValue,_splitCommandsToSend[clientID],_splitCommandsToSend_dataSize[clientID]);
            if (cmdPtr!=0)
            { /* Command already there */
                /* Now make sure we have the same command size, otherwise we have to remove the old cmd and add freshly the new */
                if (extApi_endianConversionInt(((simxInt*)(cmdPtr+simx_cmdheaderoffset_mem_size))[0])==SIMX_SUBHEADER_SIZE+extApi_getStringLength((simxChar*)stringValue)+1+bufferSize)
                    error[0]|=simx_return_split_progress_flag; /* ok, we have the same size */
                else
                { /* we don't have the same size! Remove the old command */
                    _removeChunkFromBuffer(_splitCommandsToSend[clientID],cmdPtr,extApi_endianConversionInt(((simxInt*)(cmdPtr+simx_cmdheaderoffset_mem_size))[0]),&_splitCommandsToSend_dataSize[clientID]);
                    cmdPtr=0; /* so that we will add the new command in next section */
                }
            }
            if (cmdPtr==0)
                _splitCommandsToSend[clientID]=_appendCommand_s_buff(cmdRaw+opMode,options,stringValue,buffer,bufferSize,delayOrSplit,_splitCommandsToSend[clientID],&_splitCommandsToSend_bufferSize[clientID],&_splitCommandsToSend_dataSize[clientID]);
        }
        else
        {
            cmdPtr=_getCommandPointer_s(cmdRaw,stringValue,_messageToSend[clientID]+SIMX_HEADER_SIZE,_messageToSend_dataSize[clientID]-SIMX_HEADER_SIZE);
            if ((cmdPtr!=0)&&((options&1)==0))  /* Command already there, and we can overwrite it. Remove it and add it again */
                _removeChunkFromBuffer(_messageToSend[clientID],cmdPtr,extApi_endianConversionInt(((simxInt*)(cmdPtr+simx_cmdheaderoffset_mem_size))[0]),&_messageToSend_dataSize[clientID]); /* we remove then add the command again (b/c no guarantee the buffer has the same size) */
            _messageToSend[clientID]=_appendCommand_s_buff(cmdRaw+opMode,options,stringValue,buffer,bufferSize,delayOrSplit,_messageToSend[clientID],&_messageToSend_bufferSize[clientID],&_messageToSend_dataSize[clientID]);
        }

        if (opMode==simx_opmode_blocking)
            _waitBeforeSendingAgainWhenMessageIDArrived[clientID]=_nextMessageIDToSend[clientID];
        extApi_unlockResources(clientID);

        /* wait until we received a reply, or a timeout (if we wanna wait for the reply) */
        if (_waitBeforeSendingAgainWhenMessageIDArrived[clientID]!=-1)
            _waitUntilMessageArrived(clientID,error); 
    }

    /* Check if the command is present in the input list (we might have this situation when we want to check if there was an error on the server side) */
    extApi_lockResources(clientID);
    cmdPtr=_getCommandPointer_s(cmdRaw,stringValue,_messageReceived[clientID]+SIMX_HEADER_SIZE,_messageReceived_dataSize[clientID]-SIMX_HEADER_SIZE);
    cmdPtr=_setLastFetchedCmd(clientID,cmdPtr,error);
    _waitBeforeSendingAgainWhenMessageIDArrived[clientID]=-1; /* make sure to enable the communication thread again! */
    extApi_unlockResources(clientID);
    if (opMode==simx_opmode_blocking) /* A cmd reply stays in the inbox always.. except when the mode is simx_opmode_blocking (to avoid polluting the inbox) */
        _removeCommandReply_string(clientID,cmdRaw,stringValue);
    return(cmdPtr);
}

simxUChar* _exec_intstringstring_buffer(simxInt clientID,simxInt cmdRaw,simxInt opMode,simxUChar options,simxInt intValue,const simxUChar* stringValue1,const simxUChar* stringValue2,simxUChar* buffer,simxInt bufferSize,simxInt* error)
{
    simxUChar* cmdPtr;
    simxUShort delayOrSplit;

    error[0]=simx_return_ok;

    delayOrSplit=opMode&simx_cmdmask;
    opMode-=(opMode&simx_cmdmask);

    if (opMode!=simx_opmode_buffer)
    {
        /* Check if the command is already present in the request list */
        extApi_lockResources(clientID);
        if (opMode==simx_opmode_oneshot_split)
        {
            if (delayOrSplit<_MIN_SPLIT_AMOUNT_IN_BYTES)
                delayOrSplit=_MIN_SPLIT_AMOUNT_IN_BYTES;
            cmdPtr=_getCommandPointer_iss(cmdRaw,intValue,stringValue1,stringValue2,_splitCommandsToSend[clientID],_splitCommandsToSend_dataSize[clientID]);
            if (cmdPtr!=0)
            { /* Command already there */
                /* Now make sure we have the same command size, otherwise we have to remove the old cmd and add freshly the new */
                if (extApi_endianConversionInt(((simxInt*)(cmdPtr+simx_cmdheaderoffset_mem_size))[0])==SIMX_SUBHEADER_SIZE+extApi_getStringLength((simxChar*)stringValue1)+extApi_getStringLength((simxChar*)stringValue2)+6+bufferSize)
                    error[0]|=simx_return_split_progress_flag; /* ok, we have the same size */
                else
                { /* we don't have the same size! Remove the old command */
                    _removeChunkFromBuffer(_splitCommandsToSend[clientID],cmdPtr,extApi_endianConversionInt(((simxInt*)(cmdPtr+simx_cmdheaderoffset_mem_size))[0]),&_splitCommandsToSend_dataSize[clientID]);
                    cmdPtr=0; /* so that we will add the new command in next section */
                }
            }
            if (cmdPtr==0)
                _splitCommandsToSend[clientID]=_appendCommand_iss_buff(cmdRaw+opMode,options,intValue,stringValue1,stringValue2,buffer,bufferSize,delayOrSplit,_splitCommandsToSend[clientID],&_splitCommandsToSend_bufferSize[clientID],&_splitCommandsToSend_dataSize[clientID]);
        }
        else
        {
            cmdPtr=_getCommandPointer_iss(cmdRaw,intValue,stringValue1,stringValue2,_messageToSend[clientID]+SIMX_HEADER_SIZE,_messageToSend_dataSize[clientID]-SIMX_HEADER_SIZE);
            if ((cmdPtr!=0)&&((options&1)==0))  /* Command already there, and we can overwrite it. Remove it and add it again */
                _removeChunkFromBuffer(_messageToSend[clientID],cmdPtr,extApi_endianConversionInt(((simxInt*)(cmdPtr+simx_cmdheaderoffset_mem_size))[0]),&_messageToSend_dataSize[clientID]); /* we remove then add the command again (b/c no guarantee the buffer has the same size) */
            _messageToSend[clientID]=_appendCommand_iss_buff(cmdRaw+opMode,options,intValue,stringValue1,stringValue2,buffer,bufferSize,delayOrSplit,_messageToSend[clientID],&_messageToSend_bufferSize[clientID],&_messageToSend_dataSize[clientID]);
        }

        if (opMode==simx_opmode_blocking)
            _waitBeforeSendingAgainWhenMessageIDArrived[clientID]=_nextMessageIDToSend[clientID];
        extApi_unlockResources(clientID);

        /* wait until we received a reply, or a timeout (if we wanna wait for the reply) */
        if (_waitBeforeSendingAgainWhenMessageIDArrived[clientID]!=-1)
            _waitUntilMessageArrived(clientID,error); 
    }

    /* Check if the command is present in the input list (we might have this situation when we want to check if there was an error on the server side) */
    extApi_lockResources(clientID);
    cmdPtr=_getCommandPointer_iss(cmdRaw,intValue,stringValue1,stringValue2,_messageReceived[clientID]+SIMX_HEADER_SIZE,_messageReceived_dataSize[clientID]-SIMX_HEADER_SIZE);
    cmdPtr=_setLastFetchedCmd(clientID,cmdPtr,error);
    _waitBeforeSendingAgainWhenMessageIDArrived[clientID]=-1; /* make sure to enable the communication thread again! */
    extApi_unlockResources(clientID);
    if (opMode==simx_opmode_blocking) /* A cmd reply stays in the inbox always.. except when the mode is simx_opmode_blocking (to avoid polluting the inbox) */
        _removeCommandReply_intstringstring(clientID,cmdRaw,intValue,stringValue1,stringValue2);
    return(cmdPtr);
}

simxFloat _readPureDataFloat(simxUChar* commandPointer,simxInt stringCnt,simxInt byteOffset)
{
    simxFloat retVal=0.0f;
    simxInt additionalOffset=0;
    if (commandPointer!=0)
    {
        additionalOffset+=_getCmdDataSize(commandPointer);
        while (stringCnt!=0)
        {
            additionalOffset+=extApi_getStringLength((simxChar*)commandPointer+SIMX_SUBHEADER_SIZE+additionalOffset)+1;
            stringCnt--;
        }
        additionalOffset+=byteOffset;
        retVal=extApi_endianConversionFloat(extApi_getFloatFromPtr(commandPointer+SIMX_SUBHEADER_SIZE+additionalOffset));
    }
    return(retVal);
}

simxUChar _readPureDataChar(simxUChar* commandPointer,simxInt stringCnt,simxInt byteOffset)
{
    simxUChar retVal=0;
    simxInt additionalOffset=0;
    if (commandPointer!=0)
    {
        additionalOffset+=_getCmdDataSize(commandPointer);
        while (stringCnt!=0)
        {
            additionalOffset+=extApi_getStringLength((simxChar*)commandPointer+SIMX_SUBHEADER_SIZE+additionalOffset)+1;
            stringCnt--;
        }
        additionalOffset+=byteOffset;
        retVal=commandPointer[SIMX_SUBHEADER_SIZE+additionalOffset];
    }
    return(retVal);
}

simxInt _readPureDataInt(simxUChar* commandPointer,simxInt stringCnt,simxInt byteOffset)
{
    simxInt retVal=0;
    simxInt additionalOffset=0;
    if (commandPointer!=0)
    {
        additionalOffset+=_getCmdDataSize(commandPointer);
        while (stringCnt!=0)
        {
            additionalOffset+=extApi_getStringLength((simxChar*)commandPointer+SIMX_SUBHEADER_SIZE+additionalOffset)+1;
            stringCnt--;
        }
        additionalOffset+=byteOffset;
        retVal=extApi_endianConversionInt(extApi_getIntFromPtr(commandPointer+SIMX_SUBHEADER_SIZE+additionalOffset));
    }
    return(retVal);
}

simxUChar* _getCommandPointer_(simxInt cmdRaw,const simxUChar* commandBufferStart,simxInt commandBufferSize)
{
    simxUChar* retVal=0;
    simxInt offset=0;
    while (offset<commandBufferSize)
    {
        if ((extApi_endianConversionInt(extApi_getIntFromPtr(commandBufferStart+offset+simx_cmdheaderoffset_cmd))&simx_cmdmask)==cmdRaw)
        {
            retVal=(simxUChar*)(commandBufferStart+offset);
            break;
        }
        offset+=extApi_endianConversionInt(extApi_getIntFromPtr(commandBufferStart+offset+simx_cmdheaderoffset_mem_size));
    }
    return(retVal);
}

simxUChar* _getCommandPointer_i(simxInt cmdRaw,simxInt intValue,const simxUChar* commandBufferStart,simxInt commandBufferSize)
{
    simxUChar* retVal=0;
    simxInt offset=0;
    while (offset<commandBufferSize)
    {
        if ((extApi_endianConversionInt(extApi_getIntFromPtr(commandBufferStart+offset+simx_cmdheaderoffset_cmd))&simx_cmdmask)==cmdRaw)
        {
            if (extApi_getIntFromPtr(commandBufferStart+offset+SIMX_SUBHEADER_SIZE)==extApi_endianConversionInt(intValue))
            {
                retVal=(simxUChar*)(commandBufferStart+offset);
                break;
            }
        }
        offset+=extApi_endianConversionInt(extApi_getIntFromPtr(commandBufferStart+offset+simx_cmdheaderoffset_mem_size));
    }
    return(retVal);
}

simxUChar* _getCommandPointer_ii(simxInt cmdRaw,simxInt intValue1,simxInt intValue2,const simxUChar* commandBufferStart,simxInt commandBufferSize)
{
    simxUChar* retVal=0;
    simxInt offset=0;
    while (offset<commandBufferSize)
    {
        if ((extApi_endianConversionInt(extApi_getIntFromPtr(commandBufferStart+offset+simx_cmdheaderoffset_cmd))&simx_cmdmask)==cmdRaw)
        {
            if (extApi_getIntFromPtr(commandBufferStart+offset+SIMX_SUBHEADER_SIZE)==extApi_endianConversionInt(intValue1))
            {
                if (extApi_getIntFromPtr(commandBufferStart+offset+SIMX_SUBHEADER_SIZE+sizeof(simxInt))==extApi_endianConversionInt(intValue2))
                {
                    retVal=(simxUChar*)(commandBufferStart+offset);
                    break;
                }
            }
        }
        offset+=extApi_endianConversionInt(extApi_getIntFromPtr(commandBufferStart+offset+simx_cmdheaderoffset_mem_size));
    }
    return(retVal);
}

simxUChar* _getCommandPointer_s(simxInt cmdRaw,const simxUChar* stringValue,const simxUChar* commandBufferStart,simxInt commandBufferSize)
{
    simxUChar* retVal=0;
    simxInt offset=0;
    while (offset<commandBufferSize)
    {
        if ((extApi_endianConversionInt(extApi_getIntFromPtr(commandBufferStart+offset+simx_cmdheaderoffset_cmd))&simx_cmdmask)==cmdRaw)
        {
            if (extApi_areStringsSame((simxChar*)stringValue,(simxChar*)commandBufferStart+offset+SIMX_SUBHEADER_SIZE)!=0)
            {
                retVal=(simxUChar*)(commandBufferStart+offset);
                break;
            }
        }
        offset+=extApi_endianConversionInt(((simxInt*)(commandBufferStart+offset+simx_cmdheaderoffset_mem_size))[0]);
    }
    return(retVal);
}

simxUChar* _getCommandPointer_iss(simxInt cmdRaw,simxInt intValue,const simxUChar* stringValue1,const simxUChar* stringValue2,const simxUChar* commandBufferStart,simxInt commandBufferSize)
{
    simxUChar* retVal=0;
    simxInt offset=0;
    while (offset<commandBufferSize)
    {
        if ((extApi_endianConversionInt(extApi_getIntFromPtr(commandBufferStart+offset+simx_cmdheaderoffset_cmd))&simx_cmdmask)==cmdRaw)
        {
            if (extApi_getIntFromPtr(commandBufferStart+offset+SIMX_SUBHEADER_SIZE)==extApi_endianConversionInt(intValue))
            {
                if (extApi_areStringsSame((simxChar*)stringValue1,(simxChar*)commandBufferStart+offset+SIMX_SUBHEADER_SIZE+sizeof(simxInt))!=0)
                {
                    if (extApi_areStringsSame((simxChar*)stringValue2,(simxChar*)commandBufferStart+offset+SIMX_SUBHEADER_SIZE+sizeof(simxInt)+extApi_getStringLength((simxChar*)commandBufferStart+offset+SIMX_SUBHEADER_SIZE+sizeof(simxInt))+1)!=0)
                    {
                        retVal=(simxUChar*)(commandBufferStart+offset);
                        break;
                    }
                }
            }
        }
        offset+=extApi_endianConversionInt(extApi_getIntFromPtr(commandBufferStart+offset+simx_cmdheaderoffset_mem_size));
    }
    return(retVal);
}

simxVoid _removeChunkFromBuffer(const simxUChar* bufferStart,simxUChar* chunkStart,simxInt chunkSize,simxInt* buffer_dataSize)
{
    simxInt i,off,l;
    off=(simxInt)(chunkStart-bufferStart);
    l=buffer_dataSize[0]-off-chunkSize;
    for (i=0;i<l;i++)
        chunkStart[i]=chunkStart[chunkSize+i];
    buffer_dataSize[0]-=chunkSize;
}

simxUChar* _appendCommand_(simxInt cmd,simxUChar options,simxUShort delayOrSplit,simxUChar* buffer,simxInt* buffer_bufferSize,simxInt* buffer_dataSize)
{
    simxUChar data[SIMX_SUBHEADER_SIZE+0];
    ((simxInt*)(data+simx_cmdheaderoffset_mem_size))[0]=extApi_endianConversionInt(SIMX_SUBHEADER_SIZE+0);

    ((simxInt*)(data+simx_cmdheaderoffset_full_mem_size))[0]=extApi_endianConversionInt(SIMX_SUBHEADER_SIZE+0);
    ((simxUShort*)(data+simx_cmdheaderoffset_pdata_offset0))[0]=extApi_endianConversionUShort(0);
    ((simxInt*)(data+simx_cmdheaderoffset_pdata_offset1))[0]=extApi_endianConversionInt(0);

    ((simxInt*)(data+simx_cmdheaderoffset_cmd))[0]=extApi_endianConversionInt(cmd);
    ((simxUShort*)(data+simx_cmdheaderoffset_delay_or_split))[0]=extApi_endianConversionUShort(delayOrSplit);
    data[simx_cmdheaderoffset_status]=options;
    return(_appendChunkToBuffer(data,SIMX_SUBHEADER_SIZE+0,buffer,buffer_bufferSize,buffer_dataSize));
}

simxUChar* _appendCommand_null_buff(simxInt cmd,simxUChar options,simxUChar* buffer,simxInt bufferSize,simxUShort delayOrSplit,simxUChar* destBuffer,simxInt* destBuffer_bufferSize,simxInt* destBuffer_dataSize)
{
    simxInt i;
    simxUChar* retVal;
    simxUChar* data=extApi_allocateBuffer(SIMX_SUBHEADER_SIZE+0+bufferSize);
    ((simxInt*)(data+simx_cmdheaderoffset_mem_size))[0]=extApi_endianConversionInt(SIMX_SUBHEADER_SIZE+0+bufferSize);

    ((simxInt*)(data+simx_cmdheaderoffset_full_mem_size))[0]=extApi_endianConversionInt(SIMX_SUBHEADER_SIZE+0+bufferSize);
    ((simxUShort*)(data+simx_cmdheaderoffset_pdata_offset0))[0]=extApi_endianConversionUShort(0);
    ((simxInt*)(data+simx_cmdheaderoffset_pdata_offset1))[0]=extApi_endianConversionInt(0);

    ((simxInt*)(data+simx_cmdheaderoffset_cmd))[0]=extApi_endianConversionInt(cmd);
    ((simxUShort*)(data+simx_cmdheaderoffset_delay_or_split))[0]=extApi_endianConversionUShort(delayOrSplit);
    data[simx_cmdheaderoffset_status]=options;
    for (i=0;i<bufferSize;i++)
        data[SIMX_SUBHEADER_SIZE+0+i]=buffer[i];
    retVal=_appendChunkToBuffer(data,SIMX_SUBHEADER_SIZE+0+bufferSize,destBuffer,destBuffer_bufferSize,destBuffer_dataSize);
    extApi_releaseBuffer(data);
    return(retVal);
}

simxUChar* _appendCommand_i(simxInt cmd,simxUChar options,simxInt intValue,simxUShort delayOrSplit,simxUChar* buffer,simxInt* buffer_bufferSize,simxInt* buffer_dataSize)
{
    simxUChar data[SIMX_SUBHEADER_SIZE+4];
    ((simxInt*)(data+simx_cmdheaderoffset_mem_size))[0]=extApi_endianConversionInt(SIMX_SUBHEADER_SIZE+4);

    ((simxInt*)(data+simx_cmdheaderoffset_full_mem_size))[0]=extApi_endianConversionInt(SIMX_SUBHEADER_SIZE+4);
    ((simxUShort*)(data+simx_cmdheaderoffset_pdata_offset0))[0]=extApi_endianConversionUShort(4);
    ((simxInt*)(data+simx_cmdheaderoffset_pdata_offset1))[0]=extApi_endianConversionInt(0);

    ((simxInt*)(data+simx_cmdheaderoffset_cmd))[0]=extApi_endianConversionInt(cmd);
    ((simxUShort*)(data+simx_cmdheaderoffset_delay_or_split))[0]=extApi_endianConversionUShort(delayOrSplit);
    data[simx_cmdheaderoffset_status]=options;
    ((simxInt*)(data+SIMX_SUBHEADER_SIZE))[0]=extApi_endianConversionInt(intValue);
    return(_appendChunkToBuffer(data,SIMX_SUBHEADER_SIZE+4,buffer,buffer_bufferSize,buffer_dataSize));
}

simxUChar* _appendCommand_ii(simxInt cmd,simxUChar options,simxInt intValue1,simxInt intValue2,simxUShort delayOrSplit,simxUChar* buffer,simxInt* buffer_bufferSize,simxInt* buffer_dataSize)
{
    simxUChar data[SIMX_SUBHEADER_SIZE+8];
    ((simxInt*)(data+simx_cmdheaderoffset_mem_size))[0]=extApi_endianConversionInt(SIMX_SUBHEADER_SIZE+8);

    ((simxInt*)(data+simx_cmdheaderoffset_full_mem_size))[0]=extApi_endianConversionInt(SIMX_SUBHEADER_SIZE+8);
    ((simxUShort*)(data+simx_cmdheaderoffset_pdata_offset0))[0]=extApi_endianConversionUShort(8);
    ((simxInt*)(data+simx_cmdheaderoffset_pdata_offset1))[0]=extApi_endianConversionInt(0);

    ((simxInt*)(data+simx_cmdheaderoffset_cmd))[0]=extApi_endianConversionInt(cmd);
    ((simxUShort*)(data+simx_cmdheaderoffset_delay_or_split))[0]=extApi_endianConversionUShort(delayOrSplit);
    data[simx_cmdheaderoffset_status]=options;
    ((simxInt*)(data+SIMX_SUBHEADER_SIZE))[0]=extApi_endianConversionInt(intValue1);
    ((simxInt*)(data+SIMX_SUBHEADER_SIZE))[1]=extApi_endianConversionInt(intValue2);
    return(_appendChunkToBuffer(data,SIMX_SUBHEADER_SIZE+8,buffer,buffer_bufferSize,buffer_dataSize));
}

simxUChar* _appendCommand_s(simxInt cmd,simxUChar options,const simxUChar* stringValue,simxUShort delayOrSplit,simxUChar* buffer,simxInt* buffer_bufferSize,simxInt* buffer_dataSize)
{
    simxInt i;
    simxUChar* retVal;
    simxInt strLength=extApi_getStringLength((simxChar*)stringValue);
    simxUChar* data=extApi_allocateBuffer(SIMX_SUBHEADER_SIZE+strLength+1);
    ((simxInt*)(data+simx_cmdheaderoffset_mem_size))[0]=extApi_endianConversionInt(SIMX_SUBHEADER_SIZE+strLength+1);

    ((simxInt*)(data+simx_cmdheaderoffset_full_mem_size))[0]=extApi_endianConversionInt(SIMX_SUBHEADER_SIZE+strLength+1);
    ((simxUShort*)(data+simx_cmdheaderoffset_pdata_offset0))[0]=extApi_endianConversionUShort(strLength+1);
    ((simxInt*)(data+simx_cmdheaderoffset_pdata_offset1))[0]=extApi_endianConversionInt(0);

    ((simxInt*)(data+simx_cmdheaderoffset_cmd))[0]=extApi_endianConversionInt(cmd);
    ((simxUShort*)(data+simx_cmdheaderoffset_delay_or_split))[0]=extApi_endianConversionUShort(delayOrSplit);
    data[simx_cmdheaderoffset_status]=options;
    for (i=0;i<strLength;i++)
        (data+SIMX_SUBHEADER_SIZE)[i]=stringValue[i];
    (data+SIMX_SUBHEADER_SIZE)[strLength]=0; /* terminal 0 */
    retVal=_appendChunkToBuffer(data,SIMX_SUBHEADER_SIZE+strLength+1,buffer,buffer_bufferSize,buffer_dataSize);
    extApi_releaseBuffer(data);
    return(retVal);
}

simxUChar* _appendCommand_i_i(simxInt cmd,simxUChar options,simxInt intValue,simxInt intValue2,simxUShort delayOrSplit,simxUChar* buffer,simxInt* buffer_bufferSize,simxInt* buffer_dataSize)
{
    simxUChar data[SIMX_SUBHEADER_SIZE+4+4];
    ((simxInt*)(data+simx_cmdheaderoffset_mem_size))[0]=extApi_endianConversionInt(SIMX_SUBHEADER_SIZE+4+4);

    ((simxInt*)(data+simx_cmdheaderoffset_full_mem_size))[0]=extApi_endianConversionInt(SIMX_SUBHEADER_SIZE+4+4);
    ((simxUShort*)(data+simx_cmdheaderoffset_pdata_offset0))[0]=extApi_endianConversionUShort(4);
    ((simxInt*)(data+simx_cmdheaderoffset_pdata_offset1))[0]=extApi_endianConversionInt(0);

    ((simxInt*)(data+simx_cmdheaderoffset_cmd))[0]=extApi_endianConversionInt(cmd);
    ((simxUShort*)(data+simx_cmdheaderoffset_delay_or_split))[0]=extApi_endianConversionUShort(delayOrSplit);
    data[simx_cmdheaderoffset_status]=options;
    ((simxInt*)(data+SIMX_SUBHEADER_SIZE))[0]=extApi_endianConversionInt(intValue);
    ((simxInt*)(data+SIMX_SUBHEADER_SIZE))[1]=extApi_endianConversionInt(intValue2);
    return(_appendChunkToBuffer(data,SIMX_SUBHEADER_SIZE+4+4,buffer,buffer_bufferSize,buffer_dataSize));
}


simxUChar* _appendCommand_ii_i(simxInt cmd,simxUChar options,simxInt intValue1,simxInt intValue2,simxInt intValue3,simxUShort delayOrSplit,simxUChar* buffer,simxInt* buffer_bufferSize,simxInt* buffer_dataSize)
{
    simxUChar data[SIMX_SUBHEADER_SIZE+4+4+4];
    ((simxInt*)(data+simx_cmdheaderoffset_mem_size))[0]=extApi_endianConversionInt(SIMX_SUBHEADER_SIZE+4+4+4);

    ((simxInt*)(data+simx_cmdheaderoffset_full_mem_size))[0]=extApi_endianConversionInt(SIMX_SUBHEADER_SIZE+4+4+4);
    ((simxUShort*)(data+simx_cmdheaderoffset_pdata_offset0))[0]=extApi_endianConversionUShort(4+4);
    ((simxInt*)(data+simx_cmdheaderoffset_pdata_offset1))[0]=extApi_endianConversionInt(0);

    ((simxInt*)(data+simx_cmdheaderoffset_cmd))[0]=extApi_endianConversionInt(cmd);
    ((simxUShort*)(data+simx_cmdheaderoffset_delay_or_split))[0]=extApi_endianConversionUShort(delayOrSplit);
    data[simx_cmdheaderoffset_status]=options;
    ((simxInt*)(data+SIMX_SUBHEADER_SIZE))[0]=extApi_endianConversionInt(intValue1);
    ((simxInt*)(data+SIMX_SUBHEADER_SIZE))[1]=extApi_endianConversionInt(intValue2);
    ((simxInt*)(data+SIMX_SUBHEADER_SIZE))[2]=extApi_endianConversionInt(intValue3);
    return(_appendChunkToBuffer(data,SIMX_SUBHEADER_SIZE+4+4+4,buffer,buffer_bufferSize,buffer_dataSize));
}


simxUChar* _appendCommand_i_f(simxInt cmd,simxUChar options,simxInt intValue,simxFloat floatValue,simxUShort delayOrSplit,simxUChar* buffer,simxInt* buffer_bufferSize,simxInt* buffer_dataSize)
{
    simxUChar data[SIMX_SUBHEADER_SIZE+4+4];
    ((simxInt*)(data+simx_cmdheaderoffset_mem_size))[0]=extApi_endianConversionInt(SIMX_SUBHEADER_SIZE+4+4);

    ((simxInt*)(data+simx_cmdheaderoffset_full_mem_size))[0]=extApi_endianConversionInt(SIMX_SUBHEADER_SIZE+4+4);
    ((simxUShort*)(data+simx_cmdheaderoffset_pdata_offset0))[0]=extApi_endianConversionUShort(4);
    ((simxInt*)(data+simx_cmdheaderoffset_pdata_offset1))[0]=extApi_endianConversionInt(0);

    ((simxInt*)(data+simx_cmdheaderoffset_cmd))[0]=extApi_endianConversionInt(cmd);
    ((simxUShort*)(data+simx_cmdheaderoffset_delay_or_split))[0]=extApi_endianConversionUShort(delayOrSplit);
    data[simx_cmdheaderoffset_status]=options;
    ((simxInt*)(data+SIMX_SUBHEADER_SIZE))[0]=extApi_endianConversionInt(intValue);
    ((simxFloat*)(data+SIMX_SUBHEADER_SIZE))[1]=extApi_endianConversionFloat(floatValue);
    return(_appendChunkToBuffer(data,SIMX_SUBHEADER_SIZE+4+4,buffer,buffer_bufferSize,buffer_dataSize));
}


simxUChar* _appendCommand_i_buff(simxInt cmd,simxUChar options,simxInt intValue,simxUChar* buffer,simxInt bufferSize,simxUShort delayOrSplit,simxUChar* destBuffer,simxInt* destBuffer_bufferSize,simxInt* destBuffer_dataSize)
{
    simxInt i;
    simxUChar* retVal;
    simxUChar* data=extApi_allocateBuffer(SIMX_SUBHEADER_SIZE+4+bufferSize);
    ((simxInt*)(data+simx_cmdheaderoffset_mem_size))[0]=extApi_endianConversionInt(SIMX_SUBHEADER_SIZE+4+bufferSize);

    ((simxInt*)(data+simx_cmdheaderoffset_full_mem_size))[0]=extApi_endianConversionInt(SIMX_SUBHEADER_SIZE+4+bufferSize);
    ((simxUShort*)(data+simx_cmdheaderoffset_pdata_offset0))[0]=extApi_endianConversionUShort(4);
    ((simxInt*)(data+simx_cmdheaderoffset_pdata_offset1))[0]=extApi_endianConversionInt(0);

    ((simxInt*)(data+simx_cmdheaderoffset_cmd))[0]=extApi_endianConversionInt(cmd);
    ((simxUShort*)(data+simx_cmdheaderoffset_delay_or_split))[0]=extApi_endianConversionUShort(delayOrSplit);
    data[simx_cmdheaderoffset_status]=options;
    ((simxInt*)(data+SIMX_SUBHEADER_SIZE))[0]=extApi_endianConversionInt(intValue);
    for (i=0;i<bufferSize;i++)
        data[SIMX_SUBHEADER_SIZE+4+i]=buffer[i];
    retVal=_appendChunkToBuffer(data,SIMX_SUBHEADER_SIZE+4+bufferSize,destBuffer,destBuffer_bufferSize,destBuffer_dataSize);
    extApi_releaseBuffer(data);
    return(retVal);
}

simxUChar* _appendCommand_ii_buff(simxInt cmd,simxUChar options,simxInt intValue1,simxInt intValue2,simxUChar* buffer,simxInt bufferSize,simxUShort delayOrSplit,simxUChar* destBuffer,simxInt* destBuffer_bufferSize,simxInt* destBuffer_dataSize)
{
    simxInt i;
    simxUChar* retVal;
    simxUChar* data=extApi_allocateBuffer(SIMX_SUBHEADER_SIZE+4+4+bufferSize);
    ((simxInt*)(data+simx_cmdheaderoffset_mem_size))[0]=extApi_endianConversionInt(SIMX_SUBHEADER_SIZE+4+4+bufferSize);

    ((simxInt*)(data+simx_cmdheaderoffset_full_mem_size))[0]=extApi_endianConversionInt(SIMX_SUBHEADER_SIZE+4+4+bufferSize);
    ((simxUShort*)(data+simx_cmdheaderoffset_pdata_offset0))[0]=extApi_endianConversionUShort(4+4);
    ((simxInt*)(data+simx_cmdheaderoffset_pdata_offset1))[0]=extApi_endianConversionInt(0);

    ((simxInt*)(data+simx_cmdheaderoffset_cmd))[0]=extApi_endianConversionInt(cmd);
    ((simxUShort*)(data+simx_cmdheaderoffset_delay_or_split))[0]=extApi_endianConversionUShort(delayOrSplit);
    data[simx_cmdheaderoffset_status]=options;
    ((simxInt*)(data+SIMX_SUBHEADER_SIZE))[0]=extApi_endianConversionInt(intValue1);
    ((simxInt*)(data+SIMX_SUBHEADER_SIZE))[1]=extApi_endianConversionInt(intValue2);
    for (i=0;i<bufferSize;i++)
        data[SIMX_SUBHEADER_SIZE+4+4+i]=buffer[i];
    retVal=_appendChunkToBuffer(data,SIMX_SUBHEADER_SIZE+4+4+bufferSize,destBuffer,destBuffer_bufferSize,destBuffer_dataSize);
    extApi_releaseBuffer(data);
    return(retVal);
}

simxUChar* _appendCommand_s_buff(simxInt cmd,simxUChar options,const simxUChar* stringValue,simxUChar* buffer,simxInt bufferSize,simxUShort delayOrSplit,simxUChar* destBuffer,simxInt* destBuffer_bufferSize,simxInt* destBuffer_dataSize)
{
    simxInt i;
    simxUChar* retVal;
    simxUChar* data=extApi_allocateBuffer(SIMX_SUBHEADER_SIZE+extApi_getStringLength((simxChar*)stringValue)+1+bufferSize);
    ((simxInt*)(data+simx_cmdheaderoffset_mem_size))[0]=extApi_endianConversionInt(SIMX_SUBHEADER_SIZE+extApi_getStringLength((simxChar*)stringValue)+1+bufferSize);

    ((simxInt*)(data+simx_cmdheaderoffset_full_mem_size))[0]=extApi_endianConversionInt(SIMX_SUBHEADER_SIZE+extApi_getStringLength((simxChar*)stringValue)+1+bufferSize);
    ((simxUShort*)(data+simx_cmdheaderoffset_pdata_offset0))[0]=extApi_endianConversionUShort(extApi_getStringLength((simxChar*)stringValue)+1);
    ((simxInt*)(data+simx_cmdheaderoffset_pdata_offset1))[0]=extApi_endianConversionInt(0);

    ((simxInt*)(data+simx_cmdheaderoffset_cmd))[0]=extApi_endianConversionInt(cmd);
    ((simxUShort*)(data+simx_cmdheaderoffset_delay_or_split))[0]=extApi_endianConversionUShort(delayOrSplit);
    data[simx_cmdheaderoffset_status]=options;
    for (i=0;i<extApi_getStringLength((simxChar*)stringValue)+1;i++)
        data[SIMX_SUBHEADER_SIZE+i]=stringValue[i];
    for (i=0;i<bufferSize;i++)
        data[SIMX_SUBHEADER_SIZE+extApi_getStringLength((simxChar*)stringValue)+1+i]=buffer[i];
    retVal=_appendChunkToBuffer(data,SIMX_SUBHEADER_SIZE+extApi_getStringLength((simxChar*)stringValue)+1+bufferSize,destBuffer,destBuffer_bufferSize,destBuffer_dataSize);
    extApi_releaseBuffer(data);
    return(retVal);
}

simxUChar* _appendCommand_iss_buff(simxInt cmd,simxUChar options,simxInt intValue,const simxUChar* stringValue1,const simxUChar* stringValue2,simxUChar* buffer,simxInt bufferSize,simxUShort delayOrSplit,simxUChar* destBuffer,simxInt* destBuffer_bufferSize,simxInt* destBuffer_dataSize)
{
    simxInt i;
    simxUChar* retVal;
    simxUChar* data=extApi_allocateBuffer(SIMX_SUBHEADER_SIZE+extApi_getStringLength((simxChar*)stringValue1)+extApi_getStringLength((simxChar*)stringValue2)+6+bufferSize);
    ((simxInt*)(data+simx_cmdheaderoffset_mem_size))[0]=extApi_endianConversionInt(SIMX_SUBHEADER_SIZE+extApi_getStringLength((simxChar*)stringValue1)+extApi_getStringLength((simxChar*)stringValue2)+6+bufferSize);

    ((simxInt*)(data+simx_cmdheaderoffset_full_mem_size))[0]=extApi_endianConversionInt(SIMX_SUBHEADER_SIZE+extApi_getStringLength((simxChar*)stringValue1)+extApi_getStringLength((simxChar*)stringValue2)+6+bufferSize);
    ((simxUShort*)(data+simx_cmdheaderoffset_pdata_offset0))[0]=extApi_endianConversionUShort(extApi_getStringLength((simxChar*)stringValue1)+extApi_getStringLength((simxChar*)stringValue2)+6);
    ((simxInt*)(data+simx_cmdheaderoffset_pdata_offset1))[0]=extApi_endianConversionInt(0);

    ((simxInt*)(data+simx_cmdheaderoffset_cmd))[0]=extApi_endianConversionInt(cmd);
    ((simxUShort*)(data+simx_cmdheaderoffset_delay_or_split))[0]=extApi_endianConversionUShort(delayOrSplit);
    data[simx_cmdheaderoffset_status]=options;
    ((simxInt*)(data+SIMX_SUBHEADER_SIZE))[0]=extApi_endianConversionInt(intValue);
    for (i=0;i<extApi_getStringLength((simxChar*)stringValue1)+1;i++)
        data[SIMX_SUBHEADER_SIZE+i+4]=stringValue1[i];
    for (i=0;i<extApi_getStringLength((simxChar*)stringValue2)+1;i++)
        data[SIMX_SUBHEADER_SIZE+i+4+extApi_getStringLength((simxChar*)stringValue1)+1]=stringValue2[i];
    for (i=0;i<bufferSize;i++)
        data[SIMX_SUBHEADER_SIZE+extApi_getStringLength((simxChar*)stringValue1)+extApi_getStringLength((simxChar*)stringValue2)+6+i]=buffer[i];
    retVal=_appendChunkToBuffer(data,SIMX_SUBHEADER_SIZE+extApi_getStringLength((simxChar*)stringValue1)+extApi_getStringLength((simxChar*)stringValue2)+6+bufferSize,destBuffer,destBuffer_bufferSize,destBuffer_dataSize);
    extApi_releaseBuffer(data);
    return(retVal);
}

simxUChar* _appendChunkToBuffer(const simxUChar* chunk,simxInt chunkSize,simxUChar* buffer,simxInt* buffer_bufferSize,simxInt* buffer_dataSize)
{
    simxInt i,incr;
    simxUChar* retBuffer;
    if (buffer_bufferSize[0]-buffer_dataSize[0]<chunkSize)
    { /* not enough space in the buffer. Allocate more! */
        incr=chunkSize-(buffer_bufferSize[0]-buffer_dataSize[0]);
        if (incr<SIMX_MIN_BUFF_INCR)
            incr=SIMX_MIN_BUFF_INCR;
        retBuffer=extApi_allocateBuffer(buffer_bufferSize[0]+incr);
        for (i=0;i<buffer_dataSize[0];i++)
            retBuffer[i]=buffer[i];
        extApi_releaseBuffer(buffer);
        buffer_bufferSize[0]+=incr;
    }
    else
        retBuffer=buffer;
    /* insert the data (if chunk is not NULL) */
    if (chunk!=0)
    {
        for (i=0;i<chunkSize;i++)
            retBuffer[buffer_dataSize[0]+i]=chunk[i];
    }
    buffer_dataSize[0]+=chunkSize;
    return(retBuffer);
}

simxUChar* _appendCommandToBufferAndTakeIntoAccountPreviouslyReceivedData(const simxUChar* cmdPtr,simxUChar* cmdBuffer,simxInt cmdBufferSize,const simxUChar* chunk,simxInt chunkSize,simxUChar* buffer,simxInt* buffer_bufferSize,simxInt* buffer_dataSize)
{
    simxInt cmdRaw,cmdSizeWithoutPureData,previousPureDataSize,totalSize;
    simxUChar* prevCmdPtr;
    
    cmdRaw=extApi_endianConversionInt(((simxInt*)(cmdPtr+simx_cmdheaderoffset_cmd))[0])&simx_cmdmask;
    if (cmdRaw==simx_cmd_read_string_stream)
    { /* special handling with this command: we merge it with the previous data of that type that was received: */
        prevCmdPtr=_getSameCommandPointer(cmdPtr,cmdBuffer,cmdBufferSize);
        if (prevCmdPtr!=0)
        { /* we have to merge the data: */
            totalSize=0;

            cmdSizeWithoutPureData=SIMX_SUBHEADER_SIZE+(simxInt)extApi_endianConversionUShort(((simxUShort*)(cmdPtr+simx_cmdheaderoffset_pdata_offset0))[0]);
            previousPureDataSize=extApi_endianConversionInt(((simxInt*)(prevCmdPtr+simx_cmdheaderoffset_mem_size))[0])-cmdSizeWithoutPureData;

            if (previousPureDataSize!=0)
            {
                /* 1. we append the new header part: */
                buffer=_appendChunkToBuffer(chunk,cmdSizeWithoutPureData,buffer,buffer_bufferSize,buffer_dataSize);
                totalSize=totalSize+cmdSizeWithoutPureData;

                /* 2. we append the previous data: */
                buffer=_appendChunkToBuffer(prevCmdPtr+cmdSizeWithoutPureData,previousPureDataSize,buffer,buffer_bufferSize,buffer_dataSize);
                totalSize=totalSize+previousPureDataSize;

                /* 3. we append the current data: */
                buffer=_appendChunkToBuffer(chunk+cmdSizeWithoutPureData,chunkSize-cmdSizeWithoutPureData,buffer,buffer_bufferSize,buffer_dataSize);
                totalSize=totalSize+(chunkSize-cmdSizeWithoutPureData);

                /* 4. we correct some header values: */
                ((simxInt*)(buffer+buffer_dataSize[0]-totalSize+simx_cmdheaderoffset_mem_size))[0]=extApi_endianConversionInt(totalSize);
                ((simxInt*)(buffer+buffer_dataSize[0]-totalSize+simx_cmdheaderoffset_full_mem_size))[0]=extApi_endianConversionInt(totalSize);

                /* the previous data will be discarded outside of this function */
                return(buffer);
            }
        }
        return(_appendChunkToBuffer(chunk,chunkSize,buffer,buffer_bufferSize,buffer_dataSize)); /* default behaviour is simply to append */
    }
    else
        return(_appendChunkToBuffer(chunk,chunkSize,buffer,buffer_bufferSize,buffer_dataSize)); /* default behaviour is simply to append */
}

simxUChar* _getSameCommandPointer(const simxUChar* cmdPtr,simxUChar* cmdBuffer,simxInt cmdBufferSize)
{
    simxInt off,cmd1Raw,cmd2Raw,tmp;

    cmd1Raw=extApi_endianConversionInt(((simxInt*)(cmdPtr+simx_cmdheaderoffset_cmd))[0])&simx_cmdmask;
    off=0;
    while (off<cmdBufferSize)
    {
        cmd2Raw=extApi_endianConversionInt(((simxInt*)(cmdBuffer+off+simx_cmdheaderoffset_cmd))[0])&simx_cmdmask;
        if (cmd1Raw==cmd2Raw)
        { /* The commands are same. We need to check if the command data is same too */
            if ((cmd1Raw>simx_cmd4bytes_start)&&(cmd1Raw<simx_cmd8bytes_start))
            {
                if ( ((simxInt*)(cmdPtr+SIMX_SUBHEADER_SIZE))[0]==((simxInt*)(cmdBuffer+off+SIMX_SUBHEADER_SIZE))[0] )
                    return(cmdBuffer+off);
            }
            if ((cmd1Raw>simx_cmd8bytes_start)&&(cmd1Raw<simx_cmd1string_start))
            {
                if ( ((simxInt*)(cmdPtr+SIMX_SUBHEADER_SIZE))[0]==((simxInt*)(cmdBuffer+off+SIMX_SUBHEADER_SIZE))[0] )
                {
                    if ( ((simxInt*)(cmdPtr+SIMX_SUBHEADER_SIZE+4))[0]==((simxInt*)(cmdBuffer+off+SIMX_SUBHEADER_SIZE+4))[0] )
                        return(cmdBuffer+off);
                }
            }
            if ((cmd1Raw>simx_cmd1string_start)&&(cmd1Raw<simx_cmd4bytes2strings_start))
            {
                if (extApi_areStringsSame((simxChar*)cmdPtr+SIMX_SUBHEADER_SIZE,(simxChar*)cmdBuffer+off+SIMX_SUBHEADER_SIZE)!=0)
                    return(cmdBuffer+off);
            }
            if ((cmd1Raw>simx_cmd4bytes2strings_start)&&(cmd1Raw<simx_cmd4bytes2strings_end))
            {
                if ( ((simxInt*)(cmdPtr+SIMX_SUBHEADER_SIZE))[0]==((simxInt*)(cmdBuffer+off+SIMX_SUBHEADER_SIZE))[0] )
                {
                    if (extApi_areStringsSame((simxChar*)cmdPtr+SIMX_SUBHEADER_SIZE+4,(simxChar*)cmdBuffer+off+SIMX_SUBHEADER_SIZE+4)!=0)
                    {
                        tmp=extApi_getStringLength((simxChar*)cmdPtr+SIMX_SUBHEADER_SIZE+4)+1;
                        if (extApi_areStringsSame((simxChar*)cmdPtr+SIMX_SUBHEADER_SIZE+4+tmp,(simxChar*)cmdBuffer+off+SIMX_SUBHEADER_SIZE+4+tmp)!=0)
                            return(cmdBuffer+off);
                    }
                }
            }
        }
        off+=extApi_endianConversionInt(((simxInt*)(cmdBuffer+off+simx_cmdheaderoffset_mem_size))[0]);
    }
    return(0);
}

simxInt _getCmdDataSize(simxUChar* commandPointer)
{
    simxInt retVal=0;
    simxInt tmp;
    simxInt cmdRaw=extApi_endianConversionInt(((simxInt*)(commandPointer+simx_cmdheaderoffset_cmd))[0])&simx_cmdmask;
    if ((cmdRaw>simx_cmd4bytes_start)&&(cmdRaw<simx_cmd8bytes_start))
        retVal=4;
    if ((cmdRaw>simx_cmd8bytes_start)&&(cmdRaw<simx_cmd1string_start))
        retVal=8;
    if ((cmdRaw>simx_cmd1string_start)&&(cmdRaw<simx_cmd4bytes2strings_start))
        retVal=extApi_getStringLength((simxChar*)commandPointer+SIMX_SUBHEADER_SIZE)+1;
    if ((cmdRaw>simx_cmd4bytes2strings_start)&&(cmdRaw<simx_cmd4bytes2strings_end))
    {
        tmp=extApi_getStringLength((simxChar*)commandPointer+SIMX_SUBHEADER_SIZE+4)+1;
        retVal=4+tmp+extApi_getStringLength((simxChar*)commandPointer+SIMX_SUBHEADER_SIZE+4+tmp)+1;
    }
    return(retVal);
}


simxUChar _sendMessage_socketOrSharedMem(simxInt clientID,const simxUChar* message,simxInt messageSize,simxUChar usingSharedMem)
{ /* return 1: success */
    simxShort packetCount=0;
    simxInt s=messageSize;
    simxInt ptr=0;
    simxInt sizeToSend;

    if (messageSize==0)
        return(0);

    if (usingSharedMem)
    { /* send the message via shared memory */
        #ifdef USE_ALSO_SHARED_MEMORY
            if (extApi_send_sharedMem(clientID,message,messageSize)!=messageSize)
                return(0);
        #else
            return(0);
        #endif
    }
    else
    { /* send the message via sockets */
        /* In Following we make sure we don't send too big packets (we might send the data in several packets) */
        while (s!=0)
        {
            packetCount++;
            if (s>SOCKET_MAX_PACKET_SIZE-SOCKET_HEADER_LENGTH)
                s-=SOCKET_MAX_PACKET_SIZE-SOCKET_HEADER_LENGTH;
            else
                s=0;
        }
        s=messageSize;
        while (s!=0)
        {
            packetCount--;
            sizeToSend=s;
            if (s>SOCKET_MAX_PACKET_SIZE-SOCKET_HEADER_LENGTH)
                sizeToSend=SOCKET_MAX_PACKET_SIZE-SOCKET_HEADER_LENGTH;
            s-=sizeToSend;
            if (_sendSimplePacket_socket(clientID,message+ptr,(simxShort)sizeToSend,packetCount)==0)
                return(0);
            ptr+=sizeToSend;
        }
    }
    return(1);
}

simxUChar* _receiveReplyMessage_socketOrSharedMem(simxInt clientID,simxInt* messageSize,simxUChar usingSharedMem)
{ /* return 0: failure */
    simxInt i,result;
    simxInt cnt=0;
    simxUChar* retBuff=0;
    simxInt retBuffSize=0;
    simxUChar* inDat;
    simxShort inDatSize;

    if (usingSharedMem)
    { /* receive data via shared memory */
        #ifdef USE_ALSO_SHARED_MEMORY
            return(extApi_recv_sharedMem(clientID,messageSize));
        #else
            return(0);
        #endif
    }
    else
    { /* receive data via sockets */
        while (1)
        {
            result=_receiveSimplePacket_socket(clientID,&inDat,&inDatSize);
            if (result<0)
            {
                if (cnt!=0)
                    extApi_releaseBuffer(retBuff);
                return(0);
            }
            if (cnt==0)
                retBuff=extApi_allocateBuffer((1+result)*(inDatSize+SOCKET_HEADER_LENGTH));
            for (i=0;i<inDatSize;i++)
                retBuff[retBuffSize+i]=inDat[i];
            extApi_releaseBuffer(inDat);
            retBuffSize+=inDatSize;
            if (result==0)
            { /* ok, no more packets to receive */
                messageSize[0]=retBuffSize;
                return(retBuff);
            }
            cnt+=1;
        }
    }
    return(0);
}

simxUChar _sendSimplePacket_socket(simxInt clientID,const simxUChar* packet,simxShort packetLength,simxShort packetsLeft)
{
    simxInt i;
    simxUChar toSend[SOCKET_MAX_PACKET_SIZE];

    if (packetLength==0)
        return(0);

    /* Prepare the header */
    ((simxShort*)toSend)[0]=extApi_endianConversionShort(1); /* Allows to detect Endianness on the other side */
    ((simxShort*)toSend)[1]=extApi_endianConversionShort(packetLength);
    ((simxShort*)toSend)[2]=extApi_endianConversionShort(packetsLeft);
    
    /* Prepare the rest of the packet */
    for (i=0;i<packetLength;i++)
        toSend[SOCKET_HEADER_LENGTH+i]=packet[i];

    /* Send the packet */
    if (extApi_send_socket(clientID,toSend,packetLength+SOCKET_HEADER_LENGTH)==packetLength+SOCKET_HEADER_LENGTH)
        return(1);
    return(0);
}

simxInt _receiveSimplePacket_socket(simxInt clientID,simxUChar** packet,simxShort* packetSize)
{ /* Returns the number of packets left to read if >=0, otherwise error */

    /* 1. Read the header and packet size */
    simxUChar headerAndSize[SOCKET_HEADER_LENGTH];
    simxInt totalReceived=0;
    simxInt nb;
    simxShort dataLength,packetsLeft;
    simxInt startT=extApi_getTimeInMs();
    while(totalReceived!=SOCKET_HEADER_LENGTH)
    {
        nb=extApi_recv_socket(clientID,headerAndSize+totalReceived,SOCKET_HEADER_LENGTH-totalReceived);
        if (nb<1)
            break;
        totalReceived+=nb;
        if (extApi_getTimeDiffInMs(startT)>SOCKET_TIMEOUT_READ)
            break;
    }
    
    /* 2. Check if the header is consistent */
    if (totalReceived!=SOCKET_HEADER_LENGTH)
        return(-1);
    dataLength=extApi_endianConversionShort(((simxShort*)headerAndSize)[1]);
    packetsLeft=extApi_endianConversionShort(((simxShort*)headerAndSize)[2]);

    /* 3. Read the data with correct length */
    packetSize[0]=dataLength;
    totalReceived=0;
    startT=extApi_getTimeInMs();
    packet[0]=extApi_allocateBuffer(dataLength);
    while(totalReceived!=dataLength)
    {
        nb=extApi_recv_socket(clientID,packet[0]+totalReceived,dataLength-totalReceived);
        if (nb<1)
            break;
        totalReceived+=nb;
        if (extApi_getTimeDiffInMs(startT)>3000)
            break;
    }
    if (totalReceived!=dataLength)
    {
        extApi_releaseBuffer(packet[0]);
        return(-1);
    }
    return(packetsLeft);
}

SIMX_THREAD_RET_TYPE _communicationThread(simxVoid* p)
{
    simxUChar* replyData;
    simxUChar* tempBuffer;
    simxUChar* cmdPointer;
    simxInt tempBufferDataSize;
    simxInt tempBufferBufferSize;
    simxInt replyDataSize;
    simxInt tmp,off,cmd,i,memSize,fullMemSize,memSize2;
    simxUShort crc,pureDataOffset0;
    simxInt pureDataOffset1,maxPureDataSize,pureDataSize;
    simxInt lastTime,waitBeforeSendingAgainWhenMessageIDArrived_copy;
    simxInt clientID=_clientIDForThread;
    simxUChar usingSharedMem,connectionResult;
    _clientIDForThread=-1; /* tell the simxStart function that we are set */
    usingSharedMem=(_tempConnectionPort[clientID]<0);
    while (_communicationThreadRunning[clientID]!=0)
    { /* only the main thread can have this thread end! */

        /* printf("Trying to connect...\n"); */
        if (usingSharedMem)
        { /* using shared memory */
#ifdef USE_ALSO_SHARED_MEMORY
            connectionResult=extApi_connectToServer_sharedMem(clientID,_tempConnectionPort[clientID]);
#endif
        }
        else
        { /* using sockets */
            connectionResult=extApi_connectToServer_socket(clientID,_tempConnectionAddress[clientID],_tempConnectionPort[clientID]);
        }
        if (connectionResult==1)
        {
            _connectionID[clientID]=_nextConnectionID[clientID]++;
            /* printf("Connected!\n"); */
            lastTime=extApi_getTimeInMs();
            while (_communicationThreadRunning[clientID]!=0)
            {
                /* printf("."); */
                /* 1. Check if we should wait until the input buffer got read */
                extApi_lockResources(clientID);
                waitBeforeSendingAgainWhenMessageIDArrived_copy=_waitBeforeSendingAgainWhenMessageIDArrived[clientID];
                extApi_unlockResources(clientID);
                if ((waitBeforeSendingAgainWhenMessageIDArrived_copy!=-1)&&(_messageReceived_dataSize[clientID]>=SIMX_HEADER_SIZE))
                {
                    while (1)
                    {
                        extApi_lockResources(clientID);
                        waitBeforeSendingAgainWhenMessageIDArrived_copy=_waitBeforeSendingAgainWhenMessageIDArrived[clientID];
                        extApi_unlockResources(clientID);
                        if ((_lastReceivedMessageID[clientID]<_waitBeforeSendingAgainWhenMessageIDArrived[clientID])||(waitBeforeSendingAgainWhenMessageIDArrived_copy==-1))
                            break;
                        extApi_switchThread();
                    }
                }

                /* 2. Make sure we don't send too many requests */
                while (extApi_getTimeDiffInMs(lastTime)<_minCommunicationDelay[clientID])
                    extApi_switchThread();
                lastTime=extApi_getTimeInMs();
                extApi_lockSendStart(clientID); /* if we need to guarantee that several specific commands are sent at the same time, this might be locked already! */
                /* 3. Send a request */
                extApi_lockResources(clientID);
                extApi_unlockSendStart(clientID);
                /* Take care of non-split commands first */
                tempBuffer=extApi_allocateBuffer(_messageToSend_dataSize[clientID]);
                for (i=0;i<_messageToSend_dataSize[clientID];i++)
                    tempBuffer[i]=_messageToSend[clientID][i];
                tempBufferDataSize=_messageToSend_dataSize[clientID];
                tempBufferBufferSize=tempBufferDataSize;
                _messageToSend_dataSize[clientID]=SIMX_HEADER_SIZE; /* remove all non-split commands */
                /* Take care of split commands here */
                off=0;
                while (off<_splitCommandsToSend_dataSize[clientID])
                {
                    memSize=extApi_endianConversionInt(((simxInt*)(_splitCommandsToSend[clientID]+off+simx_cmdheaderoffset_mem_size))[0]);
                    pureDataOffset0=extApi_endianConversionUShort(((simxUShort*)(_splitCommandsToSend[clientID]+off+simx_cmdheaderoffset_pdata_offset0))[0]);
                    pureDataOffset1=extApi_endianConversionInt(((simxInt*)(_splitCommandsToSend[clientID]+off+simx_cmdheaderoffset_pdata_offset1))[0]);
                    maxPureDataSize=extApi_endianConversionUShort(((simxUShort*)(_splitCommandsToSend[clientID]+off+simx_cmdheaderoffset_delay_or_split))[0]);
                    pureDataSize=memSize-SIMX_SUBHEADER_SIZE-pureDataOffset0-pureDataOffset1;
                    if (pureDataSize>maxPureDataSize)
                        pureDataSize=maxPureDataSize;
                    tempBuffer=_appendChunkToBuffer(_splitCommandsToSend[clientID]+off,SIMX_SUBHEADER_SIZE+pureDataOffset0,tempBuffer,&tempBufferBufferSize,&tempBufferDataSize);
                    tempBuffer=_appendChunkToBuffer(_splitCommandsToSend[clientID]+off+SIMX_SUBHEADER_SIZE+pureDataOffset0+pureDataOffset1,pureDataSize,tempBuffer,&tempBufferBufferSize,&tempBufferDataSize);
                    ((simxInt*)(tempBuffer+tempBufferDataSize-pureDataSize-pureDataOffset0-SIMX_SUBHEADER_SIZE+simx_cmdheaderoffset_mem_size))[0]=extApi_endianConversionInt(SIMX_SUBHEADER_SIZE+pureDataOffset0+pureDataSize);
                    if (SIMX_SUBHEADER_SIZE+pureDataOffset0+pureDataOffset1+pureDataSize>=memSize)
                    { /* command completely sent, we can remove it */
                        _removeChunkFromBuffer(_splitCommandsToSend[clientID],_splitCommandsToSend[clientID]+off,memSize,&_splitCommandsToSend_dataSize[clientID]);
                    }
                    else
                    { /* command not yet completely sent. Keep it, but adjust the pure data offset1 */
                        pureDataOffset1+=pureDataSize;
                        ((simxInt*)(_splitCommandsToSend[clientID]+off+simx_cmdheaderoffset_pdata_offset1))[0]=extApi_endianConversionInt(pureDataOffset1);
                        off+=memSize;
                    }
                }
                /* Set some message header values */
                tempBuffer[simx_headeroffset_version]=SIMX_VERSION;
                ((simxInt*)(tempBuffer+simx_headeroffset_message_id))[0]=extApi_endianConversionInt(_nextMessageIDToSend[clientID]++);
                ((simxInt*)(tempBuffer+simx_headeroffset_client_time))[0]=extApi_endianConversionInt(extApi_getTimeInMs());
                /* CRC calculation represents a bottleneck for large transmissions, and is anyway not needed with tcp or shared memory transmissions */
                /* ((simxUShort*)(tempBuffer+simx_headeroffset_crc))[0]=extApi_endianConversionUShort(_getCRC(tempBuffer+2,tempBufferDataSize-2)); */
                ((simxUShort*)(tempBuffer+simx_headeroffset_crc))[0]=extApi_endianConversionUShort(0);
                /* Send the message */
                if (_sendMessage_socketOrSharedMem(clientID,tempBuffer,tempBufferDataSize,usingSharedMem)!=1)
                {
                    extApi_releaseBuffer(tempBuffer);
                    extApi_unlockResources(clientID);
                    break;
                }
                extApi_releaseBuffer(tempBuffer);
                extApi_unlockResources(clientID);
                /* 4. Read the reply (the server always replies!) */
                replyData=_receiveReplyMessage_socketOrSharedMem(clientID,&replyDataSize,usingSharedMem);
                if (replyData==0)
                    break;

                /* Check the CRC */
                /* CRC calculation represents a bottleneck for large transmissions, and is anyway not needed with tcp or shared memory transmissions */
                crc=extApi_endianConversionUShort(((simxUShort*)(replyData+simx_headeroffset_crc))[0]);
                /* if (_getCRC(replyData+2,replyDataSize-2)==crc) */
                if (1)
                {
                    /* Place the reply into the input buffer */
                    tmp=extApi_endianConversionInt(((simxInt*)(replyData+simx_headeroffset_message_id))[0]);

                    if (replyDataSize>SIMX_HEADER_SIZE)
                    { /* We received a non-empty message */
                        extApi_lockResources(clientID);
                        /* a) Create a new buffer that will hold the merged input data */
                        tempBuffer=extApi_allocateBuffer(_messageReceived_bufferSize[clientID]);
                        tempBufferBufferSize=_messageReceived_bufferSize[clientID];
                        /* b) Copy the header from the received data, or from the existing input buffer (if id is -1) */
                        if (tmp==-1)
                        {
                            for (i=0;i<SIMX_HEADER_SIZE;i++)
                                tempBuffer[i]=_messageReceived[clientID][i];
                        }
                        else
                        {
                            for (i=0;i<SIMX_HEADER_SIZE;i++)
                                tempBuffer[i]=replyData[i];
                        }
                        tempBufferDataSize=SIMX_HEADER_SIZE;

                        /* c) go through the received data and add it (either to the temp buffer, either to the partial command buffer) */
                        off=SIMX_HEADER_SIZE;
                        while (off<replyDataSize)
                        {
                            memSize=extApi_endianConversionInt(((simxInt*)(replyData+off+simx_cmdheaderoffset_mem_size))[0]);
                            fullMemSize=extApi_endianConversionInt(((simxInt*)(replyData+off+simx_cmdheaderoffset_full_mem_size))[0]);
                            if (memSize==fullMemSize)
                            { /* the full data was sent at once! */
                                cmd=extApi_endianConversionInt(((simxInt*)(replyData+off+simx_cmdheaderoffset_cmd))[0]);
                                if ((cmd-(cmd&simx_cmdmask))!=simx_opmode_discontinue) /* only discontinue mode commands are not added */
                                {
                                    tempBuffer=_appendCommandToBufferAndTakeIntoAccountPreviouslyReceivedData(replyData+off,_messageReceived[clientID]+SIMX_HEADER_SIZE,_messageReceived_dataSize[clientID]-SIMX_HEADER_SIZE,replyData+off,extApi_endianConversionInt(((simxInt*)(replyData+off+simx_cmdheaderoffset_mem_size))[0]),tempBuffer,&tempBufferBufferSize,&tempBufferDataSize);
                                    /* tempBuffer=_appendChunkToBuffer(replyData+off,extApi_endianConversionInt(((simxInt*)(replyData+off+simx_cmdheaderoffset_mem_size))[0]),tempBuffer,&tempBufferBufferSize,&tempBufferDataSize); */
                                }
                                cmdPointer=_getSameCommandPointer(replyData+off,_messageReceived[clientID]+SIMX_HEADER_SIZE,_messageReceived_dataSize[clientID]-SIMX_HEADER_SIZE);
                                if (cmdPointer!=0)
                                { /* unmark this command (we already added its newer version) */
                                    ((simxInt*)(cmdPointer+simx_cmdheaderoffset_cmd))[0]=0;
                                }
                            }
                            else
                            { /* only partial data was sent */
                                
                                /* Try to merge the partial data with same data already present in the partial commands buffer */
                                cmdPointer=_getSameCommandPointer(replyData+off,_splitCommandsReceived[clientID],_splitCommandsReceived_dataSize[clientID]);
                                if (cmdPointer!=0)
                                { /* there is previous partial data. Is it valid? */
                                    memSize2=extApi_endianConversionInt(((simxInt*)(cmdPointer+simx_cmdheaderoffset_mem_size))[0]);                     
                                    if (memSize2!=fullMemSize)
                                    { /* we cannot use the previous version, since it has a different size. Remove it */
                                        _removeChunkFromBuffer(_splitCommandsReceived[clientID],cmdPointer,memSize2,&_splitCommandsReceived_dataSize[clientID]);
                                        cmdPointer=0;
                                    }
                                }
                                if (cmdPointer==0)
                                { /* there is not yet similar data present. Just add empty space */
                                    _splitCommandsReceived[clientID]=_appendChunkToBuffer(0,fullMemSize,_splitCommandsReceived[clientID],&_splitCommandsReceived_bufferSize[clientID],&_splitCommandsReceived_dataSize[clientID]);
                                    cmdPointer=_splitCommandsReceived[clientID]+_splitCommandsReceived_dataSize[clientID]-fullMemSize;
                                }
                                /* Now we have to overwrite the subheader, the command data, and the partial data */
                                for (i=0;i<SIMX_SUBHEADER_SIZE;i++)
                                    cmdPointer[i]=replyData[off+i];
                                ((simxInt*)(cmdPointer+simx_cmdheaderoffset_mem_size))[0]=extApi_endianConversionInt(fullMemSize); /* Important!! */

                                pureDataOffset0=extApi_endianConversionUShort(((simxUShort*)(cmdPointer+simx_cmdheaderoffset_pdata_offset0))[0]);
                                for (i=0;i<pureDataOffset0;i++)
                                    cmdPointer[SIMX_SUBHEADER_SIZE+i]=replyData[off+SIMX_SUBHEADER_SIZE+i];

                                pureDataOffset1=extApi_endianConversionInt(((simxInt*)(cmdPointer+simx_cmdheaderoffset_pdata_offset1))[0]);
                                pureDataSize=memSize-SIMX_SUBHEADER_SIZE-pureDataOffset0;
                                for (i=0;i<pureDataSize;i++)
                                    cmdPointer[SIMX_SUBHEADER_SIZE+pureDataOffset0+pureDataOffset1+i]=replyData[off+SIMX_SUBHEADER_SIZE+pureDataOffset0+i];

                                /* Is the partial data complete yet? */
                                if (SIMX_SUBHEADER_SIZE+pureDataOffset0+pureDataOffset1+pureDataSize>=fullMemSize)
                                { /* yes!! Copy the data from the partial command buffer to the tempBuffer, and erase it from the partial command buffer */

                                    tempBuffer=_appendCommandToBufferAndTakeIntoAccountPreviouslyReceivedData(tempBuffer+tempBufferDataSize-fullMemSize,_messageReceived[clientID]+SIMX_HEADER_SIZE,_messageReceived_dataSize[clientID]-SIMX_HEADER_SIZE,cmdPointer,fullMemSize,tempBuffer,&tempBufferBufferSize,&tempBufferDataSize);
                                    /* tempBuffer=_appendChunkToBuffer(cmdPointer,fullMemSize,tempBuffer,&tempBufferBufferSize,&tempBufferDataSize); */

                                    _removeChunkFromBuffer(_splitCommandsReceived[clientID],cmdPointer,fullMemSize,&_splitCommandsReceived_dataSize[clientID]);
                                    /* make sure we unmark any similar command in the _messageReceived[clientID] buffer */
                                    cmdPointer=_getSameCommandPointer(tempBuffer+tempBufferDataSize-fullMemSize,_messageReceived[clientID]+SIMX_HEADER_SIZE,_messageReceived_dataSize[clientID]-SIMX_HEADER_SIZE);
                                    if (cmdPointer!=0)
                                    { /* unmark this command (we already added its newer version) */
                                        ((simxInt*)(cmdPointer+simx_cmdheaderoffset_cmd))[0]=0;
                                    }
                                }
                            }
                            off+=extApi_endianConversionInt(((simxInt*)(replyData+off+simx_cmdheaderoffset_mem_size))[0]);
                        }
                        /* d) go through the old received data, and add only commands that were not unmarked */
                        off=SIMX_HEADER_SIZE;
                        while (off<_messageReceived_dataSize[clientID])
                        {
                            cmd=extApi_endianConversionInt(((simxInt*)(_messageReceived[clientID]+off+simx_cmdheaderoffset_cmd))[0]);
                            if (cmd!=0)
                            { /* ok, this command was not unmarked. We add it */
                                tempBuffer=_appendChunkToBuffer(_messageReceived[clientID]+off,extApi_endianConversionInt(((simxInt*)(_messageReceived[clientID]+off+simx_cmdheaderoffset_mem_size))[0]),tempBuffer,&tempBufferBufferSize,&tempBufferDataSize);
                            }
                            off+=extApi_endianConversionInt(((simxInt*)(_messageReceived[clientID]+off+simx_cmdheaderoffset_mem_size))[0]);
                        }
                        /* e) switch buffers and release 2 of them */
                        extApi_releaseBuffer(replyData);
                        extApi_releaseBuffer(_messageReceived[clientID]);
                        _messageReceived[clientID]=tempBuffer;
                        _messageReceived_bufferSize[clientID]=tempBufferBufferSize;
                        _messageReceived_dataSize[clientID]=tempBufferDataSize;
                        if (tmp!=-1)
                            _lastReceivedMessageID[clientID]=tmp;
                        extApi_unlockResources(clientID);
                    }
                    else
                        extApi_releaseBuffer(replyData);
                }
                else
                    extApi_releaseBuffer(replyData);

            }
            extApi_lockResources(clientID);
            _messageToSend_dataSize[clientID]=SIMX_HEADER_SIZE;
            _splitCommandsToSend_dataSize[clientID]=0;
            _messageReceived_dataSize[clientID]=0;
            _splitCommandsReceived_dataSize[clientID]=0;
            extApi_unlockResources(clientID);
            /* printf("Disconnected\n"); */
            _connectionID[clientID]=-1;

            if (usingSharedMem)
            { /* using shared memory */
#ifdef USE_ALSO_SHARED_MEMORY
                extApi_cleanUp_sharedMem(clientID);
#endif
            }
            else
            { /* using sockets */
                extApi_cleanUp_socket(clientID);
            }
        }
        else
            extApi_sleepMs(100);
        if (_tempDoNotReconnectOnceDisconnected[clientID])
        { /* sit here until the other thread sets _communicationThreadRunning[clientID] to 0 */
            while (_communicationThreadRunning[clientID]!=0)
                extApi_sleepMs(100);
            break;
        }
    }
    _communicationThreadRunning[clientID]=1; /* to indicate to the main thread that we just left */
    SIMX_THREAD_RET_LINE;
}

simxUShort _getCRC(const simxUChar* data,simxInt length)
{
    simxUShort crc=0;
    simxInt i,j;
    simxInt p=0;
    for (i=0;i<length;i++)
    {
        crc=crc^(((simxUShort)data[p])<<8);
        for (j=0;j<8;j++)
        {
            if (crc&((simxUShort)0x8000))
                crc=(crc<<1)^((simxUShort)0x1021);
            else
                crc<<=1;
        }
        p++;
    }
    return(crc);
}

simxInt _removeCommandReply_null(simxInt clientID,simxInt cmdRaw)
{
    simxUChar* cmdPtr;
    simxInt retVal=simx_return_ok;
    extApi_lockResources(clientID);
    cmdPtr=_getCommandPointer_(cmdRaw,_messageReceived[clientID]+SIMX_HEADER_SIZE,_messageReceived_dataSize[clientID]-SIMX_HEADER_SIZE);
    if (cmdPtr!=0)
        _removeChunkFromBuffer(_messageReceived[clientID],cmdPtr,extApi_endianConversionInt(((simxInt*)(cmdPtr+simx_cmdheaderoffset_mem_size))[0]),&_messageReceived_dataSize[clientID]);
    else
        retVal=simx_return_novalue_flag;
    /* Data is removed, but buffer keeps same size. It will be resized next time we receive something */
    extApi_unlockResources(clientID);
    return(retVal);
}

simxInt _removeCommandReply_int(simxInt clientID,simxInt cmdRaw,simxInt intValue)
{
    simxUChar* cmdPtr;
    simxInt retVal=simx_return_ok;
    extApi_lockResources(clientID);
    cmdPtr=_getCommandPointer_i(cmdRaw,intValue,_messageReceived[clientID]+SIMX_HEADER_SIZE,_messageReceived_dataSize[clientID]-SIMX_HEADER_SIZE);
    if (cmdPtr!=0)
        _removeChunkFromBuffer(_messageReceived[clientID],cmdPtr,extApi_endianConversionInt(((simxInt*)(cmdPtr+simx_cmdheaderoffset_mem_size))[0]),&_messageReceived_dataSize[clientID]);
    else
        retVal=simx_return_novalue_flag;
    /* Data is removed, but buffer keeps same size. It will be resized next time we receive something */
    extApi_unlockResources(clientID);
    return(retVal);
}

simxInt _removeCommandReply_intint(simxInt clientID,simxInt cmdRaw,simxInt intValue1,simxInt intValue2)
{
    simxUChar* cmdPtr;
    simxInt retVal=simx_return_ok;
    extApi_lockResources(clientID);
    cmdPtr=_getCommandPointer_ii(cmdRaw,intValue1,intValue2,_messageReceived[clientID]+SIMX_HEADER_SIZE,_messageReceived_dataSize[clientID]-SIMX_HEADER_SIZE);
    if (cmdPtr!=0)
        _removeChunkFromBuffer(_messageReceived[clientID],cmdPtr,extApi_endianConversionInt(((simxInt*)(cmdPtr+simx_cmdheaderoffset_mem_size))[0]),&_messageReceived_dataSize[clientID]);
    else
        retVal=simx_return_novalue_flag;
    /* Data is removed, but buffer keeps same size. It will be resized next time we receive something */
    extApi_unlockResources(clientID);
    return(retVal);
}

simxInt _removeCommandReply_string(simxInt clientID,simxInt cmdRaw,const simxUChar* stringValue)
{
    simxUChar* cmdPtr;
    simxInt retVal=simx_return_ok;
    extApi_lockResources(clientID);
    cmdPtr=_getCommandPointer_s(cmdRaw,stringValue,_messageReceived[clientID]+SIMX_HEADER_SIZE,_messageReceived_dataSize[clientID]-SIMX_HEADER_SIZE);
    if (cmdPtr!=0)
        _removeChunkFromBuffer(_messageReceived[clientID],cmdPtr,extApi_endianConversionInt(((simxInt*)(cmdPtr+simx_cmdheaderoffset_mem_size))[0]),&_messageReceived_dataSize[clientID]);
    else
        retVal=simx_return_novalue_flag;
    /* Data is removed, but buffer keeps same size. It will be resized next time we receive something */
    extApi_unlockResources(clientID);
    return(retVal);
}

simxInt _removeCommandReply_intstringstring(simxInt clientID,simxInt cmdRaw,simxInt intValue,const simxUChar* stringValue1,const simxUChar* stringValue2)
{
    simxUChar* cmdPtr;
    simxInt retVal=simx_return_ok;
    extApi_lockResources(clientID);
    cmdPtr=_getCommandPointer_iss(cmdRaw,intValue,stringValue1,stringValue2,_messageReceived[clientID]+SIMX_HEADER_SIZE,_messageReceived_dataSize[clientID]-SIMX_HEADER_SIZE);
    if (cmdPtr!=0)
        _removeChunkFromBuffer(_messageReceived[clientID],cmdPtr,extApi_endianConversionInt(((simxInt*)(cmdPtr+simx_cmdheaderoffset_mem_size))[0]),&_messageReceived_dataSize[clientID]);
    else
        retVal=simx_return_novalue_flag;
    /* Data is removed, but buffer keeps same size. It will be resized next time we receive something */
    extApi_unlockResources(clientID);
    return(retVal);
}



/*
**********************************************
Following are the remote API helper functions
**********************************************
*/

EXTAPI_DLLEXPORT simxInt simxGetConnectionId(simxInt clientID)
{
    if (_communicationThreadRunning[clientID]==0)
        return(-1);
    return(_connectionID[clientID]);
}

EXTAPI_DLLEXPORT simxInt simxGetPingTime(simxInt clientID,simxInt* pingTime)
{
    simxInt res,dummyVal;
    simxInt startTime=extApi_getTimeInMs();
    if (_communicationThreadRunning[clientID]==0)
        return(0);
    res=simxGetIntegerParameter(clientID,sim_intparam_program_version,&dummyVal,simx_opmode_blocking); /* just a dummy command */
    res=(res|simx_return_remote_error_flag)-simx_return_remote_error_flag;
    pingTime[0]=extApi_getTimeDiffInMs(startTime);
    return(res);
}

EXTAPI_DLLEXPORT simxInt simxSynchronousTrigger(simxInt clientID)
{
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    _exec_null(clientID,simx_cmd_synchronous_next,simx_opmode_blocking,0,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxSynchronous(simxInt clientID,simxUChar enable)
{
    simxInt returnValue;
    simxInt cmd=simx_cmd_synchronous_disable;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (enable)
        cmd=simx_cmd_synchronous_enable;
    _exec_null(clientID,cmd,simx_opmode_blocking,0,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxPauseCommunication(simxInt clientID,simxUChar pause)
{
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (pause)
        extApi_lockSendStart(clientID);
    else
        extApi_unlockSendStart(clientID);
    return(0);
}

EXTAPI_DLLEXPORT simxInt simxGetLastCmdTime(simxInt clientID)
{
    return(_commandReceived_simulationTime[clientID]);
}

EXTAPI_DLLEXPORT simxInt simxGetInMessageInfo(simxInt clientID,simxInt infoType,simxInt* info)
{
    simxInt retVal=-1;
    if (_communicationThreadRunning[clientID]==0)
        return(-1);
    extApi_lockResources(clientID);
    if (_messageReceived_dataSize[clientID]>=SIMX_HEADER_SIZE)
    {
        if ( (infoType==simx_headeroffset_message_id)||(infoType==simx_headeroffset_client_time)||(infoType==simx_headeroffset_server_time) )
        {
            info[0]=extApi_endianConversionInt(((simxInt*)(_messageReceived[clientID]+infoType))[0]);
            retVal=1;
        }
        if (infoType==simx_headeroffset_scene_id)
        {
            info[0]=extApi_endianConversionUShort(((simxUShort*)(_messageReceived[clientID]+infoType))[0]);
            retVal=1;
        }
        if ((infoType==simx_headeroffset_version)||(infoType==simx_headeroffset_server_state))
        {
            info[0]=(simxInt)((simxUChar*)(_messageReceived[clientID]+infoType))[0];
            retVal=1;
        }
    }
    extApi_unlockResources(clientID);
    return(retVal);
}

EXTAPI_DLLEXPORT simxInt simxGetOutMessageInfo(simxInt clientID,simxInt infoType,simxInt* info)
{
    simxInt retVal=-1;

    if (infoType==simx_headeroffset_client_time)
    {
        info[0]=extApi_getTimeInMs();
        return(1);
    }
    if (infoType==simx_headeroffset_version)
    {
        info[0]=SIMX_VERSION;
        return(1);
    }

    if (_communicationThreadRunning[clientID]==0)
        return(-1);

    extApi_lockResources(clientID);
    if (infoType==simx_headeroffset_message_id)
    {
        info[0]=_nextMessageIDToSend[clientID];
        retVal=1;
    }
    extApi_unlockResources(clientID);
    return(retVal);
}

EXTAPI_DLLEXPORT simxUChar* simxCreateBuffer(simxInt bufferSize)
{
    return(extApi_allocateBuffer(bufferSize));
}

EXTAPI_DLLEXPORT simxVoid simxReleaseBuffer(simxUChar* buffer)
{
    extApi_releaseBuffer(buffer);
}

/*
**********************************************
Following are the remote API functions
**********************************************
*/

EXTAPI_DLLEXPORT simxInt simxGetJointPosition(simxInt clientID,simxInt jointHandle,simxFloat* position,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_get_joint_position,jointHandle));
    dataPointer=_exec_int(clientID,simx_cmd_get_joint_position,operationMode,0,jointHandle,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
        position[0]=_readPureDataFloat(dataPointer,0,0);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxSetJointPosition(simxInt clientID,simxInt jointHandle,simxFloat position,simxInt operationMode)
{
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_set_joint_position,jointHandle));
    _exec_int_float(clientID,simx_cmd_set_joint_position,operationMode,0,jointHandle,position,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxGetJointMatrix(simxInt clientID,simxInt jointHandle,simxFloat* matrix,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue,i;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_get_joint_matrix,jointHandle));
    dataPointer=_exec_int(clientID,simx_cmd_get_joint_matrix,operationMode,0,jointHandle,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
    {
        for (i=0;i<12;i++)
            matrix[i]=_readPureDataFloat(dataPointer,0,4*i);
    }
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxSetSphericalJointMatrix(simxInt clientID,simxInt jointHandle,simxFloat* matrix,simxInt operationMode)
{
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_set_spherical_joint_matrix,jointHandle));
    _exec_int_buffer(clientID,simx_cmd_set_spherical_joint_matrix,operationMode,0,jointHandle,(simxUChar*)matrix,4*12,&returnValue);
    return(returnValue);
}

/*
EXTAPI_DLLEXPORT simxFloat mtlb_simxTest(simxFloat* b)
{
    float c=b[0];
    return(c);
}
*/


EXTAPI_DLLEXPORT simxInt simxSetJointTargetVelocity(simxInt clientID,simxInt jointHandle,simxFloat targetVelocity,simxInt operationMode)
{
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_set_joint_target_velocity,jointHandle));
    _exec_int_float(clientID,simx_cmd_set_joint_target_velocity,operationMode,0,jointHandle,targetVelocity,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxSetJointTargetPosition(simxInt clientID,simxInt jointHandle,simxFloat targetPosition,simxInt operationMode)
{
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_set_joint_target_position,jointHandle));
    _exec_int_float(clientID,simx_cmd_set_joint_target_position,operationMode,0,jointHandle,targetPosition,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxReadProximitySensor(simxInt clientID,simxInt sensorHandle,simxUChar* detectionState,simxFloat* detectedPoint,simxInt* detectedObjectHandle,simxFloat* detectedSurfaceNormalVector,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_read_proximity_sensor,sensorHandle));
    dataPointer=_exec_int(clientID,simx_cmd_read_proximity_sensor,operationMode,0,sensorHandle,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
    {
        if (detectionState!=0)
            detectionState[0]=_readPureDataChar(dataPointer,0,0);
        if (detectedPoint!=0)
        {
            detectedPoint[0]=_readPureDataFloat(dataPointer,0,1);
            detectedPoint[1]=_readPureDataFloat(dataPointer,0,5);
            detectedPoint[2]=_readPureDataFloat(dataPointer,0,9);
        }
        if (detectedObjectHandle!=0)
            detectedObjectHandle[0]=_readPureDataInt(dataPointer,0,13);
        if (detectedSurfaceNormalVector!=0)
        {
            detectedSurfaceNormalVector[0]=_readPureDataFloat(dataPointer,0,17);
            detectedSurfaceNormalVector[1]=_readPureDataFloat(dataPointer,0,21);
            detectedSurfaceNormalVector[2]=_readPureDataFloat(dataPointer,0,25);
        }
    }
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxStartSimulation(simxInt clientID,simxInt operationMode)
{
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_start_pause_stop_simulation,0));
    _exec_int(clientID,simx_cmd_start_pause_stop_simulation,operationMode,0,0,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxPauseSimulation(simxInt clientID,simxInt operationMode)
{
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_start_pause_stop_simulation,1));
    _exec_int(clientID,simx_cmd_start_pause_stop_simulation,operationMode,0,1,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxStopSimulation(simxInt clientID,simxInt operationMode)
{
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_start_pause_stop_simulation,2));
    _exec_int(clientID,simx_cmd_start_pause_stop_simulation,operationMode,0,2,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxGetObjectHandle(simxInt clientID,const simxChar* objectName,simxInt* handle,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_string(clientID,simx_cmd_get_object_handle,(simxUChar*)objectName));
    dataPointer=_exec_string(clientID,simx_cmd_get_object_handle,operationMode,0,(simxUChar*)objectName,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
        handle[0]=_readPureDataInt(dataPointer,0,0);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxGetUIHandle(simxInt clientID,const simxChar* uiName,simxInt* handle,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_string(clientID,simx_cmd_get_ui_handle,(simxUChar*)uiName));
    dataPointer=_exec_string(clientID,simx_cmd_get_ui_handle,operationMode,0,(simxUChar*)uiName,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
        handle[0]=_readPureDataInt(dataPointer,0,0);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxGetVisionSensorImage(simxInt clientID,simxInt sensorHandle,simxInt* resolution,simxUChar** image,simxUChar options,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue;
    simxInt cmd;
    if (options&1)
        cmd=simx_cmd_get_vision_sensor_image_bw;
    else
        cmd=simx_cmd_get_vision_sensor_image_rgb;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,cmd,sensorHandle));
    dataPointer=_exec_int(clientID,cmd,operationMode,0,sensorHandle,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
    {
        resolution[0]=_readPureDataInt(dataPointer,0,0);
        resolution[1]=_readPureDataInt(dataPointer,0,4);
        image[0]=dataPointer+SIMX_SUBHEADER_SIZE+8+_getCmdDataSize(dataPointer);
    }
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxSetVisionSensorImage(simxInt clientID,simxInt sensorHandle,simxUChar* image,simxInt bufferSize,simxUChar options,simxInt operationMode)
{
    simxInt returnValue;
    simxInt cmd;
    if (options&1)
        cmd=simx_cmd_set_vision_sensor_image_bw;
    else
        cmd=simx_cmd_set_vision_sensor_image_rgb;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,cmd,sensorHandle));
    _exec_int_buffer(clientID,cmd,operationMode,0,sensorHandle,image,bufferSize,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxGetVisionSensorDepthBuffer(simxInt clientID,simxInt sensorHandle,simxInt* resolution,simxFloat** buffer,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue;
#ifdef ENDIAN_TEST
    simxInt i;
#endif
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_get_vision_sensor_depth_buffer,sensorHandle));
    dataPointer=_exec_int(clientID,simx_cmd_get_vision_sensor_depth_buffer,operationMode,0,sensorHandle,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
    {
        resolution[0]=_readPureDataInt(dataPointer,0,0);
        resolution[1]=_readPureDataInt(dataPointer,0,4);
        buffer[0]=(simxFloat*)(dataPointer+SIMX_SUBHEADER_SIZE+8+_getCmdDataSize(dataPointer));
#ifdef ENDIAN_TEST
        for (i=0;i<resolution[0]*resolution[1];i++)
            ((simxFloat*)(buffer[0]))[i]=extApi_endianConversionFloat(((simxFloat*)(buffer[0]))[i]);
#endif
    }
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxJointGetForce(simxInt clientID,simxInt jointHandle,simxFloat* force,simxInt operationMode)
{ /* DEPRECATED since V3.1.2 */
    return(simxGetJointForce(clientID,jointHandle,force,operationMode));
}

EXTAPI_DLLEXPORT simxInt simxGetJointForce(simxInt clientID,simxInt jointHandle,simxFloat* force,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_get_joint_force,jointHandle));
    dataPointer=_exec_int(clientID,simx_cmd_get_joint_force,operationMode,0,jointHandle,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
        force[0]=_readPureDataFloat(dataPointer,0,0);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxSetJointForce(simxInt clientID,simxInt jointHandle,simxFloat force,simxInt operationMode)
{
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_set_joint_force,jointHandle));
    _exec_int_float(clientID,simx_cmd_set_joint_force,operationMode,0,jointHandle,force,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxReadForceSensor(simxInt clientID,simxInt forceSensorHandle,simxUChar* state,simxFloat* forceVector,simxFloat* torqueVector,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue,i;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_read_force_sensor,forceSensorHandle));
    dataPointer=_exec_int(clientID,simx_cmd_read_force_sensor,operationMode,0,forceSensorHandle,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
    {
        if (state!=0)
            state[0]=_readPureDataChar(dataPointer,0,0);
        if (forceVector!=0)
        {
            for (i=0;i<3;i++)
                forceVector[i]=_readPureDataFloat(dataPointer,0,1+4*i);
        }
        if (torqueVector!=0)
        {
            for (i=0;i<3;i++)
                torqueVector[i]=_readPureDataFloat(dataPointer,0,1+12+4*i);
        }
    }
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxBreakForceSensor(simxInt clientID,simxInt forceSensorHandle,simxInt operationMode)
{
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_break_force_sensor,forceSensorHandle));
    _exec_int(clientID,simx_cmd_break_force_sensor,operationMode,0,forceSensorHandle,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxReadVisionSensor(simxInt clientID,simxInt sensorHandle,simxUChar* detectionState,simxFloat** auxValues,simxInt** auxValuesCount,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue,i,packetCnt,auxValCnt;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_read_vision_sensor,sensorHandle));
    dataPointer=_exec_int(clientID,simx_cmd_read_vision_sensor,operationMode,0,sensorHandle,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
    {
        if (detectionState!=0)
            detectionState[0]=_readPureDataChar(dataPointer,0,0);
        if ((auxValues!=0)&&(auxValuesCount!=0))
        {
            packetCnt=_readPureDataInt(dataPointer,0,1);
            auxValuesCount[0]=(simxInt*)extApi_allocateBuffer(4*(1+packetCnt));
            auxValuesCount[0][0]=packetCnt;
            auxValCnt=0;
            for (i=0;i<packetCnt;i++)
            {
                auxValuesCount[0][1+i]=_readPureDataInt(dataPointer,0,1+4*(1+i));
                auxValCnt=auxValCnt+auxValuesCount[0][1+i];
            }
            auxValues[0]=(simxFloat*)extApi_allocateBuffer(4*auxValCnt);
            for (i=0;i<auxValCnt;i++)
                auxValues[0][i]=_readPureDataFloat(dataPointer,0,1+4*(1+packetCnt+i));
        }
    }
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxGetObjectParent(simxInt clientID,simxInt childObjectHandle,simxInt* parentObjectHandle,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_get_object_parent,childObjectHandle));
    dataPointer=_exec_int(clientID,simx_cmd_get_object_parent,operationMode,0,childObjectHandle,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
        parentObjectHandle[0]=_readPureDataInt(dataPointer,0,0);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxGetObjectChild(simxInt clientID,simxInt parentObjectHandle,simxInt childIndex,simxInt* childObjectHandle,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_intint(clientID,simx_cmd_get_object_child,parentObjectHandle,childIndex));
    dataPointer=_exec_intint(clientID,simx_cmd_get_object_child,operationMode,0,parentObjectHandle,childIndex,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
        childObjectHandle[0]=_readPureDataInt(dataPointer,0,0);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxTransferFile(simxInt clientID,const simxChar* filePathAndName,const simxChar* fileName_serverSide,simxInt timeOut,simxInt operationMode)
{
    simxInt returnValue=0;
    simxInt bufferLength,tmpTimeout;
    simxUChar* buffer;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_string(clientID,simx_cmd_transfer_file,(simxUChar*)filePathAndName));
    buffer=extApi_readFile(filePathAndName,&bufferLength);
    if (buffer==0)
        return(simx_return_local_error_flag);
    tmpTimeout=_replyWaitTimeoutInMs[clientID];
    _replyWaitTimeoutInMs[clientID]=timeOut;
    _exec_string_buffer(clientID,simx_cmd_transfer_file,operationMode,0,(simxUChar*)fileName_serverSide,buffer,bufferLength,&returnValue);
    _replyWaitTimeoutInMs[clientID]=tmpTimeout;
    extApi_releaseBuffer(buffer);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxEraseFile(simxInt clientID,const simxChar* fileName_serverSide,simxInt operationMode)
{
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_string(clientID,simx_cmd_erase_file,(simxUChar*)fileName_serverSide));
    _exec_string(clientID,simx_cmd_erase_file,operationMode,0,(simxUChar*)fileName_serverSide,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxLoadModel(simxInt clientID,const simxChar* modelPathAndName,simxUChar options,simxInt* baseHandle,simxInt operationMode)
{
    simxUChar* dataPointer=0;
    simxInt returnValue;
    simxChar tmpFileName[]="REMOTE_API_TEMPFILE_XXXX.ttm";
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_string(clientID,simx_cmd_load_model,(simxUChar*)modelPathAndName));
    if (options&1)
    { /* give some pseudo-random name to the temp file: */
        tmpFileName[20]='0'+(char)(extApi_rand()*9.1f);
        tmpFileName[21]='0'+(char)(extApi_rand()*9.1f);
        tmpFileName[22]='0'+(char)(extApi_rand()*9.1f);
        tmpFileName[23]='0'+(char)(extApi_rand()*9.1f);
        returnValue=simxTransferFile(clientID,modelPathAndName,tmpFileName,_replyWaitTimeoutInMs[clientID],simx_opmode_blocking);
        if (returnValue==0)
        {
            dataPointer=_exec_string(clientID,simx_cmd_load_model,operationMode,0,(simxUChar*)tmpFileName,&returnValue);
            simxEraseFile(clientID,tmpFileName,simx_opmode_oneshot);
        }
        simxTransferFile(clientID,modelPathAndName,tmpFileName,_replyWaitTimeoutInMs[clientID],simx_opmode_remove);
    }
    else
        dataPointer=_exec_string(clientID,simx_cmd_load_model,operationMode,0,(simxUChar*)modelPathAndName,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0)&&(baseHandle!=0))
        baseHandle[0]=_readPureDataInt(dataPointer,0,0);

    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxLoadUI(simxInt clientID,const simxChar* uiPathAndName,simxUChar options,simxInt* count,simxInt** uiHandles,simxInt operationMode)
{
    simxUChar* dataPointer=0;
    simxInt returnValue,i;
    simxChar tmpFileName[]="REMOTE_API_TEMPFILE_XXXX.ttb";
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_string(clientID,simx_cmd_load_ui,(simxUChar*)uiPathAndName));
    if (options&1)
    { /* give some pseudo-random name to the temp file: */
        tmpFileName[20]='0'+(char)(extApi_rand()*9.1f);
        tmpFileName[21]='0'+(char)(extApi_rand()*9.1f);
        tmpFileName[22]='0'+(char)(extApi_rand()*9.1f);
        tmpFileName[23]='0'+(char)(extApi_rand()*9.1f);
        returnValue=simxTransferFile(clientID,uiPathAndName,tmpFileName,_replyWaitTimeoutInMs[clientID],simx_opmode_blocking);
        if (returnValue==0)
        {
            dataPointer=_exec_string(clientID,simx_cmd_load_ui,operationMode,0,(simxUChar*)tmpFileName,&returnValue);
            simxEraseFile(clientID,tmpFileName,simx_opmode_oneshot);
        }
        simxTransferFile(clientID,uiPathAndName,tmpFileName,_replyWaitTimeoutInMs[clientID],simx_opmode_remove);
    }
    else
        dataPointer=_exec_string(clientID,simx_cmd_load_ui,operationMode,0,(simxUChar*)uiPathAndName,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
    {
        count[0]=_readPureDataInt(dataPointer,0,0);
        uiHandles[0]=(simxInt*)extApi_allocateBuffer(4*count[0]);
        for (i=0;i<count[0];i++)
            ((simxInt*)(uiHandles[0]))[i]=_readPureDataInt(dataPointer,0,4+4*i);
    }
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxLoadScene(simxInt clientID,const simxChar* scenePathAndName,simxUChar options,simxInt operationMode)
{
    simxInt returnValue;
    simxChar tmpFileName[]="REMOTE_API_TEMPFILE_XXXX.ttt";
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_string(clientID,simx_cmd_load_scene,(simxUChar*)scenePathAndName));
    if (options&1)
    { /* give some pseudo-random name to the temp file: */
        tmpFileName[20]='0'+(char)(extApi_rand()*9.1f);
        tmpFileName[21]='0'+(char)(extApi_rand()*9.1f);
        tmpFileName[22]='0'+(char)(extApi_rand()*9.1f);
        tmpFileName[23]='0'+(char)(extApi_rand()*9.1f);
        returnValue=simxTransferFile(clientID,scenePathAndName,tmpFileName,_replyWaitTimeoutInMs[clientID],simx_opmode_blocking); 
        if (returnValue==0)
        {
            _exec_string(clientID,simx_cmd_load_scene,operationMode,0,(simxUChar*)tmpFileName,&returnValue);
            simxEraseFile(clientID,tmpFileName,simx_opmode_oneshot);
        }
        simxTransferFile(clientID,scenePathAndName,tmpFileName,_replyWaitTimeoutInMs[clientID],simx_opmode_remove);
    }
    else
        _exec_string(clientID,simx_cmd_load_scene,operationMode,0,(simxUChar*)scenePathAndName,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxGetUISlider(simxInt clientID,simxInt uiHandle,simxInt uiButtonID,simxInt* position,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_intint(clientID,simx_cmd_get_ui_slider,uiHandle,uiButtonID));
    dataPointer=_exec_intint(clientID,simx_cmd_get_ui_slider,operationMode,0,uiHandle,uiButtonID,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
        position[0]=_readPureDataInt(dataPointer,0,0);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxSetUISlider(simxInt clientID,simxInt uiHandle,simxInt uiButtonID,simxInt position,simxInt operationMode)
{
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_intint(clientID,simx_cmd_set_ui_slider,uiHandle,uiButtonID));
    _exec_intint_int(clientID,simx_cmd_set_ui_slider,operationMode,0,uiHandle,uiButtonID,position,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxGetUIEventButton(simxInt clientID,simxInt uiHandle,simxInt* uiEventButtonID,simxInt* auxValues,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_get_ui_event_button,uiHandle));
    dataPointer=_exec_int(clientID,simx_cmd_get_ui_event_button,operationMode,0,uiHandle,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
    {
        uiEventButtonID[0]=_readPureDataInt(dataPointer,0,0);
        if (auxValues!=0)
        {
            auxValues[0]=_readPureDataInt(dataPointer,0,4);
            auxValues[1]=_readPureDataInt(dataPointer,0,8);
        }
        /* ******* SPECIAL CASE FOR THIS COMMAND ONLY !! ******** */
        if ((operationMode==simx_opmode_buffer)&&(uiEventButtonID[0]!=-1))
            _removeCommandReply_int(clientID,simx_cmd_get_ui_event_button,uiHandle); /* We received an event! The continuous command was automatically deactivated on the server side. We remove the reply  in the input buffer on the client here! */
        /* ****************************************************** */
    }
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxGetUIButtonProperty(simxInt clientID,simxInt uiHandle,simxInt uiButtonID,simxInt* prop,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_intint(clientID,simx_cmd_get_ui_button_property,uiHandle,uiButtonID));
    dataPointer=_exec_intint(clientID,simx_cmd_get_ui_button_property,operationMode,0,uiHandle,uiButtonID,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
        prop[0]=_readPureDataInt(dataPointer,0,0);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxSetUIButtonProperty(simxInt clientID,simxInt uiHandle,simxInt uiButtonID,simxInt prop,simxInt operationMode)
{
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_intint(clientID,simx_cmd_set_ui_button_property,uiHandle,uiButtonID));
    _exec_intint_int(clientID,simx_cmd_set_ui_button_property,operationMode,0,uiHandle,uiButtonID,prop,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxAddStatusbarMessage(simxInt clientID,const simxChar* message,simxInt operationMode)
{
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_string(clientID,simx_cmd_add_statusbar_message,(simxUChar*)message));
    _exec_string(clientID,simx_cmd_add_statusbar_message,operationMode,1,(simxUChar*)message,&returnValue);
    return(returnValue);
}


EXTAPI_DLLEXPORT simxInt simxCreateDummy(simxInt clientID,simxFloat size,const simxUChar* colors,simxInt* objectHandle,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue,i;
    simxUChar buffer[4+1+12];
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_null(clientID,simx_cmd_create_dummy));

    ((simxFloat*)buffer)[0]=extApi_endianConversionFloat(size);
    if (colors==NULL)
        buffer[4+0]=0; /* indicates default colors */
    else
    {
        buffer[4+0]=1;
        for (i=0;i<12;i++)
            buffer[4+1+i]=colors[i];
    }
    dataPointer=_exec_null_buffer(clientID,simx_cmd_create_dummy,operationMode,1,buffer,4+1+12,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
        objectHandle[0]=_readPureDataInt(dataPointer,0,0);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxCallScriptFunction(simxInt clientID,const simxChar* scriptDescription,simxInt options,const simxChar* functionName,simxInt inIntCnt,const simxInt* inInt,simxInt inFloatCnt,const simxFloat* inFloat,simxInt inStringCnt,const simxChar* inString,simxInt inBufferSize,const simxUChar* inBuffer,simxInt* outIntCnt,simxInt** outInt,simxInt* outFloatCnt,simxFloat** outFloat,simxInt* outStringCnt,simxChar** outString,simxInt* outBufferSize,simxUChar** outBuffer,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue;
    simxUChar* buffer;
    simxInt i,off,totStrSize,bufferSize,outIntC,outFloatC,outStringC,outBufferS;
    simxChar* outStr;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_intstringstring(clientID,simx_cmd_aux_console_open,options,(simxUChar*)scriptDescription,(simxUChar*)functionName));

    totStrSize=0;
    for (i=0;i<inStringCnt;i++)
        totStrSize+=extApi_getStringLength(inString+totStrSize)+1;
    bufferSize=4+4+4+4+4*(inIntCnt+inFloatCnt)+totStrSize+inBufferSize;
    buffer=extApi_allocateBuffer(bufferSize);

    ((simxInt*)buffer)[0]=extApi_endianConversionInt(inIntCnt);
    ((simxInt*)buffer)[1]=extApi_endianConversionInt(inFloatCnt);
    ((simxInt*)buffer)[2]=extApi_endianConversionInt(inStringCnt);
    ((simxInt*)buffer)[3]=extApi_endianConversionInt(inBufferSize);
    off=4;
    for (i=0;i<inIntCnt;i++)
        ((simxInt*)buffer)[off+i]=extApi_endianConversionInt(inInt[i]);
    off+=inIntCnt;
    for (i=0;i<inFloatCnt;i++)
        ((simxFloat*)buffer)[off+i]=extApi_endianConversionFloat(inFloat[i]);
    off+=inFloatCnt;
    off*=4;
    for (i=0;i<totStrSize;i++)
        buffer[off+i]=(simxUChar)inString[i];
    off+=totStrSize;
    for (i=0;i<inBufferSize;i++)
        buffer[off+i]=inBuffer[i];

    dataPointer=_exec_intstringstring_buffer(clientID,simx_cmd_call_script_function,operationMode,1,options,(simxUChar*)scriptDescription,(simxUChar*)functionName,buffer,bufferSize,&returnValue);

    extApi_releaseBuffer(buffer);

    if ((dataPointer!=0)&&(returnValue==0))
    {
        outIntC=_readPureDataInt(dataPointer,0,0);
        outFloatC=_readPureDataInt(dataPointer,0,4);
        outStringC=_readPureDataInt(dataPointer,0,8);
        outBufferS=_readPureDataInt(dataPointer,0,12);
        off=4+4+4+4;

        if ((outIntCnt!=0)&&(outInt!=0))
        {
            outIntCnt[0]=outIntC;
            outInt[0]=((simxInt*)(dataPointer+SIMX_SUBHEADER_SIZE+_getCmdDataSize(dataPointer)+off)); /* little/big endian conversion happened on the server side */
        }
        off+=outIntC*4;

        if ((outFloatCnt!=0)&&(outFloat!=0))
        {
            outFloatCnt[0]=outFloatC;
            outFloat[0]=((simxFloat*)(dataPointer+SIMX_SUBHEADER_SIZE+_getCmdDataSize(dataPointer)+off)); /* little/big endian conversion happened on the server side */
        }
        off+=outFloatC*4;

        outStr=((simxChar*)(dataPointer+SIMX_SUBHEADER_SIZE+_getCmdDataSize(dataPointer)+off));
        if ((outStringCnt!=0)&&(outString!=0))
        {
            outStringCnt[0]=outStringC;
            outString[0]=outStr;
        }
        totStrSize=0;
        for (i=0;i<outStringC;i++)
            totStrSize+=extApi_getStringLength(outStr+totStrSize)+1;
        off+=totStrSize;

        if ((outBufferSize!=0)&&(outBuffer!=0))
        {
            outBufferSize[0]=outBufferS;
            outBuffer[0]=((simxUChar*)(dataPointer+SIMX_SUBHEADER_SIZE+_getCmdDataSize(dataPointer)+off));
        }
    }
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxAuxiliaryConsoleOpen(simxInt clientID,const simxChar* title,simxInt maxLines,simxInt mode,simxInt* position,simxInt* size,simxFloat* textColor,simxFloat* backgroundColor,simxInt* consoleHandle,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue;
    simxUChar buffer[12*4];
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_string(clientID,simx_cmd_aux_console_open,(simxUChar*)title));
    ((simxInt*)buffer)[0]=extApi_endianConversionInt(maxLines);
    ((simxInt*)buffer)[1]=extApi_endianConversionInt(mode);
    ((simxInt*)buffer)[2]=extApi_endianConversionInt(98765);
    if (position!=0)
    {
        ((simxInt*)buffer)[2]=extApi_endianConversionInt(position[0]);
        ((simxInt*)buffer)[3]=extApi_endianConversionInt(position[1]);
    }
    ((simxInt*)buffer)[4]=extApi_endianConversionInt(98765);
    if (size!=0)
    {
        ((simxInt*)buffer)[4]=extApi_endianConversionInt(size[0]);
        ((simxInt*)buffer)[5]=extApi_endianConversionInt(size[1]);
    }
    ((simxFloat*)buffer)[6]=extApi_endianConversionFloat(-10.0f);
    if (textColor!=0)
    {
        ((simxFloat*)buffer)[6]=extApi_endianConversionFloat(textColor[0]);
        ((simxFloat*)buffer)[7]=extApi_endianConversionFloat(textColor[1]);
        ((simxFloat*)buffer)[8]=extApi_endianConversionFloat(textColor[2]);
    }
    ((simxFloat*)buffer)[9]=extApi_endianConversionFloat(-10.0f);
    if (backgroundColor!=0)
    {
        ((simxFloat*)buffer)[9]=extApi_endianConversionFloat(backgroundColor[0]);
        ((simxFloat*)buffer)[10]=extApi_endianConversionFloat(backgroundColor[1]);
        ((simxFloat*)buffer)[11]=extApi_endianConversionFloat(backgroundColor[2]);
    }
    dataPointer=_exec_string_buffer(clientID,simx_cmd_aux_console_open,operationMode,0,(simxUChar*)title,buffer,12*4,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
        consoleHandle[0]=_readPureDataInt(dataPointer,0,0);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxAuxiliaryConsoleClose(simxInt clientID,simxInt consoleHandle,simxInt operationMode)
{
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_aux_console_close,consoleHandle));
    _exec_int(clientID,simx_cmd_aux_console_close,operationMode,0,consoleHandle,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxAuxiliaryConsolePrint(simxInt clientID,simxInt consoleHandle,const simxChar* txt,simxInt operationMode)
{
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_aux_console_print,consoleHandle));
    if (txt==0)
        _exec_int_buffer(clientID,simx_cmd_aux_console_print,operationMode,1,consoleHandle,0,0,&returnValue);
    else
        _exec_int_buffer(clientID,simx_cmd_aux_console_print,operationMode,1,consoleHandle,(simxUChar*)txt,extApi_getStringLength(txt)+1,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxAuxiliaryConsoleShow(simxInt clientID,simxInt consoleHandle,simxUChar showState,simxInt operationMode)
{
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_aux_console_show,consoleHandle));
    _exec_int_int(clientID,simx_cmd_aux_console_show,operationMode,0,consoleHandle,(simxInt)showState,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxGetObjectOrientation(simxInt clientID,simxInt objectHandle,simxInt relativeToObjectHandle,simxFloat* eulerAngles,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_intint(clientID,simx_cmd_get_object_orientation2,objectHandle,relativeToObjectHandle));
    dataPointer=_exec_intint(clientID,simx_cmd_get_object_orientation2,operationMode,0,objectHandle,relativeToObjectHandle,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
    {
        eulerAngles[0]=_readPureDataFloat(dataPointer,0,0);
        eulerAngles[1]=_readPureDataFloat(dataPointer,0,4);
        eulerAngles[2]=_readPureDataFloat(dataPointer,0,8);
    }
    return(returnValue);
    /* until 10/6/2014
    simxUChar* dataPointer;
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_get_object_orientation,objectHandle));
    dataPointer=_exec_int_int(clientID,simx_cmd_get_object_orientation,operationMode,0,objectHandle,relativeToObjectHandle,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
    {
        eulerAngles[0]=_readPureDataFloat(dataPointer,0,0);
        eulerAngles[1]=_readPureDataFloat(dataPointer,0,4);
        eulerAngles[2]=_readPureDataFloat(dataPointer,0,8);
    }
    return(returnValue);
    */
}

EXTAPI_DLLEXPORT simxInt simxGetObjectPosition(simxInt clientID,simxInt objectHandle,simxInt relativeToObjectHandle,simxFloat* position,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_intint(clientID,simx_cmd_get_object_position2,objectHandle,relativeToObjectHandle));
    dataPointer=_exec_intint(clientID,simx_cmd_get_object_position2,operationMode,0,objectHandle,relativeToObjectHandle,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
    {
        position[0]=_readPureDataFloat(dataPointer,0,0);
        position[1]=_readPureDataFloat(dataPointer,0,4);
        position[2]=_readPureDataFloat(dataPointer,0,8);
    }
    return(returnValue);
    /* until 10/6/2014
    simxUChar* dataPointer;
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_get_object_position,objectHandle));
    dataPointer=_exec_int_int(clientID,simx_cmd_get_object_position,operationMode,0,objectHandle,relativeToObjectHandle,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
    {
        position[0]=_readPureDataFloat(dataPointer,0,0);
        position[1]=_readPureDataFloat(dataPointer,0,4);
        position[2]=_readPureDataFloat(dataPointer,0,8);
    }
    return(returnValue);
    */
}

EXTAPI_DLLEXPORT simxInt simxSetObjectOrientation(simxInt clientID,simxInt objectHandle,simxInt relativeToObjectHandle,const simxFloat* eulerAngles,simxInt operationMode)
{
    simxInt returnValue;
    simxUChar buffer[4*4];
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_set_object_orientation,objectHandle));
    ((simxInt*)buffer)[0]=extApi_endianConversionInt(relativeToObjectHandle);
    ((simxFloat*)buffer)[1]=extApi_endianConversionFloat(eulerAngles[0]);
    ((simxFloat*)buffer)[2]=extApi_endianConversionFloat(eulerAngles[1]);
    ((simxFloat*)buffer)[3]=extApi_endianConversionFloat(eulerAngles[2]);
    _exec_int_buffer(clientID,simx_cmd_set_object_orientation,operationMode,0,objectHandle,buffer,4*4,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxSetObjectPosition(simxInt clientID,simxInt objectHandle,simxInt relativeToObjectHandle,const simxFloat* position,simxInt operationMode)
{
    simxInt returnValue;
    simxUChar buffer[4*4];
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_set_object_position,objectHandle));
    ((simxInt*)buffer)[0]=extApi_endianConversionInt(relativeToObjectHandle);
    ((simxFloat*)buffer)[1]=extApi_endianConversionFloat(position[0]);
    ((simxFloat*)buffer)[2]=extApi_endianConversionFloat(position[1]);
    ((simxFloat*)buffer)[3]=extApi_endianConversionFloat(position[2]);
    _exec_int_buffer(clientID,simx_cmd_set_object_position,operationMode,0,objectHandle,buffer,4*4,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxSetObjectParent(simxInt clientID,simxInt objectHandle,simxInt parentObject,simxUChar keepInPlace,simxInt operationMode)
{
    simxInt returnValue;
    simxUChar buffer[4+1];
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_set_object_parent,objectHandle));
    ((simxInt*)buffer)[0]=extApi_endianConversionInt(parentObject);
    buffer[4]=keepInPlace;
    _exec_int_buffer(clientID,simx_cmd_set_object_parent,operationMode,0,objectHandle,buffer,4+1,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxSetUIButtonLabel(simxInt clientID,simxInt uiHandle,simxInt uiButtonID,const simxChar* upStateLabel,const simxChar* downStateLabel,simxInt operationMode)
{
    simxInt returnValue;
    simxInt strL1,strL2,i;
    simxUChar* buffer;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_intint(clientID,simx_cmd_set_ui_button_label,uiHandle,uiButtonID));
    strL1=extApi_getStringLength(upStateLabel);
    strL2=extApi_getStringLength(downStateLabel);
    buffer=extApi_allocateBuffer(strL1+strL2+1+1);
    for (i=0;i<strL1+1;i++)
        buffer[i]=upStateLabel[i];
    for (i=0;i<strL2+1;i++)
        buffer[strL1+1+i]=downStateLabel[i];
    _exec_intint_buffer(clientID,simx_cmd_set_ui_button_label,operationMode,0,uiHandle,uiButtonID,buffer,strL1+strL2+1+1,&returnValue);
    extApi_releaseBuffer(buffer);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxGetLastErrors(simxInt clientID,simxInt* errorCnt,simxChar** errorStrings,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_null(clientID,simx_cmd_get_last_errors));
    dataPointer=_exec_null(clientID,simx_cmd_get_last_errors,operationMode,0,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
    {
        errorCnt[0]=_readPureDataInt(dataPointer,0,0);
        errorStrings[0]=(simxChar*)dataPointer+SIMX_SUBHEADER_SIZE+4+_getCmdDataSize(dataPointer);
    }
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxGetArrayParameter(simxInt clientID,simxInt paramIdentifier,simxFloat* paramValues,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue,i;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_get_array_parameter,paramIdentifier));
    dataPointer=_exec_int(clientID,simx_cmd_get_array_parameter,operationMode,0,paramIdentifier,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
    {
        for (i=0;i<3;i++)
            paramValues[i]=_readPureDataFloat(dataPointer,0,4*i);
    }
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxSetArrayParameter(simxInt clientID,simxInt paramIdentifier,const simxFloat* paramValues,simxInt operationMode)
{
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_set_array_parameter,paramIdentifier));
    _exec_int_buffer(clientID,simx_cmd_set_array_parameter,operationMode,0,paramIdentifier,(simxUChar*)paramValues,3*4,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxGetBooleanParameter(simxInt clientID,simxInt paramIdentifier,simxUChar* paramValue,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_get_boolean_parameter,paramIdentifier));
    dataPointer=_exec_int(clientID,simx_cmd_get_boolean_parameter,operationMode,0,paramIdentifier,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
        paramValue[0]=(simxUChar)_readPureDataInt(dataPointer,0,0);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxSetBooleanParameter(simxInt clientID,simxInt paramIdentifier,simxUChar paramValue,simxInt operationMode)
{
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_set_boolean_parameter,paramIdentifier));
    _exec_int_int(clientID,simx_cmd_set_boolean_parameter,operationMode,0,paramIdentifier,(simxInt)paramValue,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxGetIntegerParameter(simxInt clientID,simxInt paramIdentifier,simxInt* paramValue,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_get_integer_parameter,paramIdentifier));
    dataPointer=_exec_int(clientID,simx_cmd_get_integer_parameter,operationMode,0,paramIdentifier,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
        paramValue[0]=_readPureDataInt(dataPointer,0,0);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxSetIntegerParameter(simxInt clientID,simxInt paramIdentifier,simxInt paramValue,simxInt operationMode)
{
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_set_integer_parameter,paramIdentifier));
    _exec_int_int(clientID,simx_cmd_set_integer_parameter,operationMode,0,paramIdentifier,paramValue,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxGetFloatingParameter(simxInt clientID,simxInt paramIdentifier,simxFloat* paramValue,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_get_floating_parameter,paramIdentifier));
    dataPointer=_exec_int(clientID,simx_cmd_get_floating_parameter,operationMode,0,paramIdentifier,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
        paramValue[0]=_readPureDataFloat(dataPointer,0,0);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxSetFloatingParameter(simxInt clientID,simxInt paramIdentifier,simxFloat paramValue,simxInt operationMode)
{
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_set_floating_parameter,paramIdentifier));
    _exec_int_float(clientID,simx_cmd_set_floating_parameter,operationMode,0,paramIdentifier,paramValue,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxGetStringParameter(simxInt clientID,simxInt paramIdentifier,simxChar** paramValue,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_get_string_parameter,paramIdentifier));
    dataPointer=_exec_int(clientID,simx_cmd_get_string_parameter,operationMode,0,paramIdentifier,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
        paramValue[0]=(simxChar*)dataPointer+SIMX_SUBHEADER_SIZE+_getCmdDataSize(dataPointer);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxGetCollisionHandle(simxInt clientID,const simxChar* collisionObjectName,simxInt* handle,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_string(clientID,simx_cmd_get_collision_handle,(simxUChar*)collisionObjectName));
    dataPointer=_exec_string(clientID,simx_cmd_get_collision_handle,operationMode,0,(simxUChar*)collisionObjectName,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
        handle[0]=_readPureDataInt(dataPointer,0,0);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxGetDistanceHandle(simxInt clientID,const simxChar* distanceObjectName,simxInt* handle,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_string(clientID,simx_cmd_get_distance_handle,(simxUChar*)distanceObjectName));
    dataPointer=_exec_string(clientID,simx_cmd_get_distance_handle,operationMode,0,(simxUChar*)distanceObjectName,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
        handle[0]=_readPureDataInt(dataPointer,0,0);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxGetCollectionHandle(simxInt clientID,const simxChar* collectionName,simxInt* handle,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_string(clientID,simx_cmd_get_collection_handle,(simxUChar*)collectionName));
    dataPointer=_exec_string(clientID,simx_cmd_get_collection_handle,operationMode,0,(simxUChar*)collectionName,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
        handle[0]=_readPureDataInt(dataPointer,0,0);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxReadCollision(simxInt clientID,simxInt collisionObjectHandle,simxUChar* collisionState,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_read_collision,collisionObjectHandle));
    dataPointer=_exec_int(clientID,simx_cmd_read_collision,operationMode,0,collisionObjectHandle,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
        collisionState[0]=(simxUChar)_readPureDataInt(dataPointer,0,0);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxReadDistance(simxInt clientID,simxInt distanceObjectHandle,simxFloat* minimumDistance,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_read_distance,distanceObjectHandle));
    dataPointer=_exec_int(clientID,simx_cmd_read_distance,operationMode,0,distanceObjectHandle,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
        minimumDistance[0]=_readPureDataFloat(dataPointer,0,0);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxRemoveObject(simxInt clientID,simxInt objectHandle,simxInt operationMode)
{
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_remove_object,objectHandle));
    _exec_int(clientID,simx_cmd_remove_object,operationMode,0,objectHandle,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxRemoveModel(simxInt clientID,simxInt objectHandle,simxInt operationMode)
{
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_remove_model,objectHandle));
    _exec_int(clientID,simx_cmd_remove_model,operationMode,0,objectHandle,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxRemoveUI(simxInt clientID,simxInt uiHandle,simxInt operationMode)
{
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_remove_ui,uiHandle));
    _exec_int(clientID,simx_cmd_remove_ui,operationMode,0,uiHandle,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxCloseScene(simxInt clientID,simxInt operationMode)
{
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_null(clientID,simx_cmd_close_scene));
    _exec_null(clientID,simx_cmd_close_scene,operationMode,0,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxGetObjects(simxInt clientID,simxInt objectType,simxInt* objectCount,simxInt** objectHandles,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue,off;
#ifdef ENDIAN_TEST
    simxInt i;
#endif
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_get_objects,objectType));
    dataPointer=_exec_int(clientID,simx_cmd_get_objects,operationMode,0,objectType,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
    {
        objectCount[0]=_readPureDataInt(dataPointer,0,0);
        off=SIMX_SUBHEADER_SIZE+_getCmdDataSize(dataPointer)+4;
#ifdef ENDIAN_TEST
        for (i=0;i<objectCount[0];i++)
            ((simxInt*)(dataPointer+off))[i]=extApi_endianConversionInt(((simxInt*)(dataPointer+off))[i]);
#endif
        objectHandles[0]=((simxInt*)(dataPointer+off));
    }
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxDisplayDialog(simxInt clientID,const simxChar* titleText,const simxChar* mainText,simxInt dialogType,const simxChar* initialText,const simxFloat* titleColors,const simxFloat* dialogColors,simxInt* dialogHandle,simxInt* uiHandle,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue,str1L,str2L,i,off;
    simxUChar* buffer;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_string(clientID,simx_cmd_display_dialog,(simxUChar*)titleText));
    str1L=extApi_getStringLength(mainText);
    str2L=extApi_getStringLength(initialText);
    buffer=extApi_allocateBuffer(str1L+1+4+str2L+1+4*6+4*6);
    off=0;
    for (i=0;i<str1L+1;i++)
        buffer[off+i]=mainText[i];
    off+=str1L+1;
    ((simxInt*)(buffer+off))[0]=extApi_endianConversionInt(dialogType);
    off+=4;
    for (i=0;i<str2L+1;i++)
        buffer[off+i]=initialText[i];
    off+=str2L+1;
    ((simxFloat*)(buffer+off))[0]=extApi_endianConversionFloat(-10.0f);
    if (titleColors!=0)
    {
        for (i=0;i<6;i++)
            ((simxFloat*)(buffer+off))[i]=extApi_endianConversionFloat(titleColors[i]);
    }
    off+=4*6;
    ((simxFloat*)(buffer+off))[0]=extApi_endianConversionFloat(-10.0f);
    if (dialogColors!=0)
    {
        for (i=0;i<6;i++)
            ((simxFloat*)(buffer+off))[i]=extApi_endianConversionFloat(dialogColors[i]);
    }
    dataPointer=_exec_string_buffer(clientID,simx_cmd_display_dialog,operationMode,0,(simxUChar*)titleText,buffer,str1L+1+4+str2L+1+4*6+4*6,&returnValue);
    extApi_releaseBuffer(buffer);
    if ((dataPointer!=0)&&(returnValue==0))
    {
        dialogHandle[0]=_readPureDataInt(dataPointer,0,0);
        if (uiHandle!=NULL)
            uiHandle[0]=_readPureDataInt(dataPointer,0,4);
    }
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxEndDialog(simxInt clientID,simxInt dialogHandle,simxInt operationMode)
{
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_end_dialog,dialogHandle));
    _exec_int(clientID,simx_cmd_end_dialog,operationMode,0,dialogHandle,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxGetDialogInput(simxInt clientID,simxInt dialogHandle,simxChar** inputText,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_get_dialog_input,dialogHandle));
    dataPointer=_exec_int(clientID,simx_cmd_get_dialog_input,operationMode,0,dialogHandle,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
    {
        inputText[0]=(simxChar*)dataPointer+SIMX_SUBHEADER_SIZE+_getCmdDataSize(dataPointer);
    }
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxGetDialogResult(simxInt clientID,simxInt dialogHandle,simxInt* result,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_get_dialog_result,dialogHandle));
    dataPointer=_exec_int(clientID,simx_cmd_get_dialog_result,operationMode,0,dialogHandle,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
    {
        result[0]=_readPureDataInt(dataPointer,0,0);
    }
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxCopyPasteObjects(simxInt clientID,const simxInt* objectHandles,simxInt objectCount,simxInt** newObjectHandles,simxInt* newObjectCount,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue,off;
#ifdef ENDIAN_TEST
    simxInt i;
#endif
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_copy_paste_objects,0));
    dataPointer=_exec_int_buffer(clientID,simx_cmd_copy_paste_objects,operationMode,1,0,(simxUChar*)objectHandles,objectCount*4,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
    {
        newObjectCount[0]=_readPureDataInt(dataPointer,0,0);
        off=SIMX_SUBHEADER_SIZE+_getCmdDataSize(dataPointer)+4;
#ifdef ENDIAN_TEST
        for (i=0;i<newObjectCount[0];i++)
            ((simxInt*)(dataPointer+off))[i]=extApi_endianConversionInt(((simxInt*)(dataPointer+off))[i]);
#endif
        newObjectHandles[0]=((simxInt*)(dataPointer+off));
    }
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxGetObjectSelection(simxInt clientID,simxInt** objectHandles,simxInt* objectCount,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue,off;
#ifdef ENDIAN_TEST
    simxInt i;
#endif
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_null(clientID,simx_cmd_get_object_selection));
    dataPointer=_exec_null(clientID,simx_cmd_get_object_selection,operationMode,0,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
    {
        objectCount[0]=_readPureDataInt(dataPointer,0,0);
        off=SIMX_SUBHEADER_SIZE+_getCmdDataSize(dataPointer)+4;
#ifdef ENDIAN_TEST
        for (i=0;i<objectCount[0];i++)
            ((simxInt*)(dataPointer+off))[i]=extApi_endianConversionInt(((simxInt*)(dataPointer+off))[i]);
#endif
        objectHandles[0]=((simxInt*)(dataPointer+off));
    }
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxSetObjectSelection(simxInt clientID,const simxInt* objectHandles,simxInt objectCount,simxInt operationMode)
{
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_set_object_selection,0));
    _exec_int_buffer(clientID,simx_cmd_set_object_selection,operationMode,0,0,(simxUChar*)objectHandles,objectCount*4,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxClearFloatSignal(simxInt clientID,const simxChar* signalName,simxInt operationMode)
{
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_string(clientID,simx_cmd_clear_float_signal,(simxUChar*)signalName));
    _exec_string(clientID,simx_cmd_clear_float_signal,operationMode,0,(simxUChar*)signalName,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxClearIntegerSignal(simxInt clientID,const simxChar* signalName,simxInt operationMode)
{
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_string(clientID,simx_cmd_clear_integer_signal,(simxUChar*)signalName));
    _exec_string(clientID,simx_cmd_clear_integer_signal,operationMode,0,(simxUChar*)signalName,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxClearStringSignal(simxInt clientID,const simxChar* signalName,simxInt operationMode)
{
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_string(clientID,simx_cmd_clear_string_signal,(simxUChar*)signalName));
    _exec_string(clientID,simx_cmd_clear_string_signal,operationMode,0,(simxUChar*)signalName,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxGetFloatSignal(simxInt clientID,const simxChar* signalName,simxFloat* signalValue,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_string(clientID,simx_cmd_get_float_signal,(simxUChar*)signalName));
    dataPointer=_exec_string(clientID,simx_cmd_get_float_signal,operationMode,0,(simxUChar*)signalName,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
        signalValue[0]=_readPureDataFloat(dataPointer,0,0);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxGetIntegerSignal(simxInt clientID,const simxChar* signalName,simxInt* signalValue,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_string(clientID,simx_cmd_get_integer_signal,(simxUChar*)signalName));
    dataPointer=_exec_string(clientID,simx_cmd_get_integer_signal,operationMode,0,(simxUChar*)signalName,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
        signalValue[0]=_readPureDataInt(dataPointer,0,0);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxGetStringSignal(simxInt clientID,const simxChar* signalName,simxUChar** signalValue,simxInt* signalLength,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_string(clientID,simx_cmd_get_string_signal,(simxUChar*)signalName));
    dataPointer=_exec_string(clientID,simx_cmd_get_string_signal,operationMode,0,(simxUChar*)signalName,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
    {
        signalValue[0]=dataPointer+SIMX_SUBHEADER_SIZE+_getCmdDataSize(dataPointer);
        signalLength[0]=extApi_endianConversionInt(((simxInt*)(dataPointer+simx_cmdheaderoffset_full_mem_size))[0])-SIMX_SUBHEADER_SIZE-_getCmdDataSize(dataPointer);
    }
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxGetAndClearStringSignal(simxInt clientID,const simxChar* signalName,simxUChar** signalValue,simxInt* signalLength,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_string(clientID,simx_cmd_get_and_clear_string_signal,(simxUChar*)signalName));
    dataPointer=_exec_string(clientID,simx_cmd_get_and_clear_string_signal,operationMode,0,(simxUChar*)signalName,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
    {
        signalValue[0]=dataPointer+SIMX_SUBHEADER_SIZE+_getCmdDataSize(dataPointer);
        signalLength[0]=extApi_endianConversionInt(((simxInt*)(dataPointer+simx_cmdheaderoffset_full_mem_size))[0])-SIMX_SUBHEADER_SIZE-_getCmdDataSize(dataPointer);

        /* ******* SPECIAL CASE FOR THIS COMMAND ONLY !! ******** */
        if (operationMode==simx_opmode_buffer)/* &&(signalLength[0]>0)) */
            _removeCommandReply_string(clientID,simx_cmd_get_and_clear_string_signal,(simxUChar*)signalName); /* We received a signal value! The continuous command was automatically deactivated on the server side. We remove the reply  in the input buffer on the client here! */
        /* ****************************************************** */

    }
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxReadStringStream(simxInt clientID,const simxChar* signalName,simxUChar** signalValue,simxInt* signalLength,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_string(clientID,simx_cmd_read_string_stream,(simxUChar*)signalName));

    /* following 2 lines special here */
    if (operationMode==simx_opmode_blocking)
        return(simx_return_illegal_opmode_flag);

    extApi_lockResources(clientID); /* special here */
    dataPointer=_exec_string(clientID,simx_cmd_read_string_stream,operationMode,0,(simxUChar*)signalName,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
    {
        signalValue[0]=dataPointer+SIMX_SUBHEADER_SIZE+_getCmdDataSize(dataPointer);
        signalLength[0]=extApi_endianConversionInt(((simxInt*)(dataPointer+simx_cmdheaderoffset_full_mem_size))[0])-SIMX_SUBHEADER_SIZE-_getCmdDataSize(dataPointer);

        /* ******* SPECIAL CASE FOR THIS COMMAND ONLY !! ******** */
        _removeCommandReply_string(clientID,simx_cmd_read_string_stream,(simxUChar*)signalName); /* We received a signal value! We remove the reply in the input buffer on the client here! */
        /* ****************************************************** */
    }
    extApi_unlockResources(clientID); /* special here */
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxSetFloatSignal(simxInt clientID,const simxChar* signalName,simxFloat signalValue,simxInt operationMode)
{
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_string(clientID,simx_cmd_set_float_signal,(simxUChar*)signalName));
    signalValue=extApi_endianConversionFloat(signalValue);
    _exec_string_buffer(clientID,simx_cmd_set_float_signal,operationMode,0,(simxUChar*)signalName,(simxUChar*)&signalValue,4,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxSetIntegerSignal(simxInt clientID,const simxChar* signalName,simxInt signalValue,simxInt operationMode)
{
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_string(clientID,simx_cmd_set_integer_signal,(simxUChar*)signalName));
    signalValue=extApi_endianConversionInt(signalValue);
    _exec_string_buffer(clientID,simx_cmd_set_integer_signal,operationMode,0,(simxUChar*)signalName,(simxUChar*)&signalValue,4,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxSetStringSignal(simxInt clientID,const simxChar* signalName,const simxUChar* signalValue,simxInt signalLength,simxInt operationMode)
{
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_string(clientID,simx_cmd_set_string_signal,(simxUChar*)signalName));
    _exec_string_buffer(clientID,simx_cmd_set_string_signal,operationMode,0,(simxUChar*)signalName,(simxUChar*)signalValue,signalLength,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxAppendStringSignal(simxInt clientID,const simxChar* signalName,const simxUChar* signalValue,simxInt signalLength,simxInt operationMode)
{ /* since 31.1.2013: append mode */
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_string(clientID,simx_cmd_append_string_signal,(simxUChar*)signalName)); 
    _exec_string_buffer(clientID,simx_cmd_append_string_signal,operationMode,1,(simxUChar*)signalName,(simxUChar*)signalValue,signalLength,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxWriteStringStream(simxInt clientID,const simxChar* signalName,const simxUChar* signalValue,simxInt signalLength,simxInt operationMode)
{ /* this is just a convenience function */
    return(simxAppendStringSignal(clientID,signalName,signalValue,signalLength,operationMode));
}

EXTAPI_DLLEXPORT simxInt simxGetObjectFloatParameter(simxInt clientID,simxInt objectHandle,simxInt parameterID,simxFloat* parameterValue,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_intint(clientID,simx_cmd_get_object_float_parameter,objectHandle,parameterID));
    dataPointer=_exec_intint(clientID,simx_cmd_get_object_float_parameter,operationMode,0,objectHandle,parameterID,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
        parameterValue[0]=_readPureDataFloat(dataPointer,0,0);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxSetObjectFloatParameter(simxInt clientID,simxInt objectHandle,simxInt parameterID,simxFloat parameterValue,simxInt operationMode)
{
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_intint(clientID,simx_cmd_set_object_float_parameter,objectHandle,parameterID));
    parameterValue=extApi_endianConversionFloat(parameterValue);
    _exec_intint_buffer(clientID,simx_cmd_set_object_float_parameter,operationMode,0,objectHandle,parameterID,(simxUChar*)&parameterValue,4,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxGetObjectIntParameter(simxInt clientID,simxInt objectHandle,simxInt parameterID,simxInt* parameterValue,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_intint(clientID,simx_cmd_get_object_int_parameter,objectHandle,parameterID));
    dataPointer=_exec_intint(clientID,simx_cmd_get_object_int_parameter,operationMode,0,objectHandle,parameterID,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
        parameterValue[0]=_readPureDataInt(dataPointer,0,0);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxSetObjectIntParameter(simxInt clientID,simxInt objectHandle,simxInt parameterID,simxInt parameterValue,simxInt operationMode)
{
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_intint(clientID,simx_cmd_set_object_int_parameter,objectHandle,parameterID));
    parameterValue=extApi_endianConversionInt(parameterValue);
    _exec_intint_buffer(clientID,simx_cmd_set_object_int_parameter,operationMode,0,objectHandle,parameterID,(simxUChar*)&parameterValue,4,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxGetModelProperty(simxInt clientID,simxInt objectHandle,simxInt* prop,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_get_model_property,objectHandle));
    dataPointer=_exec_int(clientID,simx_cmd_get_model_property,operationMode,0,objectHandle,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
        prop[0]=_readPureDataInt(dataPointer,0,0);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxSetModelProperty(simxInt clientID,simxInt objectHandle,simxInt prop,simxInt operationMode)
{
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_set_model_property,objectHandle));
    _exec_int_int(clientID,simx_cmd_set_model_property,operationMode,0,objectHandle,prop,&returnValue);
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxQuery(simxInt clientID,const simxChar* signalName,const simxUChar* signalValue,simxInt signalLength,const simxChar* retSignalName,simxUChar** retSignalValue,simxInt* retSignalLength,simxInt timeOutInMs)
{
    simxInt err;
    simxInt startTime=extApi_getTimeInMs();
    simxClearStringSignal(clientID,retSignalName,simx_opmode_oneshot); /* just in case */
    _removeCommandReply_string(clientID,simx_cmd_get_and_clear_string_signal,(simxUChar*)retSignalName); /* just in case */

    simxGetAndClearStringSignal(clientID,retSignalName,retSignalValue,retSignalLength,simx_opmode_streaming);
    err=simxSetStringSignal(clientID,signalName,signalValue,signalLength,simx_opmode_blocking);
    if (err!=0)
        return(err);
    while (extApi_getTimeDiffInMs(startTime)<timeOutInMs)
    {
        if (simxGetAndClearStringSignal(clientID,retSignalName,retSignalValue,retSignalLength,simx_opmode_buffer)==0)
            return(0); /* success */
        extApi_switchThread();
    }
    return(simx_return_timeout_flag);
}

EXTAPI_DLLEXPORT simxInt simxGetObjectGroupData(simxInt clientID,simxInt objectType,simxInt dataType,simxInt* handlesCount,simxInt** handles,simxInt* intDataCount,simxInt** intData,simxInt* floatDataCount,simxFloat** floatData,simxInt* stringDataCount,simxChar** stringData,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue,additionalOffset,intDataCount_,floatDataCount_,stringDataCount_;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_intint(clientID,simx_cmd_get_object_group_data,objectType,dataType));
    dataPointer=_exec_intint(clientID,simx_cmd_get_object_group_data,operationMode,0,objectType,dataType,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
    {
        handlesCount[0]=_readPureDataInt(dataPointer,0,0);
        intDataCount_=_readPureDataInt(dataPointer,0,4);
        floatDataCount_=_readPureDataInt(dataPointer,0,8);
        stringDataCount_=_readPureDataInt(dataPointer,0,12);
        additionalOffset=16;

        if (intDataCount!=NULL)
            intDataCount[0]=intDataCount_;
        if (floatDataCount!=NULL)
            floatDataCount[0]=floatDataCount_;
        if (stringDataCount!=NULL)
            stringDataCount[0]=stringDataCount_;

        handles[0]=((simxInt*)(dataPointer+SIMX_SUBHEADER_SIZE+_getCmdDataSize(dataPointer)+additionalOffset)); /* little/big endian conversion happened on the server side */
        additionalOffset+=handlesCount[0]*4;

        if (intData!=NULL)
            intData[0]=((simxInt*)(dataPointer+SIMX_SUBHEADER_SIZE+_getCmdDataSize(dataPointer)+additionalOffset)); /* little/big endian conversion happened on the server side */
        additionalOffset+=intDataCount_*4;

        if (floatData!=NULL)
            floatData[0]=((simxFloat*)(dataPointer+SIMX_SUBHEADER_SIZE+_getCmdDataSize(dataPointer)+additionalOffset)); /* little/big endian conversion happened on the server side */
        additionalOffset+=floatDataCount_*4;

        if (stringData!=NULL)
            stringData[0]=(simxChar*)dataPointer+SIMX_SUBHEADER_SIZE+_getCmdDataSize(dataPointer)+additionalOffset;
    }
    return(returnValue);
}

EXTAPI_DLLEXPORT simxInt simxGetObjectVelocity(simxInt clientID,simxInt objectHandle,simxFloat* linearVelocity,simxFloat* angularVelocity,simxInt operationMode)
{
    simxUChar* dataPointer;
    simxInt returnValue;
    if (_communicationThreadRunning[clientID]==0)
        return(simx_return_initialize_error_flag);
    if (operationMode==simx_opmode_remove)
        return(_removeCommandReply_int(clientID,simx_cmd_get_object_velocity,objectHandle));
    dataPointer=_exec_int(clientID,simx_cmd_get_object_velocity,operationMode,0,objectHandle,&returnValue);
    if ((dataPointer!=0)&&(returnValue==0))
    {
        if (linearVelocity!=0)
        {
            linearVelocity[0]=_readPureDataFloat(dataPointer,0,0);
            linearVelocity[1]=_readPureDataFloat(dataPointer,0,4);
            linearVelocity[2]=_readPureDataFloat(dataPointer,0,8);
        }
        if (angularVelocity!=0)
        {
            angularVelocity[0]=_readPureDataFloat(dataPointer,0,12);
            angularVelocity[1]=_readPureDataFloat(dataPointer,0,16);
            angularVelocity[2]=_readPureDataFloat(dataPointer,0,20);
        }
    }
    return(returnValue);
}


EXTAPI_DLLEXPORT simxInt mtlb_simxSetJointPosition(simxInt clientID,simxInt jointHandle,simxFloat* position,simxInt operationMode)
{
    return(simxSetJointPosition(clientID,jointHandle,position[0],operationMode));
}

EXTAPI_DLLEXPORT simxInt mtlb_simxSetJointTargetVelocity(simxInt clientID,simxInt jointHandle,simxFloat* targetVelocity,simxInt operationMode)
{
    return(simxSetJointTargetVelocity(clientID,jointHandle,targetVelocity[0],operationMode));
}

EXTAPI_DLLEXPORT simxInt mtlb_simxSetJointTargetPosition(simxInt clientID,simxInt jointHandle,simxFloat* targetPosition,simxInt operationMode)
{
    return(simxSetJointTargetPosition(clientID,jointHandle,targetPosition[0],operationMode));
}

EXTAPI_DLLEXPORT simxInt mtlb_simxSetJointForce(simxInt clientID,simxInt jointHandle,simxFloat* force,simxInt operationMode)
{
    return(simxSetJointForce(clientID,jointHandle,force[0],operationMode));
}

EXTAPI_DLLEXPORT simxInt mtlb_simxSetFloatSignal(simxInt clientID,const simxChar* signalName,simxFloat* signalValue,simxInt operationMode)
{
    return(simxSetFloatSignal(clientID,signalName,signalValue[0],operationMode));
}

EXTAPI_DLLEXPORT simxInt mtlb_simxSetObjectFloatParameter(simxInt clientID,simxInt objectHandle,simxInt parameterID,simxFloat* parameterValue,simxInt operationMode)
{
    return(simxSetObjectFloatParameter(clientID,objectHandle,parameterID,parameterValue[0],operationMode));
}

EXTAPI_DLLEXPORT simxInt mtlb_simxSetFloatingParameter(simxInt clientID,simxInt paramIdentifier,simxFloat* paramValue,simxInt operationMode)
{
    return(simxSetFloatingParameter(clientID,paramIdentifier,paramValue[0],operationMode));
}

EXTAPI_DLLEXPORT simxInt mtlb_simxCreateDummy(simxInt clientID,simxFloat* size,const simxUChar* colors,simxInt* objectHandle,simxInt operationMode)
{
    return(simxCreateDummy(clientID,size[0],colors,objectHandle,operationMode));
}


EXTAPI_DLLEXPORT simxInt mtlb_simxReadProximitySensor(simxInt* clientIDandSensorHandle,simxUChar* detectionState,simxFloat* detectedPoint,simxInt* detectedObjectHandle,simxFloat* detectedSurfaceNormalVector,simxInt operationMode)
{
    int ret;
    ret=simxReadProximitySensor(clientIDandSensorHandle[0],clientIDandSensorHandle[1],detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector,operationMode);
    return(ret);
}

EXTAPI_DLLEXPORT simxInt mtlb_simxAuxiliaryConsoleOpen(simxInt* clientIDandMaxLinesAndModeAndPositionAndSize,const simxChar* title,simxFloat* textColor,simxFloat* backgroundColor,simxInt* consoleHandle,simxInt operationMode)
{
    int ret;
    int p_[2];
    int s_[2];
    int* p;
    int* s;
    p_[0]=clientIDandMaxLinesAndModeAndPositionAndSize[3];
    p_[1]=clientIDandMaxLinesAndModeAndPositionAndSize[4];
    s_[0]=clientIDandMaxLinesAndModeAndPositionAndSize[5];
    s_[1]=clientIDandMaxLinesAndModeAndPositionAndSize[6];
    p=p_;
    s=s_;
    if (p_[0]<-9999)
        p=NULL;
    if (s_[0]<-9999)
        s=NULL;
    ret=simxAuxiliaryConsoleOpen(clientIDandMaxLinesAndModeAndPositionAndSize[0],title,clientIDandMaxLinesAndModeAndPositionAndSize[1],clientIDandMaxLinesAndModeAndPositionAndSize[2],p,s,textColor,backgroundColor,consoleHandle,operationMode);
    return(ret);
}

EXTAPI_DLLEXPORT simxInt mtlb_simxDisplayDialog(simxInt* clientIDandDlgTypeAndOperationMode,const simxChar* titleText,const simxChar* mainText,const simxChar* initialText,const simxFloat* titleColorsAndDlgColors,simxInt* dialogHandleAndUiHandle)
{
    int ret,i;
    float titleColors_[6];
    float dialogColors_[6];
    float* titleColors;
    float* dialogColors;
    for (i=0;i<6;i++)
    {
        titleColors_[i]=titleColorsAndDlgColors[i];
        dialogColors_[i]=titleColorsAndDlgColors[6+i];
    }
    titleColors=titleColors_;
    dialogColors=dialogColors_;
    if (titleColors_[0]<-9999.0f)
        titleColors=NULL;
    if (dialogColors_[0]<-9999.0f)
        dialogColors=NULL;
    ret=simxDisplayDialog(clientIDandDlgTypeAndOperationMode[0],titleText,mainText,clientIDandDlgTypeAndOperationMode[1],initialText,titleColors,dialogColors,dialogHandleAndUiHandle,dialogHandleAndUiHandle+1,clientIDandDlgTypeAndOperationMode[2]);
    return(ret);
}

EXTAPI_DLLEXPORT simxInt mtlb_simxQuery(simxInt* clientIDandSignalLengthAndTimeOutInMs,const simxChar* signalName,const simxUChar* signalValue,const simxChar* retSignalName,simxUChar** retSignalValue,simxInt* retSignalLength)
{
    int ret;
    ret=simxQuery(clientIDandSignalLengthAndTimeOutInMs[0],signalName,signalValue,clientIDandSignalLengthAndTimeOutInMs[1],retSignalName,retSignalValue,retSignalLength,clientIDandSignalLengthAndTimeOutInMs[2]);
    return(ret);
}

EXTAPI_DLLEXPORT simxInt mtlb_simxGetObjectGroupData(simxInt* clientIDandObjectTypeAndDataTypeAndOperationMode,simxInt* handlesCountAndIntDataCountAndFloatDataCountAndStringDataCount,simxInt** handles,simxInt** intData,simxFloat** floatData,simxChar** stringData)
{
    int ret;
    ret=simxGetObjectGroupData(clientIDandObjectTypeAndDataTypeAndOperationMode[0],clientIDandObjectTypeAndDataTypeAndOperationMode[1],clientIDandObjectTypeAndDataTypeAndOperationMode[2],handlesCountAndIntDataCountAndFloatDataCountAndStringDataCount+0,handles,handlesCountAndIntDataCountAndFloatDataCountAndStringDataCount+1,intData,handlesCountAndIntDataCountAndFloatDataCountAndStringDataCount+2,floatData,handlesCountAndIntDataCountAndFloatDataCountAndStringDataCount+3,stringData,clientIDandObjectTypeAndDataTypeAndOperationMode[3]);
    return(ret);
}

EXTAPI_DLLEXPORT simxInt mtlb_simxCallScriptFunction_a(const simxInt* variousIntsIn,const simxChar* scriptDescriptionAndFunctionName,const simxInt* inInt,const simxFloat* inFloat,const simxChar* inString,const simxUChar* inBuffer)
{
    int clientID;
    int i,off,tmp;
    int cnt=7*4; /* clientID, options, inIntCnt, inFloatCnt, inStringCnt, inBufferSize, opMode */
    clientID=variousIntsIn[0];
    off=extApi_getStringLength(scriptDescriptionAndFunctionName)+1;
    cnt+=off;
    off=extApi_getStringLength(scriptDescriptionAndFunctionName+off)+1;
    cnt+=off;
    cnt+=variousIntsIn[2]*4;
    cnt+=variousIntsIn[3]*4;

    off=0;
    for (i=0;i<variousIntsIn[4];i++)
        off+=extApi_getStringLength(inString+off)+1;
    cnt+=off;
    cnt+=variousIntsIn[5];

    _tmpBuffer[variousIntsIn[0]]=extApi_allocateBuffer(cnt);
    
    ((simxInt*)_tmpBuffer[clientID])[0]=variousIntsIn[0];
    ((simxInt*)_tmpBuffer[clientID])[1]=variousIntsIn[1];
    ((simxInt*)_tmpBuffer[clientID])[2]=variousIntsIn[2];
    ((simxInt*)_tmpBuffer[clientID])[3]=variousIntsIn[3];
    ((simxInt*)_tmpBuffer[clientID])[4]=variousIntsIn[4];
    ((simxInt*)_tmpBuffer[clientID])[5]=variousIntsIn[5];
    ((simxInt*)_tmpBuffer[clientID])[6]=variousIntsIn[6];

    off=7*4;
    tmp=extApi_getStringLength(scriptDescriptionAndFunctionName)+1;
    tmp+=extApi_getStringLength(scriptDescriptionAndFunctionName+tmp)+1;
    for (i=0;i<tmp;i++)
        _tmpBuffer[clientID][off+i]=scriptDescriptionAndFunctionName[i];
    off+=tmp;

    for (i=0;i<variousIntsIn[2];i++)
        ((simxInt*)(_tmpBuffer[clientID]+off))[i]=inInt[i];
    off+=variousIntsIn[2]*4;

    for (i=0;i<variousIntsIn[3];i++)
        ((simxFloat*)(_tmpBuffer[clientID]+off))[i]=inFloat[i];
    off+=variousIntsIn[3]*4;

    /* put buffer before strings, easier after */
    for (i=0;i<variousIntsIn[5];i++)
        _tmpBuffer[clientID][off+i]=inBuffer[i];
    off+=variousIntsIn[5];

    tmp=0;
    for (i=0;i<variousIntsIn[4];i++)
        tmp+=extApi_getStringLength(inString+tmp)+1;
    for (i=0;i<tmp;i++)
        _tmpBuffer[clientID][off+i]=inString[i];
    off+=tmp;
    return(0);
}

EXTAPI_DLLEXPORT simxInt mtlb_simxCallScriptFunction_b(simxInt clientID,simxInt* variousIntsOut,simxInt** outInt,simxFloat** outFloat,simxChar** outString,simxUChar** outBuffer)
{

    simxInt ret,off;
    simxInt inIntCnt,inFloatCnt,inStringCnt,inBufferSize,options,opMode;
    simxChar* scriptDescription;
    simxChar* funcName;
    simxChar* inStrings;
    simxInt* inInts;
    simxFloat* inFloats;
    simxUChar* inBuffer;

    options=((simxInt*)_tmpBuffer[clientID])[1];
    inIntCnt=((simxInt*)_tmpBuffer[clientID])[2];
    inFloatCnt=((simxInt*)_tmpBuffer[clientID])[3];
    inStringCnt=((simxInt*)_tmpBuffer[clientID])[4];
    inBufferSize=((simxInt*)_tmpBuffer[clientID])[5];
    opMode=((simxInt*)_tmpBuffer[clientID])[6];
    off=7*4;

    scriptDescription=(simxChar*)(_tmpBuffer[clientID]+off);
    off+=extApi_getStringLength(scriptDescription)+1;

    funcName=(simxChar*)(_tmpBuffer[clientID]+off);
    off+=extApi_getStringLength(funcName)+1;
    
    inInts=(simxInt*)(_tmpBuffer[clientID]+off);
    off+=inIntCnt*4;
    
    inFloats=(simxFloat*)(_tmpBuffer[clientID]+off);
    off+=inFloatCnt*4;

    inBuffer=(simxUChar*)(_tmpBuffer[clientID]+off);
    off+=inBufferSize;

    inStrings=(simxChar*)(_tmpBuffer[clientID]+off);


    /* variousIntsOut: [0]:outIntC, [1]:outFloatC, [2]:outStringC, [3]:outBufferS */

    ret=simxCallScriptFunction(clientID,scriptDescription,options,funcName,inIntCnt,inInts,inFloatCnt,inFloats,inStringCnt,inStrings,inBufferSize,inBuffer,&variousIntsOut[0],outInt,&variousIntsOut[1],outFloat,&variousIntsOut[2],outString,&variousIntsOut[3],outBuffer,opMode);
    extApi_releaseBuffer(_tmpBuffer[clientID]);

    return(ret);
}





#ifdef _Included_extApiJava

/*
void substr(char *s, char *d, int start, int len)
{
    int i=start;
    while (i < (start+len))
    {
        d[i-start]=s[i];
        i++;
    }
    d[i-start]='\0';
}
*/

extern "C" {

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxStart(JNIEnv * env, jobject obj, jstring connAddr, jint connPort, jboolean waitUntilConnected, jboolean doNotRecon, jint toInMs, jint commThreadCycleInMs)
{
    const char *connectionAddress = env->GetStringUTFChars(connAddr, 0);
    simxInt connectionPort = connPort;
    simxUChar waitUntilConnected_ = waitUntilConnected ? 1 : 0;
    simxUChar doNotReconnectOnceDisconnected = doNotRecon ? 1 : 0;
    simxInt timeOutInMs = toInMs;
    simxInt commThreadCycleInMs_ = commThreadCycleInMs;

    simxInt retVal = simxStart(connectionAddress, connectionPort, waitUntilConnected_, doNotReconnectOnceDisconnected,timeOutInMs,commThreadCycleInMs_);

    env->ReleaseStringUTFChars( connAddr, connectionAddress);

    return retVal;
}

JNIEXPORT void JNICALL Java_coppelia_remoteApi_simxFinish  (JNIEnv *env, jobject obj, jint clientID)
{
    simxInt theClientID = clientID;
    simxFinish(theClientID);
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetConnectionId(JNIEnv *env, jobject obj, jint clientID)
{
    simxInt theClientID = clientID;
    return simxGetConnectionId(theClientID);
}


JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetObjectHandle(JNIEnv *env, jobject obj, jint clientID, jstring objName, jobject hdl, jint opMode)
{
    simxInt theClientID = clientID;
    const char *objectName = env->GetStringUTFChars(objName, 0);
    simxInt objectHandle;
    simxInt operationMode = opMode;
    simxInt retVal;
    retVal = simxGetObjectHandle(theClientID,objectName, &objectHandle, operationMode);

    jclass cls = env->GetObjectClass(hdl);
    jmethodID mid = env->GetMethodID(cls, "setValue", "(I)V");
    env->CallVoidMethod(hdl, mid, objectHandle);

    env->ReleaseStringUTFChars(objName, objectName);
    
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetJointPosition(JNIEnv *env, jobject obj, jint clientID, jint hdl, jobject pos, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt jointHandle = hdl;
    simxFloat position;
    simxInt operationMode = opMode;

    simxInt retVal = simxGetJointPosition(theClientID,jointHandle, &position, operationMode);

      jclass cls = env->GetObjectClass(pos);
      jmethodID mid = env->GetMethodID(cls, "setValue", "(F)V");
    env->CallVoidMethod(pos, mid, position);

    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxSetJointPosition(JNIEnv *env, jobject obj, jint clientID, jint hdl, jfloat pos, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt jointHandle = hdl;
    simxFloat position = pos;
    simxInt operationMode = opMode;
    simxInt retVal = simxSetJointPosition(theClientID,jointHandle, position, operationMode);
    
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetObjectPosition(JNIEnv *env, jobject obj, jint clientID, jint hdl, jint rel, jobject pos, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt objectHandle = hdl;
    simxInt relativeToObjectHandle = rel;
    simxFloat position[3];
    simxInt operationMode = opMode;

    simxInt retVal = simxGetObjectPosition(theClientID,objectHandle, relativeToObjectHandle, position, operationMode);

    jsize start = 0;
    jsize size = 3;
    jclass cls = env->GetObjectClass(pos);
    jmethodID mid = env->GetMethodID(cls, "getNewArray", "(I)[F");
    jfloatArray posArray = (jfloatArray)env->CallObjectMethod(pos, mid, size);
    env->SetFloatArrayRegion(posArray, start, size, (const jfloat *)position);

    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxSetObjectPosition(JNIEnv *env, jobject obj, jint clientID, jint hdl, jint rel, jobject pos, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt objectHandle = hdl;
    simxInt relativeToObjectHandle = rel;
    simxFloat position[3];

    jclass cls = env->GetObjectClass(pos);
    jmethodID mid = env->GetMethodID(cls, "getArray", "()[F");
    jfloatArray posArray = (jfloatArray)env->CallObjectMethod(pos, mid);
    env->GetFloatArrayRegion(posArray, 0, 3, (jfloat *)position);

    simxInt operationMode = opMode;

    simxInt retVal = simxSetObjectPosition(theClientID,objectHandle, relativeToObjectHandle, position, operationMode);

    return retVal;
}


JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetVisionSensorImage(JNIEnv *env, jobject obj, jint clientID, jint hdl, jobject res, jobject img, jint opts, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt sensorHandle = hdl;

    simxUChar* image;
    simxUChar options = (simxUChar)opts;
    simxInt operationMode = opMode;
    simxInt resolution[2];

    simxInt retVal = simxGetVisionSensorImage(theClientID,sensorHandle, resolution, &image, options, operationMode);

    if (retVal==0)
    {
        jclass cls = env->GetObjectClass(res);
        jmethodID mid = env->GetMethodID(cls, "getNewArray", "(I)[I");
        jintArray resArray = (jintArray)env->CallObjectMethod(res, mid, 2);
        env->SetIntArrayRegion(resArray, 0, 2, (jint *)resolution);

        jsize size = (options&1) ? (resolution[0]*resolution[1]) : (resolution[0]*resolution[1]*3);
        cls = env->GetObjectClass(img);
        mid = env->GetMethodID(cls, "getNewArray", "(I)[C");
        jcharArray imgArray = (jcharArray)env->CallObjectMethod(img, mid, size);
        jchar* arr=env->GetCharArrayElements(imgArray,0);
        for (jsize i=0;i<size;i++)
            arr[i]=(unsigned char)image[i];
        env->ReleaseCharArrayElements(imgArray,arr, 0);         
    }

    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxSetVisionSensorImage(JNIEnv *env, jobject obj, jint clientID, jint hdl, jobject img, jint bufsize, jint opts, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt sensorHandle = hdl;
    simxInt bufferSize = bufsize;
    simxUChar options = (simxUChar)opts;
    simxInt operationMode = opMode;

    char* imgbuf=new char[bufferSize];
    jsize size = bufsize;   
    jclass cls = env->GetObjectClass(img);
    jmethodID mid = env->GetMethodID(cls, "getArray", "()[C");
    jcharArray imgArray = (jcharArray)env->CallObjectMethod(img, mid);
    jchar* arr=env->GetCharArrayElements(imgArray,0);
    for (jsize i=0;i<size;i++)
        imgbuf[i]=(arr[i]&255);
    env->ReleaseCharArrayElements(imgArray,arr, 0);         

    simxInt retVal = simxSetVisionSensorImage(theClientID,sensorHandle,  (simxUChar*)imgbuf, bufferSize, options, operationMode);
    delete[] imgbuf; 

    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetJointMatrix(JNIEnv *env, jobject obj, jint clientID, jint hdl, jobject mtx, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt jointHandle = hdl;
    simxFloat matrix[12];
    simxInt operationMode = opMode;

    simxInt retVal = simxGetJointMatrix(theClientID,jointHandle, matrix, operationMode);
    
    jsize start = 0;
    jsize size = 12;
    jclass cls = env->GetObjectClass(mtx);
    jmethodID mid = env->GetMethodID(cls, "getNewArray", "(I)[F");
    jfloatArray mtxArray = (jfloatArray)env->CallObjectMethod(mtx, mid, size);
    env->SetFloatArrayRegion(mtxArray, start, size, (const jfloat *)matrix);

    return retVal;
}


JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxSetSphericalJointMatrix(JNIEnv *env, jobject obj, jint clientID, jint hdl, jobject mtx, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt jointHandle = hdl;
    simxInt operationMode = opMode;
    simxFloat matrix[12];

    jsize size = 12;
    jclass cls = env->GetObjectClass(mtx);
    jmethodID mid = env->GetMethodID(cls, "getArray", "()[F");
    jfloatArray mtxArray = (jfloatArray)env->CallObjectMethod(mtx, mid);
    env->GetFloatArrayRegion(mtxArray, 0, size, (jfloat *)matrix);
    
    simxInt retVal = simxSetSphericalJointMatrix(theClientID,jointHandle, matrix, operationMode);

    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxSetJointTargetVelocity(JNIEnv *env, jobject obj, jint clientID, jint hdl, jfloat tv, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt jointHandle = hdl;
    simxFloat targetVelocity = tv;
    simxInt operationMode = opMode;

    simxInt retVal = simxSetJointTargetVelocity(theClientID,jointHandle, targetVelocity, operationMode);
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxSetJointTargetPosition(JNIEnv *env, jobject obj, jint clientID, jint hdl, jfloat tp, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt jointHandle = hdl;
    simxFloat targetPosition = tp;
    simxInt operationMode = opMode;

    simxInt retVal = simxSetJointTargetPosition(theClientID,jointHandle, targetPosition, operationMode);
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxJointGetForce(JNIEnv *env, jobject obj, jint clientID, jint hdl, jobject frc, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt jointHandle = hdl;
    simxFloat   force;
    simxInt operationMode = opMode; 

    simxInt retVal = simxGetJointForce(theClientID,jointHandle, &force, operationMode);

    jclass cls = env->GetObjectClass(frc);
    jmethodID mid = env->GetMethodID(cls, "setValue", "(F)V");
    env->CallVoidMethod(frc, mid, force);

    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetJointForce(JNIEnv *env, jobject obj, jint clientID, jint hdl, jobject frc, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt jointHandle = hdl;
    simxFloat   force;
    simxInt operationMode = opMode; 

    simxInt retVal = simxGetJointForce(theClientID,jointHandle, &force, operationMode);

    jclass cls = env->GetObjectClass(frc);
    jmethodID mid = env->GetMethodID(cls, "setValue", "(F)V");
    env->CallVoidMethod(frc, mid, force);

    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxSetJointForce(JNIEnv *env, jobject obj, jint clientID, jint hdl, jfloat frc, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt jointHandle = hdl;
    simxFloat force = frc;
    simxInt operationMode = opMode;

    simxInt retVal = simxSetJointForce(theClientID,jointHandle, force, operationMode);

    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxReadForceSensor(JNIEnv *env, jobject obj, jint clientID, jint hdl, jobject st, jobject fv, jobject tv, jint opMode) 
{
    simxInt theClientID = clientID;
    simxInt forceSensorHandle = hdl;
    simxInt operationMode = opMode;
    simxUChar state;        
    simxFloat forceVector[3];
    simxFloat torqueVector[3];
        
    simxInt retVal = simxReadForceSensor(theClientID,forceSensorHandle, (st==NULL) ? 0 : &state, (fv==NULL) ? 0 : forceVector, (tv==NULL) ? 0 : torqueVector, operationMode);

    if (st != NULL)
    {
        jclass cls = env->GetObjectClass(st);
        jmethodID mid = env->GetMethodID(cls, "setValue", "(I)V");
        env->CallVoidMethod(st, mid, state);    
    }

    jsize start = 0;
    jsize size = 3;

    if (fv != NULL)
    {
        jclass cls = env->GetObjectClass(fv);
        jmethodID mid = env->GetMethodID(cls, "getNewArray", "(I)[F");
        jfloatArray fvArray = (jfloatArray)env->CallObjectMethod(fv, mid, size);
        env->SetFloatArrayRegion(fvArray, start, size, (const jfloat *)forceVector);
    }

    if (tv != NULL)
    {       
        jclass cls = env->GetObjectClass(tv);
        jmethodID mid = env->GetMethodID(cls, "getNewArray", "(I)[F");
        jfloatArray tvArray = (jfloatArray)env->CallObjectMethod(tv, mid, size);
        env->SetFloatArrayRegion(tvArray, start, size, (const jfloat *)torqueVector);
    }

    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxBreakForceSensor(JNIEnv *env, jobject obj, jint clientID, jint hdl, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt forceSensorHandle = hdl;
    simxInt operationMode = opMode;
    
    simxInt retVal = simxBreakForceSensor(theClientID,forceSensorHandle, operationMode);
    return retVal;
}


JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxReadVisionSensor(JNIEnv *env, jobject obj, jint clientID, jint hdl, jobject ds, jobject av, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt sensorHandle = hdl;
    simxInt operationMode = opMode;
    simxUChar detectionState;
    simxFloat* auxValues;
    simxInt* auxValuesCount;

    simxInt retVal = simxReadVisionSensor(theClientID,sensorHandle, (ds == NULL) ? 0 : &detectionState, (av==NULL) ? 0 : &auxValues, (av==NULL) ? 0 : &auxValuesCount, operationMode);

    if (retVal==0)
    {
        if (ds != NULL)
        {
            jclass cls = env->GetObjectClass(ds);
            jmethodID mid = env->GetMethodID(cls, "setValue", "(Z)V");
            env->CallVoidMethod(ds, mid, (detectionState==1));
        }
        
        if (av != NULL && retVal==0)
        {
            jsize size = auxValuesCount[0];
            jclass cls = env->GetObjectClass(av);
            jmethodID mid = env->GetMethodID(cls, "getNewArray", "(I)[Lcoppelia/FloatWA;"); // in release 3.0.4 "coppelia/" was forgotten. Thanks to Billy Newman for reporting the bug
            jobjectArray avArray = (jobjectArray)env->CallObjectMethod(av, mid, size);
            int sizeTotal = 0;
            for (int i=0;i<size;i++)
            {

                jsize size1 = auxValuesCount[i+1];
                jobject o = env->GetObjectArrayElement(avArray, i);

                cls = env->FindClass("coppelia/FloatWA"); // in release 3.0.4 "coppelia/" was forgotten. Thanks to Billy Newman for reporting the bug
                mid = env->GetMethodID(cls, "getNewArray", "(I)[F");        

                jfloatArray fArray = (jfloatArray)env->CallObjectMethod(o, mid, size1);
                jfloat* f1 = env->GetFloatArrayElements(fArray, 0);
                for (int k=0; k<size1; k++) 
                {
                    f1[k] = auxValues[sizeTotal + k];
                }
                env->ReleaseFloatArrayElements(fArray, f1, 0);  
                sizeTotal += size1;
            }
            simxReleaseBuffer((simxUChar*)auxValues);
            simxReleaseBuffer((simxUChar*)auxValuesCount);
        }
    }

    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetObjectFloatParameter(JNIEnv *env, jobject obj, jint clientID, jint hdl, jint pi, jobject pv, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt objectHandle = hdl;
    simxInt parameterID = pi;
    simxInt operationMode = opMode;
    simxFloat parameterValue;

    simxInt retVal = simxGetObjectFloatParameter(theClientID, objectHandle, parameterID, &parameterValue, operationMode);

    jclass cls = env->GetObjectClass(pv);
      jmethodID mid = env->GetMethodID(cls, "setValue", "(F)V");
    env->CallVoidMethod(pv, mid, parameterValue);

    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxSetObjectFloatParameter(JNIEnv *env, jobject obj, jint clientID, jint hdl, jint pid, jfloat pv, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt objectHandle = hdl;
    simxInt parameterID = pid;
    simxFloat parameterValue = pv;
    simxInt operationMode = opMode;

    simxInt retVal = simxSetObjectFloatParameter(theClientID,objectHandle, parameterID, parameterValue, operationMode);
    return retVal;
}


JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetObjectIntParameter(JNIEnv *env, jobject obj, jint clientID, jint hdl, jint pid, jobject pv, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt objectHandle = hdl;
    simxInt parameterID = pid;
    simxInt operationMode = opMode;
    simxInt parameterValue;

    simxInt retVal = simxGetObjectIntParameter(theClientID,objectHandle, parameterID, &parameterValue, operationMode);

    jclass cls = env->GetObjectClass(pv);
      jmethodID mid = env->GetMethodID(cls, "setValue", "(I)V");
    env->CallVoidMethod(pv, mid, parameterValue);

    return retVal;
}


JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxSetObjectIntParameter(JNIEnv *env, jobject obj, jint clientID, jint hdl, jint pid, jint pv, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt objectHandle = hdl;
    simxInt parameterID = pid;
    simxInt parameterValue = pv;
    simxInt operationMode = opMode;

    simxInt retVal = simxSetObjectIntParameter(theClientID,objectHandle, parameterID, parameterValue, operationMode);
    return retVal;
}


JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetModelProperty(JNIEnv *env, jobject obj, jint clientID, jint hdl, jobject prp, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt objectHandle = hdl;
    simxInt operationMode = opMode;
    simxInt prop;

    simxInt retVal = simxGetModelProperty(theClientID,objectHandle, &prop, operationMode);

    jclass cls = env->GetObjectClass(prp);
    jmethodID mid = env->GetMethodID(cls, "setValue", "(I)V");
    env->CallVoidMethod(prp, mid, prop);

    return retVal;
}


JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxSetModelProperty(JNIEnv *env, jobject obj, jint clientID, jint hdl, jint prp, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt objectHandle = hdl;
    simxInt prop = prp;
    simxInt operationMode = opMode;

    simxInt retVal = simxSetModelProperty(theClientID,objectHandle, prop, operationMode);
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetVisionSensorDepthBuffer(JNIEnv *env, jobject obj, jint clientID, jint hdl, jobject res, jobject buf, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt sensorHandle = hdl;
    simxInt operationMode = opMode;
    simxInt resolution[2];
    simxFloat* buffer;

    simxInt retVal = simxGetVisionSensorDepthBuffer(theClientID,sensorHandle, resolution, &buffer, operationMode);

    if (retVal==0)
    {
        jsize start = 0;
        jsize size = resolution[0]*resolution[1];
        jclass cls = env->GetObjectClass(buf);
        jmethodID mid = env->GetMethodID(cls, "getNewArray", "(I)[F");
        jfloatArray bufArray = (jfloatArray)env->CallObjectMethod(buf, mid, size);  
        env->SetFloatArrayRegion(bufArray, start, size, (const jfloat*)buffer);

        size = 2;
        cls = env->GetObjectClass(res);
        mid = env->GetMethodID(cls, "getNewArray", "(I)[I");
        jintArray resArray = (jintArray)env->CallObjectMethod(res, mid, size);  
        env->SetIntArrayRegion(resArray, start, size, (const jint*)resolution);
    }

    return retVal;  
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetObjectChild(JNIEnv *env, jobject obj, jint clientID, jint hdl, jint ci, jobject coh, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt parentObjectHandle = hdl;
    simxInt childIndex = ci;
    simxInt childObjectHandle;
    simxInt operationMode = opMode;

    simxInt retVal = simxGetObjectChild(theClientID,parentObjectHandle, childIndex, &childObjectHandle, operationMode);

    jclass cls = env->GetObjectClass(coh);
      jmethodID mid = env->GetMethodID(cls, "setValue", "(I)V");
    env->CallVoidMethod(coh, mid, childObjectHandle);

    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetObjectParent(JNIEnv *env, jobject obj, jint clientID, jint coh, jobject poh, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt childObjectHandle = coh;
    simxInt parentObjectHandle;
    simxInt operationMode = opMode;

    simxInt retVal = simxGetObjectParent(theClientID, childObjectHandle, &parentObjectHandle, operationMode);

    jclass cls = env->GetObjectClass(poh);
    jmethodID mid = env->GetMethodID(cls, "setValue", "(I)V");
    env->CallVoidMethod(poh, mid, parentObjectHandle);

    return retVal;
}


JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxStartSimulation(JNIEnv *env, jobject obj, jint clientID, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt operationMode = opMode;

    simxInt retVal = simxStartSimulation(theClientID,operationMode);
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxPauseSimulation(JNIEnv *env, jobject obj, jint clientID, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt operationMode = opMode;

    simxInt retVal = simxPauseSimulation(theClientID,operationMode);
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxStopSimulation(JNIEnv *env, jobject obj, jint clientID, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt operationMode = opMode; 
    
    simxInt retVal = simxStopSimulation(theClientID,operationMode);
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetUISlider(JNIEnv *env, jobject obj, jint clientID, jint hdl, jint ubid, jobject pos, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt uiHandle = hdl;
    simxInt uiButtonID = ubid;
    simxInt operationMode = opMode; 
    simxInt position;

    simxInt retVal = simxGetUISlider(theClientID,uiHandle, uiButtonID, &position, operationMode);

    jclass cls = env->GetObjectClass(pos);
    jmethodID mid = env->GetMethodID(cls, "setValue", "(I)V");
    env->CallVoidMethod(pos, mid, position);

    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxSetUISlider(JNIEnv *env, jobject obj, jint clientID, jint hdl, jint ubid, jint pos, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt uiHandle = hdl;
    simxInt uiButtonID = ubid;
    simxInt position = pos;
    simxInt operationMode = opMode; 
    
    simxInt retVal = simxSetUISlider(theClientID,uiHandle, uiButtonID, position, operationMode);
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetUIEventButton(JNIEnv *env, jobject obj, jint clientID, jint hdl, jobject uieb, jobject auxv, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt uiHandle = hdl;
    simxInt uiEventButtonID;
    simxInt auxValues[2];
    simxInt operationMode = opMode; 

    simxInt retVal = simxGetUIEventButton(theClientID,uiHandle, &uiEventButtonID, auxValues, operationMode);

    jclass cls = env->GetObjectClass(uieb);
    jmethodID mid = env->GetMethodID(cls, "setValue", "(I)V");
    env->CallVoidMethod(uieb, mid, uiEventButtonID);

    jsize start = 0;
    jsize size = 2;
    cls = env->GetObjectClass(auxv);
    mid = env->GetMethodID(cls, "getNewArray", "(I)[I");
    jintArray auxvArray = (jintArray)env->CallObjectMethod(auxv, mid, size);
    env->SetIntArrayRegion(auxvArray, start, size, (const jint*)auxValues);

    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetUIButtonProperty(JNIEnv *env, jobject obj, jint clientID, jint hdl, jint uib, jobject prp, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt uiHandle = hdl;
    simxInt uiButtonID = uib;
    simxInt prop;
    simxInt operationMode = opMode; 

    simxInt retVal = simxGetUIButtonProperty(theClientID,uiHandle, uiButtonID, &prop, operationMode);

    jclass cls = env->GetObjectClass(prp);
    jmethodID mid = env->GetMethodID(cls, "setValue", "(I)V");
    env->CallVoidMethod(prp, mid, prop);

    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxSetUIButtonProperty(JNIEnv *env, jobject obj, jint clientID, jint hdl, jint uib, jint prp, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt uiHandle = hdl;
    simxInt uiButtonID = uib;
    simxInt prop = prp;
    simxInt operationMode = opMode;

    simxInt retVal = simxSetUIButtonProperty(theClientID, uiHandle, uiButtonID, prop, operationMode);

    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxAuxiliaryConsoleClose(JNIEnv *env, jobject obj, jint clientID, jint hdl, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt consoleHandle = hdl;
    simxInt operationMode = opMode;

    simxInt retVal = simxAuxiliaryConsoleClose(theClientID,consoleHandle, operationMode);
    return retVal;
}


JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxAuxiliaryConsoleShow(JNIEnv *env, jobject obj, jint clientID, jint hdl, jboolean shstate, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt consoleHandle = hdl;
    simxUChar showState = shstate ? 1 : 0;
    simxInt operationMode = opMode;

    simxInt retVal = simxAuxiliaryConsoleShow(theClientID, consoleHandle, showState, operationMode);
    return retVal;
}


JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetObjectOrientation(JNIEnv *env, jobject obj, jint clientID, jint hdl, jint rtoHdl, jobject euA, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt objectHandle = hdl;
    simxInt relativeToObjectHandle = rtoHdl;
    simxFloat eulerAngles[3];
    simxInt operationMode = opMode;

    simxInt retVal = simxGetObjectOrientation(theClientID,objectHandle, relativeToObjectHandle, eulerAngles, operationMode);

    jsize start = 0;
    jsize size = 3;
    jclass cls = env->GetObjectClass(euA);
    jmethodID mid = env->GetMethodID(cls, "getNewArray", "(I)[F");
    jfloatArray euArray = (jfloatArray)env->CallObjectMethod(euA, mid, size);
    env->SetFloatArrayRegion(euArray, start, size, eulerAngles);

    return retVal;
}


JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxSetObjectParent(JNIEnv *env, jobject obj, jint clientID, jint hdl, jint po, jboolean kip, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt objectHandle = hdl;
    simxInt parentObject = po;
    simxUChar keepInPlace = kip ? 1 : 0;
    simxInt operationMode = opMode;

    simxInt retVal = simxSetObjectParent(theClientID,objectHandle, parentObject, keepInPlace, operationMode);
    return retVal;
}


JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetArrayParameter(JNIEnv *env, jobject obj, jint clientID, jint pid, jobject pv, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt paramIdentifier = pid;
    simxInt operationMode = opMode;
    simxFloat paramValues[3];

    simxInt retVal = simxGetArrayParameter(theClientID,paramIdentifier, paramValues, operationMode);

    jsize start = 0;
    jsize size = 3;
    jclass cls = env->GetObjectClass(pv);
    jmethodID mid = env->GetMethodID(cls, "getNewArray", "(I)[F");
    jfloatArray pvArray = (jfloatArray)env->CallObjectMethod(pv, mid, size);
    env->SetFloatArrayRegion(pvArray, start, size, paramValues);
    return retVal;
}


JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxSetArrayParameter(JNIEnv *env, jobject obj, jint clientID, jint pid, jobject pv, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt paramIdentifier = pid;
    simxInt operationMode = opMode;

    jclass cls = env->GetObjectClass(pv);
    jmethodID mid = env->GetMethodID(cls, "getArray", "()[F");
    jfloatArray pvArray = (jfloatArray)env->CallObjectMethod(pv, mid);
    mid = env->GetMethodID(cls, "getLength", "()I");
    jint size = env->CallIntMethod(pv, mid);
    
    simxFloat* paramValues = new simxFloat[size];
    env->GetFloatArrayRegion(pvArray, 0, size, (jfloat *)paramValues);

    simxInt retVal = simxSetArrayParameter(theClientID,paramIdentifier, (const simxFloat*) paramValues, operationMode);

    delete[] paramValues;
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetIntegerParameter(JNIEnv *env, jobject obj, jint clientID, jint pi, jobject pv, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt paramIdentifier = pi;
    simxInt operationMode = opMode;
    simxInt paramValue;

    simxInt retVal = simxGetIntegerParameter(theClientID,paramIdentifier, &paramValue, operationMode);

    jclass cls = env->GetObjectClass(pv);
    jmethodID mid = env->GetMethodID(cls, "setValue", "(I)V");
    env->CallVoidMethod(pv, mid, paramValue);

    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxSetIntegerParameter(JNIEnv *env, jobject obj, jint clientID, jint pi, jint pv, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt paramIdentifier = pi;
    simxInt paramValue = pv;
    simxInt operationMode = opMode;

    simxInt retVal = simxSetIntegerParameter(theClientID,paramIdentifier, paramValue, operationMode);
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxSetBooleanParameter(JNIEnv *env, jobject obj, jint clientID, jint pi, jboolean pv, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt paramIdentifier = pi;
    simxUChar paramValue = pv ? 1 : 0;
    simxInt operationMode = opMode;

    simxInt retVal = simxSetBooleanParameter(theClientID,paramIdentifier, paramValue, operationMode);
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetDialogResult(JNIEnv *env, jobject obj, jint clientID, jint hdl, jobject res, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt dialogHandle = hdl;
    simxInt operationMode = opMode;
    simxInt result;

    simxInt retVal = simxGetDialogResult(theClientID,dialogHandle, &result, operationMode);

    jclass cls = env->GetObjectClass(res);
    jmethodID mid = env->GetMethodID(cls, "setValue", "(I)V");
    env->CallVoidMethod(res, mid, result);

    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxSetFloatingParameter(JNIEnv *env, jobject obj, jint clientID, jint pi, jfloat pv, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt paramIdentifier = pi;
    simxFloat paramValue = pv;
    simxInt operationMode = opMode;

    simxInt retVal = simxSetFloatingParameter(theClientID,paramIdentifier, paramValue, operationMode);
    return retVal;
}


JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxRemoveObject(JNIEnv *env, jobject obj, jint clientID, jint hdl, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt objectHandle = hdl;
    simxInt operationMode = opMode;

    simxInt retVal = simxRemoveObject(theClientID,objectHandle, operationMode);
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxRemoveModel(JNIEnv *env, jobject obj, jint clientID, jint hdl, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt objectHandle = hdl;
    simxInt operationMode = opMode;

    simxInt retVal = simxRemoveModel(theClientID,objectHandle, operationMode);
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxRemoveUI(JNIEnv *env, jobject obj, jint clientID, jint hdl, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt uiHandle = hdl;
    simxInt operationMode = opMode;

    simxInt retVal = simxRemoveUI(theClientID,uiHandle, operationMode);
    return retVal;
}


JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxCloseScene(JNIEnv *env, jobject obj, jint clientID, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt operationMode = opMode;

    simxInt retVal = simxCloseScene(theClientID,operationMode);
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxEndDialog(JNIEnv *env, jobject obj, jint clientID, jint hdl, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt dialogHandle = hdl;
    simxInt operationMode = opMode;

    simxInt retVal = simxEndDialog(theClientID,dialogHandle, operationMode);
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxReadDistance(JNIEnv *env, jobject obj, jint clientID, jint hdl, jobject md, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt distanceObjectHandle = hdl;
    simxInt operationMode = opMode;
    simxFloat minimumDistance;

    simxInt retVal = simxReadDistance(theClientID,distanceObjectHandle, &minimumDistance, operationMode);

    jclass cls = env->GetObjectClass(md);
    jmethodID mid = env->GetMethodID(cls, "setValue", "(F)V");
    env->CallVoidMethod(md, mid, minimumDistance);
    
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetFloatingParameter(JNIEnv *env, jobject obj, jint clientID, jint pi, jobject pv, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt paramIdentifier = pi;
    simxInt operationMode = opMode;
    simxFloat paramValue;

    simxInt retVal = simxGetFloatingParameter(theClientID,paramIdentifier, &paramValue, operationMode);

    jclass cls = env->GetObjectClass(pv);
    jmethodID mid = env->GetMethodID(cls, "setValue", "(F)V");
    env->CallVoidMethod(pv, mid, paramValue);

    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxSetObjectOrientation(JNIEnv *env, jobject obj, jint clientID, jint hdl, jint rtoHdl, jobject euAngles, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt objectHandle = hdl;
    simxInt relativeToObjectHandle = rtoHdl;
    simxInt operationMode = opMode;
    simxFloat eulerAngles[3];

    jclass cls = env->GetObjectClass(euAngles);
    jmethodID mid = env->GetMethodID(cls, "getArray", "()[F");
    jfloatArray euArray = (jfloatArray)env->CallObjectMethod(euAngles, mid);
    mid = env->GetMethodID(cls, "getLength", "()I");
    jint size = env->CallIntMethod(euAngles, mid);
    env->GetFloatArrayRegion(euArray, 0, size, (jfloat *)eulerAngles);

    simxInt retVal = simxSetObjectOrientation(theClientID,objectHandle, relativeToObjectHandle, eulerAngles, operationMode);


    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxReadProximitySensor(JNIEnv *env, jobject obj, jint clientID, jint hdl, jobject ds, jobject dp, jobject doh, jobject dsnv, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt sensorHandle = hdl;
    simxInt operationMode = opMode;
    simxUChar* detectionState;
    simxFloat* detectedPoint;
    simxInt detectedObjectHandle;
    simxFloat* detectedSurfaceNormalVector;

    if (ds != NULL)
        detectionState = new simxUChar[1];
    if (dp != NULL)
        detectedPoint = new simxFloat[3];
    if (dsnv != NULL)
        detectedSurfaceNormalVector = new simxFloat[3]; 

    simxInt retVal = simxReadProximitySensor(theClientID, sensorHandle, (ds==NULL) ? 0 : detectionState, (dp==NULL) ? 0 : detectedPoint, (doh==NULL) ? 0 : &detectedObjectHandle, (dsnv==NULL) ? 0 : detectedSurfaceNormalVector, operationMode);   

    if (ds != NULL)
    {
        jclass cls = env->GetObjectClass(ds);
        jmethodID mid = env->GetMethodID(cls, "setValue", "(Z)V");
        env->CallVoidMethod( ds, mid, (detectionState[0]==1) );
        delete[] detectionState;
    }
    if (dp != NULL)
    {
        jclass cls = env->GetObjectClass(dp);
        jmethodID mid = env->GetMethodID(cls, "getNewArray", "(I)[F");
        jint size = 3;
        jfloatArray dpArray = (jfloatArray)env->CallObjectMethod(dp, mid, size);
        env->SetFloatArrayRegion(dpArray, 0, size, (jfloat *)detectedPoint);
        delete[] detectedPoint;
    }
    if (doh != NULL)
    {
        jclass cls = env->GetObjectClass(doh);
        jmethodID mid = env->GetMethodID(cls, "setValue", "(I)V");
        env->CallVoidMethod( doh, mid, detectedObjectHandle );
    }
    if (dsnv != NULL)
    {
        jclass cls = env->GetObjectClass(dsnv);
        jmethodID mid = env->GetMethodID(cls, "getNewArray", "(I)[F");
        jint size = 3;
        jfloatArray dsnvArray = (jfloatArray)env->CallObjectMethod(dsnv, mid, size);
        env->SetFloatArrayRegion(dsnvArray, 0, size, (jfloat *)detectedSurfaceNormalVector);
        delete[] detectedSurfaceNormalVector;
    }
    
    return retVal;
}


JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxLoadModel(JNIEnv *env, jobject obj, jint clientID, jstring mpn, jint opts, jobject hdl, jint opMode)
{
    simxInt theClientID = clientID;
    const simxChar *modelPathAndName = (simxChar*)env->GetStringUTFChars(mpn, 0);
    simxUChar options = (simxUChar)opts;
    simxInt baseHandle;
    simxInt operationMode = opMode;

    simxInt retVal = simxLoadModel(theClientID,modelPathAndName, options, &baseHandle, operationMode);

    jclass cls = env->GetObjectClass(hdl);
    jmethodID mid = env->GetMethodID(cls, "setValue", "(I)V");
    env->CallVoidMethod( hdl, mid, baseHandle);
    env->ReleaseStringUTFChars( mpn, modelPathAndName);
    return retVal;
}


JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxLoadUI(JNIEnv *env, jobject obj, jint clientID, jstring uipan, jint opts, jobject uih, jint opMode)
{
    simxInt theClientID = clientID;
    const simxChar *uiPathAndName = (simxChar*)env->GetStringUTFChars(uipan, 0);
    simxUChar options = (simxUChar)opts;
    simxInt count;
    simxInt** uiHandles = new simxInt*[1];
    simxInt operationMode = opMode;


    simxInt retVal = simxLoadUI(theClientID,uiPathAndName, options, &count, uiHandles, operationMode);
    
    env->ReleaseStringUTFChars( uipan,uiPathAndName);

    if (retVal==0)
    {
        jsize start = 0;
        jsize size = count;
        jclass cls = env->GetObjectClass(uih);
        jmethodID mid = env->GetMethodID(cls, "getNewArray", "(I)[I");
        jintArray uihArray = (jintArray)env->CallObjectMethod(uih, mid, size);  
        env->SetIntArrayRegion(uihArray, start, size, (const jint*)uiHandles[0]);
        simxReleaseBuffer((simxUChar*)uiHandles[0]);
    }
    delete[] uiHandles;

    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxLoadScene(JNIEnv *env, jobject obj, jint clientID, jstring scpan, jint opts, jint opMode)
{
    simxInt theClientID = clientID;
    const simxChar *scenePathAndName = (simxChar*)env->GetStringUTFChars(scpan, 0);
    simxUChar options = (simxUChar)opts;
    simxInt operationMode = opMode;

    simxInt retVal = simxLoadScene(theClientID,scenePathAndName, options, operationMode);

    env->ReleaseStringUTFChars( scpan, scenePathAndName );
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetUIHandle(JNIEnv *env, jobject obj, jint clientID, jstring uin, jobject hdl, jint opMode)
{
    simxInt theClientID = clientID;
    const simxChar *uiName = (simxChar*)env->GetStringUTFChars(uin, 0);
    simxInt handle;
    simxInt operationMode = opMode;

    simxInt retVal = simxGetUIHandle(theClientID,uiName, &handle, operationMode);

    jclass cls = env->GetObjectClass(hdl);
    jmethodID mid = env->GetMethodID(cls, "setValue", "(I)V");
    env->CallVoidMethod( hdl, mid, handle);

    env->ReleaseStringUTFChars( uin, uiName);
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxAddStatusbarMessage(JNIEnv *env, jobject obj, jint clientID, jstring msg, jint opMode)
{
    simxInt theClientID = clientID;
    const simxChar *message = (simxChar*)env->GetStringUTFChars(msg, 0);
    simxInt operationMode = opMode;

    simxInt retVal = simxAddStatusbarMessage(theClientID,message, operationMode);
    env->ReleaseStringUTFChars( msg,message);
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxAuxiliaryConsoleOpen(JNIEnv *env, jobject obj, jint clientID, jstring t, jint ml, jint m, jobject pos, jobject sz, jobject tc, jobject bc, jobject ch, jint opMode)
{
    simxInt theClientID = clientID;
    const simxChar *title = (simxChar*)env->GetStringUTFChars(t, 0);
    simxInt maxLines = ml;
    simxInt mode = m;
    simxInt position[2];
    simxInt size[2];
    simxFloat textColor[3];
    simxFloat backgroundColor[3];
    simxInt consoleHandle;
    simxInt operationMode = opMode;

    if (pos != NULL)
    {
        jclass cls = env->GetObjectClass(pos);
        jmethodID mid = env->GetMethodID(cls, "getArray", "()[I");
        jintArray posArray = (jintArray)env->CallObjectMethod(pos, mid);
        env->GetIntArrayRegion(posArray, 0, 2, (jint *)position);
    }

    if (sz != NULL)
    {
        jclass cls = env->GetObjectClass(sz);
        jmethodID mid = env->GetMethodID(cls, "getArray", "()[I");
        jintArray szArray = (jintArray)env->CallObjectMethod(sz, mid);
        env->GetIntArrayRegion(szArray, 0, 2, (jint *)size);
    }

    if (tc != NULL)
    {
        jclass cls = env->GetObjectClass(tc);
        jmethodID mid = env->GetMethodID(cls, "getArray", "()[F");
        jfloatArray tcArray = (jfloatArray)env->CallObjectMethod(tc, mid);
        env->GetFloatArrayRegion(tcArray, 0, 3, (jfloat *)textColor);
    }

    if (bc != NULL)
    {   
        jclass cls = env->GetObjectClass(bc);
        jmethodID mid = env->GetMethodID(cls, "getArray", "()[F");
        jfloatArray bcArray = (jfloatArray)env->CallObjectMethod(bc, mid);
        env->GetFloatArrayRegion(bcArray, 0, 3, (jfloat *)backgroundColor);
    }

    simxInt retVal = simxAuxiliaryConsoleOpen(theClientID,title, maxLines, mode, (pos==NULL) ? 0 : position, (sz == NULL) ? 0 : size, (tc == NULL) ? 0 : textColor, (bc == NULL) ? 0 : backgroundColor, &consoleHandle, operationMode);

    jclass cls = env->GetObjectClass(ch);
    jmethodID mid = env->GetMethodID(cls, "setValue", "(I)V");
    env->CallVoidMethod( ch, mid, consoleHandle);

    env->ReleaseStringUTFChars( t,title);
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxAuxiliaryConsolePrint(JNIEnv *env, jobject obj, jint clientID, jint ch, jstring t, jint opMode)
{
    simxInt theClientID = clientID;
    const simxChar *txt = t==NULL ? 0 : (simxChar*)env->GetStringUTFChars(t, 0);
    simxInt consoleHandle = ch;
    simxInt operationMode = opMode;

    simxInt retVal = simxAuxiliaryConsolePrint(theClientID,consoleHandle, txt, operationMode);
    if (t != NULL)
        env->ReleaseStringUTFChars( t, txt);
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxSetUIButtonLabel(JNIEnv *env, jobject obj, jint clientID, jint uih, jint uibi, jstring usl, jstring dsl, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt uiHandle = uih;
    simxInt uiButtonID = uibi;
    const simxChar *upStateLabel = (simxChar*)env->GetStringUTFChars(usl, 0);
    const simxChar *downStateLabel = (simxChar*)env->GetStringUTFChars(dsl, 0);
    simxInt operationMode = opMode;

    simxInt retVal = simxSetUIButtonLabel(theClientID,uiHandle, uiButtonID, upStateLabel, downStateLabel, operationMode);

    env->ReleaseStringUTFChars( usl,upStateLabel );
    env->ReleaseStringUTFChars( dsl,downStateLabel );
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetLastErrors(JNIEnv *env, jobject obj, jint clientID, jobject es, jint opMode)
{
    simxInt theClientID = clientID;
    simxChar** errorStrings = new simxChar*[1];
    simxInt errorCnt;
    simxInt operationMode = opMode;

    simxInt retVal = simxGetLastErrors(theClientID,&errorCnt, errorStrings, operationMode);

    if (retVal==0)
    {
        jclass cls = env->GetObjectClass(es);
        jmethodID mid = env->GetMethodID(cls, "getNewArray", "(I)[Ljava/lang/String;");
        jobjectArray esArray = (jobjectArray)env->CallObjectMethod( es, mid, errorCnt);

        int slen = 0;
        for(int i=0;i<errorCnt;i++) {
            jstring s = env->NewStringUTF(errorStrings[0]+slen);
            env->SetObjectArrayElement(esArray, i, s);
            slen += (env->GetStringLength(s) + 1);
        }
    }

    delete[] errorStrings;  
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetBooleanParameter(JNIEnv *env, jobject obj, jint clientID, jint pi, jobject pv, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt paramIdentifier = pi;
    simxUChar paramValue;
    simxInt operationMode = opMode;

    simxInt retVal = simxGetBooleanParameter(theClientID,paramIdentifier, &paramValue, operationMode);

    jclass cls = env->GetObjectClass(pv);
    jmethodID mid = env->GetMethodID(cls, "setValue", "(Z)V");
    env->CallVoidMethod( pv, mid, (paramValue==1));
    
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetStringParameter(JNIEnv *env, jobject obj, jint clientID, jint pi, jobject pv, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt paramIdentifier = pi;
    simxChar** paramValue = new simxChar*[1];
    simxInt operationMode = opMode;

    simxInt retVal = simxGetStringParameter(theClientID,paramIdentifier, paramValue, operationMode);

    if (retVal==0)
    {
        jclass cls = env->GetObjectClass(pv);
        jmethodID mid = env->GetMethodID(cls, "setValue", "(Ljava/lang/String;)V");
        jstring s = env->NewStringUTF(paramValue[0]);
        env->CallVoidMethod( pv, mid, s);
    }

    delete[] paramValue;

    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetCollisionHandle(JNIEnv *env, jobject obj, jint clientID, jstring con, jobject hdl, jint opMode)
{
    simxInt theClientID = clientID;
    const simxChar *collisionObjectName = (simxChar*)env->GetStringUTFChars(con, 0);
    simxInt handle;
    simxInt operationMode = opMode;

    simxInt retVal = simxGetCollisionHandle(theClientID,collisionObjectName, &handle, operationMode);

    env->ReleaseStringUTFChars(con,collisionObjectName);

    jclass cls = env->GetObjectClass(hdl);
    jmethodID mid = env->GetMethodID(cls, "setValue", "(I)V");
    env->CallVoidMethod( hdl, mid, handle);

    return retVal;
}


JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetDistanceHandle(JNIEnv *env, jobject obj, jint clientID, jstring don, jobject hdl, jint opMode)
{
    simxInt theClientID = clientID;
    const simxChar *distanceObjectName = (simxChar*)env->GetStringUTFChars(don, 0);
    simxInt handle;
    simxInt operationMode = opMode;

    simxInt retVal = simxGetDistanceHandle(theClientID,distanceObjectName, &handle, operationMode);

    env->ReleaseStringUTFChars(don,distanceObjectName);

    jclass cls = env->GetObjectClass(hdl);
    jmethodID mid = env->GetMethodID(cls, "setValue", "(I)V");
    env->CallVoidMethod( hdl, mid, handle);

    return retVal;
}


JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetCollectionHandle(JNIEnv *env, jobject obj, jint clientID, jstring con, jobject hdl, jint opMode)
{
    simxInt theClientID = clientID;
    const simxChar *collectionName = (simxChar*)env->GetStringUTFChars(con, 0);
    simxInt handle;
    simxInt operationMode = opMode;

    simxInt retVal = simxGetCollectionHandle(theClientID,collectionName, &handle, operationMode);

    env->ReleaseStringUTFChars(con,collectionName);

    jclass cls = env->GetObjectClass(hdl);
    jmethodID mid = env->GetMethodID(cls, "setValue", "(I)V");
    env->CallVoidMethod( hdl, mid, handle);

    return retVal;
}


JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxReadCollision(JNIEnv *env, jobject obj, jint clientID, jint hdl, jobject cs, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt collisionObjectHandle = hdl;
    simxUChar collisionState;
    simxInt operationMode = opMode;

    simxInt retVal = simxReadCollision(theClientID, collisionObjectHandle, &collisionState, operationMode);

    jclass cls = env->GetObjectClass(cs);
    jmethodID mid = env->GetMethodID(cls, "setValue", "(Z)V");
    env->CallVoidMethod( cs, mid, (collisionState==1));

    return retVal;
}


JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetObjects(JNIEnv *env, jobject obj, jint clientID, jint ot, jobject oh, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt objectType = ot;
    simxInt objectCount;
    simxInt** objectHandles = new simxInt*[1];
    simxInt operationMode = opMode;

    simxInt retVal = simxGetObjects(theClientID,objectType, &objectCount, objectHandles, operationMode);

    if (retVal==0)
    {
        jsize start = 0;
        jsize size = objectCount;
        jclass cls = env->GetObjectClass(oh);
        jmethodID mid = env->GetMethodID(cls, "getNewArray", "(I)[I");
        jintArray ohArray = (jintArray)env->CallObjectMethod(oh, mid, size);    
        env->SetIntArrayRegion(ohArray, start, size, (const jint*)objectHandles[0]);
    }

    delete[] objectHandles;
    return retVal;
}


JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxDisplayDialog(JNIEnv *env, jobject obj, jint clientID, jstring ttxt, jstring mtxt, jint dt, jstring itxt, jobject tc, jobject dc, jobject dh, jobject hdl, jint opMode)
{
    simxInt theClientID = clientID;
    const simxChar *titleText = (simxChar*)env->GetStringUTFChars(ttxt, 0);
    const simxChar *mainText = (simxChar*)env->GetStringUTFChars(mtxt, 0);
    const simxChar *initialText = (simxChar*)env->GetStringUTFChars(itxt, 0);
    simxInt dialogType = dt;

    jsize start = 0;
    jsize size = 6;     
    simxFloat titleColors[6];
    if (tc != NULL)
    {
        jclass cls = env->GetObjectClass(tc);
        jmethodID mid = env->GetMethodID(cls, "getArray", "()[F");
        jfloatArray tcArray = (jfloatArray)env->CallObjectMethod(tc, mid);  
        env->GetFloatArrayRegion(tcArray, start, size, (jfloat*)titleColors);
    }

    simxFloat dialogColors[6];
    if (dc != NULL)
    {
        jclass cls = env->GetObjectClass(dc);
        jmethodID mid = env->GetMethodID(cls, "getArray", "()[F");
        jfloatArray dcArray = (jfloatArray)env->CallObjectMethod(dc, mid);  
        env->GetFloatArrayRegion(dcArray, start, size, (jfloat*)dialogColors);
    }
    
    simxInt dialogHandle;
    simxInt uiHandle;
    simxInt operationMode = opMode;

    simxInt retVal = simxDisplayDialog(theClientID,titleText, mainText, dialogType, initialText, (tc==NULL) ? 0 : titleColors, (dc==NULL) ? 0 : dialogColors, &dialogHandle, &uiHandle, operationMode);

    env->ReleaseStringUTFChars(ttxt,titleText);
    env->ReleaseStringUTFChars(mtxt,mainText);
    env->ReleaseStringUTFChars(itxt,initialText);

    jclass cls = env->GetObjectClass(dh);
    jmethodID mid = env->GetMethodID(cls, "setValue", "(I)V");
    env->CallVoidMethod( dh, mid, dialogHandle);

    cls = env->GetObjectClass(hdl);
    mid = env->GetMethodID(cls, "setValue", "(I)V");
    env->CallVoidMethod( hdl, mid, uiHandle);

    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetDialogInput(JNIEnv *env, jobject obj, jint clientID, jint dh, jobject it, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt dialogHandle = dh;
    simxChar **inputText = new simxChar*[1]; 
    simxInt operationMode = opMode;

    simxInt retVal = simxGetDialogInput(theClientID,dialogHandle, inputText, operationMode);

    if (retVal==0)
    {
        jclass cls = env->GetObjectClass(it);
        jmethodID mid = env->GetMethodID(cls, "setValue", "(Ljava/lang/String;)V");
        jstring s = env->NewStringUTF(inputText[0]);
        env->CallVoidMethod( it, mid, s );  
    }

    delete[] inputText;
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxCopyPasteObjects(JNIEnv *env, jobject obj, jint clientID, jobject oh, jobject noh, jint opMode)
{
    simxInt theClientID = clientID;
    jclass cls = env->GetObjectClass(oh);
    jmethodID mid = env->GetMethodID(cls, "getLength", "()I");
    jsize start = 0;
    jsize size = env->CallIntMethod( oh, mid ); 
    simxInt* objectHandles = new simxInt[size];
    cls = env->GetObjectClass(noh);
    mid = env->GetMethodID(cls, "getArray", "()[I");
    jintArray ohArray = (jintArray)env->CallObjectMethod(oh, mid);  
    env->GetIntArrayRegion(ohArray, start, size, (jint*)objectHandles);

    simxInt objectCount = size;
    simxInt** newObjectHandles = new simxInt*[1];
    simxInt newObjectCount;
    simxInt operationMode = opMode;

    simxInt retVal = simxCopyPasteObjects(theClientID,(const simxInt*)objectHandles, objectCount, newObjectHandles, &newObjectCount, operationMode);

    if (retVal==0)
    {
        cls = env->GetObjectClass(noh);
        mid = env->GetMethodID(cls, "getNewArray", "(I)[I");
        jintArray nohArray = (jintArray)env->CallObjectMethod(noh, mid, newObjectCount);    
        env->SetIntArrayRegion( nohArray, start, newObjectCount, (const jint*)newObjectHandles[0] );
    }

    delete[] objectHandles;
    delete[] newObjectHandles;
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetObjectSelection(JNIEnv *env, jobject obj, jint clientID, jobject oh, jint opMode)
{
    simxInt theClientID = clientID;
    simxInt** objectHandles = new simxInt*[1];
    simxInt objectCount;
    simxInt operationMode = opMode;

    simxInt retVal = simxGetObjectSelection(theClientID, objectHandles, &objectCount, operationMode);

    if (retVal==0)
    {
        jsize start = 0;
        jsize size = (jsize)objectCount;
        jclass cls = env->GetObjectClass(oh);
        jmethodID mid = env->GetMethodID(cls, "getNewArray", "(I)[I");
        jintArray ohArray = (jintArray)env->CallObjectMethod(oh, mid, size);    
        env->SetIntArrayRegion( ohArray, start, size, (const jint*)objectHandles[0] );
    }

    delete[] objectHandles;
    return retVal;
}


JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxSetObjectSelection(JNIEnv *env, jobject obj, jint clientID, jobject hdl, jint opMode)
{
    simxInt theClientID = clientID;
    jclass cls = env->GetObjectClass(hdl);
    jmethodID mid = env->GetMethodID(cls, "getLength", "()I");

    jsize start = 0;
    jsize size = env->CallIntMethod( hdl, mid );    

    simxInt* objectHandles = new simxInt[size];
    cls = env->GetObjectClass(hdl);
    mid = env->GetMethodID(cls, "getArray", "()[I");
    jintArray hdlArray = (jintArray)env->CallObjectMethod(hdl, mid);    
    env->GetIntArrayRegion(hdlArray, start, size, (jint*)objectHandles);

    simxInt objectCount = size;
    simxInt operationMode = opMode;

    simxInt retVal = simxSetObjectSelection(theClientID,objectHandles, objectCount, operationMode);

    delete[] objectHandles;
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxClearFloatSignal(JNIEnv *env, jobject obj, jint clientID, jstring sn, jint opMode)
{
    simxInt theClientID = clientID;
    const simxChar *signalName = (simxChar*)env->GetStringUTFChars(sn, 0);
    simxInt operationMode = opMode;

    simxInt retVal = simxClearFloatSignal(theClientID,signalName, operationMode);

    env->ReleaseStringUTFChars(sn,signalName);
    return retVal;
}


JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxClearStringSignal(JNIEnv *env, jobject obj, jint clientID, jstring sn, jint opMode)
{
    simxInt theClientID = clientID;
    const simxChar *signalName = (simxChar*)env->GetStringUTFChars(sn, 0);
    simxInt operationMode = opMode;

    simxInt retVal = simxClearStringSignal(theClientID,signalName, operationMode);

    env->ReleaseStringUTFChars( sn,signalName);
    return retVal;
}


JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxClearIntegerSignal(JNIEnv *env, jobject obj, jint clientID, jstring sn, jint opMode)
{
    simxInt theClientID = clientID;
    const simxChar *signalName = (simxChar*)env->GetStringUTFChars(sn, 0);
    simxInt operationMode = opMode;

    simxInt retVal = simxClearIntegerSignal(theClientID,signalName, operationMode);

    env->ReleaseStringUTFChars( sn,signalName);
    return retVal;
}


JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetFloatSignal(JNIEnv *env, jobject obj, jint clientID, jstring sn, jobject sv, jint opMode)
{
    simxInt theClientID = clientID;
    const simxChar *signalName = (simxChar*)env->GetStringUTFChars(sn, 0);
    simxFloat signalValue[1];
    simxInt operationMode = opMode; 

    simxInt retVal = simxGetFloatSignal(theClientID,signalName, signalValue, operationMode);

    jclass cls = env->GetObjectClass(sv);
    jmethodID mid = env->GetMethodID(cls, "setValue", "(F)V");
    env->CallVoidMethod( sv, mid, signalValue[0]);

    env->ReleaseStringUTFChars( sn,signalName);
    return retVal;
}


JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetIntegerSignal(JNIEnv *env, jobject obj, jint clientID, jstring sn, jobject sv, jint opMode)
{
    simxInt theClientID = clientID;
    const simxChar *signalName = (simxChar*)env->GetStringUTFChars(sn, 0);
    simxInt signalValue;
    simxInt operationMode = opMode;

    simxInt retVal = simxGetIntegerSignal(theClientID, signalName, &signalValue, operationMode);

    jclass cls = env->GetObjectClass(sv);
    jmethodID mid = env->GetMethodID(cls, "setValue", "(I)V");
    env->CallVoidMethod( sv, mid, signalValue);

    env->ReleaseStringUTFChars( sn,signalName);
    
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetStringSignal(JNIEnv *env, jobject obj, jint clientID, jstring sn, jobject sv, jint opMode)
{
    simxInt theClientID = clientID;
    const simxChar *signalName = (simxChar*)env->GetStringUTFChars(sn, 0);
    simxUChar* signalValue;
    simxInt signalLength;
    simxInt operationMode = opMode;

    simxInt retVal = simxGetStringSignal(theClientID,signalName, &signalValue, &signalLength, operationMode);

    if (retVal==0)
    {
        jclass cls = env->GetObjectClass(sv);
        jmethodID mid = env->GetMethodID(cls, "getNewArray", "(I)[C");
        jcharArray s = (jcharArray)env->CallObjectMethod(sv, mid, signalLength);
        jchar* arr=env->GetCharArrayElements(s,0);
        for (jsize i=0;i<signalLength;i++)
            arr[i]=(unsigned char)signalValue[i];
        env->ReleaseCharArrayElements(s,arr, 0);            
    }
    env->ReleaseStringUTFChars(sn,signalName);

    return retVal;
}


JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetAndClearStringSignal(JNIEnv *env, jobject obj, jint clientID, jstring sn, jobject sv, jint opMode)
{
    simxInt theClientID = clientID;
    const simxChar *signalName = (simxChar*)env->GetStringUTFChars(sn, 0);
    simxUChar* signalValue;
    simxInt signalLength;
    simxInt operationMode = opMode;

    simxInt retVal = simxGetAndClearStringSignal(theClientID,signalName, &signalValue, &signalLength, operationMode);

    if (retVal==0)
    {
        jclass cls = env->GetObjectClass(sv);
        jmethodID mid = env->GetMethodID(cls, "getNewArray", "(I)[C");
        jcharArray s = (jcharArray)env->CallObjectMethod(sv, mid, signalLength);
        jchar* arr=env->GetCharArrayElements(s,0);
        for (jsize i=0;i<signalLength;i++)
            arr[i]=(unsigned char)signalValue[i];
        env->ReleaseCharArrayElements(s,arr, 0);            
    }
    env->ReleaseStringUTFChars(sn,signalName);

    return retVal;

}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxReadStringStream(JNIEnv *env, jobject obj, jint clientID, jstring sn, jobject sv, jint opMode)
{
    simxInt theClientID = clientID;
    const simxChar *signalName = (simxChar*)env->GetStringUTFChars(sn, 0);
    simxUChar* signalValue;
    simxInt signalLength;
    simxInt operationMode = opMode;

    simxInt retVal = simxReadStringStream(theClientID,signalName, &signalValue, &signalLength, operationMode);

    if (retVal==0)
    {
        jclass cls = env->GetObjectClass(sv);
        jmethodID mid = env->GetMethodID(cls, "getNewArray", "(I)[C");
        jcharArray s = (jcharArray)env->CallObjectMethod(sv, mid, signalLength);
        jchar* arr=env->GetCharArrayElements(s,0);
        for (jsize i=0;i<signalLength;i++)
            arr[i]=(unsigned char)signalValue[i];
        env->ReleaseCharArrayElements(s,arr, 0);            
    }
    env->ReleaseStringUTFChars(sn,signalName);

    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxSetFloatSignal(JNIEnv *env, jobject obj, jint clientID, jstring sn, jfloat sv, jint opMode)
{
    simxInt theClientID = clientID;
    const simxChar *signalName = (simxChar*)env->GetStringUTFChars(sn, 0);
    simxFloat signalValue = sv;
    simxInt operationMode = opMode;

    simxInt retVal = simxSetFloatSignal(theClientID,signalName, signalValue, operationMode);

    env->ReleaseStringUTFChars( sn,signalName);
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxSetIntegerSignal(JNIEnv *env, jobject obj, jint clientID, jstring sn, jint sv, jint opMode)
{
    simxInt theClientID = clientID;
    const simxChar *signalName = (simxChar*)env->GetStringUTFChars(sn, 0);
    simxInt signalValue = sv;
    simxInt operationMode = opMode;

    simxInt retVal = simxSetIntegerSignal( theClientID,signalName, signalValue, operationMode);
    env->ReleaseStringUTFChars( sn,signalName);
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxSetStringSignal(JNIEnv *env, jobject obj, jint clientID, jstring sn, jobject sv, jint opMode)
{
    simxInt theClientID = clientID;
    const simxChar *signalName = (simxChar*)env->GetStringUTFChars(sn, 0);
    simxInt operationMode = opMode;

    jclass cls = env->GetObjectClass(sv);
    jmethodID mid1 = env->GetMethodID(cls, "getLength", "()I");
    jint signalLength = env->CallIntMethod(sv, mid1);

    char* signalValue=new char[signalLength];
    jmethodID mid2 = env->GetMethodID(cls, "getArray", "()[C");
    jcharArray sigArray = (jcharArray)env->CallObjectMethod(sv, mid2);
    jchar* arr=env->GetCharArrayElements(sigArray,0);
    for (jsize i=0;i<signalLength;i++)
        signalValue[i]=(arr[i]&255);
    env->ReleaseCharArrayElements(sigArray,arr, 0);         

    simxInt retVal = simxSetStringSignal(theClientID,signalName, (unsigned char*)signalValue, (int)signalLength, operationMode);

    delete[] signalValue; 

    env->ReleaseStringUTFChars( sn,signalName);
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxAppendStringSignal(JNIEnv *env, jobject obj, jint clientID, jstring sn, jstring sv, jint opMode)
{
    simxInt theClientID = clientID;
    const simxChar *signalName = (simxChar*)env->GetStringUTFChars(sn, 0);
    const simxUChar *signalValue = (simxUChar*)env->GetStringUTFChars(sv, 0);

    simxInt signalLength = env->GetStringLength(sv);
    simxInt operationMode = opMode;

    simxInt retVal = simxAppendStringSignal(theClientID,signalName, signalValue, signalLength, operationMode);

    env->ReleaseStringUTFChars( sn,signalName);
    env->ReleaseStringUTFChars( sv, (char*)signalValue);
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxWriteStringStream(JNIEnv *env, jobject obj, jint clientID, jstring sn, jobject sv, jint opMode)
{
    simxInt theClientID = clientID;
    const simxChar *signalName = (simxChar*)env->GetStringUTFChars(sn, 0);
    simxInt operationMode = opMode;

    jclass cls = env->GetObjectClass(sv);
    jmethodID mid1 = env->GetMethodID(cls, "getLength", "()I");
    jint signalLength = env->CallIntMethod(sv, mid1);

    char* signalValue=new char[signalLength];
    jmethodID mid2 = env->GetMethodID(cls, "getArray", "()[C");
    jcharArray sigArray = (jcharArray)env->CallObjectMethod(sv, mid2);
    jchar* arr=env->GetCharArrayElements(sigArray,0);
    for (jsize i=0;i<signalLength;i++)
        signalValue[i]=(arr[i]&255);
    env->ReleaseCharArrayElements(sigArray,arr, 0);         

    simxInt retVal = simxWriteStringStream(theClientID,signalName, (unsigned char*)signalValue, signalLength, operationMode);

    delete[] signalValue; 

    env->ReleaseStringUTFChars( sn,signalName);
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetPingTime(JNIEnv *env, jobject obj, jint clientID, jobject pt)
{
    simxInt theClientID = clientID;
    simxInt pingTime;

    simxInt retVal = simxGetPingTime(theClientID,&pingTime);

    jclass cls = env->GetObjectClass(pt);
    jmethodID mid = env->GetMethodID(cls, "setValue", "(I)V");
    env->CallVoidMethod( pt, mid, pingTime);

    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetLastCmdTime(JNIEnv *env, jobject obj, jint clientID)
{
    simxInt theClientID = clientID;
    simxInt retVal = simxGetLastCmdTime(theClientID);
    return retVal;  
}


JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxSynchronousTrigger(JNIEnv *env, jobject obj, jint clientID)
{
    simxInt theClientID = clientID;
    simxInt retVal = simxSynchronousTrigger(theClientID);
    return retVal;  
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxSynchronous(JNIEnv *env, jobject obj, jint clientID, jboolean e)
{
    simxInt theClientID = clientID;
    simxUChar enable = e ? 1 : 0;
    simxInt retVal = simxSynchronous(theClientID,enable);
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxPauseCommunication(JNIEnv *env, jobject obj, jint clientID, jboolean e)
{
    simxInt theClientID = clientID;
    simxUChar enable = e ? 1 : 0;
    simxInt retVal = simxPauseCommunication(theClientID,enable);
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetInMessageInfo(JNIEnv *env, jobject obj, jint clientID, jint it, jobject i)
{
    simxInt theClientID = clientID;
    simxInt infoType = it;
    simxInt info;
    simxInt retVal = simxGetInMessageInfo(theClientID,infoType, &info);

    jclass cls = env->GetObjectClass(i);
    jmethodID mid = env->GetMethodID(cls, "setValue", "(I)V");
    env->CallVoidMethod( i, mid, info);
    return retVal;  
}


JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetOutMessageInfo(JNIEnv *env, jobject obj, jint clientID, jint it, jobject i)
{
    simxInt theClientID = clientID;
    simxInt infoType = it;
    simxInt info;
    simxInt retVal = simxGetOutMessageInfo(theClientID,infoType, &info);

    jclass cls = env->GetObjectClass(i);
    jmethodID mid = env->GetMethodID(cls, "setValue", "(I)V");
    env->CallVoidMethod( i, mid, info);
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxTransferFile(JNIEnv *env, jobject obj, jint clientID, jstring fpan, jstring fnss, jint to, jint opMode)
{
    simxInt theClientID = clientID;
    const simxChar *filePathAndName = (simxChar*)env->GetStringUTFChars(fpan, 0);
    const simxChar *fileName_serverSide = (simxChar*)env->GetStringUTFChars(fnss, 0);
    simxInt timeOut = to;
    simxInt operationMode = opMode;

    simxInt retVal = simxTransferFile(theClientID,filePathAndName, fileName_serverSide, timeOut, operationMode);
    
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxEraseFile(JNIEnv *env, jobject obj, jint clientID, jstring fnss, jint opMode)
{
    simxInt theClientID = clientID;
    const simxChar *fileName_serverSide = (simxChar*)env->GetStringUTFChars(fnss, 0);
    simxInt operationMode = opMode;

    simxInt retVal = simxEraseFile(theClientID,fileName_serverSide, operationMode);
    
    return retVal;

}


JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxCreateDummy(JNIEnv *env, jobject obj, jint clientID, jfloat size, jobject color, jobject handle, jint opMode)
{
    simxInt theClientID = clientID;
    simxFloat theSize = size;
    simxUChar colors[12];
    simxInt dummyHandle;
    simxInt operationMode = opMode;

    if (color != NULL)
    {
        jclass cls = env->GetObjectClass(color);
        jmethodID mid = env->GetMethodID(cls, "getArray", "()[C");
        jcharArray colArray = (jcharArray)env->CallObjectMethod(color, mid);
        env->GetCharArrayRegion(colArray, 0, 12, (jchar *)colors);
    }

    simxInt retVal = simxCreateDummy(theClientID, theSize, (color==NULL) ? 0 : colors, &dummyHandle, operationMode);

    jclass cls = env->GetObjectClass(handle);
    jmethodID mid = env->GetMethodID(cls, "setValue", "(I)V");
    env->CallVoidMethod( handle, mid, dummyHandle);

    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxQuery(JNIEnv *env, jobject obj, jint clientID, jstring sn, jobject sv,jstring rsn, jobject rsv, jint timeOutInMs)
{
    simxInt theClientID = clientID;
    const simxChar *signalName = (simxChar*)env->GetStringUTFChars(sn, 0);
    const simxChar *retSignalName = (simxChar*)env->GetStringUTFChars(rsn, 0);
    simxInt timeOutInMs_ = timeOutInMs;

    jclass cls = env->GetObjectClass(sv);
    jmethodID mid1 = env->GetMethodID(cls, "getLength", "()I");
    jint signalLength = env->CallIntMethod(sv, mid1);

    char* signalValue=new char[signalLength];
    jmethodID mid2 = env->GetMethodID(cls, "getArray", "()[C");
    jcharArray sigArray = (jcharArray)env->CallObjectMethod(sv, mid2);
    jchar* arr=env->GetCharArrayElements(sigArray,0);
    for (jsize i=0;i<signalLength;i++)
        signalValue[i]=(arr[i]&255);
    env->ReleaseCharArrayElements(sigArray,arr, 0);         

    simxUChar* retSignalValue;
    simxInt retSignalLength;
    simxInt retVal = simxQuery(theClientID,signalName, (unsigned char*)signalValue, signalLength,retSignalName,&retSignalValue,&retSignalLength,timeOutInMs_);

    delete[] signalValue; 

    if (retVal==0)
    {
        jclass cls2 = env->GetObjectClass(rsv);
        jmethodID mid3 = env->GetMethodID(cls2, "getNewArray", "(I)[C");
        jcharArray s = (jcharArray)env->CallObjectMethod(rsv, mid3, retSignalLength);
        jchar* arr=env->GetCharArrayElements(s,0);
        for (jsize i=0;i<retSignalLength;i++)
            arr[i]=(unsigned char)retSignalValue[i];
        env->ReleaseCharArrayElements(s,arr, 0);            
    }

    env->ReleaseStringUTFChars( sn,signalName);
    env->ReleaseStringUTFChars( rsn,retSignalName);
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetObjectGroupData(JNIEnv *env, jobject obj, jint clientID, jint objectType, jint dataType, jobject handles, jobject intData, jobject floatData, jobject stringData, jint operationMode)
{
    simxInt theClientID = clientID;
    simxInt theHandleCount;
    simxInt theIntDataCount;
    simxInt theFloatDataCount;
    simxInt theStringDataCount;
    simxInt theOpMode=operationMode;
    simxInt* theHandles;
    simxInt* theInts;
    simxFloat* theFloats;
    simxChar* theStrings;

    simxInt retVal = simxGetObjectGroupData(theClientID,objectType,dataType,&theHandleCount,&theHandles,&theIntDataCount,&theInts,&theFloatDataCount,&theFloats,&theStringDataCount,&theStrings,theOpMode);

    if (retVal==0)
    {

        jclass handlesCls = env->GetObjectClass(handles);
        jmethodID handlesMid = env->GetMethodID(handlesCls, "getNewArray", "(I)[I");
        jintArray handlesArray = (jintArray)env->CallObjectMethod(handles, handlesMid, theHandleCount); 
        env->SetIntArrayRegion( handlesArray, 0, theHandleCount, (const jint*)theHandles );

        jclass intDataCls = env->GetObjectClass(intData);
        jmethodID intDataMid = env->GetMethodID(intDataCls, "getNewArray", "(I)[I");
        jintArray intDataArray = (jintArray)env->CallObjectMethod(intData, intDataMid, theIntDataCount);    
        env->SetIntArrayRegion( intDataArray, 0, theIntDataCount, (const jint*)theInts );

        jclass floatDataCls = env->GetObjectClass(floatData);
        jmethodID floatDataMid = env->GetMethodID(floatDataCls, "getNewArray", "(I)[F");
        jfloatArray floatDataArray = (jfloatArray)env->CallObjectMethod(floatData, floatDataMid, theFloatDataCount);    
        env->SetFloatArrayRegion( floatDataArray, 0, theFloatDataCount, (const jfloat*)theFloats );

        jclass namesCls = env->GetObjectClass(stringData);
        jmethodID namesMid = env->GetMethodID(namesCls, "getNewArray", "(I)[Ljava/lang/String;");
        jobjectArray namesArray = (jobjectArray)env->CallObjectMethod( stringData, namesMid, theStringDataCount);
        int slen = 0;
        for(int i=0;i<theStringDataCount;i++) {
            jstring s = env->NewStringUTF(theStrings+slen);
            env->SetObjectArrayElement(namesArray, i, s);
            slen += (env->GetStringLength(s) + 1);
        }

    }
    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxGetObjectVelocity(JNIEnv *env, jobject obj, jint clientID, jint hdl, jobject lv, jobject av, jint opMode) 
{
    simxInt theClientID = clientID;
    simxInt objectHandle = hdl;
    simxInt operationMode = opMode;
    simxFloat linearVel[3];
    simxFloat angularVel[3];
        
    simxInt retVal = simxGetObjectVelocity(theClientID,objectHandle, (lv==NULL) ? 0 : linearVel, (av==NULL) ? 0 : angularVel, operationMode);

    if (lv != NULL)
    {
        jclass cls = env->GetObjectClass(lv);
        jmethodID mid = env->GetMethodID(cls, "getNewArray", "(I)[F");
        jfloatArray lvArray = (jfloatArray)env->CallObjectMethod(lv, mid, 3);
        env->SetFloatArrayRegion(lvArray, 0, 3, (const jfloat *)linearVel);
    }

    if (av != NULL)
    {       
        jclass cls = env->GetObjectClass(av);
        jmethodID mid = env->GetMethodID(cls, "getNewArray", "(I)[F");
        jfloatArray avArray = (jfloatArray)env->CallObjectMethod(av, mid, 3);
        env->SetFloatArrayRegion(avArray, 0, 3, (const jfloat *)angularVel);
    }

    return retVal;
}

JNIEXPORT jint JNICALL Java_coppelia_remoteApi_simxCallScriptFunction(JNIEnv *env, jobject obj, jint clientID, jstring scriptDescription, jint options, jstring functionName, jobject inInts, jobject inFloats, jobject inStrings, jobject inBuffer, jobject outInts, jobject outFloats, jobject outStrings, jobject outBuffer, jint operationMode)
{
    simxInt theClientID = clientID;
    const char *theScriptDescription = env->GetStringUTFChars(scriptDescription, 0);
    simxInt theOptions = options;
    const char *theFunctionName = env->GetStringUTFChars(functionName, 0);
    simxInt theOpMode=operationMode;

    simxInt theOutIntCnt;
    simxInt theOutFloatCnt;
    simxInt theOutStringCnt;
    simxInt theOutBufferSize;
    simxInt* theOutInts;
    simxFloat* theOutFloats;
    simxChar* theOutStrings;
    simxUChar* theOutBuffer;


    jsize theInIntCnt=0;
    simxInt* theInInts=NULL;
    if (inInts!=NULL)
    {
        jclass cls = env->GetObjectClass(inInts);
        jmethodID mid = env->GetMethodID(cls, "getLength", "()I");
        theInIntCnt = env->CallIntMethod(inInts, mid ); 
        theInInts = new simxInt[theInIntCnt];
        cls = env->GetObjectClass(inInts);
        mid = env->GetMethodID(cls, "getArray", "()[I");
        jintArray hdlArray = (jintArray)env->CallObjectMethod(inInts, mid); 
        env->GetIntArrayRegion(hdlArray,0, theInIntCnt, (jint*)theInInts);
    }

    jsize theInFloatCnt=0;
    simxFloat* theInFloats=NULL;
    if (inFloats!=NULL)
    {
        jclass cls = env->GetObjectClass(inFloats);
        jmethodID mid = env->GetMethodID(cls, "getLength", "()I");
        theInFloatCnt = env->CallIntMethod(inFloats, mid ); 
        theInFloats = new simxFloat[theInFloatCnt];
        cls = env->GetObjectClass(inFloats);
        mid = env->GetMethodID(cls, "getArray", "()[F");
        jfloatArray hdlArray = (jfloatArray)env->CallObjectMethod(inFloats, mid);   
        env->GetFloatArrayRegion(hdlArray,0, theInFloatCnt, (jfloat*)theInFloats);
    }

    jsize theInStringCnt=0;
    simxChar* theInStrings=NULL;
    if (inStrings!=NULL)
    {
        jclass cls = env->GetObjectClass(inStrings);
        jmethodID mid = env->GetMethodID(cls, "getLength", "()I");
        theInStringCnt = env->CallIntMethod(inStrings, mid );
        cls = env->GetObjectClass(inStrings);
        mid = env->GetMethodID(cls, "getArray", "()[Ljava/lang/String;");
        jobjectArray hdlArray = (jobjectArray)env->CallObjectMethod(inStrings, mid);    
        int totSize=0;
        for (int i=0;i<theInStringCnt;i++)
        {
            jstring str=(jstring)env->GetObjectArrayElement(hdlArray,i);
            const char *str2=env->GetStringUTFChars(str,0);
            totSize=extApi_getStringLength(str2)+1;
        }
        theInStrings = new simxChar[totSize];
        int _of=0;
        for (int i=0;i<theInStringCnt;i++)
        {
            jstring str=(jstring)env->GetObjectArrayElement(hdlArray,i);
            const char *str2=env->GetStringUTFChars(str,0);
            for (int j=0;j<int(extApi_getStringLength(str2))+1;j++)
            {
                theInStrings[_of]=str2[j];
                _of++;
            }
        }
    }

    jint theInBufferSize=0;
    char* theInBuffer=NULL;
    if (inBuffer!=NULL)
    {
        jclass inBufferCls = env->GetObjectClass(inBuffer);
        jmethodID inBufferMid1 = env->GetMethodID(inBufferCls, "getLength", "()I");
        theInBufferSize = env->CallIntMethod(inBuffer, inBufferMid1);
        theInBuffer=new char[theInBufferSize];
        jmethodID inBufferMid2 = env->GetMethodID(inBufferCls, "getArray", "()[C");
        jcharArray buffArray = (jcharArray)env->CallObjectMethod(inBuffer, inBufferMid2);
        jchar* arr=env->GetCharArrayElements(buffArray,0);
        for (jsize i=0;i<theInBufferSize;i++)
            theInBuffer[i]=(arr[i]&255);
        env->ReleaseCharArrayElements(buffArray,arr, 0);            
    }

    simxInt retVal = simxCallScriptFunction(theClientID,theScriptDescription,theOptions,theFunctionName,theInIntCnt,theInInts,theInFloatCnt,theInFloats,theInStringCnt,theInStrings,theInBufferSize,(simxUChar*)theInBuffer,&theOutIntCnt,&theOutInts,&theOutFloatCnt,&theOutFloats,&theOutStringCnt,&theOutStrings,&theOutBufferSize,&theOutBuffer,theOpMode);


    delete[] theInInts; 
    delete[] theInFloats; 
    delete[] theInStrings; 
    delete[] theInBuffer; 

    if (retVal==0)
    {
        if (outInts!=NULL)
        {
            jclass outIntsCls = env->GetObjectClass(outInts);
            jmethodID outIntsMid = env->GetMethodID(outIntsCls, "getNewArray", "(I)[I");
            jintArray outIntsArray = (jintArray)env->CallObjectMethod(outInts, outIntsMid, theOutIntCnt);   
            env->SetIntArrayRegion( outIntsArray, 0, theOutIntCnt, (const jint*)theOutInts );
        }

        if (outFloats!=NULL)
        {
            jclass outFloatsCls = env->GetObjectClass(outFloats);
            jmethodID outFloatsMid = env->GetMethodID(outFloatsCls, "getNewArray", "(I)[F");
            jfloatArray outFloatsArray = (jfloatArray)env->CallObjectMethod(outFloats, outFloatsMid, theOutFloatCnt);   
            env->SetFloatArrayRegion( outFloatsArray, 0, theOutFloatCnt, (const jfloat*)theOutFloats );
        }

        if (outStrings!=NULL)
        {
            jclass outStringsCls = env->GetObjectClass(outStrings);
            jmethodID outStringsMid = env->GetMethodID(outStringsCls, "getNewArray", "(I)[Ljava/lang/String;");
            jobjectArray outStringsArray = (jobjectArray)env->CallObjectMethod( outStrings, outStringsMid, theOutStringCnt);
            int slen = 0;
            for(int i=0;i<theOutStringCnt;i++)
            {
                jstring s = env->NewStringUTF(theOutStrings+slen);
                env->SetObjectArrayElement(outStringsArray, i, s);
                slen += (env->GetStringLength(s) + 1);
            }
        }

        if (outBuffer!=NULL)
        {
            jclass outBufferCls = env->GetObjectClass(outBuffer);
            jmethodID outBufferMid = env->GetMethodID(outBufferCls, "getNewArray", "(I)[C");
            jcharArray s = (jcharArray)env->CallObjectMethod(outBuffer, outBufferMid, theOutBufferSize);
            jchar* arr=env->GetCharArrayElements(s,0);
            for (jsize i=0;i<theOutBufferSize;i++)
                arr[i]=(unsigned char)theOutBuffer[i];
            env->ReleaseCharArrayElements(s,arr, 0);
        }
    }
    return retVal;
}


}

#endif /* _Included_extApiJava */
