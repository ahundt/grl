#ifndef GRL_READ_KUKAIIWA_FROM_FLATBUFFER
#define GRL_READ_KUKAIIWA_FROM_FLATBUFFER

#define FLATBUFFERS_DEBUG_VERIFICATION_FAILURE

#include "flatbuffers/flatbuffers.h"
#include "grl/flatbuffer/JointState_generated.h"
#include "grl/flatbuffer/ArmControlState_generated.h"
#include "grl/flatbuffer/KUKAiiwa_generated.h"
#include "grl/flatbuffer/LinkObject_generated.h"
#include "grl/flatbuffer/Time_generated.h"
#include "grl/flatbuffer/flatbuffer.hpp"

#include <cstdio> // For printing and file access.
#include <iostream>
namespace grl {
void getDeviceTime(){
    std::string curWorkingDir = grl::GetCurrentWorkingDir();
    std::cout << "CurrentWorkingDir: " << curWorkingDir << std::endl;
    FILE* file = fopen("2018_01_22_19_00_29_Kukaiiwa.iiwa", "rb");
    fseek(file, 0L, SEEK_END);
    int length = ftell(file);
    fseek(file, 0L, SEEK_SET);
    char *data = new char[length];
    fread(data, sizeof(char), length, file);
    fclose(file);
    auto kukaStates = grl::flatbuffer::GetKUKAiiwaStates(data);
    auto states = kukaStates->states();
    auto KUKAiiwaState = states->Get(0);
    auto FRIMessage = KUKAiiwaState->FRIMessage();
    auto timeEvent = FRIMessage->timeStamp();
    auto device_time = timeEvent->device_time();
    std::cout<<"Device Time: "<< device_time << std::endl;

    std::size_t state_size = states->size();
    for(int i = 1; i<state_size; ++i){
        KUKAiiwaState = states->Get(i);
        FRIMessage = KUKAiiwaState->FRIMessage();
        timeEvent = FRIMessage->timeStamp();
        auto nextdevice_time = timeEvent->device_time();
        auto diff = nextdevice_time - device_time;
        device_time = nextdevice_time;
        std::cout<<"Time Diff: "<< diff <<std::endl;
    }

}

}
#endif