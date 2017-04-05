#ifndef OMNIBOT_STATERECEIVER
#define OMNIBOT_STATERECEIVER

#include "OmniBotComm.h"
#include <thread>
#include <atomic>

class OmniBotStateReceiver {
    std::thread thread;
    std::atomic<RobotState> state;
    void socketThread();
public:
    OmniBotStateReceiver();
    RobotState getState();
};

#endif