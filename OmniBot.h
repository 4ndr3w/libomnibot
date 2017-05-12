#ifndef OMNIBOT_H
#define OMNIBOT_H

#include "OmniBotComm.h"
#include "OmniBotCommandSender.h"
#include "OmniBotStateReceiver.h"

class OmniBot {
    OmniBotCommandSender sender;
    OmniBotStateReceiver receiver;
public:
    OmniBot(const char* address);

    RobotResponse setOmniGoal(bool closedLoop, double x, double y, double theta);
    RobotResponse setDifferentialGoal(bool closedLoop, double linear, double angular);
    RobotResponse startProfiledDifferentialGoal();

    RobotResponse pushProfilePoint(PathPoint p);
    RobotResponse clearProfileBuffer();

    RobotResponse setDifferentialLinearPID(double p, double i, double d, double f);
    RobotResponse setDifferentialAngularPID(double p, double i, double d, double f);

    RobotPose getPose();
    PIDInfo getPIDInfo();
};

#endif