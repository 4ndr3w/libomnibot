#include "OmniBot.h"

// endian swap
static double byteSwap(double v)
{
    // Only swap if the build target is little endian, cRIO is big endian
    #if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    return v;

    #else

    union {
        uint64_t i;
        double  d;
    } conv;
    conv.d = v;

    conv.i = (conv.i & 0x00000000FFFFFFFF) << 32 | (conv.i & 0xFFFFFFFF00000000) >> 32;
    conv.i = (conv.i & 0x0000FFFF0000FFFF) << 16 | (conv.i & 0xFFFF0000FFFF0000) >> 16;
    conv.i = (conv.i & 0x00FF00FF00FF00FF) << 8  | (conv.i & 0xFF00FF00FF00FF00) >> 8;

    return conv.d;

    #endif
}


OmniBot::OmniBot(const char* address)
    : sender(address) {

}

RobotResponse OmniBot::setOmniGoal(bool closedLoop, double x, double y, double theta) {
    RobotMessage msg;
    msg.type = MSG_OMNI_GOAL;
    msg.data.omniGoal.useClosedLoop = closedLoop;
    msg.data.omniGoal.x = byteSwap(x);
    msg.data.omniGoal.y = byteSwap(y);
    msg.data.omniGoal.theta = byteSwap(theta);

    return sender.pushCommand(msg);
}

RobotResponse OmniBot::setDifferentialGoal(bool closedLoop, double linear, double angular) {
    RobotMessage msg;
    msg.type = MSG_DIFFERENTIAL_GOAL;

    msg.data.differentialGoal.useClosedLoop = closedLoop;
    msg.data.differentialGoal.distanceGoal = byteSwap(linear);
    msg.data.differentialGoal.angleGoal = byteSwap(angular);

    return sender.pushCommand(msg);
}

RobotResponse OmniBot::startProfiledDifferentialGoal() {
    RobotMessage msg;
    msg.type = MSG_PROFILED_DIFFERENTIAL_GOAL;
    return sender.pushCommand(msg);
}

RobotResponse OmniBot::pushProfilePoint(PathPoint p) {
    RobotMessage msg;
    msg.type = MSG_APPEND_PROFILE_BUFFER;

    msg.data.point.linear.pos = byteSwap(msg.data.point.linear.pos);
    msg.data.point.linear.pos = byteSwap(msg.data.point.linear.vel);
    msg.data.point.linear.acc = byteSwap(msg.data.point.linear.acc);

    msg.data.point.angular.pos = byteSwap(msg.data.point.angular.pos);
    msg.data.point.angular.vel = byteSwap(msg.data.point.angular.vel);
    msg.data.point.angular.acc = byteSwap(msg.data.point.angular.acc);

    msg.data.point = p;
    return sender.pushCommand(msg);
}

RobotResponse OmniBot::clearProfileBuffer() {
    RobotMessage msg;
    msg.type = MSG_CLEAR_PROFILE_BUFFER;

    msg.data.point.linear.pos = byteSwap(msg.data.point.linear.pos);
    msg.data.point.linear.pos = byteSwap(msg.data.point.linear.vel);
    msg.data.point.linear.acc = byteSwap(msg.data.point.linear.acc);

    msg.data.point.angular.pos = byteSwap(msg.data.point.angular.pos);
    msg.data.point.angular.vel = byteSwap(msg.data.point.angular.vel);
    msg.data.point.angular.acc = byteSwap(msg.data.point.angular.acc);

    return sender.pushCommand(msg);
}

RobotResponse OmniBot::setDifferentialLinearPID(double p, double i, double d, double f) {
    RobotMessage msg;
    msg.type = MSG_SET_DIFFERENTIAL_LINEAR_PID;

    msg.data.pid.p = byteSwap(p);
    msg.data.pid.i = byteSwap(i);
    msg.data.pid.d = byteSwap(d);
    msg.data.pid.f = byteSwap(f);

    return sender.pushCommand(msg);
}


RobotResponse OmniBot::setDifferentialAngularPID(double p, double i, double d, double f) {
    RobotMessage msg;
    msg.type = MSG_SET_DIFFERENTIAL_ANGULAR_PID;

    msg.data.pid.p = byteSwap(p);
    msg.data.pid.i = byteSwap(i);
    msg.data.pid.d = byteSwap(d);
    msg.data.pid.f = byteSwap(f);

    return sender.pushCommand(msg);
}