#include "OmnibotStateReceiver.h"
#include <netinet/in.h>
#include <sys/socket.h>
#include <stdio.h>
#include <errno.h>


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

OmniBotStateReceiver::OmniBotStateReceiver()
    : thread(&OmniBotStateReceiver::socketThread, this) {
        
}

void OmniBotStateReceiver::socketThread() {
    int sock;
    if ( (sock = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) {
      perror("socket() failed");
    }

    sockaddr_in bindAddr;
    bindAddr.sin_family = AF_INET;
    bindAddr.sin_addr.s_addr = INADDR_ANY;
    bindAddr.sin_port = htons(1338);

    bind(sock, (sockaddr*)&bindAddr, sizeof(sockaddr_in)); 
    while ( 1 ) {
        sockaddr_in robotAddr;
        socklen_t len = sizeof(sockaddr_in);
        RobotState robot;

        recvfrom(sock, &robot, sizeof(RobotState), 0, (sockaddr*)&robotAddr, &len);

        // All the values we receive are big endian
        // byteSwap if we're running on a little endian machine
        state.pose.x = byteSwap(state.pose.x); 
        state.pose.y = byteSwap(state.pose.y); 
        state.pose.theta = byteSwap(state.pose.theta); 


        state.pose.vx = byteSwap(state.pose.vx); 
        state.pose.vy = byteSwap(state.pose.vy); 
        state.pose.vth = byteSwap(state.pose.vth); 

        state.pose.forwardDistance = byteSwap(state.pose.forwardDistance); 

        state.control.linearActual = byteSwap(state.control.linearActual);
        state.control.linearSetpoint = byteSwap(state.control.linearSetpoint);

        state.control.angularActual = byteSwap(state.control.angularActual);
        state.control.angularSetpoint = byteSwap(state.control.angularSetpoint);

        state = robot;
    }
}


RobotState OmniBotStateReceiver::getState() {
    return state;
}