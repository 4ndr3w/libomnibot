#include "OmniBot.h"
#include <stdio.h>
#include <unistd.h>

int main() {
    OmniBot bot("10.49.77.2");
    
    while ( 1 ){
        RobotPose pose = bot.getPose();

        printf("pos: %2.2f\n", pose.forwardDistance);

        usleep(20000);
    }
}