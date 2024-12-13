

#include "config.h"
#include "robot-extension.h"




bool arm_up = false;


void ext_move_up() {
    if(arm_up) {
        return;
    }
    arm_up = true;
    pneumatic_robot_extension.set_value(true);
}

void ext_move_down() {
    if(arm_up) {
        arm_up = false;
        pneumatic_robot_extension.set_value(false);
    }
}