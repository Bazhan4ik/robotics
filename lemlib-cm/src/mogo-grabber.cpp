#include "mogo-grabber.h"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"

#include "config.h"


bool grab = true;
bool givetime = false;
bool grabbed = false;

void mogo_disable() {
  grab = false;
}
void mogo_enable() {
  grab = true;
}

void mogo_grabber() {
  while(true) {
    pros::delay(30);

    pros::lcd::print(3, "%d", distance_sensor.get_distance());

    if(grabbed) {
      continue;
    }
    if(givetime) {
      givetime = false;
      pros::delay(700);
    }

    if(grab) {


      if(distance_sensor.get_distance() < 37) {
        pneumatic_mogo_grabber.set_value(true);
        master.rumble("..");
        grabbed = true;
      }

      
    }
    
  }
}
void mogo_ungrab() {
  pneumatic_mogo_grabber.set_value(false);
  givetime = true;
  grab = true;
  grabbed = false;
}