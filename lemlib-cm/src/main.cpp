#include "main.h"
#include <cstdio>
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"

#include "config.h"
#include "lady-brown.h"
#include "mogo-grabber.h"
#include "intake.h"



ASSET(curve_txt);
ASSET(curve2_txt);
ASSET(curve3_txt);



Intake intake;







void opcontrol() {



  while (true) {

    int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    // move the robot
    chassis.tank(leftY, rightY);


    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      mogo_ungrab();
    }



    


    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
      pros::Task my_task(lb_get_ready, "My Task Name");
    }
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
      pros::Task my_task(lb_move_up, "My Task Name");
    }
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
      pros::Task my_task(lb_move_down, "My task name 2");
    }

    pros::delay(40);
  }
}



void autonomous() {
  // chassis.moveToPose(0, -24, 0, 1000, { .forwards=false, .maxSpeed=70 });


  chassis.setPose(-37.684, -30.027, 0);
  chassis.follow(curve3_txt, 12, 5000);

  // chassis.waitUntilDone();

  // pros::delay(1000);


  // chassis.setPose(11.727, -6.237, 0);
  // chassis.follow(curve2_txt, 10, 5000, false);
}



void initialize(){
  pros::lcd::initialize(); // initialize brain screen
  chassis.calibrate();     // calibrate sensors
  rotation_arm.reset();
  rotation_arm.reset_position();

  pros::Task screen_task([&]() {
    while (true) {
      // print robot location to the brain screen
      pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
      pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
      pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading

      pros::delay(50);
    }
  });


  vision_sensor.set_signature(RED_SIGNATURE, &RED_SIG);

  // intake.run_auto();
  intake.set_team_alliance("blue");

  pros::Task intake_task(intake.task);
  pros::Task grabber_task(mogo_grabber);
}

void on_center_button(){
  static bool pressed = false;
  pressed = !pressed;
  if (pressed)
  {
    pros::lcd::set_text(2, "I was pressed!");
  }
  else
  {
    pros::lcd::clear_line(2);
  }
}
void disabled() {}
void competition_initialize(){}