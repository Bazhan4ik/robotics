#include "main.h"
#include <cstdio>
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"

#include "config.h"
#include "lady-brown.h"
#include "mogo-grabber.h"
#include "intake.h"
#include "robot-extension.h"


using namespace pros;





ASSET(curve_txt);
ASSET(curve2_txt);
ASSET(curve3_txt);
ASSET(curve_awp_txt);
ASSET(curveAwppointshighstake_txt);



Intake intake;







void opcontrol() {

  intake.stop();


  while (true) {

    int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    // move the robot
    chassis.tank(leftY, rightY);


    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      mogo_ungrab();
    }

    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      ext_move_up();
    } else {
      ext_move_down();
    }

    


    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
      pros::Task my_task(lb_get_ready, "My Task Name");
    }
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
      pros::Task my_task(lb_move_up, "My Task Name");
    }
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
      pros::Task my_task(lb_move_down, "My task name 2");
    }
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
      pros::Task my_task(lb_take_of, "My task 33");
    }

    pros::delay(40);
  }
}



void autonomous_skills() {



  intake.run_auto();
  pros::delay(500);
  intake.stop();

  chassis.moveToPose(0, 12.4, 0, 1000, { .maxSpeed=50 });
  chassis.waitUntilDone();

  chassis.turnToHeading(90, 1000, { .maxSpeed=70 });
  chassis.waitUntilDone();

  chassis.setPose(0,0,0);
  chassis.waitUntilDone();

// =-
  // =-
  // PICKING UP FIRST MOGO
  // =-
// =-

  chassis.moveToPose(0, -14, 0, 1000, { .forwards=false, .maxSpeed=80 });
  chassis.moveToPose(0, -12.4, 0, 1000, { .maxSpeed=50 });
  chassis.waitUntilDone();

  chassis.turnToHeading(-110, 1000, { .maxSpeed=80 });
  chassis.waitUntilDone();

  intake.run_auto();


  chassis.setPose(-41.271, 15.271, 75);
  chassis.follow(curve_txt, 15, 5000);

  chassis.waitUntilDone();

  chassis.setPose(0,0,0);
  chassis.turnToHeading(-215, 1000, { .maxSpeed=80 });
  chassis.waitUntilDone();

  pros::Task grabber_task(lb_get_ready);



  chassis.setPose(23.848, 46.519, 220);
  chassis.follow(curve2_txt, 10, 2600);

  
  // lb_move_up();
  

  chassis.waitUntilDone();

  chassis.setPose(0,0,0);
  chassis.moveToPose(0, 3, 0, 700, { .maxSpeed=50 });
  chassis.waitUntilDone();

  pros::delay(500);

  intake.stop();

  lady_brown_arm.move(127);
  // lb_move_up();
  pros::delay(700);
  chassis.setPose(0,0,0);
  chassis.moveToPose(0, -2.2, 0, 700, { .forwards=false, .maxSpeed=50, });
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);


  lb_move_down();


  chassis.moveToPose(0, -11.4, 0, 1000, { .forwards=false, .maxSpeed=50 });
  chassis.turnToHeading(-85, 1000, { .maxSpeed=80 });
  chassis.waitUntilDone();
  

  chassis.setPose(0,0,0);
  intake.run_auto();
  // chassis.moveToPose(0, 56, 0, 5000, { .maxSpeed=50 });
  chassis.moveToPose(0, 62, 0, 5000, { .maxSpeed=40 });

  chassis.waitUntilDone();

  chassis.setPose(0,0,0);
  chassis.turnToHeading(130, 1000, { .maxSpeed=60 });
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.moveToPose(0, 17.5, 0, 2000, { .maxSpeed=40 });
  chassis.turnToHeading(75, 1000, { .maxSpeed=80 });
  chassis.waitUntilDone();
  pros::delay(500);
  // chassis.setPose(0,0,0);
  // chassis.moveToPose(0)

  chassis.setPose(0,0,0);
  chassis.moveToPose(0, -11, 0, 1500, { .forwards=false, });
  chassis.waitUntilDone();

  mogo_ungrab();
  mogo_disable();

  chassis.setPose(0,0,0);
  chassis.moveToPose(0,10,0, 1500, { .minSpeed=90 });
  chassis.turnToHeading(50, 1000);
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.moveToPose(0, -15, 0, 1000,  { .forwards=false, .maxSpeed=35, });

  mogo_enable();

  pros::delay(2000);

  intake.stop();

  return;





}

void autonomous_blue_initial() {

  // intake.run_auto();
  // chassis.setPose(-24.176, -23.271, 0);
  // chassis.follow(curve_awp_txt, 15, 5000);
  // chassis.waitUntilDone();
  // pros::delay(1000);
  // intake.stop();;

  // return;
  
  
  chassis.turnToHeading(25, 1000, { .maxSpeed=80 });
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.moveToPose(0, 14.4, 0, 1000, { .maxSpeed=60 });
  chassis.turnToHeading(68, 1000, { .maxSpeed=80 });
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.moveToPose(0, -6, 0, 1000, { .forwards=false, .maxSpeed=40 });
  chassis.waitUntilDone();

  intake.run_auto();

  pros::delay(700);

  intake.stop();

  chassis.moveToPose(0, -0.5, 0, 1000, { .maxSpeed=100 });
  chassis.turnToHeading(-141, 1000);
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.moveToPose(0, -33, 0, 1500, { .forwards=false, .maxSpeed=90, });
  chassis.turnToHeading(-165, 1000);
  chassis.waitUntilDone();

  intake.run_auto();
  chassis.setPose(0,0,0);
  chassis.moveToPose(0, 25.5, 0, 2000, { .maxSpeed=40 });
  chassis.waitUntilDone();
  pros::delay(500);

  chassis.turnToHeading(98, 1000);
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.moveToPose(0, 10, 0, 1000, { .maxSpeed=70 });
  pros::delay(500);
  chassis.turnToHeading(120, 1000);
  chassis.waitUntilDone();
  pros::delay(200);
  chassis.setPose(0,0,0);
  chassis.moveToPose(0, 30, 0, 1000, { .maxSpeed=80 });
  pros::delay(200);
  lb_move_up();


  // chassis.moveToPose(0, 15, 0, 1000, { .maxSpeed=80 });
  
  return;

  // chassis.turnToHeading(36, 1000, { .maxSpeed=80 });
  // chassis.waitUntilDone();
  // chassis.setPose(0,0,0);
  // chassis.moveToPose(0, 8.4, 0, 1000, { .maxSpeed=40 });
  // chassis.waitUntilDone();
  // chassis.setPose(0,0,0);

  

  pros::delay(1500);

  intake.stop();

  chassis.moveToPose(0, -25, 0, 5000, { .forwards=false, .maxSpeed=80 });
  chassis.turnToHeading(80, 1000, { .maxSpeed=80 });
  chassis.waitUntilDone();
  pros::delay(700);
  lb_move_up();


  // chassis.moveToPose(0, -17, 0, 1000, { .forwards=false, .maxSpeed=80 });
  // chassis.turnToHeading(40, 1000, { .maxSpeed=80 });
  // chassis.waitUntilDone();

  // chassis.setPose(0,0,0);

  // chassis.moveToPose(0, 15, 0, 1000, { .maxSpeed=50 });
  // chassis.waitUntilDone();
  // pros::delay(500);

  // lb_move_up();
  // chassis.moveToPose(0, -50, 0, 2000, { .forwards=false });




  return;

  intake.run_auto();
  chassis.setPose(-46.409, 23.347, 0);
  chassis.follow(curve_awp_txt, 15, 5000);
  chassis.waitUntilDone();
  pros::delay(1000);
  intake.stop();

  return;
  chassis.setPose(0,0,0);
  chassis.moveToPose(0, 18.6, 0, 1000, { .maxSpeed=60 });
  intake.run_auto();
  chassis.waitUntilDone();
  pros::delay(1500);
  intake.stop();

  chassis.turnToHeading(180, 1000, { .maxSpeed=80 });
  chassis.waitUntilDone();

  return;
  
  chassis.setPose(-18.61, -33.628, 0);
  chassis.follow(curve_awp_txt, 15, 5000);

  intake.run_auto();



  
}

// void autonomous_aaa() {

//   awp_blue(intake);
//   awp_red(intake);

// }

void autonomous_blue() {

  chassis.turnToHeading(25, 1000, { .maxSpeed=80 });
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.moveToPose(0, 14.4, 0, 1000, { .maxSpeed=60 });
  chassis.turnToHeading(68, 1000, { .maxSpeed=80 });
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.moveToPose(0, -6, 0, 1000, { .forwards=false, .maxSpeed=40 });
  chassis.waitUntilDone();

  intake.run_auto();

  pros::delay(700);

  intake.stop();

  chassis.moveToPose(0, -0.5, 0, 1000, { .maxSpeed=100 });
  chassis.turnToHeading(-141, 1000);
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.moveToPose(0, -33, 0, 1500, { .forwards=false, .maxSpeed=90, });
  chassis.turnToHeading(-165, 1000);
  chassis.waitUntilDone();

  intake.run_auto();
  chassis.setPose(0,0,0);
  chassis.moveToPose(0, 25.5, 0, 2000, { .maxSpeed=40 });
  chassis.waitUntilDone();
  pros::delay(500);

  chassis.turnToHeading(98, 1000);
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.moveToPose(0, 10, 0, 1000, { .maxSpeed=70 });
  pros::delay(500);
  chassis.turnToHeading(120, 1000);
  chassis.waitUntilDone();
  pros::delay(200);
  chassis.setPose(0,0,0);
  chassis.moveToPose(0, 19, 0, 1000, { .maxSpeed=80 });
  pros::delay(200);
  lb_move_up();
}

void autonomous_red() {
  chassis.turnToHeading(-25, 1000, { .maxSpeed=80 });
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.moveToPose(0, 14.4, 0, 1000, { .maxSpeed=60 });
  chassis.turnToHeading(-68, 1000, { .maxSpeed=80 });
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.moveToPose(0, -6, 0, 1000, { .forwards=false, .maxSpeed=40 });
  chassis.waitUntilDone();

  intake.run_auto();

  pros::delay(700);

  intake.stop();

  chassis.moveToPose(0, -0.5, 0, 1000, { .maxSpeed=100 });
  chassis.turnToHeading(141, 1000);
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.moveToPose(0, -33, 0, 1500, { .forwards=false, .maxSpeed=90, });
  chassis.turnToHeading(165, 1000);
  chassis.waitUntilDone();

  intake.run_auto();
  chassis.setPose(0,0,0);
  chassis.moveToPose(0, 25.5, 0, 2000, { .maxSpeed=40 });
  chassis.waitUntilDone();
  pros::delay(500);

  chassis.turnToHeading(-98, 1000);
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.moveToPose(0, 10, 0, 1000, { .maxSpeed=70 });
  pros::delay(500);
  chassis.turnToHeading(-120, 1000);
  chassis.waitUntilDone();
  pros::delay(200);
  chassis.setPose(0,0,0);
  chassis.moveToPose(0, 30, 0, 1000, { .maxSpeed=80 });
  pros::delay(200);
  lb_move_up();
}

void autonomous_failed_high_stake() {


  chassis.turnToHeading(25, 1000, { .maxSpeed=80 });
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.moveToPose(0, 14.4, 0, 1000, { .maxSpeed=60 });
  chassis.turnToHeading(68, 1000, { .maxSpeed=80 });
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.moveToPose(0, -6, 0, 1000, { .forwards=false, .maxSpeed=40 });
  chassis.waitUntilDone();

  intake.run_auto();

  pros::delay(700);

  intake.stop();

  chassis.moveToPose(0, -0.5, 0, 1000, { .maxSpeed=100 });
  chassis.turnToHeading(-141, 1000);
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.moveToPose(0, -33, 0, 1500, { .forwards=false, .maxSpeed=90, });
  chassis.turnToHeading(-157, 1000);
  chassis.waitUntilDone();

  intake.run_auto();
  chassis.setPose(0,0,0);
  chassis.moveToPose(0, 25.5, 0, 2000, { .maxSpeed=40 });
  chassis.waitUntilDone();
  pros::delay(500);
  chassis.moveToPose(0, 13, 0, 1000, { .forwards=false, .maxSpeed=80, });

  // last ring

  chassis.turnToHeading(68, 1000);
  chassis.waitUntilDone();

  lb_get_ready();
  pros::delay(500);
  intake.run_auto();
  chassis.setPose(-12.215, -36.271, 230);
  chassis.follow(curveAwppointshighstake_txt, 15, 5000);
  chassis.waitUntilDone();

  intake.stop();
  return;

  intake.stop();
  pros::delay(200);

  lady_brown_arm.move(127);
  pros::delay(2000);
  chassis.setPose(0,0,0);
  chassis.moveToPose(0, -5, 0, 1000, { .forwards=false, .maxSpeed=50 });

  lady_brown_arm.set_brake_mode(MOTOR_BRAKE_COAST);
  lady_brown_arm.brake();
  
}

void autonomous() {
  ext_move_up();
  chassis.moveToPose(0, 37.5, 0, 2000);
  chassis.waitUntilDone();
  ext_move_down();
  pros::delay(100);
  chassis.moveToPose(0, 10, 0, 2000, { .forwards=false });
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


void initialize(){



  pros::lcd::initialize(); // initialize brain screen



  chassis.calibrate();     // calibrate sensors
  rotation_arm.reset();
  rotation_arm.reset_position();

  pros::lcd::register_btn1_cb(on_center_button);



  pros::Task screen_task([&]() {
    while (true) {
      // print robot location to the brain screen
      pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
      pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
      pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
      pros::lcd::print(4, "ARM ROTATION: %d", rotation_arm.get_position());

      pros::delay(50);
    }
  });


  vision_sensor.set_signature(RED_SIGNATURE, &RED_SIG);

  // intake.run_auto();
  intake.set_team_alliance("red");



  pros::Task intake_task(intake.task);
  pros::Task grabber_task(mogo_grabber);


  
}