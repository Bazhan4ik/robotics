// #include "intake.h"
// #include "main.h"
// #include <cstdio>
// #include "pros/llemu.hpp"
// #include "pros/rtos.hpp"
// #include "config.h"
// #include "lady-brown.h"



// void awp_blue(Intake intake) {


//   chassis.turnToHeading(25, 1000, { .maxSpeed=80 });
//   chassis.waitUntilDone();
//   chassis.setPose(0,0,0);
//   chassis.moveToPose(0, 14.4, 0, 1000, { .maxSpeed=60 });
//   chassis.turnToHeading(68, 1000, { .maxSpeed=80 });
//   chassis.waitUntilDone();
//   chassis.setPose(0,0,0);
//   chassis.moveToPose(0, -6, 0, 1000, { .forwards=false, .maxSpeed=40 });
//   chassis.waitUntilDone();

//   intake.run_auto();

//   pros::delay(700);

//   intake.stop();

//   chassis.moveToPose(0, -0.5, 0, 1000, { .maxSpeed=100 });
//   chassis.turnToHeading(-141, 1000);
//   chassis.waitUntilDone();
//   chassis.setPose(0,0,0);
//   chassis.moveToPose(0, -33, 0, 1500, { .forwards=false, .maxSpeed=90, });
//   chassis.turnToHeading(-165, 1000);
//   chassis.waitUntilDone();

//   intake.run_auto();
//   chassis.setPose(0,0,0);
//   chassis.moveToPose(0, 25.5, 0, 2000, { .maxSpeed=40 });
//   chassis.waitUntilDone();
//   pros::delay(500);

//   chassis.turnToHeading(98, 1000);
//   chassis.waitUntilDone();
//   chassis.setPose(0,0,0);
//   chassis.moveToPose(0, 10, 0, 1000, { .maxSpeed=70 });
//   pros::delay(500);
//   chassis.turnToHeading(120, 1000);
//   chassis.waitUntilDone();
//   pros::delay(200);
//   chassis.setPose(0,0,0);
//   chassis.moveToPose(0, 30, 0, 1000, { .maxSpeed=80 });
//   pros::delay(200);
//   lb_move_up();
// }

// void awp_blue(Intake intake) {


//   chassis.turnToHeading(-25, 1000, { .maxSpeed=80 });
//   chassis.waitUntilDone();
//   chassis.setPose(0,0,0);
//   chassis.moveToPose(0, 14.4, 0, 1000, { .maxSpeed=60 });
//   chassis.turnToHeading(-68, 1000, { .maxSpeed=80 });
//   chassis.waitUntilDone();
//   chassis.setPose(0,0,0);
//   chassis.moveToPose(0, -6, 0, 1000, { .forwards=false, .maxSpeed=40 });
//   chassis.waitUntilDone();

//   intake.run_auto();

//   pros::delay(700);

//   intake.stop();

//   chassis.moveToPose(0, -0.5, 0, 1000, { .maxSpeed=100 });
//   chassis.turnToHeading(141, 1000);
//   chassis.waitUntilDone();
//   chassis.setPose(0,0,0);
//   chassis.moveToPose(0, -33, 0, 1500, { .forwards=false, .maxSpeed=90, });
//   chassis.turnToHeading(165, 1000);
//   chassis.waitUntilDone();

//   intake.run_auto();
//   chassis.setPose(0,0,0);
//   chassis.moveToPose(0, 25.5, 0, 2000, { .maxSpeed=40 });
//   chassis.waitUntilDone();
//   pros::delay(500);

//   chassis.turnToHeading(-98, 1000);
//   chassis.waitUntilDone();
//   chassis.setPose(0,0,0);
//   chassis.moveToPose(0, 10, 0, 1000, { .maxSpeed=70 });
//   pros::delay(500);
//   chassis.turnToHeading(-120, 1000);
//   chassis.waitUntilDone();
//   pros::delay(200);
//   chassis.setPose(0,0,0);
//   chassis.moveToPose(0, 30, 0, 1000, { .maxSpeed=80 });
//   pros::delay(200);
//   lb_move_up();
// }