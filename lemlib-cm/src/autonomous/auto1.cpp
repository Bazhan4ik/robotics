// RED TEAM
void autonomous_ueaouoaeu() {


  chassis.moveToPose(0, -97, 0, 1500, { .forwards=false, .maxSpeed=100 });
  chassis.waitUntilDone();
  chassis.turnToHeading(-37, 600, { .maxSpeed=80 });
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.waitUntilDone();
  // chassis.moveToPose(0, -28, chassis.getPose().theta, 1000, { .forwards=false, .maxSpeed=100 });
  chassis.moveToPose(0, -40, chassis.getPose().theta, 700, { .forwards=false, .maxSpeed=55, .minSpeed=45});
  chassis.waitUntilDone();
  // pneumatic_grabber.set_value(true);
  pros::delay(100);
  chassis.moveToPose(0, -20, chassis.getPose().theta, 1000, { .maxSpeed=90, .minSpeed=50 });
  chassis.waitUntilDone();
  intake.move(-120);
  // global_allow_intake = true;
  chassis.turnToHeading(75, 600, {.maxSpeed=60});
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.waitUntilDone();
  chassis.moveToPose(0, 35, 0, 1000, { .maxSpeed=55 });
  chassis.waitUntilDone();

  pros::delay(800);
  // global_allow_intake = false;
  intake.move(127);

  chassis.moveToPose(0, 15, 0, 1000, { .forwards=false, .maxSpeed=80, .minSpeed=50 });
  chassis.waitUntilDone();

  // global_allow_intake = false;
  intake.brake();
  ungrab();

  chassis.moveToPose(0, 37, 0, 1000, { .maxSpeed=80, .minSpeed=50 });
  chassis.waitUntilDone();

  chassis.turnToHeading(-130, 1000, { .maxSpeed=60, });
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.waitUntilDone();

  chassis.moveToPose(0, -47, chassis.getPose().theta, 1000, { .forwards=false, .maxSpeed=70 });
  chassis.waitUntilDone();


  pros::delay(200);

  chassis.turnToHeading(137, 1000, { .maxSpeed=60 });
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.waitUntilDone();

  // global_allow_intake = true;
  intake.move(-127);

  chassis.moveToPose(chassis.getPose().x, 130, chassis.getPose().theta, 5000, { .maxSpeed=55, .minSpeed=50 });
  chassis.waitUntilDone();

  // chassis.turnToHeading(-85, 1000, { .maxSpeed=80 });
  // chassis.waitUntilDone();

  // chassis.setPose(0,0,0);
  // chassis.waitUntilDone();

  // chassis.moveToPose(0, 7, 0, 1000, { .maxSpeed=80 });
  // chassis.waitUntilDone();
  // chassis.turnToHeading(-90, 1000, { .maxSpeed=80 });
  // chassis.waitUntilDone();
  chassis.turnToHeading(130, 1000, { .maxSpeed=80 });
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.waitUntilDone();

  chassis.moveToPose(0, 110, 0, 1000, { .maxSpeed=90 });
  move_arm_max();
  chassis.waitUntilDone();
  move_arm_max();

  return;

  chassis.setPose(0,0,0);
  chassis.waitUntilDone();
  chassis.turnToHeading(45, 1000, { .maxSpeed=80 });
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.waitUntilDone();

  move_arm_up();

  return;

  move_arm_up();

  chassis.moveToPose(0, 100, 0, 2000, { .forwards=false, .maxSpeed=80, });
}
// BLUE TEAM
void autonomous_blue() {

  chassis.moveToPose(0, -97, 0, 1500, { .forwards=false, .maxSpeed=100 });
  chassis.waitUntilDone();
  chassis.turnToHeading(37, 600, { .maxSpeed=80 });
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.waitUntilDone();
  // chassis.moveToPose(0, -28, chassis.getPose().theta, 1000, { .forwards=false, .maxSpeed=100 });
  chassis.moveToPose(0, -30, chassis.getPose().theta, 700, { .forwards=false, .maxSpeed=55, .minSpeed=45});
  chassis.waitUntilDone();

// *
  // *
  // GRABS THE MOGO HERE
  // *
// *

  // pneumatic_grabber.set_value(true);



  pros::delay(100);
  chassis.moveToPose(0, -20, chassis.getPose().theta, 1000, { .maxSpeed=90, .minSpeed=50 });
  chassis.waitUntilDone();

// *
  // *
  // RUNS THE INTAKE HERE
  // *
// *

  intake.move(-127);
  // global_allow_intake = true;


  chassis.turnToHeading(-75, 600, {.maxSpeed=60});
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.waitUntilDone();
  chassis.moveToPose(0, 35, 0, 1000, { .maxSpeed=55 });
  chassis.waitUntilDone();


// *
  // *
  // STOPS THE INTAKE HERE
  // *
// *

  pros::delay(1000);
  // global_allow_intake = false;
  intake.move(127);




  chassis.moveToPose(0, 15, 0, 1000, { .forwards=false, .maxSpeed=80, .minSpeed=50 });
  chassis.waitUntilDone();

  // global_allow_intake = false;
  ungrab();
  // pneumatic_grabber.set_value(false);

  chassis.moveToPose(0, 33, 0, 1000, { .maxSpeed=80, .minSpeed=50 });
  chassis.waitUntilDone();

  intake.brake();

  chassis.turnToHeading(130, 1000, { .maxSpeed=60, });
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.waitUntilDone();

  chassis.moveToPose(0, -47, chassis.getPose().theta, 1000, { .forwards=false, .maxSpeed=60 });
  chassis.waitUntilDone();


// *
  // *
  // GRABS THE MOGO HERE
  // *
// *


  // pneumatic_grabber.set_value(true);
  pros::delay(200);

  chassis.turnToHeading(-139, 1000, { .maxSpeed=85, .minSpeed=80, });
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.waitUntilDone();

// *
  // *
  // RUN THE INTAKE
  // *
// *

  // global_allow_intake = true;
  intake.move(-127);

  chassis.moveToPose(chassis.getPose().x, 135, chassis.getPose().theta, 5000, { .maxSpeed=49, .minSpeed=40 });
  chassis.waitUntilDone();

  // chassis.turnToHeading(-85, 1000, { .maxSpeed=80 });
  // chassis.waitUntilDone();

  // chassis.setPose(0,0,0);
  // chassis.waitUntilDone();

  // chassis.moveToPose(0, 7, 0, 1000, { .maxSpeed=80 });
  // chassis.waitUntilDone();
  // chassis.turnToHeading(-90, 1000, { .maxSpeed=80 });
  // chassis.waitUntilDone();
  chassis.turnToHeading(-145, 1000, { .maxSpeed=80 });
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.waitUntilDone();

  chassis.moveToPose(0, 110, 0, 1000, { .maxSpeed=90 });
  move_arm_max();
  chassis.waitUntilDone();
  move_arm_max();

  return;

  chassis.setPose(0,0,0);
  chassis.waitUntilDone();
  chassis.turnToHeading(45, 1000, { .maxSpeed=80 });
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.waitUntilDone();

  move_arm_up();

  return;

  move_arm_up();

  chassis.moveToPose(0, 100, 0, 2000, { .forwards=false, .maxSpeed=80, });
}
// SKILLS AUTO
void autonomousA() {
  intake.move(-127);
  pros::delay(700);
  intake.brake();

  chassis.moveToPose(0, 31, 0, 1000, { .maxSpeed=50 });
  chassis.waitUntilDone();

  chassis.turnToHeading(90, 1000);
  chassis.waitUntilDone();

  chassis.setPose(0,0,0);
  chassis.waitUntilDone();

// *
  // *
  // FIRST MOGO PICKED UP
  // *
// *

  chassis.moveToPose(0, -64, 0, 2000, { .forwards=false, .maxSpeed=60 });
  chassis.waitUntilDone();

  chassis.turnToHeading(-90, 1000, { .maxSpeed=90 });
  chassis.waitUntilDone();

  chassis.setPose(0,0,0);
  chassis.waitUntilDone();

  intake.move(-127);

  // FIRST RING PICKUP
  chassis.moveToPose(0, 50, 0, 2000, { .maxSpeed=40 });
  chassis.waitUntilDone();

  chassis.moveToPose(0,44, 0, 100, { .forwards=false, .maxSpeed=60 });

  pros::delay(700);

  intake.brake();

// *
  // *
  // BIG DIAGONAL STARTS
  // *
// *

  chassis.turnToHeading(-26, 1000, { .maxSpeed=50 });
  chassis.waitUntilDone();

  chassis.setPose(0,0,0);
  chassis.waitUntilDone();

  // LONG DISTANCE HERE
  chassis.moveToPose(0, 135, 0, 2000, { .maxSpeed=65 });
  pros::delay(200);
  intake.move(-127);
  chassis.waitUntilDone();
  chassis.moveToPose(0, 120, 0, 1000, { .forwards=false, .maxSpeed=60 });
  chassis.waitUntilDone();


  pros::delay(300);

  // TURN TO THE RING BESIDE THE HIGH STAKE
  chassis.turnToHeading(-153.8, 1000, { .maxSpeed=70 });
  chassis.waitUntilDone();

  // chassis.setPose(0,0,0);
  // chassis.waitUntilDone();

  // chassis.moveToPose(0, 36, 0, 1000, { .maxSpeed=40 });
  // chassis.waitUntilDone();

  // chassis.turnToHeading(90, 1000, { .maxSpeed=60 });
  // chassis.waitUntilDone();

  // move_arm_up();

  // chassis.setPose(0,0,0);
  // intake.move(-127);
  // chassis.moveToPose(0, 60, 0, 2000, { .maxSpeed=35 });
  // chassis.waitUntilDone();

  // chassis.setPose(0,0,0);
  // chassis.moveToPose(0, -6, 0, 1000, { .forwards=false, .maxSpeed=35, });
  // chassis.waitUntilDone();

  // pros::delay(200);
  // intake.brake();

  // move_arm_max();

  // pros::delay(300);

  // move_arm_max();

  // pros::delay(400);

  // move_arm_down();

  // chassis.moveToPose(0, -33, 0, 1000, { .forwards=false, .maxSpeed=60 });
  // chassis.waitUntilDone();

  // chassis.turnToHeading(-90, 1000, { .maxSpeed=80 });
  // chassis.waitUntilDone();



  chassis.setPose(0,0,0);
  chassis.waitUntilDone();

  intake.move(-127);
  chassis.moveToPose(0, 189, 0, 5000, { .maxSpeed=50 });
  chassis.waitUntilDone();


  pros::delay(5000);

  intake.brake();

  return;


  pros::delay(2000);
  intake.set_brake_mode(MOTOR_BRAKE_COAST);
  intake.brake();

  return

  chassis.turnToHeading(52, 1000, { .maxSpeed=60 });
  chassis.waitUntilDone();

  chassis.setPose(0,0,0);
  chassis.waitUntilDone();

  chassis.moveToPose(0, 14, 0, 750, { .maxSpeed=40 });
  chassis.waitUntilDone();

  chassis.setPose(0,0,0);
  chassis.waitUntilDone();

  chassis.moveToPose(0, -7, 0, 1000, { .forwards=false, .maxSpeed=40, });
  chassis.waitUntilDone();

  move_arm_max();
  pros::delay(500);
  chassis.turnToHeading(5, 200);
  chassis.waitUntilDone();
  move_arm_max();

  pros::delay(500);

  ungrab();
  intake.brake();
}


//CURRENT REDDDDD
void autonomous() {

  chassis.moveToPose(0, -97, 0, 1500, { .forwards=false, .maxSpeed=100 });
  chassis.waitUntilDone();
  chassis.turnToHeading(-37, 600, { .maxSpeed=80 });
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.waitUntilDone();
  // chassis.moveToPose(0, -28, chassis.getPose().theta, 1000, { .forwards=false, .maxSpeed=100 });
  chassis.moveToPose(0, -30, chassis.getPose().theta, 700, { .forwards=false, .maxSpeed=55, .minSpeed=45});
  chassis.waitUntilDone();

// *
  // *
  // GRABS THE MOGO HERE
  // *
// *

  // pneumatic_grabber.set_value(true);



  pros::delay(100);
  chassis.moveToPose(0, -20, chassis.getPose().theta, 1000, { .maxSpeed=90, .minSpeed=50 });
  chassis.waitUntilDone();

// *
  // *
  // RUNS THE INTAKE HERE
  // *
// *

  intake.move(-127);
  // global_allow_intake = true;


  chassis.turnToHeading(75, 600, {.maxSpeed=60});
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.waitUntilDone();
  chassis.moveToPose(0, 35, 0, 1000, { .maxSpeed=55 });
  chassis.waitUntilDone();


// *
  // *
  // STOPS THE INTAKE HERE
  // *
// *

  pros::delay(1000);
  // global_allow_intake = false;
  intake.move(127);




  chassis.moveToPose(0, 15, 0, 1000, { .forwards=false, .maxSpeed=80, .minSpeed=50 });
  chassis.waitUntilDone();

  // global_allow_intake = false;
  ungrab();
  // pneumatic_grabber.set_value(false);

  chassis.moveToPose(0, 33, 0, 1000, { .maxSpeed=80, .minSpeed=50 });
  chassis.waitUntilDone();

  intake.brake();

  chassis.turnToHeading(-130, 1000, { .maxSpeed=60, });
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.waitUntilDone();

  chassis.moveToPose(0, -47, chassis.getPose().theta, 1000, { .forwards=false, .maxSpeed=60 });
  chassis.waitUntilDone();


// *
  // *
  // GRABS THE MOGO HERE
  // *
// *


  // pneumatic_grabber.set_value(true);
  pros::delay(200);

  chassis.turnToHeading(139, 1000, { .maxSpeed=85, .minSpeed=80, });
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.waitUntilDone();

// *
  // *
  // RUN THE INTAKE
  // *
// *

  // global_allow_intake = true;
  intake.move(-127);

  chassis.moveToPose(chassis.getPose().x, 135, chassis.getPose().theta, 5000, { .maxSpeed=49, .minSpeed=40 });
  chassis.waitUntilDone();

  // chassis.turnToHeading(-85, 1000, { .maxSpeed=80 });
  // chassis.waitUntilDone();

  // chassis.setPose(0,0,0);
  // chassis.waitUntilDone();

  // chassis.moveToPose(0, 7, 0, 1000, { .maxSpeed=80 });
  // chassis.waitUntilDone();
  // chassis.turnToHeading(-90, 1000, { .maxSpeed=80 });
  // chassis.waitUntilDone();
  chassis.turnToHeading(145, 1000, { .maxSpeed=80 });
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.waitUntilDone();

  chassis.moveToPose(0, 110, 0, 1000, { .maxSpeed=90 });
  move_arm_max();
  chassis.waitUntilDone();
  move_arm_max();

  return;

  chassis.setPose(0,0,0);
  chassis.waitUntilDone();
  chassis.turnToHeading(45, 1000, { .maxSpeed=80 });
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.waitUntilDone();

  move_arm_up();

  return;

  move_arm_up();

  chassis.moveToPose(0, 100, 0, 2000, { .forwards=false, .maxSpeed=80, });
}
// SKILLS AUTO

