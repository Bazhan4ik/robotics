#include "main.h"
#include <string>
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/rtos.hpp"









double t_wheel_diameter = lemlib::Omniwheel::NEW_2 * 2.54 - 0.24;
double wheel_diatemeter = lemlib::Omniwheel::NEW_325 * 2.54;

double track_width = 12.375 * 2.54;
double horizontal_tw_offset = 0.0;
// double horizontal_tw_offset = -0.2813 * 2.54;
double vertical_tw_offset = 0.1895 * 2.54;











pros::MotorGroup intake ({7});
pros::MotorGroup motor_arm ({8});



pros::adi::DigitalOut preumatic_grabber('D', LOW);


pros::Imu imu(18);


pros::MotorGroup left_motors({1, -2, 3}, pros::MotorGearset::blue);   // left motors on ports 1, 2, 3
pros::MotorGroup right_motors({-4, 5, -6}, pros::MotorGearset::blue); // right motors on ports 4, 5, 6

lemlib::Drivetrain drivetrain(&left_motors, &right_motors, track_width, wheel_diatemeter, 450, 2);


pros::Rotation rotation_horizontal(9);
pros::Rotation rotation_vertical(19);

lemlib::TrackingWheel horizontal(&rotation_horizontal, t_wheel_diameter, horizontal_tw_offset);
lemlib::TrackingWheel vertical(&rotation_vertical, t_wheel_diameter, vertical_tw_offset);

// lemlib::OdomSensors sensors(&vertical, nullptr, &horizontal, nullptr, nullptr);
lemlib::OdomSensors sensors(&vertical, nullptr, &horizontal, nullptr, &imu);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10,  // proportional gain (kP)
                                              0,   // integral gain (kI)
                                              20,   // derivative gain (kD)
                                              3,   // anti windup
                                              0.5 * 2.54,   // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              1.5 * 2.54,   // large error range, in inches
                                              700, // large error range timeout, in milliseconds
                                              20   // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2,   // proportional gain (kP)
                                              0,   // integral gain (kI)
                                              10,  // derivative gain (kD)
                                              3,   // anti windup
                                              2,   // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3,   // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0    // maximum acceleration (slew)
);


lemlib::ExpoDriveCurve steer_curve(3, 10, 1.019);
lemlib::ExpoDriveCurve throttle_curve(3, 10, 1.019);


lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors, &throttle_curve, &steer_curve);



pros::Controller master(CONTROLLER_MASTER);


pros::Rotation rotation_arm(16);



/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button()
{
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

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize()
{
  pros::lcd::initialize(); // initialize brain screen
  chassis.calibrate();     // calibrate sensors
  rotation_arm.reset();
  rotation_arm.reset_position();

  // print position to brain screen
  pros::Task screen_task([&]() {
    while (true) {
        // print robot location to the brain screen
        pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
        pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
        pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
        // delay to save resources

        // master.set_text(1, 1, "X: " + std::to_string(chassis.getPose().x));

        pros::delay(40);
    }
  });

}
void disabled() {}
void competition_initialize(){}
void run_intake() {
  while(true) {
    intake.move(50);
    pros::delay(20);
  }
}
void move_arm_up() {

  int target = 2750; // ticks up 

  double maxSpeed = 35;

  // start PID
  bool enableDrivePID = true;


  // constants
  double kP = 1;
  double kD = 5;

  double prevError = 0;



  left_motors.brake();
  right_motors.brake();


  // reset rotation positions
  rotation_arm.reset();
  rotation_arm.reset_position();

  pros::lcd::print(4, "INITIAL VALUE: %d", rotation_arm.get_position());
  
  // pros::Task task1([&]() {
  //   pros::delay(2000);
  //   enableDrivePID = false;
  // });


  while(enableDrivePID) {
    int ticks = rotation_arm.get_position();

    int error = target - abs(ticks);
    int derivative = error - prevError;

    double lateralMotorPower = abs((error * kP) + (derivative * kD));
    pros::lcd::print(2, "ARM SPEED: %f", lateralMotorPower);
    pros::lcd::print(3, "ERROR: %d", error);
    pros::lcd::print(3, "DERIVATIVE: %f", (derivative * kD));


    // DO SOME LOGGING


    // CHANGE ALL THE NUMBERS IN THESE IFs
    if(lateralMotorPower > maxSpeed) {
      lateralMotorPower = maxSpeed;
    }
    if(lateralMotorPower < -maxSpeed) {
      lateralMotorPower = -maxSpeed;
    }


    // CHECK ALL THE NUMBERS
    if(abs(error) < 100) {
      // STOP ALL THE MOTORS
      
      motor_arm.set_brake_mode(MOTOR_BRAKE_HOLD);
      motor_arm.brake();

      pros::lcd::print(4, "STOPPED");

      return;
    } else {
      // KEEP SPINNING
      motor_arm.move(lateralMotorPower);
    }


    prevError = error;

    pros::delay(10);
  }
  return;
}
void move_arm_down() {
  motor_arm.move(-80);
  pros::delay(300);
  motor_arm.set_brake_mode(MOTOR_BRAKE_COAST);
  motor_arm.brake();
  return;
}
void move_arm_max() {
  motor_arm.move(90);
  pros::delay(500);
  motor_arm.set_brake_mode(MOTOR_BRAKE_COAST);
  motor_arm.brake();
  return;
}
void autonomous_main()
{

  preumatic_grabber.set_value(false);

  pros::delay(1000);

  chassis.moveToPose(0, -60, 0, 2000, { .forwards=false,  });
  chassis.moveToPose(0, -103, 0, 2000, { .forwards=false, .maxSpeed=75 });

  pros::delay(2000);

  preumatic_grabber.set_value(true);

  pros::delay(1000);

  chassis.moveToPose(0, -95, 0, 2000);

  pros::delay(500);

  pros::Task task([&]() {
    while (true) {
      intake.move(127);
      pros::delay(20);
    }
  });

  pros::delay(1500);

  task.remove();

  pros::Task task2([&]() {
    while (true) {
      intake.move(-127);
      pros::delay(20);
    }
  });

  pros::delay(500);

  task2.remove();

  intake.brake();

  preumatic_grabber.set_value(false);
  pros::delay(1000);
  preumatic_grabber.set_value(true);
  pros::delay(500);

  chassis.moveToPose(47, chassis.getPose().y, 90, 2000);


  preumatic_grabber.set_value(false);
  pros::delay(1000);


  // chassis.moveToPose(0, -110, 0, 1000, { .forwards=false });
  // chassis.moveToPose(0, -90, 0, 1000, { .forwards=false });

  for(int i = 0; i < 20; i++) {
    intake.move(127);
    pros::delay(20);
  }


  intake.brake();



  // chassis.turnToHeading(90, 1000);
  // chassis.moveToPose(50, 50, 90, 2000);





  // int index = 0;

  // while (true)
  // {
  //   if (index < 100)
  //   {
  //     chassis.tank(50, -50);
  //     index++;
  //   }
  // }
}

void autonomous_other() {
  // preumatic_grabber.set_value(false);

  // pros::delay(1000);

  chassis.moveToPose(0, -73, 0, 950, { .forwards=false, .maxSpeed=90, });

  chassis.moveToPose(0, -89, 0, 700, { .forwards=false, .maxSpeed=55 });

  chassis.moveToPose(0, -100, 0, 300, { .forwards=false, .maxSpeed=55 });

  pros::delay(100);

  preumatic_grabber.set_value(true);

  pros::delay(200);

  for(int i = 0; i < 50; i+=1) {
    intake.move(127);
    pros::delay(20);
  }

  intake.brake();

  chassis.turnToHeading(78, 500);

  chassis.waitUntilDone();


  intake.move(100);

  chassis.moveToPose(52, -91, 78, 10000, { .maxSpeed=80 });

  chassis.waitUntilDone();

  pros::delay(1000);

  intake.brake();

  pros::delay(700);

  return;

  chassis.turnToHeading(151, 500);

  chassis.waitUntilDone();

  chassis.moveToPose(57, -111, 151, 1000, { .maxSpeed=80 });

  chassis.waitUntilDone();

  chassis.moveToPose(57, -9, 0, 3000, { .maxSpeed=80 });

  chassis.waitUntilDone();

  pros::delay(2500);

  intake.brake();


  


  // pros::delay(2000);

}

void autonomous_another() {
  // chassis.moveToPose(0, -73, 0, 950, { .forwards=false, .maxSpeed=90, });

  chassis.moveToPose(0, -96, 0, 1200, { .forwards=false, .maxSpeed=85 });

  chassis.waitUntilDone();

  preumatic_grabber.set_value(true);

  intake.move(127);

  pros::delay(1000);

  intake.brake();

  chassis.turnToHeading(-90, 500);

  chassis.waitUntilDone();
  
  chassis.moveToPose(-20, chassis.getPose().y, chassis.getPose().theta, 300, { .maxSpeed=70, .minSpeed=50 });

  chassis.waitUntilDone();

  intake.move(127);

  chassis.moveToPose(-80, chassis.getPose().y, chassis.getPose().theta, 1000, { .maxSpeed=70 });

  chassis.waitUntilDone();

  pros::delay(1000);
  
  chassis.turnToHeading(-195, 1000, { .maxSpeed=70 });

  chassis.waitUntilDone();

  chassis.moveToPose(chassis.getPose().x + 10, chassis.getPose().y - 25, chassis.getPose().theta, 500, { .maxSpeed=60 });

  chassis.waitUntilDone();

  pros::delay(1000);

  chassis.turnToHeading(chassis.getPose().theta - 24, 300);

  chassis.waitUntilDone();

  chassis.moveToPose(chassis.getPose().x + 6, chassis.getPose().y - 5, chassis.getPose().theta, 500, { .maxSpeed=60 });

  chassis.waitUntilDone();

  pros::delay(6000);

  intake.brake();

}

void autonomous() {
  chassis.moveToPose(0, -97, 0, 1500, { .forwards=false, .maxSpeed=100 });
  chassis.waitUntilDone();
  chassis.turnToHeading(37, 600, { .maxSpeed=80 });
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.waitUntilDone();
  // chassis.moveToPose(0, -28, chassis.getPose().theta, 1000, { .forwards=false, .maxSpeed=100 });
  chassis.moveToPose(0, -37, chassis.getPose().theta, 700, { .forwards=false, .maxSpeed=55, .minSpeed=45});
  chassis.waitUntilDone();
  preumatic_grabber.set_value(true);
  pros::delay(100);
  chassis.moveToPose(0, -20, chassis.getPose().theta, 1000, { .maxSpeed=90, .minSpeed=50 });
  chassis.waitUntilDone();
  intake.move(-120);
  chassis.turnToHeading(-75, 600, {.maxSpeed=60});
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.waitUntilDone();
  chassis.moveToPose(0, 35, 0, 1000, { .maxSpeed=55 });
  chassis.waitUntilDone();

  pros::delay(800);
  intake.move(127);

  chassis.moveToPose(0, 15, 0, 1000, { .forwards=false, .maxSpeed=80, .minSpeed=50 });
  chassis.waitUntilDone();

  intake.brake();
  preumatic_grabber.set_value(false);

  chassis.moveToPose(0, 37, 0, 1000, { .maxSpeed=80, .minSpeed=50 });
  chassis.waitUntilDone();

  chassis.turnToHeading(130, 1000, { .maxSpeed=60, });
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.waitUntilDone();

  chassis.moveToPose(0, -50, chassis.getPose().theta, 1000, { .forwards=false, .maxSpeed=70 });
  chassis.waitUntilDone();

  preumatic_grabber.set_value(true);
  pros::delay(200);

  chassis.turnToHeading(-142, 1000, { .maxSpeed=60 });
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.waitUntilDone();

  intake.move(-127);

  chassis.moveToPose(chassis.getPose().x, 120, chassis.getPose().theta, 5000, { .maxSpeed=55, .minSpeed=50 });
  chassis.waitUntilDone();

  chassis.turnToHeading(-130, 1000, { .maxSpeed=80 });
  chassis.waitUntilDone();
  move_arm_max();
  chassis.setPose(0,0,0);
  chassis.waitUntilDone();

  chassis.moveToPose(0, 95, 0, 1000, { .maxSpeed=50 });
  chassis.waitUntilDone();


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

  return;

  chassis.turnToHeading(120, 1000, { .maxSpeed=80 });
  chassis.waitUntilDone();

  chassis.setPose(0,0,0);
  chassis.waitUntilDone();
  chassis.moveToPose(0, -55, 0, 1000, { .forwards=false, .maxSpeed=80 });
  chassis.waitUntilDone();
  intake.brake();
  preumatic_grabber.set_value(true);

  // preumatic_grabber.set_value(true);

  // intake.move(127);
}


void opcontrol()
{

  // pros::Task screen_task([&]() {
  //   while (true) {
  //       // print robot location to the brain screen
  //       pros::lcd::print(0, "ARM TICKS: %d", rotation_arm.get_position());

  //       pros::delay(40);
  //   }
  // });


  // autonomous();

  // return;


  preumatic_grabber.set_value(true);
  // controller

  // motors used to grab the circles and push them onto the pole
  // pros::Motor motor7(7);
  // pros::Motor motor8(8);

  // // left motors
  // pros::Motor motor1 (1);
  // pros::Motor motor2 (2); // reverse
  // pros::Motor motor3 (3);
  // // right motors
  // pros::Motor motor4 (4); // reverse
  // pros::Motor motor5 (5);
  // pros::Motor motor6 (6); // reverse


  while (true)
  {

    int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    // move the robot
    chassis.tank(leftY, rightY);


    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      preumatic_grabber.set_value(false);
    } else {
      preumatic_grabber.set_value(true);
    }

    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      intake.move(127);
    } else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      intake.move(-127);
    } else {
      intake.brake();
    }


    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
      pros::Task my_task(move_arm_up, "My Task Name");
    }
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
      pros::Task my_task(move_arm_down, "My Task Name");
    }
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
      pros::Task my_task(move_arm_max, "My task name 2");
    }

    // if (index < 100) {
    //   chassis.tank(50, -50);
    //   index++;
    // }

    // pros::lcd::set_text(6, "===: " + std::to_string(vs.get_by_sig(0, 1).x_middle_coord));
    // pros::lcd::set_text(5, "===: " + std::to_string(vs.get_by_sig(0, 2).x_middle_coord));
    // pros::lcd::set_text(7, "===: " + std::to_string(vs.get_by_sig(0, 3).x_middle_coord));

    // if(rotation.get_position() > 1000) {
    //   motor1.move(30);
    // 	motor2.move(-30);
    // 	motor3.move(30);
    //   pros::lcd::set_text(2, "rotating");

    //   pros::delay(1000);
    // } else {
    //   motor1.brake();
    // 	motor2.brake();
    // 	motor3.brake();
    // }

    // pros::lcd::set_text(1, std::to_string(rotation.get_position()));

    // L1 is used for disabling the grabber (letting it go)
    // bool L1_pressed = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);

    // if (L1_pressed)
    // {
    //   preumatic_grabber.set_value(LOW);
    // }
    // else if (!L1_pressed)
    // {
    //   preumatic_grabber.set_value(HIGH);
    // }

    // // used for grabbing the circles and putting them on the pole
    // if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
    // {
    //   motor7.move(-127);
    //   motor8.move(127);
    // pros:
    //   printf("R1 PRESSED");
    // }
    // // move them other way
    // else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
    // {
    //   motor7.move(127);
    //   motor8.move(-127);
    // }
    // else
    // {
    //   motor7.brake();
    //   motor8.brake();
    // }

    // power from left and right joystick

    // left wheels
    // if(leftPower) {
    // 	motor1.move(leftPower);
    // 	motor2.move(-leftPower);
    // 	motor3.move(leftPower);
    // } else {
    // 	motor1.brake();
    // 	motor2.brake();
    // 	motor3.brake();
    // }

    // // right wheels
    // if(rightPower) {
    //   motor4.move(-rightPower);
    // 	motor5.move(rightPower);
    // 	motor6.move(-rightPower);
    // } else {
    // 	motor4.brake();
    // 	motor5.brake();
    // 	motor6.brake();
    // }

    pros::delay(20);
  }
}

void move_to_top() {
  // motor set speed about 60-80
  // check rotation sensor for when the arm rotated enough for the ring to go in
  // stop the arm
  // wait until a ring is inside
  // move the ring

  // arm.move(60);
}


