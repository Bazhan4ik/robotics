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
                                              1 * 2.54,   // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              2 * 2.54,   // large error range, in inches
                                              500, // large error range timeout, in milliseconds
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

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize()
{
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void run_intake() {
  while(true) {
    intake.move(127);
    pros::delay(20);
  }
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

void autonomous() {
  // preumatic_grabber.set_value(false);

  // pros::delay(1000);

  chassis.moveToPose(0, -70, 0, 950, { .forwards=false, .maxSpeed=80, });

  chassis.moveToPose(0, -93, 0, 700, { .forwards=false, .maxSpeed=50 });

  chassis.waitUntilDone();

  preumatic_grabber.set_value(true);

  pros::delay(200);

  for(int i = 0; i < 50; i+=1) {
    intake.move(127);
    pros::delay(20);
  }

  intake.brake();

  chassis.turnToHeading(60, 500);

  chassis.waitUntilDone();

  intake.move(127);

  chassis.moveToPose(51, -65, 60, 1000, { .maxSpeed=80 });

  chassis.waitUntilDone();

  // pros::delay(700);

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

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{

  autonomous();

  return;


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
