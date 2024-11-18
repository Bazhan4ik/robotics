#include "main.h"
#include <string>
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/llemu.hpp"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "pros/vision.h"

#define EXAMPLE_SIG 1




double t_wheel_diameter = lemlib::Omniwheel::NEW_2 * 2.54 - 0.24;
double wheel_diatemeter = lemlib::Omniwheel::NEW_325 * 2.54;

double track_width = 12.375 * 2.54;
double horizontal_tw_offset = 0.0;
// double horizontal_tw_offset = -0.2813 * 2.54;
double vertical_tw_offset = 0.1895 * 2.54;


pros::MotorGroup intake ({7});
pros::MotorGroup motor_arm ({8});

pros::adi::DigitalOut pneumatic_grabber('D', LOW);

pros::Imu imu(18);

pros::Rotation rotation_horizontal(9);
pros::Rotation rotation_vertical(19);

lemlib::TrackingWheel horizontal(&rotation_horizontal, t_wheel_diameter, horizontal_tw_offset);
lemlib::TrackingWheel vertical(&rotation_vertical, t_wheel_diameter, vertical_tw_offset);

pros::MotorGroup left_motors({1, -2, 3}, pros::MotorGearset::blue);   // left motors on ports 1, 2, 3
pros::MotorGroup right_motors({-4, 5, -6}, pros::MotorGearset::blue); // right motors on ports 4, 5, 6

lemlib::Drivetrain drivetrain(&left_motors, &right_motors, track_width, wheel_diatemeter, 450, 2);


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

pros::Vision vision_sensor (15);

pros::vision_signature_s_t RED_SIG = pros::Vision::signature_from_utility(EXAMPLE_SIG, 7921, 10923, 9422, -2907, -859, -1883, 2.5, 0);






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

void opcontrol() {

  pneumatic_grabber.set_value(true);

  while (true) {

    int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

    // move the robot
    chassis.tank(leftY, rightY);


    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      pneumatic_grabber.set_value(false);
    } else {
      pneumatic_grabber.set_value(true);
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

    pros::delay(40);
  }
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
  pneumatic_grabber.set_value(true);
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
  pneumatic_grabber.set_value(false);

  chassis.moveToPose(0, 37, 0, 1000, { .maxSpeed=80, .minSpeed=50 });
  chassis.waitUntilDone();

  chassis.turnToHeading(130, 1000, { .maxSpeed=60, });
  chassis.waitUntilDone();
  chassis.setPose(0,0,0);
  chassis.waitUntilDone();

  chassis.moveToPose(0, -50, chassis.getPose().theta, 1000, { .forwards=false, .maxSpeed=70 });
  chassis.waitUntilDone();

  pneumatic_grabber.set_value(true);
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
}


bool valid_ring(pros::vision_object_s_t ring) {
  return ring.width > 100 || ring.height > 100;
}
void intake_with_sorting() {
  vision_sensor.set_signature(EXAMPLE_SIG, &RED_SIG);

  bool allow_intake = true;
  bool ring_detected = false;

  while(true) {

    if(allow_intake) {
      intake.move(-127);
    } else {
      intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      intake.brake();
      if(ring_detected) {
        pros::Task throw_ring_task([&]() {
          pros::delay(400);
          allow_intake = true;
          ring_detected = false;
        });
      }
    }

    pros::vision_object_s_t ring = vision_sensor.get_by_sig(0, EXAMPLE_SIG);
  
    pros::lcd::print(4, "WH: %d -- %d", ring.width, ring.height);

    // if ring detected
    if(valid_ring(ring)) {
      pros::lcd::print(5, "OBJECT DETECTED");

      ring_detected = true;
    }
    // after ring cannot be seen, remove the ring if existed
    else {
      if(ring_detected) {
        allow_intake = false;
        pros::Task throw_ring_task([&]() {
          pros::delay(100);
        });
      }
      pros::lcd::print(5, "-");
    }
    pros::delay(20);
  }
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

        pros::delay(40);
    }
  });

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