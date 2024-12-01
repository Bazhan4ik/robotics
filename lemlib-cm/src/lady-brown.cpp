#include "lady-brown.h"
#include "config.h"
#include "pros/llemu.hpp"




bool lady_brown_allowed = false;


void lb_get_ready() {
  int target = 2750; // ticks up 

  double maxSpeed = 35;

  // start PID
  lady_brown_allowed = true;


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


  while(lady_brown_allowed) {
    int ticks = rotation_arm.get_position();

    int error = target - std::abs(ticks);
    int derivative = error - prevError;

    double lateralMotorPower = std::abs((error * kP) + (derivative * kD));
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
      
      lady_brown_arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      lady_brown_arm.brake();

      pros::lcd::print(4, "STOPPED");

      return;
    } else {
      // KEEP SPINNING
      lady_brown_arm.move(lateralMotorPower);
    }


    prevError = error;

    pros::delay(10);
  }
  return;
}
void lb_move_down() {
  lady_brown_allowed = false;
  lady_brown_arm.move(-80);
  pros::delay(300);
  lady_brown_arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  lady_brown_arm.brake();
  return;
}
void lb_move_up() {
  lady_brown_arm.move(90);
  pros::delay(700);
  lady_brown_arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  lady_brown_arm.brake();
  return;
}