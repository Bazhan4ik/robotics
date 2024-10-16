#include "main.h"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"




namespace PID {
  double kP;
  double kD;

  double avgPosition;
  double error;
  double prevError;
  double derivative;

  double lateralMotorPower;
  bool enableDrivePID;

  double targetValue;


  void drivePID (pros::Imu, pros::MotorGroup, pros::MotorGroup, pros::Rotation, pros::Rotation, double tgt);
}



double tracking_wheel_diameter = 5.10;
double tracking_wheel_radius = tracking_wheel_diameter / 2.0;

double ticks_per_rotation = std::round(360.0 / 0.088);

double cm_per_tick_h = 2.0 * M_PI * tracking_wheel_radius / ticks_per_rotation;
double cm_per_tick_v = 2.0 * M_PI * (tracking_wheel_radius - 0.04) / ticks_per_rotation;


extern void PID::drivePID(pros::Imu imu, pros::MotorGroup leftMotors, pros::MotorGroup rightMotors, pros::Rotation tw_h, pros::Rotation tw_v, double target) {

  // start PID
  PID::enableDrivePID = true;
  PID::targetValue = target;


  PID::kP = 0;
  PID::kD = 0;



  leftMotors.brake();
  rightMotors.brake();


  // reset rotation positions
  tw_h.reset();
  tw_v.reset();
  imu.reset();
  


  while(PID::enableDrivePID) {
    double XRotatePosition = tw_h.get_position();
    double YRotatePosition = tw_v.get_position();

    PID::avgPosition = YRotatePosition * cm_per_tick_v;

    PID::error = PID::targetValue - PID::avgPosition;

    PID::derivative = PID::error - PID::prevError;

    PID::lateralMotorPower = (PID::error * PID::kP) + (PID::derivative * PID::kD);


    // DO SOME LOGGING


    // CHANGE ALL THE NUMBERS IN THESE IFs
    if(PID::lateralMotorPower > 50) {
      PID::lateralMotorPower = 50;
    }
    if(PID::lateralMotorPower < -50) {
      PID::lateralMotorPower = -50;
    }


    int LPower = PID::lateralMotorPower;
    int RPower = PID::lateralMotorPower;


    // DRIFT CORRECTION
    if(imu.get_rotation() > 0.2) {
      RPower = RPower - 1;
    } else if(imu.get_rotation() < -0.2) {
      LPower = LPower - 1;
    }


    // CHECK ALL THE NUMBERS
    if(fabs(PID::error) < 2.0 && leftMotors.get_voltage() < 30) {
      // STOP ALL THE MOTORS
      leftMotors.brake();
      rightMotors.brake();
      break;
    } else {
      // KEEP SPINNING
      leftMotors.move(LPower);
      rightMotors.move(RPower);
    }


    PID::prevError = PID::error;
  }



  PID::error = 0.0;
  PID::lateralMotorPower = 0.0;
  PID::targetValue = 0.0;
  PID::derivative = 0.0;
}
