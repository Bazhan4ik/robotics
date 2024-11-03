#include "main.h"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"


#include "PID.h"
#include "config.h"



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


  void drivePID (double tgt);
}




extern void PID::drivePID(double target) {

  // start PID
  PID::enableDrivePID = true;
  PID::targetValue = target;


  // 
  PID::kP = 0;
  PID::kD = 0;



  left_motors.brake();
  right_motors.brake();


  // reset rotation positions
  rotation_horizontal.reset();
  rotation_vertical.reset();
  imu.reset();
  


  while(PID::enableDrivePID) {
    double XRotatePosition = rotation_horizontal.get_position();
    double YRotatePosition = rotation_vertical.get_position();

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
    if(fabs(PID::error) < 0.1 && left_motors.get_voltage() < 30) {
      // STOP ALL THE MOTORS
      left_motors.brake();
      right_motors.brake();
      break;
    } else {
      // KEEP SPINNING
      left_motors.move(LPower);
      right_motors.move(RPower);
    }


    PID::prevError = PID::error;
  }



  PID::error = 0.0;
  PID::lateralMotorPower = 0.0;
  PID::targetValue = 0.0;
  PID::derivative = 0.0;
}
