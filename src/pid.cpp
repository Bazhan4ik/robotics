



#include "main.h"



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

  double circumference;


}


double ticksPerInch = (2.0 * M_PI) / 360;


extern void PID::drivePID(double target) {

  pros::Motor motor1 (1);
  pros::Motor motor2 (2);
  pros::Motor motor3 (3);
  pros::Motor motor4 (4);
  pros::Motor motor5 (5);
  pros::Motor motor6 (6);


  // define the rotation sensors
  pros::Rotation rotateX (0);
  pros::Rotation rotateY (0);

  // start PID
  PID::enableDrivePID = true;
  PID::targetValue = target;


  PID::kP = 0;
  PID::kD = 0;


  PID::circumference = 2.0 * M_PI;

  motor1.brake();
  motor2.brake();
  motor3.brake();
  motor4.brake();
  motor5.brake();
  motor6.brake();

  // reset rotation positions
  rotateX.reset();
  rotateY.reset();

  while(PID::enableDrivePID) {
    double XRotatePosition = rotateX.get_position();
    double YRotatePosition = rotateY.get_position();

    PID::avgPosition = XRotatePosition * ticksPerInch;

    PID::error = PID::targetValue - PID::avgPosition;

    PID::derivative = PID::error - PID::prevError;

    PID::lateralMotorPower = (PID::error * PID::kP) + (PID::derivative * PID::kD);


    // DO SOME LOGGING


    // CHANGE ALL THE NUMBERS IN THESE IFs
    if(PID::lateralMotorPower > 9) {
      PID::lateralMotorPower = 9;
    }
    if(PID::lateralMotorPower < -9) {
      PID::lateralMotorPower = -9;
    }


    int LPower = PID::lateralMotorPower;
    int RPower = PID::lateralMotorPower;


    // DRIFT CORRECTION
    // if((LRotatePosition - RRotatePosition) < -5) {
    //   RPower = RPower - 1;
    // } else if((LRotatePosition - RRotatePosition) > 5) {
    //   LPower = LPower - 1;
    // }


    // CHECK ALL THE NUMBERS
    if(fabs(PID::error) < 2.0 && motor1.get_voltage() < 6) {
      // STOP ALL THE MOTORS

      break;
    } else {
      // KEEP SPINNING
      motor1.move(PID::lateralMotorPower);
    }


    PID::prevError = PID::error;
  }



  PID::error = 0.0;
  PID::lateralMotorPower = 0.0;
  PID::targetValue = 0.0;
  PID::derivative = 0.0;
}
