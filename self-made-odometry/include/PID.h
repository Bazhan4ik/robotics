
namespace PID {
  extern double kP;
  extern double kD;

  extern double avgPosition;
  extern double error;
  extern double prevError;
  extern double derivative;

  extern double lateralMotorPower;
  extern bool enableDrivePID;

  extern double targetValue;


  extern void drivePID (double tgt);
}