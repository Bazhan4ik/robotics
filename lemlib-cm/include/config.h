#include <cstdio>
#include "lemlib/api.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/distance.hpp"
#include "pros/vision.h"
#include "pros/vision.hpp"

#define RED_SIGNATURE 1


extern double track_width;

extern double t_wheel_diameter;
extern double wheel_diatemeter;

extern double horizontal_tw_offset;
extern double vertical_tw_offset;


extern pros::MotorGroup intake_chain;
extern pros::MotorGroup intake_stage1;

extern pros::MotorGroup lady_brown_arm;

extern pros::adi::DigitalOut pneumatic_mogo_grabber;
extern pros::adi::DigitalOut pneumatic_robot_extension;



extern pros::Imu imu;

extern pros::Rotation rotation_horizontal;
extern pros::Rotation rotation_vertical;

extern lemlib::TrackingWheel tw_horizontal;
extern lemlib::TrackingWheel tw_vertical;

extern lemlib::OdomSensors sensors;


extern pros::Rotation rotation_arm;
extern pros::Vision vision_sensor;
extern pros::Distance distance_sensor;




extern pros::MotorGroup left_motors;   // left motors on ports 1, 2, 3
extern pros::MotorGroup right_motors; // right motors on ports 4, 5, 6

extern lemlib::Drivetrain drivetrain;


// lemlib::OdomSensors sensors(&vertical, nullptr, &horizontal, nullptr, nullptr);

// lateral PID controller
extern lemlib::ControllerSettings lateral_controller;
// angular PID controller
extern lemlib::ControllerSettings angular_controller;



extern lemlib::ExpoDriveCurve steer_curve;
extern lemlib::ExpoDriveCurve throttle_curve;



extern lemlib::Chassis chassis;
extern pros::Controller master;



extern pros::vision_signature_s_t RED_SIG;



