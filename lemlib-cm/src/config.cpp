#include <cstdio>
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/distance.hpp"
#include "pros/vision.h"
#include "pros/vision.hpp"


#include "config.h"

#define RED_SIGNATURE 1


// double track_width = 12.375 * 2.54;
double track_width = 12.375;


// double t_wheel_diameter = lemlib::Omniwheel::NEW_2 * 2.54 - 0.24;
// double t_wheel_diameter = lemlib::Omniwheel::NEW_2 - (0.24 / 2.54);
double t_wheel_diameter = lemlib::Omniwheel::NEW_2 - 0.1;
// double wheel_diatemeter = lemlib::Omniwheel::NEW_325 * 2.54;
double wheel_diatemeter = lemlib::Omniwheel::NEW_325;


double horizontal_tw_offset = 0.0;
double vertical_tw_offset = 0.0;


pros::MotorGroup intake_chain ({7});
pros::MotorGroup intake_stage1({13});

pros::MotorGroup lady_brown_arm ({8});

pros::adi::DigitalOut pneumatic_mogo_grabber('B', LOW);
// extension arm to move corner rings
pros::adi::DigitalOut pneumatic_robot_extension('D', LOW);



/**
 * =-
 * SENSORS 
 * =-
 */
pros::Imu imu(18);

pros::Rotation rotation_horizontal(9);
pros::Rotation rotation_vertical(-19);

lemlib::TrackingWheel tw_horizontal(&rotation_horizontal, t_wheel_diameter, horizontal_tw_offset);
lemlib::TrackingWheel tw_vertical(&rotation_vertical, t_wheel_diameter, vertical_tw_offset);

// lemlib::OdomSensors sensors(&tw_vertical, nullptr, &tw_horizontal, nullptr, &imu);
lemlib::OdomSensors sensors(&tw_vertical, nullptr, nullptr, nullptr, &imu);


pros::Rotation rotation_arm(16);
pros::Vision vision_sensor (14);
pros::Distance distance_sensor (12);
/**
 * =-
 * SENSORS END 
 * =-
 */


pros::MotorGroup left_motors({1, -2, 3}, pros::MotorGearset::blue);   // left motors on ports 1, 2, 3
pros::MotorGroup right_motors({-4, 5, -6}, pros::MotorGearset::blue); // right motors on ports 4, 5, 6

lemlib::Drivetrain drivetrain(&left_motors, &right_motors, track_width, wheel_diatemeter, 450, 2);


lemlib::ControllerSettings lateral_controller(15,  // proportional gain (kP)
                                              0,   // integral gain (kI)
                                              20,   // derivative gain (kD)
                                              3,   // anti windup
                                              1.0, // * 2.54   // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              1.5,   // large error range, in inches
                                              200, // large error range timeout, in milliseconds
                                              20   // maximum acceleration (slew)
);
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
pros::Controller master(pros::E_CONTROLLER_MASTER);






pros::vision_signature_s_t RED_SIG = pros::Vision::signature_from_utility(RED_SIGNATURE, 7921, 10923, 9422, -2907, -859, -1883, 2.5, 0);

