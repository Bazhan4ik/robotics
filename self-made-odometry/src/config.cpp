#include "main.h"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"



pros::IMU imu (18);

pros::MotorGroup left_motors({1, -2, 3}, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees);   // left motors on ports 1, 2, 3
pros::MotorGroup right_motors({-4, 5, -6}, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees); // right motors on ports 4, 5, 6

pros::Rotation rotation_horizontal(-9);
pros::Rotation rotation_vertical(19);


double tracking_wheel_diameter = 5.10;
double tracking_wheel_radius = tracking_wheel_diameter / 2.0;

double ticks_per_rotation = std::round(360.0 / 0.088);

double cm_per_tick_h = 2.0 * M_PI * tracking_wheel_radius / ticks_per_rotation;
double cm_per_tick_v = 2.0 * M_PI * (tracking_wheel_radius - 0.04) / ticks_per_rotation;