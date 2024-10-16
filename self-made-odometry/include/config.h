#include "main.h"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"



pros::IMU imu (18);

pros::MotorGroup left_motors({1, -2, 3}, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees);   // left motors on ports 1, 2, 3
pros::MotorGroup right_motors({-4, 5, -6}, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees); // right motors on ports 4, 5, 6

pros::Rotation rotation_horizontal(-9);
pros::Rotation rotation_vertical(19);