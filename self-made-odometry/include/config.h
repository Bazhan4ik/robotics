#include "main.h"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"

extern pros::IMU imu;

extern pros::MotorGroup left_motors;   // left motors on ports 1, 2, 3
extern pros::MotorGroup right_motors; // right motors on ports 4, 5, 6

extern pros::Rotation rotation_horizontal;
extern pros::Rotation rotation_vertical;


extern double tracking_wheel_diameter;
extern double tracking_wheel_radius;

extern double ticks_per_rotation;

extern double cm_per_tick_h;
extern double cm_per_tick_v;