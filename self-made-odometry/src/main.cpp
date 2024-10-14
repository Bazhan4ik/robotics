#include "main.h"
#include "pros/imu.hpp"
#include "pros/rtos.hpp"
#include <cmath>






pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::IMU imu (18);

pros::MotorGroup left_motors({1, -2, 3}, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees);   // left motors on ports 1, 2, 3
pros::MotorGroup right_motors({-4, 5, -6}, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees); // right motors on ports 4, 5, 6

pros::Rotation rotation_horizontal(-9);
pros::Rotation rotation_vertical(19);




double tracking_wheel_diameter = 5.10;
double tracking_wheel_radius = tracking_wheel_diameter / 2.0;

double ticks_per_rotation = std::round(360.0 / 0.088);

double cm_per_tick = 2.0 * M_PI * tracking_wheel_radius / ticks_per_rotation;




void opcontrol() {
	while (true) {
    pros::delay(20);
	}
}




void initialize() {
	pros::lcd::initialize();

  rotation_horizontal.reset_position();
  rotation_vertical.reset_position();
  imu.reset();
  imu.set_heading(0.0);
  imu.set_rotation(0.0);


  double x_global = 0.0;
  double y_global = 0.0;


  double global_horizontal = 0.0;
  double global_vertical = 0.0;
  double global_angle = 0.0;


  while(true) {

    if(imu.get_heading() == infinity()) {
      pros::lcd::print(0, "ANGLE INFINITE");
      pros::lcd::print(1, "%f", imu.get_heading());
      pros::delay(20);
      continue;
    }


    // left_motors.move(30);
    // right_motors.move(-30);

    global_angle = imu.get_heading() * M_PI / 180.0;




    double vertical_position = rotation_vertical.get_position() / 100.0 / 0.088 * cm_per_tick;
    double horizontal_position = rotation_horizontal.get_position() / 100.0 / 0.088 * cm_per_tick;

    double theta = imu.get_rotation() * M_PI / 180.0;

    if(vertical_position) {
    // if(vertical_position || horizontal_position) {

      double hypotenuse;
      double hypotenuse2;
      double half_angle;

      if(theta == 0.0) {

        hypotenuse = vertical_position;
        hypotenuse2 = horizontal_position;

        half_angle = 0.0;
      } else {
        half_angle = theta / 2.0;

        hypotenuse = 2.0 * sin(half_angle) * (vertical_position / theta - 0.6);
        hypotenuse2 = 2.0 * sin(half_angle) * (horizontal_position / theta - 0.8); // horizontal tracking wheel offset is 0.7... forward-backward
      }

      // y_global += hypotenuse * cos(global_angle + half_angle);
      // x_global += hypotenuse * sin(global_angle + half_angle);

      y_global += hypotenuse2 * sin(-(global_angle + half_angle));
      x_global += hypotenuse2 * cos(-(global_angle + half_angle));



      global_horizontal += horizontal_position;
      global_vertical += vertical_position;


    }





    // pros::lcd::print(3, "ROTATION: %f", imu.get_rotation());
    pros::lcd::print(0, "Y: %f", y_global);
    pros::lcd::print(1, "X: %f", x_global);
    pros::lcd::print(2, "ANGLE: %frad  -  %fdeg", global_angle, imu.get_heading());


    rotation_horizontal.reset_position();
    rotation_vertical.reset_position();
    imu.set_rotation(0.0);

    // if(179.5 < imu.get_heading() && imu.get_heading() < 180.5) {
    //   left_motors.brake();
    //   right_motors.brake();
    //   break;
    // }

    pros::delay(10);
  }
}






/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
