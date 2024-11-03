#include "main.h"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"
#include <cmath>

#include "PID.h"







void opcontrol() {
	while (true) {
    pros::delay(20);
	}
}




void initialize() {
	pros::lcd::initialize();
  // rotation_horizontal.reset_position();
  // rotation_vertical.reset_position();
  // imu.reset();
  // imu.set_rotation(0.0);
  // imu.set_heading(0.0);






  return;







  // while (true) {
  //   if(imu.get_heading() == infinity()) {
  //     pros::lcd::print(0, "ANGLE INFINITE");
  //     pros::lcd::print(1, "%f", imu.get_heading());
  //     pros::delay(20);
  //     continue;
  //   }

  //   break;
  // }




  double x_global = 0.0;
  double y_global = 0.0;
  double global_angle = 0.0;


  // double global_horizontal = 0.0;
  // double global_vertical = 0.0;


  // while(true) {
  //   global_angle = imu.get_heading() * M_PI / 180.0;




  //   double vertical_position = rotation_vertical.get_position() / 100.0 / 0.088 * cm_per_tick_v;
  //   double horizontal_position = rotation_horizontal.get_position() / 100.0 / 0.088 * cm_per_tick_h;

  //   double theta = imu.get_rotation() * M_PI / 180.0;

  //   // if(vertical_position) {
  //   if(vertical_position || horizontal_position) {

  //     double hypotenuse;
  //     double hypotenuse2;
  //     double half_angle;

  //     if(theta == 0.0 || (theta < 0.0004 && theta > -6.279185)) {
  //     // if(theta == 0.0) {

  //       hypotenuse = vertical_position;
  //       hypotenuse2 = horizontal_position;

  //       half_angle = 0.0;
  //     } else {
  //       half_angle = theta / 2.0;

  //       double radius = vertical_position / theta + 0.48133;

  //       double Dv = sqrt(2 * radius * (1 - cos(theta)));

  //       // hypotenuse = 2.0 * sin(half_angle) * (vertical_position / theta + 0.48133);
  //       // hypotenuse = 
  //       // hypotenuse = 2.0 * (2 * (vertical_position / theta + 0.48133) * sqrt((1.0 - cos(theta))/2.0));
  //       // hypotenuse2 = 2.0 * sin(half_angle) * (horizontal_position / theta); // horizontal tracking wheel offset is 0.7... forward-backward
  //       // hypotenuse2 = 2.0 * (2 * (horizontal_position / theta) * sqrt((1.0 - cos(theta))/2.0));
  //     }

  //     y_global += hypotenuse * cos(global_angle + half_angle);
  //     x_global += hypotenuse * sin(global_angle + half_angle);

  //     y_global += hypotenuse2 * sin(global_angle + half_angle);
  //     x_global += hypotenuse2 * cos(global_angle + half_angle);



  //     // global_horizontal += horizontal_position;
  //     // global_vertical += vertical_position;
  //   }





  //   // pros::lcd::print(3, "ROTATION: %f", imu.get_rotation());
  //   pros::lcd::print(0, "Y: %f", y_global);
  //   pros::lcd::print(1, "X: %f", x_global);
  //   pros::lcd::print(2, "ANGLE: %frad  -  %fdeg", global_angle, imu.get_heading());


  //   rotation_horizontal.reset_position();
  //   rotation_vertical.reset_position();
  //   imu.set_rotation(0.0);

  //   // if(179.5 < imu.get_heading() && imu.get_heading() < 180.5) {
  //   //   left_motors.brake();
  //   //   right_motors.brake();
  //   //   break;
  //   // }

  //   pros::delay(10);
  // }
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
void autonomous() {
  PID::drivePID(48.0 * 2.54);
}

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
