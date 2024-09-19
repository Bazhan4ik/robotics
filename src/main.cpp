#include "main.h"
#include <string>

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
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hiii  :3");

	pros::lcd::register_btn1_cb(on_center_button);



  pros::Motor motor1 (1); // left wheels
  pros::Motor motor2 (2); // left wheels reverse
  pros::Motor motor3 (3); // left wheels
  pros::Motor motor4 (4); // right wheels reverse
  pros::Motor motor5 (5); // right wheels
  pros::Motor motor6 (6); // right wheels reverse



  motor1.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  motor2.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  motor3.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  motor4.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  motor5.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  motor6.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);



}

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
void competition_initialize() {

}

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
	pros::Motor motor1 (1);
  pros::Motor motor2 (2);
  pros::Motor motor3 (3);
	pros::Motor motor4 (4);
  pros::Motor motor5 (5);
  pros::Motor motor6 (6);
  pros::Rotation rotation (19);



  
  rotation.reset();

	pros::lcd::set_text(1, "AUTONOMOUS");


  while (true) {


    pros::lcd::set_text(2, std::to_string(rotation.get_position()));

    if((rotation.get_position() / 100) > 300) {
      motor1.brake();
      motor2.brake();
      motor3.brake();
      motor4.brake();
      motor5.brake();
      motor6.brake();
      continue;
    }

    // motor1.move(100);
    // motor2.move(-100);
    // motor3.move(100);
    // motor4.move(-100);
    // motor5.move(100);
    // motor6.move(-100);

    pros::delay(2);

  }
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
void opcontrol() {
  // controller
  pros::Controller master (CONTROLLER_MASTER);

  // motors used to grab the circles and push them onto the pole
  pros::Motor motor7 (7);
  pros::Motor motor8 (8);


  // left motors
  pros::Motor motor1 (1);
  pros::Motor motor2 (2); // reverse
  pros::Motor motor3 (3);
  // right motors
  pros::Motor motor4 (4); // reverse
  pros::Motor motor5 (5); 
  pros::Motor motor6 (6); // reverse


  // grabber for grabbing the poles
  // default is HIGH pressure, which means that the pneumatics are enabled by default
  pros::ADIDigitalOut preumatic_grabber ('H', HIGH);


  // ?????
  pros::Rotation rotation (19); // rotation sensor




  // vision sensor
  pros::Vision vs (9);

  pros::vision_signature_s_t blue1 = pros::Vision::signature_from_utility(1, -14623, -14289, -14456, 3477, 3843, 3660, 2.5, 0);
  pros::vision_signature_s_t blue2 = pros::Vision::signature_from_utility(2, -3997, -3507, -3752, 10923, 11509, 11216, 2.5, 0);
  pros::vision_signature_s_t blue3 = pros::Vision::signature_from_utility(3, -2475, -2105, -2290, 7669, 8067, 7868, 2.5, 0);


  vs.clear_led();
  vs.set_zero_point(pros::E_VISION_ZERO_CENTER);
  
  vs.set_exposure(120);

  vs.set_auto_white_balance(1);

  vs.set_signature(1, &blue1);
  vs.set_signature(2, &blue2);
  vs.set_signature(3, &blue3);

  pros::vision_color_code_t colorc = vs.create_color_code(1, 2, 3, 0, 0);

  
  int inde = 1;

	while (true) {

    
    pros::lcd::set_text(3, "sig: " + std::to_string(vs.get_signature(1).u_min) + " " + std::to_string(vs.get_signature(1).u_max) + " " + std::to_string(vs.get_signature(1).u_mean));
    pros::lcd::set_text(4, "exposure: " + std::to_string(vs.get_exposure()));


    pros::lcd::set_text(6, "===: " + std::to_string(vs.get_vision().get_by_size(0).x_middle_coord));
    pros::lcd::set_text(5, "===: " + std::to_string(vs.get_vision().get_by_size(1).x_middle_coord));
    pros::lcd::set_text(7, "===: " + std::to_string(vs.get_vision().get_by_size(2).x_middle_coord));

    pros::lcd::set_text(1, "going" + std::to_string(inde));

    inde++;

    pros::delay(500);

    // pros::lcd::set_text(6, "===: " + std::to_string(vs.get_by_sig(0, 1).x_middle_coord));
    // pros::lcd::set_text(5, "===: " + std::to_string(vs.get_by_sig(0, 2).x_middle_coord));
    // pros::lcd::set_text(7, "===: " + std::to_string(vs.get_by_sig(0, 3).x_middle_coord));

    pros::delay(500);


    // L1 is used for disabling the grabber (letting it go)
		bool L1_pressed = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);

		if(L1_pressed) {
			preumatic_grabber.set_value(LOW);
		} else if (!L1_pressed) {
			preumatic_grabber.set_value(HIGH);
		}



		

    // used for grabbing the circles and putting them on the pole
		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			motor7.move(-127);
			motor8.move(127);
			pros:printf("R1 PRESSED");
		} 
    // move them other way
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
			motor7.move(127);
			motor8.move(-127);
		} else {
			motor7.brake();
			motor8.brake();
		}


    // power from left and right joystick
		int leftPower = master.get_analog(ANALOG_LEFT_Y);
		int rightPower = master.get_analog(ANALOG_RIGHT_Y);

		// left wheels 
		if(leftPower) {
			motor1.move(leftPower);
			motor2.move(-leftPower);
			motor3.move(leftPower);
		} else {
			motor1.brake();
			motor2.brake();
			motor3.brake();
		}

		// right wheels 
		if(rightPower) {
      motor4.move(-rightPower);
			motor5.move(rightPower);
			motor6.move(-rightPower);
		} else {
			motor4.brake();
			motor5.brake();
			motor6.brake();		
		}


		pros::delay(20);
	}
}


