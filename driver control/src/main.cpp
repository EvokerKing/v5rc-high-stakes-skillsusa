#include "main.h"
#include <cmath>

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
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
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER); // the object for the controller to get inputs
	pros::MotorGroup left_mg({1, -2, 3});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	pros::MotorGroup right_mg({-4, 5, -6});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
	pros::Motor conveyor(-10); // motor for the conveyor belt
    pros::adi::DigitalOut clamp(1); // pneumatics solenoid controlling the clamp
    pros::Motor arm(9); // motor for the arm (lady brown mech)
    pros::Rotation rotation(7); // rotation sensor to get location of the arm
    pros::Optical color(16); // color sensor, ended up not using it but the light looks cool so its still in the code

    color.set_led_pwm(100); // turn on the color sensor light

    rotation.reset_position(); // reset rotation sensor position to 0 to make everything relative to starting position
    const int ideal_angle = 1500; // 15 degrees

	bool conveyorMoving = false; // variable to track if the conveyor is actively moving. used for checks when no button is pressed but power draw is low
    bool clamped = false; // variable to track if the clamp is currently down. used to allow both actions to be mapped to 1 button
	bool last_clamped = false; // variable to track if the clamp was activated or deactivated on the last cycle. used to prevent the clamp going up and down too quickly on accident

	while (true) { // forever loop that does a cycle each 20ms and updates each cycle
		pros::lcd::print(0, "left %d right %d", master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_X));  // prints the status of the joysticks
		pros::lcd::print(1, "rotational %d", rotation.get_position()); // prints the current rotation according to the rotation sensor for debugging purposes

		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y) * -1;    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_mg.move(dir - turn);                      // Sets left motor voltage
		right_mg.move(dir + turn);                     // Sets right motor voltage

        // checks if buttons a, b, r1, or l1 are being pressed and stores them in corresponding variables
		int a = master.get_digital(DIGITAL_A); // start button
		int b = master.get_digital(DIGITAL_B); // stop button
		int r1 = master.get_digital(DIGITAL_R1); // slow move button
		int l1 = master.get_digital(DIGITAL_L1); // slow reverse button
        // control flow logic for different controls regarding the conveyor
		if (b == 1 && conveyor.get_power() > 0.1) { // if the b button is pressed and the conveyor is being powered NOTE: THIS TAKES MASSIVE PRIORITY OVER ALL CONTROLS
			conveyor.brake(); // then brake the conveyor motor
			conveyorMoving = false; // and update variables
		} else if (a == 1) { // or if the a button is pressed
			conveyor.move(127); // begin moving the conveyor at full speed
			conveyorMoving = true; // and update variables
		} else if (r1 == 1) { // or if the r1 button is pressed
			conveyor.move_voltage(9000); // move at 9v out of 12v to be at a slower pace for fixing issues mid run
			conveyorMoving = false; // and update variables
		} else if (l1 == 1) { // or if l1 is pressed
			conveyor.move_voltage(-9000); // move reverse at 9v out of 12v to fix issues mid run
			conveyorMoving = false; // and update variables
		} else if (abs(conveyor.get_current_draw()) <= 5000 && !conveyorMoving) { // and finally if the conveyor power draw is lower than 5v and the conveyor isnt supposed to be moving
			conveyor.brake(); // then brake the conveyor motor
            // note this is mainly to prevent issues with the conveyor not stopping after l1 or r1 is pressed or not stopping for the main a button
		}

        // checks if the x button is pressed
        int x = master.get_digital(DIGITAL_X); // clamp interact button
        if (x == 1) { // if x is pressed
          	// enter another logic flow
			if (last_clamped) {} /* prevent flow from continuing if it x was pressed last cycle */ else if (clamped) { // if it is already clamped
            	clamp.set_value(false); // unclamp it
                clamped = false; // and update variables
            } else { // if it isn't already clamped
            	clamp.set_value(true); // clamp it
                clamped = true; // and update variables
            }
			last_clamped = true; // update variables at end to say x was pressed this (which will be last) cycle
        } else { // or if x isn't pressed
			last_clamped = false; // update variables to reflect this for next cycle
		}

        // checks if y, l2, or r2 are pressed
        int y = master.get_digital(DIGITAL_Y); // prime button
        int l2 = master.get_digital(DIGITAL_L2); // reverse button
        int r2 = master.get_digital(DIGITAL_R2); // forward button
        if (y == 1) { // if y is pressed
			const int current_angle = rotation.get_position(); // get the current angle of the arm
			if (current_angle != ideal_angle) { // and check to make sure it is not already at the ideal angle (not possible btw)
				arm.move_relative(ideal_angle + current_angle, 100); // and then move it the proper relative angle to move toward the ideal angle set earlier
			}
        } else if (l2 == 1) { // or if l2 is pressed
			arm.move(-30); // reverse the arm at 30/127 speed
        } else if (r2 == 1) { // or if r2 is pressed
			arm.move(30); // move the arm forward at 30/127 speed
        } else { // if none are pressed
			arm.move(0); // stop the arm from moving
			arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD); // set the brake mode to hold
			arm.brake(); // and brake
            // the holding brakes should prevent the arm from falling from the force of gravity if in the air
		}

		pros::delay(20);                               // Run for 20 ms then update
	}
}