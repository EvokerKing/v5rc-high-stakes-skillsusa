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
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::MotorGroup left_mg({1, -2, 3});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	pros::MotorGroup right_mg({-4, 5, -6});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
	pros::Motor conveyor(-10);
    pros::adi::DigitalOut clamp(1);
    pros::Motor arm(9);
    pros::Rotation rotation(7);
    pros::Optical color(16);

    color.set_led_pwm(100);

    rotation.reset_position();
    const int ideal_angle = 1500; // 15 degrees

	bool conveyorMoving = false;
    bool clamped = false;
	bool last_clamped = false;
    bool throw_blues = true;

	while (true) {
		pros::lcd::print(0, "left %d right %d", master.get_analog(ANALOG_LEFT_Y),
		                 master.get_analog(ANALOG_RIGHT_X));  // Prints status of the emulated screen LCDs
		pros::lcd::print(1, "rotational %d", rotation.get_position());

		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y) * -1;    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_mg.move(dir - turn);                      // Sets left motor voltage
		right_mg.move(dir + turn);                     // Sets right motor voltage

		int a = master.get_digital(DIGITAL_A);
		int b = master.get_digital(DIGITAL_B);
		int r1 = master.get_digital(DIGITAL_R1);
		int l1 = master.get_digital(DIGITAL_L1);
		if (b == 1 && conveyor.get_power() > 0.1) {
			conveyor.brake();
			conveyorMoving = false;
		} else if (a == 1) {
			conveyor.move(127);
			conveyorMoving = true;
		} else if (r1 == 1) {
			conveyor.move_voltage(9000);
			conveyorMoving = false;
		} else if (l1 == 1) {
			conveyor.move_voltage(-9000);
			conveyorMoving = false;
		} else if (abs(conveyor.get_current_draw()) <= 5000 && !conveyorMoving) {
			conveyor.brake();
		}

        int x = master.get_digital(DIGITAL_X);
        if (x == 1) {
			if (last_clamped) {} else if (clamped) {
            	clamp.set_value(false);
                clamped = false;
            } else {
            	clamp.set_value(true);
                clamped = true;
            }
			last_clamped = true;
        } else {
			last_clamped = false;
		}

        int y = master.get_digital(DIGITAL_Y);
        int l2 = master.get_digital(DIGITAL_L2);
        int r2 = master.get_digital(DIGITAL_R2);
        if (y == 1) {
			const int current_angle = rotation.get_position();
			if (current_angle != ideal_angle) {
				arm.move_relative(ideal_angle + current_angle, 100);
			}
        } else if (l2 == 1) {
			arm.move(-30);
        } else if (r2 == 1) {
			arm.move(30);
        } else {
			arm.move(0);
			arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
			arm.brake();
		}

		pros::delay(20);                               // Run for 20 ms then update
	}
}