#include "main.h"
#include <string>

using namespace std;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	autonomous();
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
void autonomous() {
	pros::MotorGroup left_mg({1, -2, 3});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	pros::MotorGroup right_mg({-4, 5, -6});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6
	pros::Motor conveyor(-7);
    pros::adi::AnalogOut clamp(1);

	bool conveyorMoving = false;
    bool clamped = false;

	FILE* file = fopen("/usd/recording.txt", "r");
	if (file == NULL) {return;}
	char buf[100000];
	fread(buf, 1, 100000, file);

	int time = 0;

	vector<char*> instr;
	char* add = strtok(buf, "\n");
	do {
		instr.push_back(add);
		add = strtok(NULL, "\n");
	} while (add);

	for (int i = 0; i <= instr.size(); i++) {
		char* current = instr[i];
        string currentStr = current;

		char delimiter = ':';
		char* token = strtok(current, &delimiter);
		int dir = atoi(token);    // Gets amount forward/backward from left joystick
		token = strtok(NULL, &delimiter);
		int turn = atoi(token);  // Gets the turn left/right from right joystick
		left_mg.move(dir - turn);                      // Sets left motor voltage
		right_mg.move(dir + turn);                     // Sets right motor voltage
		int a, b, r1, l1;
		if (currentStr.find("a") != string::npos) { a = 1; } else { a = 0; }
		if (currentStr.find("b") != string::npos) { b = 1; } else { b = 0; }
		if (currentStr.find("r") != string::npos) { r1 = 1; } else { r1 = 0; }
		if (currentStr.find("l") != string::npos) { l1 = 1; } else { l1 = 0; }
		if (b == 1 && conveyor.get_power() > 0.1) {
			conveyor.brake();
			conveyorMoving = false;
		} else if (a == 1) {
			conveyor.move(127);
			conveyorMoving = true;
		} else if (r1 == 1) {
			conveyor.move_voltage(6000);
			conveyorMoving = false;
		} else if (l1 == 1) {
			conveyor.move_voltage(-6000);
			conveyorMoving = false;
		} else if (abs(conveyor.get_current_draw()) <= 6000 && !conveyorMoving) {
			conveyor.brake();
		}
        int x;
        if (currentStr.find("x") != string::npos) { x = 1; } else { x = 0; }
		if (x == 1) {
			if (clamped) {
				clamp.set_value(false);
				clamped = false;
			} else {
				clamp.set_value(true);
				clamped = true;
			}
		}
		pros::delay(20);                               // Run for 20 ms then update
		time += 20;
	}

	pros::lcd::print(1, "DONE");
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
	
}