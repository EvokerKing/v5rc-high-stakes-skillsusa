#include "../include/main.h"
#include <cmath>
#include <cstring>
#include <string>
#include "lemlib/api.hpp"

using namespace std;

// initialize robot
pros::MotorGroup left_motors({ -1, 2, -3 }, pros::MotorGearset::green); // setup a variable to store the controls for the left side drivetrain motors
pros::MotorGroup right_motors({ 4, -5, 6 }, pros::MotorGearset::green); // setup a variable to store the controls for the right side drivetrain motors
lemlib::Drivetrain drivetrain( // create a drivetrain object that declares the following:
	&left_motors, // the left motor group
	&right_motors, // the right motor group
	12.75, // track width in inches
	lemlib::Omniwheel::NEW_4, // wheel type and diameter
	320, // calculated drivetrain RPM
	4 //TODO: figure out horizontal drift, 2 is recommended for all omni, 8 is recommended for all traction
	//         higher value means faster but overshoots more on turns
);
pros::Imu imu(7); // setup a variable to store the inertial sensor
lemlib::OdomSensors sensors( // create an odom object that declares the following:
	nullptr, // no tracking wheels, same for all nullptrs
	nullptr,
	nullptr,
	nullptr,
	&imu // inertial sensor
);
lemlib::ControllerSettings lateral_controller(
	20, // proportional gain (kP)
	0, // integral gain (kI)
	4, // derivative gain (kD)
	3, // anti windup
	1, // small error range in inches
	100, // small error range timeout in milliseconds
	3, // large error range in inches
	500, // large error range timeout in milliseconds
	70 // maximum acceleration (slew)
);
lemlib::ControllerSettings angular_controller(
	2, // proportional gain (kP)
	0, // integral gain (kI)
	10, // derivative gain (kD)
	6, // anti windup
	1, // small error range in inches
	100, // small error range timeout in milliseconds
	3, // large error range in inches
	500, // large error range timeout in milliseconds
	0 // maximum acceleration (slew)
);
lemlib::Chassis chassis( // create a chassis object that declares the following:
	drivetrain, // the drivetrain controls
	lateral_controller, // pid lateral information
	angular_controller, // pid angular information
	sensors // drivetrain sensors
);
pros::Motor conveyor(-10); // set the motor for the conveyor to port 10 reversed
pros::adi::DigitalOut clampPort(1); // set the pneumatics solenoid controlling the clamp

// auton replay stuff
vector<char*> instr; // create the instr variable as a list of char*s
// rest is in initialize

void initialize() {
	std::printf("debug\n"); //GOOD
	pros::lcd::initialize(); // initialize the robot screen
	std::printf("debug\n"); //GOOD
	chassis.calibrate(); // calibrate the sensors
	std::printf("debug\n"); //GOOD
	FILE* file = fopen("/usd/recording.txt", "r"); // open the saved auton recording file
	std::printf("debug\n"); //GOOD
	char buf[200000]; // create a buffer variable for the file contents
	fread(buf, 1, 200000, file); // read the file and add the contents to the buf variable
	std::printf("debug\n"); //BAD
	std::printf("%s\n", buf);
	char* add = strtok(buf, "\n"); // get first index of split strings
	std::printf("debug\n");
	do { // while loop but it will run at least once
		instr.push_back(add); // add the char* to the instr list
		add = strtok(NULL, "\n"); // update add variable to be the next index of the split strings
	} while (add); // if theres no value for add, stop adding variables
	std::printf("debug\n");
}

void autonomous() {
	bool conveyorMoving = false; // variable to track if the conveyor is actively moving. used for checks when no button is pressed but power draw is low
	bool clamped = false; // variable to track if the clamp is currently down. used to allow both actions to be mapped to 1 button
	bool last_clamped = false; // variable to track if the clamp was activated or deactivated on the last cycle. used to prevent the clamp going up and down too quickly on accident

	chassis.setPose(0, 0, 0);

	for (int i = 0; i <= instr.size(); i++) { // for each instruction in the instr variable
		// print each:
		pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
		pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
		pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
		pros::lcd::print(3, "Time: %u", pros::millis()); // screen time info

		char* current = instr[i]; // make it into a new variable for easier use
		string currentStr = current; // cast it to a string

		char delimiter = ':'; // has to be a pointer so make it a variable
		int xPos = atoi(strtok(current, &delimiter)); // get x
		int yPos = atoi(strtok(NULL, &delimiter)); // get y
		int theta = atoi(strtok(NULL, &delimiter)); // get heading
		int time = atoi(strtok(NULL, &delimiter)); // get ideal time

		// move to position
		chassis.moveToPose(xPos, yPos, theta, time);

		int a, b, r1, l1; // get values for buttons a, b, r1, and l1
		if (currentStr.find("a") != string::npos) { a = 1; } else { a = 0; } // every button is checked by seeing if the letter is anywhere in the current cycle string since everything but the buttons are numbers and symbols anyway
		if (currentStr.find("b") != string::npos) { b = 1; } else { b = 0; }
		if (currentStr.find("r") != string::npos) { r1 = 1; } else { r1 = 0; }
		if (currentStr.find("l") != string::npos) { l1 = 1; } else { l1 = 0; }
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
        int x; // check if x is pressed
        if (currentStr.find("x") != string::npos) { x = 1; } else { x = 0; }
		if (x == 1) { // if x is pressed
          	// enter another logic flow
			if (last_clamped) {} /* prevent flow from continuing if it x was pressed last cycle */ else if (clamped) { // if it is already clamped
            	clampPort.set_value(false); // unclamp it
                clamped = false; // and update variables
            } else { // if it isn't already clamped
            	clampPort.set_value(true); // clamp it
                clamped = true; // and update variables
            }
			last_clamped = true; // update variables at end to say x was pressed this (which will be last) cycle
        } else { // or if x isn't pressed
			last_clamped = false; // update variables to reflect this for next cycle
		}

		// delay until movement is complete
		// chassis.waitUntilDone();
	}
}

// driver control function
void opcontrol() {
	autonomous();
}