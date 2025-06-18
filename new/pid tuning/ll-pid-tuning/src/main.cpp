#include "../include/main.h"
#include <cmath>
#include "lemlib/api.hpp"

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
	8, // derivative gain (kD)
	3, // anti windup
	1, // small error range in inches
	100, // small error range timeout in milliseconds
	3, // large error range in inches
	500, // large error range timeout in milliseconds
	70 // maximum acceleration (slew)
);
lemlib::ControllerSettings angular_controller(
	5, // proportional gain (kP)
	0, // integral gain (kI)
	20, // derivative gain (kD)
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
pros::adi::DigitalOut clamp(1); // set the pneumatics solenoid controlling the clamp

void initialize() {
	pros::lcd::initialize(); // initialize the robot screen
	chassis.calibrate(); // calibrate the sensors
}

void autonomous() {}

// driver control function
void opcontrol() {
	chassis.setPose(0, 0, 0);
	chassis.moveToPose(0, 48, 0, 100000);
}