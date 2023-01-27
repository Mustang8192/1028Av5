#include "main.h"
#include "1028A/init.h"

using namespace _1028A;
using namespace okapi;

void initialize() {
	ui::init();
}

void disabled() {
}

void competition_initialize() {
	//robot::preMatchCheck();
}

void autonomous() {
	robot::auton();
}

void opcontrol() {
	task::start("FlywheelCTRL", driver::FlywheelCTRL);
	task::start("DriveCTRL", driver::DriveCTRL);
	task::start("IntakeCTRL", driver::IntakeCTRL);
	task::start("ExpansionCTRL", driver::ExpansionCTRL);
	task::start("checkBrakeType", driver::checkBrakeType);
	task::start("ModeCTRL", driver::ModeCTRL);
	while (true) {
		
		pros::delay(20);
	}
}
