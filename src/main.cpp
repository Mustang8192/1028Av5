#include "main.h"
#include "1028A/init.h"

using namespace _1028A;

void initialize() {
  ui::init();
  comp::preauton::preMatchChecks();
}

void disabled() {}

void competition_initialize() {}

void autonomous() { comp::auton::auton(); }

void opcontrol() {
  task::start("FlywheelCTRL", comp::driver::FlywheelCTRL);
  task::start("DriveCTRL", comp::driver::DriveCTRL);
  task::start("IntakeCTRL", comp::driver::IntakeCTRL);
  task::start("checkBrakeType", comp::driver::checkBrakeType);
  while (true) {
    pros::delay(200);
  }
}
