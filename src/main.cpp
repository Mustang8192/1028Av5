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
  task::start("ExpansionCTRL", comp::driver::ExpansionCTRL);
  task::start("AngleCTRL", comp::driver::AngleCTRL);
  task::start("checkBrakeType", comp::driver::checkBrakeType);
  task::start("ModeCTRL", comp::driver::ModeCTRL);
  while (true) {
    pros::delay(200);
  }
}
