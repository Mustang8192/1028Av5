#include "main.h"
#include "1028A/init.h"

using namespace _1028A;

void initialize() {
  // robot::underglowInit();
  ui::init();
}

void disabled() {}

void competition_initialize() {}

void autonomous() { robot::auton(); }

void opcontrol() {
  task::start("FlywheelCTRL", driver::FlywheelCTRL);
  task::start("DriveCTRL", driver::DriveCTRL);
  task::start("IntakeCTRL", driver::IntakeCTRL);
  task::start("ExpansionCTRL", driver::ExpansionCTRL);
  task::start("checkBrakeType", driver::checkBrakeType);
  task::start("ModeCTRL", driver::ModeCTRL);
  while (true) {
    pros::delay(200);
  }
}
