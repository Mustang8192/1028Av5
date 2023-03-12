#include "1028A/init.h"
#include "1028A/robot.h"

void _1028A::robot::auton() {
  pros::Inertial.set_rotation(0);
  autonSelect = 2;
  if (autonSelect == 1) {

  } else if (autonSelect == 2) {

  } else if (autonSelect == 3) {

  } else if (autonSelect == 8) {

  } else if (autonSelect == 9) {
  }
}