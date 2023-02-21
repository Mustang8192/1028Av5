#include "1028A/init.h"
#include "pros/rtos.hpp"

/**
 * @brief
 *  This function checks the brake type and changes it if it is not set right
 * during Driver Control
 */

void _1028A::driver::checkBrakeType(void *ptr) {
  while (1) {
    if ((pros::LeftFront.get_brake_mode() != pros::E_MOTOR_BRAKE_BRAKE) or
        (pros::LeftMid.get_brake_mode() != pros::E_MOTOR_BRAKE_BRAKE) or
        (pros::LeftBack.get_brake_mode() != pros::E_MOTOR_BRAKE_BRAKE) or
        (pros::RightFront.get_brake_mode() != pros::E_MOTOR_BRAKE_BRAKE) or
        (pros::RightMid.get_brake_mode() != pros::E_MOTOR_BRAKE_BRAKE) or
        (pros::RightBack.get_brake_mode() != pros::E_MOTOR_BRAKE_BRAKE)) {
      pros::LeftFront.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
      pros::LeftMid.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
      pros::LeftBack.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
      pros::RightFront.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
      pros::RightMid.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
      pros::RightBack.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    }
    pros::delay(3000);
  }
}

/**
 * @brief
 *  This function is used to control the flywheel during Driver Control
 */

void _1028A::driver::FlywheelCTRL(void *ptr) {
  FWcontinueTask = 0;
  while (1) {
    if (flywheelMode == 1) {

      if (pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        flywheelstate = 1;
      } else if (pros::mainController.get_digital(
                     pros::E_CONTROLLER_DIGITAL_A)) {
        flywheelstate = 2;
      } else if (pros::mainController.get_digital(
                     pros::E_CONTROLLER_DIGITAL_B)) {
        flywheelstate = 3;
      } else if (pros::mainController.get_digital(
                     pros::E_CONTROLLER_DIGITAL_R2)) {
        flywheelstate = 0;
      }

      if (flywheelstate == 0) {
        pros::FlyWheel.brake();
      } else if (flywheelstate == 1) {
        flywheel::startFlywheel(127);
      } else if (flywheelstate == 2 && !pros::mainController.get_digital(
                                           pros::E_CONTROLLER_DIGITAL_L1)) {
        flywheel::startFlywheel(80);
      } else if (flywheelstate == 3 && !pros::mainController.get_digital(
                                           pros::E_CONTROLLER_DIGITAL_L1)) {
        flywheel::startFlywheel(95);
      }
    } else if (flywheelMode == 2) {
      if (pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        flywheelstate = 1;
      } else if (pros::mainController.get_digital(
                     pros::E_CONTROLLER_DIGITAL_A)) {
        flywheelstate = 2;
      } else if (pros::mainController.get_digital(
                     pros::E_CONTROLLER_DIGITAL_B)) {
        flywheelstate = 3;
      } else if (pros::mainController.get_digital(
                     pros::E_CONTROLLER_DIGITAL_R2)) {
        flywheelstate = 0;
      }

      if (flywheelstate == 0) {
        pros::FlyWheel.brake();
      } else if (flywheelstate == 1) {
        flywheel::startFlywheel(127);
      } else if (flywheelstate == 2 && !pros::mainController.get_digital(
                                           pros::E_CONTROLLER_DIGITAL_L1)) {
        flywheel::startFlywheel(80);
      } else if (flywheelstate == 3 && !pros::mainController.get_digital(
                                           pros::E_CONTROLLER_DIGITAL_L1)) {
        flywheel::startFlywheel(95);
      }
    }
    pros::delay(5);
  }
}

/**
 * @brief
 *  This function is used to control the Drive train during Driver Control
 */

void _1028A::driver::DriveCTRL(void *ptr) {
  while (1) {
    double power =
        pros::mainController.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    double turn =
        pros::mainController.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    pros::LeftFront.move((-1 * power) + (-1 * turn));
    pros::LeftBack.move((-1 * power) + (-1 * turn));
    pros::LeftMid.move((1 * power) + (1 * turn));

    pros::RightFront.move((1 * power) + (-1 * turn));
    pros::RightBack.move((1 * power) + (-1 * turn));
    pros::RightMid.move((-1 * power) + (1 * turn));

    pros::delay(10);
  }
}

/**
 * @brief
 *  This function is used to control the intake during Driver Control
 */

void _1028A::driver::IntakeCTRL(void *ptr) {
  while (1) {
    if (flywheelMode == 1) {
      if (pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_L1) &&
          (flywheelstate != 3 or flywheelstate != 2)) {
        pros::Intake.move(-127);
      } else if (pros::mainController.get_digital(
                     pros::E_CONTROLLER_DIGITAL_L2)) {
        pros::Intake.move(127);
      } else {
        pros::Intake.move(0);
      }

      if (pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_L1) &&
          (flywheelstate == 3 or flywheelstate == 2)) {
        pros::Intake.move(-127);
        pros::FlyWheel.move(127);
        pros::delay(140);
        pros::Intake.move(0);
        pros::delay(90 + FWoffset);
        FWoffset += 120;
      } else if (pros::mainController.get_digital(
                     pros::E_CONTROLLER_DIGITAL_L1) &&
                 (flywheelstate == 2)) {
        pros::Intake.move(-127);
        pros::FlyWheel.move(127);
        pros::delay(140);
        pros::Intake.move(0);
        pros::delay(90 + FWoffset);
        FWoffset += 150;
      } else {
        FWoffset = 0;
      }
    } else if (flywheelMode == 2) {

      if (pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_L1) &&
          (flywheelstate != 3 or flywheelstate != 2)) {
        pros::Intake.move(-127);
      } else if (pros::mainController.get_digital(
                     pros::E_CONTROLLER_DIGITAL_L2)) {
        pros::Intake.move(127);
      } else {
        pros::Intake.move(0);
      }

      if (pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_L1) &&
          (flywheelstate == 3 or flywheelstate == 2)) {
        pros::Intake.move(-127);
        pros::FlyWheel.move(127);
        pros::delay(140);
        pros::Intake.move(0);
        pros::delay(60 + FWoffset);
        FWoffset += 80;
      } else if (pros::mainController.get_digital(
                     pros::E_CONTROLLER_DIGITAL_L1) &&
                 (flywheelstate == 2)) {
        pros::Intake.move(-127);
        pros::FlyWheel.move(127);
        pros::delay(140);
        pros::Intake.move(0);
        pros::delay(90 + FWoffset);
        FWoffset += 150;
      } else {
        FWoffset = 0;
      }
    }
    pros::delay(10);
  }
}

void _1028A::driver::ModeCTRL(void *ptr) {
  while (1) {
    if (((pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_R1) &&
          pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_R2) &&
          pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_L1) &&
          pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_L2) &&
          pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_A)) or
         pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_X)) &&
        flywheelMode == 1) {
      flywheelMode = 2;
      pros::mainController.print(1, 1, "FW Mode: 2");
      pros::delay(1000);
    }

    else if (((pros::mainController.get_digital(
                   pros::E_CONTROLLER_DIGITAL_R1) &&
               pros::mainController.get_digital(
                   pros::E_CONTROLLER_DIGITAL_R2) &&
               pros::mainController.get_digital(
                   pros::E_CONTROLLER_DIGITAL_L1) &&
               pros::mainController.get_digital(
                   pros::E_CONTROLLER_DIGITAL_L2) &&
               pros::mainController.get_digital(
                   pros::E_CONTROLLER_DIGITAL_A)) or
              pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_X)) &&
             flywheelMode == 2) {
      flywheelMode = 1;
      pros::mainController.print(1, 1, "FW Mode: 1");
      pros::delay(1000);
    }
    pros::delay(10);
  }
}

void _1028A::driver::AngleCTRL(void *ptr) {
  int Angle = 0;
  while (1) {
    if (pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_Y) &&
        Angle == 0) {
      pros::angle.set_value(1);
      Angle = 1;
      pros::delay(500);
    }
    if (pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_Y) &&
        Angle == 1) {
      pros::angle.set_value(0);
      Angle = 0;
      pros::delay(500);
    }

    pros::delay(5);
  }
}
/**
 * @brief
 *  This function is used to control the expansion during Driver Control
 */
void _1028A::driver::ExpansionCTRL(void *ptr) {
  while (1) {
    if (pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) &&
        pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) &&
        pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      pros::expansionSide.set_value(1);
      pros::expansionMid.set_value(1);
    } else if (pros::mainController.get_digital(
                   pros::E_CONTROLLER_DIGITAL_LEFT) &&
               pros::mainController.get_digital(
                   pros::E_CONTROLLER_DIGITAL_DOWN)) {
      pros::expansionSide.set_value(1);
      // pros::expansionMid.set_value(1);
    } else if (pros::mainController.get_digital(
                   pros::E_CONTROLLER_DIGITAL_RIGHT) &&
               pros::mainController.get_digital(
                   pros::E_CONTROLLER_DIGITAL_DOWN)) {
      // pros::expansionSide.set_value(1);
      pros::expansionMid.set_value(1);
    }
    pros::delay(5);
  }
}