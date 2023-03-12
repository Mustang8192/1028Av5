#include "1028A/init.h"
#include "pros/misc.h"
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
    if (pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      pros::Intake.move(127);
    } else if (pros::mainController.get_digital(
                   pros::E_CONTROLLER_DIGITAL_L2)) {
      pros::Intake.move(-127);
    }
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
      pros::expansionLeft.set_value(1);
      pros::expansionRight.set_value(1);
    } else if (pros::mainController.get_digital(
                   pros::E_CONTROLLER_DIGITAL_LEFT) &&
               pros::mainController.get_digital(
                   pros::E_CONTROLLER_DIGITAL_DOWN)) {
      pros::expansionLeft.set_value(1);
    } else if (pros::mainController.get_digital(
                   pros::E_CONTROLLER_DIGITAL_RIGHT) &&
               pros::mainController.get_digital(
                   pros::E_CONTROLLER_DIGITAL_DOWN)) {
      pros::expansionRight.set_value(1);
    } else if (pros::mainController.get_digital(
                   pros::E_CONTROLLER_DIGITAL_LEFT) &&
               pros::mainController.get_digital(
                   pros::E_CONTROLLER_DIGITAL_UP)) {
      pros::expansionSide.set_value(1);
    } else if (pros::mainController.get_digital(
                   pros::E_CONTROLLER_DIGITAL_RIGHT) &&
               pros::mainController.get_digital(
                   pros::E_CONTROLLER_DIGITAL_UP)) {
      pros::expansionMid.set_value(1);
    }
    pros::delay(5);
  }
}

void _1028A::driver::CataCTRL(void *ptr) {
  while (1) {
    if (pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      pros::Cata.move(127);
      pros::delay(500);
      double lastError;
      double error;
      double kp = 0.1;
      double ki = 0.0001;
      double kd = 0.1;
      double sensorValue;
      double powerValue;
      while (1) {
        sensorValue = pros::CataPosition.get_angle();
        error = 0 - sensorValue;
        powerValue = pid::math(error, lastError, kp, ki, kd, 127);
        pros::Cata.move(powerValue);
        lastError = error;
        if (pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_R2) or
            fabs(error <= 1)) {
          pros::Cata.move(0);
          break;
        }
        pros::delay(5);
      }
    }
    pros::delay(5);
  }
}

void _1028A::driver::CataAssistCTRL(void *ptr) {
  int assist = 0;
  while (1) {
    if (pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_X) &&
        assist == 0) {
      pros::cataAssist.set_value(1);
      assist = 1;
    } else if (pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_X) &&
               assist == 1) {
      pros::cataAssist.set_value(0);
      assist = 0;
    }
    pros::delay(5);
  }
}