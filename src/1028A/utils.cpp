#include "1028A/init.h"
#include "1028A/robot.h"
#include "pros/adi.hpp"
#include "pros/apix.h"
#include "pros/colors.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"

/**
 * @brief
 *    This function is used to stop all motors of the robot
 */
void _1028A::robot::DriveStop() {
  pros::LeftFront.brake();
  pros::LeftMid.brake();
  pros::LeftBack.brake();
  pros::RightFront.brake();
  pros::RightMid.brake();
  pros::RightBack.brake();
}

/**
 * @brief
 *    This function is used to reset the drive encoders
 */
void _1028A::robot::resetDrive() { pros::Rotation.reset_position(); }

void _1028A::robot::underglowInit() {
  // pros::ADILED led('c', 5);
  // led.set_all(0x808080);
  // led.update();
}

void _1028A::robot::preMatchChecks() {
  pros::c::v5_device_e_t LeftFrontcheck =
      pros::c::registry_get_plugged_type((LeftFrontpt - 1));
  pros::c::v5_device_e_t LeftMidcheck =
      pros::c::registry_get_plugged_type((LeftMidpt - 1));
  pros::c::v5_device_e_t LeftBackcheck =
      pros::c::registry_get_plugged_type((LeftBackpt - 1));
  pros::c::v5_device_e_t RightFrontcheck =
      pros::c::registry_get_plugged_type((RightFrontpt - 1));
  pros::c::v5_device_e_t RightMidcheck =
      pros::c::registry_get_plugged_type((RightMidpt - 1));
  pros::c::v5_device_e_t RightBackcheck =
      pros::c::registry_get_plugged_type((RightBackpt - 1));
  pros::c::v5_device_e_t Intakecheck =
      pros::c::registry_get_plugged_type((Intakept - 1));
  pros::c::v5_device_e_t Inertialcheck =
      pros::c::registry_get_plugged_type((Inertialpt - 1));
  pros::c::v5_device_e_t Rotationcheck =
      pros::c::registry_get_plugged_type((Rotationpt - 1));

  pros::c::registry_bind_port((LeftFrontpt - 1), LeftFrontcheck);
  pros::c::registry_bind_port((LeftMidpt - 1), LeftMidcheck);
  pros::c::registry_bind_port((LeftBackpt - 1), LeftBackcheck);
  pros::c::registry_bind_port((RightFrontpt - 1), RightFrontcheck);
  pros::c::registry_bind_port((RightMidpt - 1), RightMidcheck);
  pros::c::registry_bind_port((RightBackpt - 1), RightBackcheck);
  pros::c::registry_bind_port((Intakept - 1), Intakecheck);
  pros::c::registry_bind_port((Inertialpt - 1), Inertialcheck);
  pros::c::registry_bind_port((Rotationpt - 1), Rotationcheck);

  if (LeftFrontcheck != pros::c::E_DEVICE_MOTOR) {
    pros::mainController.print(1, 1, "Left Front Motor Error");
  }
  if (LeftMidcheck != pros::c::E_DEVICE_MOTOR) {
    pros::mainController.print(1, 1, "Left Mid Motor Error");
  }
  if (LeftBackcheck != pros::c::E_DEVICE_MOTOR) {
    pros::mainController.print(1, 1, "Left Back Motor Error");
  }
  if (RightFrontcheck != pros::c::E_DEVICE_MOTOR) {
    pros::mainController.print(1, 1, "Right Front Motor Error");
  }
  if (RightMidcheck != pros::c::E_DEVICE_MOTOR) {
    pros::mainController.print(1, 1, "Right Mid Motor Error");
  }
  if (RightBackcheck != pros::c::E_DEVICE_MOTOR) {
    pros::mainController.print(1, 1, "Right Back Motor Error");
  }
  if (RightFrontcheck != pros::c::E_DEVICE_MOTOR) {
    pros::mainController.print(1, 1, "Right Front Motor Error");
  }
  if (Intakecheck != pros::c::E_DEVICE_MOTOR) {
    pros::mainController.print(1, 1, "Intake Motor Error");
  }
  if (Inertialcheck != pros::c::E_DEVICE_IMU) {
    pros::mainController.print(1, 1, "Inertial Sensor Error");
  }
  if (Rotationcheck != pros::c::E_DEVICE_ROTATION) {
    pros::mainController.print(1, 1, "Rotation Sensor Error");
  }
}

/**
 * @brief
 *  This function is used to start a task
 * @param name
 *  The name of the task
 * @param func
 *  The function that the task will run
 */

void _1028A::task::start(std::string name, void (*func)(void *)) {
  if (!exists(name)) {
    tasks.insert(std::pair<std::string, std::unique_ptr<pros::Task>>(
        name,
        std::move(std::make_unique<pros::Task>(func, &v, TASK_PRIORITY_DEFAULT,
                                               TASK_STACK_DEPTH_DEFAULT, ""))));
  }
}

/**
 * @brief
 *  This function is used to check if a task exists
 * @param name
 * @return true
 * @return false
 */

bool _1028A::task::exists(std::string name) {
  return tasks.find(name) != tasks.end();
}

/**
 * @brief
 *  This function is used to kill a task
 * @param name
 *  The name of the task
 */

void _1028A::task::kill(std::string name) {
  if (exists(name)) {
    tasks.erase(name);
  }
}

/**
 * @brief
 *   This function is used to calculate the PID Power Values
 * @param Error
 *  The error between the requested value and the current value
 * @param lastError
 *  The error from the last loop
 * @param Kp
 *  The Kp value
 * @param Ki
 *  The Ki value
 * @param Kd
 *  The Kd value
 * @param maxSpd
 *  The maximum speed of the motor
 * @return
 *  The power value
 */
int _1028A::pid::math(float Error, float lastError, float Kp, float Ki,
                      float Kd, double maxSpd) {
  float P;
  float D;
  float Drive;

  P = (Kp * Error);
  static float I = 0;
  I += Error * Ki;
  if (I > 1) {
    I = 1;
  }
  if (I < -1) {
    I = -1;
  }
  D = (Error - lastError) * Kd;
  Drive = P + I + D;

  if (Drive > maxSpd) {
    Drive = maxSpd;
  }
  if (Drive < -maxSpd) {
    Drive = -maxSpd;
  }
  return Drive;
}

/**
 * @brief
 *  This function is used to exit the PID loop
 *
 * @param Error
 *   The error between the requested value and the current value
 * @param Threshold
 *   The threshold of the error
 * @param currTime
 *   The current time
 * @param startTime
 *   The start time
 * @param timeExit
 *   The time to exit the loop
 * @return true
 * @return false
 */
bool _1028A::pid::exit(float Error, float Threshold, float currTime,
                       float startTime, float timeExit, float powerValue) {
  if ((Error < Threshold and Error > -Threshold) && powerValue <= 10) {
    return true;
  } else if (currTime - startTime >= timeExit) {
    return true;
  } else {
    return false;
  }
}

/**
 * @brief
 *  This function is used to turn on the PID controller
 *
 * @param RequestedValue
 *  The target value
 * @param spd
 *  The maximum speed of the motor
 * @param thre
 *  The threshold of the error
 * @param time
 *  The time to exit the loop
 */
void _1028A::pid::turn(double RequestedValue, double spd, double thre,
                       double time, double kpOffset, double kdOffset) {
  float SensorCurrentValue;
  float error;
  float lastError = 0;

  float Kp = 1.5 + kpOffset;
  float Ki = 0;
  float Kd = 5 + kdOffset;
  double timeExit = 0;
  double startTime = pros::millis();
  while (1) {
    // Reads the sensor value and scale
    SensorCurrentValue = pros::Inertial.get_rotation();
    double currentTime = pros::millis();

    // calculates error
    error = (RequestedValue - SensorCurrentValue);

    // calculate drive PID
    float powerValue = math(error, lastError, Kp, Ki, Kd, spd);

    if (exit(error, thre, currentTime, startTime, time, powerValue)) {
      pros::LeftFront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      pros::LeftMid.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      pros::LeftBack.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      pros::RightFront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      pros::RightMid.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      pros::RightBack.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

      pros::LeftFront.brake();
      pros::LeftBack.brake();
      pros::RightFront.brake();
      pros::RightBack.brake();
      break;
    }

    // Move Motors with PID
    pros::LeftFront.move((0 * powerValue) + (-1 * powerValue));
    pros::LeftBack.move((0 * powerValue) + (-1 * powerValue));
    pros::LeftMid.move((0 * powerValue) + (1 * powerValue));

    pros::RightFront.move((0 * powerValue) + (-1 * powerValue));
    pros::RightBack.move((0 * powerValue) + (-1 * powerValue));
    pros::RightMid.move((0 * powerValue) + (1 * powerValue));

    lastError = error;
    pros::delay(20);
  }
  pros::LeftFront.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  pros::LeftMid.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  pros::LeftBack.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  pros::RightFront.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  pros::RightMid.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  pros::RightBack.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  pros::mainController.print(1, 1, "Turn Done");
}

/**
 * @brief
 *  This function is used to move the robot forward using the PID controller
 *
 * @param RequestedValue
 *  The target value
 * @param spd
 *  The maximum speed of the motor
 * @param thre
 *  The threshold of the error
 * @param time
 *  The time to exit the loop
 */
void _1028A::pid::forward(double RequestedValue, double spd, double thre,
                          double time, double kpOffset, double kdOffset) {
  float SensorCurrentValue;
  float Error;
  float lastError = 0;

  float Kp = 0.2 + kpOffset;
  float Ki = 0;
  float Kd = 0.8 + kdOffset;

  double startTime = pros::millis();
  double timeExit = 0;
  while (1) {
    // Reads the sensor value and scale
    SensorCurrentValue = (pros::Rotation.get_position() / 100.0);
    double currentTime = pros::millis();

    // calculates error
    Error = (RequestedValue - SensorCurrentValue);
    printf("%f", Error);

    // calculate drive PID
    // pros::screen::print(TEXT_SMALL, 1,3,"test");
    float powerValue = math(Error, lastError, Kp, Ki, Kd, spd);

    if (exit(Error, thre, currentTime, startTime, time, powerValue)) {
      pros::LeftFront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      pros::LeftMid.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      pros::LeftBack.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      pros::RightFront.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      pros::RightMid.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
      pros::RightBack.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

      pros::LeftFront.brake();
      pros::LeftMid.brake();
      pros::LeftBack.brake();
      pros::RightFront.brake();
      pros::RightMid.brake();
      pros::RightBack.brake();
      break;
    }
    // Move Motors with PID
    pros::LeftFront.move((-1 * powerValue) + (0 * powerValue));
    pros::LeftBack.move((-1 * powerValue) + (0 * powerValue));
    pros::LeftMid.move((1 * powerValue) + (0 * powerValue));

    pros::RightFront.move((1 * powerValue) + (0 * powerValue));
    pros::RightBack.move((1 * powerValue) + (0 * powerValue));
    pros::RightMid.move((-1 * powerValue) + (0 * powerValue));

    lastError = Error;

    pros::delay(20);
  }
  pros::LeftFront.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  pros::LeftMid.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  pros::LeftBack.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  pros::RightFront.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  pros::RightMid.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  pros::RightBack.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

void _1028A::time::turn(double spd, double time) {
  pros::LeftFront.move((0 * spd) + (-1 * spd));
  pros::LeftBack.move((0 * spd) + (-1 * spd));
  pros::LeftMid.move((0 * spd) + (1 * spd));

  pros::RightFront.move((0 * spd) + (-1 * spd));
  pros::RightBack.move((0 * spd) + (-1 * spd));
  pros::RightMid.move((0 * spd) + (1 * spd));

  pros::LeftFront.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  pros::LeftMid.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  pros::LeftBack.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  pros::RightFront.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  pros::RightMid.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  pros::RightBack.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

  pros::delay(time);

  pros::LeftFront.brake();
  pros::LeftMid.brake();
  pros::LeftBack.brake();
  pros::RightFront.brake();
  pros::RightMid.brake();
  pros::RightBack.brake();
}

void _1028A::time::forward(double spd, double time) {
  pros::LeftFront.move((-1 * spd) + (0 * spd));
  pros::LeftBack.move((-1 * spd) + (0 * spd));
  pros::LeftMid.move((1 * spd) + (0 * spd));

  pros::RightFront.move((1 * spd) + (0 * spd));
  pros::RightBack.move((1 * spd) + (0 * spd));
  pros::RightMid.move((-1 * spd) + (0 * spd));

  pros::LeftFront.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  pros::LeftMid.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  pros::LeftBack.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  pros::RightFront.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  pros::RightMid.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  pros::RightBack.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

  pros::delay(time);
  pros::LeftFront.brake();
  pros::LeftMid.brake();
  pros::LeftBack.brake();
  pros::RightFront.brake();
  pros::RightMid.brake();
  pros::RightBack.brake();
}

void _1028A::time::rightOnly(double spd, double time) {
  pros::LeftFront.brake();
  pros::LeftMid.brake();
  pros::LeftBack.brake();

  pros::RightFront.move((1 * spd));
  pros::RightBack.move((1 * spd));
  pros::RightMid.move((-1 * spd));

  pros::delay(time);

  pros::RightFront.brake();
  pros::RightMid.brake();
  pros::RightBack.brake();
}

void _1028A::time::leftOnly(double spd, double time) {
  pros::RightFront.brake();
  pros::RightMid.brake();
  pros::RightBack.brake();

  pros::LeftFront.move((1 * spd));
  pros::LeftBack.move((1 * spd));
  pros::LeftMid.move((-1 * spd));

  pros::delay(time);

  pros::LeftFront.brake();
  pros::LeftMid.brake();
  pros::LeftBack.brake();
}

void _1028A::OdomDebug::resetSens() { pros::Rotation.reset(); }

void _1028A::OdomDebug::startOdomDebug(void *ptr) {
  _1028A::OdomDebug display(lv_scr_act(), LV_COLOR_ORANGE);
  display.setResetCallback(resetSens);

  while (1) {
    display.setData(
        {CurrentPosition.x, CurrentPosition.y, CurrentPosition.theta},
        {double(pros::Rotation.get_position() / 100.0),
         double(pros::Rotation.get_position() / 100.0),
         double(pros::Rotation.get_position() / 100.0)});
    pros::delay(20);
  }
}