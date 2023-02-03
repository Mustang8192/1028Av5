#include "1028A/init.h"
#include "1028A/robot.h"
#include "pros/misc.h"

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
void _1028A::robot::resetDrive() {
  pros::LeftRotation.reset_position();
  pros::RightRotation.reset_position();
  pros::HorizontalRotation.reset_position();
}

/**
* @brief
*  This function is used to set the target speed of the flywheel with PID during Driver Control
*/

void _1028A::flywheel::startFlywheel(double target){
  if (flywheelstate != 3){
    double error = (target - pros::FlyWheel.get_actual_velocity());
    if (error > 10){
      pros::FlyWheel.move(127);
    }
    else if (error <= 10){
      pros::FlyWheel.move(target);
    }
  }
  else if (flywheelstate == 3){
    pros::FlyWheel.move(target);

    if (pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
      pros::delay(100);
      pros::FlyWheel.move(127);
    }
  }
}

void _1028A::flywheel::startFlywheelTask(void *ptr){
  while (FWcontinueTask){
    startFlywheel(fwtarget);
    pros::delay(20);
  }
}
/**
 * @brief 
 *  This function checks the brake type and changes it if it is not set right during Driver Control
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
      } else if (pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
        flywheelstate = 2;
      } else if (pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        flywheelstate = 3;
      } else if (pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
        flywheelstate = 4;
      } else if (pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        flywheelstate = 0;
      }

      if (flywheelstate == 0) {
        pros::FlyWheel.brake();
      } else if (flywheelstate == 1) {
        flywheel::startFlywheel(127);
      } else if (flywheelstate == 2) {
        flywheel::startFlywheel(115);
      } else if (flywheelstate == 3 && !pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        flywheel::startFlywheel(100);
      } else if (flywheelstate == 4) {
        flywheel::startFlywheel(77.5);
      }
    }
    else if (flywheelMode == 2){
        if (pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        flywheelstate = 1;
      } else if (pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
        flywheelstate = 2;
      } else if (pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
        flywheelstate = 3;
      } else if (pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
        flywheelstate = 4;
      } else if (pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        flywheelstate = 0;
      }

      if (flywheelstate == 0) {
        pros::FlyWheel.brake();
      } else if (flywheelstate == 1) {
        flywheel::startFlywheel(127);
      } else if (flywheelstate == 2) {
        flywheel::startFlywheel(120);
      } else if (flywheelstate == 3 && !pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        flywheel::startFlywheel(95);
      } else if (flywheelstate == 4) {
        flywheel::startFlywheel(77.5);
      }
    }
  pros::delay(20);
  }
}

/**
* @brief 
*  This function is used to control the Drive train during Driver Control 
*/

void _1028A::driver::DriveCTRL(void *ptr) {
  while (1) {
    double power = pros::mainController.get_analog(ANALOG_LEFT_Y);
    double turn = pros::mainController.get_analog(ANALOG_RIGHT_X);

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
    if(flywheelMode == 1){
      if (pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && (flywheelstate != 3)) {
        pros::Intake.move(-127);
      } else if (pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        pros::Intake.move(127);
      } else {
        pros::Intake.move(0);
      }

      if (pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && (flywheelstate == 3)){
        pros::Intake.move(-127);
        pros::FlyWheel.move(127);
        pros::delay(140);
        pros::Intake.move(0);
        pros::delay(320);
      }

      if(pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) && (flywheelstate == 3)){
        pros::FlyWheel.move(127);
        pros::Intake.move(-106);
      }
      else if (pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
        pros::Intake.move(-127);
      }
      else if (pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)){
        pros::Intake.move(-127);
        pros::delay(300);
        pros::Intake.move(0);
        pros::delay(250);
      }
    }
    else if (flywheelMode == 2){

      if (pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && (flywheelstate != 3)) {
        pros::Intake.move(-127);
      } else if (pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        pros::Intake.move(127);
      } else {
        pros::Intake.move(0);
      }

      if (pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && (flywheelstate == 3)){
        pros::Intake.move(-127);
        pros::FlyWheel.move(127);
        pros::delay(140);
        pros::Intake.move(0);
        pros::delay(60 + FWoffset);
        FWoffset += 25;
      }
      else{
        FWoffset = 0;
      }

      if(pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) && (flywheelstate == 3)){
        pros::FlyWheel.move(127);
        pros::Intake.move(-106);
      }
      else if (pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
        pros::Intake.move(-127);
      }
      else if (pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)){
        pros::Intake.move(-127);
        pros::delay(300);
        pros::Intake.move(0);
        pros::delay(250);
      }
    }
    pros::delay(10);
  }
}

void _1028A::driver::ModeCTRL(void *ptr){
  while (1){
    if (((pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_A)) or pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_X)) && flywheelMode == 1){
      flywheelMode = 2;
      pros::mainController.print(1, 1, "FW Mode: 2");
      pros::delay(1000);
    }

    else if (((pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_L2) && pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_A)) or pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_X)) && flywheelMode == 2){
      flywheelMode = 1;
      pros::mainController.print(1, 1, "FW Mode: 1");
      pros::delay(1000);
    }
    pros::delay(10);
  }
}

/**
 * @brief 
 *  This function is used to control the expansion during Driver Control
 */
void _1028A::driver::ExpansionCTRL(void *ptr){
  while(1){
    if (pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) &&
        pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) &&
        pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
        pros::expansionLeft.set_value(1);
        pros::expansionMid.set_value(1);
        pros::expansionRight.set_value(1);
    }
    else if (pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) &&
        pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        pros::expansionLeft.set_value(1);
        pros::expansionMid.set_value(1);
    }
    else if (pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) &&
        pros::mainController.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
          pros::expansionRight.set_value(1);
          pros::expansionMid.set_value(1);
        }
    pros::delay(10);
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

bool _1028A::task::exists(std::string name) { return tasks.find(name) != tasks.end(); }

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
int _1028A::pid::math(float Error, float lastError, float Kp, float Ki, float Kd,
            double maxSpd) {
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
bool _1028A::pid::exit(float Error, float Threshold, float currTime, float startTime,
                float timeExit) {
  if ((Error < Threshold and Error > -Threshold)) {
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
void _1028A::pid::turn(double RequestedValue, double spd, double thre, double time) {
  float SensorCurrentValue;
  float error;
  float lastError = 0;

  float Kp = 1.5;
  float Ki = 0;
  float Kd = 5;
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

    if (exit(error, thre, currentTime, startTime, time)) {
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
void _1028A::pid::forward(double RequestedValue, double spd, double thre, double time) {
  spd = ((spd / 100) * 127);
  float SensorCurrentValue;
  float Error;
  float lastError = 0;

  float Kp = 0.3;
  float Ki = 0.6;
  float Kd = 1;

  double startTime = pros::millis();
  double timeExit = 0;
  double start = pros::LeftRotation.get_position();
  while (1) {
    // Reads the sensor value and scale
    SensorCurrentValue = (start / 100) - (pros::LeftRotation.get_position() / 100.0);
    double currentTime = pros::millis();

    // calculates error
    Error = (RequestedValue - SensorCurrentValue);

    // calculate drive PID
    // pros::screen::print(TEXT_SMALL, 1,3,"test");
    float powerValue = math(Error, lastError, Kp, Ki, Kd, spd);

    if (exit(Error, thre, currentTime, startTime, time)) {
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

void _1028A::time::turn(double spd, double time){
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

void _1028A::time::forward(double spd, double time){
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

void _1028A::time::rightOnly(double spd, double time){
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

void _1028A::time::leftOnly(double spd, double time){
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

void _1028A::OdomDebug::resetSens(){
  pros::LeftRotation.reset();
  pros::RightRotation.reset();
  pros::HorizontalRotation.reset();
}

void _1028A::OdomDebug::startOdomDebug(void *ptr){
  _1028A::OdomDebug display(lv_scr_act(), LV_COLOR_ORANGE);
  display.setResetCallback(resetSens);

  while (1){
    display.setData({CurrentPosition.x, CurrentPosition.y, CurrentPosition.theta}, {double(pros::LeftRotation.get_position()/100.0), double(pros::RightRotation.get_position()/100.0), double(pros::HorizontalRotation.get_position()/100.0)});
    pros::delay(20);
  }
}