#include "main.h"
#include "okapi/impl/device/motor/motor.hpp"

#ifndef Config
#define Config

namespace pros{
  extern Motor LeftFront;
  extern Motor LeftMid;
  extern Motor LeftBack;
  extern Motor RightFront;
  extern Motor RightMid;
  extern Motor RightBack;
  extern Motor FlyWheel;
  extern Motor Intake;
  extern Imu Inertial;
  extern Rotation LeftRotation;
  extern Rotation RightRotation;
  extern Rotation HorizontalRotation;
  extern GPS GPS1;
  extern Controller mainController;
  extern ADIDigitalOut expansionLeft;
  extern ADIDigitalOut expansionRight;
  extern ADIDigitalOut expansionMid;
};
namespace okapi{
  extern Motor LeftFrontok;
  extern Motor LeftMidok;
  extern Motor LeftBackok;
  extern Motor RightFrontok;
  extern Motor RightMidok;
  extern Motor RightBackok;
  extern Motor Flywheelok;
  extern Motor Intakeok;
};

#endif