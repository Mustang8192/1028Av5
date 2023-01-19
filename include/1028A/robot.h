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
  extern ADIDigitalOut expansion;
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
namespace sylib{
  extern Motor LeftFrontsy;
  extern Motor LeftMidsy;
  extern Motor LeftBacksy;
  extern Motor RightFrontsy;
  extern Motor RightMidsy;
  extern Motor RightBacksy;
  extern Motor Flywheelsy;
  extern Motor Intakesy;
};

#endif