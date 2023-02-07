#include "1028A/robot.h"

#define LeftFrontpt 19
#define RightFrontpt 20
#define LeftMidpt 17
#define RightMidpt 18
#define LeftBackpt 16
#define RightBackpt 15
#define LeftRotationpt 2
#define RightRotationpt 1
#define HorizontalRotationpt 4
#define Flywheelpt 14
#define Intakept 21

namespace pros{
  Motor LeftFront(LeftFrontpt);
  Motor LeftMid (LeftMidpt);
  Motor LeftBack (LeftBackpt);
  Motor RightFront (RightFrontpt);
  Motor RightMid (RightMidpt);
  Motor RightBack (RightBackpt);
  Motor FlyWheel (Flywheelpt);
  Motor Intake (Intakept);
  Imu Inertial(11);
  Rotation LeftRotation(LeftRotationpt);
  Rotation RightRotation(RightRotationpt);
  Rotation HorizontalRotation(HorizontalRotationpt);
  GPS GPS1(1);
  Controller mainController(CONTROLLER_MASTER);
  ADIDigitalOut expansionLeft('a');
  ADIDigitalOut expansionRight('c');
  ADIDigitalOut expansionMid('b');
};

namespace okapi{
  Motor LeftFrontok(LeftFrontpt);
  Motor LeftMidok (LeftMidpt);
  Motor LeftBackok (LeftBackpt);
  Motor RightFrontok (RightFrontpt);
  Motor RightMidok (RightMidpt);
  Motor RightBackok (RightBackpt);
  Motor Flywheelok (Flywheelpt);
  Motor Intakeok (Intakept);
};
