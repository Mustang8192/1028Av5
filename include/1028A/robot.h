#include "main.h"
#include "okapi/impl/device/motor/motor.hpp"

#define LeftFrontpt 19
#define RightFrontpt 20
#define LeftMidpt 17
#define RightMidpt 18
#define LeftBackpt 16
#define RightBackpt 15
#define Rotationpt 12
#define Flywheelpt 14
#define Intakept 21
#define Inertialpt 11

#ifndef Config
#define Config

namespace pros {
extern Motor LeftFront;
extern Motor LeftMid;
extern Motor LeftBack;
extern Motor RightFront;
extern Motor RightMid;
extern Motor RightBack;
extern Motor FlyWheel;
extern Motor Intake;
extern Imu Inertial;
extern Rotation Rotation;
extern GPS GPS1;
extern Controller mainController;
extern ADIDigitalOut angle;
extern ADIDigitalOut expansionSide;
extern ADIDigitalOut expansionMid;
// extern ADILed UnderglowLeft;
// extern ADILed UnderglowRight;
}; // namespace pros
namespace okapi {
extern Motor LeftFrontok;
extern Motor LeftMidok;
extern Motor LeftBackok;
extern Motor RightFrontok;
extern Motor RightMidok;
extern Motor RightBackok;
extern Motor Flywheelok;
extern Motor Intakeok;
}; // namespace okapi

#endif