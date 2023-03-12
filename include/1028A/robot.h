#include "main.h"
#include "okapi/impl/device/motor/motor.hpp"
#include "pros/adi.hpp"

#define LeftFrontpt 19
#define RightFrontpt 20
#define LeftMidpt 17
#define RightMidpt 18
#define LeftBackpt 16
#define RightBackpt 15
#define Catapt 10
#define Catapositionpt 8
#define Rotationpt 9
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
extern Motor Intake;
extern Motor Cata;
extern Imu Inertial;
extern Rotation CataPosition;
extern Rotation Rotation;
extern GPS GPS1;
extern Controller mainController;
extern ADIDigitalOut cataAssist;
extern ADIDigitalOut expansionLeft;
extern ADIDigitalOut expansionRight;
extern ADIDigitalOut expansionMid;
extern ADIDigitalOut expansionSide;
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
extern Motor Cataok;
extern Motor Intakeok;
}; // namespace okapi
#endif