#include "1028A/robot.h"

namespace pros {
Motor LeftFront(LeftFrontpt);
Motor LeftMid(LeftMidpt);
Motor LeftBack(LeftBackpt);
Motor RightFront(RightFrontpt);
Motor RightMid(RightMidpt);
Motor RightBack(RightBackpt);
Motor FlyWheel(Flywheelpt);
Motor Intake(Intakept);
Imu Inertial(Inertialpt);
class Rotation Rotation(Rotationpt);
GPS GPS1(1);
Controller mainController(CONTROLLER_MASTER);
ADIDigitalOut angle('a');
ADIDigitalOut expansionSide('c');
ADIDigitalOut expansionMid('b');
}; // namespace pros

namespace okapi {
Motor LeftFrontok(LeftFrontpt);
Motor LeftMidok(LeftMidpt);
Motor LeftBackok(LeftBackpt);
Motor RightFrontok(RightFrontpt);
Motor RightMidok(RightMidpt);
Motor RightBackok(RightBackpt);
Motor Flywheelok(Flywheelpt);
Motor Intakeok(Intakept);
}; // namespace okapi