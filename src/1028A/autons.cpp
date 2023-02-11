#include "1028A/init.h"
#include "1028A/robot.h"

void _1028A::robot::auton() {
  pros::Inertial.set_rotation(0);
  autonSelect = 3;
  if (autonSelect == 1) {
    // Left Side
    time::forward(-127, 100);
    pros::Intake.move(-127);
    pros::delay(140);
    pros::Intake.brake();
    time::forward(80, 100);
    pros::delay(400);
    time::turn(50, 500);
    fwtarget = 70;
    task::start("fw", flywheel::startFlywheelTask);
    pros::delay(5000);
    pros::Intake.move(-127);
  } else if (autonSelect == 2) {
    // Right Side
    flywheelMode = 2;
    fwtarget = 106;
    task::start("fw", flywheel::startFlywheelTask);
    pros::Intake.move(127);
    time::forward(45, 1000);
    pros::delay(2500);
    pros::Intake.move(-127);
    pros::delay(170);
    fwtarget = 110;
    pros::Intake.brake();
    pros::delay(700);
    pros::Intake.move(-127);
    pros::delay(170);
    pros::Intake.brake();
    fwtarget = 113;
    pros::delay(800);
    pros::Intake.move(-127);
    pros::delay(170);
    pros::Intake.brake();
    pros::delay(500);
    time::turn(-127, 240);
    pros::delay(500);
    time::forward(-80, 700);
    time::turn(127, 230);
    pros::delay(500);
    time::forward(-100, 250);
    pros::Intake.move(127);
    pros::delay(350);
    pros::Intake.brake();
    time::forward(100, 150);
    pros::delay(250);
    fwtarget = 106;
    pid::turn(-74, 127, 1, 800);
    pros::delay(250);
    pros::Intake.move(127);
    time::forward(127, 600);
    time::forward(40, 1200);
    pros::delay(250);
    pid::turn(20, 127, 1, 800);
    pros::delay(700);
    pros::Intake.move(-127);
    pros::delay(165);
    pros::Intake.brake();
    fwtarget = 110;
    pros::delay(700);
    pros::Intake.move(-127);
    pros::delay(165);
    pros::Intake.brake();
  } else if (autonSelect == 3) {
    // Solo Win Point
    fwtarget = 105;
    task::start("fw", flywheel::startFlywheelTask);
    pros::Intake.move(127);
    time::forward(-100, 100);
    pros::delay(510);
    time::forward(127, 100);
    pid::turn(-70, 127, 1, 700);
    time::forward(-127, 535);
    pid::turn(3, 127, 1, 700);
    time::forward(127, 550);
    pid::turn(-38, 127, 1, 800);
    pros::Intake.move(-127);
    pros::delay(165);
    pros::Intake.brake();
    fwtarget = 127;
    pros::delay(700);
    pros::Intake.move(-127);
    pros::delay(165);
    pros::Intake.brake();
    fwtarget = 118;
    pros::delay(500);
    pid::turn(47, 127, 1, 600);
    pros::Intake.move(127);
    time::forward(40, 2600);
    fwtarget = 115;
    pros::delay(600);
    pid::turn(-70, 127, 1, 800);
    pros::Intake.move(-127);
    pros::delay(165);
    pros::Intake.brake();
    fwtarget = 119;
    pros::delay(700);
    pros::Intake.move(-127);
    pros::delay(165);
    fwtarget = 125;
    pros::Intake.brake();
    pros::delay(700);
    pros::Intake.move(-127);
    pros::delay(165);
    pros::Intake.brake();
    pros::delay(500);
    pid::turn(-136, 127, 1, 700);
    time::forward(-127, 550);
    pid::turn(-70, 127, 1, 700);
    time::forward(-127, 400);
    pros::Intake.move(127);
    pros::delay(550);
    pros::Intake.brake();
  } else if (autonSelect == 8) {
    // Skills
    pros::Intake.move(127);
    time::forward(-100, 100);
    pros::delay(400);
    time::rightOnly(127, 300);
    time::forward(70, 550);
    fwtarget = 92;
    task::start("fw", flywheel::startFlywheelTask);
    pid::turn(75, 127, 1, 2000);
    pid::turn(75, 127, 1, 1000);
    pid::turn(75, 127, 1, 500);
    pid::turn(75, 127, 1, 100);
    time::forward(-127, 535);
    pros::delay(550);
    time::forward(127, 200);
    pid::turn(-7, 127, 1, 1200);
    pid::turn(-7, 127, 1, 600);
    pid::turn(-7, 127, 1, 250);
    pid::turn(-7, 127, 1, 100);
    time::forward(127, 725);
    pros::delay(700);
    pros::Intake.move(-127);
    pros::delay(140);
    pros::Intake.brake();
    fwtarget = 120;
    pros::delay(300);
    pros::Intake.move(-127);
    pros::delay(140);
    pros::Intake.brake();
    pros::delay(100);
    pros::Intake.move(-127);
    pros::delay(300);
    pros::Intake.brake();
    pros::delay(1000);
    pid::turn(-8, 127, 1, 500);
    pid::turn(-8, 127, 1, 250);
    pid::turn(-8, 127, 1, 100);
    pid::turn(-8, 127, 1, 50);
    pros::delay(1000);
    time::forward(-127, 600);
    fwtarget = 87;
    pid::turn(43, 127, 1, 1000);
    pid::turn(43, 127, 1, 500);
    pid::turn(43, 127, 1, 250);
    pid::turn(43, 127, 1, 100);
    pros::Intake.move(127);
    time::forward(38, 3100);
    pros::delay(900);
    pid::turn(-50, 127, 1, 1000);
    pid::turn(-50, 127, 1, 500);
    pid::turn(-50, 127, 1, 250);
    pid::turn(-50, 127, 1, 100);
    pros::delay(700);
    pros::Intake.move(-127);
    pros::delay(165);
    fwtarget = 105;
    pros::Intake.brake();
    pros::delay(700);
    pros::Intake.move(-127);
    pros::delay(165);
    pros::Intake.brake();
    fwtarget = 115;
    pros::delay(700);
    pros::Intake.move(-127);
    pros::delay(165);
    pros::Intake.brake();
    pros::delay(500);
    pid::turn(7, 127, 1, 1000);
    pid::turn(7, 127, 1, 500);
    pid::turn(7, 127, 1, 250);
    pid::turn(7, 127, 1, 100);
    fwtarget = 95;
    pros::Intake.move(127);
    pros::delay(500);
    time::forward(25, 5800);
    pros::delay(1000);
    pid::turn(-108, 127, 1, 1000);
    pid::turn(-108, 127, 1, 500);
    pid::turn(-108, 127, 1, 250);
    pid::turn(-108, 127, 1, 100);
    pros::delay(700);
    pros::Intake.move(-127);
    pros::delay(165);
    pros::Intake.brake();
    pros::delay(700);
    pros::Intake.move(-127);
    pros::delay(165);
    fwtarget = 105;
    pros::Intake.brake();
    pros::delay(700);
    pros::Intake.move(-127);
    pros::delay(165);
    pros::Intake.brake();
    pros::delay(500);
    fwtarget = 115;
    pros::Intake.brake();
    pros::delay(700);
    pros::Intake.move(-127);
    pros::delay(165);
    pros::Intake.brake();
    pros::delay(500);
    pid::turn(-75, 127, 1, 1000);
    pid::turn(-75, 127, 1, 500);
    pid::turn(-75, 127, 1, 250);
    pid::turn(-75, 127, 1, 100);
    time::forward(-127, 950);
    pros::delay(1000);
    pid::turn(-180, 127, 1, 1000);
    pid::turn(-180, 127, 1, 500);
    pid::turn(-180, 127, 1, 250);
    pid::turn(-180, 127, 1, 100);
    pros::Intake.move(127);
    time::forward(-100, 400);
    pros::delay(400);
    time::rightOnly(127, 500);
    time::forward(70, 550);
    pid::turn(-75, 127, 1, 1000);
    pid::turn(-75, 127, 1, 500);
    pid::turn(-75, 127, 1, 250);
    pid::turn(-75, 127, 1, 100);
    time::forward(-127, 900);
    pros::delay(600);
    time::forward(127, 225);
    pros::Intake.move(-127);
    pros::delay(140);
    pros::Intake.brake();
    pid::turn(-140, 127, 1, 1000);
    pid::turn(-140, 127, 1, 500);
    pid::turn(-140, 127, 1, 250);
    pid::turn(-140, 127, 1, 100);
    /*
    pros::Intake.move(127);
    time::forward(40, 3800);
    fwtarget = 95;
    pid::turn(-245, 127, 1, 1000);
    pid::turn(-245, 127, 1, 500);
    pid::turn(-245, 127, 1, 250);
    pid::turn(-245, 127, 1, 100);
    pros::delay(700);
    pros::Intake.move(-127);
    pros::delay(165);
    fwtarget = 100;
    pros::Intake.brake();
    pros::delay(700);
    pros::Intake.move(-127);
    pros::delay(165);
    fwtarget = 103;
    pros::Intake.brake();
    pros::delay(700);
    pros::Intake.move(-127);
    pros::delay(165);
    pros::Intake.brake();
    pid::turn(-140, 127, 1, 1000);
    time::forward(-127, 1100);
    pid::turn(-145, 127, 1, 700);
    */
    pros::expansionMid.set_value(1);
    pros::expansionLeft.set_value(1);
    pros::expansionRight.set_value(1);
  }
}