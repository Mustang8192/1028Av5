#include "1028A/init.h"
#include "1028A/robot.h"

void _1028A::robot::auton() {
  pros::Inertial.set_rotation(0);
  autonSelect = 2;
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
    fwtarget = 107;
    task::start("fw", flywheel::startFlywheelTask);
    pros::Intake.move(127);
    time::forward(45, 1250);
    pros::delay(1800);
    pros::Intake.move(-127);
    pros::delay(145);
    fwtarget = 109;
    pros::Intake.brake();
    pros::delay(700);
    pros::Intake.move(-127);
    pros::delay(145);
    pros::Intake.brake();
    fwtarget = 114;
    pros::delay(800);
    pros::Intake.move(-127);
    pros::delay(145);
    pros::Intake.brake();
    pros::delay(500);
    time::turn(-127, 230);
    pros::delay(500);
    time::forward(-80, 750);
    time::turn(127, 260);
    pros::delay(500);
    pros::Intake.move(127);
    pros::delay(350);
    pros::Intake.brake();
    time::forward(100, 150);
    pros::delay(250);
    fwtarget = 103;
    pid::turn(-72, 127, 1, 800);
    pros::delay(250);
    pros::Intake.move(127);
    time::forward(127, 600);
    time::forward(40, 1200);
    pros::delay(250);
    pid::turn(16.99, 127, 1, 800);
    pros::delay(700);
    pros::Intake.move(-127);
    pros::delay(165);
    pros::Intake.brake();
    fwtarget = 111;
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
    pid::forward(-925, 127, 1.5, 800);
    pid::turn(3, 127, 1, 700);
    pid::forward(400, 127, 1.5, 1000);
    pid::turn(-45, 127, 1, 800);
    pros::Intake.move(-127);
    pros::delay(165);
    pros::Intake.brake();
    fwtarget = 113;
    pros::delay(900);
    pros::Intake.move(-127);
    pros::delay(165);
    pros::Intake.brake();
    fwtarget = 121;
    pros::delay(500);
    pid::turn(53, 127, 1, 600);
    pros::Intake.move(127);
    time::forward(40, 2600);
    fwtarget = 115;
    pros::delay(600);
    pid::turn(-76, 127, 1, 800);
    pros::Intake.move(-127);
    pros::delay(165);
    pros::Intake.brake();
    fwtarget = 118;
    pros::delay(700);
    pros::Intake.move(-127);
    pros::delay(165);
    fwtarget = 120;
    pros::Intake.brake();
    pros::delay(700);
    pros::Intake.move(-127);
    pros::delay(165);
    pros::Intake.brake();
    pid::turn(-136, 127, 1, 700);
    time::forward(-127, 580);
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
    fwtarget = 93;
    task::start("fw", flywheel::startFlywheelTask);
    pid::turn(84, 127, 1, 2000);
    pid::turn(84, 127, 1, 1000);
    pid::turn(84, 127, 1, 500);
    pid::turn(84, 127, 1, 100);
    time::forward(-80, 200);
    pros::delay(550);
    time::forward(127, 60);
    pid::turn(-3, 127, 1, 1200);
    pid::turn(-3, 127, 1, 600);
    pid::turn(-3, 127, 1, 250);
    pid::turn(-3, 127, 1, 100);
    // time::forward(127, 725);
    pros::Rotation.reset_position();
    pid::forward(1500, 127, 1.5, 1200);
    pid::turn(-5, 127, 1, 1200);
    pid::turn(-5, 127, 1, 600);
    pid::turn(-5, 127, 1, 250);
    pid::turn(-5, 127, 1, 100);
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
    // time::forward(-127, 600);
    pros::Rotation.reset_position();
    pid::forward(-1475, 127, 1.5, 1200);
    fwtarget = 95;
    pid::turn(49, 127, 1, 1000);
    pid::turn(49, 127, 1, 500);
    pid::turn(49, 127, 1, 250);
    pid::turn(49, 127, 1, 100);
    pros::Intake.move(127);
    // time::forward(38, 3100);
    pros::Rotation.reset_position();
    pid::forward(3200, 38, 1, 3500);
    pros::delay(2000);
    pid::turn(-56, 127, 1, 1000);
    pid::turn(-56, 127, 1, 500);
    pid::turn(-56, 127, 1, 250);
    pid::turn(-56, 127, 1, 100);
    pros::delay(700);
    pros::Intake.move(-127);
    pros::delay(140);
    fwtarget = 100;
    pros::Intake.brake();
    pros::delay(700);
    pros::Intake.move(-127);
    pros::delay(140);
    pros::Intake.brake();
    fwtarget = 105;
    pros::delay(700);
    pros::Intake.move(-127);
    pros::delay(140);
    pros::Intake.brake();
    pros::delay(500);
    pid::turn(3.5, 127, 1, 1000);
    pid::turn(3.5, 127, 1, 500);
    pid::turn(3.5, 127, 1, 250);
    pid::turn(3.5, 127, 1, 100);
    fwtarget = 95;
    pros::Intake.move(127);
    pros::delay(500);
    time::forward(25, 6500);
    pros::delay(1000);
    pid::turn(-108, 127, 1, 1000);
    pid::turn(-108, 127, 1, 500);
    pid::turn(-108, 127, 1, 250);
    pid::turn(-108, 127, 1, 100);
    pros::delay(700);
    pros::Intake.move(-127);
    pros::delay(140);
    pros::Intake.brake();
    pros::delay(700);
    pros::Intake.move(-127);
    pros::delay(140);
    fwtarget = 95;
    pros::Intake.brake();
    pros::delay(700);
    pros::Intake.move(-127);
    pros::delay(140);
    pros::Intake.brake();
    pros::delay(500);
    fwtarget = 115;
    pros::Intake.brake();
    pros::delay(700);
    pros::Intake.move(-127);
    pros::delay(140);
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
    time::rightOnly(127, 300);
    time::forward(70, 550);
    pid::turn(-75, 127, 1, 1000);
    pid::turn(-75, 127, 1, 500);
    pid::turn(-75, 127, 1, 250);
    pid::turn(-75, 127, 1, 100);
    time::forward(-127, 900);
    pros::delay(400);
    time::forward(127, 225);
    pros::Intake.move(-127);
    pros::delay(140);
    pros::Intake.brake();
    pid::turn(-140, 127, 1, 1000);
    pid::turn(-140, 127, 1, 500);
    pid::turn(-140, 127, 1, 250);
    pid::turn(-140, 127, 1, 100);
    pros::expansionMid.set_value(1);
    pros::expansionSide.set_value(1);
  } else if (autonSelect == 9) {
    // Skills
    pros::Intake.move(127);
    time::forward(-100, 100);
    pros::delay(400);
    time::rightOnly(127, 300);
    time::forward(70, 545);
    fwtarget = 88;
    task::start("fw", flywheel::startFlywheelTask);
    pid::turn(84, 127, 1, 1000);
    pid::turn(84, 127, 1, 500);
    pid::turn(84, 127, 1, 100);
    pid::turn(84, 127, 1, 100);
    time::forward(-80, 200);
    pros::delay(650);
    time::forward(127, 80);
    pid::turn(-3, 127, 1, 1200);
    pid::turn(-3, 127, 1, 600);
    pid::turn(-3, 127, 1, 250);
    pid::turn(-3, 127, 1, 100);
    // time::forward(127, 725);
    pros::Rotation.reset_position();
    pid::forward(1500, 127, 1.5, 1000);
    pid::turn(-5, 127, 1, 1000);
    pid::turn(-5, 127, 1, 500);
    pid::turn(-5, 127, 1, 250);
    pid::turn(-5, 127, 1, 100);
    pros::delay(700);
    pros::Intake.move(-127);
    pros::delay(140);
    pros::Intake.brake();
    fwtarget = 115;
    pros::delay(300);
    pros::Intake.move(-127);
    pros::delay(140);
    pros::Intake.brake();
    pros::delay(100);
    pros::Intake.move(-127);
    pros::delay(300);
    pros::Intake.brake();
    pid::turn(-8, 127, 1, 500);
    pid::turn(-8, 127, 1, 250);
    pid::turn(-8, 127, 1, 100);
    pid::turn(-8, 127, 1, 50);
    pros::delay(1000);
    // time::forward(-127, 600);
    pros::Rotation.reset_position();
    pid::forward(-1050, 120, 1.5, 1200);
    fwtarget = 90;
    pid::turn(50, 127, 1, 1000);
    pid::turn(50, 127, 1, 500);
    pid::turn(50, 127, 1, 250);
    pid::turn(50, 127, 1, 100);
    pros::Intake.move(127);
    // time::forward(38, 3100);
    pros::Rotation.reset_position();
    pid::forward(2725, 38, 1, 4000);
    pros::delay(2000);
    pid::turn(-64, 127, 1, 1000);
    pid::turn(-64, 127, 1, 500);
    pid::turn(-64, 127, 1, 250);
    pid::turn(-64, 127, 1, 100);
    pros::delay(700);
    pros::Intake.move(-127);
    pros::delay(140);
    fwtarget = 92;
    pros::Intake.brake();
    pros::delay(700);
    pros::Intake.move(-127);
    pros::delay(140);
    pros::Intake.brake();
    fwtarget = 100;
    pros::delay(700);
    pros::Intake.move(-127);
    pros::delay(140);
    pros::Intake.brake();
    pros::delay(500);
    fwtarget = 100;
    pid::turn(50, 127, 0.5, 1000);
    pid::turn(50, 127, 0.5, 500);
    pid::turn(50, 127, 0.5, 250);
    pid::turn(50, 127, 0.5, 100);
    time::forward(70, 400);
    pros::Intake.move(127);
    pros::Rotation.reset_position();
    pid::forward(500, 127, 1.5, 1200);
    pros::delay(300);
    pros::Rotation.reset_position();
    fwtarget = 100;
    pid::forward(1000, 40, 1.5, 3000);
    pid::turn(-94.5, 127, 1, 1000);
    pid::turn(-94.5, 127, 1, 500);
    pid::turn(-94.5, 127, 1, 250);
    pid::turn(-94.5, 127, 1, 100);
    pros::delay(700);
    pros::Intake.move(-127);
    pros::delay(140);
    fwtarget = 104;
    pros::Intake.brake();
    pros::delay(700);
    pros::Intake.move(-127);
    pros::delay(140);
    pros::Intake.brake();
    fwtarget = 105;
    pros::delay(700);
    pros::Intake.move(-127);
    pros::delay(140);
    pros::Intake.brake();
    pros::delay(500);
    pid::turn(-220, 127, 1, 1000);
    pid::turn(-220, 127, 1, 500);
    pid::turn(-220, 127, 1, 250);
    pid::turn(-220, 127, 1, 100);
    pros::Intake.brake();
    pros::delay(500);
    time::forward(70, 400);
    pros::Intake.move(127);
    pros::Rotation.reset_position();
    pid::forward(500, 127, 1.5, 1200);
    pros::delay(300);
    pros::Rotation.reset_position();
    pid::forward(800, 40, 1.5, 3000);
    pid::turn(-178.5, 127, 1, 1000);
    pid::turn(-178.5, 127, 1, 500);
    pid::turn(-178.5, 127, 1, 250);
    pid::turn(-178.5, 127, 1, 100);
    fwtarget = 94;
    pros::delay(700);
    pros::Intake.move(-127);
    pros::delay(145);
    fwtarget = 95;
    pros::Intake.brake();
    pros::delay(700);
    pros::Intake.move(-127);
    pros::delay(145);
    pros::Intake.brake();
    fwtarget = 107;
    pros::delay(700);
    pros::Intake.move(-127);
    pros::delay(145);
    pros::Intake.brake();
    pros::delay(500);
    fwtarget = 106;
    pid::turn(-165, 127, 1, 1000);
    pid::turn(-165, 127, 1, 500);
    pid::turn(-165, 127, 1, 250);
    pid::turn(-165, 127, 1, 100);
    pros::Rotation.reset_position();
    pid::forward(-990, 127, 1, 700);
    pros::delay(500);
    pros::Intake.move(127);
    pid::turn(-90, 127, 1, 1000);
    pid::turn(-90, 127, 1, 500);
    pid::turn(-90, 127, 1, 250);
    pid::turn(-90, 127, 1, 100);
    time::forward(-100, 200);
    pros::delay(400);
    pros::Intake.brake();
    time::leftOnly(-127, 250);
    time::forward(70, 550);
    pid::turn(-184, 127, 1, 1000);
    pid::turn(-184, 127, 1, 500);
    pid::turn(-184, 127, 1, 100);
    pid::turn(-184, 127, 1, 100);
    pros::Intake.move(127);
    time::forward(-80, 300);
    pros::delay(400);
    time::forward(127, 350);
    pros::delay(250);
    pid::turn(-120, 127, 1, 500);
    pros::expansionLeft.set_value(1);
    pros::expansionRight.set_value(1);
    pros::expansionSide.set_value(1);
    pros::expansionMid.set_value(1);
    pros::expansionLeft.set_value(1);
    pros::expansionRight.set_value(1);
  }
}