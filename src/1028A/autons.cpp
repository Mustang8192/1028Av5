#include "1028A/init.h"
#include "1028A/robot.h"

void _1028A::robot::auton(){
    if (autonSelect == 1){
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
    }
    else if (autonSelect == 2){
        fwtarget = 107;
        task::start("fw", flywheel::startFlywheelTask);
        pros::Intake.move(127);
        time::forward(45, 850);
        pros::delay(2300);
        pros::Intake.move(-127);
        pros::delay(165);
        pros::Intake.brake();
        pros::delay(1200);
        pros::Intake.move(-127);
        pros::delay(165);
        pros::Intake.brake();
        pros::delay(1400);
        pros::Intake.move(-127);
        pros::delay(165);
        pros::Intake.brake();
        pros::delay(500);
        time::turn(-127, 240);
        pros::delay(500);
        time::forward(-80, 600);
        time::turn(127, 250);
        pros::delay(500);
        fwtarget = 100;
        time::forward(-100, 150);
        pros::Intake.move(-127);
        pros::delay(190);
        pros::Intake.brake();
        time::forward(100, 150);
        pros::delay(250);
        time::turn(-127, 225);
        pros::delay(250);
        pros::Intake.move(127);
        time::forward(127, 700);
        time::forward(40, 1200);
        pros::delay(250);
        time::turn(127, 283);
        pros::delay(700);
        pros::Intake.move(-127);
        pros::delay(165);
        pros::Intake.brake();
        pros::delay(1200);
        pros::Intake.move(-127);
        pros::delay(165);
        pros::Intake.brake();
    }
}