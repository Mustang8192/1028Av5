#include "1028A/init.h"
#include "1028A/robot.h"

void _1028A::robot::auton(){
    autonSelect = 2;
    if (autonSelect == 1){
        //Station Side
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
        //Right Side
        flywheelMode = 2;
        fwtarget = 110;
        task::start("fw", flywheel::startFlywheelTask);
        pros::Intake.move(127);
        time::forward(45, 1000);
        pros::delay(1700);
        pros::Intake.move(-127);
        pros::delay(165);
        fwtarget = 112;
        pros::Intake.brake();
        pros::delay(1200);
        pros::Intake.move(-127);
        pros::delay(165);
        pros::Intake.brake();
        fwtarget = 115;
        pros::delay(1400);
        pros::Intake.move(-127);
        pros::delay(165);
        pros::Intake.brake();
        pros::delay(500);
        time::turn(-127, 240);
        pros::delay(500);
        time::forward(-80, 600);
        time::turn(127, 230);
        pros::delay(500);
        time::forward(-100, 150);
        pros::Intake.move(127);
        pros::delay(300);
        pros::Intake.brake();
        time::forward(100, 150);
        pros::delay(250);
        fwtarget = 100;
        pid::turn(-74, 127, 1, 1000);
        pros::delay(250);
        pros::Intake.move(127);
        time::forward(127, 700);
        time::forward(40, 1200);
        pros::delay(250);
        time::turn(127, 290);
        pros::delay(700);
        pros::Intake.move(-127);
        pros::delay(165);
        pros::Intake.brake();
        fwtarget = 107;
        pros::delay(1200);
        pros::Intake.move(-127);
        pros::delay(165);
        pros::Intake.brake();
    }
    else if (autonSelect == 3){
        //Solo Win Point
    }
    else if (autonSelect == 8){
        //Skills
        //Roller Turn
        pros::Intake.move(127);
        time::forward(-100, 100);
        pros::delay(100);
        time::forward(127, 500);
        pros::delay(400);
        //Turn toward 2nd Roller
        pid::turn(280, 127, 1, 1000);
        pid::turn(280, 127, 1, 500);
        pid::turn(280, 127, 1, 100);
        pid::turn(280, 127, 1, 50);
        //Spinning 2nd Roller
        time::forward(-100, 315);
        pros::delay(175);
        time::forward(127, 200);
        //Turning toward goal
        fwtarget = 102;
        task::start("fw", flywheel::startFlywheelTask);
        pros::delay(250);
        pid::turn(195, 127, 1, 1150);
        pid::turn(195, 127, 1, 250);
        pid::turn(195, 127, 1, 100);
        pros::Intake.move(0);
        pros::delay(200);
        time::forward(127, 700);
        pros::delay(500);
        //Shooting 1st Volley
        pros::Intake.move(-127);
        pros::delay(140);
        pros::Intake.brake();
        pros::delay(500);
        pros::Intake.move(-127);
        pros::delay(140);
        pros::Intake.brake();
        pros::delay(500);
        pros::Intake.move(-127);
        pros::delay(140);
        pros::Intake.brake();
        pros::delay(500);
        
    }
}