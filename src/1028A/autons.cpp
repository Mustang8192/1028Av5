#include "1028A/init.h"
#include "1028A/robot.h"
#include "pros/rtos.h"

void _1028A::robot::auton(){
    autonSelect = 8;
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
        pros::Intake.move(127);
        time::forward(-100, 100);
        pros::delay(400);
        rightOnly(127, 375);
        time::forward(70, 550);
        pid::turn(75, 127, 1, 2000);
        pid::turn(75, 127, 1, 1000);
        pid::turn(75, 127, 1, 500);
        pid::turn(75, 127, 1, 100);
        time::forward(-127, 535);
        pros::delay(1000);
        time::forward(127, 200);
        fwtarget = 95;
        task::start("fw", flywheel::startFlywheelTask);
        pid::turn(-12.5, 127, 1, 1000);
        pid::turn(-12.5, 127, 1, 500);
        pid::turn(-12.5, 127, 1, 250);
        pid::turn(-12.5, 127, 1, 100);
        time::forward(127, 725);
        pros::delay(700);
        pros::Intake.move(-127);
        pros::delay(165);
        fwtarget = 97;
        pros::Intake.brake();
        pros::delay(700);
        pros::Intake.move(-127);
        pros::delay(165);
        fwtarget = 99;
        pros::Intake.brake();
        pros::delay(700);
        pros::Intake.move(-127);
        pros::delay(165);
        pros::Intake.brake();
        pros::delay(1000);
        time::forward(-127, 600);
        fwtarget = 95;
        pid::turn(40, 127, 1, 1000);
        pid::turn(40, 127, 1, 500);
        pid::turn(40, 127, 1, 250);
        pid::turn(40, 127, 1, 100);
        pros::Intake.move(127);
        time::forward(38, 3580);
        pros::delay(2000);
        pid::turn(-50, 127, 1, 1000);
        pid::turn(-50, 127, 1, 500);
        pid::turn(-50, 127, 1, 250);
        pid::turn(-50, 127, 1, 100);
        pros::delay(700);
        pros::Intake.move(-127);
        pros::delay(165);
        fwtarget = 102;
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
        pid::turn(7, 127, 1, 1000);
        pid::turn(7, 127, 1, 500);
        pid::turn(7, 127, 1, 250);
        pid::turn(7, 127, 1, 100);
        fwtarget = 95;
        pros::Intake.move(127);
        pros::delay(500);
        time::forward(25, 6450);
        pros::delay(2000);
        pid::turn(-100.5, 127, 1, 1000);
        pid::turn(-100.5, 127, 1, 500);
        pid::turn(-100.5, 127, 1, 250);
        pid::turn(-100.5, 127, 1, 100);
        pros::delay(700);
        pros::Intake.move(-127);
        pros::delay(165);
        fwtarget = 98;
        pros::Intake.brake();
        pros::delay(700);
        pros::Intake.move(-127);
        pros::delay(165);
        fwtarget = 100;
        pros::Intake.brake();
        pros::delay(700);
        pros::Intake.move(-127);
        pros::delay(165);
        pros::Intake.brake();
        pros::delay(500);
        fwtarget = 103;
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
        time::forward(-127, 835);
        pros::delay(1000);
        pid::turn(-180, 127, 1, 1000);
        pid::turn(-180, 127, 1, 500);
        pid::turn(-180, 127, 1, 250);
        pid::turn(-180, 127, 1, 100);
        pros::Intake.move(127);
        time::forward(-100, 400);
        pros::delay(400);
        rightOnly(127, 375);
        time::forward(70, 550);
        pid::turn(-75, 127, 1, 1000);
        pid::turn(-75, 127, 1, 500);
        pid::turn(-75, 127, 1, 250);
        pid::turn(-75, 127, 1, 100);
        time::forward(-127, 535);
        pros::delay(600);
        time::forward(127, 200);
        pid::turn(-150, 127, 1, 1000);
        pid::turn(-150, 127, 1, 500);
        pid::turn(-150, 127, 1, 250);
        pid::turn(-150, 127, 1, 100);
        pros::expansionMid.set_value(1);
        pros::expansionLeft.set_value(1);
        pros::expansionRight.set_value(1);


        //Skills
        //Roller Turn
        /*
        pros::Inertial.set_rotation(0);
        pros::Intake.move(127);
        time::forward(-100, 100);
        pros::delay(550);
        time::forward(127, 500);
        pros::delay(200);
        pid::turn(75, 127, 1, 2000);
        pid::turn(75, 127, 1, 1000);
        pid::turn(75, 127, 1, 500);
        pid::turn(75, 127, 1, 100);
        time::forward(-127, 535);
        pros::delay(700);
        time::forward(127, 200);
        fwtarget = 100;
        task::start("fw", flywheel::startFlywheelTask);
        pros::Intake.move(0);
        pid::turn(-10, 127, 1, 1000);
        pid::turn(-10, 127, 1, 500);
        pid::turn(-10, 127, 1, 250);
        pid::turn(-10, 127, 1, 100);
        time::forward(127, 725);
        pros::delay(700);
        pros::Intake.move(-127);
        pros::delay(165);
        pros::Intake.brake();
        pros::delay(700);
        pros::Intake.move(-127);
        pros::delay(165);
        pros::Intake.brake();
        pros::delay(700);
        pros::Intake.move(-127);
        pros::delay(165);
        pros::Intake.brake();
        pros::delay(500);
        pid::turn(100, 127, 1, 1000);
        pid::turn(100, 127, 1, 500);
        pid::turn(100, 127, 1, 250);
        pid::turn(100, 127, 1, 100);
        */
        /*
        time::forward(-127, 1200);
        pid::turn(30, 127, 1, 2000);
        pid::turn(, 127, 1, 1000);
        pid::turn(75, 127, 1, 500);
        pid::turn(75, 127, 1, 100);
        */
        /*
        pid::turn(55, 127, 1, 1000);
        pid::turn(55, 127, 1, 500);
        pid::turn(55, 127, 1, 250);
        pid::turn(55, 127, 1, 100);
        pros::Intake.move(127);
        time::forward(40, 2300);
        pid::turn(-55, 127, 1, 1000);
        pid::turn(-55, 127, 1, 500);
        pid::turn(-55, 127, 1, 250);
        pid::turn(-55, 127, 1, 100);
        pros::delay(1000);
        pros::delay(700);
        pros::Intake.move(-127);
        pros::delay(165);
        pros::Intake.brake();
        pros::delay(700);
        pros::Intake.move(-127);
        pros::delay(165);
        pros::Intake.brake();
        pros::delay(700);
        pros::Intake.move(-127);
        pros::delay(165);
        pros::Intake.brake();
        */
        /*
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
        */
    }   
}