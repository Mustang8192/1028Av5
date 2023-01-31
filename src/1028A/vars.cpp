#include "1028A/vars.h"

//Robot Options
int autonSelect = 0;
int flywheelMode = 1;

//Drive
int flywheelstate = 0;

//Tasks
std::map<std::string, std::unique_ptr<pros::Task>> tasks;
std::atomic<double> v;

//Flywheel
int FWcontinueTask = 1;
double fwtarget;

//Odometry
okapi::OdomState CurrentPosition;
std::shared_ptr<okapi::OdomChassisController> chassis;
std::atomic<double> y = 0;
std::atomic<double> x = 0;
std::string recorded_points;
double lastPowerError;
double lastTurnError;
double radius = 0.5;
double y2;
double x2;
int max;
int min;
double true_max;
double scalar;
std::vector<double> pos1;
std::vector<double> pos2;
double dx;
double dy;
float d1;
float D2;
float discriminant;
float x11;
float y11;
float x12;
float y12;
std::vector<double> intersection1;
std::vector<double> intersection2;
float distance1;
float distance2;
std::vector<double> calc1;
std::vector<double> calc2;
double new_y;
double new_x;
double heading;
int timeForMove_To = 0;
std::deque<double> motion;
double y_error;
double x_error;
int coefficient;
double last_x;
double last_y;
double heading2;
double imu_error;
double sum;
double motion_average;
double phi;
double powerError;
double powerPurePursuit;
double turnError;
double turnPurePursuit;
std::vector<double> end;
std::vector<double> start;
std::vector<double> target;
std::vector<double> cur{(float)y, (float)x};
double headingPurePursuit;
std::vector<double> pose;