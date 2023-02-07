#include "main.h"
#include "okapi/api/chassis/controller/odomChassisController.hpp"
#include "okapi/api/odometry/odomState.hpp"
#include <deque>
#include <map>

// Ui
extern int autonSelect;
extern int flywheelMode;

// Drive
extern int flywheelstate;

// Tasks
extern std::map<std::string, std::unique_ptr<pros::Task>> tasks;
extern std::atomic<double> v;

// Flywheel
extern int FWcontinueTask;
extern int FWoffset;
extern double fwtarget;

// Odometry
extern okapi::OdomState CurrentPosition;
extern std::shared_ptr<okapi::OdomChassisController> chassis;
extern std::atomic<double> y;
extern std::atomic<double> x;
extern std::string recorded_points;
extern double lastPowerError;
extern double lastTurnError;
extern double radius;
extern double y2;
extern double x2;
extern int max;
extern int min;
extern double true_max;
extern double scalar;
extern std::vector<double> pos1;
extern std::vector<double> pos2;
extern double dx;
extern double dy;
extern float d1;
extern float D2;
extern float discriminant;
extern float x11;
extern float y11;
extern float x12;
extern float y12;
extern std::vector<double> intersection1;
extern std::vector<double> intersection2;
extern float distance1;
extern float distance2;
extern std::vector<double> calc1;
extern std::vector<double> calc2;
extern double new_y;
extern double new_x;
extern double heading;
extern int timeForMove_To;
extern std::deque<double> motion;
extern double y_error;
extern double x_error;
extern int coefficient;
extern double last_x;
extern double last_y;
extern double heading2;
extern double imu_error;
extern double sum;
extern double motion_average;
extern double phi;
extern double powerError;
extern double powerPurePursuit;
extern double turnError;
extern double turnPurePursuit;
extern std::vector<double> end;
extern std::vector<double> start;
extern std::vector<double> target;
extern std::vector<double> cur;
extern double headingPurePursuit;
extern std::vector<double> pose;
extern float ForwardTracker_center_distance;
extern float SidewaysTracker_center_distance;
extern float ForwardTracker_position;
extern float SideWaysTracker_position;
extern float X_position;
extern float Y_position;
extern float orientation_deg;