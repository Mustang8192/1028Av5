#include "1028A/init.h"
#include "1028A/robot.h"

#define SQD(n) pow(n, 2)
#define TO_RAD(n) n *M_PI / 180

void _1028A::odom::Record(void *ptr) {
  while (1) {
    recorded_points =
        CurrentPosition.str(okapi::inch, "_in", okapi::degree, "_deg");
    printf("%s", const_cast<char *>(recorded_points.c_str()));

    pros::delay(250);
  }
}

void _1028A::odom::Refresh(void *ptr) {
  while (1) {
    CurrentPosition = chassis->getState();
    x = CurrentPosition.x.convert(okapi::inch);
    y = CurrentPosition.y.convert(okapi::inch);
    pros::delay(10);
  }
}

int _1028A::odom::sign(double x) {
  if (x > 0.0)
    return 1;
  if (x < 0.0)
    return -1;
  return 0;
}

double _1028A::odom::get_degrees(std::vector<double> p1,
                                 std::vector<double> p2) {
  y2 = p1[0] - p2[0];
  x2 = p1[1] - p2[1];
  return atan2(-x2, y2) * 180 / M_PI;
}

double _1028A::odom::distance(std::vector<double> p1, std::vector<double> p2) {
  return sqrt(SQD(p1[0] - p2[0]) + SQD(p1[1] - p2[1]));
}

void _1028A::odom::Drive(int power, int turn) {

  int powers[]{power + turn, power - turn, power + turn, power - turn};

  max = *std::max_element(powers, powers + 4);
  min = abs(*std::min_element(powers, powers + 4));

  true_max = double(std::max(max, min));
  scalar = (true_max > 127) ? 127 / true_max : 1;

  pros::LeftFront.move((power + turn) * scalar);
  pros::RightFront.move((power - turn) * scalar);
  pros::LeftMid.move((power + turn) * scalar);
  pros::RightMid.move((power - turn) * scalar);
  pros::LeftBack.move((power - turn) * scalar);
  pros::RightBack.move((power - turn) * scalar);
}

std::vector<double> _1028A::odom::get_intersection(std::vector<double> start,
                                                   std::vector<double> end,
                                                   std::vector<double> cur,
                                                   double radius) {
  pos1 = {(start[0] - cur[0]), (start[1] - cur[1])};
  pos2 = {(end[0] - cur[0]), (end[1] - cur[1])};

  dx = pos2[0] - pos1[0];
  dy = pos2[1] - pos1[1];
  d1 = sqrt(SQD(dx) + SQD(dy));
  D2 = pos1[0] * pos2[1] - pos2[0] * pos1[1];
  discriminant = abs(SQD(radius) * SQD(d1) - SQD(D2));

  x11 = (D2 * dy + sign(dy) * dx * sqrt(discriminant)) / SQD(d1);
  y11 = (-D2 * dx + abs(dy) * sqrt(discriminant)) / SQD(d1);
  x12 = (D2 * dy - sign(dy) * dx * sqrt(discriminant)) / SQD(d1);
  y12 = (-D2 * dx - abs(dy) * sqrt(discriminant)) / SQD(d1);

  intersection1 = {x11, y11};
  intersection2 = {x12, y12};

  distance1 = distance(pos2, intersection1);
  distance2 = distance(pos2, intersection2);

  calc1 = {(x11 + cur[0]), (y11 + cur[1])};
  calc2 = {(x12 + cur[0]), (y12 + cur[1])};

  if (distance1 < distance2)
    return calc1;
  else if (distance1 > distance2)
    return calc2;
  else
    return {1.0};
}

void _1028A::odom::move_to(std::vector<double> pose, double stop_threshold,
                           bool pure_pursuit, std::vector<double> speeds) {
  new_y = pose[0];
  new_x = pose[1];
  heading = pose[2];

  y_error = new_y - y;
  x_error = -(new_x - x);
  coefficient = 0;
  last_x = x;
  last_y = y;

  timeForMove_To = 0;

  heading2 = (heading < 0) ? heading + 360 : heading - 360;
  if (pure_pursuit)
    heading = (abs(pros::Inertial.get_rotation() - heading) <
               abs(pros::Inertial.get_rotation() - heading2))
                  ? heading
                  : heading2;
  imu_error = -(pros::Inertial.get_rotation() - heading);

  while (abs(y_error) > 0.2 || abs(x_error) > 0.2 || abs(imu_error) > 0.5) {

    if ((int)motion.size() == 10)
      motion.pop_front();
    motion.push_back(abs(last_x - x) + abs(last_y - y));
    sum = 0;
    for (int i = 0; i < motion.size(); i++)
      sum += motion[i];
    motion_average = sum / 10;
    if (motion_average < stop_threshold && timeForMove_To > 100)
      break;

    last_x = x;
    last_y = y;

    phi = TO_RAD(pros::Inertial.get_rotation());
    powerError = y_error * std::cos(phi) + x_error * std::sin(phi);
    powerPurePursuit =
        _1028A::pid::math(powerError, lastPowerError, 1, 0, 1, 127) * speeds[0];
    // double strafe = strafe_PD.get_value(x_error * std::cos(phi) - y_error *
    // std::sin(phi)) * speeds[1];
    turnError = imu_error;
    turnPurePursuit =
        _1028A::pid::math(turnError, lastTurnError, 1, 0, 1, 127) * speeds[2];
    Drive(powerPurePursuit, turnPurePursuit);

    imu_error = -(pros::Inertial.get_rotation() - heading);
    y_error = new_y - y;
    x_error = -(new_x - x);

    if (pure_pursuit)
      return;
    pros::delay(5);
    timeForMove_To += 5;

    lastPowerError = powerError;
    lastTurnError = turnError;
  }
  _1028A::robot::DriveStop();
}

void _1028A::odom::move_to_pure_pursuit(std::vector<std::vector<double>> points,
                                        std::vector<double> final_point,
                                        std::vector<double> speeds) {
  for (int index = 0; index < points.size() - 1; index++) {
    start = points[index];
    end = points[index + 1];
    while (distance(cur, end) > radius) {
      target = get_intersection(start, end, cur, radius);
      heading = get_degrees(target, cur);

      pose = {target[0], target[1], heading};
      move_to(pose, 0, true, speeds);
      cur = {(float)y, (float)x};
      pros::delay(5);
    }
  }
  move_to(final_point, 0.15, true, speeds);
  _1028A::robot::DriveStop();
}

int _1028A::odom::sign(int num) {
  if (num >= 0)
    return 1;
  else
    return -1;
}

// finds the minimum angle to turn between two angles
double _1028A::odom::find_min_angle(double target_angle,
                                    double current_heading) {
  double min_angle = target_angle - current_heading;

  if (min_angle > 180 || min_angle < -180)
    min_angle = -1 * sign(min_angle) * (360 - abs(min_angle));

  return min_angle;
}

std::vector<std::vector<double>>
_1028A::odom::smoothing(std::vector<std::vector<double>> path,
                        double weight_data, double weight_smooth,
                        double tolerance) {
  std::vector<std::vector<double>> smoothed_path = path;
  double change = tolerance;

  while (change >= tolerance) {
    change = 0.0;

    for (int i = 1; i < path.size() - 1; i++) {
      for (int j = 0; j < path[i].size(); j++) {
        const double aux = smoothed_path[i][j];
        smoothed_path[i][j] +=
            weight_data * (path[i][j] - smoothed_path[i][j]) +
            weight_smooth * (smoothed_path[i - 1][j] + smoothed_path[i + 1][j] -
                             (2.0 * smoothed_path[i][j]));
        change += abs(aux - smoothed_path[i][j]);
      }
    }
  }

  return smoothed_path;
}

std::vector<std::vector<double>>
_1028A::odom::auto_smooth(std::vector<std::vector<double>> path,
                          double max_angle) {
  double current_max = 0;
  double param = 0.01;
  std::vector<std::vector<double>> new_path = path;
  bool first_loop = true;

  // let counter = 0;

  while (current_max >= max_angle || first_loop) { // && (counter <= 15) {
    param += 0.01;
    first_loop = false;

    // counter += 1;
    // console.log(`this is the ${counter} iteration`);

    new_path = smoothing(path, 0.1, param, 0.1);
    current_max = 0;

    for (int i = 1; i < new_path.size() - 2; i++) {
      double angle1 = atan2(new_path[i][1] - new_path[i - 1][1],
                            new_path[i][0] - new_path[i - 1][0]) *
                      180 / M_PI;
      if (angle1 < 0)
        angle1 += 360;

      double angle2 = atan2(new_path[i + 1][1] - new_path[i][1],
                            new_path[i + 1][0] - new_path[i][0]) *
                      180 / M_PI;
      if (angle2 < 0)
        angle2 += 360;

      const double min_angle = find_min_angle(angle2, angle1);
      if (abs(min_angle) > current_max)
        current_max = abs(min_angle);
    }
  }

  return new_path;
}
