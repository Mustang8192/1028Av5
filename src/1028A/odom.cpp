#include "1028A/init.h"
float to_rad(float angle_deg){
  return(angle_deg/(180.0/M_PI));
}

void _1028A::odom::set_physical_distances(float ForwardTracker_center_distance, float SidewaysTracker_center_distance){
  ForwardTracker_center_distance = ForwardTracker_center_distance;
  SidewaysTracker_center_distance = SidewaysTracker_center_distance;
}

void _1028A::odom::set_position(float X_position, float Y_position, float orientation_deg, float ForwardTracker_position, float SidewaysTracker_position){
  ForwardTracker_position = ForwardTracker_position;
  SideWaysTracker_position = SidewaysTracker_position;
  X_position = X_position;
  Y_position = Y_position;
  orientation_deg = orientation_deg;
}

void _1028A::odom::update_position(float ForwardTracker_position, float SidewaysTracker_position, float orientation_deg){
  float Forward_delta = ForwardTracker_position-ForwardTracker_position;
  float Sideways_delta = SidewaysTracker_position-SideWaysTracker_position;
  ForwardTracker_position=ForwardTracker_position;
  SideWaysTracker_position=SidewaysTracker_position;
  float orientation_rad = to_rad(orientation_deg);
  float prev_orientation_rad = to_rad(orientation_deg);
  float orientation_delta_rad = orientation_rad-prev_orientation_rad;
  orientation_deg=orientation_deg;

  float local_X_position;
  float local_Y_position;

  if (orientation_delta_rad == 0) {
    local_X_position = Sideways_delta;
    local_Y_position = Forward_delta;
  } else {
    local_X_position = (2*sin(orientation_delta_rad/2))*((Sideways_delta/orientation_delta_rad)+SidewaysTracker_center_distance); 
    local_Y_position = (2*sin(orientation_delta_rad/2))*((Forward_delta/orientation_delta_rad)+ForwardTracker_center_distance);
  }

  float local_polar_angle;
  float local_polar_length;

  if (local_X_position == 0 && local_Y_position == 0){
    local_polar_angle = 0;
    local_polar_length = 0;
  } else {
    local_polar_angle = atan2(local_Y_position, local_X_position); 
    local_polar_length = sqrt(pow(local_X_position, 2) + pow(local_Y_position, 2)); 
  }

  float global_polar_angle = local_polar_angle - prev_orientation_rad - (orientation_delta_rad/2);

  float X_position_delta = local_polar_length*cos(global_polar_angle); 
  float Y_position_delta = local_polar_length*sin(global_polar_angle);

  X_position+=X_position_delta;
  Y_position+=Y_position_delta;
}