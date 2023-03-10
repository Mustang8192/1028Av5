#ifndef PATHFINDER_MOD_TANK_H_DEF
#define PATHFINDER_MOD_TANK_H_DEF

#include "1028A/okapi/pathfinder/include/pathfinder/lib.h"
#include "1028A/okapi/pathfinder/include/pathfinder/structs.h"

CAPI void pathfinder_modify_tank(Segment *original, int length, Segment *left, Segment *right, double wheelbase_width);

#endif