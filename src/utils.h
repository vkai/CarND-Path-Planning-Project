#ifndef UTILS_H
#define UTILS_H

#include <math.h>
#include <vector>

using namespace std;

// Max s value before wrapping around the track back to 0
#define MAX_S       6945.554
// Maximum speed in mph
#define MAX_SPEED   20.0
// Planning horizon in seconds
#define HORIZON     2.0
// Time increment
#define TIME_INCR   0.02
// Lane width
#define LANE_WIDTH  4.0

enum State {
  KEEP_LANE,
  CHANGE_LEFT,
  CHANGE_RIGHT
};

enum Lane {
  LEFT,
  CENTER,
  RIGHT
};


#endif