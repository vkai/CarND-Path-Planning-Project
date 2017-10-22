#ifndef MAP_H
#define MAP_H

#include "spline.h"
#include <vector>

using namespace std;

class Map {
  tk::spline spline_x;
	tk::spline spline_y;
	tk::spline spline_dx;
	tk::spline spline_dy;

public:
  Map(vector<double> map_waypoints_x,
      vector<double> map_waypoints_y,
      vector<double> map_waypoints_s,
      vector<double> map_waypoints_dx,
      vector<double> map_waypoints_dy);
  ~Map(){};

  vector<double> getXY(double s, double d);
};

#endif