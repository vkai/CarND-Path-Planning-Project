#include "map.h"

using namespace std;

Map::Map( vector<double> map_waypoints_x,
          vector<double> map_waypoints_y,
          vector<double> map_waypoints_s,
          vector<double> map_waypoints_dx,
          vector<double> map_waypoints_dy) {
  
  tk::spline spline_x;
  tk::spline spline_y;
  tk::spline spline_dx;
  tk::spline spline_dy;
  
  spline_x.set_points(map_waypoints_s, map_waypoints_x);
  spline_y.set_points(map_waypoints_s, map_waypoints_y);
  spline_dx.set_points(map_waypoints_s, map_waypoints_dx);
  spline_dy.set_points(map_waypoints_s, map_waypoints_dy);

  this->spline_x = spline_x;
  this->spline_y = spline_y;
  this->spline_dx = spline_dx;
  this->spline_dy = spline_dy;
}

vector<double> Map::getXY(double s, double d) {
  double x = this->spline_x(s);
	double y = this->spline_y(s);
	double dx = this->spline_dx(s);
	double dy = this->spline_dy(s);

	x += d * dx;
	y += d * dy;

	return {x, y};
}