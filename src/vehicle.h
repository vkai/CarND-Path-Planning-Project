#ifndef VEHICLE_H
#define VEHICLE_H

#include "utils.h"

using namespace std;

class Vehicle {

public:
  Vehicle();
  Vehicle(double x, double y, double s, double d, double speed);
  ~Vehicle(){};

  void setPrev(vector<double> prev_s, vector<double> prev_d);
  Lane get_lane();

  double x;
  double y;
  double s;
  double d;
  double speed;
  vector<double> prev_s;
  vector<double> prev_d;
};


#endif