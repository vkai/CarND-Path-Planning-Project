#ifndef ROAD_H
#define ROAD_H

#include <vector>
#include "utils.h"
#include "vehicle.h"


using namespace std;

class Road {
public:
  vector<vector<Vehicle>> lanes;

  Road();
  ~Road(){};
  void setRoad(vector<Vehicle> vehicles);
  vector<double> findClosestDistanceForwardAndBehind(Vehicle self, int lane);
};

#endif