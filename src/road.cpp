#include "road.h"

using namespace std;

Road::Road() {
}

void Road::setRoad(vector<Vehicle> vehicles) {
  vector<vector<Vehicle>> lanes;
  vector<Vehicle> left_lane;
  vector<Vehicle> center_lane;
  vector<Vehicle> right_lane;
  
  lanes.push_back(left_lane);
  lanes.push_back(center_lane);
  lanes.push_back(right_lane);
  for(int i = 0; i < vehicles.size(); i++) {
    Vehicle v = vehicles[i];
    lanes[v.get_lane()].push_back(v);
  }

  for(int i = 0; i < lanes.size(); i++) {
    std::sort(lanes[i].begin(), lanes[i].end(), [](const Vehicle& lhs, const Vehicle& rhs){ return lhs.s < rhs.s; });
  }

  this->lanes = lanes;
}

vector<double> Road::findClosestDistanceForwardAndBehind(Vehicle self, int lane) {
  vector<double> distances = {-100000, 100000};
  vector<Vehicle> vehicles = this->lanes[lane];
  for(int i = 0; i < vehicles.size(); i++) {
    double diff = vehicles[i].s - self.s;
    if(diff < 0) {
      distances[0] = diff;
    }
    else {
      distances[1] = diff;
      break;
    }
  }
  return distances;
}