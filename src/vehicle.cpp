#include "vehicle.h"

using namespace std;

Vehicle::Vehicle() {
  Vehicle(0,0,0,0,0);
}

Vehicle::Vehicle(double x, double y, double s, double d, double speed) {
  this->x = x;
  this->y = y;
  this->s = s;
  this->d = d;
  this->speed = speed;

  this->prev_s.push_back(s);
  this->prev_s.push_back(speed);
  this->prev_d.push_back(d);
}

void Vehicle::setPrev(vector<double> prev_s, vector<double> prev_d) {
  this->prev_s = prev_s;
  this->prev_d = prev_d;
}

Lane Vehicle::get_lane() {
  if(0 <= this->d && this->d < 4) {
    return Lane::LEFT;
  }
  else if(4 <= this->d && this->d < 8) {
    return Lane::CENTER;
  }
  else {
    return Lane::RIGHT;
  }
}
