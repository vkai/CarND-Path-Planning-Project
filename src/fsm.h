#ifndef FSM_H
#define FSM_H

#include <vector>
#include "vehicle.h"
#include "road.h"
#include "utils.h"

using namespace std;

class FSM {


public:
  FSM();
  ~FSM(){};

  State transitionState(Vehicle self, Road road);

private:
  vector<State> getValidNextStates(Lane lane);
  double calculateCost(Vehicle self, Road road, State state);
};

#endif