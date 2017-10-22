#include "fsm.h"
#include <limits>
#include <vector>
#include <iostream>

using namespace std;


FSM::FSM() {
}


State FSM::transitionState(Vehicle self, Road road) {
  Lane self_lane = self.get_lane();
  vector<State> valid_next_states = getValidNextStates(self_lane);

  State min_cost_state;
  double min_cost = std::numeric_limits<double>::max();
  for(int i = 0; i < valid_next_states.size(); i++) {
    State curr_state = valid_next_states[i];
    double cost = calculateCost(self, road, curr_state);
    cout << "State: " << curr_state << " Cost: " << cost << " ";
    if(cost < min_cost) {
      min_cost = cost;
      min_cost_state = curr_state;
    }
  }
  cout << endl;

  return min_cost_state;
}

vector<State> FSM::getValidNextStates(Lane lane) {
  vector<State> states;
  states.push_back(State::KEEP_LANE);

  if (lane == Lane::LEFT) {
    states.push_back(State::CHANGE_RIGHT);
  }
  else if (lane == Lane::CENTER) {
    states.push_back(State::CHANGE_LEFT);
    states.push_back(State::CHANGE_RIGHT);
  }
  else if (lane == Lane::RIGHT) {
    states.push_back(State::CHANGE_LEFT);
  }
  
  return states;
}

double FSM::calculateCost(Vehicle self, Road road, State state) {
  int target_lane = self.get_lane();
  if(state == State::CHANGE_LEFT) {
    target_lane = self.get_lane() - 1;
  }
  else if(state == State::CHANGE_RIGHT) {
    target_lane = self.get_lane() + 1;
  }

  vector<double> distances_between = road.findClosestDistanceForwardAndBehind(self, target_lane);
  double behind = distances_between[0];
  double forward = distances_between[1];

  double base_cost = 100;
  double cost = base_cost;
  if(state == State::KEEP_LANE) {
    cost = base_cost/forward;
    if(forward > 40) {
      cost = 0;
    }
  }
  else if(forward == 0 || behind == 0) {
    cost = base_cost * 10;
  }
  else {
    cost = (base_cost/forward) + ((base_cost / -2) / behind);
    if(behind <= -40 && forward >= 40) {
      cost = 1;
    }
  }

  return cost;
}