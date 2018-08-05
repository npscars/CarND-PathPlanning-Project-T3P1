#include "behaviourPlanner.hpp"

// Constructor
BehaviourPlanner::BehaviourPlanner() {}

// Destructor
BehaviourPlanner::~BehaviourPlanner() {}

int BehaviourPlanner::laneCalc(double d) {
  // Check which lane the d-value comes from
  // Left is 0, middle is 1, right is 2
  int lane;
  if (d < 4) {
    lane = 0;
  } else if (d < 8) {
    lane = 1;
  } else {
    lane = 2;
  }
  return lane;
}

float BehaviourPlanner::distanceCalc(int pathSize, double speed, double s) {
    // Check distance after the timestep of other cars around the ego vehicle
    return (pathSize*0.02*speed + s);
}

vector<float> BehaviourPlanner::decideLaneAndSpeed(int lane, vector<int> otherCarLane, double position, vector<double> otherCarFutureDistance) {
    vector<float> scores = {0,0,0}; //initialize and revaluate all lanes every iteration
    float speedScore = 0;

    double S_BETWEEN_CARS = 30; // +/- 30 m -- can be increased to look further and hence plan ahead (could also be a bit conservative if increased too much)
    
    // technically following logic only looks at immediate left and right lane, as car needs to go through them anyway for extremes
    for (int iothercars =0; iothercars < otherCarLane.size(); iothercars++) {
        
        double sDistOtherCars = otherCarFutureDistance[iothercars]-position;

        // if other car in same lane, only accelerate and decelerate in only when going straight
        if ((lane == otherCarLane[iothercars]) && ((sDistOtherCars > S_BETWEEN_CARS) || (sDistOtherCars < -S_BETWEEN_CARS))) {
            //no penalty - can be use for lane change optimisation
        } else if (lane == otherCarLane[iothercars] && (sDistOtherCars >= 0) && (sDistOtherCars <=S_BETWEEN_CARS)) {
            scores[otherCarLane[iothercars]] -= 1;// current lane definetly not good
            speedScore -=1; // go slow to avoid forward collision
        } else if (lane == otherCarLane[iothercars] && (sDistOtherCars < 0)&& (sDistOtherCars >=-S_BETWEEN_CARS)){
            //no penalty
        }
        
        // if other car in right lane
        if ((lane < otherCarLane[iothercars]) && ((sDistOtherCars > S_BETWEEN_CARS) || (sDistOtherCars < -S_BETWEEN_CARS))) {
            //no need for penalty  - but can be use for lane change optimisation
        } else if ((lane < otherCarLane[iothercars]) && (sDistOtherCars >= 0) && (sDistOtherCars <=S_BETWEEN_CARS)){
            scores[otherCarLane[iothercars]] -= 1;
        } else if ((lane < otherCarLane[iothercars]) && (sDistOtherCars < 0) && (sDistOtherCars >=-S_BETWEEN_CARS)){
            scores[otherCarLane[iothercars]] -= 1;
        }

        // if other car in left lane
        if ((lane > otherCarLane[iothercars]) && ((sDistOtherCars > S_BETWEEN_CARS) || (sDistOtherCars < -S_BETWEEN_CARS))) {
            //no need for penalty  - but can be use for lane change optimisation
        } else if ((lane > otherCarLane[iothercars]) && (sDistOtherCars >= 0) && (sDistOtherCars <=S_BETWEEN_CARS)){
            scores[otherCarLane[iothercars]] -= 1;
        } else if ((lane > otherCarLane[iothercars]) && (sDistOtherCars < 0) && (sDistOtherCars >=-S_BETWEEN_CARS)){
            scores[otherCarLane[iothercars]] -= 1;
        }
        
    }
    
    return {scores[0],scores[1],scores[2],speedScore};
}
