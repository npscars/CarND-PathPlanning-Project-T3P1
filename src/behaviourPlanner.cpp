#include "behaviourPlanner.hpp"

// Constructor
BehaviourPlanner::BehaviourPlanner() {}

// Destructor
BehaviourPlanner::~BehaviourPlanner() {}

vector<float> BehaviourPlanner::decideLaneAndSpeed(int lane, vector<int> other_car_lane, double position, vector<double> other_car_futureDistance) {
    vector<float> scores = {0,0,0}; //initialize and revaluate all lanes every iteration
    float speed_score = 0;

    double s_between_cars = 30; // +/- 30 m
    
    // technically following logic only looks at immediate left and right lane, as car needs to go through them anyway for extremes
    for (int iothercars =0; iothercars < other_car_lane.size(); iothercars++) {
        
        double s_dist_other_car = other_car_futureDistance[iothercars]-position;

        // if other car in same lane, only accelerate and decelerate in only when going straight
        if ((lane == other_car_lane[iothercars]) && ((s_dist_other_car > s_between_cars) || (s_dist_other_car < -s_between_cars))) {
            //no penalty
        } else if (lane == other_car_lane[iothercars] && (s_dist_other_car >= 0) && (s_dist_other_car <=s_between_cars)) {
            scores[other_car_lane[iothercars]] -= 1;//score_laneFactor; // current lane definetly not good
            speed_score -=1; // go slow to avoid forward collision
        } else if (lane == other_car_lane[iothercars] && (s_dist_other_car < 0)&& (s_dist_other_car >=-s_between_cars)){
            //no penalty
        }
        
        // if other car in right lane
        if ((lane < other_car_lane[iothercars]) && ((s_dist_other_car > s_between_cars) || (s_dist_other_car < -s_between_cars))) {
            //no need for penalty
        } else if ((lane < other_car_lane[iothercars]) && (s_dist_other_car >= 0) && (s_dist_other_car <=s_between_cars)){
            scores[other_car_lane[iothercars]] -= 1;//score_laneFactor;
        } else if ((lane < other_car_lane[iothercars]) && (s_dist_other_car < 0) && (s_dist_other_car >=-s_between_cars)){
            scores[other_car_lane[iothercars]] -= 1;//score_laneFactor;
        }

        // if other car in left lane
        if ((lane > other_car_lane[iothercars]) && ((s_dist_other_car > s_between_cars) || (s_dist_other_car < -s_between_cars))) {
            //no need for penalty
        } else if ((lane > other_car_lane[iothercars]) && (s_dist_other_car >= 0) && (s_dist_other_car <=s_between_cars)){
            scores[other_car_lane[iothercars]] -= 1;//score_laneFactor;
        } else if ((lane > other_car_lane[iothercars]) && (s_dist_other_car < 0) && (s_dist_other_car >=-s_between_cars)){
            scores[other_car_lane[iothercars]] -= 1;//score_laneFactor;
        }
        
    }
    
    return {scores[0],scores[1],scores[2],speed_score};
}

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

float BehaviourPlanner::distanceCalc(int path_size, double speed, double s) {
    // Check distance after the timestep of other cars around the ego vehicle
    return (path_size*0.02*speed + s);
}