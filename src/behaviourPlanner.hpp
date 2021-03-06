//#ifndef behaviourPlanner_hpp
//#define behaviourPlanner_hpp

#include <vector>
#include <string>

using namespace std;

class BehaviourPlanner {
  public:
    int counterLaneChange = 0;
    
    //Constructor
    BehaviourPlanner();
    
    // Destructor
    virtual ~BehaviourPlanner();
    
    // Calculates if d value corresponds to left, right, or center lane
    int laneCalc(double d);
    
    // Calculates future distance of the car based on simulation time and path size
    float distanceCalc(int path_size, double speed, double s);
    
    // final lane the vehicle should move to and speed reduction if required
    vector<float> decideLaneAndSpeed(int lane, vector<int> otherCarLane, double position, vector<double> otherCarFutureDistance);
   
};

//#endif