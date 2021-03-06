# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

## Code decisions
* Use another class to implement behaviour planner part required for lane changes. It does help simplify the code a lot. I could have done the same thing with path planner. Maybe a future implementation.
* Tried to keep camelCase style of new variable names.
* Main states used (though not named explicitly) for behaviour planning were Keep lane, Turn to left lane and Turn to Right lane. This is decided by behaviour planner [decideLaneAndSpeed method](https://github.com/npscars/CarND-PathPlanning-Project-T3P1/blob/2dd3b9be3f9fee5465e9f3e6b1f4ddc0db645a22/src/behaviourPlanner.cpp#L28-L67) and lines [276 to 283 in main.cpp](https://github.com/npscars/CarND-PathPlanning-Project-T3P1/blob/2dd3b9be3f9fee5465e9f3e6b1f4ddc0db645a22/src/main.cpp#L276-L283)
* 'bp.counterLaneChange > 200' in [main.cpp line 278](https://github.com/npscars/CarND-PathPlanning-Project-T3P1/blob/2dd3b9be3f9fee5465e9f3e6b1f4ddc0db645a22/src/main.cpp#L278) allowed for not having too many lane changes in quick successions. The planner will wait for 4 seconds (i.e. 200 x 0.02) before another lane change. 
* 'abs(possibleLane - lane)==1' in [main.cpp line 278](https://github.com/npscars/CarND-PathPlanning-Project-T3P1/blob/2dd3b9be3f9fee5465e9f3e6b1f4ddc0db645a22/src/main.cpp#L273-L274) allowed for making sure that only single lane changes happen as not to give high lateral acceleration as well as unrealistic movement of vehicle.
* Collision is avoided by having a scoring system as shown in [decideLaneAndSpeed method](https://github.com/npscars/CarND-PathPlanning-Project-T3P1/blob/2dd3b9be3f9fee5465e9f3e6b1f4ddc0db645a22/src/behaviourPlanner.cpp#L42-L44). It penalises the current lane and reduces the speed of the vehicle to avoid forward collision as shown in [line 272-273 of main.cpp](https://github.com/npscars/CarND-PathPlanning-Project-T3P1/blob/2dd3b9be3f9fee5465e9f3e6b1f4ddc0db645a22/src/main.cpp#L272-L273). The rearward collision is avoided by increasing the vehicle speed within speed limits when 'speedScore = 0' as shown in [main.cpp](https://github.com/npscars/CarND-PathPlanning-Project-T3P1/blob/2dd3b9be3f9fee5465e9f3e6b1f4ddc0db645a22/src/main.cpp#L270-L271). If other lanes are not penalised as the current lane then there will be a lane change suggestion as shown in [line 276 of main.cpp](https://github.com/npscars/CarND-PathPlanning-Project-T3P1/blob/2dd3b9be3f9fee5465e9f3e6b1f4ddc0db645a22/src/main.cpp#L276). And then the speed can be increased once the lane is changed to keep the overall average speed higher.
* Increase and decrease of reference velocity by 0.225 mph for every 0.02s is equivalent to 0.225 miles/hour x 1.609 km/miles x 0.278 m/s x hour/km x 1/0.02 s ~ ± 5 m/s^2 acceleration or deceleration respectively which is within the 10 m/s^2 target. [Spline trajectory plan](https://github.com/npscars/CarND-PathPlanning-Project-T3P1/blob/2dd3b9be3f9fee5465e9f3e6b1f4ddc0db645a22/src/main.cpp#L369-L384) also helps in lateral jerk and acceleration below the limit values. [Lines 347 and 348](https://github.com/npscars/CarND-PathPlanning-Project-T3P1/blob/2dd3b9be3f9fee5465e9f3e6b1f4ddc0db645a22/src/main.cpp#L347-L348) help in adding points including lane changes with 'd' defined as '(2+4 x lane)'.
* To avoid high acceleration and jerk limits previous untraversed path were used as part of new trajectory as shown in [lines 306 to 310 in main.cpp](https://github.com/npscars/CarND-PathPlanning-Project-T3P1/blob/2dd3b9be3f9fee5465e9f3e6b1f4ddc0db645a22/src/main.cpp#L306-L310). Then the new path is added to the final trajectory in [lines 398 to 399 of main.cpp](https://github.com/npscars/CarND-PathPlanning-Project-T3P1/blob/2dd3b9be3f9fee5465e9f3e6b1f4ddc0db645a22/src/main.cpp#L398-L399).


## Challenges
* Modifications in CMakeLists.txt required if I want to use ccmake to generate the make file properly. As described above that a helper class file for behaviour planner was created and to use it with proper C++ code styling CMakeLists.txt was appended as 'set(sources src/main.cpp src/behaviourPlanner.cpp)'

* The walkthrough video did help a lot for understanding how to implement the spline curve for changing lanes. It took some time to understand following things though:
   - Why we do such a long calculations just to add 1 to 2 new points in the path.
   - Convert to frenet coordinates and back for easier addition of path points.
   - Calculate how to break up the points so that the average speed is near to target speed

* Implementation of behaviour planner took a great amount of time. Especially in following situations:
   - Deciding on which scenarios should be penalized and which shouldn't. (check 'BehaviourPlanner::decideLaneAndSpeed' method in behaviourPlanner.cpp)
   - How to avoid quick lane changes (check line 278 to 283 in main.cpp)


## Next Steps
* Change lane with Bezier curve instead of Spline. I believe Bezier curve represents more realistic human like lane changing.
* Challenge myself to write an algorithm for optimised lane changing algorithm to get quicker around the track. Use state machines.


### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

