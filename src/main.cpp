#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <valarray>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}
// Euclidean distance between two points
double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}
// NextWaypoint needed as closet point can be a point behind
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }
  
  // Start in lane 1
  int lane = 1;
  // Reference velocity to target at start
  double ref_vel =0.0; //mph

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane,&ref_vel](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner 
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;
                
                // Needed as if the car has passed two points then the size of the points will be 48.
                int previous_path_size = previous_path_x.size();
                
                /// SENSOR FUSION -- Done in Frenet coordinates
                if (previous_path_size > 0) { car_s = end_path_s;} // to make it representative of previous path point s
                bool too_close = false;
                
                vector<bool> are_other_car_inlane;
                vector<float> are_other_cars_around;
                cout<<"------------------"<<endl;
                for (int i = 0; i < sensor_fusion.size(); i++){
                    float d = sensor_fusion[i][6];
                    are_other_car_inlane.push_back(d < (2+4*lane+2) && d > (2+4*lane-2));
                    double vx = sensor_fusion[i][3];
                    double vy = sensor_fusion[i][4];
                    double speed_of_car = std::sqrt(vx*vx+vy*vy);
                    double distance_of_car_in_future = ((previous_path_size*0.02*speed_of_car)+(double)sensor_fusion[i][5]);
                    are_other_cars_around.push_back((distance_of_car_in_future - car_s > -30 ) && (distance_of_car_in_future - car_s < 30)); // independent of lane
                    cout<< are_other_car_inlane[i] << " , " << d << " , " << are_other_cars_around[i] <<endl;
                }
                cout<<"------------------"<<endl;
                
                int sum_cars_inlane = std::accumulate(are_other_car_inlane.begin(),are_other_car_inlane.end(),0);
                if (std::accumulate(are_other_car_inlane.begin(),are_other_car_inlane.end(),0)){
                   // cout << are_other_car_inlane << " :Car ahead " << endl;
                }
                // go through each car identified by sensor fusion 
                // main goal is to reduce the ref_vel so that we don't crash in front.
                for (int i = 0; i < sensor_fusion.size(); i++){
                    float d = sensor_fusion[i][6]; // which lane is current vehicle
                    if (d < (2+4*lane+2) && d > (2+4*lane-2)){ // is it in my current lane
                        double vx = sensor_fusion[i][3];
                        double vy = sensor_fusion[i][4];
                        double check_speed = std::sqrt(vx*vx+vy*vy); //get velocity magnitude
                        double check_car_s = sensor_fusion[i][5];
                        
                        //calculate where the car will be in the future i.e. after N*0.02 in my path?
                        check_car_s += previous_path_size*0.02*check_speed;
                        // will it be in my path
                        if ((check_car_s > car_s) && (check_car_s - car_s < 30)){
                            //Logic of either changing lane
                            
                            //OR Logic of reducing the vehicle speed
                            ref_vel = 30; //mph
                            too_close = true;
                            if (lane > 0){
                                lane = 0; // turn left
                            }
                            else if (lane == 0){
                                lane = 1;
                            }
                        }
                    }
                }
                
                if (too_close){
                    ref_vel -= 0.2; //mph
                }
                else if (ref_vel < 49.5){
                    ref_vel += 0.2; //mph
                }
                /// \return SENSOR FUSION
                
                
                /// ACTUAL PATH PLANNER
                // kind of coarse path points as the finer path is going to be created using spline
                vector<double> ptsx;
                vector<double> ptsy;
                
                // use car's current state as reference i.e. first waypoint
                double ref_path_x = car_x;
                double ref_path_y = car_y;
                double ref_path_yaw = deg2rad(car_yaw); // as output from simulator is in degrees
                
                // if not enough data available like at start then use car's current position as starting reference
                if (previous_path_size < 2){
                    double prev_car_x = car_x - cos(deg2rad(car_yaw));
                    double prev_car_y = car_y - sin(deg2rad(car_yaw));
                    // need minimum 2 points to create a path
                    ptsx.push_back(prev_car_x);
                    ptsx.push_back(car_x);
                    ptsy.push_back(prev_car_y);
                    ptsy.push_back(car_y);
                }
                else { // if enough previous points use last two path points to car as next start position
                    ref_path_x = previous_path_x[previous_path_size - 1]; // just passed
                    ref_path_y = previous_path_y[previous_path_size - 1];
                    double prev_ref_path_x = previous_path_x[previous_path_size - 2]; // one before
                    double prev_ref_path_y = previous_path_y[previous_path_size - 2];
                    ref_path_yaw = atan2(ref_path_y-prev_ref_path_y, ref_path_x - prev_ref_path_x);
                    // need minimum 2 points to create a path
                    ptsx.push_back(prev_ref_path_x);
                    ptsx.push_back(ref_path_x);
                    ptsy.push_back(prev_ref_path_y);
                    ptsy.push_back(ref_path_y);    
                }
                
                //add more points for making a bigger path and then add spline to make a smooth path
                // Using Frenet coordinates helps to easily add points.
                vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y); // 30m ahead in same lane of current car position in frenet
                vector<double> next_wp1 = getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y); // 60m ahead in frenet
                vector<double> next_wp2 = getXY(car_s+150,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y); // 90m ahead in frenet
                // add to the coarse path points
                ptsx.push_back(next_wp0[0]);
                ptsx.push_back(next_wp1[0]);
                ptsx.push_back(next_wp2[0]);
                ptsy.push_back(next_wp0[1]);
                ptsy.push_back(next_wp1[1]);
                ptsy.push_back(next_wp2[1]);
                
                //bring the path to car's local coordinates for easier spline calculations
                for (int i = 0; i < ptsx.size(); i++){
                    //shift the path to zero x,y i.e. zero degrees
                    double shift_x = ptsx[i]-ref_path_x;
                    double shift_y = ptsy[i]-ref_path_y;
                    ptsx[i] = shift_x*cos(0-ref_path_yaw)-shift_y*sin(0-ref_path_yaw);
                    ptsy[i] = shift_x*sin(0-ref_path_yaw)+shift_y*cos(0-ref_path_yaw);
                }
                
                // Do the spline calculations (ref: http://kluge.in-chemnitz.de/opensource/spline/)
                tk::spline s;
                s.set_points(ptsx,ptsy);
                
                // Calculate how to break up the points so that the average speed is near to target speed
                //Equation ==> (number of points, N * timestep, 0.02s) * target velocity, 49.5mph = distance
                double target_x = 30; //m
                double target_y = s(target_x);
                double target_dist = std::sqrt(target_x*target_x + target_y*target_y);// euclidean hypotenuse distance
                
                vector<double> next_x_vals;
          	vector<double> next_y_vals;
                // Add previous path points as a starting points to make it smooth transition
                for (int i = 0; i < previous_path_size; i++){
                    next_x_vals.push_back(previous_path_x[i]);
                    next_y_vals.push_back(previous_path_y[i]);
                }
                
                //fill in the rest of the path after filling the previous points, here we need to make sure that we add exactly 50 points
                double x_add_on = 0;
                for (int i = 1; i <= 50-previous_path_size; i++){
                    double N = target_dist/((ref_vel*1.609/3.6)*0.02); //0.02 is timestep and ref_vel is in miles per hour
                    double x_point = x_add_on + target_x/N;
                    double y_point = s(x_point);
                    //new x_add_on for next iteration
                    x_add_on = x_point;
                    //bring the path back to global coordinates
                    double x_ref = x_point;
                    double y_ref = y_point;
                    //rotate
                    x_point = x_ref*cos(ref_path_yaw)-y_ref*sin(ref_path_yaw);
                    y_point = x_ref*sin(ref_path_yaw)+y_ref*cos(ref_path_yaw);
                    //shift back to reference path
                    x_point = x_point + ref_path_x;
                    y_point = y_point + ref_path_y;
                    //add x and y values those to previous path points
                    next_x_vals.push_back(x_point);
                    next_y_vals.push_back(y_point);                    
                }
                /// \return ACTUAL PATH PLANNER
                
                /* // straight line and follow circular path as in class notes
                double pos_x;
                double pos_y;
                double angle;
                int path_size = previous_path_x.size();

                for(int i = 0; i < path_size; i++)
                {
                    next_x_vals.push_back(previous_path_x[i]);
                    next_y_vals.push_back(previous_path_y[i]);
                }

                if(path_size == 0)
                {
                    pos_x = car_x;
                    pos_y = car_y;
                    angle = deg2rad(car_yaw);
                }
                else
                {
                    pos_x = previous_path_x[path_size-1];
                    pos_y = previous_path_y[path_size-1];

                    double pos_x2 = previous_path_x[path_size-2];
                    double pos_y2 = previous_path_y[path_size-2];
                    angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
                }

                double dist_inc = 0.5;
                for(int i = 0; i < 50-path_size; i++)
                {    
                    double next_s = car_s+(i+1)*dist_inc; //i+1 is because i is exactly where the car is sitting, so 1st point is ahead if it i+1
                    double next_d = 2+4;
                    
                    vector<double> xy = getXY(next_s,next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                    
                    next_x_vals.push_back(xy[0]);
                    next_y_vals.push_back(xy[1]);
                    
                    //next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100)));
                    //next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100)));
                    
                    // // to make car go in straight line
                    //next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
                    //next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
                    
                    pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
                    pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
                }
                */

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
