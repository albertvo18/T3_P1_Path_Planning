#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
using Eigen::MatrixXd;
using Eigen::VectorXd;

#include "json.hpp"
#include "spline.h"
#include <ctime>
using namespace std;
////////////////////////////////////////////////////////////////////

double left_lane_free_score_global = 0.0;
double right_lane_free_score_global = 0.0;

double lane_0_free_score_global = 0.0;
double lane_1_free_score_global = 0.0;
double lane_2_free_score_global = 0.0;
////////////////////////////////////////////////////////////////////
class Car {
public:
  double s;
  double s_d;
  double s_dd;
  double d;
  double d_d;
  double d_dd;
  string state;
  vector<string> state_options;
  vector<double> s_traj_coeffs, d_traj_coeffs;

  Car();
  Car(double s, double s_d, double s_dd, double d, double d_d, double d_dd);
  virtual ~Car();

};


//  Initialize
Car::Car() {}


Car::Car(double s, double s_d, double s_dd, double d, double d_d, double d_dd) {
  this->s    = s;         // s position
  this->s_d  = s_d;       // s dot - velocity in s
  this->s_dd = s_dd;      // s dot-dot - acceleration in s
  this->d    = d;         // d position
  this->d_d  = d_d;       // d dot - velocity in d
  this->d_dd = d_dd;      // d dot-dot - acceleration in d
  state = "CS";

}

Car::~Car(){}

//////////////////////////////////////////
double check_car_in_our_lane(double car_s, double other_car_s,double car_speed, double other_car_speed) 
{
  double closestDist_s =1000;
  bool change_lanes = false;

  //check s values greater than mine and s gap
 if((other_car_s > car_s) && ((other_car_s-car_s) < 30) && ((other_car_s-car_s) < closestDist_s ) )
 {
   closestDist_s = (other_car_s - car_s);
   if((other_car_s-car_s) > 20)
//   if((other_car_s-car_s) > 30)
   {
     cout << "Car in Front...Reduce Speed and Check if Safe to Change Lanes"  << endl;
     //match that cars speed
     car_speed = other_car_speed*2.237;
     change_lanes = true;
   }
   else
   {
     cout << "Car in Front...Go SLOWER and Check if Safe to Change Lanes "  << endl;
     //go slightly slower than the cars speed
     car_speed = other_car_speed*2.237-5;
//     car_speed = other_car_speed*2.237-2;
//                                ref_vel = check_speed*2.237-2;
     change_lanes = true;
   }
 }
 return car_speed;
}



//////////////////////////////////////////
class BehaviorPlanner {
  public:
    int curr_lane;
    double curr_lead_vehicle_speed = 22.352 - 0.5;
    double target_vehicle_speed;
    vector<double> avg_scores = {0,0,0};

    // Decides whether to go left, right, or stay in the same lane
    // Returns amount of meters left or right to move
    int lanePlanner(double s, double d, vector<vector<double>> sensor_fusion);

    // Calculates if d value corresponds to left, right, or center lane
    int laneCalc(double d);

    // Calculates the closest vehicle either in front or behind the car in a given lane
    // Returns distance and speed of that vehicle
    vector<double> closestVehicle(double s, int lane, vector<vector<double>> sensor_fusion, bool direction);

    // Scores each lane on factors such as distance to nearest vehicle & speed
    // Returns the lane with the best score (0 left, 1 middle, 2 right)
    int laneScore(double s, int lane, vector<vector<double>> sensor_fusion);
};

int BehaviorPlanner::lanePlanner(double s, double d, vector<vector<double>> sensor_fusion) {
  int lane = laneCalc(d);
  int new_lane;
  double distance = closestVehicle(s, lane, sensor_fusion, true)[0];

  curr_lane = lane; // Keep the current lane to later calculate desired move

  // check if blocked, i.e. car is within 20 meters
  if (distance > 20) { // if lots of space, stay in lane and go near the speed limit
    new_lane = lane;
    target_vehicle_speed = 22.352 - 0.5;
    avg_scores = {0,0,0}; // Reset average scores for laneScore()
    return 0;
  } else {
    new_lane = laneScore(s, lane, sensor_fusion);
    vector <double> vehicle = closestVehicle(s, new_lane, sensor_fusion, true);
    target_vehicle_speed = vehicle[1];
  }

  // Space between middle of each lane is four meters, so move accordingly
  if (new_lane == lane) {
    return 0;
  } else if (new_lane < lane) {
    return -4;
  } else {
    return 4;
  }
}
//////////////////////////////////////////

int BehaviorPlanner::laneCalc(double d) {
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

//////////////////////////////////////////
vector<double> BehaviorPlanner::closestVehicle(double s, int lane, vector<vector<double>> sensor_fusion, bool direction) {
  double dist = 10000;
  double velocity = 22.352 - 0.5; // Set in case of no cars
  double vehicle_s;
  double vehicle_d;
  double vehicle_v;
  int vehicle_lane;

  // Check each vehicle in sensor range
  for(int vehicle = 0; vehicle < sensor_fusion.size(); vehicle++) {
    vehicle_s = sensor_fusion[vehicle][5];
    vehicle_d = sensor_fusion[vehicle][6];
    vehicle_v = sqrt(pow(sensor_fusion[vehicle][3], 2)+pow(sensor_fusion[vehicle][4], 2));
    vehicle_lane = laneCalc(vehicle_d);

    if (vehicle_lane == lane) { // if same lane
      if (direction == true) {
        if (vehicle_s > s and (vehicle_s - s) < dist) { // and ahead of my vehicle
          dist = vehicle_s - s;
          velocity = vehicle_v;
        }
      } else {
        if (s >= vehicle_s and (s - vehicle_s) < dist) { // if behind my vehicle
          dist = s - vehicle_s;
          velocity = vehicle_v;
        }
      }
    }
  }
  if (dist <= 0) { // Avoid dividing by zero in laneScore()
    dist = 1.0;
  }
  if (lane == curr_lane and direction == true) {
    curr_lead_vehicle_speed = velocity;
  }
  return {dist, velocity};
}

int BehaviorPlanner::laneScore(double s, int lane, vector<vector<double>> sensor_fusion) {
  vector <double> scores = {0,0,0};
  vector <double> front_vehicle;
  vector <double> back_vehicle;

  for (int i = 0; i < 3; i++) {
    if (i == lane) {  // benefit to keeping lane
      scores[i] += 0.5;
    }
    front_vehicle = closestVehicle(s, i, sensor_fusion, true);
    back_vehicle = closestVehicle(s, i, sensor_fusion, false);
    if (front_vehicle[0] > 1000 and back_vehicle[0] > 1000) {
      scores[i] += 5; // if wide open lane, move into that lane
    } else {
      if (front_vehicle[0] < 10) {
        scores[i] -= 5; // if car too close in front, negative score
      }
      if (back_vehicle[0] < 10) {
        scores[i] -= 5; // if car too close in back, negative score
      }
      scores[i] += 1 - (10/(front_vehicle[0]/3)); // benefit for large open distance in lane in front
      scores[i] += 1 - (10/(back_vehicle[0]/3)); // benefit for large open distance in lane in back
      scores[i] += 1 - (10/(front_vehicle[1]/2)); // benefit for faster car speed in lane in front
      scores[i] += 1 / (back_vehicle[1]/2); // benefit for slower car speed in lane in back
    }
    // Simple in-exact calculation for scores over the last ten iterations
    avg_scores[i] = (avg_scores[i] * 10) - avg_scores[i];
    avg_scores[i] += scores[i];
    avg_scores[i] /= 10;
  }

  // Only compare applicable lanes
  if (lane == 0) {
    return max_element(avg_scores.begin(), avg_scores.end() - 1) - avg_scores.begin();
  } else if (lane == 1) {
    return max_element(avg_scores.begin(), avg_scores.end())  - avg_scores.begin();
  } else {
    return max_element(avg_scores.begin() + 1, avg_scores.end())  - avg_scores.begin();
  }
}

//////////////////////////////////////////
//////////////////////////////////////////
vector<double> get_traj_coeffs(vector<double> start, vector<double> end, double T)
{
 MatrixXd a(3,3);

 double T2 =  T*T,
        T3 = T2*T,
        T4 = T3*T,
        T5 = T4*T;

 a <<  T3,    T4,    T5,
       3*T2,  4*T3,  5*T4,
       6*T, 12*T2, 20*T3;
 MatrixXd aInv = a.inverse();


 VectorXd b(3);
 b << end[0] - (start[0] + start[1]*T + 0.5*start[2]*T2),
      end[1] - (           start[1]   +     start[2]*T),
      end[2] - (                            start[2]);
 VectorXd alpha = aInv * b;

 vector<double> output = {start[0], start[1], 0.5*start[2], alpha[0], alpha[1], alpha[2]};
// vector<double> output ;
 return output;

}

//////////////////////////////////////////


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

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
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

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y, vector<double> maps_dx, vector<double> maps_dy)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	//heading vector
    double hx = map_x-x;
    double hy = map_y-y;
    
    //Normal vector:
    double nx = maps_dx[closestWaypoint];
    double ny = maps_dy[closestWaypoint];
    
    //Vector into the direction of the road (perpendicular to the normal vector)
    double vx = -ny;
    double vy = nx;

    //If the inner product of v and h is positive then we are behind the waypoint so we do not need to
    //increment closestWaypoint, otherwise we are beyond the waypoint and we need to increment closestWaypoint.

    double inner = hx*vx+hy*vy;
    if (inner<0.0) {
        closestWaypoint++;
    }

    return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y,vector<double> maps_dx, vector<double> maps_dy)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y, maps_dx, maps_dy);

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
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
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
////////////////////////////////////////////////////////////////////
// Record Type for other cars
struct other_car_rec
{
    int    other_car_id;
    int    other_car_lane;
    double other_car_s;
    double other_car_speed;
};

// Telemetry Type Record
struct telemetry_t
{
    int    other_car_lane;
    double other_car_s;
    double other_car_speed;
    vector<other_car_rec> other_cars_list;
};

////////////////////////////////////////////////////////////////////
// convert lane  to Frenet d-coordinate
double convert_lane_to_d(int lane)
{
    if (lane == 1)
    {
        return 2.0;
    }
    if (lane == 2)
    {
        return 6.0;
    }
    if (lane == 3)
    {
        return 10.0;
    }
    return 0;
};

// convert from Frenet d-coordinate to lane
int convert_d_to_lane(double d)
{
    if (d<4)
    {
         return 0;
    }
    if (d>=4.0 && d<8.0)
    {
        return 1;
    }
    if (d>=8.0)
    {
        return 2;
    }
    return 0;
};

////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
int main() {
  uWS::Hub h;

  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

//  string map_file_ = "../data/highway_map.csv";
  string map_file_ = "highway_map.csv";
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



  //start in lane 1;
  int lane = 1;
  int lane_change_wp = 0;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane,&lane_change_wp](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          //double car_x = j[1]["x"];

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
                int car_lane = convert_d_to_lane(car_d);
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];
          	vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];


          	int prev_size = previous_path_x.size();

          	int next_wp = -1;
          	double ref_x = car_x;
          	double ref_y = car_y;
          	double ref_yaw = deg2rad(car_yaw);
//          	double ref_vel = 49.5; 
          	double ref_vel = 49.0; 
//          	double ref_vel = 49.25; 
//          	double ref_vel = 49.2; 
//          	double ref_vel = 49.15; 
//          	double ref_vel = 49.1; 

////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
                // First path
                if (previous_path_x.size() == 0)
                {
                  vector<other_car_rec> other_cars_list = {};
                  for (int i=0; i<sensor_fusion.size(); i++)
                  {
                    int other_car_id         = sensor_fusion[i][0];
                    double other_car_s       = sensor_fusion[i][5];
                    float other_car_d       = sensor_fusion[i][6];
                    int other_car_lane       = convert_d_to_lane(other_car_d);
                    double other_car_vx      = sensor_fusion[i][3];
                    double other_car_vy      = sensor_fusion[i][4];
                    double other_car_speed   = sqrt(other_car_vx*other_car_vx + other_car_vy*other_car_vy);
                    other_car_rec other_car_record = {other_car_id, other_car_lane, other_car_s, other_car_speed};
                    other_cars_list.push_back(other_car_record);
                  }
                }

////////////////////////////////////////////////////////////////////////////////
          	if(prev_size < 2)
          	{
          		next_wp = NextWaypoint(ref_x, ref_y, ref_yaw, map_waypoints_x,map_waypoints_y,map_waypoints_dx,map_waypoints_dy);
          	}
          	else
          	{
	          ref_x = previous_path_x[prev_size-1];
	          double ref_x_prev = previous_path_x[prev_size-2];
		  ref_y = previous_path_y[prev_size-1];
		  double ref_y_prev = previous_path_y[prev_size-2];
		  ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);
		  next_wp = NextWaypoint(ref_x,ref_y,ref_yaw,map_waypoints_x,map_waypoints_y,map_waypoints_dx,map_waypoints_dy);

		  car_s = end_path_s;

		  car_speed = (sqrt((ref_x-ref_x_prev)*(ref_x-ref_x_prev)+(ref_y-ref_y_prev)*(ref_y-ref_y_prev))/.02)*2.237;
          	}

          	//find ref_v to use
          	double closestDist_s = 100000;
          	bool change_lanes = false;

                bool car_in_front = false;
                bool too_close_to_car_in_front = false;
                bool too_close_to_car_in_back = false;
                bool change_to_left_lane = (lane > 0);
                bool change_to_right_lane = (lane < 2);
             
                bool lane_0_free_at_least_100m = false;
                bool lane_1_free_at_least_100m = false;
                bool lane_2_free_at_least_100m = false;
                bool lane_0_free_at_least_200m = false;
                bool lane_1_free_at_least_200m = false;
                bool lane_2_free_at_least_200m = false;
               // threshold for s gap between us and other cars before classifying them as being too close
                double threshold_front = 30;
                double threshold_back = 10;
                double left_lane_free_score = 0.0;
                double right_lane_free_score = 0.0;
                double lane_0_free_score = 0.0;
                double lane_1_free_score = 0.0;
                double lane_2_free_score = 0.0;
//                time_t time_now = time(0);


                
//                cout << "-----------------  Time = " << time_now << "-----------" << endl;
                cout << "---------------------------------------------------------" << endl;
                cout << "My Car Lane   = " << lane << endl;
                cout << "My Car Speed  = " << car_speed << endl;
                cout << "My Car Dist_s = " << car_s << endl;


          	for(int i = 0; i < sensor_fusion.size(); i++)
          	{
         	  //Check car lane
//          	  float d = sensor_fusion[i][6];
          	  float other_car_d = sensor_fusion[i][6];
          	  double other_car_s = sensor_fusion[i][5];
                  int other_car_lane = floor(other_car_d/4.0);
//                cout << "---------------------------Sensor Fusion  =" << i  << endl;
/*
                  cout <<  sensor_fusion[i][1] << " , " ;
                  cout <<  sensor_fusion[i][2] << " , ";
                  cout <<  sensor_fusion[i][3] << " , ";
                  cout <<  sensor_fusion[i][4] << " , ";
                  cout <<  sensor_fusion[i][5] << " , ";
                  cout <<  sensor_fusion[i][6] << endl;
*/
////////////////////////////////////////////////////////////////////////////////////////////
                  if (other_car_lane == 0 && other_car_s - car_s < 30) {
                    lane_0_free_score += 1;
                  }
                  if (other_car_lane == 1 && other_car_s - car_s < 30) {
                    lane_1_free_score += 1;
                  }
                  if (other_car_lane == 2 && other_car_s - car_s < 30 ) {
                    lane_2_free_score += 1;
                  }




/*

                  //  Path Planning 
                  if ( other_car_lane == 0 && other_car_s - car_s > 0 && other_car_s - car_s < 100) {
                    lane_0_free_at_least_100m = false;
                    left_lane_free_score += 1;
                    lane_0_free_score += 1;
                    lane_0_free_score_global += 1;
                  }
                  else {
                    lane_0_free_score -= 1;
                    lane_0_free_score_global -= 1;
                  }
                  if ( other_car_lane == 1 && other_car_s - car_s > 0 && other_car_s - car_s < 100) {
                    lane_1_free_at_least_100m = false;
                    lane_1_free_score += 1;
                    lane_1_free_score_global += 1;
                  }
                  else {
                    lane_1_free_score -= 1;
                    lane_1_free_score_global -= 1;
                  }
                  if ( other_car_lane == 2 && other_car_s - car_s > 0 && other_car_s - car_s < 100) {
                    lane_2_free_at_least_100m = false;
                    right_lane_free_score += 1;
                    lane_2_free_score += 1;
                    lane_2_free_score_global += 1;
                  }
                  else {
                    lane_2_free_score -= 1;
                    lane_2_free_score_global -= 1;
                  }
                  if ( other_car_lane == 0 && other_car_s - car_s > 0 && other_car_s - car_s < 200) {
                    lane_0_free_at_least_200m = false;
                    lane_0_free_score += 1;
                    lane_0_free_score_global += 1;
                  }
                  else {
                    lane_0_free_score -= 1;
                    lane_0_free_score_global -= 1;
                  }
                  if ( other_car_lane == 1 && other_car_s - car_s > 0 && other_car_s - car_s < 200) {
                    lane_1_free_at_least_200m = false;
                    lane_1_free_score += 1;
                    lane_1_free_score_global += 1;
                  }
                  else {
                    lane_1_free_score -= 1;
                    lane_1_free_score_global -= 1;
                  }
                  if ( other_car_lane == 2 && other_car_s - car_s > 0 && other_car_s - car_s < 200) {
                    lane_2_free_at_least_200m = false;
                    lane_2_free_score += 1;
                    lane_2_free_score_global += 1;
                  }
                  else {
                    lane_2_free_score -= 1;
                    lane_2_free_score_global -= 1;
                  }
                   

*/
////////////////////////////////////////////////////////////////////////////////////////////

                  // other car in front
                  if (other_car_lane == lane && other_car_s - car_s > 0 && other_car_s -car_s < 50 ) 
                  {
                    car_in_front = true;
                    cout << "car in front in lane  " << other_car_lane << endl;
                  }


                     double other_car_dist = other_car_s - car_s;
                     if (other_car_lane == lane - 1 && other_car_dist > 0 && other_car_dist < 10 ) {
                        change_to_left_lane = false;
                        left_lane_free_score += 1;
                        cout << "WARNING:  CAR ON LEFT AHEAD...don't change to left. car_lane: " << other_car_lane << endl;
                      }
                     if (other_car_lane == lane + 1 && other_car_dist > 0 && other_car_dist < 10 ) {
                        change_to_right_lane = false;
                        right_lane_free_score += 1;
                        cout << "WARNING:  CAR ON RIGHT AHEAD...don't change to right. car_lane: " << other_car_lane << endl;
                      }
                     if (other_car_lane == lane - 1 && other_car_dist < 0 && other_car_dist > -10 ) {
                        change_to_left_lane = false;
                        left_lane_free_score += 1;
                        cout << "WARNING:  CAR ON LEFT ...don't change to left. car_lane: " << other_car_lane << endl;
                      }
                     if (other_car_lane == lane + 1 && other_car_dist < 0 && other_car_dist > -10 ) {
                        change_to_left_lane = false;
                        right_lane_free_score += 1;
                        cout << "WARNING:  CAR ON RIGHT ...don't change to right. car_lane: " << other_car_lane << endl;
                     }
                     left_lane_free_score_global = left_lane_free_score;   
                     right_lane_free_score_global = right_lane_free_score;   


//                     if (other_car_lane == lane - 1 && car_in_front && other_car_dist > 0 && other_car_dist > 30 ) {
//                     if (other_car_lane == lane - 1 && car_in_front && too_close_to_car_in_front && other_car_dist > -10 &&  other_car_dist  > 50) {
                     if (other_car_lane == lane - 1 && car_in_front && too_close_to_car_in_front ) {
                        if ( other_car_dist > -10 && other_car_dist < 30 && left_lane_free_score_global == 0) {
                          change_to_left_lane = true;
                          cout << "------------------LEFT LANE FREE:   " << endl;
                        }
                      }
                     if (other_car_lane == lane + 1 && car_in_front && too_close_to_car_in_front && other_car_dist > -10  && other_car_dist > 50)  {
                        if ( other_car_dist > -10 && other_car_dist < 30 && right_lane_free_score_global == 0) {
                          change_to_right_lane = true;
                          cout << "------------------RIGHT LANE FREE:   " << endl;
                        }
                      }


                  //  Path Planning
                  if ( lane_0_free_score == 0 &&  lane_0_free_score_global == 0 && other_car_s - car_s < 30) {
                    lane_0_free_at_least_100m = true;
                    left_lane_free_score -= 1;
                  }
                  if ( lane_1_free_score == 0 && other_car_s - car_s < 30) {
                    lane_1_free_at_least_100m = true;
                  }
                  if ( lane_2_free_score == 0 && lane_2_free_score_global && other_car_s - car_s < 30) {
                    lane_2_free_at_least_100m = true;
                    right_lane_free_score -= 1;
                  }
                  if ( lane_0_free_score  == 0 && lane_0_free_score_global == 0 && other_car_s - car_s > 200) {
                    lane_0_free_at_least_100m = true;
                    left_lane_free_score -= 1;
                  }
                  if ( lane_1_free_score == 0 && other_car_s - car_s > 200) {
                    lane_1_free_at_least_100m = true;
                  }
                  if ( lane_2_free_score == 0 && other_car_s - car_s > 200) {
                    lane_2_free_at_least_100m = true;
                    right_lane_free_score -= 1;
                  }
                  left_lane_free_score_global = left_lane_free_score;
                  right_lane_free_score_global = right_lane_free_score;
                  lane_0_free_score_global = lane_0_free_score;
                  lane_1_free_score_global = lane_1_free_score;
                  lane_2_free_score_global = lane_2_free_score;






////////////////////////////////////////////////////////////////////////////////////////////
                  // other car in left lane
                 if (other_car_lane == lane - 1 && (other_car_s - car_s > 0) && (other_car_s - car_s) < 10) 
                 {
                   change_to_left_lane = false;
                   left_lane_free_score += 1;
//                   cout << "WARNING: CAR ON LEFT AHEAD... in car lane " << other_car_lane << endl;
                 }

                 // other car in right lane
                 if (other_car_lane == lane + 1 && (other_car_s - car_s > 0) && (other_car_s - car_s) < 10) {

                   change_to_right_lane = false;
                   right_lane_free_score += 1;
 //                  cout << "WARNING: CAR ON RIGHT AHEAD..in car lane " << other_car_lane << endl;

                  }

          	  if(other_car_d < (2+4*lane+2) && other_car_d > (2+4*lane-2) )
          	  {
          	    double vx = sensor_fusion[i][3];
          	    double vy = sensor_fusion[i][4];
          	    double other_car_d = sensor_fusion[i][6];
                    int other_car_lane = convert_d_to_lane(other_car_d);
          	    double check_speed = sqrt(vx*vx+vy*vy);
          	    double check_car_s = sensor_fusion[i][5];
          	    check_car_s+=((double)prev_size*.02*check_speed);

                    double other_car_speed = check_speed;
                    double other_car_s = check_car_s;
                    double gap_between_front_car;
                    
//                    double left_lane_free_score = 0.0;
//                    double right_lane_free_score = 0.0;



///////////////////////////////////////////////////////////////
                    other_car_s += ((double)prev_size*.02*other_car_speed);


                    gap_between_front_car = other_car_s - car_s;
                    if (gap_between_front_car > 0 && gap_between_front_car < 60) {
                       cout << "other_car_lane " << other_car_lane << endl;
                       cout << "gap_between_front_car: " << gap_between_front_car << endl;
                    }

                    if (gap_between_front_car > 0 && gap_between_front_car < 50) {
                       too_close_to_car_in_front = true;
                    }
                    if (gap_between_front_car < 0 && gap_between_front_car < -10) {
                       too_close_to_car_in_back = true;
                    }



                    bool car_ahead = other_car_s > car_s;
                    bool gap_within_threshold = (other_car_s - car_s < threshold_front && car_ahead) or other_car_s - car_s < -threshold_back;
 

                    //  Other Car in Our Lane
                    //  Lane is 4m wide
                    if ( other_car_lane == lane && car_ahead && too_close_to_car_in_front) {
                       too_close_to_car_in_front = true;
             	      
//                       ref_vel = check_car_in_our_lane(car_s,other_car_s,car_speed,other_car_speed);
                       cout << "too close to car in front. lane: " << lane << endl;
                       cout << "other_car_lane: " << other_car_lane << endl;
                     }

                     // other car in left lane
                                  
                     double other_car_dist = other_car_s - car_s;
                     if (other_car_lane == lane - 1 && other_car_dist > 0 && other_car_dist < 10 ) {
                        change_to_left_lane = false;
                        left_lane_free_score += 1;
                        cout << "WARNING:  CAR ON LEFT AHEAD...don't change to left. car_lane: " << other_car_lane << endl;
                      }

                      // other car in right lane
                      if (other_car_lane == lane + 1 && other_car_dist > 0 && other_car_dist < 10) {

                         change_to_right_lane = false;
                         right_lane_free_score += 1;
                         cout << "WARNING:  CAR ON RIGHT AHEAD...don't change to right. car_lane: " << other_car_lane << endl;

                       }

//////////////////////////////////////////////////////////////////////////////////////////          	
                     if (other_car_lane == lane - 1 && other_car_dist < 0 && other_car_dist > -10 ) {
                        change_to_left_lane = false;
                        left_lane_free_score += 1;
                        cout << "WARNING:  CAR ON LEFT ...don't change to left. car_lane: " << other_car_lane << endl;
                      }
                     if (other_car_lane == lane + 1 && other_car_dist < 0 && other_car_dist > -10 ) {
                        change_to_left_lane = false;
                        right_lane_free_score += 1;
                        cout << "WARNING:  CAR ON RIGHT ...don't change to right. car_lane: " << other_car_lane << endl;
                      }


                     if (other_car_lane == lane - 1 &&  car_in_front && too_close_to_car_in_front && other_car_dist > -10 &&other_car_dist > 50 && left_lane_free_score <= 0 ) {
                        change_to_left_lane = true;
                        left_lane_free_score -= 1;
                        left_lane_free_score_global = left_lane_free_score;
                        cout << "------------------LEFT LANE FREE:   " << endl;
                      }
                     if (other_car_lane == lane + 1 && car_in_front && too_close_to_car_in_front && other_car_dist > -10 && other_car_dist > 50 && right_lane_free_score <= 0 ) {
                        change_to_right_lane = true;
                        right_lane_free_score -= 1;
                        right_lane_free_score_global = right_lane_free_score;
                        cout << "------------------RIGHT LANE FREE:   " << endl;
                      }



//////////////////////////////////////////////////////////////////////////////////////////          	


          	      //check s values greater than mine and s gap
          	      if((check_car_s > car_s) && ((check_car_s-car_s) < 30) && ((check_car_s-car_s) < closestDist_s ) )
          	      {
          		closestDist_s = (check_car_s - car_s);
          		if((check_car_s-car_s) > 20)
//          		if((check_car_s-car_s) > 30)
          		{
                          cout << "Car in Front...Reduce Speed and Check if Safe to Change Lanes"  << endl;
          	          //match that cars speed
          		  ref_vel = check_speed*2.237;
          		  change_lanes = true;
          		}
          		else
          		{
                          cout << "Car in Front...Go SLOWER and Check if Safe to Change Lanes "  << endl;
          	          //go slightly slower than the cars speed
             		  ref_vel = check_speed*2.237-5;
//             		  ref_vel = check_speed*2.237-2;
          		  change_lanes = true;
          		}
                       }


               	    }
          	} //  END OF FOR LOOP THROUGH SENSOR FUSION
          	
//////////////////////////////////////////////////////////////////////////////////////////          	
          	//try to change lanes if too close to car in front
          	if(change_lanes && ((next_wp-lane_change_wp)%map_waypoints_x.size() > 2))
          	{
         	  bool changed_lanes = false;

                  if (lane_0_free_at_least_100m && lane_0_free_score == 0 && lane_0_free_score_global <= 0) {
                    cout << "--------------------Free Lane 0 at least 100m"  << endl;
                  }
                  if (lane_1_free_at_least_100m && lane_1_free_score == 0 && lane_1_free_score_global <= 0) {
                    cout << "--------------------Free Lane 1 at least 100m"  << endl;
                  }
                  if (lane_2_free_at_least_100m && lane_2_free_score == 0 && lane_2_free_score_global <= 0) {
                    cout << "--------------------Free Lane 2 at least 100m"  << endl;
                  }

                  if ( car_lane == 0) {
                    change_to_left_lane = false;
                    change_to_right_lane = true;
                    left_lane_free_score -= 1;
                    right_lane_free_score += 1;
                  }
                  if ( car_lane == 2) {
                    change_to_left_lane = true;
                    change_to_right_lane = false;
                    left_lane_free_score -= 1;
                    right_lane_free_score += 1;
                  }
          	  //first try to change to left lane
//          	  if(lane != 0 && !changed_lanes && change_to_left_lane )
//          	  if(lane != 0 && !changed_lanes )
//          	  if(lane != 0 && !changed_lanes && left_lane_free_score_global > 0 && right_lane_free_score_global == 0 )
//          	  if(lane != 0 && !changed_lanes && left_lane_free_score_global < right_lane_free_score_global  )
                  cout << "--------------------Left Lane Free Score  " << left_lane_free_score  << endl;
                  cout << "--------------------Right Lane Free Score " << right_lane_free_score  << endl;
                  cout << "--------------------Lane Free 0 Score     " << lane_0_free_score  << endl;
                  cout << "--------------------Lane Free 1 Score     " << lane_1_free_score  << endl;
                  cout << "--------------------Lane Free 2 Score     " << lane_2_free_score  << endl;

          	  if(lane != 0 && !changed_lanes && left_lane_free_score <= right_lane_free_score && change_to_left_lane   )
//          	  if(lane != 0 && !changed_lanes && left_lane_free_score_global < right_lane_free_score_global   )
//          	  if(lane != 0 && !changed_lanes && left_lane_free_score < right_lane_free_score   )
//          	  if(lane != 0 && !changed_lanes && left_lane_free_score < right_lane_free_score  )
//          	  if(lane != 0 && !changed_lanes && left_lane_free_score < right_lane_free_score && lane_0_free_score < lane_2_free_score && lane_2_free_score > 0  )
//          	  else if(lane != 0 && !changed_lanes && left_lane_free_score_global < right_lane_free_score_global  )
          	  {
                    cout << "Check if LEFT LANE is safe"  << endl;
          	    bool lane_safe = true;
          	    for(int i = 0; i < sensor_fusion.size(); i++)
                    { 	
                      //car is in left lane
                      float d = sensor_fusion[i][6];
          	      if(d < (2+4*(lane-1)+2) && d > (2+4*(lane-1)-2) )
          	      {
                        double vx = sensor_fusion[i][3];
          		double vy = sensor_fusion[i][4];
          		double check_speed = sqrt(vx*vx+vy*vy);
          		double other_car_speed = sqrt(vx*vx+vy*vy);
          		double other_car_s = sensor_fusion[i][5];
          		other_car_s+=((double)prev_size*.02*other_car_speed);
          		double dist_s = other_car_s-car_s;
          		float other_car_d = sensor_fusion[i][6];
                        int other_car_lane = convert_d_to_lane(other_car_d);
          	        double ref_vel = 49.5; 

            		if(dist_s < 15 && dist_s > -10)
//            		if(dist_s < 15 && dist_s > -20)
//            		if(dist_s < 10 && dist_s > -20)
//  AGGRESSIVE_CRASH          		if(dist_s < 10 && dist_s > -15)
//          		if(dist_s < 20 && dist_s > -20)
//          		if(dist_s < 25 && dist_s > -25)
//          		if(dist_s > 25 && dist_s < -30)
//          		if(dist_s < 30 && dist_s > -30)
          		{
//             		  ref_vel = check_speed*2.237-7;
//             		  ref_vel = check_speed*2.237+2;
//             		  ref_vel = check_speed*2.237+5;
//             		  ref_vel = check_speed*2.237+3;
//             		  ref_vel = check_speed*2.237+4;
//             		  ref_vel = check_speed*2.237+3;
//             		  ref_vel = check_speed*2.237;
             		  ref_vel = check_speed*2.237 -2 ;
//             		  ref_vel = check_speed*2.237 - 5;
                          cout << "Speed up and change LEFT "  << endl;
          	  	  lane_safe = false;
//                          cout << "WARNING:  CAR ON LEFT...don't change to left. Other Car in Lane " << other_car_lane << endl;
          		}
          	       }
          	      }
                      if(lane_safe)
          	      {
          		changed_lanes = true;
          		lane -= 1;
          		lane_change_wp = next_wp;
          		}
          	       }
//////////////////////////////////////////////////////////////////////////////////////////          	
          	       //next try to change to right lane
          	       if(lane != 2 && !changed_lanes)
          	       {
                         cout << "Check if RIGHT LANE is safe"  << endl;
          	         bool lane_safe = true;
          		 for(int i = 0; i < sensor_fusion.size(); i++)
          	   	 {
          	  	   //car is in right lane
          		   float d = sensor_fusion[i][6];
          	           if(d < (2+4*(lane+1)+2) && d > (2+4*(lane+1)-2) )
          		   {
          	     	     double vx = sensor_fusion[i][3];
          		     double vy = sensor_fusion[i][4];
          		     double check_speed = sqrt(vx*vx+vy*vy);
          		     double other_car_speed = sqrt(vx*vx+vy*vy);

          		     double check_car_s = sensor_fusion[i][5];
          		     double other_car_s = sensor_fusion[i][5];
          		     other_car_s+=((double)prev_size*.02*other_car_speed);
          		     double dist_s = other_car_s-car_s;
          		     float other_car_d = sensor_fusion[i][6];
                             int other_car_lane = convert_d_to_lane(other_car_d);
          	             double ref_vel = 49.5; 

            		     if(dist_s < 15 && dist_s > -10)
//            		     if(dist_s < 10 && dist_s > -15)
//            		     if(dist_s < 15 && dist_s > -20)
//          		     if(dist_s < 10 && dist_s > -20)
//          		     if(dist_s < 10 && dist_s > -15)
//          		     if(dist_s < 20 && dist_s > -20)
//          		     if(dist_s < 25 && dist_s > -25)
//          		     if(dist_s > 25 && dist_s < -30)
//                             if(dist_s < 30 && dist_s > -30)
          		     {
//             		       ref_vel = check_speed*2.237-9;
                               cout << "Speed up and change RIGHT "  << endl;
//             		       ref_vel = check_speed*2.237+2;
//             		       ref_vel = check_speed*2.237+5;
//             		       ref_vel = check_speed*2.237+4;
//             		       ref_vel = check_speed*2.237+3;
//             		       ref_vel = check_speed*2.237;
             		       ref_vel = check_speed*2.237 - 2;
//             		       ref_vel = check_speed*2.237 - 5;
          		       lane_safe = false;
          		     }
//                             cout << "WARNING:  CAR ON RIGHT...don't change to right. Other Car in Lane " << other_car_lane  << endl;
          		     }
          		    }
          		    if(lane_safe)
          		    {
          		      changed_lanes = true;
          		      lane += 1;
          		      lane_change_wp = next_wp;
          		     }
          		}
          	}
//////////////////////////////////////////////////////////////////////////////////////////          	

          	vector<double> ptsx;
          	vector<double> ptsy;

          	if(prev_size < 2)
          	{
          		double prev_car_x = car_x - cos(car_yaw);
          		double prev_car_y = car_y - sin(car_yaw);

          		ptsx.push_back(prev_car_x);
          		ptsx.push_back(car_x);

          		ptsy.push_back(prev_car_y);
          		ptsy.push_back(car_y);
          	}
          	else
          	{
          		ptsx.push_back(previous_path_x[prev_size-2]);
          		ptsx.push_back(previous_path_x[prev_size-1]);

          		ptsy.push_back(previous_path_y[prev_size-2]);
          		ptsy.push_back(previous_path_y[prev_size-1]);


          	}

          	vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          	vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          	vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

          	ptsx.push_back(next_wp0[0]);
          	ptsx.push_back(next_wp1[0]);
          	ptsx.push_back(next_wp2[0]);

          	ptsy.push_back(next_wp0[1]);
          	ptsy.push_back(next_wp1[1]);
          	ptsy.push_back(next_wp2[1]);

          	
          	for (int i = 0; i < ptsx.size(); i++ )
          	{
          	
          		//shift car reference angle to 0 degrees
          		double shift_x = ptsx[i]-ref_x;
          		double shift_y = ptsy[i]-ref_y;

				ptsx[i] = (shift_x *cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
				ptsy[i] = (shift_x *sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));

          	}
          	
          	
          	tk::spline s;

          	
          	s.set_points(ptsx,ptsy);

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

                //  Previous Path 
          	for(int i = 0; i < previous_path_x.size(); i++)
          	{
          		next_x_vals.push_back(previous_path_x[i]);
          		next_y_vals.push_back(previous_path_y[i]);
          	}

          	double target_x = 30.0;
          	double target_y = s(target_x);
          	double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
          	
          	double x_add_on = 0;

			for (int i = 1; i <= 50-previous_path_x.size(); i++) {
				
				if(ref_vel > car_speed)
				{
					car_speed+=.224;
				}
				else if(ref_vel < car_speed)
				{
					car_speed-=.224;
				}


				double N = (target_dist/(.02*car_speed/2.24));
				double x_point = x_add_on+(target_x)/N;
				double y_point = s(x_point);

				x_add_on = x_point;

				double x_ref = x_point;
				double y_ref = y_point;

				x_point = (x_ref *cos(ref_yaw)-y_ref*sin(ref_yaw));
				y_point = (x_ref *sin(ref_yaw)+y_ref*cos(ref_yaw));

				x_point += ref_x;
				y_point += ref_y;


				next_x_vals.push_back(x_point);
				next_y_vals.push_back(y_point);
			}

          	json msgJson;
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
