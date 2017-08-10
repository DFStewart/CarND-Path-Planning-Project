#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
//Additional Includes
#include "car.h"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;
using spline = tk::spline;

double ref_vel            = 0.0;
bool   debug_lanescore    = 1;
double CAR_PROX           = 40.0; // Proximity to other cars used for lane scoring
double TGT_LANE           = 1.0;  // Current lane target
double HORIZON            = 30.0; // Timestep horizon to plan over
double TGT_SPD            = 49.5; // Current speed target (MPH)
double GAIN_CHGLANE       = 900.0; // Gain on lane change cost
double DESIRED_SPD        = 47; // Desired Speed (MPH)
double GAIN_COLLISION     = 50.0;
double GAIN_SLOWSPD       = 3.0;
bool   prev_lane_change   = false;
int    PREV_TGT_LANE      = 1.0;
bool  lane_chg_inprogress = false;

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

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
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

// Set the current lane of the car
void setlane(Car &car)
{
	car.lane = 0;
	if(car.pos_d > 8.0)
		car.lane = 2;
	else if(car.pos_d > 4.0)
		car.lane = 1;
}

// Score the lanes
vector<double> lane_score(Car &our_car, vector<Car> &other_cars)
{
	vector<double> lanescores(3);
	vector<double> cars_in_lane(3);

	//Initialize lane scores to 0
	lanescores[0] = 0;
	lanescores[1] = 0;
	lanescores[2] = 0;

	//Tally the cars in each lane
	for(int i=0; i<other_cars.size();i++)
	{
		if(other_cars[i].lane == 0)
			cars_in_lane[0] += 1.0;
		else if(other_cars[i].lane == 1)
			cars_in_lane[1] += 1.0;
		else if(other_cars[i].lane == 2)
			cars_in_lane[2] += 1.0;
	}
	if(debug_lanescore)
	{
		cout << "Cars in Lane 0: " << cars_in_lane[0] << endl;
		cout << "Cars in Lane 1: " << cars_in_lane[1] << endl;
		cout << "Cars in Lane 2: " << cars_in_lane[2] << endl;
	}

	//Find lane with lowest cars
	double min_car_lane = our_car.lane; //initialize to our current lane
	for(int i=0; i<cars_in_lane.size();i++)
	{
		if(cars_in_lane[i] < cars_in_lane[min_car_lane])
			min_car_lane = i;
	}
	if(debug_lanescore)
	{
		cout << "Minimum Cars in Lane: " << min_car_lane << endl;
	}

	//Score Lanes based on distance to cars in current lane
	for(int i=0; i<other_cars.size();i++)
	{
		double dist2othercar    = other_cars[i].pos_s - our_car.pos_s;
		bool   othercar_behind  = (dist2othercar < 0);
		bool   proximity        = fabs(dist2othercar) > CAR_PROX;
		if(othercar_behind && proximity)
			lanescores[other_cars[i].lane] -= 1;
		if(proximity)
			lanescores[other_cars[i].lane] += 99;
	}

	if(debug_lanescore)
	{
		cout << "Lane 0 Score: " << lanescores[0] << endl;
		cout << "Lane 1 Score: " << lanescores[1] << endl;
		cout << "Lane 2 Score: " << lanescores[2] << endl;
	}

	//Find minimum lane score for best lane
	TGT_LANE = our_car.lane;
	double minscore = 999;
	for(int i=0; i<lanescores.size();i++)
	{
		if(lanescores[i] < minscore)
		{
			minscore = lanescores[i];
			TGT_LANE = i;
		}
	}

	if(debug_lanescore)
	{
		cout << "Best Lane: " << TGT_LANE << endl;
	}

	return lanescores;
}

// Compute cost in case of collision
double cost_Collisions(Car &our_car, vector<Car> &other_cars,int lane_tgt)
{
	double cost = 0.0;
	for(int i=0; i<other_cars.size();i++)
	{
		//If car is in our desired lane
		//cout << "other_cars[i].lane" << other_cars[i].lane << endl;
		if(other_cars[i].lane == lane_tgt)
		{

			//Predict over time horizon
			for(int j=0; j<30; j++)
			{
				double pred_other_car_s = (other_cars[i].pos_s - 10.0) + other_cars[i].spd * 2.0 * j;
				double pred_our_car_s   = our_car.pos_s + our_car.spd * j;
				double dist2othercar    = pred_other_car_s - pred_our_car_s;
				bool   othercar_behind  = (dist2othercar < 0);
				bool   proximity        = fabs(dist2othercar) < CAR_PROX;
				//if(j == 0)
				//	cout << "Car in Lane "<< other_cars[i].lane << " at s,d: " <<  dist2othercar << ", " << other_cars[i].pos_d <<endl;
				//if(othercar_behind && proximity)
					//cost -= 0.0; // if car is behind us a sufficient amount no cost added
				if(proximity)
					cost = cost + fabs(1.0 - 1.0/fabs(dist2othercar))*GAIN_COLLISION;// large cost added because collision expected
			}

		}
	}
	return cost;
}

// Compute cost to change lane
double cost_ChangeLane(Car &our_car, vector<Car> &other_cars, int lane_tgt)
{
	double cost = 0.0;
	if(lane_tgt != our_car.lane)
	{
		cost = GAIN_CHGLANE*fabs(lane_tgt - our_car.lane); // Weight more heavily 2 lane changes
	}

	return cost;
}

// Compute cost to slow down
double cost_SlowDown(Car &our_car, int tgt_spd)
{
	double cost = 0.0;
	if(DESIRED_SPD > (tgt_spd-0.224))
	{
		cost = GAIN_SLOWSPD * fabs(50.0 - our_car.spd);
	}

	return cost;
}

//Identify and Detect Vehicle ahead of our vehicle
int detect_vehicle_ahead(Car &our_car, vector<Car> &other_cars)
{
  int other_car_idx = -1;
  double min_dist   = 1e9;
  double distance   = 1e9;
  bool car_ahead    = false;
  bool closest_car  = false;
  //Loop over all the other cars nearby
  for(int i = 0; i < other_cars.size(); i++)
  {
	  //Find only cars in our lane
	  if (our_car.lane == other_cars[i].lane)
	  {
		  distance = (other_cars[i].pos_s - our_car.pos_s);
		  car_ahead   = distance > 0.0;
		  closest_car = distance < min_dist;
		  // Narrow down to nearest car
		  if (car_ahead && closest_car)
		  {
			  other_car_idx = i;
			  min_dist    = distance;
		  }
	  }
  }
  return other_car_idx;
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

  Car our_car;
  vector<Car> other_cars;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&our_car,&other_cars](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

        	///////////////////////////////////////////
        	// Process Data from Simulator
        	///////////////////////////////////////////

        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	//Process our vehicle's data into a class
          	our_car.pos_s   = car_s;
          	our_car.pos_d   = car_d;
          	our_car.pos_x   = car_x;
          	our_car.pos_y   = car_y;
          	our_car.accel_s = 0.0;
          	our_car.accel_d = 0.0;
          	setlane(our_car);

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	//Process other vehicles data into vector of classes
          	vector<Car> other_cars(sensor_fusion.size());
          	for(int i=0;i < sensor_fusion.size();i++)
          	{
				other_cars[i].pos_s   = sensor_fusion[i][5];
				other_cars[i].pos_d   = sensor_fusion[i][6];
				double vs_x           = sensor_fusion[i][3];
				double vs_y           = sensor_fusion[i][4];
				double vel_s          = sqrt(vs_x*vs_x + vs_y*vs_y)/50.0;
				other_cars[i].vel_d   = 0.0;
				other_cars[i].vel_s   = vel_s;
				other_cars[i].accel_d = 0.0;
				other_cars[i].accel_s = 0.0;
				setlane(other_cars[i]);
          	}

          	cout << "Our Car is in Lane : " << our_car.lane << endl;

          	// Previous Path initialization
          	int prev_size = previous_path_x.size();

          	if(prev_size >0)
          	{
          		our_car.pos_s = end_path_s;
          	}

          	///////////////////////////////////////////
          	// Lane Scoring
          	///////////////////////////////////////////
          	if(prev_size >0)
          	{
          		cout << "//------------------------------------------//" << endl;
				vector <double> lanescores = lane_score(our_car,other_cars); // Sets TGT_LANE

				cout << "//-------------//" << endl;

				// Find other car to match speed if one is in our lane ahead of us
				int nearest_car_idx = detect_vehicle_ahead(our_car, other_cars);
				double dist_to_nearest = 999.0;
				bool car_ahead = false;
				if (nearest_car_idx != -1)
				{
					dist_to_nearest = fabs(other_cars[nearest_car_idx].pos_s - our_car.pos_s);
					if (dist_to_nearest < CAR_PROX)
					{
						car_ahead = true;
					}
				 }

				if(car_ahead)
					TGT_SPD  = other_cars[nearest_car_idx].spd;

				///////////////////////////////////////////
				// Finite State Machine (FSM)
				///////////////////////////////////////////
				bool FSM_KeepLane    = true;
				bool FSM_ChangeLaneL = !(our_car.lane == 0);
				bool FSM_ChangeLaneR = !(our_car.lane == 2);
				double costs_KL = 999.0;//keep lane
				double costs_CL = 999.0;//change lane left
				double costs_CR = 999.0;//change lane right
				int lane_tgt  = our_car.lane;
				double collision_cost  = 0.0;
				double changelane_cost = 0.0;
				double slowdown_cost   = 0.0;
				double speed_target    = DESIRED_SPD;

				if(FSM_KeepLane)
				{
					lane_tgt 		= our_car.lane;
					collision_cost  = cost_Collisions(our_car, other_cars,lane_tgt);
					changelane_cost = cost_ChangeLane(our_car, other_cars,lane_tgt);
					speed_target    = TGT_SPD;
					slowdown_cost   = cost_SlowDown(our_car, speed_target);
					costs_KL        = collision_cost + slowdown_cost;

					cout << "Keep Lane Collision Cost    : " << collision_cost << endl;
					//cout << "Keep Lane Change Cost       : " << changelane_cost << endl;
					//cout << "Keep Lane Slow Down Cost    : " << slowdown_cost << endl;

				}

				if(FSM_ChangeLaneL)
				{
					lane_tgt 		= our_car.lane-1;
					collision_cost  = cost_Collisions(our_car, other_cars,lane_tgt);
					changelane_cost = cost_ChangeLane(our_car, other_cars,lane_tgt);
					speed_target    = TGT_SPD;
					slowdown_cost   = cost_SlowDown(our_car, speed_target);
					costs_CL        = collision_cost + changelane_cost;
					cout << "Change Lane L Collision Cost: " << collision_cost << endl;
					//cout << "Change Lane L Change Cost   : " << changelane_cost << endl;
					//cout << "Change Lane L Slow Down Cost: " << slowdown_cost << endl;

				}

				if(FSM_ChangeLaneR)
				{
					lane_tgt 		= our_car.lane+1;
					collision_cost  = cost_Collisions(our_car, other_cars,lane_tgt);
					changelane_cost = cost_ChangeLane(our_car, other_cars,lane_tgt);
					speed_target    = TGT_SPD;
					slowdown_cost   = cost_SlowDown(our_car, speed_target);
					costs_CR        = collision_cost + changelane_cost;
					cout << "Change Lane R Collision Cost: " << collision_cost << endl;
					//cout << "Change Lane R Change Cost   : " << changelane_cost << endl;
					//cout << "Change Lane R Slow Down Cost: " << slowdown_cost << endl;
				}

				vector<double> costs;
				costs.push_back(costs_KL);
				costs.push_back(costs_CL);
				costs.push_back(costs_CR);

				//Find Minimum Cost Trajectory
				double min_cost = 999.0;
				int    min_cost_idx = 0;
				for(int i=0; i<costs.size();i++)
				{
					if(costs[i]<min_cost)
					{
						min_cost = costs[i];
						min_cost_idx = i;
					}
				}

				cout << "Keep Lane Cost    : " << costs_KL << endl;
				cout << "Change Left Cost  : " << costs_CL << endl;
				cout << "Change Right Cost : " << costs_CR << endl;

				if(min_cost_idx == 0)
				{
					cout << "Best Decision: " << "Keep Lane" << endl;
				}

				if(min_cost_idx == 1)
				{
					cout << "Best Decision: " << "Change Lane L" << endl;
				}

				if(min_cost_idx == 2)
				{
					cout << "Best Decision: " << "Change Lane R" << endl;
				}

				// State selected, set speed and lane targets
				FSM_KeepLane    = min_cost_idx == 0;
				FSM_ChangeLaneL = min_cost_idx == 1;
				FSM_ChangeLaneR = min_cost_idx == 2;

				if(FSM_KeepLane)
				{
					TGT_SPD  = 47.0;
					TGT_LANE = our_car.lane;
				}

				if(FSM_ChangeLaneL)
				{
					TGT_SPD  = 47.0;
					TGT_LANE = our_car.lane-1;
					if(!lane_chg_inprogress)
					{
						lane_chg_inprogress = true;
						PREV_TGT_LANE = TGT_LANE;
					}
				}

				if(FSM_ChangeLaneR)
				{
					TGT_SPD  = 47.0;
					TGT_LANE = our_car.lane+1;
					if(!lane_chg_inprogress)
					{
						lane_chg_inprogress = true;
						PREV_TGT_LANE = TGT_LANE;
					}
				}

				if(lane_chg_inprogress)
				{
					//cout << "/--LANE CHANGE IN PROGRESS--/" << endl;
					//cout << "PREV_TGT_LANE: " << PREV_TGT_LANE << endl;
					if(fabs((PREV_TGT_LANE*4+2)-our_car.pos_d) < 0.15)
					{
						lane_chg_inprogress = false;
					}
					TGT_LANE = PREV_TGT_LANE;
				}
				else
				{
					PREV_TGT_LANE = TGT_LANE;
				}
				cout << "//-------------//" << endl;
				cout << "FINAL TGT SPEED: " << TGT_SPD  <<endl;
				cout << "FINAL TGT LANE:  " << TGT_LANE <<endl;
				cout << "PREV  TGT LANE:  " << PREV_TGT_LANE <<endl;

          	}//end if prev_path_x.size() <0

			///////////////////////////////////////////
			// EMERGENCY BRAKING
			///////////////////////////////////////////

			// Find other car to match speed if one is in our lane ahead of us
			int nearest_car_idx = detect_vehicle_ahead(our_car, other_cars);
			double dist_to_nearest = 999.0;
			bool emergency_brake = false;
			if (nearest_car_idx != -1)
			{
				dist_to_nearest = fabs(other_cars[nearest_car_idx].pos_s - our_car.pos_s);
				if (dist_to_nearest < 10.0)
				{
					emergency_brake = true;
				}
			}

			//Emergency braking
			if(emergency_brake)
			{
				TGT_SPD = 30.0;//other_cars[nearest_car_idx].spd-0.05;//drive a little slower than the car in front
				cout << "EMERGENCY BRAKE" << endl;
			}

          	//Set velocity and lane targets
          	if(ref_vel > TGT_SPD)//Speed Up
          	{
          		if(emergency_brake)
          		    ref_vel -= 0.224;
          		else
          		    ref_vel -= 0.224/4.0;
          	}
          	else if(ref_vel < TGT_SPD)//Slow Down
          	{
          			ref_vel += 0.224;
          	}

          	vector<double> ptsx;
          	vector<double> ptsy;

          	double ref_x = car_x;
			double ref_y = car_y;
			double ref_yaw = deg2rad(car_yaw);

          	if(prev_size <2)
          	{
          		double prev_car_x = our_car.pos_x - cos(car_yaw);
          		double prev_car_y = our_car.pos_y - sin(car_yaw);

          		ptsx.push_back(prev_car_x);
          		ptsx.push_back(our_car.pos_x);

          		ptsy.push_back(prev_car_y);
          		ptsy.push_back(our_car.pos_y);

          	}
          	else
          	{
          		ref_x = previous_path_x[prev_size-1];
          		ref_y = previous_path_y[prev_size-1];

          		double ref_x_prev = previous_path_x[prev_size-2];
          		double ref_y_prev = previous_path_y[prev_size-2];
          		ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

          		ptsx.push_back(ref_x_prev);
          		ptsx.push_back(ref_x);

          		ptsy.push_back(ref_y_prev);
          		ptsy.push_back(ref_y);

          	}

          	//In Frenet space add waypoints 30 m ahead of the starting reference
          	vector<double> next_wp0 = getXY(our_car.pos_s + 30, (2+4*TGT_LANE),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          	vector<double> next_wp1 = getXY(our_car.pos_s + 60, (2+4*TGT_LANE),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          	vector<double> next_wp2 = getXY(our_car.pos_s + 90, (2+4*TGT_LANE),map_waypoints_s,map_waypoints_x,map_waypoints_y);

          	ptsx.push_back(next_wp0[0]);
          	ptsx.push_back(next_wp1[0]);
          	ptsx.push_back(next_wp2[0]);

          	ptsy.push_back(next_wp0[1]);
          	ptsy.push_back(next_wp1[1]);
          	ptsy.push_back(next_wp2[1]);

          	for (int i=0; i <ptsx.size(); i++)
          	{
          		double shift_x = ptsx[i]-ref_x;
          		double shift_y = ptsy[i]-ref_y;

          		ptsx[i] = (shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
          		ptsy[i] = (shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
          	}

          	spline s;
          	s.set_points(ptsx,ptsy);

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

          	for(int i=0; i< previous_path_x.size(); i++)
          	{
          		next_x_vals.push_back(previous_path_x[i]);
          		next_y_vals.push_back(previous_path_y[i]);
          	}

          	//Calculate how to split up our spline points so we travel at our target velocity
          	double target_x = HORIZON;//horizon
          	double target_y = s(target_x);
          	double target_dist = sqrt(target_x*target_x+target_y*target_y);

          	double x_add_on = 0.0;

          	for(int i=1;i<=50-previous_path_x.size();i++)
          	{
          		double N = (target_dist/(0.02*ref_vel/2.24));
          		double x_point = x_add_on+(target_x)/N;
          		double y_point = s(x_point);

          		x_add_on = x_point;

          		double x_ref = x_point;
          		double y_ref = y_point;

          		x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
          		y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));

          		x_point += ref_x;
          		y_point += ref_y;

          		next_x_vals.push_back(x_point);
          		next_y_vals.push_back(y_point);
          	}

          	cout << "ref_vel: " << ref_vel << endl;

          	// define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
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
