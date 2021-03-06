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
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }


constexpr double LANE_WIDTH_METERS = 4.0;
constexpr int INITIAL_LANE_ID = 1;
constexpr int TRAJECTORY_LENGTH = 50;
constexpr double SIMULATION_PERIOD_SECS = 0.02;
constexpr double TARGET_SPEED_MPH = 49.0;
constexpr double MPH_TO_METERS_PER_SECOND = 0.447;

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

/// Returns true when other car is in the same lane with my car.
bool in_same_lane(double my_car_d, double other_car_d){
	return other_car_d >= my_car_d - LANE_WIDTH_METERS / 2 
									&& other_car_d <= my_car_d + LANE_WIDTH_METERS / 2;
}

/// Returns true when other car is in in front and close of my car.
bool close_and_in_front(double my_car_s, double other_car_s, double too_close_distance_meters){
	return other_car_s >= my_car_s && other_car_s - my_car_s <= too_close_distance_meters;
}

/// Converts points in global coordinates to vehicle coordinates.
void convert_points_to_vehicle_coordinates(vector<double>& pts_x, vector<double>& pts_y, 
			const double car_x, const double car_y, const double car_yaw)
{
	for(int i = 0; i < pts_x.size(); ++i)
	{
		double shift_x = pts_x[i] - car_x;
		double shift_y = pts_y[i] - car_y;
		pts_x[i] = shift_x * cos(0 - car_yaw) - shift_y * sin(0 - car_yaw);
		pts_y[i] = shift_x * sin(0 - car_yaw) + shift_y * cos(0 - car_yaw);
	}
}

/// Converts point in vehicle coordinates to global coordinates.
void convert_point_to_global_coordinates(double& x, double& y, 	
			const double car_x, const double car_y, const double car_yaw){
	
	double original_x = x;
	double original_y = y;

	x = original_x * cos(car_yaw) - original_y * sin(car_yaw);
	y = original_x * sin(car_yaw) + original_y * cos(car_yaw);

	x += car_x;
	y += car_y;
}

/// print trajectory points.
void print_trajectory(const vector<double>& next_x_vals, const vector<double>& next_y_vals){
	cout << "##### Print Path #####" << endl;
	for(int i = 0; i < next_x_vals.size(); ++i)
	{
		cout << "path x: " << next_x_vals[i] << ", path y: " << next_y_vals[i] << endl;
	}
	cout << "path len: " << next_x_vals.size() << endl;
}

int main() {
  uWS::Hub h;

	// 0: left lane, 1: middle lane, 2: right lane.
	int curr_lane = 1;
	double curr_speed_mph = 0.0;

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

  h.onMessage([&curr_speed_mph, &curr_lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;
            int previous_path_size = previous_path_x.size();
          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
			bool too_close = false;
			bool left_lane_change_safe =  curr_lane != 0;
			bool right_lane_change_safe = curr_lane != 2;

			double future_car_s = car_s + previous_path_size * SIMULATION_PERIOD_SECS * curr_speed_mph * MPH_TO_METERS_PER_SECOND;
			double future_car_d = curr_lane * LANE_WIDTH_METERS + LANE_WIDTH_METERS / 2;
			for(const auto& tracked_car : sensor_fusion)
			{
				double tracked_car_x = tracked_car[1];
				double tracked_car_y = tracked_car[2];
				double tracked_car_vx = tracked_car[3];
				double tracked_car_vy = tracked_car[4];
				double tracked_car_s = tracked_car[5];
				double tracked_car_d = tracked_car[6];							
				
				tracked_car_x += previous_path_size * SIMULATION_PERIOD_SECS * tracked_car_vx;
				tracked_car_y += previous_path_size * SIMULATION_PERIOD_SECS * tracked_car_vy;
				
				auto future_tracked_car_frenet = getFrenet(tracked_car_x, tracked_car_y, 
															atan2(tracked_car_vy, tracked_car_vx), 
															map_waypoints_x, 
															map_waypoints_y);
				double future_tracked_car_s = future_tracked_car_frenet[0];
				double future_tracked_car_d = future_tracked_car_frenet[1];
				
				if(in_same_lane(future_car_d, future_tracked_car_d) 
						&& close_and_in_front(future_car_s, future_tracked_car_s, 20.0)) 
				{
					too_close = true;
					continue;
				}

				// check whether it is safe for left lane change.
				if(in_same_lane(future_car_d - LANE_WIDTH_METERS, future_tracked_car_d)
						&& (close_and_in_front(future_car_s, future_tracked_car_s, 20.0) 
									|| close_and_in_front(future_tracked_car_s, future_car_s, 15.0)
									|| close_and_in_front(car_s, tracked_car_s, 15.0)
									|| close_and_in_front(tracked_car_s, car_s, 15.0)
								)
				) {	
					left_lane_change_safe = false;
				}

				// check whether it is safe for right lane change.
				if(in_same_lane(future_car_d + LANE_WIDTH_METERS, future_tracked_car_d)
						&& (close_and_in_front(future_car_s, future_tracked_car_s, 20.0) 
									|| close_and_in_front(future_tracked_car_s, future_car_s, 15.0)
									|| close_and_in_front(car_s, tracked_car_s, 15.0)
									|| close_and_in_front(tracked_car_s, car_s, 15.0)
								)
				) {
					right_lane_change_safe = false;
				}
			}

			if(too_close)
			{
				if(left_lane_change_safe)
				{
					--curr_lane;
				} 
				else if (right_lane_change_safe)
				{
					++curr_lane;
				}
				else 
				{
					curr_speed_mph = max(0.0, curr_speed_mph - 0.2);
				}
			}
			else if(curr_speed_mph < TARGET_SPEED_MPH)
			{
				curr_speed_mph += 0.2;		
			}

			cout << "too close: " << too_close 
						<< ", left_lane_change_safe:" << left_lane_change_safe 
						<< ", right_lane_change_safe: " << right_lane_change_safe 
						<< ", target lane: " << curr_lane << endl;
		

			// Choose anchor points.
			vector<double> pts_x;
			vector<double> pts_y;
			double ref_x = car_x;
			double ref_y = car_y;
			double ref_yaw = deg2rad(car_yaw);
			if(previous_path_size >= 2)
			{
				ref_x = previous_path_x[previous_path_size - 1];
				ref_y = previous_path_y[previous_path_size - 1];
				double ref_x_prev = previous_path_x[previous_path_size - 2];
				double ref_y_prev = previous_path_y[previous_path_size - 2];
				ref_yaw = atan2(ref_y - ref_y_prev,  ref_x - ref_x_prev);

				pts_x.push_back(ref_x_prev);
				pts_x.push_back(ref_x);

				pts_y.push_back(ref_y_prev);
				pts_y.push_back(ref_y);
			} 
			else 
			{
				double ref_x_prev = car_x - SIMULATION_PERIOD_SECS * cos(ref_yaw) * curr_speed_mph * MPH_TO_METERS_PER_SECOND;
				double ref_y_prev = car_y - SIMULATION_PERIOD_SECS * sin(ref_yaw) * curr_speed_mph * MPH_TO_METERS_PER_SECOND;

				pts_x.push_back(ref_x_prev);
				pts_x.push_back(ref_x);

				pts_y.push_back(ref_y_prev);
				pts_y.push_back(ref_y);
			}

			double new_d = curr_lane * LANE_WIDTH_METERS + 0.5 * LANE_WIDTH_METERS;
			vector<double> new_wp1 = getXY(car_s + 50, new_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
			vector<double> new_wp2 = getXY(car_s + 70, new_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
			vector<double> new_wp3 = getXY(car_s + 90, new_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

			pts_x.push_back(new_wp1[0]);
			pts_x.push_back(new_wp2[0]);
			pts_x.push_back(new_wp3[0]);

			pts_y.push_back(new_wp1[1]);	
			pts_y.push_back(new_wp2[1]);	
			pts_y.push_back(new_wp3[1]);	

			// Fit anchor points in vehicle coordinate.
			convert_points_to_vehicle_coordinates(pts_x, pts_y, ref_x, ref_y, ref_yaw);
			tk::spline s;
			s.set_points(pts_x, pts_y);

			// Use previous path.
			for(int i = 0; i < previous_path_size; ++i)
			{
				next_x_vals.push_back(previous_path_x[i]);
				next_y_vals.push_back(previous_path_y[i]);
			}

			double target_x = 30.0;
			double target_y = s(target_x);
			double target_distance = sqrt(target_x * target_x + target_y * target_y);
			int slice = target_distance / (curr_speed_mph * MPH_TO_METERS_PER_SECOND * SIMULATION_PERIOD_SECS);
			double x_unit = target_x / slice;

			for(int i = 0; i < TRAJECTORY_LENGTH - previous_path_size; ++i)
			{
				double new_x = x_unit * (i + 1);
				double new_y = s(new_x);

				convert_point_to_global_coordinates(new_x, new_y, ref_x, ref_y, ref_yaw);							
				next_x_vals.push_back(new_x);
				next_y_vals.push_back(new_y);
			}

						
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
