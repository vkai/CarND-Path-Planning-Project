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
#include "utils.h"
#include "fsm.h"
#include "jmt.h"
#include "vehicle.h"
#include "map.h"
#include "road.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double get_lane_d(int lane) { return (LANE_WIDTH*lane) + (LANE_WIDTH/2); }

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

// Generate new path points from JMT
void generateNewPoints(	Map map, 
                        vector<double> jmt_s, 
                        vector<double> jmt_d, 
                        vector<double> &path_x, 
                        vector<double> &path_y) {
	for(double t = 0; t <= HORIZON; t += TIME_INCR) {
		double s = 0;
		double d = 0;
		for(int i = 0; i < jmt_s.size(); i++) {
			s += jmt_s[i] * pow(t, i);
			d += jmt_d[i] * pow(t, i);
		}

		vector<double> map_point = map.getXY(fmod(s, MAX_S), d);
		path_x.push_back(map_point[0]);
		path_y.push_back(map_point[1]);
	}
}

Map readAndCreateMap(string map_file) {
	// Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

	ifstream in_map_(map_file.c_str(), ifstream::in);
	
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
	// append first waypoint to end to complete loop around map
	map_waypoints_x.push_back(map_waypoints_x[0]);
	map_waypoints_y.push_back(map_waypoints_y[0]);
	map_waypoints_s.push_back(MAX_S);
	map_waypoints_dx.push_back(map_waypoints_dx[0]);
	map_waypoints_dy.push_back(map_waypoints_dy[0]);

	Map map(map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy);

	return map;
}


int main() {
  uWS::Hub h;

  // Waypoint map to read from
	string map_file = "../data/highway_map.csv";

	Map map = readAndCreateMap(map_file);
	Road road;
	FSM fsm;
	Vehicle self_vehicle;

  h.onMessage([&map, &road, &fsm, &self_vehicle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
	        double car_x = j[1]["x"];
	        double car_y = j[1]["y"];
	        double car_s = j[1]["s"];
	        double car_d = j[1]["d"];
	        double car_yaw = j[1]["yaw"];
	        double car_speed = j[1]["speed"];

	        self_vehicle.x = car_x;
	        self_vehicle.y = car_y;
	        self_vehicle.s = car_s;
	        self_vehicle.d = car_d;
	        self_vehicle.speed = car_speed;

	        // Previous path data given to the Planner
	        auto previous_path_x = j[1]["previous_path_x"];
	        auto previous_path_y = j[1]["previous_path_y"];
	        // Previous path's end s and d values 
	        double end_path_s = j[1]["end_path_s"];
	        double end_path_d = j[1]["end_path_d"];

	        // Sensor Fusion Data, a list of all other cars on the same side of the road.
	        auto sensor_fusion = j[1]["sensor_fusion"];
	        vector<Vehicle> vehicles;
	        for(int i = 0; i < sensor_fusion.size(); i++) {
		        double x = sensor_fusion[i][1];
		        double y = sensor_fusion[i][2];
		        double vx = sensor_fusion[i][3];
		        double vy = sensor_fusion[i][4];
		        double s = sensor_fusion[i][5];
		        double d = sensor_fusion[i][6];
		        double v = sqrt(vx*vx + vy*vy);

		        Vehicle vehicle(x,y,s,d,v);
		        vehicles.push_back(vehicle);
	        }
	        road.setRoad(vehicles);


	        vector<double> next_x_vals = previous_path_x;
	        vector<double> next_y_vals = previous_path_y;
	        vector<double> start_s = self_vehicle.prev_s;
	        vector<double> start_d = self_vehicle.prev_d;

	        double start_speed = self_vehicle.speed;
	        double target_speed = MAX_SPEED / 2;
	        double target_d = get_lane_d(self_vehicle.get_lane());
	
	        bool generate_new_points = false;
	        if(previous_path_x.size() == 0) {
		        start_s = {self_vehicle.s, self_vehicle.speed, 0};
		        start_d = {get_lane_d(self_vehicle.get_lane()), 0, 0};
		        self_vehicle.setPrev(start_s, start_d);

		        generate_new_points = true;
	        }
	        else if(previous_path_x.size() < 20) {
		        start_speed = self_vehicle.prev_s[1];
		        target_speed = min(start_speed + 2, MAX_SPEED);

		        State next_state = fsm.transitionState(self_vehicle, road);
		        if(next_state == State::KEEP_LANE) {
			        vector<double> distances = road.findClosestDistanceForwardAndBehind(self_vehicle, self_vehicle.get_lane());
			        double distanceForward = distances[1];
			        if(distanceForward <= 30) {
				        target_speed = start_speed * 0.8;
			        }
		        }
		        else if(next_state == State::CHANGE_LEFT) {
			        target_d = get_lane_d(self_vehicle.get_lane() - 1);
		        }
		        else if(next_state == State::CHANGE_RIGHT) {
			        target_d = get_lane_d(self_vehicle.get_lane() + 1);
		        }
		        // right lane adjustment
		        if(target_d >= 10.0) {
			        target_d -= 0.2;
		        }

		        generate_new_points = true;
	        }

	        if(generate_new_points) {
		        double target_s = self_vehicle.prev_s[0] + ((start_speed + target_speed)/2 * HORIZON);
		        vector<double> end_s = {target_s, target_speed, 0.0};
		        vector<double> end_d = {target_d, 0.0, 0.0};

	          self_vehicle.setPrev(end_s, end_d);

		        vector<double> jmt_s = JMT(start_s, end_s, HORIZON);
		        vector<double> jmt_d = JMT(start_d, end_d, HORIZON);

		        generateNewPoints(map, jmt_s, jmt_d, next_x_vals, next_y_vals);
		        cout << "StartS: " << start_s[0] << "StartD: " << start_d[0] << "; TargetS: " << target_s << " TargetD: " << target_d << endl;
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
