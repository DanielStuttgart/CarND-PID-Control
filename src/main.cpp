#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

#include <vector>

// for convenience
using nlohmann::json;
using std::string;

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
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;  
  PID pid_long;
  
  // for determining range of cte --> -10 ... 8 --> -8 .. 8
  double min_cte = 1.;
  double max_cte = 0.;

  /**
   * TODO: Initialize the pid variable.
   */  
  pid.Init(0.12, 0.0001, 2.8);
  pid_long.Init(0.5, 0.0001, 2);

  h.onMessage([&pid, &pid_long, &min_cte, &max_cte](uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    bool debug_info = false;
    bool debug_pid = true;
    bool use_optimizer = false;
    double v_set = 35;

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */          
          pid.UpdateError(cte);
          if(use_optimizer)
            pid.Optimize(0.001);

          double lat_error = pid.TotalError();
          if (lat_error < min_cte)
              min_cte = lat_error;
          if (lat_error > max_cte)
              max_cte = lat_error;
          if (debug_pid)
              pid.Info();
          // limit the steering value to be in range [-1, 1]
          if(abs(lat_error) <= 1.)
            steer_value = lat_error;
          else {
              if (lat_error < -1)
                  steer_value = -1;
              else
                  steer_value = 1;
          }

          double long_error = speed - v_set;
          double throttle_value;
          pid_long.UpdateError(long_error);
          
          // limit the long value to be in range [-1, 1]
          throttle_value = pid_long.TotalError();
          
          throttle_value = pid_long.TotalError();              
          if (throttle_value < -0.2)
              throttle_value = -0.2;
          else if (throttle_value > 0.3)
              throttle_value = 0.3;
          
          // DEBUG
          if(debug_info)
            std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
                    << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          //msgJson["throttle"] = 0.3;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          
          if(debug_info)
            std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

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