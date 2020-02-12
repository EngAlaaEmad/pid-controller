#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "PID.h"
#include "json.hpp"

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
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;
  /**
   * TODO: Initialize the pid variable.
   */
  std::vector<double> p = {0.05, 0.0001, 1.0};
  std::vector<double> dp = p;
  int i = 0;
  double best_error = 99999.9;
  double error = 0.0;
  bool decreased_dp = false;
  bool increased_dp = false;
  bool twiddle = true;
  pid.Init(p[0], p[1], p[2]);

  h.onMessage([&pid, &p, &dp, &i, &best_error, &error, &decreased_dp,
               &increased_dp, &twiddle](uWS::WebSocket<uWS::SERVER> ws,
                                        char *data, size_t length,
                                        uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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
          pid.UpdateError(cte);
          double steer_value = -1 * pid.TotalError();
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          // Twiddle

          // std::cout << pid.counter << std::endl;
          // outer while loop for tolerance
          if (twiddle == true) {
            if (dp[0] + dp[1] + dp[2] > 0.10) {
              // do 100 steps w new param
              if (pid.counter > 750 || cte > 4.0) {
                std::cout << "p: " << p[0] << ", " << p[1] << ", " << p[2]
                          << std::endl;
                std::cout << "dp: " << dp[0] << ", " << dp[1] << ", " << dp[2]
                          << std::endl;
                double error = pid.AverageError() + pid.MaxError();
                // RESET
                std::string reset_msg = "42[\"reset\",{}]";
                ws.send(reset_msg.data(), reset_msg.length(),
                        uWS::OpCode::TEXT);
                std::cout << "RESET" << std::endl;
                pid.counter = 0;
                // check where to go
                if (error < best_error) {
                  best_error = error;
                  std::cout << "best error: " << best_error << std::endl;
                  dp[i] *= 1.1;
                  std::cout << "INCREASE" << std::endl;
                } else if (decreased_dp == false) {
                  p[i] -= 3 * dp[i];
                  i -= 1;  // run with same i once more
                  decreased_dp = true;
                  std::cout << "DECREASE" << std::endl;

                } else if (decreased_dp == true) {
                  p[i] += dp[i];  // bump back up
                  dp[i] *= 0.9;
                  decreased_dp = false;
                  std::cout << "NARROW" << std::endl;
                }
                i++;
                i %= 3;  // keep looping i = 0..2 until tolerance is reached
                p[i] += dp[i];  // modify p for next n steps
                pid.Init(p[0], p[1], p[2]);
              }
            } else {
              // p[i] -= dp[i]
              std::cout << "Found optimal parameters: " << p[0] << ", " << p[1]
                        << ", " << p[2] << std::endl;
              twiddle = false;
            }
          }

          // DEBUG
          /*std::cout << "CTE: " << cte << " Steering Value: " << steer_value
                    << std::endl;
          std::cout << "Average Error : " << pid.AverageError() << " ["
                    << pid.MinError() << ", " << pid.MaxError() << "]"
                    << std::endl;
                    */

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  });  // end h.onMessage

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