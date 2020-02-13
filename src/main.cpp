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
  // Initialize PID controller
  std::vector<double> p = {0.229981, 0.000849273, 1.70926};
  pid.Init(p[0], p[1], p[2]);

  // Variables for Twiddle algorithm
  std::vector<double> dp = {0.01, 0.0001, 0.1};
  int i = 0;
  double best_error = 99999.9;
  double total_error = 0.0;
  bool tried_decreasing_dp = false;
  bool twiddle = false;
  int max_timestep = 750;

  h.onMessage([&pid, &p, &dp, &i, &best_error, &total_error,
               &tried_decreasing_dp, &twiddle,
               &max_timestep](uWS::WebSocket<uWS::SERVER> ws, char *data,
                              size_t length, uWS::OpCode opCode) {
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
          total_error += pow(cte, 2);

          // Optional Twiddle for parameter tuning
          if (twiddle == true) {
            if (dp[0] + dp[1] + dp[2] > 0.001) {
              if (pid.counter > max_timestep) {
                // debug info
                std::cout << "-------------------------" << std::endl;
                std::cout << "i: " << i << std::endl;
                std::cout << "p: " << p[0] << ", " << p[1] << ", " << p[2]
                          << std::endl;
                std::cout << "dp: " << dp[0] << ", " << dp[1] << ", " << dp[2]
                          << std::endl;

                // RESET after max timestep is reached
                std::string reset_msg = "42[\"reset\",{}]";
                ws.send(reset_msg.data(), reset_msg.length(),
                        uWS::OpCode::TEXT);
                pid.counter = 0;

                // check how to change parameters according to error
                if (total_error < best_error) {
                  best_error = total_error;
                  std::cout << "best error: " << total_error << std::endl;
                  dp[i] *= 1.1;
                  std::cout << "INCREASE dp[" << i << "] "
                            << "& BUMP UP NEXT" << std::endl;
                  tried_decreasing_dp = false;
                } else if (tried_decreasing_dp == false) {
                  p[i] -= 3 * dp[i];
                  i -= 1;  // run with same i once more
                  tried_decreasing_dp = true;
                  std::cout << "BUMP DOWN p[" << i + 1 << "]" << std::endl;

                } else if (tried_decreasing_dp == true) {
                  p[i] += dp[i];  // bump back up
                  dp[i] *= 0.9;
                  tried_decreasing_dp = false;
                  std::cout << "NARROW p[" << i << "] "
                            << "& BUMP UP NEXT" << std::endl;
                }
                total_error = 0;
                i++;
                i %= 3;  // keep looping i = 0..2 until tolerance is reached
                p[i] += dp[i];  // modify p with dp for next steps
                pid.Init(p[0], p[1], p[2]);
              }
            } else {
              p[i] -= dp[i];
              pid.Init(p[0], p[1], p[2]);
              std::cout << "Found optimal parameters: " << p[0] << ", " << p[1]
                        << ", " << p[2] << std::endl;
              twiddle = false;
            }
          }

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value
                    << std::endl;
          std::cout << "Average Error : " << pid.AverageError() << " ["
                    << pid.MinError() << ", " << pid.MaxError() << "]"
                    << std::endl;

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
    // std::cout << "Connected!!!" << std::endl;
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