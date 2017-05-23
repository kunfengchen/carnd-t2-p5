#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"
#include "matplotlibcpp.h"


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
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

// cout helper
void print_vector(const std::vector<double>& v) {
  for (const auto& i: v) {
      std::cout << i << " ";
  }
  std::cout << endl;
}

// cout helper
void print_eigne_vector(const Eigen::VectorXd& v) {
  for (int i = 0; i < v.size(); i++) {
      std::cout << v[i] << " ";
  }
  std::cout << endl;
}

// A wrapper
Eigen::VectorXd polyfit(vector<double> xs, vector<double> ys, int order ) {
  size_t size_x = xs.size();
  size_t size_y = ys.size();

  double* x_ptr = &xs[0];
  double* y_ptr = &ys[0];

  // cout << "std: ";
  // print_vector(xs);
  // cout << "std: ";
  // print_vector(ys);

  Eigen::Map<Eigen::VectorXd> vx(x_ptr, size_x);
  Eigen::Map<Eigen::VectorXd> vy(y_ptr, size_y);

  // cout << "Eigen: ";
  // print_eigne_vector(vx);
  // cout << "Eigen: ";
  // print_eigne_vector(vy);

  return polyfit(vx, vy, order);
}

/**
 * Caculate the distance from point p to line (ps, pe) using linear algebra
 * Used in map coordination
 */
double distance_to_line(double p_start_x, double p_start_y,
                        double p_end_x, double p_end_y, double p_x, double p_y) {

  // line vector p to ps
  double p_ps_x = p_x - p_start_x;
  double p_ps_y = p_y - p_start_y;
  // line vector pe to ps
  double pe_ps_x = p_end_x - p_start_x;
  double pe_ps_y = p_end_y - p_start_y;
  // p_ps x pe_ps
  double cross_product = p_ps_x * pe_ps_y - p_ps_y * pe_ps_x;
  // norm
  double norm_pe_ps = std::sqrt(pe_ps_x*pe_ps_x+pe_ps_y*pe_ps_y);
  double dist = cross_product/norm_pe_ps;
  // cout << "cross_product= " << cross_product << " ,norm_pe_ps= "  << norm_pe_ps <<  " ,dist= "  << dist << std::endl;
  return dist;
}

/**
 * Translate coord from map to car
 * Formula suggested from class discussion
 * Counter-clockwise
 */
void to_car_coord (double src_map_x, double src_map_y,
                   double theta, double car_map_x, double car_map_y,
                    double& car_x, double& car_y) {
  double dis_x = src_map_x - car_map_x;
  double dis_y = src_map_y - car_map_y;
  car_x = dis_x*std::cos(theta) + dis_y*std::sin(theta);
  car_y = -dis_x*std::sin(theta) + dis_y*std::cos(theta);
}

// debug
int m_debug_tries = 5;
int m_debug_try = 0;

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    Eigen::VectorXd state(6);
    m_debug_try++;

    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          /*
          * DONE: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          //Display the waypoints/reference line
          int size_points = ptsx.size();
          vector<double> next_x_vals(size_points);
          vector<double> next_y_vals(size_points);

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          for (int i = 0; i < size_points; i++) {
            // cout << " map coor: " << ptsx[i] << ", " << ptsy[i] << std::endl;
            to_car_coord(ptsx[i], ptsy[i], psi, px, py, next_x_vals[i], next_y_vals[i]);
            // cout << " car coor: " << next_x_vals[i] << ", " << next_y_vals[i] << std::endl;
          }

          /// auto coeffs = polyfit(ptsx, ptsy, 3);  /// Using map coordinate
          /// According to class discussion, using car coordiante is easier.
          auto coeffs = polyfit(next_x_vals, next_y_vals, 3);  /// Using car coordinate
          // The cross track error is calculated by evaluating at polynomial at x, f(x)
          // and subtracting y.

          // NOTE: from MPC.cpp
          // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
          // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
          /// double cte = distance_to_line(ptsx[3], ptsy[3], ptsx[0], ptsy[0], px, py);  /// Using Map coordinate
          /// double cte = distance_to_line(next_x_vals[3], next_y_vals[3], next_x_vals[0], next_y_vals[0], 0.0, 0.0);  /// Using Car coordinate
          /// double cte = distance_to_line(next_x_vals[0], next_y_vals[0], next_x_vals[3], next_y_vals[3], 0.0, 0.0);  /// Using Car coordinate
          double cte = polyeval(coeffs, 0) - 0; /// car coordinates

          std::cout << "CTE= " << cte << std::endl;
          // Due to the sign starting at 0, the orientation error is -f'(x).
          // derivative of coeffs[0] + coeffs[1] * x -> coeffs[1]
          // double epsi = psi - std::atan(coeffs[1] + (2*coeffs[2]*px) + (3*coeffs[3]*(px*px)));
          // double psides = -std::atan(coeffs[1] + (2*coeffs[2]*ptsx[0]) + (3*coeffs[3]*(ptsx[0]*ptsx[0])));

          //// need to check quardrant ? maybe for Map coordinate
          double psides = std::atan(coeffs[1] + (2*coeffs[2]*px) + (3*coeffs[3]*px*px));
          /// psides+=pi();

          /*
          double psides = std::atan2(ptsy[2]-ptsy[0], ptsx[2]-ptsx[0]);
          if (psides < 0) {
             psides = 2*pi()+psides; // convert (0, -pi) to (pi, 2pi) for third and fourth quadrants.
          }
          */

          /// Car coordinates
          double car_px = 0.0;
          double car_py = 0.0;
          double car_psi = 0.0;
          cout << "main(): psides= " << psides << endl;
          psi = 0;  /// Car coorinate

          /// double epsi = psi - psides;
          double epsi = car_psi - psides;

          state << car_px, car_py, car_psi, v, cte, epsi;

          cout << "SIM state: ";
          print_eigne_vector(state);

          double steer_value;
          double throttle_value;

          auto vars = mpc.Solve(state, coeffs);

          /// cout << "MPC state: ";
          /// print_vector(vars);
          /// cout << endl;
          steer_value = -vars[6];
          /// steer_value = -0.01;
          throttle_value = vars[7];
          std::cout << "STEER_VALUE= " << steer_value
                    << " THROTTLE= " << throttle_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          int size_n = mpc.size_n;
          vector<double> mpc_x_vals(size_n-1);
          vector<double> mpc_y_vals(size_n-1);

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          for (int i=0; i<size_n-1; i++) {
            /// cout << " mpc map coor: " << vars[0 + i*8] << ", " << vars[1 + i*8] << std::endl;
            /// to_car_coord(vars[0 + i*8], vars[1 + i*8], psi, px, py, mpc_x_vals[i], mpc_y_vals[i]);
            mpc_x_vals[i] = vars[0 + i*8];
            mpc_y_vals[i] = vars[1 + i*8];
            cout << " mpc car coor: " << mpc_x_vals[i] << ", " << mpc_y_vals[i] << std::endl;
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          /// std::cout << msg << std::endl;

          // Latency
          // The purpose is to mimic real driving conditions where // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

            /*
          // DEBUG
          if ((m_debug_try % 2) == 0) {
            matplotlibcpp::figure();
            matplotlibcpp::xlim(-200, 200);
            matplotlibcpp::ylim(-200, 200);
            matplotlibcpp::plot(ptsx, ptsy, "go");
            matplotlibcpp::annotate("*", ptsx[0], ptsy[0]);
            matplotlibcpp::annotate("<", ptsx[2], ptsy[2]);
            matplotlibcpp::annotate(std::to_string(coeffs[0]) + ","
                    + std::to_string(coeffs[1]) + ","
                    + std::to_string(coeffs[2]) + ","
                    + std::to_string(coeffs[3]) + ","
                    , ptsx[3], ptsy[3]);
            int np = 100;
            int startx = ptsx[3] - np/2;
            vector<double> npx(np);
            vector<double> npy(np);
            for (int i = 0; i < np; i++) {
                npx[i] = startx + i;
                npy[i] = polyeval(coeffs, npx[i]);
            }
            matplotlibcpp::plot(npx, npy, "y");
            matplotlibcpp::plot({px}, {py}, "bo");
            matplotlibcpp::plot(ptsx, ptsy, "g");
            matplotlibcpp::annotate("x " + std::to_string(cte)
                                    // + " p:" + std::to_string(psi)
                                    // + " d:" + std::to_string(psides)
                                    + " e:" + std::to_string(epsi)
                                    + " s:" + std::to_string(steer_value), px, py);
          }

          if (m_debug_try > m_debug_tries) {
             matplotlibcpp::show();
             std::exit(1);
          }
             */
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
