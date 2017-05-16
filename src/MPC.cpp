#include <math.h>
#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <cppad/local/limits.hpp>
#include <cppad/poly.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// DONE: Set the timestep length and duration
size_t N = 5;
double dt = 0.1;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;
// symmetric difference quotient
// const AD<double> h_ad_double = CppAD::numeric_limits<AD<double>>::epsilon();
const AD<double> h_ad_double = 1e-8;

// Both the reference cross track and orientation errors are 0.
// The reference velocity is set to 40 mph.
double ref_v = 40;

// set the weights of the error function.
double w_delta = 1.0; // 100.0;
double w_a = 1.0; // 500.0;

std::clock_t cur_time;
std::clock_t pre_time;

bool isFirstUpdate = true;

// DEBUG
int debug_tries = 3;
int debug_try = 0;
std::vector<double> x_vals = {};
std::vector<double> y_vals = {};
std::vector<double> psi_vals = {};
std::vector<double> v_vals = {};
std::vector<double> cte_vals = {};
std::vector<double> epsi_vals = {};
std::vector<double> delta_vals = {};
std::vector<double> a_vals = {};

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

// Evaluate a polynomial for AD.
AD<double> polyevalAD(Eigen::VectorXd coeffs, AD<double> x) {
  // cout << "polyevalAD: x= " << x << endl;
  AD<double> result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * CppAD::pow(x, i);
  }
  return result;
}

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // fg a vector of constraints, x is a vector of constraints.
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // cout << "op(): h_ad_double= " << h_ad_double << endl;
    cout << "op(): coeffs= " << coeffs << endl;
    // The part of the cost based on the reference state.
    AD<double> ad_ref_v=ref_v;
    for (int i = 0; i < N - 1; i++) {
      fg[0] += CppAD::pow(vars[cte_start + i + 1] - vars[cte_start + i], 2);
      fg[0] += CppAD::pow(vars[epsi_start + i + 1] - vars[epsi_start + i], 2);
      fg[0] += CppAD::pow(vars[v_start + i] - ad_ref_v, 2);
    }

    // Minimize the use of actuators.
    for (int i = 0; i < N - 1; i++) {
      // fg[0] += CppAD::pow(vars[delta_start + i], 2);
      fg[0] += CppAD::pow(vars[a_start + i], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (int i = 0; i < N - 2; i++) {
      // fg[0] += w_delta * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
      // fg[0] += w_a * CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
      fg[0] += CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
      fg[0] += CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
    }

    //
    // Setup Constraints
    //
    // NOTE: In this section you'll setup the model constraints.

    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (int i = 0; i < N - 1; i++) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + i + 1];
      AD<double> y1 = vars[y_start + i + 1];
      AD<double> psi1 = vars[psi_start + i + 1];
      AD<double> v1 = vars[v_start + i + 1];
      AD<double> cte1 = vars[cte_start + i + 1];
      AD<double> epsi1 = vars[epsi_start + i + 1];

      // The state at time t.
      AD<double> x0 = vars[x_start + i];
      AD<double> y0 = vars[y_start + i];
      AD<double> psi0 = vars[psi_start + i];
      AD<double> v0 = vars[v_start + i];
      AD<double> cte0 = vars[cte_start + i];
      AD<double> epsi0 = vars[epsi_start + i];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + i];
      AD<double> a0 = vars[a_start + i];
/*
      // AD<double> f0 = coeffs[0] + coeffs[1] * x0;
      cout << "op(): x0 = " << x0 << endl;
      AD<double> f0 = polyevalAD(coeffs, x0);
      // double xi = vars[x_start +i];
      // AD<double> f0 = polyeval(coeffs, xi);
      // AD<double> psides0 = CppAD::atan(coeffs[1]);
      // symmetric difference quotient
      AD<double> x0h2 = x0+h_ad_double;
      AD<double> fh2 = polyevalAD(coeffs, x0h2);
      AD<double> x0h1 = x0-h_ad_double;
      AD<double> fh1 = polyevalAD(coeffs, x0h1);
      AD<double> dfdx = (fh2 - fh1) / (2*h_ad_double);
      // AD<double> dfdx = Poly(1, coeffs, x0);  // f' at x0
      // double dfdx = CppAD::Poly(1, coeffs, CppAD::Value(x0));  // f' at x0
      cout << "op(): f0= " << f0 << endl;
      cout << "op(): x0h2= " << x0h2 << endl;
      cout << "op(): x0h1= " << x0h1 << endl;
      cout << "op(): fh2= " << fh2 << endl;
      cout << "op(): fh1= " << fh1 << endl;
      cout << "op(): dfdx= " << dfdx << endl;
*/
      // AD<double> psides0 = -CppAD::atan(dfdx);
      AD<double> coeffs1 = coeffs[1];
      AD<double> coeffs2 = coeffs[2];
      AD<double> coeffs3 = coeffs[3];
      AD<double> psides0 = CppAD::atan(coeffs1 + (2*coeffs2*x0) + (3*coeffs3*(x0*x0)));
      // TODO
      psides0 += M_PI;

      cout << "OOOOOop(): psides0= " << psides0 << endl;

      AD<double> ad_dt=dt;
      cout << "op(): ad_dt= " << ad_dt << endl;

      AD<double> ad_Lf=Lf;
      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // Recall the equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * ad_dt);
      fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * ad_dt);
      fg[2 + psi_start + i] = psi1 - (psi0 + v0 * delta0 / ad_Lf * ad_dt);
      fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
      // TODO: f0 - y0
      // fg[2 + cte_start + i] =
      //          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[2 + cte_start + i] =
              cte1 - (cte0 + (v0 * CppAD::sin(epsi0) * dt));
      fg[2 + epsi_start + i] =
             epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  if (isFirstUpdate) {
    pre_time = std::clock();
    isFirstUpdate = false;
  }

  cur_time = std::clock();
  dt = (cur_time - pre_time) * 1.0 / CLOCKS_PER_SEC; // delta time per second'

  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // DONE: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = 6*N + 2*(N-1);
  // DONE: Set the number of constraints
  size_t n_constraints = 6*N;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // DONE: Set lower and upper limits for variables.
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }
  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // print out solution
  int n = N-1;

  for (int i = 0; i < n; i++) {
    cout << "s" << i << ": "
         << solution.x[i + x_start + 1] << " "
         << solution.x[i + y_start + 1] << " "
         << solution.x[i + psi_start + 1] << " "
         << solution.x[i + v_start + 1] << " "
         << solution.x[i + cte_start + 1] << " "
         << solution.x[i + epsi_start + 1] << " "
         << solution.x[i + delta_start]  << " "
         << solution.x[i + a_start] << endl;
  }

  pre_time = cur_time;

  // debug
  cte_vals.push_back(solution.x[cte_start +1]);
  delta_vals.push_back(solution.x[delta_start +1]);

    /*
  if (debug_try > debug_tries) {
    // Plot values for debugging
    plt::subplot(2, 1, 1);
    plt::title("CTE");
    plt::plot(cte_vals);
    plt::subplot(2, 1, 2);
    plt::title("Delta (Radians)");
    plt::plot(delta_vals);

    plt::show();
    // std::exit(1);
  }
  debug_try++;
     */

  // DONE: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  return {solution.x[x_start + 1],   solution.x[y_start + 1],
          solution.x[psi_start + 1], solution.x[v_start + 1],
          solution.x[cte_start + 1], solution.x[epsi_start + 1],
          solution.x[delta_start],   solution.x[a_start]};
}
