#include "mpc/MPC.hpp"

MPC::MPC(const std::string & MPC_config)
: Predictor(MPC_config)
{
  // Load YAML config from file path stored in config
  YAML::Node parameters = YAML::LoadFile(MPC_config);
  const YAML::Node & params = parameters["mpc_node"]["ros__parameters"];

  // MPC parameters
  pose_lowerbound =
    params["MPC"]["pose_lowerbound"] ? params["MPC"]["pose_lowerbound"].as<double>() : -20;
  pose_upperbound =
    params["MPC"]["pose_upperbound"] ? params["MPC"]["pose_upperbound"].as<double>() : 20;
  std::cout << "pose upperbound: " << pose_upperbound << std::endl;
  std::cout << "pose lowerbound: " << pose_lowerbound << std::endl;

  v_lowerbound = params["MPC"]["v_lowerbound"] ? params["MPC"]["v_lowerbound"].as<double>() : -20;
  v_upperbound = params["MPC"]["v_upperbound"] ? params["MPC"]["v_upperbound"].as<double>() : 20;
  std::cout << "v upperbound: " << v_upperbound << std::endl;
  std::cout << "v lowerbound: " << v_lowerbound << std::endl;

  cte_lowerbound =
    params["MPC"]["cte_lowerbound"] ? params["MPC"]["cte_lowerbound"].as<double>() : -10;
  cte_upperbound =
    params["MPC"]["cte_upperbound"] ? params["MPC"]["cte_upperbound"].as<double>() : 10;
  std::cout << "cte upperbound: " << cte_upperbound << std::endl;
  std::cout << "cte lowerbound: " << cte_lowerbound << std::endl;

  epsi_lowerbound =
    params["MPC"]["epsi_lowerbound"] ? params["MPC"]["epsi_lowerbound"].as<double>() : -15;
  epsi_upperbound =
    params["MPC"]["epsi_upperbound"] ? params["MPC"]["epsi_upperbound"].as<double>() : 15;
  std::cout << "epsi upperbound: " << epsi_upperbound << std::endl;
  std::cout << "epsi lowerbound: " << epsi_lowerbound << std::endl;

  delta_deg_lowerbound =
    params["MPC"]["delta_deg_lowerbound"] ? params["MPC"]["delta_deg_lowerbound"].as<double>() : -24;
  delta_deg_upperbound =
    params["MPC"]["delta_deg_upperbound"] ? params["MPC"]["delta_deg_upperbound"].as<double>() : 24;
  std::cout << "delta upperbound: " << delta_deg_upperbound << std::endl;
  std::cout << "delta lowerbound: " << delta_deg_lowerbound << std::endl;

  a_lowerbound = params["MPC"]["a_lowerbound"] ? params["MPC"]["a_lowerbound"].as<double>() : -1.0;
  a_upperbound = params["MPC"]["a_upperbound"] ? params["MPC"]["a_upperbound"].as<double>() : 1.0;
  std::cout << "a upperbound: " << a_upperbound << std::endl;
  std::cout << "a lowerbound: " << a_lowerbound << std::endl;

  // Initialise private variables
  n_vars = mpc_config.N * NUM_VARIABLES + (mpc_config.N - 1) * NUM_ACTUATORS;
  n_constraints = mpc_config.N * NUM_VARIABLES;
  vars = Dvector(n_vars);
  for (size_t i = 0; i < n_vars; ++i) {
    vars[i] = 0;
  }

  // Initialise prediction to empty
  prediction.x_pred.resize(mpc_config.N, 0.0);
  prediction.y_pred.resize(mpc_config.N, 0.0);
  prediction.psi_pred.resize(mpc_config.N, 0.0);
  prediction.v_pred.resize(mpc_config.N, 0.0);
  prediction.cte_pred.resize(mpc_config.N, 0.0);
  prediction.epsi_pred.resize(mpc_config.N, 0.0);
  prediction.delta_pred.resize(mpc_config.N - 1, 0.0);
  prediction.a_pred.resize(mpc_config.N - 1, 0.0);
}

MPC::~MPC() {}

MPC::MPCPrediction MPC::solve(
  const Eigen::VectorXd & state,
  const std::vector<geometry_msgs::msg::Point32> & points,
  int lap_count)
{
  if (points.empty()) {
    std::cout << "Waypoints empty! Returning the last or empty prediction" << std::endl;
    return prediction;
  }

  if ((state.array().isNaN()).any()) {
    std::cout << "State contains a NaN! Returning the last or empty prediction" << std::endl;
    return prediction;
  }

  // Variables
  const double x = state(0);
  const double y = state(1);
  const double psi = state(2);
  const double v = state(3);
  const double cte = state(4);
  const double epsi = state(5);

  // Actuators
  const double prev_steer_angle = state(6);
  const double prev_a = state(7);

  // Initial conditions
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  // Upper and lower bounds
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  for (size_t i = 0; i < v_start; i++) {
    vars_lowerbound[i] = pose_lowerbound;
    vars_upperbound[i] = pose_upperbound;
  }

  for (size_t i = v_start; i < cte_start; i++) {
    vars_lowerbound[i] = v_lowerbound;
    vars_upperbound[i] = v_upperbound;
  }

  for (size_t i = cte_start; i < epsi_start; i++) {
    vars_lowerbound[i] = cte_lowerbound;
    vars_upperbound[i] = cte_upperbound;
  }

  for (size_t i = epsi_start; i < delta_start; i++) {
    vars_lowerbound[i] = epsi_lowerbound;
    vars_upperbound[i] = epsi_upperbound;
  }

  for (size_t i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = delta_deg_lowerbound * M_PI / 180;
    vars_upperbound[i] = delta_deg_upperbound * M_PI / 180;
    vars[i] = prev_steer_angle;
  }

  for (size_t i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = a_lowerbound;
    vars_upperbound[i] = a_upperbound;
    vars[i] = prev_a;
  }

  // Constraits. All set to 0 except for initial states
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (size_t i = 0; i < n_constraints; i++) {
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

  // HOTLAPS
  if (lap_count > 0) {
    mpc_config.ref_v = hot_lap_speed;
    mpc_config.wt3 = hot_lap_speed_weight;
  }

  FG_eval fg_eval(points, mpc_config);

  std::string options;
  options += "Integer print_level 0\n";
  options += "Sparse  true        forward\n";
  // options += "Sparse  true        reverse\n";
  options += "Numeric max_cpu_time          0.5\n";
  CppAD::ipopt::solve_result<Dvector> solution;

  CppAD::ipopt::solve<Dvector, FG_eval>(
    options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
    constraints_upperbound, fg_eval, solution);


  if (solution.status != CppAD::ipopt::solve_result<Dvector>::success) {
    std::cout << "Solver has failed! Returning the last or empty prediction." << std::endl;
    return prediction;
  }

  for (size_t i = 0; i < mpc_config.N; i++) {
    prediction.x_pred[i] = solution.x[x_start + i];
    prediction.y_pred[i] = solution.x[y_start + i];
    prediction.psi_pred[i] = solution.x[psi_start + i];
    prediction.v_pred[i] = solution.x[v_start + i];
    prediction.cte_pred[i] = solution.x[cte_start + i];
    prediction.epsi_pred[i] = solution.x[epsi_start + i];
  }

  for (size_t i = 0; i < mpc_config.N - 1; i++) {
    prediction.delta_pred[i] = solution.x[delta_start + i];
    prediction.a_pred[i] = solution.x[a_start + i];
  }

  // Update vars array for the next iteration
  for (size_t i = 0; i < n_vars; i++) {
    vars[i] = solution.x[i];
  }

  prediction.cost_terms[0] = mpc_config.wt1 * pow(prediction.cte_pred[0] - mpc_config.ref_cte, 2);
  prediction.cost_terms[1] = mpc_config.wt2 * pow(prediction.epsi_pred[0] - mpc_config.ref_epsi, 2);
  prediction.cost_terms[2] = mpc_config.wt3 * pow(prediction.v_pred[0] - mpc_config.ref_v, 2);
  prediction.cost_terms[3] = mpc_config.wt4 * pow(prediction.v_pred[0] * prediction.epsi_pred[0], 2);
  prediction.cost_terms[4] = mpc_config.wt5 * pow(prediction.delta_pred[0], 2);
  prediction.cost_terms[5] = mpc_config.wt6 * pow(prediction.a_pred[0], 2);
  prediction.cost_terms[6] = mpc_config.wt7 * pow(prediction.delta_pred[1] - prediction.delta_pred[0], 2);
  prediction.cost_terms[7] = mpc_config.wt8 * pow(prediction.a_pred[1] - prediction.a_pred[0], 2);
  prediction.cost_terms[8] = mpc_config.wt9 * pow(prediction.delta_pred[0] * prediction.epsi_pred[0], 2);
  prediction.cost_terms[9] = mpc_config.wt10 * pow(prediction.a_pred[0] * prediction.epsi_pred[0], 2);
  prediction.cost_terms[10] = mpc_config.wt11 * pow(prediction.v_pred[0] * prediction.cte_pred[0], 2);


  return prediction;
}
