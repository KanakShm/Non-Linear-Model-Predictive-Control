// Documentation on using CppAD with IPOPT solver can be found at
// https://coin-or.github.io/CppAD/doc/ipopt_solve.htm

#ifndef MPC_HPP
#define MPC_HPP

#define NUM_VARIABLES 6
#define NUM_ACTUATORS 2

#include <vector>
#include <cppad/ipopt/solve.hpp>

#include "mpc/FG_eval.hpp"

typedef CPPAD_TESTVECTOR(double) Dvector;

class MPC : public Predictor
{
public:
  struct MPCPrediction
  {
    std::vector<double> x_pred;
    std::vector<double> y_pred;
    std::vector<double> psi_pred;
    std::vector<double> v_pred;
    std::vector<double> cte_pred;
    std::vector<double> epsi_pred;
    std::vector<double> delta_pred;
    std::vector<double> a_pred;
    std::vector<int> cost_terms = std::vector<int>(11, 0);
  };

  MPC(const std::string & MPC_config);
  ~MPC();
  MPCPrediction solve(
    const Eigen::VectorXd & state,
    const std::vector<geometry_msgs::msg::Point32> & points, int lap_count);

private:
  double pose_lowerbound;
  double pose_upperbound;

  double v_lowerbound;
  double v_upperbound;

  double cte_lowerbound;
  double cte_upperbound;

  double epsi_lowerbound;
  double epsi_upperbound;

  double delta_deg_lowerbound;
  double delta_deg_upperbound;

  double a_lowerbound;
  double a_upperbound;

  size_t n_vars;
  size_t n_constraints;

  Dvector vars;
  MPCPrediction prediction;
};

#endif
