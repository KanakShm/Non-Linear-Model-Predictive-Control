#ifndef FG_EVAL_HPP
#define FG_EVAL_HPP

#include <tuple>

#include "predictor.hpp"
#include "utils.hpp"

class FG_eval : public Predictor
{
public:
  typedef CppAD::AD<double> AD;
  typedef CPPAD_TESTVECTOR(AD) ADvector;

  FG_eval(const std::vector<Point32> & points, const MPCConfig & mpc_config);
  std::tuple<AD, AD> projectPoint(AD x0, AD y0, AD psi0);
  void operator()(ADvector & fg, const ADvector & vars);

private:
  const std::vector<Point32> & waypoints;
  double car_length_;
};

#endif
