#include "mpc/FG_eval.hpp"

FG_eval::FG_eval(const std::vector<Point32> & points, const MPCConfig & mpc_config)
: Predictor(mpc_config), waypoints(points) {}

std::tuple<FG_eval::AD, FG_eval::AD> FG_eval::projectPoint(
  AD x0, AD y0, AD psi0)
{
  if (waypoints.empty()) {
    std::cout << "Waypoints empty!" << std::endl;
    return std::make_tuple(
      CppAD::numeric_limits<double>::max(),
      CppAD::numeric_limits<double>::max());
  }

  double min_dist = std::numeric_limits<double>::max();
  Point32 closest_point = waypoints[0];
  Point32 next_point = waypoints[0];

  const double xd = CppAD::Value(CppAD::Var2Par(x0));
  const double yd = CppAD::Value(CppAD::Var2Par(y0));

  for (size_t i = 0; i < waypoints.size() - 1; i++) {
    const Point32 & p1 = waypoints[i];
    const Point32 & p2 = waypoints[i + 1];

    const double dx = p1.x - xd;
    const double dy = p1.y - yd;
    const double dist = dx * dx + dy * dy;

    if (dist < min_dist) {
      min_dist = dist;
      closest_point = p1;
      next_point = p2;
    }
  }

  // Switching to AD math
  const AD dx = AD(next_point.x - closest_point.x);
  const AD dy = AD(next_point.y - closest_point.y);

  // Heading error
  const AD desired_heading = CppAD::atan2(dy, dx);
  const AD epsi = desired_heading - psi0;

  // Cross-track error
  const AD CTE = (dx * (y0 - closest_point.y) - dy * (x0 - closest_point.x)) /
    CppAD::sqrt(dx * dx + dy * dy);

  if (CppAD::isnan(CTE) || CppAD::isnan(epsi)) {
    return std::make_tuple(
      CppAD::numeric_limits<double>::max(),
      CppAD::numeric_limits<double>::max());
  }

  return std::make_tuple(-CTE, epsi);
}


void FG_eval::operator()(ADvector & fg, const ADvector & vars)
{
  fg[0] = 0;

  // 1. Tack reference trajectory
  for (size_t i = 0; i < mpc_config.N; ++i) {
    AD x = vars[x_start + i];
    AD y = vars[y_start + i];
    AD psi = vars[psi_start + i];

    // core tracking costs
    fg[0] += mpc_config.wt1 * CppAD::pow((vars[cte_start + i] - mpc_config.ref_cte), 2);
    fg[0] += mpc_config.wt2 * CppAD::pow(vars[epsi_start + i] - mpc_config.ref_epsi, 2);
    fg[0] += mpc_config.wt3 * CppAD::pow(vars[v_start + i] - mpc_config.ref_v, 2);

    // coupling term between heading error and velocity
    fg[0] += mpc_config.wt4 * CppAD::pow(vars[v_start + i] * vars[epsi_start + i], 2);
  }

  // 2. Minimise actuator usage
  for (size_t i = 0; i < mpc_config.N - 1; ++i) {
    fg[0] += mpc_config.wt5 * CppAD::pow(vars[delta_start + i], 2);
    fg[0] += mpc_config.wt6 * CppAD::pow(vars[a_start + i], 2);
  }

  // 3. Minimise rate of change of actuation
  for (size_t i = 0; i < mpc_config.N - 2; ++i) {
    AD x = vars[x_start + i];
    AD y = vars[y_start + i];
    AD psi = vars[psi_start + i];

    fg[0] += mpc_config.wt7 * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
    fg[0] += mpc_config.wt8 * CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);

    // Additional penalties
    fg[0] += mpc_config.wt9 * CppAD::pow(vars[delta_start + i] * vars[epsi_start + i], 2);
    fg[0] += mpc_config.wt10 * CppAD::pow(vars[a_start + i] * vars[epsi_start + i], 2);
    fg[0] += mpc_config.wt11 * CppAD::pow(vars[v_start + i] * vars[cte_start + i], 2);
  }

  // 4. Penalise reverse motion
  for (size_t i = 0; i < mpc_config.N; ++i) {
    AD dx = vars[x_start + i] - vars[x_start];
    AD v = vars[v_start + i];

    // Just a big number based on cte and epsi tracking
    AD reverse_wt = (mpc_config.wt1 + mpc_config.wt2) + (mpc_config.wt1 * mpc_config.wt2);

    fg[0] += reverse_wt * CppAD::pow(CppAD::CondExpLe(dx, AD(0.0), AD(reverse_wt), AD(0.0)), 2);
    fg[0] += reverse_wt * CppAD::pow(CppAD::CondExpLe(v, AD(0.0), -v, AD(0.0)), 2);

    // incentivise forward direction
    fg[0] -= reverse_wt * CppAD::pow(CppAD::CondExpGt(dx, AD(0.0), AD(2), AD(0.0)), 2);
  }

  // Model constraints
  fg[1 + x_start] = vars[x_start];
  fg[1 + y_start] = vars[y_start];
  fg[1 + psi_start] = vars[psi_start];
  fg[1 + v_start] = vars[v_start];
  fg[1 + cte_start] = vars[cte_start];
  fg[1 + epsi_start] = vars[epsi_start];

  for (size_t i = 0; i < mpc_config.N - 1; i++) {
    // State at time t
    const AD x0 = vars[x_start + i];
    const AD y0 = vars[y_start + i];
    const AD psi0 = vars[psi_start + i];
    const AD v0 = vars[v_start + i];
    const AD delta0 = vars[delta_start + i];
    const AD a0 = vars[a_start + i];

    auto [cte0, epsi0] = projectPoint(x0, y0, psi0);
    cte0 = AD(cte0);
    epsi0 = AD(epsi0);

    // State at time t+1
    const AD x1 = vars[x_start + i + 1];
    const AD y1 = vars[y_start + i + 1];
    const AD psi1 = vars[psi_start + i + 1];
    const AD v1 = vars[v_start + i + 1];
    const AD cte1 = vars[cte_start + i + 1];
    const AD epsi1 = vars[epsi_start + i + 1];

    // Kinematic bicycle model
    fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * mpc_config.dt);
    fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * mpc_config.dt);
    fg[2 + psi_start + i] = psi1 -
      (psi0 + v0 * CppAD::tan(delta0) / mpc_config.car_length * mpc_config.dt);
    fg[2 + v_start + i] = v1 - (v0 + a0 * mpc_config.dt);
    fg[2 + cte_start + i] = cte1 - (cte0 + v0 * CppAD::sin(epsi0) * mpc_config.dt);
    fg[2 + epsi_start + i] = epsi1 -
      ((epsi0) + v0 * CppAD::tan(delta0) / mpc_config.car_length * mpc_config.dt);
  }
}
