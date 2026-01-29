#ifndef PREDICTOR_HPP
#define PREDICTOR_HPP

#include <string>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include <cppad/cppad.hpp>
#include <eigen3/Eigen/Dense>

#include "geometry_msgs/msg/point32.hpp"

class Predictor
{
public:
  struct MPCConfig
  {
    // Trajectory horizon
    size_t N;
    double dt;

    // Reference
    double ref_cte;
    double ref_epsi;
    double ref_v;

    // Weights
    int wt1, wt2, wt3, wt4, wt5, wt6, wt7, wt8, wt9, wt10, wt11;

    // Global constants
    double car_length;
  };

  Predictor();
  Predictor(const std::string & MPC_YAML);
  Predictor(const MPCConfig & init);
  virtual ~Predictor() = default;

protected:
  std::string config_path;
  MPCConfig mpc_config{};

  // Indices for fg vector
  size_t x_start;
  size_t y_start;
  size_t psi_start;
  size_t v_start;
  size_t cte_start;
  size_t epsi_start;
  size_t delta_start;
  size_t a_start;

  double hot_lap_speed;
  int hot_lap_speed_weight;
};

#endif
