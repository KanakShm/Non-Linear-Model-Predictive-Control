#include "mpc/predictor.hpp"
#include <yaml-cpp/yaml.h>
#include <iostream>

Predictor::Predictor() {}

Predictor::Predictor(const MPCConfig & init)
{
  // Trajectory horizon
  mpc_config.N = init.N;
  mpc_config.dt = init.dt;

  // Reference
  mpc_config.ref_epsi = init.ref_epsi;
  mpc_config.ref_cte = init.ref_cte;
  mpc_config.ref_v = init.ref_v;

  // Weights
  mpc_config.wt1 = init.wt1;
  mpc_config.wt2 = init.wt2;
  mpc_config.wt3 = init.wt3;
  mpc_config.wt4 = init.wt4;
  mpc_config.wt5 = init.wt5;
  mpc_config.wt6 = init.wt6;
  mpc_config.wt7 = init.wt7;
  mpc_config.wt8 = init.wt8;
  mpc_config.wt9 = init.wt9;
  mpc_config.wt10 = init.wt10;
  mpc_config.wt11 = init.wt11;

  // Universal
  mpc_config.car_length = init.car_length;

  // Indices for fg vector
  x_start = 0;
  y_start = x_start + mpc_config.N;
  psi_start = y_start + mpc_config.N;
  v_start = psi_start + mpc_config.N;
  cte_start = v_start + mpc_config.N;
  epsi_start = cte_start + mpc_config.N;
  delta_start = epsi_start + mpc_config.N;
  a_start = delta_start + mpc_config.N - 1;
}

Predictor::Predictor(const std::string & MPC_YAML)
{
  YAML::Node config;
  try {
    config = YAML::LoadFile(MPC_YAML);
  } catch (const YAML::BadFile & e) {
    std::cerr << "Failed to load YAML: " << e.what() << std::endl;
    return;
  }

  const YAML::Node & MPCParams = config["mpc_node"]["ros__parameters"];

  // Trajectory horizon
  mpc_config.N = MPCParams["Horizon"]["N"] ? MPCParams["Horizon"]["N"].as<int>() : 10;
  mpc_config.dt = MPCParams["Horizon"]["dt"] ? MPCParams["Horizon"]["dt"].as<double>() : 0.2;

  // Reference
  mpc_config.ref_cte =
    MPCParams["Horizon"]["ref_cte"] ? MPCParams["Horizon"]["ref_cte"].as<double>() : 0.0;
  mpc_config.ref_epsi =
    MPCParams["Horizon"]["ref_epsi"] ? MPCParams["Horizon"]["ref_epsi"].as<double>() : 0.0;
  mpc_config.ref_v =
    MPCParams["Horizon"]["ref_v"] ? MPCParams["Horizon"]["ref_v"].as<double>() : 5.0;

  hot_lap_speed =
    MPCParams["Horizon"]["hot_lap_speed"] ? MPCParams["Horizon"]["hot_lap_speed"].as<double>() : 1.0;
  hot_lap_speed_weight =
    MPCParams["Horizon"]["hot_lap_speed_weight"] ? MPCParams["Horizon"]["hot_lap_speed_weight"].as<double>()
    :
    500;

  // Weights
  mpc_config.wt1 = MPCParams["FG_eval"]["wt1"] ? MPCParams["FG_eval"]["wt1"].as<int>() : 5000;
  mpc_config.wt2 = MPCParams["FG_eval"]["wt2"] ? MPCParams["FG_eval"]["wt2"].as<int>() : 10000;
  mpc_config.wt3 = MPCParams["FG_eval"]["wt3"] ? MPCParams["FG_eval"]["wt3"].as<int>() : 4;
  mpc_config.wt4 = MPCParams["FG_eval"]["wt4"] ? MPCParams["FG_eval"]["wt4"].as<int>() : 1000;
  mpc_config.wt5 = MPCParams["FG_eval"]["wt5"] ? MPCParams["FG_eval"]["wt5"].as<int>() : 200;
  mpc_config.wt6 = MPCParams["FG_eval"]["wt6"] ? MPCParams["FG_eval"]["wt6"].as<int>() : 50;
  mpc_config.wt7 = MPCParams["FG_eval"]["wt7"] ? MPCParams["FG_eval"]["wt7"].as<int>() : 300;
  mpc_config.wt8 = MPCParams["FG_eval"]["wt8"] ? MPCParams["FG_eval"]["wt8"].as<int>() : 1000;
  mpc_config.wt9 = MPCParams["FG_eval"]["wt9"] ? MPCParams["FG_eval"]["wt9"].as<int>() : 0;
  mpc_config.wt10 = MPCParams["FG_eval"]["wt10"] ? MPCParams["FG_eval"]["wt10"].as<int>() : 0;
  mpc_config.wt11 = MPCParams["FG_eval"]["wt11"] ? MPCParams["FG_eval"]["wt11"].as<int>() : 0;

  // Universal
  mpc_config.car_length =
    MPCParams["universal"]["car_length_"] ? MPCParams["universal"]["car_length_"].as<double>() : 1.7;

  x_start = 0;
  y_start = x_start + mpc_config.N;
  psi_start = y_start + mpc_config.N;
  v_start = psi_start + mpc_config.N;
  cte_start = v_start + mpc_config.N;
  epsi_start = cte_start + mpc_config.N;
  delta_start = epsi_start + mpc_config.N;
  a_start = delta_start + mpc_config.N - 1;

  config_path = MPC_YAML;
}
