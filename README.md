# Non-Linear Model Predictive Control (NMPC) for Autonomous Racing

A high-performance C++ implementation of a Non-Linear Model Predictive Controller designed for a Formula Student autonomous race car. This repository contains the core optimisation logic, vehicle modelling, and ROS 2 integration used by **UNSW Redback Racing**.

## 🚀 Overview

This project tackles the challenge of high-speed autonomous navigation by solving a constrained non-linear optimisation problem in real-time. By predicting future vehicle states and optimising control inputs over a receding horizon, the controller accounts for non-linear vehicle dynamics and tire slip, which are critical at the limits of handling.

## 🛠 Technical Stack

* **Language:** C++17/20
* **Optimization Solver:** [IPOPT](https://coin-or.github.io/Ipopt/) (Interior Point Optimizer)
* **Automatic Differentiation:** [CppAD](https://coin-or.github.io/CppAD/)
* **Middleware:** ROS 2 (Humble)
* **Linear Algebra:** Eigen 3

## ✨ Key Features

* **Non-Linear Vehicle Model:** Utilises a kinematic/dynamic bicycle model that incorporates non-linear constraints.
* **Algorithmic Differentiation:** Leverages **CppAD** to provide the solver with exact Jacobians and Hessians, significantly improving convergence speed and reliability compared to numerical finite-difference methods.
* **Constraints Handling:** Enforces hard physical constraints on steering angle, steering rate, acceleration, and track boundaries.
* **Real-Time Performance:** Optimized for low-latency execution on the **NVIDIA Jetson Orin**, enabling high-frequency control loops.
* **Multi-Threaded Architecture:** Designed as a thread-safe ROS 2 node to minimise blocking and maximise throughput in a distributed system.

## 📈 Performance Impact

* **Lap Time Reduction:** Successfully achieved a **50% improvement in lap times** compared to previous geometric (Pure Pursuit) controllers.
* **Precision:** Drastically reduced cross-track error at high velocities ($>15 m/s$) by anticipating cornering forces.

## 🏗 System Architecture

The solver is structured to separate the mathematical model from the ROS 2 communication layer, making it easy to unit test or port to different simulation environments:

1.  **`MPC_Solver.cpp`**: Core logic for interfacing with IPOPT and CppAD.
2.  **`VehicleModel.hpp`**: Defines the system dynamics equations ($x_{t+1} = f(x_t, u_t)$).
3.  **`MPC_Node.cpp`**: ROS 2 wrapper for sensor fusion input and control output.

## 🔧 Installation & Build

Note that the code here is only present to demonstrate my work and will not build on systems that do not have the right structure. Additionally, without the other nodes
in the pipeline, including Perception, Mapping and Path-Planning, the MPC node will not output any relevant messages. This implementation is tailored specifically for the vehicle presented
and will not work elsewhere without substantial modifications - which is by design.
