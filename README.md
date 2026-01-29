# Non-Linear Model Predictive Control (NMPC) for Autonomous Racing

![C++](https://img.shields.io/badge/c++-%2300599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white)
![ROS2](https://img.shields.io/badge/ros2-%2322314E.svg?style=for-the-badge&logo=ros&logoColor=white)
![Ubuntu](https://img.shields.io/badge/Ubuntu-E9433F?style=for-the-badge&logo=ubuntu&logoColor=white)
![NVIDIA](https://img.shields.io/badge/NVIDIA-%2376B900.svg?style=for-the-badge&logo=nvidia&logoColor=white)

A high-performance C++ implementation of a Non-Linear Model Predictive Controller designed for a Formula Student autonomous race car. This repository contains the core optimisation logic, vehicle modelling, and ROS 2 integration used by **UNSW Redback Racing**.

## 🚀 Overview

This project tackles the challenge of high-speed autonomous navigation by solving a constrained non-linear optimisation problem in real-time. By predicting future vehicle states and optimising control inputs over a receding horizon, the controller accounts for non-linear vehicle dynamics and tire slip, which are critical at the limits of handling.

![RB25](https://github.com/user-attachments/assets/9f908056-de76-4f4b-ac26-7a2e891ce93a)

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
* **Real-Time Performance:** Optimised for low-latency execution on the **NVIDIA Jetson Orin**, enabling high-frequency control loops.
* **Multi-Threaded Architecture:** Designed as a thread-safe ROS 2 node to minimise blocking and maximise throughput in a distributed system.

## 🏎️ Results & Real-Time Performance

The primary objective was to transition from a conservative geometric controller to a high-performance **NMPC** capable of navigating complex track layouts at the vehicle's limits.

| NMPC in Action | Autonomous Driving Visuals |
| :---: | :---: |
| <video src="https://github.com/user-attachments/assets/be1207db-87cc-45f5-b1d3-032caa8e3bf2" width="300px"></video> | <video src="https://github.com/user-attachments/assets/7aedd5e7-7608-482e-9575-449b361527c3" width="500px"></video> |

### ⚡ Optimized for the Edge (<10ms Latency)
To ensure safety at high speeds, the control loop required ultra-low latency. By optimizing the **IPOPT** interior-point solver and leveraging **CppAD** for exact derivatives, the system consistently achieves solve times of **<10ms** on the **NVIDIA Jetson Orin**.

* **Algorithmic Differentiation:** Used **CppAD** to provide the solver with exact analytical Jacobians and Hessians. This eliminated the overhead and inaccuracy of numerical finite-difference methods, leading to faster convergence in fewer iterations.
* **Warm-Starting:** Implemented a warm-starting strategy where the solver initialized with the solution from the previous time step, reducing the number of iterations required to find the optimal trajectory.
* **CPU Optimization:** Achieved a total IPOPT CPU time of **~6ms** per solve, with NLP function evaluations taking only **~1ms**, ensuring the vehicle can react to dynamic changes in real-time.

### 🛡️ Safety & Reliability
The NMPC doesn't just drive fast; it drives **reliably**:
* **Constraint Satisfaction:** The solver explicitly respects physical limits, such as maximum steering rates and tire friction circles, preventing "tank-slappers" or loss of traction.
* **Predictive Collision Avoidance:** By predicting a 2-3 second horizon, the controller can begin braking or adjusting lines well before reaching a sharp turn, ensuring the car stays within track boundaries even at **15m/s+**.

### 📊 Comparative Analysis
| Metric | Stanley (Previous) | Non-Linear MPC (Current) | Improvement |
| :--- | :--- | :--- | :--- |
| **Avg. Lap Time** | 120s | 60s | **50% Faster** |
| **Max Velocity** | 4 m/s | 8 m/s | **100% Increase** |
| **Cross-Track Error** | ~0.5m | <0.1m | **80% More Precise** |

## 🔧 Installation & Build

Note that the code here is only present to demonstrate my work and will not build on systems that do not have the right structure. Additionally, without the other nodes
in the pipeline, including Perception, Mapping and Path-Planning, the MPC node will not output any relevant messages. This implementation is tailored specifically for the vehicle presented
and will not work elsewhere without substantial modifications - which is by design.
