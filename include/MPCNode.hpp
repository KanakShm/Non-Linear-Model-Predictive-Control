#ifndef MPCNODE_HPP
#define MPCNODE_HPP

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <tuple>
#include <tf2/LinearMath/Quaternion.h>
#include <eigen3/Eigen/Dense>

#include "rclcpp_lifecycle/lifecycle_node_impl.hpp"
#include "geometry_msgs/msg/polygon.hpp"

#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/bool.hpp"

#include "driving_interfaces/msg/wheel_speed.hpp"
#include "control_systems_interfaces/msg/debug.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <std_msgs/msg/u_int16.hpp>

#include "mpc/MPC.hpp"
#include "utils.hpp"

typedef rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn CallbackReturn;

class MPCNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  MPCNode();
  ~MPCNode();

  // ROS2 Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr waypointsSub;
  rclcpp::Subscription<driving_interfaces::msg::WheelSpeed>::SharedPtr wheelSpeedsSub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr missionCompleteSub;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr lapCountSub;


  // ROS2 Publishers
  rclcpp_lifecycle::LifecyclePublisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
    cmdPub;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    predictedTrajectoryPub;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    MPCVelocityPub;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>::SharedPtr
    torqueRequestPub;
  rclcpp_lifecycle::LifecyclePublisher<control_systems_interfaces::msg::Debug>::SharedPtr
    debugMsgPub;

  // Lifecycle States
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state);
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state);
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state);
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);

private:
  // ROS interfaces
  void loadParameters();
  void establishPubsAndSubs();
  void waypointsCallback(const geometry_msgs::msg::Polygon::SharedPtr msg);
  void wheelSpeedsCallback(const driving_interfaces::msg::WheelSpeed::SharedPtr msg);
  void lapCountCallback(const std_msgs::msg::UInt16 msg);
  void checkWaypoints();
  void stopRequestCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  // Geometry helpers
  auto getProjectedPoint(const Point32 & p1, const Point32 & p2);
  std::tuple<double, Point32, Point32, double> processBezierPoints();
  std::vector<Point32> filterPoints(std::vector<Point32> points);

  void runMPC();

  // Publishers
  void publishDriveMsg(const double delta, const double acceleration) const;
  void publishTorqueMsg(double acceleration) const;
  void publishDebugMsg(const MPC::MPCPrediction) const;
  void publishPredictedTrajectory(
    const std::vector<double> & x_pred,
    const std::vector<double> & y_pred,
    const std::vector<double> & v_pred) const;
  void publishVelocityVector(const double velocity, const double steering) const;

  // Subscribers
  std::string waypoints_topic_;
  std::string wheel_speeds_topic_;
  std::string lap_count_topic_;
  std::string predicted_trajectory_topic_;
  std::string cmd_topic_;
  std::string MPC_velocity_vector_topic_;
  std::string torque_request_topic_;
  std::string control_systems_debug_topic_;

  // ROS Servers
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr stopRequestServer_;

  // Parameters
  int waypoint_timeout_;
  int qos_;
  double car_length_;
  double dt_prop_;
  int period_;
  std::string mission_;
  double control_delay_;
  double waypoints_offset_;

  // Vehicle's cooridinates and heading relative to its own coordinate axis.
  const double x_curr = 0.0;
  const double y_curr = 0.0;
  const double psi_curr = 0.0;

  double v_curr = 0.0;

  // MPC
  rclcpp::TimerBase::SharedPtr mpc_timer_;
  std::vector<Point32> waypoints;
  double horizon_dist_;
  double step_size_;
  Eigen::VectorXd vehicle_state;    // x, y, psi, v, cte, epsi, prev_steer_angle, prev_throttle
  std::unique_ptr<MPC> mpc;

  // Vehicle's projected states
  double x1 = 0.0;
  double y1 = 0.0;
  double psi1 = 0.0;
  double v1 = 0.0;
  double cte1 = 0.0;
  double epsi1 = 0.0;

  // steering angle and throttle
  double prev_delta = 0;
  double prev_a = 0;

  // Exponential moving average filter
  static constexpr double alpha = 0.1;
  double avg_latency;
  double latency;

  //RTF adjust for clock in sim (defaults for RTF and lasts to not brick stuff)
  double RTF = 1;
  double last_time_real_ = 0;
  double last_time_sim_ = 0;
  double curr_time_real_;
  double curr_time_sim_;
  double total_CTE = 0.0;
  double total_epsi = 0.0;
  double global_start = 0.0;
  double current_time = 0.0;


  rclcpp::TimerBase::SharedPtr waypoint_timer_;
  rclcpp::Time last_waypoint_time;
  int lap_count_ = 0;
};

#endif  // MPC__MPC_HPP_
