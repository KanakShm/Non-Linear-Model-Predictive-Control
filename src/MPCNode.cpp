#include "MPCNode.hpp"

MPCNode::MPCNode()
: rclcpp_lifecycle::LifecycleNode("MPCNode"),
  vehicle_state(Eigen::VectorXd::Zero(8))
{
  /*
    "Wouldn't you like to know"
  */
  RCLCPP_INFO(this->get_logger(), "MPC is Running... Now bring me that horizon");
}

MPCNode::~MPCNode()
{
  RCLCPP_INFO(this->get_logger(), "MPC Shutting Down");
}

void MPCNode::loadParameters()
{

  // Subscribers
  this->declare_parameter<std::string>("subscriptions.waypoints_topic_", "/waypoints");
  waypoints_topic_ = this->get_parameter("subscriptions.waypoints_topic_").as_string();
  RCLCPP_INFO(this->get_logger(), "Subscribed to topic: %s", waypoints_topic_.c_str());

  this->declare_parameter<std::string>("subscriptions.wheel_speeds_topic_", "/wheel_speeds");
  wheel_speeds_topic_ = this->get_parameter("subscriptions.wheel_speeds_topic_").as_string();
  RCLCPP_INFO(this->get_logger(), "Subscribed to topic: %s", wheel_speeds_topic_.c_str());

  this->declare_parameter<std::string>("subscriptions.lap_count_topic_", "/lap_count");
  lap_count_topic_ = this->get_parameter("subscriptions.lap_count_topic_").as_string();
  RCLCPP_INFO(this->get_logger(), "Subscribed to topic: %s", lap_count_topic_.c_str());

  // Publishers
  this->declare_parameter<std::string>(
    "publishers.predicted_trajectory_topic_",
    "/predicted_trajectory");
  predicted_trajectory_topic_ =
    this->get_parameter("publishers.predicted_trajectory_topic_").as_string();
  RCLCPP_INFO(
    this->get_logger(), "Publishing to predicted_trajectory_topic_: %s",
    predicted_trajectory_topic_.c_str());

  this->declare_parameter<std::string>("publishers.MPC_velocity_vector_topic_", "/velocity_vector");
  MPC_velocity_vector_topic_ =
    this->get_parameter("publishers.MPC_velocity_vector_topic_").as_string();
  RCLCPP_INFO(
    this->get_logger(), "Publishing to MPC_velocity_vector_topic_: %s",
    MPC_velocity_vector_topic_.c_str());

  this->declare_parameter<std::string>("publishers.cmd_topic_", "/cmd");
  cmd_topic_ = this->get_parameter("publishers.cmd_topic_").as_string();
  RCLCPP_INFO(this->get_logger(), "Publishing to cmd_ topic: %s", cmd_topic_.c_str());

  this->declare_parameter<std::string>("publishers.torque_request_topic_", "/torque_request");
  torque_request_topic_ = this->get_parameter("publishers.torque_request_topic_").as_string();
  RCLCPP_INFO(
    this->get_logger(), "Publishing to torque_request_topic_: %s", torque_request_topic_.c_str());

  this->declare_parameter<std::string>(
    "publishers.control_systems_debug_topic_",
    "/control_systems_debug");
  control_systems_debug_topic_ =
    this->get_parameter("publishers.control_systems_debug_topic_").as_string();
  RCLCPP_INFO(
    this->get_logger(),
    "Publishing to control_systems_debug_topic_ topic: %s", control_systems_debug_topic_.c_str());

  this->declare_parameter<int>("publishers.qos_", 10);
  qos_ = this->get_parameter("publishers.qos_").as_int();
  RCLCPP_INFO(this->get_logger(), "Loaded qos_: %d", qos_);

  // Universal
  this->declare_parameter<double>("universal.car_length_", 1.7);
  car_length_ = this->get_parameter("universal.car_length_").as_double();
  RCLCPP_INFO(this->get_logger(), "Loaded car_length_: %.2f", car_length_);

  // Misc
  this->declare_parameter<int>("misc.waypoint_timeout_", 1000);
  waypoint_timeout_ = this->get_parameter("misc.waypoint_timeout_").as_int();
  RCLCPP_INFO(this->get_logger(), "Loaded waypoint_timeout_: %d", waypoint_timeout_);

  this->declare_parameter<int>("misc.period_", 40);
  period_ = this->get_parameter("misc.period_").as_int();
  RCLCPP_INFO(this->get_logger(), "Loaded period_: %d", period_);

  this->declare_parameter<double>("misc.dt_prop_", 0.15);
  dt_prop_ = this->get_parameter("misc.dt_prop_").as_double();
  RCLCPP_INFO(this->get_logger(), "Loaded dt_prop_: %.2f", dt_prop_);

  this->declare_parameter<double>("misc.horizon_dist_", 20);
  horizon_dist_ = this->get_parameter("misc.horizon_dist_").as_double();
  RCLCPP_INFO(this->get_logger(), "Loaded horizon_dist: %.2f", horizon_dist_);

  this->declare_parameter<double>("misc.step_size_", 8);
  step_size_ = this->get_parameter("misc.step_size_").as_double();
  RCLCPP_INFO(this->get_logger(), "Loaded step size: %.2f", step_size_);

  this->declare_parameter<double>("misc.control_delay_", 0.0);
  control_delay_ = this->get_parameter("misc.control_delay_").as_double();
  RCLCPP_INFO(this->get_logger(), "Loaded control_delay: %.2f", control_delay_);

  this->declare_parameter<double>("misc.waypoints_offset", 0.0);
  waypoints_offset_ = this->get_parameter("misc.waypoints_offset").as_double();
  RCLCPP_INFO(this->get_logger(), "Loaded waypoints offset: %.2f", waypoints_offset_);

  // Load in mission specific yaml files
  this->declare_parameter<std::string>("mission", "trackdrive");
  mission_ = this->get_parameter("mission").as_string();
  RCLCPP_INFO(this->get_logger(), "Launching %s mission", mission_.c_str());
  std::string yaml_path = ament_index_cpp::get_package_share_directory("mpc");
  yaml_path += "/config/" + mission_ + ".yaml";
  mpc = std::make_unique<MPC>(yaml_path);
}

void MPCNode::establishPubsAndSubs()
{
  // Subscribing to
  waypointsSub = this->create_subscription<geometry_msgs::msg::Polygon>(
    waypoints_topic_,
    qos_,
    std::bind(&MPCNode::waypointsCallback, this, std::placeholders::_1)
  );

  wheelSpeedsSub = this->create_subscription<driving_interfaces::msg::WheelSpeed>(
    wheel_speeds_topic_,
    qos_,
    std::bind(&MPCNode::wheelSpeedsCallback, this, std::placeholders::_1)
  );

  lapCountSub = this->create_subscription<std_msgs::msg::UInt16>(
    lap_count_topic_,
    qos_,
    std::bind(&MPCNode::lapCountCallback, this, std::placeholders::_1)
  );

  // Publishing to
  cmdPub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
    cmd_topic_,
    qos_
  );

  torqueRequestPub = this->create_publisher<std_msgs::msg::Float32>(
    torque_request_topic_,
    qos_
  );

  predictedTrajectoryPub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    predicted_trajectory_topic_,
    qos_
  );

  MPCVelocityPub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    MPC_velocity_vector_topic_,
    qos_
  );

  debugMsgPub = this->create_publisher<control_systems_interfaces::msg::Debug>(
    control_systems_debug_topic_,
    qos_
  );

  stopRequestServer_ = this->create_service<std_srvs::srv::SetBool>(
    "/stop_request",
    std::bind(&MPCNode::stopRequestCallback, this, std::placeholders::_1, std::placeholders::_2));
}

void MPCNode::stopRequestCallback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  (void) request;
  auto output_msg = ackermann_msgs::msg::AckermannDriveStamped();
  output_msg.header.stamp = this->now();
  output_msg.drive.steering_angle = 0.0;
  output_msg.drive.acceleration = 0.0;
  cmdPub->publish(output_msg);
  response->success = true;
}

// Project x,y coordinates onto p1 and p1
auto MPCNode::getProjectedPoint(const Point32 & p1, const Point32 & p2)
{
  Eigen::Vector2d v(p2.x - p1.x, p2.y - p1.y);
  Eigen::Vector2d w(x1 - p1.x, y1 - p1.y);

  double t = w.dot(v) / v.dot(v);
  t = std::max(0.0, std::min(1.0, t));

  Point32 projected_point = Point32();
  projected_point.x = p1.x + t * v.x();
  projected_point.y = p1.y + t * v.y();
  projected_point.z = 0.0;

  return projected_point;
}

// Caclulate the cross-track error, projected point on the curve and psi.
std::tuple<double, Point32, Point32, double> MPCNode::processBezierPoints()
{
  // This shits gonna crash idk what to do here
  if (waypoints.empty()) {
    RCLCPP_WARN(this->get_logger(), "points vector is empty");
  }

  // Get the point that the car (x1,y1) projects onto the closest segment
  // of the bezier points.
  double min_dist = std::numeric_limits<double>::max();
  double dist = std::numeric_limits<double>::max();
  auto closest_point = waypoints[0];
  auto lookahead_point = waypoints[0];

  for (size_t i = 0; i < waypoints.size() - 1; i++) {
    const auto & p1 = waypoints[i];
    const auto & p2 = waypoints[i + 1];
    dist = (p1.x - x1) * (p1.x - x1) + (p1.y - y1) * (p1.y - y1);
    if (dist < min_dist) {
      min_dist = dist;
      closest_point.x = p1.x;
      closest_point.y = p1.y;
      lookahead_point.x = p2.x;
      lookahead_point.y = p2.y;
    }
  }

  Point32 projected_point = getProjectedPoint(closest_point, lookahead_point);

  // Compute psi angle. Clockwise is negative.
  double dx = lookahead_point.x - projected_point.x;
  double dy = lookahead_point.y - projected_point.y;
  double desired_heading = std::atan2(dy, dx);
  double epsi = desired_heading - psi1;

  // Compute CTE
  double dx_car = x1 - projected_point.x;
  double dy_car = y1 - projected_point.y;

  double dx_seg = lookahead_point.x - projected_point.x;
  double dy_seg = lookahead_point.y - projected_point.y;

  // Compute 2D cross product
  double cross_product = dx_seg * dy_car - dy_seg * dx_car;

  // Normalize to get proper signed distance
  double norm = std::sqrt(dx_seg * dx_seg + dy_seg * dy_seg);
  double signed_cte = cross_product / norm;

  return std::make_tuple(-signed_cte, projected_point, lookahead_point, epsi);
}

std::vector<Point32> MPCNode::filterPoints(const std::vector<Point32> points)
{
  if (points.size() < 2) {
    return points;
  }

  std::vector<Point32> filtered_points;

  Point32 first_point;
  first_point.x = points[0].x - waypoints_offset_;
  first_point.y = points[0].y;
  filtered_points.push_back(first_point);

  double cumulative_dist = 0.0;
  Point32 last_point = points[0];

  for (size_t i = 1; i < points.size(); ++i) {
    const auto & curr = points[i];
    double dx = curr.x - last_point.x;
    double dy = curr.y - last_point.y;

    double dist = std::sqrt(dx * dx + dy * dy);
    cumulative_dist += dist;

    if (cumulative_dist > horizon_dist_) {
      break;
    }

    Point32 new_point;
    new_point.x = curr.x - waypoints_offset_;
    new_point.y = curr.y;
    filtered_points.push_back(new_point);

    last_point = curr;
  }

  std::vector<Point32> downsampled;
  for (size_t i = 0; i < filtered_points.size(); i += step_size_) {
    downsampled.push_back(filtered_points[i]);
  }

  return downsampled;
}

void MPCNode::checkWaypoints()
{
  double last_time = last_waypoint_time.seconds();
  double curr_time = this->now().seconds();

  if (last_time == 0) {
    static bool warned_once = false;
    if (!warned_once) {
      RCLCPP_WARN(this->get_logger(), "...I said bring me that horizon");
      warned_once = true;
    }
    return;
  }

  double time_diff = curr_time - last_time;
  if (time_diff > static_cast<double>(waypoint_timeout_ / 1000)) {
    // Publish to /cmd
    auto stop_msg = ackermann_msgs::msg::AckermannDriveStamped();
    stop_msg.header.stamp = this->now();
    stop_msg.drive.steering_angle = 0;
    stop_msg.drive.acceleration = static_cast<_Float32>(0);
    cmdPub->publish(stop_msg);

    // Publish to /torque_request
    std_msgs::msg::Float32 torque_msg;
    double torque_at_motor = 0;
    torque_msg.data = torque_at_motor;
    torqueRequestPub->publish(torque_msg);
  }
}

// Publish Ackermann control message
void MPCNode::publishDriveMsg(const double delta, const double acceleration) const
{
  ackermann_msgs::msg::AckermannDriveStamped drive_msg;
  drive_msg.header.stamp = this->now();
  drive_msg.drive.steering_angle = delta * 180 / M_PI;
  drive_msg.drive.acceleration = static_cast<float>(acceleration);
  cmdPub->publish(drive_msg);
}


// Publish torque command
void MPCNode::publishTorqueMsg(double acceleration) const
{
  static constexpr double wheel_radius = 0.2032;
  static constexpr double gear_ratio = 13.68512111;
  static constexpr double mass = 272.0;

  const double torque = mass * acceleration * wheel_radius;
  const double torque_at_motor = torque / gear_ratio;

  std_msgs::msg::Float32 torque_msg;
  torque_msg.data = static_cast<float>(torque_at_motor);
  torqueRequestPub->publish(torque_msg);
}

// Build and publish debug message
void MPCNode::publishDebugMsg(const MPC::MPCPrediction vars) const
{
  control_systems_interfaces::msg::Debug debug_msg;
  debug_msg.latency = latency;

  const size_t N = std::min<size_t>(vars.delta_pred.size(), 10);
  debug_msg.steering.reserve(N);
  debug_msg.acceleration.reserve(N);
  debug_msg.cte.reserve(N);
  debug_msg.epsi.reserve(N);
  debug_msg.cost_terms = vars.cost_terms;

  for (size_t i = 0; i < N; ++i) {
    debug_msg.steering.push_back(vars.delta_pred[i]);
    debug_msg.acceleration.push_back(vars.a_pred[i]);
    debug_msg.cte.push_back(vars.cte_pred[i]);
    debug_msg.epsi.push_back(vars.epsi_pred[i]);
  }

  debugMsgPub->publish(debug_msg);
}

void MPCNode::runMPC()
{
  if (waypoints.empty()) {return;}

  const auto start_time = rclcpp::Clock().now();

  // Process reference trajectory
  const auto [CTE, projected_point, lookahead_point, epsi] = processBezierPoints();
  const auto [x0, y0] = std::tie(projected_point.x, projected_point.y);

  avg_latency = (alpha * latency) + (1.0 - alpha) * avg_latency;
  dt_prop_ = avg_latency + control_delay_;

  // Predict next state considering actuation latency
  const double x1 = x_curr + v_curr * std::cos(psi_curr) * dt_prop_;
  const double y1 = y_curr + v_curr * std::sin(psi_curr) * dt_prop_;
  const double psi1 = psi_curr + v_curr * std::tan(prev_delta) / car_length_ * dt_prop_;
  const double v1 = v_curr + prev_a * dt_prop_;

  vehicle_state << x1, y1, psi1, v1, CTE, epsi, prev_delta, prev_a;
  if (vehicle_state.array().isNaN().any()) {return;}

  auto to_deg = [](double rad) {return rad * 180.0 / M_PI;};

  RCLCPP_DEBUG(
    this->get_logger(),
    "CTE: %.3f, x0: %.2f, y0: %.2f, epsi: %.2f, v_curr: %.2f, latency: %.4f, dt_prop: %.4f, RTF: %.2f, delta_out: %.4f, a_out: %.4f",
    CTE, x0, y0, to_deg(vehicle_state(5)), vehicle_state(3), latency, dt_prop_, RTF,
    to_deg(prev_delta), prev_a
  );

  current_time = rclcpp::Clock().now().nanoseconds();

  // Solve MPC
  const MPC::MPCPrediction vars = mpc->solve(vehicle_state, waypoints, lap_count_);
  prev_delta = vars.delta_pred.front();
  prev_a = vars.a_pred.front();

  // Compute latency
  const rclcpp::Time end_time = rclcpp::Clock().now();
  latency = (end_time - start_time).seconds();

  publishDriveMsg(prev_delta, prev_a);
  publishTorqueMsg(prev_a);
  publishDebugMsg(vars);

  // Visualisers
  publishPredictedTrajectory(vars.x_pred, vars.y_pred, vars.v_pred);
  publishVelocityVector(vars.v_pred.front(), vars.delta_pred.front());
}


void MPCNode::waypointsCallback(const geometry_msgs::msg::Polygon::SharedPtr msg)
{
  if (msg->points.empty()) {RCLCPP_WARN(this->get_logger(), "Received no waypoints");}
  waypoints = filterPoints(msg->points);
  last_waypoint_time = this->now();
}

void MPCNode::lapCountCallback(const std_msgs::msg::UInt16 msg)
{
  lap_count_ = msg.data;
}


void MPCNode::wheelSpeedsCallback(const driving_interfaces::msg::WheelSpeed::SharedPtr msg)
{
  // Compare dt irl vs dt clock
  curr_time_real_ = rclcpp::Clock().now().nanoseconds() * 1e-9;
  curr_time_sim_ = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;

  RTF = (curr_time_sim_ - last_time_sim_ ) / (curr_time_real_ - last_time_real_);

  last_time_real_ = curr_time_real_;
  last_time_sim_ = curr_time_sim_;

  (msg->rl == 0) ? v_curr = msg->rr :
    (msg->rr == 0) ? v_curr = msg->rl :
    v_curr = (msg->rl + msg->rr) / 2;
}

void MPCNode::publishVelocityVector(double velocity, double steering) const
{
  visualization_msgs::msg::Marker velocity_marker;

  // Create the marker
  velocity_marker.header.frame_id = "base_footprint";
  velocity_marker.header.stamp = rclcpp::Clock().now();
  velocity_marker.ns = "MPC_velocity";
  velocity_marker.id = 1;
  velocity_marker.type = visualization_msgs::msg::Marker::ARROW;
  velocity_marker.action = visualization_msgs::msg::Marker::ADD;
  velocity_marker.lifetime = rclcpp::Duration(1, 0);

  // Orientation (convert steering degrees to radians)
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, steering * M_PI / 180.0);
  velocity_marker.pose.orientation.x = q.x();
  velocity_marker.pose.orientation.y = q.y();
  velocity_marker.pose.orientation.z = q.z();
  velocity_marker.pose.orientation.w = q.w();

  velocity_marker.scale.x = 0.2;        // shaft length
  velocity_marker.scale.y = 0.4;        // head size
  velocity_marker.scale.z = 0.2;        // thickness

  // Arrow points
  static constexpr double scale = 3.0;
  velocity_marker.points.resize(2);
  geometry_msgs::msg::Point p0;
  p0.x = 0.0;
  p0.y = 0.0;
  p0.z = 0.0;

  geometry_msgs::msg::Point p1;
  p1.x = scale * velocity * std::cos(steering);
  p1.y = scale * velocity * std::sin(steering);
  p1.z = 0.0;

  velocity_marker.points = {p0, p1};

  // Color (orange)
  velocity_marker.color.a = 1.0;
  velocity_marker.color.r = 1.0;
  velocity_marker.color.g = 0.5;
  velocity_marker.color.b = 0.0;

  // Wrap in MarkerArray and publish
  visualization_msgs::msg::MarkerArray markers;
  markers.markers.push_back(velocity_marker);
  MPCVelocityPub->publish(markers);
}

void MPCNode::publishPredictedTrajectory(
  const std::vector<double> & x_pred,
  const std::vector<double> & y_pred,
  const std::vector<double> & v_pred) const
{
  if (x_pred.size() != y_pred.size() || x_pred.size() != v_pred.size()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Predicted trajectory size mismatch: x_pred=%zu, y_pred=%zu, speeds=%zu",
      x_pred.size(), y_pred.size(), v_pred.size());
    return;
  }

  // Create the marker for predicted trajectory
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "base_footprint";
  marker.header.stamp = rclcpp::Clock().now();
  marker.ns = "predicted_trajectory";
  marker.id = 1;
  marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.lifetime = rclcpp::Duration(1, 0);

  // Orientation (none)
  marker.pose.orientation.w = 1.0;

  // Sphere scale (size of each predicted point)
  marker.scale.x = 0.25;
  marker.scale.y = 0.25;
  marker.scale.z = 0.25;

  // Reserve space
  marker.points.reserve(x_pred.size());
  marker.colors.reserve(x_pred.size());

  // Min and max speeds for color mapping
  static constexpr double min_speed = 0.0;      // m/s
  static constexpr double max_speed = 12.0;     // m/s

  // Avoid division by zero
  double range = max_speed - min_speed;
  if (range == 0.0) {range = 1.0;}

  for (size_t i = 0; i < x_pred.size(); ++i) {
    geometry_msgs::msg::Point p;
    p.x = x_pred[i];
    p.y = y_pred[i];
    p.z = 0.0;
    marker.points.push_back(p);

    // Normalise speed between 0 (slowest) and 1 (fastest)
    double norm_speed = (v_pred[i] - min_speed) / range;
    norm_speed = std::clamp(norm_speed, 0.0, 1.0);

    std_msgs::msg::ColorRGBA color;
    color.a = 1.0;
    // red (fast) = (1.0, 0.0, 0.0)
    // orange (mid) = (r=1.0, g=0.647, b=0.0)
    // green (slow) = (0.212, 1, 0.208)

    // set color based on normalised speed ratio
    if (norm_speed > 0.85) {
      // fast (>0.66) = red
      color.r = 1.0;
      color.g = 0.0;
      color.b = 0.0;
    } else if (norm_speed > 0.45) {
      // mid (0.33-0.66) = orange
      color.r = 1.0;
      color.g = 0.647;
      color.b = 0.0;
    } else {
      // slow (0.0-0.33) = green
      color.r = 0.212;
      color.g = 1.0;
      color.b = 0.208;
    }

    marker.colors.push_back(color);
  }

  // Wrap in MarkerArray and publish
  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.push_back(marker);
  predictedTrajectoryPub->publish(marker_array);
}


// Lifecycle states
CallbackReturn MPCNode::on_configure(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(this->get_logger(), "MPCNode configure called");

  // Declare parameters
  loadParameters();

  // Initialise subscription and publishers
  establishPubsAndSubs();

  return CallbackReturn::SUCCESS;
}

CallbackReturn MPCNode::on_activate(
  const rclcpp_lifecycle::State & state)
{
  LifecycleNode::on_activate(state);
  RCLCPP_INFO(this->get_logger(), "MPCNode activate called");

  global_start = rclcpp::Clock().now().nanoseconds();
  mpc_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(
      static_cast<int>(period_)), std::bind(&MPCNode::runMPC, this));
  waypoint_timer_ =
    this->create_wall_timer(
    std::chrono::milliseconds(
      static_cast<int>(waypoint_timeout_)), std::bind(&MPCNode::checkWaypoints, this));

  this->cmdPub->on_activate();
  this->torqueRequestPub->on_activate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn MPCNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  LifecycleNode::on_deactivate(state);
  RCLCPP_INFO(this->get_logger(), "MPCNode deactivate called");

  this->cmdPub->on_deactivate();
  this->torqueRequestPub->on_deactivate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn MPCNode::on_cleanup(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(this->get_logger(), "MPCNode cleanup called");

  // May not need if don't want to release resources here
  this->waypointsSub.reset();
  this->wheelSpeedsSub.reset();
  this->cmdPub.reset();
  this->torqueRequestPub.reset();


  return CallbackReturn::SUCCESS;
}

CallbackReturn MPCNode::on_shutdown(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  RCLCPP_INFO(this->get_logger(), "MPCNode shutting down");
  if (cmdPub) {
    this->cmdPub.reset();
  }
  if (torqueRequestPub) {
    this->torqueRequestPub.reset();
  }

  return CallbackReturn::SUCCESS;
}
