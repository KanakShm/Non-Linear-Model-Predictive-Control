#include "MPCNode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/rate.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  std::shared_ptr<MPCNode> mpc_node = std::make_shared<MPCNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(mpc_node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
