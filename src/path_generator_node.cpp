#include "path_generator/resample_path.hpp"
#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto component = std::make_shared<path_generator::ResamplePath>(options);
  exec.add_node(component);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}