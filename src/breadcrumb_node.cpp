#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <breadcrumb/planner.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Breadcrumb::Planner>());
  rclcpp::shutdown();
  return 0;
}