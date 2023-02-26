#include <pls101/sick.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Sick>());
  rclcpp::shutdown();
  return 0;
}
