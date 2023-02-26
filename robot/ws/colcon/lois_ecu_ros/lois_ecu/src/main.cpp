#include <lois_ecu/ecu.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ECU>());
  rclcpp::shutdown();
  return 0;
}
