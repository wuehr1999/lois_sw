cmake_minimum_required(VERSION 3.8)

project(lois_ecu)
set(TARGET ${PROJECT_NAME}_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

include_directories(
  include
)

add_executable(${TARGET}
  src/ecu.cpp
  src/main.cpp
)

target_compile_features(${TARGET} PRIVATE cxx_std_17)
ament_target_dependencies(${TARGET} rclcpp geometry_msgs nav_msgs std_msgs diagnostic_msgs sensor_msgs)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/RuntimeParameters.msg"
        DEPENDENCIES std_msgs
)

rosidl_target_interfaces(${TARGET} ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  ${TARGET}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
