cmake_minimum_required(VERSION 3.8)

project(pls101)
set(TARGET ${PROJECT_NAME}_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
)

add_executable(${TARGET}
  src/sick.cpp
  src/main.cpp
)

target_compile_features(${TARGET} PRIVATE cxx_std_17)
ament_target_dependencies(${TARGET} rclcpp std_msgs sensor_msgs)

install(TARGETS
  ${TARGET}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
