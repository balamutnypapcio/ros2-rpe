cmake_minimum_required(VERSION 3.8)
project(rob_mon_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/NotifyError.srv"
  "srv/RestartSensor.srv"
)

ament_package()