cmake_minimum_required(VERSION 3.8)
project(led_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LEDState.msg"
  "msg/LEDStateArray.msg"
  "srv/SetLED.srv"
  "srv/SetLEDs.srv"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
