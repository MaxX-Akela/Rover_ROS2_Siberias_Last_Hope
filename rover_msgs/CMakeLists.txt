cmake_minimum_required(VERSION 3.8)
project(rover_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/State.msg"
  "srv/Navigate.srv"
  "srv/SetVelocity.srv"
  "srv/GetTelemetry.srv"
  "srv/ArmDisarm.srv"
  "srv/Stop.srv"
  "srv/SetArmPose.srv"
  "srv/SetLEDEffect.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
