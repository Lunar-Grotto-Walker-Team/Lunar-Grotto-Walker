cmake_minimum_required(VERSION 3.8)
project(yoloe_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/YoloeDetection3D.msg"
  "msg/YoloeDetection3DArray.msg"
  "msg/ObservationRequest.msg"
  "msg/ObservationCommand.msg"
  "msg/ObservationResult.msg"
  "msg/UnifiedTrack.msg"
  "msg/UnifiedTrackArray.msg"
  "srv/MarkTrack.srv"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

