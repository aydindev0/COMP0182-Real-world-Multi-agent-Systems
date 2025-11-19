cmake_minimum_required(VERSION 3.5)
project(phasespace_msgs_2)


if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()


find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)


set(msg_files
  "msg/Camera.msg"
  "msg/Cameras.msg"
  "msg/Marker.msg"
  "msg/Markers.msg"
  "msg/Rigid.msg"
  "msg/Rigids.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)


install(
  DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}/msg
)


ament_package()

