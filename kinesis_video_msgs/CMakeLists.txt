cmake_minimum_required(VERSION 3.5)
project(kinesis_video_msgs)

# Add support for C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(diagnostic_msgs REQUIRED)

set(INCLUDE_DIRS
  ${ament_cmake_INCLUDE_DIRS}
  ${rosidl_default_generators_INCLUDE_DIRS}
  ${diagnostic_msgs_INCLUDE_DIRS}
)
include_directories(${INCLUDE_DIRS})

## Generate messages in the 'msg' folder
## Generate added messages and services with any dependencies listed here
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/KinesisVideoFrame.msg"
  "msg/KinesisImageMetadata.msg"
  DEPENDENCIES diagnostic_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(${INCLUDE_DIRS})

ament_package()
