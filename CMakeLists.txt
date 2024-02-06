cmake_minimum_required(VERSION 3.5)
project(maxxii_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

add_library(error_codes include/${PROJECT_NAME}/error_codes.cpp)
add_library(RoboteqDevice include/${PROJECT_NAME}/RoboteqDevice.cpp)
add_library(driver_interface include/${PROJECT_NAME}/driver_interface.cpp)
add_library(motor include/${PROJECT_NAME}/motor.cpp)
add_library(encoder include/${PROJECT_NAME}/encoder.cpp)

add_executable(maxxii_node src/maxxii_node.cpp)

target_link_libraries(driver_interface error_codes)
target_link_libraries(driver_interface RoboteqDevice)
target_link_libraries(motor driver_interface)
target_link_libraries(encoder driver_interface)
target_link_libraries(maxxii_node 
  driver_interface
  motor
  encoder
  RoboteqDevice)

target_include_directories(maxxii_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


ament_target_dependencies(maxxii_node 
  rclcpp 
  geometry_msgs
  sensor_msgs 
  std_msgs)

install(TARGETS maxxii_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
