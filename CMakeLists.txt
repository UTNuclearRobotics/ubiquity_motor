cmake_minimum_required(VERSION 3.8)
project(ubiquity_motor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(CMAKE_CXX_STANDARD 17) # use C++17

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(generate_parameter_library REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread system)

generate_parameter_library(
  motor_parameters
  src/motor_parameters.yaml
)

include_directories(
  include
)

#######################################
## Declare ROS messages and services ##
#######################################

rosidl_generate_interfaces(
  ${PROJECT_NAME}_interfaces
    "msg/Leds.msg"
    "msg/MotorState.msg"
  DEPENDENCIES
    std_msgs
  LIBRARY_NAME
    ${PROJECT_NAME}
)
ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_interfaces rosidl_typesupport_cpp)

###########
## Build ##
###########


###########
# SERIAL LIBRARY
# Remove once serial is released into noetic
###########

if(APPLE)
	find_library(IOKIT_LIBRARY IOKit)
	find_library(FOUNDATION_LIBRARY Foundation)
endif()

## Sources
set(serial_SRCS
    serial/src/serial.cc
    serial/include/serial/serial.h
    serial/include/serial/v8stdint.h
)
if(APPLE)
    # If OSX
    list(APPEND serial_SRCS serial/src/impl/unix.cc)
    list(APPEND serial_SRCS serial/src/impl/list_ports/list_ports_osx.cc)
elseif(UNIX)
    # If unix
    list(APPEND serial_SRCS serial/src/impl/unix.cc)
    list(APPEND serial_SRCS serial/src/impl/list_ports/list_ports_linux.cc)
else()
    # If windows
    list(APPEND serial_SRCS serial/src/impl/win.cc)
    list(APPEND serial_SRCS serial/src/impl/list_ports/list_ports_win.cc)
endif()

## Add serial library
add_library(ur_serial ${serial_SRCS})
if(APPLE)
	target_link_libraries(ur_serial ${FOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
elseif(UNIX)
	target_link_libraries(ur_serial rt pthread)
else()
	target_link_libraries(ur_serial setupapi)
endif()

##H HACK to contain all serial stuff in here
list(APPEND catkin_LIBRARIES ur_serial)
list(APPEND catkin_INCLUDE_DIRS serial/include)

include_directories(serial/include)

## Mark executables and/or libraries for installation
install(TARGETS ur_serial
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

###############
# END SERIAL ##
###############

add_library(${PROJECT_NAME}
  src/motor_message.cc
  src/motor_serial.cc
  src/motor_hardware.cc
  src/motor_diagnostics.cc
  src/mcb_interface.cc
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  pluginlib
  sensor_msgs
  rclcpp_lifecycle
  diagnostic_updater
)
target_link_libraries(${PROJECT_NAME} motor_parameters ur_serial ${Boost_LIBRARIES} "${cpp_typesupport_target}")
pluginlib_export_plugin_description_file(hardware_interface ubiquity_motor.xml)

#############
## Install ##
#############

install(
  TARGETS
    ${PROJECT_NAME}
  DESTINATION
    lib
)

install(
  DIRECTORY
    include
  DESTINATION
    include/${PROJECT_NAME}
)

install(
  DIRECTORY
    cfg
  DESTINATION
    share/${PROJECT_NAME}
)

install(
  TARGETS
    ${PROJECT_NAME}
    motor_parameters
    ur_serial
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

#############
## Exports ##
#############

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  std_msgs
  pluginlib
  sensor_msgs
  rclcpp_lifecycle
  diagnostic_updater
)

ament_package()
