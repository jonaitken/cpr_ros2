cmake_minimum_required(VERSION 3.5)
project(cpr_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rviz2 REQUIRED)
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
#find_package(cpr_ros2 REQUIRED)
find_package(pluginlib REQUIRED)

find_package(rviz_common REQUIRED)
find_package(rviz_common REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

include_directories( PUBLIC
  include
)


## Generate added messages and services with any dependencies listed here
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotState.msg"
  "msg/ChannelStates.msg"
  "srv/GetJointInfo.srv"
  "srv/GetRobotInfo.srv"
  "srv/RobotCommand.srv"
  DEPENDENCIES std_msgs sensor_msgs control_msgs
)
# Include directories for generated message and service headers
include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}/include  # Where generated headers are placed
  ${rclcpp_INCLUDE_DIRS}               # Include rclcpp headers (if needed)
  ${std_msgs_INCLUDE_DIRS}             # Include std_msgs headers (if needed)
)


add_executable(CPRMover6 
  src/main_CPRMover6.cpp 
  src/Bus.cpp 
  src/MotorModule.cpp 
  src/Robot.cpp 
  src/Joint.cpp 
  src/Joint.cpp 
  src/CPRMover6.cpp 
)


rosidl_target_interfaces(CPRMover6 ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_target_dependencies(CPRMover6 rclcpp std_msgs sensor_msgs control_msgs)

add_library(RobotPanel SHARED
  src/RobotPanel.cpp 
  src/JointControl.cpp
  src/TaggedButton.cpp 
)

rosidl_target_interfaces(RobotPanel ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(RobotPanel Qt5::Core Qt5::Widgets ${rviz2_LIBRARIES})
ament_target_dependencies(RobotPanel rclcpp std_msgs sensor_msgs control_msgs rviz_common pluginlib)


# Install include files
install(DIRECTORY include/
  DESTINATION include/
)

install( DIRECTORY robots/CPRMover6 
DESTINATION share/${PROJECT_NAME}/urdf)



install( PROGRAMS launch/CPRMover6.py
DESTINATION share/${PROJECT_NAME}/launch
)


install(TARGETS CPRMover6 DESTINATION  lib/${PROJECT_NAME})
install(TARGETS RobotPanel DESTINATION  bin/${PROJECT_NAME}/lib)
pluginlib_export_plugin_description_file(rviz_common RobotPanel.xml)
install(FILES robots/CPRMover6.urdf.xacro DESTINATION share/${PROJECT_NAME}/urdf)
install(FILES robots/CPRMover6.rviz DESTINATION share/${PROJECT_NAME}/urdf)
install(FILES RobotPanel.xml DESTINATION share/${PROJECT_NAME})
#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
#  ament_lint_auto_find_test_dependencies()
#endif()

ament_package()
