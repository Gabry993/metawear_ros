cmake_minimum_required(VERSION 3.5)
project(metawear_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)

set(msg_files
  "msg/CalibrationState.msg"
	"msg/Vibration.msg"
  "msg/VibrationPattern.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}_msgs
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# Install Python modules
# TODO(Jerome): installation broken beacuse of name conflict with the package above
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR ${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/metawear_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
