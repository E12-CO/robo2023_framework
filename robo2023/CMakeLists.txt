cmake_minimum_required(VERSION 3.8)
project(robo2023)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(robo2023_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclpy REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_python_install_package(${PROJECT_NAME})

# add_executable(main_node src/main_node.py)
# ament_target_dependencies(main_node 
#   "rclpy"
#   "geometry_msgs"
#   "robo2023_interfaces"
# )

# add_executable(navigation_node src/navigation_node.py)
# ament_target_dependencies(navigation_node 
#   "rclpy"
#   "geometry_msgs"
#   "robo2023_interfaces"
# )

# add_executable(imgproc_node src/imgproc_node.py)
# ament_target_dependencies(imgproc_node 
#   "rclpy"
#   "robo2023_interfaces"
# )

# add_executable(servo_node src/servo_node.py)
# ament_target_dependencies(servo_node 
#   "rclpy"
#   "robo2023_interfaces"
# )



install(
  DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/main_node.py scripts/imgproc_node.py scripts/navigation_node.py scripts/servo_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
