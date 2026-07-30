cmake_minimum_required(VERSION 3.8)
project(my_swerve_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. FIND DEPENDENCIES
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)         # <--- ADDED THIS
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# 2. BUILD THE EXECUTABLE
add_executable(swerve_optimizer src/swerve_optimizer.cpp)

# 3. LINK DEPENDENCIES
# Use ament_target_dependencies for all ROS2 message types and rclcpp
ament_target_dependencies(swerve_optimizer 
  rclcpp 
  std_msgs              # <--- ADDED THIS
  sensor_msgs 
  trajectory_msgs 
  geometry_msgs
)

# Use target_link_libraries for non-Ament packages like Eigen
target_link_libraries(swerve_optimizer Eigen3::Eigen)

# 4. INSTALL
install(TARGETS
  swerve_optimizer
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()