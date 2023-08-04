cmake_minimum_required(VERSION 3.8)
project(a1_sdk)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(a1_msg REQUIRED)

include_directories(
    include
)
file(GLOB_RECURSE SRC_LIST
    "src/*/*.cpp"
    "src/*/*.cc"
)

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

ament_package()

add_executable(robot_joint src/robot_joint.cpp ${SRC_LIST})
add_executable(robot_balance src/robot_balance.cpp ${SRC_LIST})
add_executable(robot_swingtest src/robot_swingtest.cpp ${SRC_LIST})
add_executable(robot_trotting src/robot_trotting.cpp ${SRC_LIST})

ament_target_dependencies(robot_joint rclcpp std_msgs sensor_msgs geometry_msgs a1_msg)
ament_target_dependencies(robot_balance rclcpp std_msgs sensor_msgs geometry_msgs a1_msg)
ament_target_dependencies(robot_trotting rclcpp std_msgs sensor_msgs geometry_msgs a1_msg)
ament_target_dependencies(robot_swingtest rclcpp std_msgs sensor_msgs geometry_msgs a1_msg)


install(TARGETS
  robot_joint
  robot_balance
  robot_swingtest
  robot_trotting
  DESTINATION lib/${PROJECT_NAME}
)
