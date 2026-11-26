cmake_minimum_required(VERSION 3.5)
# Modified from upstream to install the covariance relay and exclude local backups.
project(articubot_one)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 主要内容：安装项目资源
install(
  DIRECTORY config description launch worlds maps rviz
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.yaml"
  PATTERN "*.xacro"
  PATTERN "*.launch.py"
  PATTERN "*_launch.py"
  PATTERN "*.sdf"
  PATTERN "*.pgm"
  PATTERN "*.rviz"
  PATTERN "ekf_[A-D].yaml" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)
# 安装python节点
install(PROGRAMS
  scripts/patrol_node.py
  scripts/frontier_explorer.py
  scripts/sensor_covariance_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
