cmake_minimum_required(VERSION 3.8)
project(mpc)

# Default to C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE "Release")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -O3)
endif()

# Interfaces
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(driving_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(control_systems_interfaces REQUIRED)

# Dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(CPPAD REQUIRED cppad)
pkg_check_modules(IPOPT REQUIRED ipopt)
add_compile_definitions(HAVE_CSTDDEF)

if (NOT CPPAD_FOUND OR NOT IPOPT_FOUND)
  message(FATAL_ERROR "Missing CPPAD and/or IPOPT.
  An installation guide can be found here:
  https://redbackracing63.atlassian.net/wiki/spaces/t8000/pages/3284008962/MPC+-+Model+Predictive+Control")
endif()

include_directories(
  include
  ${CPPAD_INCLUDE_DIRS}
  ${IPOPT_INCLUDE_DIRS}
)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  std_msgs
  std_srvs
  geometry_msgs
  driving_interfaces
  visualization_msgs
  ackermann_msgs
  Eigen3
  tf2
  tf2_ros
  control_systems_interfaces
)

# MPC library
add_library(MPC_lib
  src/MPCNode.cpp
  src/MPC.cpp
  src/FG_eval.cpp
  src/predictor.cpp
)

# Add include directories for MPC and CPPAD
target_include_directories(MPC_lib PUBLIC include ${CPPAD_INCLUDE_DIRS} ${IPOPT_INCLUDE_DIRS})

# Link dependencies
ament_target_dependencies(MPC_lib ${dependencies})
target_link_libraries(MPC_lib ${CPPAD_LIBRARIES} ${IPOPT_LIBRARIES} yaml-cpp)

# MPC Executable
add_executable(MPC_executable src/main.cpp)
target_link_libraries(MPC_executable PRIVATE MPC_lib ${CPPAD_LIBRARIES} ${IPOPT_LIBRARIES})

install(TARGETS
  MPC_executable
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)  # This is usually commented out unless needed
  # ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(MPC_lib)
ament_export_dependencies(${dependencies})
ament_package()