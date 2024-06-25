cmake_minimum_required(VERSION 3.8)
project(breadcrumb)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# find_package(Eigen3 REQUIRED)
# include(ConfigExtras.cmake)

## Generate interfaces
ament_auto_generate_code()

# ament_python_install_package(${PROJECT_NAME})

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME}_a_star
  src/${PROJECT_NAME}/AStar.cpp
)

ament_auto_add_library(${PROJECT_NAME}_planner
  src/${PROJECT_NAME}/planner.cpp
)
set_target_properties(${PROJECT_NAME}_planner PROPERTIES OUTPUT_NAME breadcrumb_planner PREFIX "")

ament_auto_add_executable(${PROJECT_NAME}_node
  src/breadcrumb_node.cpp
)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME breadcrumb PREFIX "")

#############
## Install ##
#############

# Install launch files
install(DIRECTORY
  launch
  # resources
  DESTINATION share/${PROJECT_NAME}/
)

file(GLOB ${PROJECT_NAME}_scripts ${PROJECT_SOURCE_DIR}/scripts/*)
install(PROGRAMS
  ${${PROJECT_NAME}_scripts}
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(CONFIG_EXTRAS) # ConfigExtras.cmake)


#############
## Testing ##
#############

# if (BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
#   find_package(ament_cmake_gtest REQUIRED)
#   ament_add_gtest(${PROJECT_NAME}-some-test src/test/test_something.cpp)
#   ament_target_dependencies(${PROJECT_NAME)-some-test
#     "rclcpp"
#     "std_msgs")
#   target_link_libraries(${PROJECT_NAME}-some-test
#     ${PROJECT_NAME}_some_dependency)
# endif()