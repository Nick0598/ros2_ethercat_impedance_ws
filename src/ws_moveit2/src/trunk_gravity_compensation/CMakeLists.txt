cmake_minimum_required(VERSION 3.8)
project(trunk_gravity_compensation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}_lib
  src/gravity_model.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  kdl_parser
)
target_link_libraries(${PROJECT_NAME}_lib
  orocos-kdl
)
target_compile_features(${PROJECT_NAME}_lib PUBLIC cxx_std_17)

add_executable(gravity_compensation_node
  src/gravity_compensation_node.cpp
)

ament_target_dependencies(gravity_compensation_node
  ament_index_cpp
  rclcpp
  sensor_msgs
)
target_link_libraries(gravity_compensation_node
  ${PROJECT_NAME}_lib
  orocos-kdl
)
target_compile_features(gravity_compensation_node PUBLIC cxx_std_17)

install(TARGETS
  ${PROJECT_NAME}_lib
  gravity_compensation_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
