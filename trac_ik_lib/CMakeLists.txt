cmake_minimum_required(VERSION 3.5)
project(trac_ik_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
  set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(NLopt REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)

# Build
add_library(trac_ik SHARED
  src/kdl_tl.cpp
  src/nlopt_ik.cpp
  src/trac_ik.cpp)
target_include_directories(trac_ik
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(trac_ik
  PUBLIC
    ${geometry_msgs_TARGETS}
    Eigen3::Eigen
    kdl_parser::kdl_parser
    rclcpp::rclcpp
    urdf::urdf
    ${NLOPT_LIBRARIES})

# Install
install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS trac_ik
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

# Ament export
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp geometry_msgs urdf kdl_parser NLopt)

# Package
ament_package()
