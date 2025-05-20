cmake_minimum_required(VERSION 3.5)
project(trac_ik_kinematics_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
  set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(class_loader REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(trac_ik_lib REQUIRED)

# Generate parameter library
generate_parameter_library(${PROJECT_NAME}_parameters src/trac_ik_kinematics_plugin_parameters.yaml)

# Build
add_library(${PROJECT_NAME} SHARED src/trac_ik_kinematics_plugin.cpp)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${PROJECT_NAME}_parameters
    class_loader::class_loader
    moveit_core::moveit_kinematics_base
    moveit_core::moveit_robot_model
    moveit_core::moveit_robot_state
    rclcpp::rclcpp
    tf2_kdl::tf2_kdl
    trac_ik_lib::trac_ik
    urdf::urdf)

# Install
install(
  DIRECTORY include/
  DESTINATION include)
install(
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

# Causes the visibility macros to use dllexport rather than dllimport, which is
# appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME}
                           PRIVATE "TRAC_IK_KINEMATICS_PLUGIN_BUILDING_DLL")

# Pluginlib export
pluginlib_export_plugin_description_file(moveit_core
                                         trac_ik_kinematics_description.xml)

# Ament export
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  class_loader
  generate_parameter_library
  moveit_core
  pluginlib
  rclcpp
  tf2_kdl
  trac_ik_lib
  urdf)

# Package
ament_package()
