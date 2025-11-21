cmake_minimum_required(VERSION 3.10)
project(lukas_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(angles REQUIRED)
find_package(angles REQUIRED)


set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  angles
  sensor_msgs
  rclpy
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Compile 
add_library(
  lukas_control
  SHARED
  hardware/topic_based_system.cpp
)

#add_executable(observer src/state_observer_lqr.cpp)
#ament_target_dependencies(observer rclcpp std_msgs)

target_compile_features(lukas_control PUBLIC cxx_std_17)
target_include_directories(lukas_control PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/lukas_control>
)
ament_target_dependencies(
  lukas_control PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface lukas_control.xml)

install(TARGETS
 # observer
  DESTINATION lib/${PROJECT_NAME})

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_ros2_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  hardware
  description
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
