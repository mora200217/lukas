# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/Users/amoralesma/Documents/UN/servos/servos-proyecto/uros_ws/src/uros/micro-ROS-Agent/micro_ros_agent")
  file(MAKE_DIRECTORY "/Users/amoralesma/Documents/UN/servos/servos-proyecto/uros_ws/src/uros/micro-ROS-Agent/micro_ros_agent")
endif()
file(MAKE_DIRECTORY
  "/Users/amoralesma/Documents/UN/servos/servos-proyecto/uros_ws/build/micro_ros_agent"
  "/Users/amoralesma/Documents/UN/servos/servos-proyecto/uros_ws/build/micro_ros_agent/micro_ros_agent-prefix"
  "/Users/amoralesma/Documents/UN/servos/servos-proyecto/uros_ws/build/micro_ros_agent/micro_ros_agent-prefix/tmp"
  "/Users/amoralesma/Documents/UN/servos/servos-proyecto/uros_ws/build/micro_ros_agent/micro_ros_agent-prefix/src/micro_ros_agent-stamp"
  "/Users/amoralesma/Documents/UN/servos/servos-proyecto/uros_ws/build/micro_ros_agent/micro_ros_agent-prefix/src"
  "/Users/amoralesma/Documents/UN/servos/servos-proyecto/uros_ws/build/micro_ros_agent/micro_ros_agent-prefix/src/micro_ros_agent-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/amoralesma/Documents/UN/servos/servos-proyecto/uros_ws/build/micro_ros_agent/micro_ros_agent-prefix/src/micro_ros_agent-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/amoralesma/Documents/UN/servos/servos-proyecto/uros_ws/build/micro_ros_agent/micro_ros_agent-prefix/src/micro_ros_agent-stamp${cfgdir}") # cfgdir has leading slash
endif()
