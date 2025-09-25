# Install script for directory: /Users/amoralesma/Documents/UN/servos/servos-proyecto/uros_ws/src/micro_ros_setup

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/Users/amoralesma/Documents/UN/servos/servos-proyecto/uros_ws/install/micro_ros_setup")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/Users/amoralesma/conda/envs/humble/bin/llvm-objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/Users/amoralesma/Documents/UN/servos/servos-proyecto/uros_ws/build/micro_ros_setup/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/micro_ros_setup")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/Users/amoralesma/Documents/UN/servos/servos-proyecto/uros_ws/build/micro_ros_setup/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/micro_ros_setup")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/micro_ros_setup/environment" TYPE FILE FILES "/Users/amoralesma/conda/envs/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/micro_ros_setup/environment" TYPE FILE FILES "/Users/amoralesma/Documents/UN/servos/servos-proyecto/uros_ws/build/micro_ros_setup/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/micro_ros_setup/environment" TYPE FILE FILES "/Users/amoralesma/conda/envs/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/micro_ros_setup/environment" TYPE FILE FILES "/Users/amoralesma/Documents/UN/servos/servos-proyecto/uros_ws/build/micro_ros_setup/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/micro_ros_setup" TYPE FILE FILES "/Users/amoralesma/Documents/UN/servos/servos-proyecto/uros_ws/build/micro_ros_setup/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/micro_ros_setup" TYPE FILE FILES "/Users/amoralesma/Documents/UN/servos/servos-proyecto/uros_ws/build/micro_ros_setup/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/micro_ros_setup" TYPE FILE FILES "/Users/amoralesma/Documents/UN/servos/servos-proyecto/uros_ws/build/micro_ros_setup/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/micro_ros_setup" TYPE FILE FILES "/Users/amoralesma/Documents/UN/servos/servos-proyecto/uros_ws/build/micro_ros_setup/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/micro_ros_setup" TYPE FILE FILES "/Users/amoralesma/Documents/UN/servos/servos-proyecto/uros_ws/build/micro_ros_setup/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/Users/amoralesma/Documents/UN/servos/servos-proyecto/uros_ws/build/micro_ros_setup/ament_cmake_index/share/ament_index/resource_index/packages/micro_ros_setup")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/micro_ros_setup/cmake" TYPE FILE FILES
    "/Users/amoralesma/Documents/UN/servos/servos-proyecto/uros_ws/build/micro_ros_setup/ament_cmake_core/micro_ros_setupConfig.cmake"
    "/Users/amoralesma/Documents/UN/servos/servos-proyecto/uros_ws/build/micro_ros_setup/ament_cmake_core/micro_ros_setupConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/micro_ros_setup" TYPE FILE FILES "/Users/amoralesma/Documents/UN/servos/servos-proyecto/uros_ws/src/micro_ros_setup/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/." TYPE DIRECTORY FILES "/Users/amoralesma/Documents/UN/servos/servos-proyecto/uros_ws/src/micro_ros_setup/config" USE_SOURCE_PERMISSIONS)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/micro_ros_setup" TYPE PROGRAM FILES
    "/Users/amoralesma/Documents/UN/servos/servos-proyecto/uros_ws/src/micro_ros_setup/scripts/create_ws.sh"
    "/Users/amoralesma/Documents/UN/servos/servos-proyecto/uros_ws/src/micro_ros_setup/scripts/create_agent_ws.sh"
    "/Users/amoralesma/Documents/UN/servos/servos-proyecto/uros_ws/src/micro_ros_setup/scripts/create_firmware_ws.sh"
    "/Users/amoralesma/Documents/UN/servos/servos-proyecto/uros_ws/src/micro_ros_setup/scripts/configure_firmware.sh"
    "/Users/amoralesma/Documents/UN/servos/servos-proyecto/uros_ws/src/micro_ros_setup/scripts/flash_firmware.sh"
    "/Users/amoralesma/Documents/UN/servos/servos-proyecto/uros_ws/src/micro_ros_setup/scripts/build_firmware.sh"
    "/Users/amoralesma/Documents/UN/servos/servos-proyecto/uros_ws/src/micro_ros_setup/scripts/build_agent.sh"
    "/Users/amoralesma/Documents/UN/servos/servos-proyecto/uros_ws/src/micro_ros_setup/scripts/yaml_filter.py"
    "/Users/amoralesma/Documents/UN/servos/servos-proyecto/uros_ws/src/micro_ros_setup/scripts/clean_env.sh"
    "/Users/amoralesma/Documents/UN/servos/servos-proyecto/uros_ws/src/micro_ros_setup/scripts/component"
    )
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/Users/amoralesma/Documents/UN/servos/servos-proyecto/uros_ws/build/micro_ros_setup/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
if(CMAKE_INSTALL_COMPONENT)
  if(CMAKE_INSTALL_COMPONENT MATCHES "^[a-zA-Z0-9_.+-]+$")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
  else()
    string(MD5 CMAKE_INST_COMP_HASH "${CMAKE_INSTALL_COMPONENT}")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INST_COMP_HASH}.txt")
    unset(CMAKE_INST_COMP_HASH)
  endif()
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/Users/amoralesma/Documents/UN/servos/servos-proyecto/uros_ws/build/micro_ros_setup/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
