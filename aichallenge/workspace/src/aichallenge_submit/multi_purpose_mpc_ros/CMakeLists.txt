cmake_minimum_required(VERSION 3.5.0)
project(multi_purpose_mpc_ros)

# Default to C++23
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/boost_commander.cpp
)

ament_auto_add_executable(boost_commander
  src/boost_commander_node.cpp
)

ament_python_install_package(${PROJECT_NAME}
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY multi_purpose_mpc_ros
  DESTINATION lib/python3.10/site-packages
)

install(PROGRAMS
  scripts/run_mpc_controller.bash
  scripts/run_mpc_simulation.bash
  scripts/run_reference_path_visualizer.bash
  scripts/run_reference_velocity_configulator.bash
  scripts/path_constraints_provider.bash
  scripts/mpc_controller
  scripts/mpc_simulation
  scripts/obstacle_parser.py
  scripts/path_constraints_provider
  scripts/publish_initialpose
  scripts/publish_initialpose_gnss
  scripts/reference_path_visualizer
  scripts/reference_velocity_configulator
  scripts/analyze_trajectory.py
  scripts/run_analyze_trajectory.bash
  DESTINATION lib/${PROJECT_NAME}
)

# install python dependencies in install directory site-packages
execute_process(
    COMMAND /usr/bin/python3 -m pip install --target ${CMAKE_INSTALL_PREFIX}/lib/python3.10/site-packages -r ./resources/requirements.txt
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    RESULT_VARIABLE install_result
)
if(install_result)
    message(FATAL_ERROR "Failed to install python dependencies in virtual environment")
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
  env
  resources
  include
)
