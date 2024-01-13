cmake_minimum_required(VERSION 2.8.3)
project(kxr_controller)

message(STATUS "CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")

execute_process(
  COMMAND ${CMAKE_CXX_COMPILER} -dumpversion
  OUTPUT_VARIABLE GCC_VERSION
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm" OR CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  message(STATUS "Configuring for ARM or AArch64 architecture.")
endif()

# To avoid issues with pub/sub in ROS topic on ARM architecture, compile with O2 optimization.
# https://github.com/ros/roscpp_core/pull/136
if(GCC_VERSION VERSION_LESS 9.4.0 AND (CMAKE_SYSTEM_PROCESSOR MATCHES "arm" OR CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64") AND CMAKE_BUILD_TYPE STREQUAL "Release")
  set(CMAKE_CXX_FLAGS_RELEASE "-O2")
  message(STATUS "GCC version is less than 9.4.0 and target architecture is ARM. Setting CXX_FLAGS_RELEASE to -O2.")
endif()

get_filename_component(RCB4_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
message(STATUS "RCB4 Directory: ${RCB4_DIR}")

find_package(catkin REQUIRED COMPONENTS
  catkin_virtualenv
  actionlib
  actionlib_msgs
  std_msgs
  roscpp
  cmake_modules
  controller_manager
  hardware_interface
  angles
  urdf
)
find_package(Eigen REQUIRED)

catkin_python_setup()

add_action_files(
  DIRECTORY action
  FILES ServoOnOff.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp controller_manager hardware_interface angles urdf
)

catkin_generate_virtualenv(
  INPUT_REQUIREMENTS requirements.in
  PYTHON_INTERPRETER python3
  USE_SYSTEM_PACKAGES TRUE
  ISOLATE_REQUIREMENTS FALSE
  CHECK_VENV FALSE
)

set(_python "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/venv/bin/python")
set(_venv_setup "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/venv/bin/activate")

add_custom_command(
  OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/after_venv_script"
  COMMAND echo "${_python} -m pip install ${RCB4_DIR}"
  COMMAND . ${_venv_setup} && ${_python} -m pip install ${RCB4_DIR}
  DEPENDS ${PROJECT_NAME}_generate_virtualenv ${_venv_setup} ${_python}
)


add_custom_target(
  run_after_venv ALL
  DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/after_venv_script"
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(kxr_controller src/kxr_ros_controller.cpp src/kxr_robot_hardware.cpp)
add_dependencies(kxr_controller ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(kxr_controller
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)

file(GLOB PYTHON_SCRIPT_FILES node_scripts/* test/*)
catkin_install_python(
  PROGRAMS ${PYTHON_SCRIPT_FILES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS kxr_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roslaunch REQUIRED)
  if(roslaunch_VERSION VERSION_LESS "1.11.1")
    message(WARNING "roslaunch_add_file check fails with unsupported doc attributes ${roslaunch_VERSION}")
  else()
    file(GLOB LAUNCH_FILES launch/*.launch)
    foreach(LAUNCH_FILE ${LAUNCH_FILES})
      roslaunch_add_file_check(${LAUNCH_FILE})
    endforeach()
  endif()
  add_rostest(test/kxr_controller.test ARGS namespace:="")
  add_rostest(test/kxr_controller.test ARGS namespace:="robot_a")
endif()
