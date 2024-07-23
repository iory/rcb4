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

# Command to copy the modified requirements.in to requirements.in.with_rcb4
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/requirements.in ${CMAKE_CURRENT_SOURCE_DIR}/requirements.in.with_rcb4 COPYONLY)
file(APPEND ${CMAKE_CURRENT_SOURCE_DIR}/requirements.in.with_rcb4 "${RCB4_DIR}\n")

# Add a CMake option for enabling/disabling virtualenv
option(USE_VIRTUALENV "Use Python virtual environment" ON)

if(USE_VIRTUALENV)
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
else()
  find_package(catkin REQUIRED COMPONENTS
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
  message(STATUS "Not using catkin_virtualenv as USE_VIRTUALENV is OFF.")
endif()

find_package(Eigen REQUIRED)

catkin_python_setup()

add_action_files(
  DIRECTORY action
  FILES ServoOnOff.action Stretch.action PressureControl.action
)

add_message_files(
  DIRECTORY msg
  FILES Stretch.msg
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp controller_manager hardware_interface angles urdf
)

if(USE_VIRTUALENV)
  catkin_generate_virtualenv(
    INPUT_REQUIREMENTS requirements.in.with_rcb4
    PYTHON_INTERPRETER python3
    USE_SYSTEM_PACKAGES TRUE
    ISOLATE_REQUIREMENTS FALSE
    CHECK_VENV FALSE
  )
endif()

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

file(GLOB PYTHON_SCRIPT_FILES scripts/* node_scripts/* test/*)
catkin_install_python(
  PROGRAMS ${PYTHON_SCRIPT_FILES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES requirements.txt
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS kxr_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

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
  add_rostest(test/kxr_controller.test ARGS namespace:="" use_rcb4:=true)
  add_rostest(test/kxr_controller.test ARGS namespace:="robot_a" use_rcb4:=true)
endif()
