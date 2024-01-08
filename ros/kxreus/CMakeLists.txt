cmake_minimum_required(VERSION 2.8.3)
project(kxreus)

find_package(catkin REQUIRED
  catkin_virtualenv
  kxr_models
)


if(EXISTS ${kxr_models_SOURCE_PREFIX}/urdf/kxrl2l2a6h2m.urdf)
  set(kxr_urdf ${kxr_models_SOURCE_PREFIX}/urdf/kxrl2l2a6h2m.urdf)
else()
  message(WARNING "Could not found kxr_models")
endif()

catkin_package(
  DEPENDS
  LIBRARIES ${PROJECT_NAME}
)

catkin_generate_virtualenv(
  INPUT_REQUIREMENTS requirements.in
  PYTHON_INTERPRETER python3
  USE_SYSTEM_PACKAGES TRUE
  ISOLATE_REQUIREMENTS FALSE
  CHECK_VENV FALSE
)

set(_urdf2eus "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/venv/bin/urdf2eus")
set(_venv_setup "${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/venv/bin/activate")
set(_yaml_path ${PROJECT_SOURCE_DIR}/config/kxrl2l2a6h2m.yaml)

add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/kxrl2l2a6h2m.l
  COMMAND echo "${_urdf2eus} ${kxr_urdf} ${PROJECT_SOURCE_DIR}/kxrl2l2a6h2m.l"
  COMMAND . ${_venv_setup} && ${_urdf2eus} ${kxr_urdf} ${PROJECT_SOURCE_DIR}/models/kxrl2l2a6h2m.l --yaml-path ${_yaml_path}
  DEPENDS ${kxr_urdf} ${_urdf2eus} ${_venv_setup} ${_yaml_path})

add_custom_target(compile_kxr ALL DEPENDS ${PROJECT_SOURCE_DIR}/kxrl2l2a6h2m.l ${PROJECT_NAME}_generate_virtualenv)
