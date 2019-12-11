# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "race: 3 messages, 0 services")

set(MSG_I_FLAGS "-Irace:/home/musaup/Documents/Research/Platooning-F1Tenth/src/f110-fall2018-skeletons/simulator/f1_10_sim/race/msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(race_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/musaup/Documents/Research/Platooning-F1Tenth/src/f110-fall2018-skeletons/simulator/f1_10_sim/race/msg/drive_param.msg" NAME_WE)
add_custom_target(_race_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "race" "/home/musaup/Documents/Research/Platooning-F1Tenth/src/f110-fall2018-skeletons/simulator/f1_10_sim/race/msg/drive_param.msg" ""
)

get_filename_component(_filename "/home/musaup/Documents/Research/Platooning-F1Tenth/src/f110-fall2018-skeletons/simulator/f1_10_sim/race/msg/drive_values.msg" NAME_WE)
add_custom_target(_race_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "race" "/home/musaup/Documents/Research/Platooning-F1Tenth/src/f110-fall2018-skeletons/simulator/f1_10_sim/race/msg/drive_values.msg" ""
)

get_filename_component(_filename "/home/musaup/Documents/Research/Platooning-F1Tenth/src/f110-fall2018-skeletons/simulator/f1_10_sim/race/msg/pid_input.msg" NAME_WE)
add_custom_target(_race_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "race" "/home/musaup/Documents/Research/Platooning-F1Tenth/src/f110-fall2018-skeletons/simulator/f1_10_sim/race/msg/pid_input.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(race
  "/home/musaup/Documents/Research/Platooning-F1Tenth/src/f110-fall2018-skeletons/simulator/f1_10_sim/race/msg/drive_param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/race
)
_generate_msg_cpp(race
  "/home/musaup/Documents/Research/Platooning-F1Tenth/src/f110-fall2018-skeletons/simulator/f1_10_sim/race/msg/drive_values.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/race
)
_generate_msg_cpp(race
  "/home/musaup/Documents/Research/Platooning-F1Tenth/src/f110-fall2018-skeletons/simulator/f1_10_sim/race/msg/pid_input.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/race
)

### Generating Services

### Generating Module File
_generate_module_cpp(race
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/race
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(race_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(race_generate_messages race_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/musaup/Documents/Research/Platooning-F1Tenth/src/f110-fall2018-skeletons/simulator/f1_10_sim/race/msg/drive_param.msg" NAME_WE)
add_dependencies(race_generate_messages_cpp _race_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/musaup/Documents/Research/Platooning-F1Tenth/src/f110-fall2018-skeletons/simulator/f1_10_sim/race/msg/drive_values.msg" NAME_WE)
add_dependencies(race_generate_messages_cpp _race_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/musaup/Documents/Research/Platooning-F1Tenth/src/f110-fall2018-skeletons/simulator/f1_10_sim/race/msg/pid_input.msg" NAME_WE)
add_dependencies(race_generate_messages_cpp _race_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(race_gencpp)
add_dependencies(race_gencpp race_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS race_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(race
  "/home/musaup/Documents/Research/Platooning-F1Tenth/src/f110-fall2018-skeletons/simulator/f1_10_sim/race/msg/drive_param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/race
)
_generate_msg_eus(race
  "/home/musaup/Documents/Research/Platooning-F1Tenth/src/f110-fall2018-skeletons/simulator/f1_10_sim/race/msg/drive_values.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/race
)
_generate_msg_eus(race
  "/home/musaup/Documents/Research/Platooning-F1Tenth/src/f110-fall2018-skeletons/simulator/f1_10_sim/race/msg/pid_input.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/race
)

### Generating Services

### Generating Module File
_generate_module_eus(race
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/race
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(race_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(race_generate_messages race_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/musaup/Documents/Research/Platooning-F1Tenth/src/f110-fall2018-skeletons/simulator/f1_10_sim/race/msg/drive_param.msg" NAME_WE)
add_dependencies(race_generate_messages_eus _race_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/musaup/Documents/Research/Platooning-F1Tenth/src/f110-fall2018-skeletons/simulator/f1_10_sim/race/msg/drive_values.msg" NAME_WE)
add_dependencies(race_generate_messages_eus _race_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/musaup/Documents/Research/Platooning-F1Tenth/src/f110-fall2018-skeletons/simulator/f1_10_sim/race/msg/pid_input.msg" NAME_WE)
add_dependencies(race_generate_messages_eus _race_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(race_geneus)
add_dependencies(race_geneus race_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS race_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(race
  "/home/musaup/Documents/Research/Platooning-F1Tenth/src/f110-fall2018-skeletons/simulator/f1_10_sim/race/msg/drive_param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/race
)
_generate_msg_lisp(race
  "/home/musaup/Documents/Research/Platooning-F1Tenth/src/f110-fall2018-skeletons/simulator/f1_10_sim/race/msg/drive_values.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/race
)
_generate_msg_lisp(race
  "/home/musaup/Documents/Research/Platooning-F1Tenth/src/f110-fall2018-skeletons/simulator/f1_10_sim/race/msg/pid_input.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/race
)

### Generating Services

### Generating Module File
_generate_module_lisp(race
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/race
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(race_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(race_generate_messages race_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/musaup/Documents/Research/Platooning-F1Tenth/src/f110-fall2018-skeletons/simulator/f1_10_sim/race/msg/drive_param.msg" NAME_WE)
add_dependencies(race_generate_messages_lisp _race_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/musaup/Documents/Research/Platooning-F1Tenth/src/f110-fall2018-skeletons/simulator/f1_10_sim/race/msg/drive_values.msg" NAME_WE)
add_dependencies(race_generate_messages_lisp _race_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/musaup/Documents/Research/Platooning-F1Tenth/src/f110-fall2018-skeletons/simulator/f1_10_sim/race/msg/pid_input.msg" NAME_WE)
add_dependencies(race_generate_messages_lisp _race_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(race_genlisp)
add_dependencies(race_genlisp race_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS race_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(race
  "/home/musaup/Documents/Research/Platooning-F1Tenth/src/f110-fall2018-skeletons/simulator/f1_10_sim/race/msg/drive_param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/race
)
_generate_msg_nodejs(race
  "/home/musaup/Documents/Research/Platooning-F1Tenth/src/f110-fall2018-skeletons/simulator/f1_10_sim/race/msg/drive_values.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/race
)
_generate_msg_nodejs(race
  "/home/musaup/Documents/Research/Platooning-F1Tenth/src/f110-fall2018-skeletons/simulator/f1_10_sim/race/msg/pid_input.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/race
)

### Generating Services

### Generating Module File
_generate_module_nodejs(race
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/race
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(race_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(race_generate_messages race_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/musaup/Documents/Research/Platooning-F1Tenth/src/f110-fall2018-skeletons/simulator/f1_10_sim/race/msg/drive_param.msg" NAME_WE)
add_dependencies(race_generate_messages_nodejs _race_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/musaup/Documents/Research/Platooning-F1Tenth/src/f110-fall2018-skeletons/simulator/f1_10_sim/race/msg/drive_values.msg" NAME_WE)
add_dependencies(race_generate_messages_nodejs _race_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/musaup/Documents/Research/Platooning-F1Tenth/src/f110-fall2018-skeletons/simulator/f1_10_sim/race/msg/pid_input.msg" NAME_WE)
add_dependencies(race_generate_messages_nodejs _race_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(race_gennodejs)
add_dependencies(race_gennodejs race_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS race_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(race
  "/home/musaup/Documents/Research/Platooning-F1Tenth/src/f110-fall2018-skeletons/simulator/f1_10_sim/race/msg/drive_param.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/race
)
_generate_msg_py(race
  "/home/musaup/Documents/Research/Platooning-F1Tenth/src/f110-fall2018-skeletons/simulator/f1_10_sim/race/msg/drive_values.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/race
)
_generate_msg_py(race
  "/home/musaup/Documents/Research/Platooning-F1Tenth/src/f110-fall2018-skeletons/simulator/f1_10_sim/race/msg/pid_input.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/race
)

### Generating Services

### Generating Module File
_generate_module_py(race
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/race
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(race_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(race_generate_messages race_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/musaup/Documents/Research/Platooning-F1Tenth/src/f110-fall2018-skeletons/simulator/f1_10_sim/race/msg/drive_param.msg" NAME_WE)
add_dependencies(race_generate_messages_py _race_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/musaup/Documents/Research/Platooning-F1Tenth/src/f110-fall2018-skeletons/simulator/f1_10_sim/race/msg/drive_values.msg" NAME_WE)
add_dependencies(race_generate_messages_py _race_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/musaup/Documents/Research/Platooning-F1Tenth/src/f110-fall2018-skeletons/simulator/f1_10_sim/race/msg/pid_input.msg" NAME_WE)
add_dependencies(race_generate_messages_py _race_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(race_genpy)
add_dependencies(race_genpy race_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS race_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/race)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/race
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(race_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/race)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/race
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(race_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/race)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/race
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(race_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/race)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/race
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(race_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/race)
  install(CODE "execute_process(COMMAND \"/home/musaup/anaconda3/envs/ros_test/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/race\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/race
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(race_generate_messages_py sensor_msgs_generate_messages_py)
endif()
