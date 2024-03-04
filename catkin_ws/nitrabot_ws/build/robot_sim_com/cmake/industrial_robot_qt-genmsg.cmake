# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "industrial_robot_qt: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iindustrial_robot_qt:/home/musashi/catkin_ws/nitrabot_ws/src/robot_sim_com/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(industrial_robot_qt_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/musashi/catkin_ws/nitrabot_ws/src/robot_sim_com/msg/RobotWheelVel.msg" NAME_WE)
add_custom_target(_industrial_robot_qt_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "industrial_robot_qt" "/home/musashi/catkin_ws/nitrabot_ws/src/robot_sim_com/msg/RobotWheelVel.msg" ""
)

get_filename_component(_filename "/home/musashi/catkin_ws/nitrabot_ws/src/robot_sim_com/msg/EncoderWheelVel.msg" NAME_WE)
add_custom_target(_industrial_robot_qt_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "industrial_robot_qt" "/home/musashi/catkin_ws/nitrabot_ws/src/robot_sim_com/msg/EncoderWheelVel.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(industrial_robot_qt
  "/home/musashi/catkin_ws/nitrabot_ws/src/robot_sim_com/msg/RobotWheelVel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/industrial_robot_qt
)
_generate_msg_cpp(industrial_robot_qt
  "/home/musashi/catkin_ws/nitrabot_ws/src/robot_sim_com/msg/EncoderWheelVel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/industrial_robot_qt
)

### Generating Services

### Generating Module File
_generate_module_cpp(industrial_robot_qt
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/industrial_robot_qt
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(industrial_robot_qt_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(industrial_robot_qt_generate_messages industrial_robot_qt_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/musashi/catkin_ws/nitrabot_ws/src/robot_sim_com/msg/RobotWheelVel.msg" NAME_WE)
add_dependencies(industrial_robot_qt_generate_messages_cpp _industrial_robot_qt_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/musashi/catkin_ws/nitrabot_ws/src/robot_sim_com/msg/EncoderWheelVel.msg" NAME_WE)
add_dependencies(industrial_robot_qt_generate_messages_cpp _industrial_robot_qt_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(industrial_robot_qt_gencpp)
add_dependencies(industrial_robot_qt_gencpp industrial_robot_qt_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS industrial_robot_qt_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(industrial_robot_qt
  "/home/musashi/catkin_ws/nitrabot_ws/src/robot_sim_com/msg/RobotWheelVel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/industrial_robot_qt
)
_generate_msg_eus(industrial_robot_qt
  "/home/musashi/catkin_ws/nitrabot_ws/src/robot_sim_com/msg/EncoderWheelVel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/industrial_robot_qt
)

### Generating Services

### Generating Module File
_generate_module_eus(industrial_robot_qt
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/industrial_robot_qt
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(industrial_robot_qt_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(industrial_robot_qt_generate_messages industrial_robot_qt_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/musashi/catkin_ws/nitrabot_ws/src/robot_sim_com/msg/RobotWheelVel.msg" NAME_WE)
add_dependencies(industrial_robot_qt_generate_messages_eus _industrial_robot_qt_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/musashi/catkin_ws/nitrabot_ws/src/robot_sim_com/msg/EncoderWheelVel.msg" NAME_WE)
add_dependencies(industrial_robot_qt_generate_messages_eus _industrial_robot_qt_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(industrial_robot_qt_geneus)
add_dependencies(industrial_robot_qt_geneus industrial_robot_qt_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS industrial_robot_qt_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(industrial_robot_qt
  "/home/musashi/catkin_ws/nitrabot_ws/src/robot_sim_com/msg/RobotWheelVel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/industrial_robot_qt
)
_generate_msg_lisp(industrial_robot_qt
  "/home/musashi/catkin_ws/nitrabot_ws/src/robot_sim_com/msg/EncoderWheelVel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/industrial_robot_qt
)

### Generating Services

### Generating Module File
_generate_module_lisp(industrial_robot_qt
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/industrial_robot_qt
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(industrial_robot_qt_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(industrial_robot_qt_generate_messages industrial_robot_qt_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/musashi/catkin_ws/nitrabot_ws/src/robot_sim_com/msg/RobotWheelVel.msg" NAME_WE)
add_dependencies(industrial_robot_qt_generate_messages_lisp _industrial_robot_qt_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/musashi/catkin_ws/nitrabot_ws/src/robot_sim_com/msg/EncoderWheelVel.msg" NAME_WE)
add_dependencies(industrial_robot_qt_generate_messages_lisp _industrial_robot_qt_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(industrial_robot_qt_genlisp)
add_dependencies(industrial_robot_qt_genlisp industrial_robot_qt_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS industrial_robot_qt_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(industrial_robot_qt
  "/home/musashi/catkin_ws/nitrabot_ws/src/robot_sim_com/msg/RobotWheelVel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/industrial_robot_qt
)
_generate_msg_nodejs(industrial_robot_qt
  "/home/musashi/catkin_ws/nitrabot_ws/src/robot_sim_com/msg/EncoderWheelVel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/industrial_robot_qt
)

### Generating Services

### Generating Module File
_generate_module_nodejs(industrial_robot_qt
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/industrial_robot_qt
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(industrial_robot_qt_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(industrial_robot_qt_generate_messages industrial_robot_qt_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/musashi/catkin_ws/nitrabot_ws/src/robot_sim_com/msg/RobotWheelVel.msg" NAME_WE)
add_dependencies(industrial_robot_qt_generate_messages_nodejs _industrial_robot_qt_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/musashi/catkin_ws/nitrabot_ws/src/robot_sim_com/msg/EncoderWheelVel.msg" NAME_WE)
add_dependencies(industrial_robot_qt_generate_messages_nodejs _industrial_robot_qt_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(industrial_robot_qt_gennodejs)
add_dependencies(industrial_robot_qt_gennodejs industrial_robot_qt_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS industrial_robot_qt_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(industrial_robot_qt
  "/home/musashi/catkin_ws/nitrabot_ws/src/robot_sim_com/msg/RobotWheelVel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/industrial_robot_qt
)
_generate_msg_py(industrial_robot_qt
  "/home/musashi/catkin_ws/nitrabot_ws/src/robot_sim_com/msg/EncoderWheelVel.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/industrial_robot_qt
)

### Generating Services

### Generating Module File
_generate_module_py(industrial_robot_qt
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/industrial_robot_qt
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(industrial_robot_qt_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(industrial_robot_qt_generate_messages industrial_robot_qt_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/musashi/catkin_ws/nitrabot_ws/src/robot_sim_com/msg/RobotWheelVel.msg" NAME_WE)
add_dependencies(industrial_robot_qt_generate_messages_py _industrial_robot_qt_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/musashi/catkin_ws/nitrabot_ws/src/robot_sim_com/msg/EncoderWheelVel.msg" NAME_WE)
add_dependencies(industrial_robot_qt_generate_messages_py _industrial_robot_qt_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(industrial_robot_qt_genpy)
add_dependencies(industrial_robot_qt_genpy industrial_robot_qt_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS industrial_robot_qt_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/industrial_robot_qt)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/industrial_robot_qt
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(industrial_robot_qt_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/industrial_robot_qt)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/industrial_robot_qt
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(industrial_robot_qt_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/industrial_robot_qt)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/industrial_robot_qt
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(industrial_robot_qt_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/industrial_robot_qt)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/industrial_robot_qt
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(industrial_robot_qt_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/industrial_robot_qt)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/industrial_robot_qt\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/industrial_robot_qt
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(industrial_robot_qt_generate_messages_py std_msgs_generate_messages_py)
endif()
