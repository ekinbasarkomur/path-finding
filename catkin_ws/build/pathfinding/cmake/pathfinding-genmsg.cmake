# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "pathfinding: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ipathfinding:/home/ekin/Freelance/Pathfinding/catkin_ws/src/pathfinding/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(pathfinding_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ekin/Freelance/Pathfinding/catkin_ws/src/pathfinding/msg/Target.msg" NAME_WE)
add_custom_target(_pathfinding_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pathfinding" "/home/ekin/Freelance/Pathfinding/catkin_ws/src/pathfinding/msg/Target.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(pathfinding
  "/home/ekin/Freelance/Pathfinding/catkin_ws/src/pathfinding/msg/Target.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pathfinding
)

### Generating Services

### Generating Module File
_generate_module_cpp(pathfinding
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pathfinding
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(pathfinding_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(pathfinding_generate_messages pathfinding_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ekin/Freelance/Pathfinding/catkin_ws/src/pathfinding/msg/Target.msg" NAME_WE)
add_dependencies(pathfinding_generate_messages_cpp _pathfinding_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pathfinding_gencpp)
add_dependencies(pathfinding_gencpp pathfinding_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pathfinding_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(pathfinding
  "/home/ekin/Freelance/Pathfinding/catkin_ws/src/pathfinding/msg/Target.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pathfinding
)

### Generating Services

### Generating Module File
_generate_module_eus(pathfinding
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pathfinding
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(pathfinding_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(pathfinding_generate_messages pathfinding_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ekin/Freelance/Pathfinding/catkin_ws/src/pathfinding/msg/Target.msg" NAME_WE)
add_dependencies(pathfinding_generate_messages_eus _pathfinding_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pathfinding_geneus)
add_dependencies(pathfinding_geneus pathfinding_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pathfinding_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(pathfinding
  "/home/ekin/Freelance/Pathfinding/catkin_ws/src/pathfinding/msg/Target.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pathfinding
)

### Generating Services

### Generating Module File
_generate_module_lisp(pathfinding
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pathfinding
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(pathfinding_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(pathfinding_generate_messages pathfinding_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ekin/Freelance/Pathfinding/catkin_ws/src/pathfinding/msg/Target.msg" NAME_WE)
add_dependencies(pathfinding_generate_messages_lisp _pathfinding_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pathfinding_genlisp)
add_dependencies(pathfinding_genlisp pathfinding_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pathfinding_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(pathfinding
  "/home/ekin/Freelance/Pathfinding/catkin_ws/src/pathfinding/msg/Target.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pathfinding
)

### Generating Services

### Generating Module File
_generate_module_nodejs(pathfinding
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pathfinding
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(pathfinding_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(pathfinding_generate_messages pathfinding_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ekin/Freelance/Pathfinding/catkin_ws/src/pathfinding/msg/Target.msg" NAME_WE)
add_dependencies(pathfinding_generate_messages_nodejs _pathfinding_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pathfinding_gennodejs)
add_dependencies(pathfinding_gennodejs pathfinding_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pathfinding_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(pathfinding
  "/home/ekin/Freelance/Pathfinding/catkin_ws/src/pathfinding/msg/Target.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pathfinding
)

### Generating Services

### Generating Module File
_generate_module_py(pathfinding
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pathfinding
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(pathfinding_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(pathfinding_generate_messages pathfinding_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ekin/Freelance/Pathfinding/catkin_ws/src/pathfinding/msg/Target.msg" NAME_WE)
add_dependencies(pathfinding_generate_messages_py _pathfinding_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pathfinding_genpy)
add_dependencies(pathfinding_genpy pathfinding_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pathfinding_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pathfinding)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pathfinding
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(pathfinding_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pathfinding)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pathfinding
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(pathfinding_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pathfinding)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pathfinding
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(pathfinding_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pathfinding)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pathfinding
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(pathfinding_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pathfinding)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pathfinding\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pathfinding
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(pathfinding_generate_messages_py std_msgs_generate_messages_py)
endif()
