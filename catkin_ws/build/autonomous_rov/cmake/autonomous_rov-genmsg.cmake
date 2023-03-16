# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "autonomous_rov: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iautonomous_rov:/home/farooq/Documents/MIRBlueROV/catkin_ws/src/autonomous_rov/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(autonomous_rov_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/farooq/Documents/MIRBlueROV/catkin_ws/src/autonomous_rov/msg/Thruster.msg" NAME_WE)
add_custom_target(_autonomous_rov_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "autonomous_rov" "/home/farooq/Documents/MIRBlueROV/catkin_ws/src/autonomous_rov/msg/Thruster.msg" "std_msgs/Header:std_msgs/UInt16MultiArray:std_msgs/MultiArrayDimension:std_msgs/MultiArrayLayout"
)

get_filename_component(_filename "/home/farooq/Documents/MIRBlueROV/catkin_ws/src/autonomous_rov/msg/Health.msg" NAME_WE)
add_custom_target(_autonomous_rov_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "autonomous_rov" "/home/farooq/Documents/MIRBlueROV/catkin_ws/src/autonomous_rov/msg/Health.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(autonomous_rov
  "/home/farooq/Documents/MIRBlueROV/catkin_ws/src/autonomous_rov/msg/Thruster.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt16MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/autonomous_rov
)
_generate_msg_cpp(autonomous_rov
  "/home/farooq/Documents/MIRBlueROV/catkin_ws/src/autonomous_rov/msg/Health.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/autonomous_rov
)

### Generating Services

### Generating Module File
_generate_module_cpp(autonomous_rov
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/autonomous_rov
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(autonomous_rov_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(autonomous_rov_generate_messages autonomous_rov_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/farooq/Documents/MIRBlueROV/catkin_ws/src/autonomous_rov/msg/Thruster.msg" NAME_WE)
add_dependencies(autonomous_rov_generate_messages_cpp _autonomous_rov_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/farooq/Documents/MIRBlueROV/catkin_ws/src/autonomous_rov/msg/Health.msg" NAME_WE)
add_dependencies(autonomous_rov_generate_messages_cpp _autonomous_rov_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(autonomous_rov_gencpp)
add_dependencies(autonomous_rov_gencpp autonomous_rov_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS autonomous_rov_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(autonomous_rov
  "/home/farooq/Documents/MIRBlueROV/catkin_ws/src/autonomous_rov/msg/Thruster.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt16MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/autonomous_rov
)
_generate_msg_eus(autonomous_rov
  "/home/farooq/Documents/MIRBlueROV/catkin_ws/src/autonomous_rov/msg/Health.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/autonomous_rov
)

### Generating Services

### Generating Module File
_generate_module_eus(autonomous_rov
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/autonomous_rov
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(autonomous_rov_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(autonomous_rov_generate_messages autonomous_rov_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/farooq/Documents/MIRBlueROV/catkin_ws/src/autonomous_rov/msg/Thruster.msg" NAME_WE)
add_dependencies(autonomous_rov_generate_messages_eus _autonomous_rov_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/farooq/Documents/MIRBlueROV/catkin_ws/src/autonomous_rov/msg/Health.msg" NAME_WE)
add_dependencies(autonomous_rov_generate_messages_eus _autonomous_rov_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(autonomous_rov_geneus)
add_dependencies(autonomous_rov_geneus autonomous_rov_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS autonomous_rov_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(autonomous_rov
  "/home/farooq/Documents/MIRBlueROV/catkin_ws/src/autonomous_rov/msg/Thruster.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt16MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/autonomous_rov
)
_generate_msg_lisp(autonomous_rov
  "/home/farooq/Documents/MIRBlueROV/catkin_ws/src/autonomous_rov/msg/Health.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/autonomous_rov
)

### Generating Services

### Generating Module File
_generate_module_lisp(autonomous_rov
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/autonomous_rov
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(autonomous_rov_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(autonomous_rov_generate_messages autonomous_rov_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/farooq/Documents/MIRBlueROV/catkin_ws/src/autonomous_rov/msg/Thruster.msg" NAME_WE)
add_dependencies(autonomous_rov_generate_messages_lisp _autonomous_rov_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/farooq/Documents/MIRBlueROV/catkin_ws/src/autonomous_rov/msg/Health.msg" NAME_WE)
add_dependencies(autonomous_rov_generate_messages_lisp _autonomous_rov_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(autonomous_rov_genlisp)
add_dependencies(autonomous_rov_genlisp autonomous_rov_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS autonomous_rov_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(autonomous_rov
  "/home/farooq/Documents/MIRBlueROV/catkin_ws/src/autonomous_rov/msg/Thruster.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt16MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/autonomous_rov
)
_generate_msg_nodejs(autonomous_rov
  "/home/farooq/Documents/MIRBlueROV/catkin_ws/src/autonomous_rov/msg/Health.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/autonomous_rov
)

### Generating Services

### Generating Module File
_generate_module_nodejs(autonomous_rov
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/autonomous_rov
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(autonomous_rov_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(autonomous_rov_generate_messages autonomous_rov_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/farooq/Documents/MIRBlueROV/catkin_ws/src/autonomous_rov/msg/Thruster.msg" NAME_WE)
add_dependencies(autonomous_rov_generate_messages_nodejs _autonomous_rov_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/farooq/Documents/MIRBlueROV/catkin_ws/src/autonomous_rov/msg/Health.msg" NAME_WE)
add_dependencies(autonomous_rov_generate_messages_nodejs _autonomous_rov_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(autonomous_rov_gennodejs)
add_dependencies(autonomous_rov_gennodejs autonomous_rov_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS autonomous_rov_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(autonomous_rov
  "/home/farooq/Documents/MIRBlueROV/catkin_ws/src/autonomous_rov/msg/Thruster.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/UInt16MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autonomous_rov
)
_generate_msg_py(autonomous_rov
  "/home/farooq/Documents/MIRBlueROV/catkin_ws/src/autonomous_rov/msg/Health.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autonomous_rov
)

### Generating Services

### Generating Module File
_generate_module_py(autonomous_rov
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autonomous_rov
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(autonomous_rov_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(autonomous_rov_generate_messages autonomous_rov_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/farooq/Documents/MIRBlueROV/catkin_ws/src/autonomous_rov/msg/Thruster.msg" NAME_WE)
add_dependencies(autonomous_rov_generate_messages_py _autonomous_rov_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/farooq/Documents/MIRBlueROV/catkin_ws/src/autonomous_rov/msg/Health.msg" NAME_WE)
add_dependencies(autonomous_rov_generate_messages_py _autonomous_rov_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(autonomous_rov_genpy)
add_dependencies(autonomous_rov_genpy autonomous_rov_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS autonomous_rov_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/autonomous_rov)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/autonomous_rov
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(autonomous_rov_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/autonomous_rov)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/autonomous_rov
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(autonomous_rov_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/autonomous_rov)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/autonomous_rov
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(autonomous_rov_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/autonomous_rov)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/autonomous_rov
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(autonomous_rov_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autonomous_rov)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autonomous_rov\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/autonomous_rov
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(autonomous_rov_generate_messages_py std_msgs_generate_messages_py)
endif()