# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "custom_msg_f32: 1 messages, 0 services")

set(MSG_I_FLAGS "-Icustom_msg_f32:/lhome/mkhoshk/Desktop/MoCap_Project/camera/MoCap_ws/src/custom_msg_f32/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(custom_msg_f32_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/lhome/mkhoshk/Desktop/MoCap_Project/camera/MoCap_ws/src/custom_msg_f32/msg/Float32MultiArrayPlusStamp.msg" NAME_WE)
add_custom_target(_custom_msg_f32_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "custom_msg_f32" "/lhome/mkhoshk/Desktop/MoCap_Project/camera/MoCap_ws/src/custom_msg_f32/msg/Float32MultiArrayPlusStamp.msg" "std_msgs/MultiArrayDimension:std_msgs/Header:std_msgs/Float32MultiArray:std_msgs/MultiArrayLayout"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(custom_msg_f32
  "/lhome/mkhoshk/Desktop/MoCap_Project/camera/MoCap_ws/src/custom_msg_f32/msg/Float32MultiArrayPlusStamp.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/custom_msg_f32
)

### Generating Services

### Generating Module File
_generate_module_cpp(custom_msg_f32
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/custom_msg_f32
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(custom_msg_f32_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(custom_msg_f32_generate_messages custom_msg_f32_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/lhome/mkhoshk/Desktop/MoCap_Project/camera/MoCap_ws/src/custom_msg_f32/msg/Float32MultiArrayPlusStamp.msg" NAME_WE)
add_dependencies(custom_msg_f32_generate_messages_cpp _custom_msg_f32_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(custom_msg_f32_gencpp)
add_dependencies(custom_msg_f32_gencpp custom_msg_f32_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS custom_msg_f32_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(custom_msg_f32
  "/lhome/mkhoshk/Desktop/MoCap_Project/camera/MoCap_ws/src/custom_msg_f32/msg/Float32MultiArrayPlusStamp.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/custom_msg_f32
)

### Generating Services

### Generating Module File
_generate_module_eus(custom_msg_f32
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/custom_msg_f32
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(custom_msg_f32_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(custom_msg_f32_generate_messages custom_msg_f32_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/lhome/mkhoshk/Desktop/MoCap_Project/camera/MoCap_ws/src/custom_msg_f32/msg/Float32MultiArrayPlusStamp.msg" NAME_WE)
add_dependencies(custom_msg_f32_generate_messages_eus _custom_msg_f32_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(custom_msg_f32_geneus)
add_dependencies(custom_msg_f32_geneus custom_msg_f32_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS custom_msg_f32_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(custom_msg_f32
  "/lhome/mkhoshk/Desktop/MoCap_Project/camera/MoCap_ws/src/custom_msg_f32/msg/Float32MultiArrayPlusStamp.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/custom_msg_f32
)

### Generating Services

### Generating Module File
_generate_module_lisp(custom_msg_f32
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/custom_msg_f32
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(custom_msg_f32_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(custom_msg_f32_generate_messages custom_msg_f32_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/lhome/mkhoshk/Desktop/MoCap_Project/camera/MoCap_ws/src/custom_msg_f32/msg/Float32MultiArrayPlusStamp.msg" NAME_WE)
add_dependencies(custom_msg_f32_generate_messages_lisp _custom_msg_f32_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(custom_msg_f32_genlisp)
add_dependencies(custom_msg_f32_genlisp custom_msg_f32_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS custom_msg_f32_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(custom_msg_f32
  "/lhome/mkhoshk/Desktop/MoCap_Project/camera/MoCap_ws/src/custom_msg_f32/msg/Float32MultiArrayPlusStamp.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/custom_msg_f32
)

### Generating Services

### Generating Module File
_generate_module_nodejs(custom_msg_f32
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/custom_msg_f32
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(custom_msg_f32_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(custom_msg_f32_generate_messages custom_msg_f32_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/lhome/mkhoshk/Desktop/MoCap_Project/camera/MoCap_ws/src/custom_msg_f32/msg/Float32MultiArrayPlusStamp.msg" NAME_WE)
add_dependencies(custom_msg_f32_generate_messages_nodejs _custom_msg_f32_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(custom_msg_f32_gennodejs)
add_dependencies(custom_msg_f32_gennodejs custom_msg_f32_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS custom_msg_f32_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(custom_msg_f32
  "/lhome/mkhoshk/Desktop/MoCap_Project/camera/MoCap_ws/src/custom_msg_f32/msg/Float32MultiArrayPlusStamp.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Float32MultiArray.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/custom_msg_f32
)

### Generating Services

### Generating Module File
_generate_module_py(custom_msg_f32
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/custom_msg_f32
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(custom_msg_f32_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(custom_msg_f32_generate_messages custom_msg_f32_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/lhome/mkhoshk/Desktop/MoCap_Project/camera/MoCap_ws/src/custom_msg_f32/msg/Float32MultiArrayPlusStamp.msg" NAME_WE)
add_dependencies(custom_msg_f32_generate_messages_py _custom_msg_f32_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(custom_msg_f32_genpy)
add_dependencies(custom_msg_f32_genpy custom_msg_f32_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS custom_msg_f32_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/custom_msg_f32)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/custom_msg_f32
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(custom_msg_f32_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/custom_msg_f32)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/custom_msg_f32
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(custom_msg_f32_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/custom_msg_f32)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/custom_msg_f32
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(custom_msg_f32_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/custom_msg_f32)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/custom_msg_f32
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(custom_msg_f32_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/custom_msg_f32)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/custom_msg_f32\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/custom_msg_f32
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(custom_msg_f32_generate_messages_py std_msgs_generate_messages_py)
endif()
