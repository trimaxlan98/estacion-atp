cmake_minimum_required(VERSION 3.16)

# El componente micro_ros_espidf_component (rama jazzy, commit fijado en el
# Dockerfile) lo deja build.sh en una cache escribible y avisa por entorno.
if(DEFINED ENV{MICRO_ROS_COMPONENT_DIR})
    list(APPEND EXTRA_COMPONENT_DIRS $ENV{MICRO_ROS_COMPONENT_DIR})
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(atp_esp32_eje)
