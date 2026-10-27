# Nodo micro-ROS del eje sobre Linux (planta J/b en lugar de hardware).
#   cmake -S host -B .cache/host_build && cmake --build .cache/host_build
# Requiere .cache/host_uros/ (host/build_libmicroros.sh).
cmake_minimum_required(VERSION 3.16)
project(eje_host C)

set(CMAKE_C_STANDARD 17)
set(UROS ${CMAKE_CURRENT_SOURCE_DIR}/../.cache/host_uros)
if(NOT EXISTS ${UROS}/libmicroros.a)
  message(FATAL_ERROR "falta ${UROS}/libmicroros.a: correr host/build_libmicroros.sh")
endif()

add_executable(eje_host
  main_host.c compartido_host.c
  ../core/eje_lazo.c ../core/eje_planta.c ../core/eje_abs.c
  ../uros/eje_uros.c)

target_include_directories(eje_host PRIVATE ../core ../uros ${UROS}/include)
# igual que el componente de IDF: una carpeta de include por paquete
file(GLOB UROS_INC LIST_DIRECTORIES true ${UROS}/include/*)
foreach(d ${UROS_INC})
  if(IS_DIRECTORY ${d})
    target_include_directories(eje_host PRIVATE ${d})
  endif()
endforeach()
target_compile_options(eje_host PRIVATE -Wall -Wextra -Wno-unused-parameter)
target_link_libraries(eje_host ${UROS}/libmicroros.a pthread m)
