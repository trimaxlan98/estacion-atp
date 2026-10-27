# Pruebas del lazo del ESP32 en el host:
#   cmake -S . -B build && cmake --build build && ctest --test-dir build --output-on-failure
# Solo C99 + libm. Sin ESP-IDF, sin ROS.
cmake_minimum_required(VERSION 3.16)
project(eje_pruebas C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wshadow -Wconversion -Wno-sign-conversion -Werror -O2)

set(NUCLEO ../core/eje_lazo.c ../core/eje_abs.c ../core/eje_planta.c)
enable_testing()

foreach(t test_lazo test_abs test_vectores test_dos_ejes)
  add_executable(${t} ${t}.c ${NUCLEO})
  target_link_libraries(${t} m)
endforeach()

add_test(NAME lazo COMMAND test_lazo)
add_test(NAME abs COMMAND test_abs)
add_test(NAME dos_ejes COMMAND test_dos_ejes)
add_test(NAME vectores_atp_ros
         COMMAND test_vectores ${CMAKE_CURRENT_SOURCE_DIR}/vectores)

# el buzon compartido de la version de Linux (mutex POSIX) lo prueba test_dos_ejes
target_sources(test_dos_ejes PRIVATE ../host/compartido_host.c)
target_include_directories(test_dos_ejes PRIVATE ../uros)
target_link_libraries(test_dos_ejes pthread)
