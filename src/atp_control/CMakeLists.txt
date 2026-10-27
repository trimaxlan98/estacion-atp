cmake_minimum_required(VERSION 3.16)
project(atp_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)

# ros2_control NO viene en la imagen base ros2-jazzy-dev (si en ros2-jazzy-gz, servicio `gz`).
# Con `colcon build` en la imagen base (CI, validar.sh) este paquete se omite sin error.
find_package(hardware_interface QUIET)
find_package(controller_interface QUIET)
find_package(controller_manager QUIET)
if(NOT hardware_interface_FOUND OR NOT controller_interface_FOUND OR NOT controller_manager_FOUND)
  message(WARNING "atp_control: ros2_control no esta instalado (imagen ros2-jazzy-dev); "
    "se omite. Use el servicio `gz` (docker/Dockerfile.gz).")
  ament_package()
  return()
endif()

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Un solo .so: el plugin de hardware, el del controlador y las piezas puras
add_library(atp_control SHARED
  src/mount_plant.cpp
  src/laws.cpp
  src/serial_protocol.cpp
  src/serial_port.cpp
  src/mount_backend.cpp
  src/atp_mount_system.cpp
  src/atp_tracking_controller.cpp)
target_include_directories(atp_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/atp_control>)
target_link_libraries(atp_control PUBLIC Eigen3::Eigen)
ament_target_dependencies(atp_control PUBLIC
  rclcpp rclcpp_lifecycle hardware_interface controller_interface pluginlib
  realtime_tools sensor_msgs)

pluginlib_export_plugin_description_file(hardware_interface atp_control_hardware.xml)
pluginlib_export_plugin_description_file(controller_interface atp_control_controllers.xml)

add_executable(lockstep_node src/lockstep_node.cpp)
target_link_libraries(lockstep_node atp_control)
ament_target_dependencies(lockstep_node controller_manager rosgraph_msgs std_msgs lifecycle_msgs)

install(TARGETS atp_control
  EXPORT export_atp_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS lockstep_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/atp_control)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/pointing_eval.py scripts/identify_plant.py scripts/atp_gz_stepper.py scripts/atp_gz_escena.py scripts/position_check.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(t test_plant test_laws test_serial test_controller_ref)
    ament_add_gtest(${t} test/${t}.cpp)
    target_link_libraries(${t} atp_control)
    target_compile_definitions(${t} PRIVATE
      TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
  endforeach()
endif()

ament_export_include_directories(include/atp_control)
ament_export_libraries(atp_control)
ament_export_dependencies(rclcpp_lifecycle hardware_interface controller_interface pluginlib realtime_tools)
ament_package()
