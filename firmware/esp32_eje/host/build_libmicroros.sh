#!/usr/bin/env bash
# Compila libmicroros.a para LINUX (x86_64) con el mismo conjunto de paquetes y
# ramas que usa micro_ros_espidf_component (libmicroros.mk, rama jazzy), pero con
# el compilador nativo y el transporte UDP de Micro-XRCE-DDS-Client.
#
# Dentro del contenedor ROS (imagen ros2-jazzy-dev), desde la raiz del repo:
#   docker compose run --rm -T ros bash firmware/esp32_eje/host/build_libmicroros.sh
# Resultado: firmware/esp32_eje/.cache/host_uros/{libmicroros.a,include/,commits.txt}
# Necesita red (clona ~25 repos de GitHub por RAMA; commits.txt deja constancia).
set -euo pipefail
RAIZ="$(cd "$(dirname "$0")/../../.." && pwd)"
SALIDA="${1:-$RAIZ/firmware/esp32_eje/.cache/host_uros}"
META="$RAIZ/firmware/esp32_eje/host/colcon.meta"

# entorno limpio: ni ROS 2 instalado ni su PYTHONPATH (libmicroros compila sus
# propias herramientas ament y se pisaria con las del sistema)
if [ -z "${EJE_ENTORNO_LIMPIO:-}" ]; then
    exec env -i EJE_ENTORNO_LIMPIO=1 HOME="${HOME:-/tmp}" PATH=/usr/local/bin:/usr/bin:/bin \
         bash "$0" "$@"
fi

if [ -f "$SALIDA/libmicroros.a" ]; then
    echo "ya existe $SALIDA/libmicroros.a (borrar $SALIDA para recompilar)"
    exit 0
fi
mkdir -p "$SALIDA"
cd "$SALIDA"

echo "=== 1/3 herramientas ament nativas (micro_ros_dev)"
if [ ! -d micro_ros_dev/install ]; then
rm -rf micro_ros_dev
mkdir micro_ros_dev
(
    cd micro_ros_dev
    git clone -q -b jazzy https://github.com/ament/ament_cmake src/ament_cmake
    git clone -q -b jazzy https://github.com/ament/ament_lint src/ament_lint
    git clone -q -b jazzy https://github.com/ament/ament_package src/ament_package
    git clone -q -b jazzy https://github.com/ament/googletest src/googletest
    git clone -q -b jazzy https://github.com/ros2/ament_cmake_ros src/ament_cmake_ros
    git clone -q -b jazzy https://github.com/ament/ament_index src/ament_index
    colcon build --cmake-args -DBUILD_TESTING=OFF -DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=gcc
)
fi

echo "=== 2/3 fuentes de micro-ROS (mismas ramas que libmicroros.mk)"
if [ ! -f commits.txt ]; then
rm -rf micro_ros_src
mkdir micro_ros_src
(
    cd micro_ros_src
    git clone -q -b ros2 https://github.com/eProsima/Micro-XRCE-DDS-Client src/Micro-XRCE-DDS-Client
    git clone -q -b jazzy https://github.com/micro-ROS/rmw_microxrcedds src/rmw_microxrcedds
    git clone -q -b ros2 https://github.com/eProsima/micro-CDR src/micro-CDR
    git clone -q -b jazzy https://github.com/micro-ROS/rcl src/rcl
    git clone -q -b jazzy https://github.com/ros2/rclc src/rclc
    git clone -q -b jazzy https://github.com/micro-ROS/rcutils src/rcutils
    git clone -q -b jazzy https://github.com/micro-ROS/micro_ros_msgs src/micro_ros_msgs
    git clone -q -b jazzy https://github.com/micro-ROS/rosidl_typesupport src/rosidl_typesupport
    git clone -q -b jazzy https://github.com/micro-ROS/rosidl_typesupport_microxrcedds src/rosidl_typesupport_microxrcedds
    git clone -q -b jazzy https://github.com/ros2/rosidl src/rosidl
    git clone -q -b jazzy https://github.com/ros2/rosidl_dynamic_typesupport src/rosidl_dynamic_typesupport
    git clone -q -b jazzy https://github.com/ros2/rmw src/rmw
    git clone -q -b jazzy https://github.com/ros2/rcl_interfaces src/rcl_interfaces
    git clone -q -b jazzy https://github.com/ros2/rosidl_defaults src/rosidl_defaults
    git clone -q -b jazzy https://github.com/ros2/unique_identifier_msgs src/unique_identifier_msgs
    git clone -q -b jazzy https://github.com/ros2/common_interfaces src/common_interfaces
    git clone -q -b jazzy https://github.com/ros2/example_interfaces src/example_interfaces
    git clone -q -b jazzy https://github.com/ros2/test_interface_files src/test_interface_files
    git clone -q -b jazzy https://github.com/ros2/rmw_implementation src/rmw_implementation
    git clone -q -b jazzy https://github.com/ros2/rcl_logging src/rcl_logging
    git clone -q -b jazzy https://github.com/ros2/ros2_tracing src/ros2_tracing
    git clone -q -b jazzy https://github.com/micro-ROS/micro_ros_utilities src/micro_ros_utilities
    git clone -q -b jazzy https://github.com/ros2/rosidl_core src/rosidl_core
    touch src/rosidl/rosidl_typesupport_introspection_cpp/COLCON_IGNORE 2>/dev/null || true
    touch src/rcl_logging/rcl_logging_log4cxx/COLCON_IGNORE 2>/dev/null || true
    touch src/rcl_logging/rcl_logging_spdlog/COLCON_IGNORE 2>/dev/null || true
    touch src/rclc/rclc_examples/COLCON_IGNORE 2>/dev/null || true
    touch src/rcl/rcl_yaml_param_parser/COLCON_IGNORE 2>/dev/null || true
    touch src/ros2_tracing/test_tracetools/COLCON_IGNORE 2>/dev/null || true
    touch src/ros2_tracing/lttngpy/COLCON_IGNORE 2>/dev/null || true
)
for d in micro_ros_src/src/*/; do
    printf '%s %s\n' "$(git -C "$d" rev-parse HEAD)" "$(basename "$d")"
done > commits.txt
fi

echo "=== 3/3 libmicroros para x86_64 (UDP)"
cd micro_ros_src
set +u
# shellcheck disable=SC1091
. ../micro_ros_dev/install/local_setup.sh
set -u
colcon build \
    --merge-install \
    --packages-ignore-regex='.*_cpp' \
    --metas "$META" \
    --cmake-args \
    "--no-warn-unused-cli" \
    -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=OFF \
    -DTHIRDPARTY=ON \
    -DBUILD_SHARED_LIBS=OFF \
    -DBUILD_TESTING=OFF \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_C_STANDARD=17 \
    -DUCLIENT_C_STANDARD=17

# una sola libmicroros.a con todos los .a (como hace libmicroros.mk)
INST="$(pwd)/install"
rm -rf libmicroros
mkdir -p libmicroros
cd libmicroros
for f in $(find "$INST/lib" -name '*.a'); do
    n=$(basename "$f" .a)
    mkdir -p "$n"
    (cd "$n" && ar x "$f" && for o in *; do mv "$o" "../$n-$o"; done)
    rm -rf "$n"
done
ar rc -s libmicroros.a ./*.obj ./*.o 2>/dev/null || ar rc -s libmicroros.a ./*
cp libmicroros.a "$SALIDA/libmicroros.a"
cp -R "$INST/include" "$SALIDA/include"
echo "listo: $SALIDA/libmicroros.a ($(du -h "$SALIDA/libmicroros.a" | cut -f1))"
