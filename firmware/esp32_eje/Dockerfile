# Compilacion reproducible del firmware del eje (ESP-IDF + micro-ROS Jazzy).
#
#   docker build -t atp-esp32-idf:5.2.8 firmware/esp32_eje      (una vez; lo hace build.sh)
#
# Versiones fijadas (las dos cambian el binario, por eso se fijan):
#   - ESP-IDF v5.2.8: imagen oficial espressif/idf. Fijada tambien por digest
#     para que un re-etiquetado en Docker Hub no cambie la compilacion.
#   - micro_ros_espidf_component, rama jazzy, commit fijado (ARG de abajo). El
#     README del componente lo prueba en ESP-IDF v5.2 a v6.0.
# Lo que NO queda fijado por commit: libmicroros.mk clona ~25 repositorios de
# micro-ROS/ros2 por RAMA (-b jazzy) al compilar la libreria; build.sh guarda
# los commits que resultaron en build/microros-commits.txt para auditarlo.
FROM espressif/idf:v5.2.8@sha256:3038a53c0b6c93fea1baecf84460a0502bbf533c18ea2bf2b2d1ee32eb3d24a3

ARG MICROROS_COMPONENT_SHA=5cd71f525eedc1e59a48e3a12184ba75b60b05b7

# Dependencias de Python que pide el README del componente (dentro del entorno
# virtual de IDF) y empy 3.3.4: Jazzy usa la API de empy 3 (empy 4 la rompe).
RUN bash -c ". ${IDF_PATH}/export.sh >/dev/null 2>&1 \
    && pip install --no-cache-dir catkin_pkg lark colcon-common-extensions empy==3.3.4 numpy" \
    && git config --system --add safe.directory '*'

# El componente se deja en /opt (solo lectura util); build.sh lo copia a una
# cache escribible porque libmicroros se compila DENTRO de su carpeta.
RUN git clone --branch jazzy https://github.com/micro-ROS/micro_ros_espidf_component.git \
        /opt/micro_ros_espidf_component \
    && git -C /opt/micro_ros_espidf_component checkout ${MICROROS_COMPONENT_SHA} \
    && rm -rf /opt/micro_ros_espidf_component/.git/hooks

WORKDIR /project
