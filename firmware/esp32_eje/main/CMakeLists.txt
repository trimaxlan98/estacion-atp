# La logica del lazo (core/) y el nodo micro-ROS (uros/) son C puro / rclc y se
# comparten con el host (pruebas y nodo sobre Linux); aqui solo se enlazan.
set(RAIZ ${CMAKE_CURRENT_LIST_DIR}/..)

idf_component_register(
    SRCS main.c tarea_lazo.c config_eje.c compartido_esp.c transporte.c
         encoder.c motor.c estop.c auxiliares.c
         ${RAIZ}/core/eje_lazo.c ${RAIZ}/core/eje_abs.c ${RAIZ}/core/eje_planta.c
         ${RAIZ}/uros/eje_uros.c
    INCLUDE_DIRS "." ${RAIZ}/core ${RAIZ}/uros
    REQUIRES micro_ros_espidf_component driver esp_timer esp_adc esp_wifi nvs_flash lwip esp_hw_support hal soc)
