/* Firmware del eje de la montura (ESP32 + micro-ROS Jazzy). Ver README.md.
 *
 * Arranque, en este orden y por seguridad:
 *   1. el puente H se deja APAGADO (EN bajo) antes que nada;
 *   2. paro de emergencia y encoder;
 *   3. tarea del lazo (1 kHz, nucleo 1, prioridad alta);
 *   4. micro-ROS (nucleo 0, prioridad baja): si no hay agente, el lazo sigue
 *      vivo y el watchdog de referencia protege al motor.
 */
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "config_eje.h"
#include "eje_uros.h"
#include "perifericos.h"
#include "tarea_lazo.h"
#include "transporte.h"

static const char *TAG = "atp_eje";

static uint32_t ahora_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }
static void dormir_ms(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

static void tarea_uros(void *arg)
{
    (void)arg;
    const eje_uros_cfg_t cfg = {
        .n_ejes = N_EJES,
#if N_EJES == 1
        .eje_nombre = { CONFIG_ATP_EJE_NOMBRE },
        .eje_indice = { CONFIG_ATP_EJE_INDICE },
#else
        .eje_nombre = { "az", "el" },       /* banco de dos ejes: siempre az (0) y el (1) */
        .eje_indice = { 0, 1 },
#endif
#ifdef CONFIG_ATP_ECO_LATENCIA
        .eco = true,
#endif
        .domain_id = CONFIG_ATP_ROS_DOMAIN_ID,
        .tel_hz = CONFIG_ATP_TEL_HZ,
        .diag_hz = CONFIG_ATP_DIAG_HZ,
        .opciones = NULL,
        .ahora_ms = ahora_ms,
        .dormir_ms = dormir_ms,
    };
    eje_uros_correr(&cfg);
    vTaskDelete(NULL);
}

void app_main(void)
{
#if N_EJES == 1
    ESP_LOGI(TAG, "eje '%s' (indice %d), dominio ROS %d", CONFIG_ATP_EJE_NOMBRE,
             CONFIG_ATP_EJE_INDICE, CONFIG_ATP_ROS_DOMAIN_ID);
#else
    ESP_LOGI(TAG, "banco de %d ejes (az, el), dominio ROS %d", N_EJES, CONFIG_ATP_ROS_DOMAIN_ID);
#endif

    motor_iniciar();                 /* 1. puente apagado */
    estop_iniciar();                 /* 2. paro y sensores */
    led_iniciar();
#ifndef CONFIG_ATP_MODO_SIMULADO
    encoder_iniciar();
#endif
    corriente_iniciar();

    tarea_lazo_iniciar();            /* 3. lazo de 1 kHz */

    transporte_configurar();         /* 4. micro-ROS */
    xTaskCreatePinnedToCore(tarea_uros, "uros", CONFIG_ATP_UROS_STACK, NULL,
                            CONFIG_ATP_UROS_PRIO, NULL, 0);
}
