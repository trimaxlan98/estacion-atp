/* Tarea del lazo: 1 kHz, prioridad fija, nucleo 1.
 *
 * Presupuesto de un tick (1000 us a 240 MHz = 240000 ciclos), ver README:
 *   leer encoder (PCNT: un registro; SPI: ~16 us a 1 MHz)  ~ 2-20 us
 *   eje_paso (PID + estados + limites, float)               ~ unos 10 us (estimado)
 *   motor_aplicar (LEDC: 2 x set/update duty)               ~ 5-10 us
 *   publicar telemetria (seccion critica de 1 copia)        ~ 1-2 us
 *   corriente (ADC1 oneshot, cada 10 ticks)                 ~ 40-60 us cada 10 ms
 * La duracion REAL de cada tick se mide (esp_timer) y se publica en
 * ~/diag (t_lazo_us, t_lazo_max_us, jitter_max_us): lo estimado arriba
 * hay que confirmarlo en la placa.
 */
#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "config_eje.h"
#include "eje_compartido.h"
#include "eje_lazo.h"
#include "perifericos.h"
#include "tarea_lazo.h"

#ifdef CONFIG_ATP_MODO_SIMULADO
#include "eje_planta.h"
#endif

static const char *TAG = "lazo";
static eje_t s_eje[N_EJES];
#ifdef CONFIG_ATP_MODO_SIMULADO
static eje_planta_t s_planta[N_EJES];
#endif

static void apagar_al_reiniciar(void)
{
    motor_apagar_rapido();       /* esp_restart() y los panics llaman a esto antes de reiniciar */
}

static void tarea(void *arg)
{
    (void)arg;
    eje_cfg_t cfg[N_EJES];
    for (int k = 0; k < N_EJES; k++) {
        config_eje_cargar(k, &cfg[k]);
        eje_iniciar(&s_eje[k], &cfg[k]);
        if (s_eje[k].estado == EJE_FALLA) ESP_LOGE(TAG, "eje %d: configuracion invalida: no se armara", k);
#ifdef CONFIG_ATP_MODO_SIMULADO
        eje_planta_iniciar(&s_planta[k], 2.0, 0.5, 0.0, 0.0);   /* J_EJE, B_EJE de atp_ros/model.py */
#endif
#ifdef CONFIG_ATP_AUTOARMAR
        eje_armar(&s_eje[k], true);
#endif
    }
#ifdef CONFIG_ATP_MODO_SIMULADO
    ESP_LOGW(TAG, "MODO SIMULADO: el encoder y el motor NO se usan");
#endif
#ifdef CONFIG_ATP_AUTOARMAR
    ESP_LOGW(TAG, "armado automatico al arrancar (ATP_AUTOARMAR)");
#endif

    /* el lazo reinicia el chip si se cuelga mas de 100 ms */
    const esp_task_wdt_config_t wdt = { .timeout_ms = 100, .idle_core_mask = 0, .trigger_panic = true };
    ESP_ERROR_CHECK(esp_task_wdt_reconfigure(&wdt));
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));

    eje_tel_t tel;
    memset(&tel, 0, sizeof tel);
    float corriente[N_EJES];
    for (int k = 0; k < N_EJES; k++) corriente[k] = NAN;
    uint32_t perdidos = 0;
    int64_t t_prev = esp_timer_get_time();
    TickType_t despertar = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&despertar, 1);                  /* 1 tick = 1 ms (CONFIG_FREERTOS_HZ=1000) */
        const int64_t t0 = esp_timer_get_time();
        const int32_t periodo = (int32_t)(t0 - t_prev);
        t_prev = t0;
        int32_t jitter = periodo - 1000;
        if (jitter < 0) jitter = -jitter;
        if (periodo > 1500) perdidos++;                  /* se paso medio periodo del plazo */

        eje_salida_t out[N_EJES];
        for (int k = 0; k < N_EJES; k++) {
            /* ordenes de ROS */
            eje_cmd_t cmd;
            if (eje_cmd_tomar(k, &cmd)) {
                if (cmd.hay_armar) {
                    if (cmd.armar) estop_limpiar();      /* solo se quita si el contacto ya cerro */
                    eje_armar(&s_eje[k], cmd.armar);
                }
#ifdef CONFIG_ATP_MODO_SIMULADO
                if (cmd.hay_origen) (void)eje_fijar_origen(&s_eje[k], cmd.origen_rad,
                                        eje_planta_cuentas(&s_planta[k], cfg[k].cuentas_por_vuelta) * cfg[k].signo_enc);
#else
                if (cmd.hay_origen) (void)eje_fijar_origen(&s_eje[k], cmd.origen_rad, encoder_leer(k));
#endif
            }
            eje_ref_t ref;
            if (eje_ref_tomar(k, &ref)) eje_ref_nueva(&s_eje[k], ref.pos, ref.vel, ref.dt_muestras_s);

            /* entradas */
            eje_entrada_t in;
#ifdef CONFIG_ATP_MODO_SIMULADO
            in.cuentas = eje_planta_cuentas(&s_planta[k], cfg[k].cuentas_por_vuelta) * cfg[k].signo_enc;
#else
            in.cuentas = encoder_leer(k);
#endif
            in.estop = estop_abierto();
#ifdef CONFIG_ATP_CORR_ADC
            /* 100 Hz por eje; el eje 1 se lee 5 ticks despues para no sumar dos lecturas (~50 us cada una) en el mismo tick */
            if (((s_eje[k].tick + 5u * (uint32_t)k) % 10u) == 0u) corriente[k] = corriente_leer_a(k);
#endif
            in.corriente_a = corriente[k];

            /* lazo */
            eje_paso(&s_eje[k], &in, &out[k]);

            /* salidas */
#ifdef CONFIG_ATP_MODO_SIMULADO
            const double u = out[k].habilitar ? (double)out[k].duty * cfg[k].u_pleno * cfg[k].signo_motor : 0.0;
            eje_planta_paso(&s_planta[k], u, 0.0, (double)cfg[k].dt);
#else
            motor_aplicar(k, out[k].habilitar, out[k].duty);
#endif
        }
        /* LED: el peor estado de los ejes (FALLA > RETENCION > ACTIVO > ESPERA > DESARMADO) */
        int peor = 0;
        for (int k = 0; k < N_EJES; k++) if ((int)s_eje[k].estado > peor) peor = (int)s_eje[k].estado;
        led_estado(peor, s_eje[0].tick);

        /* telemetria: el tiempo del lazo es el de todo el tick (todos los ejes) */
        const uint32_t t_lazo_us = (uint32_t)(esp_timer_get_time() - t0);
        for (int k = 0; k < N_EJES; k++) {
            tel.pos = s_eje[k].pos;
            tel.vel = s_eje[k].vel;
            tel.u = s_eje[k].u;
            tel.duty = out[k].duty;
            tel.corriente_a = corriente[k];
            tel.ref_cmd = s_eje[k].ref_cmd;
            tel.ref_edad_s = s_eje[k].ref_edad_s;
            tel.estado = (uint32_t)s_eje[k].estado;
            tel.fallas = s_eje[k].fallas;
            tel.ticks = s_eje[k].tick;
            tel.t_lazo_us = t_lazo_us;
            tel.t_lazo_max_us = tel.t_lazo_us;           /* eje_tel_publicar lleva el maximo corrido */
            tel.jitter_max_us = jitter;
            tel.ticks_perdidos = perdidos;
            tel.abs_err = encoder_errores(k);
            eje_tel_publicar(k, &tel);
        }

        esp_task_wdt_reset();
    }
}

void tarea_lazo_iniciar(void)
{
    ESP_ERROR_CHECK(esp_register_shutdown_handler(apagar_al_reiniciar));
    /* nucleo 1 (APP_CPU): Wi-Fi y lwIP viven en el 0 */
    xTaskCreatePinnedToCore(tarea, "lazo_1khz", 4096, NULL, configMAX_PRIORITIES - 2, NULL, 1);
}
