/* Puente H con LEDC: BTS7960 (RPWM + LPWM + EN) o PWM + DIR + EN. */
#include <math.h>

#include "driver/gpio.h"
#include "esp_attr.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "hal/gpio_ll.h"
#include "sdkconfig.h"
#include "soc/gpio_struct.h"

#include "config_eje.h"
#include "perifericos.h"

static const char *TAG = "motor";

#define PWM_RES LEDC_TIMER_10_BIT           /* 1023 pasos: 80 MHz / 20 kHz = 4000 ciclos, caben 11 bits */
#define PWM_MAX 1023
#if SOC_LEDC_SUPPORT_HS_MODE
#define PWM_MODO LEDC_HIGH_SPEED_MODE       /* el modo rapido del ESP32 cambia el ciclo sin glitches */
#else
#define PWM_MODO LEDC_LOW_SPEED_MODE
#endif

static void ch_config(ledc_channel_t ch, int gpio)
{
    ledc_channel_config_t c = {
        .gpio_num = gpio,
        .speed_mode = PWM_MODO,
        .channel = ch,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .intr_type = LEDC_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&c));
}

static inline void ch_duty(ledc_channel_t ch, uint32_t d)
{
    ledc_set_duty(PWM_MODO, ch, d);
    ledc_update_duty(PWM_MODO, ch);
}

/* GPIO de EN de cada eje, en DRAM (no `const`): la ISR del paro la lee con la cache de flash
 * posiblemente deshabilitada; un const iria a la flash. */
static int s_en_gpio[N_EJES];

static ledc_channel_t canal(int e, int lado) { return (ledc_channel_t)(2 * e + lado); }

int motor_iniciar(void)
{
    /* EN primero y en bajo, de TODOS los ejes: el puente nace apagado */
    for (int e = 0; e < N_EJES; e++) {
        s_en_gpio[e] = pines_eje(e)->en;
        gpio_config_t en = {
            .pin_bit_mask = 1ULL << s_en_gpio[e],
            .mode = GPIO_MODE_OUTPUT,
        };
        ESP_ERROR_CHECK(gpio_config(&en));
        gpio_set_level(s_en_gpio[e], 0);
    }

    ledc_timer_config_t t = {
        .speed_mode = PWM_MODO,
        .duty_resolution = PWM_RES,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = CONFIG_ATP_PWM_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&t));
    for (int e = 0; e < N_EJES; e++) {
        const pines_eje_t *p = pines_eje(e);
        ch_config(canal(e, 0), p->pwm_a);
#if defined(CONFIG_ATP_DRV_BTS7960)
        ch_config(canal(e, 1), p->pwm_b);
        ESP_LOGI(TAG, "eje %d BTS7960: RPWM=GPIO%d LPWM=GPIO%d EN=GPIO%d, %d Hz, 10 bits",
                 e, p->pwm_a, p->pwm_b, p->en, CONFIG_ATP_PWM_HZ);
#else
        gpio_config_t dir = {
            .pin_bit_mask = 1ULL << p->pwm_b,
            .mode = GPIO_MODE_OUTPUT,
        };
        ESP_ERROR_CHECK(gpio_config(&dir));
        ESP_LOGI(TAG, "eje %d PWM+DIR: PWM=GPIO%d DIR=GPIO%d EN=GPIO%d, %d Hz, 10 bits",
                 e, p->pwm_a, p->pwm_b, p->en, CONFIG_ATP_PWM_HZ);
#endif
        motor_aplicar(e, false, 0.0f);
    }
    return 0;
}

void IRAM_ATTR motor_apagar_rapido(void)
{
    /* seguro desde ISR: un registro de GPIO por eje, sin llamadas a la flash; el paro es COMUN */
    for (int e = 0; e < N_EJES; e++) gpio_ll_set_level(&GPIO, s_en_gpio[e], 0);
}

void motor_aplicar(int e, bool habilitar, float duty)
{
    const pines_eje_t *p = pines_eje(e);
    if (!habilitar) {
        gpio_ll_set_level(&GPIO, p->en, 0);          /* primero el puente de ESTE eje, luego el PWM */
        ch_duty(canal(e, 0), 0);
#if defined(CONFIG_ATP_DRV_BTS7960)
        ch_duty(canal(e, 1), 0);
#endif
        return;
    }
    if (!(duty == duty)) duty = 0.0f;                /* NaN -> 0 */
    const float a = fabsf(duty) > 1.0f ? 1.0f : fabsf(duty);
    const uint32_t d = (uint32_t)(a * (float)PWM_MAX + 0.5f);
#if defined(CONFIG_ATP_DRV_BTS7960)
    if (duty >= 0.0f) { ch_duty(canal(e, 0), d); ch_duty(canal(e, 1), 0); }
    else              { ch_duty(canal(e, 0), 0); ch_duty(canal(e, 1), d); }
#else
    gpio_set_level(p->pwm_b, duty >= 0.0f ? 1 : 0);
    ch_duty(canal(e, 0), d);
#endif
    gpio_set_level(p->en, 1);                        /* habilita al final: ya con el ciclo puesto */
}
