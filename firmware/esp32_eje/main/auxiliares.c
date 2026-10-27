/* Sensor de corriente (ADC1) y LED de estado. */
#include <math.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "eje_lazo.h"
#include "config_eje.h"
#include "perifericos.h"

#ifdef CONFIG_ATP_CORR_ADC
#include "esp_adc/adc_oneshot.h"

static const char *TAG = "corriente";
static adc_oneshot_unit_handle_t s_adc;
static adc_channel_t s_ch_r[N_EJES], s_ch_l[N_EJES];
static float s_filtrada[N_EJES];

int corriente_iniciar(void)
{
    adc_oneshot_unit_init_cfg_t ic = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&ic, &s_adc));
    for (int e = 0; e < N_EJES; e++) {
        const pines_eje_t *p = pines_eje(e);
        adc_unit_t u1, u2;
        ESP_ERROR_CHECK(adc_oneshot_io_to_channel(p->corr_r, &u1, &s_ch_r[e]));
        ESP_ERROR_CHECK(adc_oneshot_io_to_channel(p->corr_l, &u2, &s_ch_l[e]));
        /* ADC2 comparte recursos con el Wi-Fi: solo ADC1 */
        if (u1 != ADC_UNIT_1 || u2 != ADC_UNIT_1) {
            ESP_LOGE(TAG, "eje %d: R_IS y L_IS deben estar en pines ADC1 (GPIO32-39)", e);
            return -1;
        }
        adc_oneshot_chan_cfg_t cc = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_12 };
        ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, s_ch_r[e], &cc));
        ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, s_ch_l[e], &cc));
        ESP_LOGI(TAG, "eje %d ADC1: R_IS=GPIO%d L_IS=GPIO%d, %d mA/V (SIN calibrar: ver README)",
                 e, p->corr_r, p->corr_l, CONFIG_ATP_CORR_MA_POR_V);
    }
    return 0;
}

float corriente_leer_a(int e)
{
    int r = 0, l = 0;
    if (adc_oneshot_read(s_adc, s_ch_r[e], &r) != ESP_OK) return NAN;
    if (adc_oneshot_read(s_adc, s_ch_l[e], &l) != ESP_OK) return NAN;
    /* con 12 dB el ADC1 mide ~0..3.1 V sobre 4095 cuentas (valor nominal; sin
     * esp_adc_cali, error de varios %). Solo conduce una mitad: la mayor manda. */
    const float v = (float)(r > l ? r : l) * (3.1f / 4095.0f);
    const float i = v * (float)CONFIG_ATP_CORR_MA_POR_V * 1.0e-3f;
    s_filtrada[e] += 0.3f * (i - s_filtrada[e]);     /* ~3 muestras a 100 Hz */
    return s_filtrada[e];
}
#else
int corriente_iniciar(void) { return 0; }
float corriente_leer_a(int e) { (void)e; return NAN; }
#endif

/* ---- LED: DESARMADO lento, ESPERA doble, ACTIVO fijo, RETENCION rapido, FALLA muy rapido ---- */
#define LED_GPIO 2

void led_iniciar(void)
{
    gpio_config_t c = { .pin_bit_mask = 1ULL << LED_GPIO, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&c);
    gpio_set_level(LED_GPIO, 0);
}

void led_estado(int estado, uint32_t tick)
{
    int on = 0;
    switch (estado) {
    case EJE_DESARMADO: on = (tick / 500u) % 2u == 0u; break;                 /* 1 Hz */
    case EJE_ESPERA:    on = ((tick / 100u) % 10u) < 2u; break;               /* destello cada 1 s */
    case EJE_ACTIVO:    on = 1; break;
    case EJE_RETENCION: on = (tick / 125u) % 2u == 0u; break;                 /* 4 Hz */
    default:            on = (tick / 50u) % 2u == 0u; break;                  /* 10 Hz: falla */
    }
    gpio_set_level(LED_GPIO, on);
}
