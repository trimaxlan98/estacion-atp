/* Encoder del eje: incremental en cuadratura (PCNT) o absoluto SPI (AS5048A).
 * Se elige en menuconfig (ATP_ENCODER). */
#include "config_eje.h"
#include "perifericos.h"

#include "sdkconfig.h"
#include "esp_log.h"

static const char *TAG __attribute__((unused)) = "encoder";

#if defined(CONFIG_ATP_ENC_PCNT)
/* ------------------------------------------------------------------------
 * Cuadratura x4 con el periferico PCNT del ESP32 (patron del ejemplo oficial
 * de ESP-IDF "rotary_encoder"). Dos canales por unidad: cada flanco de A y de
 * B cuenta +-1 segun el nivel del otro -> 4 cuentas por periodo de A.
 * `accum_count` suma los desbordes de los 16 bits del contador por hardware
 * (watch points en los limites): la cuenta es un int32 continuo.
 * ---------------------------------------------------------------------- */
#include "driver/pulse_cnt.h"

#define PCNT_LIM 30000     /* los limites del contador de 16 bits son +-32767 */

static pcnt_unit_handle_t s_unit[N_EJES];

static int pcnt_iniciar_eje(int e)
{
    const pines_eje_t *p = pines_eje(e);
    pcnt_unit_handle_t *unidad = &s_unit[e];
    pcnt_unit_config_t uc = {
        .low_limit = -PCNT_LIM,
        .high_limit = PCNT_LIM,
        .flags.accum_count = true,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&uc, unidad));

    if (CONFIG_ATP_ENC_FILTRO_NS > 0) {
        pcnt_glitch_filter_config_t fc = { .max_glitch_ns = CONFIG_ATP_ENC_FILTRO_NS };
        ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(*unidad, &fc));
    }

    pcnt_channel_handle_t ca, cb;
    pcnt_chan_config_t cfg_a = {
        .edge_gpio_num = p->enc_a,
        .level_gpio_num = p->enc_b,
    };
    pcnt_chan_config_t cfg_b = {
        .edge_gpio_num = p->enc_b,
        .level_gpio_num = p->enc_a,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(*unidad, &cfg_a, &ca));
    ESP_ERROR_CHECK(pcnt_new_channel(*unidad, &cfg_b, &cb));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ca, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
                                                 PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(ca, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(cb, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                 PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(cb, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                  PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(*unidad, PCNT_LIM));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(*unidad, -PCNT_LIM));
    ESP_ERROR_CHECK(pcnt_unit_enable(*unidad));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(*unidad));
    ESP_ERROR_CHECK(pcnt_unit_start(*unidad));
    ESP_LOGI(TAG, "eje %d: PCNT en cuadratura x4: A=GPIO%d B=GPIO%d, filtro %d ns", e, p->enc_a, p->enc_b,
             CONFIG_ATP_ENC_FILTRO_NS);
    return 0;
}

int encoder_iniciar(void)
{
    for (int e = 0; e < N_EJES; e++) pcnt_iniciar_eje(e);
    ESP_LOGI(TAG, "%d eje(s); %d cuentas/vuelta de carga en el eje 0", N_EJES, CONFIG_ATP_ENC_CPR);
    return 0;
}

int32_t encoder_leer(int e)
{
    int c = 0;
    (void)pcnt_unit_get_count(s_unit[e], &c);
    return (int32_t)c;
}

uint32_t encoder_errores(int e) { (void)e; return 0; }

#elif defined(CONFIG_ATP_ENC_SPI)
/* ------------------------------------------------------------------------
 * Absoluto AS5048A por SPI, modo 1, lectura encadenada (core/eje_abs.h
 * documenta la trama). Una transaccion de 16 bits por tick del lazo: a
 * 1 MHz son 16 us de reloj. SIN PROBAR en hardware.
 * ---------------------------------------------------------------------- */
#include "driver/spi_master.h"
#include "eje_abs.h"

static spi_device_handle_t s_dev;
static eje_abs_t s_abs;
static int32_t s_cuentas;

int encoder_iniciar(void)
{
    spi_bus_config_t bus = {
        .sclk_io_num = CONFIG_ATP_SPI_SCLK_GPIO,
        .mosi_io_num = CONFIG_ATP_SPI_MOSI_GPIO,
        .miso_io_num = CONFIG_ATP_SPI_MISO_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_DISABLED));
    spi_device_interface_config_t dev = {
        .clock_speed_hz = CONFIG_ATP_SPI_KHZ * 1000,
        .mode = 1,                                  /* CPOL=0, CPHA=1 */
        .spics_io_num = CONFIG_ATP_SPI_CS_GPIO,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev, &s_dev));
    eje_abs_iniciar(&s_abs);
    /* dos lecturas para llenar la cadena y fijar la primera posicion */
    (void)encoder_leer(0);
    (void)encoder_leer(0);
    ESP_LOGI(TAG, "AS5048A por SPI: SCLK=%d MOSI=%d MISO=%d CS=%d, %d kHz",
             CONFIG_ATP_SPI_SCLK_GPIO, CONFIG_ATP_SPI_MOSI_GPIO, CONFIG_ATP_SPI_MISO_GPIO,
             CONFIG_ATP_SPI_CS_GPIO, CONFIG_ATP_SPI_KHZ);
    return 0;
}

int32_t encoder_leer(int e)
{
    (void)e;                                       /* el SPI es de un solo eje (ATP_EJES=1) */
    spi_transaction_t t = {
        .length = 16,
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .tx_data = { 0xFF, 0xFF },                  /* orden "leer angulo" = 0xFFFF */
    };
    if (spi_device_polling_transmit(s_dev, &t) == ESP_OK) {
        const uint16_t trama = (uint16_t)((t.rx_data[0] << 8) | t.rx_data[1]);
        int32_t c;
        if (eje_abs_decodificar(&s_abs, trama, &c) == 0) s_cuentas = c;
        /* en error se conserva la ultima cuenta buena y se cuenta el fallo */
    }
    return s_cuentas;
}

uint32_t encoder_errores(int e) { (void)e; return s_abs.err_paridad + s_abs.err_bandera; }

#else
#error "Elegir el tipo de encoder en menuconfig (ATP_ENCODER)"
#endif
