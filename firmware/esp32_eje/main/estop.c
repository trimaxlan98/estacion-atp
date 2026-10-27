/* Parada de emergencia por GPIO.
 *
 * Contacto normalmente CERRADO a masa con pull-up: abierto (o cable cortado)
 * = parada. La ISR baja EN del puente en el mismo flanco (microsegundos),
 * sin esperar al lazo de 1 ms, y deja un enclavamiento que solo se quita al
 * armar de nuevo con el contacto ya cerrado. El lazo ademas lee el nivel en
 * cada tick (eje_lazo.c: falla EJE_F_ESTOP).
 *
 * Esto es una proteccion de SOFTWARE: no sustituye al corte de potencia por
 * contactor del paro real (README, "Seguridad").
 */
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "perifericos.h"

static const char *TAG = "estop";
static volatile uint32_t s_disparo;

#ifdef CONFIG_ATP_ESTOP_INVERTIDO
#define NIVEL_ABIERTO 0
#else
#define NIVEL_ABIERTO 1
#endif

static void IRAM_ATTR isr_estop(void *arg)
{
    (void)arg;
    if (gpio_get_level(CONFIG_ATP_ESTOP_GPIO) == NIVEL_ABIERTO) {
        motor_apagar_rapido();
        s_disparo = 1;
    }
}

int estop_iniciar(void)
{
    gpio_config_t c = {
        .pin_bit_mask = 1ULL << CONFIG_ATP_ESTOP_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = NIVEL_ABIERTO ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = NIVEL_ABIERTO ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&c));
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_IRAM));
    ESP_ERROR_CHECK(gpio_isr_handler_add(CONFIG_ATP_ESTOP_GPIO, isr_estop, NULL));
    ESP_LOGI(TAG, "paro de emergencia en GPIO%d (%s)", CONFIG_ATP_ESTOP_GPIO,
             NIVEL_ABIERTO ? "NC a masa: abierto = paro" : "NA: cerrado = paro");
    /* si arranca ya abierto, queda enclavado */
    if (gpio_get_level(CONFIG_ATP_ESTOP_GPIO) == NIVEL_ABIERTO) s_disparo = 1;
    return 0;
}

bool estop_abierto(void)
{
    return s_disparo || gpio_get_level(CONFIG_ATP_ESTOP_GPIO) == NIVEL_ABIERTO;
}

void estop_limpiar(void)
{
    if (gpio_get_level(CONFIG_ATP_ESTOP_GPIO) != NIVEL_ABIERTO) s_disparo = 0;
}
