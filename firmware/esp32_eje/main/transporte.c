/* Transporte de micro-ROS: serie (UART) o UDP por Wi-Fi.
 *
 * Las dos variantes usan el transporte PERSONALIZADO de XRCE-DDS (la libreria
 * se compila con RMW_UXRCE_TRANSPORT=custom, app-colcon.meta), de modo que el
 * mismo binario-fuente sirve para ambos y se elige en menuconfig. */
#include <string.h>

#include "esp_log.h"
#include "sdkconfig.h"

#include <uxr/client/transport.h>
#include <rmw_microros/rmw_microros.h>

#include "transporte.h"

static const char *TAG = "transporte";

#if defined(CONFIG_MICRO_ROS_ESP_UART_TRANSPORT)
/* ------------------------------------------------------------------------
 * Serie: tramas HDLC de XRCE (framing = true). El agente: -b <baudios>.
 * ---------------------------------------------------------------------- */
#include "driver/uart.h"

#define UART_RX_BUF 4096     /* a 921600 baudios son 4.4 ms de datos si la tarea tarda */

static bool serie_abrir(struct uxrCustomTransport *t)
{
    const uart_port_t port = *(const uart_port_t *)t->args;
    const uart_config_t uc = {
        .baud_rate = CONFIG_ATP_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    if (uart_param_config(port, &uc) != ESP_OK) return false;
    if (uart_set_pin(port, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) return false;
    if (uart_is_driver_installed(port)) return true;     /* reconexion: ya esta */
    return uart_driver_install(port, UART_RX_BUF, 0, 0, NULL, 0) == ESP_OK;
}

static bool serie_cerrar(struct uxrCustomTransport *t)
{
    (void)t;
    return true;     /* el driver se deja instalado: la reconexion es mas rapida */
}

static size_t serie_escribir(struct uxrCustomTransport *t, const uint8_t *buf, size_t len, uint8_t *err)
{
    const uart_port_t port = *(const uart_port_t *)t->args;
    const int n = uart_write_bytes(port, (const char *)buf, len);
    if (n < 0) { *err = 1; return 0; }
    return (size_t)n;
}

static size_t serie_leer(struct uxrCustomTransport *t, uint8_t *buf, size_t len, int timeout, uint8_t *err)
{
    const uart_port_t port = *(const uart_port_t *)t->args;
    const int n = uart_read_bytes(port, buf, len, pdMS_TO_TICKS(timeout < 0 ? 0 : timeout));
    if (n < 0) { *err = 1; return 0; }
    return (size_t)n;
}

static uart_port_t s_puerto = CONFIG_ATP_UART_NUM;

void transporte_configurar(void)
{
    ESP_LOGI(TAG, "serie: UART%d a %d baudios", CONFIG_ATP_UART_NUM, CONFIG_ATP_UART_BAUD);
    rmw_uros_set_custom_transport(true, &s_puerto, serie_abrir, serie_cerrar,
                                  serie_escribir, serie_leer);
}

#else
/* ------------------------------------------------------------------------
 * UDP por Wi-Fi: un datagrama por mensaje XRCE, sin trama HDLC (framing = false).
 * El agente: udp4 --port <puerto>. Direccion y puerto: menuconfig.
 * ---------------------------------------------------------------------- */
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "esp_wifi.h"
#include "uros_network_interfaces.h"

static int s_sock = -1;

static bool udp_abrir(struct uxrCustomTransport *t)
{
    (void)t;
    struct sockaddr_in agente = { 0 };
    agente.sin_family = AF_INET;
    agente.sin_port = htons((uint16_t)atoi(CONFIG_MICRO_ROS_AGENT_PORT));
    if (inet_pton(AF_INET, CONFIG_MICRO_ROS_AGENT_IP, &agente.sin_addr) != 1) return false;
    s_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s_sock < 0) return false;
    if (connect(s_sock, (struct sockaddr *)&agente, sizeof agente) != 0) {
        close(s_sock);
        s_sock = -1;
        return false;
    }
    return true;
}

static bool udp_cerrar(struct uxrCustomTransport *t)
{
    (void)t;
    if (s_sock >= 0) close(s_sock);
    s_sock = -1;
    return true;
}

static size_t udp_escribir(struct uxrCustomTransport *t, const uint8_t *buf, size_t len, uint8_t *err)
{
    (void)t;
    const ssize_t n = send(s_sock, buf, len, 0);
    if (n < 0) { *err = 1; return 0; }
    return (size_t)n;
}

static size_t udp_leer(struct uxrCustomTransport *t, uint8_t *buf, size_t len, int timeout, uint8_t *err)
{
    (void)t;
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(s_sock, &fds);
    struct timeval tv = { .tv_sec = timeout / 1000, .tv_usec = (timeout % 1000) * 1000 };
    const int r = select(s_sock + 1, &fds, NULL, NULL, timeout < 0 ? NULL : &tv);
    if (r <= 0) return 0;                          /* tiempo agotado: sin datos, sin error */
    const ssize_t n = recv(s_sock, buf, len, 0);
    if (n < 0) { *err = 1; return 0; }
    return (size_t)n;
}

void transporte_configurar(void)
{
    ESP_ERROR_CHECK(uros_network_interface_initialize());   /* NVS + Wi-Fi STA; espera la IP */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));          /* sin ahorro de energia: menos jitter */
    ESP_LOGI(TAG, "UDP hacia el agente %s:%s", CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT);
    rmw_uros_set_custom_transport(false, NULL, udp_abrir, udp_cerrar, udp_escribir, udp_leer);
}
#endif
