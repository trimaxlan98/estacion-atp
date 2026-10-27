/* Encoder absoluto SPI de 14 bits (AS5048A) como alternativa al incremental.
 *
 * Solo la parte pura: paridad, banderas y desenrollado multivuelta. El
 * transporte SPI vive en main/encoder_spi.c.
 *
 * Formato de la trama (AS5048A, hoja de datos de ams; NO verificado contra
 * la hoja en esta rama: se reproduce de memoria y hay que contrastarlo antes
 * de cablear): 16 bits, MSB primero; bit 15 = paridad PAR de toda la trama,
 * bit 14 = bandera de error, bits 13..0 = dato. El registro de angulo es
 * 0x3FFF; la orden de lectura 0x7FFF con paridad queda 0xFFFF. La respuesta
 * a una orden llega en la trama SIGUIENTE (lectura encadenada).
 */
#ifndef EJE_ABS_H
#define EJE_ABS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EJE_ABS_BITS 14
#define EJE_ABS_CUENTAS (1 << EJE_ABS_BITS)       /* 16384 por vuelta */
#define EJE_ABS_ORDEN_ANGULO 0xFFFFu

typedef struct {
    int32_t vueltas;        /* vueltas completas acumuladas */
    uint16_t prev;          /* ultima lectura valida de 14 bits */
    bool tiene_prev;
    uint32_t err_paridad;   /* contadores de diagnostico */
    uint32_t err_bandera;
} eje_abs_t;

void eje_abs_iniciar(eje_abs_t *a);
/* paridad par de 16 bits: true si el numero de unos es par */
bool eje_abs_paridad_ok(uint16_t trama);
/* Decodifica una trama. 0 = ok y *cuentas = vueltas*16384 + angulo (multivuelta,
 * relativo a la primera lectura); -1 paridad mala; -2 bandera de error. En
 * error NO se actualiza el estado. */
int eje_abs_decodificar(eje_abs_t *a, uint16_t trama, int32_t *cuentas);

#ifdef __cplusplus
}
#endif
#endif
