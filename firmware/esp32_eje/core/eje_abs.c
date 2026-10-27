#include "eje_abs.h"

void eje_abs_iniciar(eje_abs_t *a)
{
    a->vueltas = 0;
    a->prev = 0;
    a->tiene_prev = false;
    a->err_paridad = 0;
    a->err_bandera = 0;
}

bool eje_abs_paridad_ok(uint16_t trama)
{
    trama ^= trama >> 8;
    trama ^= trama >> 4;
    trama ^= trama >> 2;
    trama ^= trama >> 1;
    return (trama & 1u) == 0u;     /* paridad par: la suma de bits es par */
}

int eje_abs_decodificar(eje_abs_t *a, uint16_t trama, int32_t *cuentas)
{
    if (!eje_abs_paridad_ok(trama)) { a->err_paridad++; return -1; }
    if (trama & 0x4000u)            { a->err_bandera++; return -2; }
    const uint16_t raw = trama & 0x3FFFu;
    if (a->tiene_prev) {
        /* el salto mas corto entre dos lecturas: pasar de 16383 a 0 es +1 cuenta */
        const int32_t d = (int32_t)raw - (int32_t)a->prev;
        if (d < -(EJE_ABS_CUENTAS / 2)) a->vueltas++;
        else if (d > (EJE_ABS_CUENTAS / 2)) a->vueltas--;
    }
    a->prev = raw;
    a->tiene_prev = true;
    *cuentas = a->vueltas * EJE_ABS_CUENTAS + (int32_t)raw;
    return 0;
}
