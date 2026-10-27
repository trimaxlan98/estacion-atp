/* Encoder absoluto de 14 bits: paridad, banderas y desenrollado multivuelta. */
#include <stdio.h>

#include "../core/eje_abs.h"
#include "minitest.h"

/* arma una trama valida: dato de 14 bits, bandera de error y paridad par */
static uint16_t trama(uint16_t dato, int err)
{
    uint16_t t = (uint16_t)((dato & 0x3FFF) | (err ? 0x4000 : 0));
    int unos = __builtin_popcount(t);
    if (unos & 1) t |= 0x8000;
    return t;
}

static void test_paridad(void)
{
    CHECK(eje_abs_paridad_ok(0x0000));
    CHECK(!eje_abs_paridad_ok(0x0001));
    CHECK(eje_abs_paridad_ok(0x0003));
    CHECK(eje_abs_paridad_ok(0xFFFF));              /* 16 unos: par */
    CHECK(!eje_abs_paridad_ok(0x7FFF));             /* 15 unos: impar */
    for (int d = 0; d < 16384; d += 37) CHECK(eje_abs_paridad_ok(trama((uint16_t)d, 0)));
    /* un solo bit cambiado siempre se detecta */
    for (int bit = 0; bit < 16; bit++)
        CHECK(!eje_abs_paridad_ok((uint16_t)(trama(1234, 0) ^ (1u << bit))));
}

static void test_decodificar_basico(void)
{
    eje_abs_t a;
    eje_abs_iniciar(&a);
    int32_t c = -1;
    CHECK(eje_abs_decodificar(&a, trama(8192, 0), &c) == 0);
    CHECK(c == 8192);                               /* 180 grados */
    CHECK(eje_abs_decodificar(&a, trama(8200, 0), &c) == 0);
    CHECK(c == 8200);
}

static void test_desenrollado_adelante_y_atras(void)
{
    eje_abs_t a;
    eje_abs_iniciar(&a);
    int32_t c = 0;
    eje_abs_decodificar(&a, trama(16000, 0), &c);
    eje_abs_decodificar(&a, trama(16380, 0), &c);
    CHECK(c == 16380);
    eje_abs_decodificar(&a, trama(100, 0), &c);     /* paso por el cero hacia adelante */
    CHECK(c == 16384 + 100);
    eje_abs_decodificar(&a, trama(16380, 0), &c);   /* y de vuelta */
    CHECK(c == 16380);
    eje_abs_decodificar(&a, trama(16000, 0), &c);
    CHECK(c == 16000);
    /* una vuelta entera hacia adelante a pasos de 1000 cuentas */
    eje_abs_iniciar(&a);
    int32_t esperado = 0;
    for (int i = 0; i < 40; i++) {
        esperado += 1000;
        eje_abs_decodificar(&a, trama((uint16_t)(esperado % 16384), 0), &c);
    }
    CHECK(c == 40000);                              /* 2.44 vueltas sin perder la cuenta */
}

static void test_errores_no_tocan_el_estado(void)
{
    eje_abs_t a;
    eje_abs_iniciar(&a);
    int32_t c = 0;
    eje_abs_decodificar(&a, trama(5000, 0), &c);
    CHECK(eje_abs_decodificar(&a, (uint16_t)(trama(5001, 0) ^ 0x0004), &c) == -1);   /* paridad mala */
    CHECK(eje_abs_decodificar(&a, trama(5001, 1), &c) == -2);                        /* bandera de error */
    CHECK(a.err_paridad == 1 && a.err_bandera == 1);
    CHECK(eje_abs_decodificar(&a, trama(5002, 0), &c) == 0);
    CHECK(c == 5002);
}

int main(void)
{
    RUN(test_paridad);
    RUN(test_decodificar_basico);
    RUN(test_desenrollado_adelante_y_atras);
    RUN(test_errores_no_tocan_el_estado);
    MT_FIN();
}
