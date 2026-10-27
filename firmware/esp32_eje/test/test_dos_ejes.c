/* Dos ejes en un lazo (ATP_EJES=2): el buzon indexado de uros/eje_compartido.h (la version de
 * Linux, mutex POSIX; la del ESP32 tiene el mismo cuerpo con la seccion critica de FreeRTOS),
 * la independencia de dos eje_t que comparten tick, el paro comun, y el costo de CPU del tick
 * de 1 y de 2 ejes medido en el host. */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "../core/eje_lazo.h"
#include "../core/eje_planta.h"
#include "../uros/eje_compartido.h"
#include "minitest.h"

#define RAD(d) ((d) * M_PI / 180.0)

/* ---------------------------------------------------------------- buzones */
static void test_buzon_indexado(void)
{
    eje_ref_t r = { .pos = 1.0f, .vel = NAN, .dt_muestras_s = 0.05f }, o;
    eje_ref_enviar(1, &r);
    CHECK(!eje_ref_tomar(0, &o));            /* el eje 0 no ve la referencia del 1 */
    CHECK(eje_ref_tomar(1, &o));
    CHECK_NEAR(o.pos, 1.0, 0.0);
    CHECK(!eje_ref_tomar(1, &o));            /* se toma una vez */

    eje_cmd_armar(0, true);
    eje_cmd_origen(1, 0.5f);
    eje_cmd_t c;
    CHECK(eje_cmd_tomar(0, &c) && c.hay_armar && c.armar && !c.hay_origen);
    CHECK(eje_cmd_tomar(1, &c) && !c.hay_armar && c.hay_origen);
    CHECK_NEAR(c.origen_rad, 0.5, 0.0);
    CHECK(!eje_cmd_tomar(0, &c) && !eje_cmd_tomar(1, &c));

    /* indices fuera de rango: se ignoran sin tocar nada */
    eje_ref_enviar(-1, &r);
    eje_ref_enviar(ATP_EJES_MAX, &r);
    CHECK(!eje_ref_tomar(-1, &o) && !eje_ref_tomar(ATP_EJES_MAX, &o));
    eje_cmd_armar(7, true);
    CHECK(!eje_cmd_tomar(7, &c));
}

static void test_telemetria_maximos_por_eje(void)
{
    eje_tel_t t, l;
    memset(&t, 0, sizeof t);
    CHECK(!eje_tel_leer(0, &l));
    t.t_lazo_max_us = 30; t.jitter_max_us = 5; t.pos = 1.0f;
    eje_tel_publicar(0, &t);
    t.t_lazo_max_us = 80; t.jitter_max_us = 40; t.pos = 2.0f;
    eje_tel_publicar(1, &t);
    t.t_lazo_max_us = 10; t.jitter_max_us = 1; t.pos = 1.5f;
    eje_tel_publicar(0, &t);                 /* el maximo corrido del eje 0 sigue siendo 30 */
    CHECK(eje_tel_leer(0, &l));
    CHECK(l.t_lazo_max_us == 30 && l.jitter_max_us == 5);
    CHECK_NEAR(l.pos, 1.5, 0.0);
    CHECK(eje_tel_leer(1, &l));
    CHECK(l.t_lazo_max_us == 80 && l.jitter_max_us == 40);
    CHECK_NEAR(l.pos, 2.0, 0.0);
    t.t_lazo_max_us = 5; t.jitter_max_us = 0;
    eje_tel_publicar(0, &t);                 /* tras leer, el maximo se reinicia */
    CHECK(eje_tel_leer(0, &l) && l.t_lazo_max_us == 5);
}

/* ---------------------------------------------------------------- dos ejes en un tick */
typedef struct {
    eje_t e;
    eje_planta_t p;
    eje_entrada_t in;
    eje_salida_t out;
} carril_t;

static void carril_ini(carril_t *c, float lim_min_deg, float lim_max_deg, double theta0)
{
    eje_cfg_t cfg;
    eje_cfg_defecto(&cfg);
    cfg.cuentas_por_vuelta = 40000.0f;
    cfg.lim_min = (float)RAD(lim_min_deg);
    cfg.lim_max = (float)RAD(lim_max_deg);
    eje_iniciar(&c->e, &cfg);
    eje_planta_iniciar(&c->p, 2.0, 0.5, 0.0, theta0);
    c->in.estop = false;
    c->in.corriente_a = NAN;
    eje_armar(&c->e, true);
}

static void carril_tick(carril_t *c, bool estop)
{
    c->in.cuentas = eje_planta_cuentas(&c->p, c->e.cfg.cuentas_por_vuelta);
    c->in.estop = estop;
    eje_paso(&c->e, &c->in, &c->out);
    const double u = c->out.habilitar ? (double)c->out.duty * c->e.cfg.u_pleno : 0.0;
    eje_planta_paso(&c->p, u, 0.0, c->e.cfg.dt);
}

static double rampa_az(double t) { return RAD(10.0) + RAD(9.0) * t; }
static double rampa_el(double t) { return RAD(20.0) + RAD(0.8) * t; }

static void test_ejes_independientes_bit_a_bit(void)
{
    /* el az solo y el az junto al el recorren EXACTAMENTE la misma trayectoria */
    carril_t a1, a2, e2;
    carril_ini(&a1, -5.0f, 365.0f, RAD(10.0));
    carril_ini(&a2, -5.0f, 365.0f, RAD(10.0));
    carril_ini(&e2, 0.0f, 90.0f, RAD(20.0));
    for (long k = 0; k < 20000; k++) {
        const double t = (double)k * 0.001;
        if (k % 50 == 0) {
            eje_ref_nueva(&a1.e, (float)rampa_az(t), NAN, 0.0f);
            eje_ref_nueva(&a2.e, (float)rampa_az(t), NAN, 0.0f);
            eje_ref_nueva(&e2.e, (float)rampa_el(t), NAN, 0.0f);
        }
        carril_tick(&a1, false);
        carril_tick(&a2, false);
        carril_tick(&e2, false);
    }
    CHECK(a1.p.theta == a2.p.theta);          /* identicos: no hay estado compartido */
    CHECK(a1.e.estado == EJE_ACTIVO && e2.e.estado == EJE_ACTIVO);
    /* error de regimen < 0.1 deg (objetivo del curso) + el retardo de la referencia de 20 Hz */
    CHECK(fabs(rampa_az(20.0) - a1.p.theta) < RAD(0.1) + RAD(9.0) * 0.1);
    CHECK(fabs(rampa_el(20.0) - e2.p.theta) < RAD(0.1));
}

static void test_paro_comun_y_falla_aislada(void)
{
    /* paro de emergencia: UN contacto para los dos ejes */
    carril_t a, e;
    carril_ini(&a, -5.0f, 365.0f, RAD(10.0));
    carril_ini(&e, 0.0f, 90.0f, RAD(20.0));
    eje_ref_nueva(&a.e, (float)RAD(10.0), NAN, 0.0f);
    eje_ref_nueva(&e.e, (float)RAD(20.0), NAN, 0.0f);
    for (int k = 0; k < 300; k++) { carril_tick(&a, false); carril_tick(&e, false); }
    CHECK(a.e.estado == EJE_ACTIVO && e.e.estado == EJE_ACTIVO);
    carril_tick(&a, true); carril_tick(&e, true);
    CHECK(a.e.estado == EJE_FALLA && e.e.estado == EJE_FALLA);
    CHECK((a.e.fallas & EJE_F_ESTOP) && (e.e.fallas & EJE_F_ESTOP));
    CHECK(!a.out.habilitar && !e.out.habilitar);

    /* una falla propia de un eje NO afecta al otro: el el sale de su limite */
    carril_t a2, e2;
    carril_ini(&a2, -5.0f, 365.0f, RAD(10.0));
    carril_ini(&e2, 0.0f, 90.0f, RAD(95.0));              /* 95 deg > 90 + margen de 2 deg */
    eje_ref_nueva(&a2.e, (float)RAD(10.0), NAN, 0.0f);
    eje_ref_nueva(&e2.e, (float)RAD(95.0), NAN, 0.0f);
    for (int k = 0; k < 300; k++) { carril_tick(&a2, false); carril_tick(&e2, false); }
    CHECK(e2.e.estado == EJE_FALLA && (e2.e.fallas & EJE_F_LIMITE));
    CHECK(a2.e.estado == EJE_ACTIVO && a2.e.fallas == 0);
}

/* ---------------------------------------------------------------- costo de CPU del tick */
static double ns_por_tick(int ejes, long n)
{
    carril_t c[2];
    carril_ini(&c[0], -5.0f, 365.0f, RAD(10.0));
    carril_ini(&c[1], 0.0f, 90.0f, RAD(20.0));
    struct timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    for (long k = 0; k < n; k++) {
        const double t = (double)k * 0.001;
        if (k % 50 == 0) {
            eje_ref_nueva(&c[0].e, (float)rampa_az(t), NAN, 0.0f);
            eje_ref_nueva(&c[1].e, (float)rampa_el(t), NAN, 0.0f);
        }
        for (int j = 0; j < ejes; j++) carril_tick(&c[j], false);
    }
    clock_gettime(CLOCK_MONOTONIC, &t1);
    if (c[0].p.theta == 12345.678) printf("?\n");           /* evita que se descarte el calculo */
    return ((double)(t1.tv_sec - t0.tv_sec) * 1e9 + (double)(t1.tv_nsec - t0.tv_nsec)) / (double)n;
}

static void test_costo_del_tick(void)
{
    const long n = 400000;
    ns_por_tick(1, 20000);                                  /* calienta */
    double a1 = 1e18, a2 = 1e18;
    for (int rep = 0; rep < 5; rep++) {                     /* mejor de 5: el host puede estar cargado */
        const double x1 = ns_por_tick(1, n), x2 = ns_por_tick(2, n);
        if (x1 < a1) a1 = x1;
        if (x2 < a2) a2 = x2;
    }
    /* incluye el lazo + la planta en double (solo de prueba): cota superior del costo del lazo */
    printf("  costo del tick en el host (lazo + planta de prueba): 1 eje %.0f ns, 2 ejes %.0f ns (x%.2f)\n",
           a1, a2, a2 / a1);
    CHECK(a2 < 3.0 * a1);                  /* dos ejes cuestan ~el doble, no mas */
    CHECK(a2 < 100000.0);                  /* y muy por debajo del periodo de 1 ms */
}

int main(void)
{
    RUN(test_buzon_indexado);
    RUN(test_telemetria_maximos_por_eje);
    RUN(test_ejes_independientes_bit_a_bit);
    RUN(test_paro_comun_y_falla_aislada);
    RUN(test_costo_del_tick);
    MT_FIN();
}
