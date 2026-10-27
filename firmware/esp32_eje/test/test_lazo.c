/* Pruebas del lazo completo (eje_t) contra la planta J/b en C: conversiones,
 * referencia, watchdog, limites, parada de emergencia, fallas y mapeo a PWM.
 * Cada prueba dice de donde sale su umbral. */
#include <math.h>
#include <stdio.h>

#include "../core/eje_lazo.h"
#include "../core/eje_planta.h"
#include "minitest.h"

#define J_EJE 2.0      /* kg m^2: atp_ros.model.J_EJE */
#define B_EJE 0.5      /* N m s/rad: atp_ros.model.B_EJE */
#define RAD(d) ((d) * M_PI / 180.0)
#define GRAD(r) ((r) * 180.0 / M_PI)

/* banco: el eje, su planta y el encoder ideal */
typedef struct {
    eje_t e;
    eje_planta_t p;
    eje_entrada_t in;
    eje_salida_t out;
    int bloqueada;       /* 1: la planta no se mueve (eje atascado) */
    double rafaga;
} banco_t;

static void banco_ini(banco_t *b, const eje_cfg_t *cfg, double theta0)
{
    eje_iniciar(&b->e, cfg);
    eje_planta_iniciar(&b->p, J_EJE, B_EJE, 0.0, theta0);
    b->in.estop = false;
    b->in.corriente_a = NAN;
    b->bloqueada = 0;
    b->rafaga = 0.0;
    b->out.duty = 0.0f;
    b->out.habilitar = false;
}

/* un tick: encoder -> lazo -> planta (con el mando aplicado como retencion de orden cero) */
static void banco_tick(banco_t *b)
{
    b->in.cuentas = eje_planta_cuentas(&b->p, b->e.cfg.cuentas_por_vuelta) * b->e.cfg.signo_enc;
    eje_paso(&b->e, &b->in, &b->out);
    const double u = b->out.habilitar
        ? (double)b->out.duty * b->e.cfg.u_pleno * b->e.cfg.signo_motor : 0.0;
    if (!b->bloqueada) eje_planta_paso(&b->p, u, b->rafaga, b->e.cfg.dt);
}

/* corre `n` ticks enviando la referencia analitica cada 50 ms (20 Hz, como atp_ros) */
static void banco_con_ref(banco_t *b, long n, double (*ref)(double), double *err_rms, double *err_max)
{
    double s = 0.0, mx = 0.0;
    long m = 0;
    for (long k = 0; k < n; k++) {
        const double t = (double)b->e.tick * b->e.cfg.dt;
        if (b->e.tick % 50 == 0) eje_ref_nueva(&b->e, (float)ref(t), NAN, 0.0f);
        banco_tick(b);
        if (b->e.estado == EJE_ACTIVO) {
            const double e = fabs(ref((double)b->e.tick * b->e.cfg.dt) - b->p.theta);
            s += e * e; m++;
            if (e > mx) mx = e;
        }
    }
    if (err_rms) *err_rms = m ? sqrt(s / (double)m) : 0.0;
    if (err_max) *err_max = mx;
}

static double ref_rampa(double t) { return RAD(9.0) * t; }
static double ref_cte(double t)   { (void)t; return RAD(20.0); }

static eje_cfg_t cfg_base(void)
{
    eje_cfg_t c;
    eje_cfg_defecto(&c);
    return c;
}

/* ---------------------------------------------------------------- piezas */

static void test_cfg_defecto_valida(void)
{
    eje_cfg_t c = cfg_base();
    CHECK(eje_cfg_validar(&c) == 0);
    c.dt = 0.0f;                         CHECK(eje_cfg_validar(&c) < 0);
    c = cfg_base(); c.lim_min = c.lim_max; CHECK(eje_cfg_validar(&c) < 0);
    c = cfg_base(); c.duty_max = 1.5f;     CHECK(eje_cfg_validar(&c) < 0);
    c = cfg_base(); c.signo_enc = 0;       CHECK(eje_cfg_validar(&c) < 0);
    /* los valores por defecto son los de atp_ros (pid_controller.py, mount_sim.py) */
    c = cfg_base();
    CHECK_NEAR(c.kp, 25.0, 0); CHECK_NEAR(c.ki, 2.0, 0); CHECK_NEAR(c.kd, 8.0, 0);
    CHECK_NEAR(c.tau_d, 0.08, 1e-7); CHECK_NEAR(c.u_max, 8.0, 0);
    CHECK_NEAR(c.t_watchdog_s, 0.5, 0);    /* src/atp_hw/README.md, punto 4 */
    CHECK_NEAR(c.dt, 0.001, 1e-9);        /* 1 kHz */
}

static void test_conversion_cuentas_rad(void)
{
    eje_cfg_t c = cfg_base();
    c.cuentas_por_vuelta = 4000.0f;
    CHECK_NEAR(eje_cuentas_a_rad(&c, 4000), 2.0 * M_PI, 1e-5);       /* una vuelta */
    CHECK_NEAR(eje_cuentas_a_rad(&c, 1000), M_PI / 2.0, 1e-5);
    CHECK_NEAR(eje_cuentas_a_rad(&c, -2000), -M_PI, 1e-5);
    CHECK(eje_rad_a_cuentas(&c, (float)M_PI) == 2000);
    /* ida y vuelta: el error nunca pasa de media cuenta */
    const double q = 2.0 * M_PI / 4000.0;
    for (int i = -50; i <= 50; i++) {
        const float r = (float)(i * 0.0371);
        const float r2 = eje_cuentas_a_rad(&c, eje_rad_a_cuentas(&c, r));
        CHECK(fabs((double)r2 - (double)r) <= 0.5 * q + 1e-6);
    }
    c.signo_enc = -1;
    CHECK_NEAR(eje_cuentas_a_rad(&c, 1000), -M_PI / 2.0, 1e-5);
    CHECK(eje_rad_a_cuentas(&c, (float)(M_PI / 2.0)) == -1000);
    /* cuentas por grado de la configuracion por defecto: 40000/360 = 111.1; un
     * escalon = 0.009 deg, un decimo del objetivo de 0.1 deg (model.OBJETIVO_DEG) */
    eje_cfg_t d = cfg_base();
    CHECK_NEAR(GRAD(eje_cuentas_a_rad(&d, 1)), 0.009, 1e-6);
}

static void test_limitar_y_slew(void)
{
    CHECK_NEAR(eje_limitar(5.0f, -1.0f, 1.0f), 1.0, 0);
    CHECK_NEAR(eje_limitar(-5.0f, -1.0f, 1.0f), -1.0, 0);
    CHECK_NEAR(eje_limitar(0.3f, -1.0f, 1.0f), 0.3, 1e-7);
    CHECK_NEAR(eje_slew(0.0f, 10.0f, 2.0f, 0.1f), 0.2, 1e-6);       /* vmax dt */
    CHECK_NEAR(eje_slew(0.0f, -10.0f, 2.0f, 0.1f), -0.2, 1e-6);
    CHECK_NEAR(eje_slew(0.0f, 0.05f, 2.0f, 0.1f), 0.05, 1e-7);      /* llega sin pasarse */
}

/* ---------------------------------------------------------------- PID */

static void test_pid_antiwindup(void)
{
    /* Paso de 90 grados: kp*e = 39 N m >> 8 N m. Con clamping la integral se
     * congela mientras u_raw satura y empuja mas. Se compara con la integral
     * "ingenua" (sumar e dt siempre) que se lleva en paralelo: sin anti-windup
     * esa seria la integral del lazo. */
    eje_cfg_t c = cfg_base();
    c.vel_max = 10.0f;                       /* sin tope de referencia: aislar el PID */
    c.lim_max = (float)RAD(400.0);
    c.vel_falla = 100.0f;                    /* y sin proteccion de velocidad: a 8 N m la planta */
    c.salto_max_vel = 100.0f;                /* llega a 16 rad/s (u/b), mucho mas que la montura */
    c.acc_limite = 1000.0f;
    banco_t b;
    banco_ini(&b, &c, 0.0);
    eje_armar(&b.e, true);
    double int_max = 0.0, int_ingenua = 0.0, int_ingenua_max = 0.0, sobrepico = 0.0;
    const double r = RAD(90.0);
    for (long k = 0; k < 60000; k++) {
        if (b.e.tick % 50 == 0) eje_ref_nueva(&b.e, (float)r, 0.0f, 0.0f);
        banco_tick(&b);
        if (int_ingenua_max < int_ingenua) int_ingenua_max = int_ingenua;
        int_ingenua += (r - b.p.theta) * 0.001;
        if (b.e.pid.integral > int_max) int_max = b.e.pid.integral;
        if (b.p.theta - r > sobrepico) sobrepico = b.p.theta - r;
    }
    printf("  integral max = %.4f rad s (ingenua llegaria a %.4f), sobrepico = %.2f deg\n",
           int_max, int_ingenua_max, GRAD(sobrepico));
    CHECK(b.e.estado == EJE_ACTIVO);
    CHECK(int_max < 0.3);
    CHECK(int_ingenua_max > 3.0 * int_max);   /* el clamping evita al menos 2/3 de la acumulacion */
    CHECK(GRAD(sobrepico) < 8.0);           /* medido ~4.7 deg; el PID de atp_ros es el mismo (vector paso_90deg_saturado) */
    CHECK_NEAR(b.p.theta, r, RAD(0.1));
}

static void test_pid_u_max(void)
{
    eje_pid_t p;
    eje_pid_iniciar(&p, 25.0f, 2.0f, 8.0f, 0.08f, 0.001f, 8.0f);
    float u = eje_pid_paso(&p, 10.0f, 0.0f);
    CHECK_NEAR(u, 8.0, 0);
    u = eje_pid_paso(&p, -10.0f, 0.0f);
    CHECK_NEAR(u, -8.0, 0);
    /* error pequeno: kp e */
    eje_pid_iniciar(&p, 25.0f, 0.0f, 0.0f, 0.08f, 0.001f, 8.0f);
    CHECK_NEAR(eje_pid_paso(&p, 0.1f, 0.0f), 2.5, 1e-6);
    /* pre-alimentacion se suma */
    eje_pid_iniciar(&p, 25.0f, 0.0f, 0.0f, 0.08f, 0.001f, 8.0f);
    CHECK_NEAR(eje_pid_paso(&p, 0.0f, 1.5f), 1.5, 1e-6);
}

static void test_pid_integral_error_cero(void)
{
    /* un par constante de 1 N m se compensa con la integral: error estacionario 0 */
    eje_cfg_t c = cfg_base();
    banco_t b;
    banco_ini(&b, &c, 0.0);
    b.rafaga = 1.0;
    eje_armar(&b.e, true);
    banco_con_ref(&b, 150000, ref_cte, NULL, NULL);     /* ki = 2: la integral tarda ~100 s */
    printf("  error final con 1 N m constante (150 s) = %.5f deg\n", GRAD(ref_cte(0) - b.p.theta));
    CHECK_NEAR(b.p.theta, RAD(20.0), RAD(0.02));
}

/* ---------------------------------------------------------------- seguimiento */

static void test_seguimiento_rampa_9degs(void)
{
    /* la referencia llega a 20 Hz con saltos de 0.45 deg (v dt); el lazo la
     * interpola con la velocidad estimada y la sigue. Umbral: el presupuesto de
     * apuntamiento del curso, 0.1 deg (model.OBJETIVO_DEG), ya pasado el
     * transitorio de enganche (la montura arranca en reposo sobre una
     * referencia que se mueve a 9 deg/s: docs/CONTROL.md, seccion 5) */
    eje_cfg_t c = cfg_base();
    banco_t b;
    banco_ini(&b, &c, 0.0);
    eje_armar(&b.e, true);
    eje_ref_nueva(&b.e, (float)ref_rampa(0.0), NAN, 0.0f);
    double rms_t, mx_t, rms, mx;
    banco_con_ref(&b, 3000, ref_rampa, &rms_t, &mx_t);        /* transitorio: 3 s */
    banco_con_ref(&b, 27000, ref_rampa, &rms, &mx);           /* regimen: 27 s */
    printf("  rampa 9 deg/s: transitorio (3 s) max=%.3f deg; regimen (27 s) rms=%.4f deg max=%.4f deg\n",
           GRAD(mx_t), GRAD(rms), GRAD(mx));
    CHECK(GRAD(mx) < 0.1);
    CHECK(GRAD(mx_t) < 2.5);                 /* el pico de enganche, ~1.7 deg medido */
    CHECK(b.e.estado == EJE_ACTIVO);
    CHECK(GRAD(fabs(ref_rampa(30.0) - b.p.theta)) < 0.05);
}

static void test_seguimiento_con_feedforward(void)
{
    /* kff = b da el par de arrastre J*0 + b*w: reduce el error de la rampa */
    eje_cfg_t c = cfg_base();
    banco_t b0, b1;
    banco_ini(&b0, &c, 0.0);
    c.kff_v = (float)B_EJE;
    banco_ini(&b1, &c, 0.0);
    double rms0, rms1;
    eje_armar(&b0.e, true); eje_armar(&b1.e, true);
    eje_ref_nueva(&b0.e, 0.0f, NAN, 0.0f); eje_ref_nueva(&b1.e, 0.0f, NAN, 0.0f);
    banco_con_ref(&b0, 20000, ref_rampa, &rms0, NULL);
    banco_con_ref(&b1, 20000, ref_rampa, &rms1, NULL);
    printf("  rms sin ff = %.5f deg, con ff = %.5f deg\n", GRAD(rms0), GRAD(rms1));
    CHECK(rms1 < rms0);
}

static void test_arranque_sin_salto(void)
{
    /* la referencia esta lejos (30 deg) al armar: el mando sale de la posicion
     * actual y sube con el tope de velocidad; nunca pide mas de vel_max */
    eje_cfg_t c = cfg_base();
    banco_t b;
    banco_ini(&b, &c, 0.0);
    eje_armar(&b.e, true);
    double vmax = 0.0;
    for (long k = 0; k < 6000; k++) {
        if (b.e.tick % 50 == 0) eje_ref_nueva(&b.e, (float)RAD(30.0), 0.0f, 0.0f);
        banco_tick(&b);
        if (fabs(b.e.ref_cmd_vel) > vmax) vmax = fabs(b.e.ref_cmd_vel);
    }
    printf("  velocidad max de la referencia comandada = %.3f deg/s (tope %.1f)\n",
           GRAD(vmax), GRAD(c.vel_max));
    CHECK(vmax <= (double)c.vel_max * 1.0001);
    CHECK(vmax > 0.9 * (double)c.vel_max);
    CHECK(GRAD(b.p.theta) > 5.0);           /* ya se esta moviendo hacia la referencia */
}

/* ---------------------------------------------------------------- watchdog */

static void test_watchdog_retencion_y_apagado(void)
{
    eje_cfg_t c = cfg_base();
    banco_t b;
    banco_ini(&b, &c, 0.0);
    eje_armar(&b.e, true);
    /* 5 s siguiendo una referencia constante de 20 deg (20 Hz) */
    banco_con_ref(&b, 5000 - 1, ref_cte, NULL, NULL);
    CHECK(b.e.estado == EJE_ACTIVO);
    const double th_corte = b.p.theta;
    /* se corta la referencia. Con t_watchdog = 0.5 s: RETENCION a los 500 ms */
    long k_ret = -1, k_esp = -1;
    for (long k = 0; k < 4000; k++) {
        banco_tick(&b);
        if (k_ret < 0 && b.e.estado == EJE_RETENCION) k_ret = k;
        if (k_esp < 0 && b.e.estado == EJE_ESPERA) k_esp = k;
    }
    printf("  RETENCION a los %ld ms, driver apagado a los %ld ms del corte\n", k_ret + 1, k_esp + 1);
    CHECK(k_ret >= 0 && k_ret + 1 >= 450 && k_ret + 1 <= 550);           /* ~0.5 s */
    CHECK(k_esp >= 0 && k_esp + 1 >= 2400 && k_esp + 1 <= 2600);          /* 0.5 + 2.0 s */
    CHECK(!b.out.habilitar && b.out.duty == 0.0f);
    CHECK_NEAR(b.e.u, 0.0, 0);
    (void)th_corte;
}

static void test_watchdog_mantiene_posicion(void)
{
    /* en RETENCION el lazo sigue activo: un par externo no arrastra la carga */
    eje_cfg_t c = cfg_base();
    banco_t b;
    banco_ini(&b, &c, 0.0);
    eje_armar(&b.e, true);
    banco_con_ref(&b, 8000, ref_cte, NULL, NULL);
    const double th0 = b.p.theta;
    long ticks_ret = 0;
    b.rafaga = 0.2;
    for (long k = 0; k < 2300; k++) {
        banco_tick(&b);
        if (b.e.estado == EJE_RETENCION) {
            ticks_ret++;
            CHECK(b.out.habilitar);
        }
    }
    /* un par de 0.2 N m contra kp = 25 N m/rad deja 0.2/25 rad = 0.46 deg */
    printf("  deriva en RETENCION con 0.2 N m: %.4f deg tras %ld ms\n", GRAD(b.p.theta - th0), ticks_ret);
    CHECK(ticks_ret > 1500);
    CHECK(fabs(GRAD(b.p.theta - th0)) < 0.6);
}

static void test_watchdog_retoma(void)
{
    eje_cfg_t c = cfg_base();
    banco_t b;
    banco_ini(&b, &c, 0.0);
    eje_armar(&b.e, true);
    banco_con_ref(&b, 6000, ref_cte, NULL, NULL);
    for (long k = 0; k < 5000; k++) banco_tick(&b);       /* 5 s sin referencia: ya en ESPERA */
    CHECK(b.e.estado == EJE_ESPERA);
    eje_ref_nueva(&b.e, (float)RAD(25.0), 0.0f, 0.0f);    /* vuelve */
    banco_tick(&b);
    CHECK(b.e.estado == EJE_ACTIVO && b.out.habilitar);
    for (long k = 0; k < 8000; k++) {
        if (b.e.tick % 50 == 0) eje_ref_nueva(&b.e, (float)RAD(25.0), 0.0f, 0.0f);
        banco_tick(&b);
    }
    CHECK_NEAR(b.p.theta, RAD(25.0), RAD(0.05));
}

static void test_sin_referencia_no_arranca(void)
{
    eje_cfg_t c = cfg_base();
    banco_t b;
    banco_ini(&b, &c, 0.0);
    eje_armar(&b.e, true);
    for (int k = 0; k < 2000; k++) { banco_tick(&b); CHECK(!b.out.habilitar); }
    CHECK(b.e.estado == EJE_ESPERA);
    /* una referencia NaN no cuenta */
    eje_ref_nueva(&b.e, NAN, NAN, 0.0f);
    banco_tick(&b);
    CHECK(b.e.estado == EJE_ESPERA);
    /* desarmado: nunca habilita aunque llegue referencia */
    banco_ini(&b, &c, 0.0);
    for (int k = 0; k < 2000; k++) {
        if (k % 50 == 0) eje_ref_nueva(&b.e, 0.1f, 0.0f, 0.0f);
        banco_tick(&b);
        CHECK(!b.out.habilitar);
    }
    CHECK(b.e.estado == EJE_DESARMADO);
}

/* ---------------------------------------------------------------- limites */

static void test_limite_de_software(void)
{
    eje_cfg_t c = cfg_base();
    c.lim_min = (float)RAD(0.0);
    c.lim_max = (float)RAD(90.0);                       /* eje de elevacion */
    banco_t b;
    banco_ini(&b, &c, RAD(80.0));
    eje_armar(&b.e, true);
    double mx = 0.0;
    for (long k = 0; k < 30000; k++) {
        if (b.e.tick % 50 == 0) eje_ref_nueva(&b.e, (float)RAD(120.0), 0.0f, 0.0f);   /* pide 120 deg */
        banco_tick(&b);
        if (b.p.theta > mx) mx = b.p.theta;
    }
    printf("  referencia 120 deg con limite 90: maximo alcanzado %.3f deg\n", GRAD(mx));
    CHECK(b.e.estado == EJE_ACTIVO);                    /* limitar la referencia no es una falla */
    CHECK_NEAR(b.e.ref_cmd, RAD(90.0), 1e-5);
    CHECK(GRAD(mx) <= 90.5);
    /* y por abajo */
    banco_ini(&b, &c, RAD(10.0));
    eje_armar(&b.e, true);
    double mn = 1e9;
    for (long k = 0; k < 30000; k++) {
        if (b.e.tick % 50 == 0) eje_ref_nueva(&b.e, (float)RAD(-30.0), 0.0f, 0.0f);
        banco_tick(&b);
        if (b.p.theta < mn) mn = b.p.theta;
    }
    CHECK(GRAD(mn) >= -0.5);
}

static void test_falla_fuera_de_limites(void)
{
    eje_cfg_t c = cfg_base();
    c.lim_min = (float)RAD(0.0);
    c.lim_max = (float)RAD(90.0);
    banco_t b;
    banco_ini(&b, &c, RAD(50.0));
    eje_armar(&b.e, true);
    eje_ref_nueva(&b.e, (float)RAD(50.0), 0.0f, 0.0f);
    for (int k = 0; k < 100; k++) banco_tick(&b);
    CHECK(b.e.estado == EJE_ACTIVO);
    /* el eje aparece en 93 deg (margen de falla 2 deg): lo empujaron o el encoder salto */
    b.p.theta = RAD(93.0);
    banco_tick(&b);
    banco_tick(&b);
    CHECK(b.e.estado == EJE_FALLA);
    CHECK((b.e.fallas & EJE_F_LIMITE) != 0);
    CHECK(!b.out.habilitar);
}

/* ---------------------------------------------------------------- paro de emergencia */

static void test_estop(void)
{
    eje_cfg_t c = cfg_base();
    banco_t b;
    banco_ini(&b, &c, 0.0);
    eje_armar(&b.e, true);
    banco_con_ref(&b, 3000, ref_rampa, NULL, NULL);
    CHECK(b.e.estado == EJE_ACTIVO && b.out.habilitar);
    b.in.estop = true;                                   /* se abre el lazo del paro */
    banco_tick(&b);
    CHECK(b.e.estado == EJE_FALLA);                      /* el MISMO tick */
    CHECK(!b.out.habilitar && b.out.duty == 0.0f);
    CHECK((b.e.fallas & EJE_F_ESTOP) != 0);
    /* sigue en falla aunque llegue referencia */
    for (int k = 0; k < 500; k++) {
        if (k % 50 == 0) eje_ref_nueva(&b.e, 1.0f, 0.0f, 0.0f);
        banco_tick(&b);
        CHECK(!b.out.habilitar);
    }
    /* con el paro aun abierto no se rearma */
    eje_armar(&b.e, true);
    banco_tick(&b);
    CHECK(b.e.estado == EJE_FALLA);
    /* se cierra el paro: sigue enclavada hasta rearmar (no arranca sola) */
    b.in.estop = false;
    for (int k = 0; k < 100; k++) banco_tick(&b);
    CHECK(b.e.estado == EJE_FALLA && !b.out.habilitar);
    eje_armar(&b.e, true);
    banco_tick(&b);
    CHECK(b.e.estado == EJE_ESPERA || b.e.estado == EJE_ACTIVO);
    CHECK(b.e.fallas == 0u);
}

static void test_desarmar_apaga(void)
{
    eje_cfg_t c = cfg_base();
    banco_t b;
    banco_ini(&b, &c, 0.0);
    eje_armar(&b.e, true);
    banco_con_ref(&b, 3000, ref_rampa, NULL, NULL);
    CHECK(b.out.habilitar);
    eje_armar(&b.e, false);
    banco_tick(&b);
    CHECK(b.e.estado == EJE_DESARMADO && !b.out.habilitar && b.out.duty == 0.0f);
}

/* ---------------------------------------------------------------- otras fallas */

static void test_falla_salto_encoder(void)
{
    eje_cfg_t c = cfg_base();
    banco_t b;
    banco_ini(&b, &c, RAD(10.0));
    eje_armar(&b.e, true);
    eje_ref_nueva(&b.e, (float)RAD(10.0), 0.0f, 0.0f);
    for (int k = 0; k < 50; k++) banco_tick(&b);
    CHECK(b.e.estado == EJE_ACTIVO);
    b.p.theta += RAD(1.0);                  /* 1 deg en 1 ms = 17 rad/s > 5 rad/s */
    banco_tick(&b);
    CHECK(b.e.estado == EJE_FALLA && (b.e.fallas & EJE_F_ENCODER));
}

static void test_falla_sobrevelocidad(void)
{
    /* la velocidad medida sostenida por encima de vel_falla (30 deg/s) enclava */
    eje_cfg_t c = cfg_base();
    c.vel_max = (float)RAD(100.0);          /* para que el slew no lo impida */
    banco_t b;
    banco_ini(&b, &c, 0.0);
    eje_armar(&b.e, true);
    eje_ref_nueva(&b.e, 0.0f, 0.0f, 0.0f);
    for (int k = 0; k < 20; k++) banco_tick(&b);
    for (int k = 0; k < 300 && b.e.estado != EJE_FALLA; k++) {
        b.p.theta += RAD(50.0) * 0.001;     /* alguien la gira a 50 deg/s */
        banco_tick(&b);
    }
    CHECK(b.e.estado == EJE_FALLA && (b.e.fallas & EJE_F_VEL));
}

static void test_falla_atasco(void)
{
    /* el eje no se mueve (cuerda trabada): el error crece a vel_max, el par
     * llega a >90 % de u_max y tras 1 s parado se enclava */
    eje_cfg_t c = cfg_base();
    banco_t b;
    banco_ini(&b, &c, 0.0);
    b.bloqueada = 1;
    eje_armar(&b.e, true);
    long k_f = -1;
    for (long k = 0; k < 8000; k++) {
        if (b.e.tick % 50 == 0) eje_ref_nueva(&b.e, (float)RAD(60.0), 0.0f, 0.0f);
        banco_tick(&b);
        if (b.e.estado == EJE_FALLA) { k_f = k; break; }
    }
    printf("  atasco detectado a los %ld ms\n", k_f);
    CHECK(k_f > 0);
    CHECK((b.e.fallas & EJE_F_ATASCO) != 0);
    CHECK(!b.out.habilitar);
}

static void test_falla_sobrecorriente(void)
{
    eje_cfg_t c = cfg_base();
    c.i_max_a = 20.0f;
    c.i_max_s = 0.05f;
    banco_t b;
    banco_ini(&b, &c, 0.0);
    eje_armar(&b.e, true);
    eje_ref_nueva(&b.e, 0.0f, 0.0f, 0.0f);
    for (int k = 0; k < 20; k++) banco_tick(&b);
    /* un pico de 10 ms no dispara */
    b.in.corriente_a = 25.0f;
    for (int k = 0; k < 10; k++) banco_tick(&b);
    b.in.corriente_a = 5.0f;
    banco_tick(&b);
    CHECK(b.e.estado != EJE_FALLA);
    /* 60 ms sostenidos si */
    b.in.corriente_a = 25.0f;
    for (int k = 0; k < 60; k++) banco_tick(&b);
    CHECK(b.e.estado == EJE_FALLA && (b.e.fallas & EJE_F_CORR));
    /* sin sensor (NAN) nunca dispara */
    banco_ini(&b, &c, 0.0);
    eje_armar(&b.e, true);
    eje_ref_nueva(&b.e, 0.0f, 0.0f, 0.0f);
    for (int k = 0; k < 500; k++) banco_tick(&b);
    CHECK(b.e.estado == EJE_ACTIVO);
}

static void test_cfg_invalida_no_arma(void)
{
    eje_cfg_t c = cfg_base();
    c.u_max = -1.0f;
    banco_t b;
    banco_ini(&b, &c, 0.0);
    CHECK(b.e.estado == EJE_FALLA && (b.e.fallas & EJE_F_CFG));
    eje_armar(&b.e, true);
    for (int k = 0; k < 10; k++) { eje_ref_nueva(&b.e, 0.0f, 0.0f, 0.0f); banco_tick(&b); }
    CHECK(b.e.estado == EJE_FALLA && !b.out.habilitar);
}

/* ---------------------------------------------------------------- mando -> PWM */

static void test_duty_mapeo(void)
{
    eje_cfg_t c = cfg_base();
    c.u_pleno = 10.0f;               /* 10 N m al 100 % */
    c.u_max = 8.0f;
    c.duty_max = 0.95f;
    banco_t b;
    banco_ini(&b, &c, 0.0);
    eje_armar(&b.e, true);
    /* referencia lejana: el par satura en min(u_max, duty_max*u_pleno) = 8 N m -> duty 0.8 */
    float dmax = 0.0f;
    for (long k = 0; k < 400; k++) {
        if (b.e.tick % 50 == 0) eje_ref_nueva(&b.e, 5.0f, 0.0f, 0.0f);
        banco_tick(&b);
        if (fabsf(b.out.duty) > dmax) dmax = fabsf(b.out.duty);
    }
    CHECK(dmax <= 0.95f + 1e-6f);
    CHECK(dmax <= 0.8f + 1e-6f);
    /* si el driver da menos que u_max, manda el driver */
    eje_cfg_t d = cfg_base();
    d.u_pleno = 6.0f;
    d.duty_max = 0.9f;               /* 5.4 N m */
    banco_ini(&b, &d, 0.0);
    eje_armar(&b.e, true);
    for (long k = 0; k < 400; k++) {
        if (b.e.tick % 50 == 0) eje_ref_nueva(&b.e, 5.0f, 0.0f, 0.0f);
        banco_tick(&b);
    }
    CHECK(fabsf(b.e.u) <= 5.4f + 1e-4f);
}

static void test_signos_motor_y_encoder(void)
{
    /* montura con el motor y el encoder invertidos: el lazo sigue funcionando
     * porque banco_tick aplica los mismos signos que el hardware */
    eje_cfg_t c = cfg_base();
    c.signo_enc = -1;
    c.signo_motor = -1;
    banco_t b;
    banco_ini(&b, &c, 0.0);
    eje_armar(&b.e, true);
    for (long k = 0; k < 8000; k++) {
        if (b.e.tick % 50 == 0) eje_ref_nueva(&b.e, (float)RAD(10.0), 0.0f, 0.0f);
        banco_tick(&b);
    }
    /* la planta ve u = duty*u_pleno*signo_motor, y el encoder cuenta al reves */
    CHECK_NEAR(b.p.theta, RAD(10.0), RAD(0.05));
}

static void test_duty_zona_muerta(void)
{
    eje_cfg_t c = cfg_base();
    c.duty_min = 0.1f;
    banco_t b;
    banco_ini(&b, &c, 0.0);
    eje_armar(&b.e, true);
    eje_ref_nueva(&b.e, 0.0f, 0.0f, 0.0f);
    banco_tick(&b);
    banco_tick(&b);
    /* error 0: duty 0 (no se suma el piso a un mando nulo) */
    CHECK_NEAR(b.out.duty, 0.0, 1e-6);
    /* mando pequeno pero no nulo: pasa el piso de 10 % */
    banco_ini(&b, &c, 0.0);
    eje_armar(&b.e, true);
    eje_ref_nueva(&b.e, (float)RAD(0.2), 0.0f, 0.0f);
    for (int k = 0; k < 3; k++) banco_tick(&b);
    CHECK(fabsf(b.out.duty) >= 0.1f);
}

/* ---------------------------------------------------------------- origen */

static void test_fijar_origen(void)
{
    eje_cfg_t c = cfg_base();
    banco_t b;
    banco_ini(&b, &c, 0.0);
    /* 40000 cuentas por vuelta: 10000 cuentas = 90 deg. Se declara que ahi hay 45 deg */
    CHECK(eje_fijar_origen(&b.e, (float)RAD(45.0), 10000));
    b.in.cuentas = 10000;
    eje_paso(&b.e, &b.in, &b.out);
    CHECK_NEAR(GRAD(b.e.pos), 45.0, 1e-3);
    b.in.cuentas = 10010;                       /* 10 cuentas = 0.09 deg mas -> 45.09 */
    eje_paso(&b.e, &b.in, &b.out);
    CHECK_NEAR(GRAD(b.e.pos), 45.09, 1e-3);
    /* con el motor en marcha no se acepta */
    eje_armar(&b.e, true);
    eje_ref_nueva(&b.e, (float)RAD(45.09), 0.0f, 0.0f);
    eje_paso(&b.e, &b.in, &b.out);
    CHECK(b.e.estado == EJE_ACTIVO);
    CHECK(!eje_fijar_origen(&b.e, 0.0f, 10010));
}

static void test_nombres(void)
{
    CHECK(eje_estado_nombre(EJE_ACTIVO)[0] == 'A');
    CHECK(eje_estado_nombre(EJE_FALLA)[0] == 'F');
}

int main(void)
{
    RUN(test_cfg_defecto_valida);
    RUN(test_conversion_cuentas_rad);
    RUN(test_limitar_y_slew);
    RUN(test_pid_u_max);
    RUN(test_pid_antiwindup);
    RUN(test_pid_integral_error_cero);
    RUN(test_seguimiento_rampa_9degs);
    RUN(test_seguimiento_con_feedforward);
    RUN(test_arranque_sin_salto);
    RUN(test_watchdog_retencion_y_apagado);
    RUN(test_watchdog_mantiene_posicion);
    RUN(test_watchdog_retoma);
    RUN(test_sin_referencia_no_arranca);
    RUN(test_limite_de_software);
    RUN(test_falla_fuera_de_limites);
    RUN(test_estop);
    RUN(test_desarmar_apaga);
    RUN(test_falla_salto_encoder);
    RUN(test_falla_sobrevelocidad);
    RUN(test_falla_atasco);
    RUN(test_falla_sobrecorriente);
    RUN(test_cfg_invalida_no_arma);
    RUN(test_duty_mapeo);
    RUN(test_signos_motor_y_encoder);
    RUN(test_duty_zona_muerta);
    RUN(test_fijar_origen);
    RUN(test_nombres);
    MT_FIN();
}
