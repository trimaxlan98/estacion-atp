/* El PID en float y la planta en double del C contra los CSV que genera
 * gen_vectores.py desde atp_ros (PlantaMontura de planta.py, J y b de model.py).
 *
 * Uso: test_vectores <directorio de vectores>
 * Cada escenario se corre a 1 kHz y se compara fila a fila (theta, omega, u).
 * Las tolerancias salen de aplicar float (24 bits de mantisa) a un lazo estable:
 * el error NO crece con el tiempo, y se imprime lo medido.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../core/eje_lazo.h"
#include "../core/eje_planta.h"
#include "minitest.h"

static const char *DIR_VEC = "vectores";

typedef struct {
    char tipo[16];
    double ref, vel, u, t_fin, h, J, b, kp, ki, kd, tau, u_max;
    double rafaga_t0, rafaga_dur, rafaga_nm;
    int bits, decimacion;
} esc_t;

static double buscar(const char *linea, const char *clave, double defecto)
{
    char pat[48];
    snprintf(pat, sizeof pat, " %s=", clave);
    const char *p = strstr(linea, pat);
    return p ? atof(p + strlen(pat)) : defecto;
}

static int leer_cabecera(const char *linea, esc_t *e)
{
    memset(e, 0, sizeof *e);
    const char *t = strstr(linea, "tipo=");
    if (!t) return -1;
    sscanf(t + 5, "%15s", e->tipo);
    e->ref = buscar(linea, "ref", 0);
    e->vel = buscar(linea, "vel", 0);
    e->u = buscar(linea, "u", 0);
    e->t_fin = buscar(linea, "t_fin", 0);
    e->h = buscar(linea, "h", 0.001);
    e->J = buscar(linea, "J", 2);
    e->b = buscar(linea, "b", 0.5);
    e->kp = buscar(linea, "kp", 25);
    e->ki = buscar(linea, "ki", 2);
    e->kd = buscar(linea, "kd", 8);
    e->tau = buscar(linea, "tau", 0.08);
    e->u_max = buscar(linea, "u_max", 8);
    e->rafaga_t0 = buscar(linea, "rafaga_t0", -1);
    e->rafaga_dur = buscar(linea, "rafaga_dur", 0);
    e->rafaga_nm = buscar(linea, "rafaga_nm", 0);
    e->bits = (int)buscar(linea, "bits", 0);
    e->decimacion = (int)buscar(linea, "decimacion", 50);
    return 0;
}

static void escenario(const char *nombre, double tol_theta, double tol_omega, double tol_u)
{
    char ruta[512];
    snprintf(ruta, sizeof ruta, "%s/%s.csv", DIR_VEC, nombre);
    FILE *f = fopen(ruta, "r");
    if (!f) { printf("  no se pudo abrir %s\n", ruta); CHECK(f != NULL); return; }
    char linea[512];
    esc_t e;
    CHECK(fgets(linea, sizeof linea, f) != NULL);
    CHECK(leer_cabecera(linea, &e) == 0);
    CHECK(fgets(linea, sizeof linea, f) != NULL);          /* k,theta,omega,u */

    eje_planta_t pl;
    eje_planta_iniciar(&pl, e.J, e.b, 0.0, 0.0);
    eje_pid_t pid;
    eje_pid_iniciar(&pid, (float)e.kp, (float)e.ki, (float)e.kd, (float)e.tau,
                    (float)e.h, (float)e.u_max);
    eje_cfg_t cfg;
    eje_cfg_defecto(&cfg);
    cfg.cuentas_por_vuelta = e.bits ? (float)(1 << e.bits) : cfg.cuentas_por_vuelta;

    double m_th = 0, m_om = 0, m_u = 0;
    long k_row, k_sig = 0;
    double th, om, u_py;
    long n = (long)(e.t_fin / e.h + 0.5), filas = 0;
    int dec = e.decimacion;
    for (long k = 0; k < n; k++) {
        const double t = (double)k * e.h;
        float u = 0.0f;
        if (strcmp(e.tipo, "abierta") == 0) {
            u = (float)e.u;
        } else {
            const double r = strcmp(e.tipo, "paso") == 0 ? e.ref : e.vel * t;
            double y = pl.theta;
            if (e.bits) y = (double)eje_cuentas_a_rad(&cfg, eje_planta_cuentas(&pl, cfg.cuentas_por_vuelta));
            u = eje_pid_paso(&pid, (float)(r - y), 0.0f);
        }
        if (k % dec == 0) {
            if (fscanf(f, "%ld,%lf,%lf,%lf\n", &k_row, &th, &om, &u_py) != 4) { CHECK(0); break; }
            CHECK(k_row == k);
            const double d_th = fabs(pl.theta - th), d_om = fabs(pl.omega - om), d_u = fabs((double)u - u_py);
            if (d_th > m_th) m_th = d_th;
            if (d_om > m_om) m_om = d_om;
            if (d_u > m_u) m_u = d_u;
            filas++;
        }
        double par = 0.0;
        if (e.rafaga_t0 >= 0.0 && t >= e.rafaga_t0 && t < e.rafaga_t0 + e.rafaga_dur) par = e.rafaga_nm;
        eje_planta_paso(&pl, (double)u, par, e.h);
        k_sig = k;
    }
    (void)k_sig;
    fclose(f);
    printf("  %-26s %4ld filas  max|d theta|=%.2e rad  max|d omega|=%.2e rad/s  max|d u|=%.2e N m\n",
           nombre, filas, m_th, m_om, m_u);
    CHECK(filas > 50);
    CHECK(m_th <= tol_theta);
    CHECK(m_om <= tol_omega);
    CHECK(m_u <= tol_u);
}

static void test_planta_identica(void)       { escenario("planta_abierta", 1e-9, 1e-9, 1e-9); }
static void test_paso_5deg(void)             { escenario("paso_5deg", 1e-7, 1e-6, 1e-5); }
static void test_rampa_9degs(void)           { escenario("rampa_9degs", 1e-7, 1e-6, 1e-5); }
static void test_paso_90_saturado(void)      { escenario("paso_90deg_saturado", 1e-6, 5e-6, 1e-4); }
static void test_rafaga(void)                { escenario("rafaga_1nm", 1e-7, 1e-6, 1e-5); }
/* con 16 bits el cuantizador tiene escalones: un empate de redondeo (numpy
 * redondea al par, el C hacia afuera) o un escalon distinto cambia u en
 * kp*q = 25 * 9.6e-5 = 2.4e-3 N m (y kd*q/tau mas): tolerancia mayor, a ojo
 * de ese escalon */
static void test_rampa_cuantizada(void)      { escenario("rampa_cuantizada_16bits", 1e-5, 1e-4, 0.1); }

int main(int argc, char **argv)
{
    if (argc > 1) DIR_VEC = argv[1];
    RUN(test_planta_identica);
    RUN(test_paso_5deg);
    RUN(test_rampa_9degs);
    RUN(test_paso_90_saturado);
    RUN(test_rafaga);
    RUN(test_rampa_cuantizada);
    MT_FIN();
}
