/* Nodo micro-ROS del eje SOBRE LINUX: el mismo lazo (core/) y el mismo nodo
 * rclc (uros/eje_uros.c) que el firmware, con la planta J/b de atp_ros
 * (core/eje_planta.c) en lugar del encoder y el motor. Habla con un agente
 * micro-ROS por UDP: es el "hardware en el lazo" sin hardware.
 *
 *   eje_host [--agente IP:PUERTO] [--eje az|el] [--dominio N] [--armar]
 *            [--duracion S] [--rafaga T0 DUR NM] [--kff] [--theta0 DEG]
 *            [--ejes 1|2] [--theta0-el DEG] [--tel-hz N] [--eco] [--rt] [--stats f.json]
 *   --ejes 2: acimut y elevacion en el mismo proceso (el banco de un ESP32 de dos ejes).
 */
#define _GNU_SOURCE
#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <rcl/rcl.h>
#include <rmw_microros/rmw_microros.h>

#include "eje_compartido.h"
#include "eje_lazo.h"
#include "eje_planta.h"
#include "eje_uros.h"

static char g_agente_ip[64] = "127.0.0.1";
static char g_agente_puerto[16] = "8888";
static volatile int g_fin = 0;
static double g_duracion_s = 0.0;
static int g_armar = 0;
static double g_rafaga[3] = { -1.0, 0.0, 0.0 };
static double g_theta0_rad = 0.0;
static int g_kff = 0;

static uint32_t ahora_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000u + ts.tv_nsec / 1000000u);
}

static void dormir_ms(uint32_t ms)
{
    struct timespec ts = { .tv_sec = ms / 1000u, .tv_nsec = (long)(ms % 1000u) * 1000000L };
    nanosleep(&ts, NULL);
}

static bool opciones_udp(rcl_init_options_t *io)
{
    rmw_init_options_t *r = rcl_init_options_get_rmw_init_options(io);
    return rmw_uros_options_set_udp_address(g_agente_ip, g_agente_puerto, r) == RMW_RET_OK;
}

static void en_senal(int s) { (void)s; g_fin = 1; }

static const char *g_nombres[ATP_EJES_MAX] = { "az", "el" };
static int g_n_ejes = 1;
static int g_tel_hz = 50;
static int g_eco = 0;
static int g_rt = 0;
static const char *g_stats = NULL;
static double g_theta0_el_rad = 0.0;

#define MAX_PERIODOS 4000000L
static float *g_periodos_us;       /* periodo real de cada tick (us), para los percentiles */
static float *g_tlazo_us;          /* duracion del cuerpo de cada tick (us) */
static long g_n_per = 0;

static int cmp_f(const void *a, const void *b)
{
    const float x = *(const float *)a, y = *(const float *)b;
    return (x > y) - (x < y);
}

static double percentil(float *v, long n, double p)
{
    if (n <= 0) return 0.0;
    qsort(v, (size_t)n, sizeof(float), cmp_f);
    long k = (long)(p * (double)(n - 1) + 0.5);
    return (double)v[k < 0 ? 0 : (k >= n ? n - 1 : k)];
}

/* Estadisticas de temporizacion del lazo en Linux (NO es tiempo real: ver README, seccion 7c). */
static void escribir_estadisticas(long ticks, uint32_t perdidos)
{
    if (g_n_per <= 0) return;
    const long n = g_n_per;
    double suma = 0.0;
    for (long i = 0; i < n; i++) suma += (double)g_periodos_us[i];
    float *copia = malloc((size_t)n * sizeof(float));
    if (!copia) return;
    memcpy(copia, g_periodos_us, (size_t)n * sizeof(float));
    const double p50 = percentil(copia, n, 0.50), p99 = percentil(copia, n, 0.99),
                 p999 = percentil(copia, n, 0.999), pmax = percentil(copia, n, 1.0),
                 pmin = percentil(copia, n, 0.0);
    long mayor_1500 = 0, mayor_2000 = 0;
    for (long i = 0; i < n; i++) {
        if (g_periodos_us[i] > 1500.0f) mayor_1500++;
        if (g_periodos_us[i] > 2000.0f) mayor_2000++;
    }
    memcpy(copia, g_tlazo_us, (size_t)n * sizeof(float));
    const double t50 = percentil(copia, n, 0.50), t99 = percentil(copia, n, 0.99),
                 tmax = percentil(copia, n, 1.0);
    free(copia);
    printf("eje_host: temporizacion del lazo (%ld ticks, %d eje(s)): periodo us min %.0f p50 %.0f p99 %.0f p99.9 %.0f max %.0f "
           "media %.1f | >1.5 ms: %ld, >2 ms: %ld | cuerpo del tick us p50 %.1f p99 %.1f max %.1f\n",
           ticks, g_n_ejes, pmin, p50, p99, p999, pmax, suma / (double)n, mayor_1500, mayor_2000, t50, t99, tmax);
    if (g_stats) {
        FILE *f = fopen(g_stats, "w");
        if (f) {
            fprintf(f, "{\"ticks\": %ld, \"ejes\": %d, \"periodo_us\": {\"min\": %.1f, \"p50\": %.1f, \"p99\": %.1f, "
                       "\"p999\": %.1f, \"max\": %.1f, \"media\": %.2f}, \"mayor_1500us\": %ld, \"mayor_2000us\": %ld, "
                       "\"ticks_perdidos\": %u, \"cuerpo_us\": {\"p50\": %.2f, \"p99\": %.2f, \"max\": %.2f}, \"rt\": %d}\n",
                    ticks, g_n_ejes, pmin, p50, p99, p999, pmax, suma / (double)n, mayor_1500, mayor_2000,
                    (unsigned)perdidos, t50, t99, tmax, g_rt);
            fclose(f);
        }
    }
}

/* un eje: lazo + planta simulada (J/b de atp_ros) */
typedef struct {
    eje_cfg_t cfg;
    eje_t eje;
    eje_planta_t planta;
    double err_max, err_ref;
    long n_seg;
} carril_t;

/* ---- lazo de 1 kHz en un hilo (sin tiempo real: es una prueba funcional) ---- */
static void *hilo_lazo(void *arg)
{
    (void)arg;
    carril_t car[ATP_EJES_MAX];
    for (int k = 0; k < g_n_ejes; k++) {
        carril_t *c = &car[k];
        memset(c, 0, sizeof *c);
        eje_cfg_defecto(&c->cfg);
        c->cfg.cuentas_por_vuelta = 40000.0f;
        if (k == 1) { c->cfg.lim_min = 0.0f; c->cfg.lim_max = 90.0f * EJE_PI_F / 180.0f; }   /* elevacion: 0..90 deg (Kconfig) */
        if (g_kff) c->cfg.kff_v = 0.5f;                 /* = B_EJE */
        eje_iniciar(&c->eje, &c->cfg);
        eje_planta_iniciar(&c->planta, 2.0, 0.5, 0.0, k == 0 ? g_theta0_rad : g_theta0_el_rad);   /* J_EJE, B_EJE: atp_ros/model.py */
        if (g_armar) eje_armar(&c->eje, true);
    }

    if (g_rt) {
        struct sched_param sp = { .sched_priority = 50 };
        if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0)
            printf("eje_host: SCHED_FIFO no disponible (%s): sigue con el planificador normal\n", strerror(errno));
        else
            printf("eje_host: lazo en SCHED_FIFO prio 50\n");
    }
    g_periodos_us = malloc(MAX_PERIODOS * sizeof(float));
    g_tlazo_us = malloc(MAX_PERIODOS * sizeof(float));

    eje_tel_t tel;
    memset(&tel, 0, sizeof tel);
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    struct timespec t_ini = t;
    long long t_prev_ns = (long long)t.tv_sec * 1000000000LL + t.tv_nsec;
    uint32_t perdidos = 0;
    long ticks_total = 0;

    while (!g_fin) {
        t.tv_nsec += 1000000L;                                   /* periodo de 1 ms */
        if (t.tv_nsec >= 1000000000L) { t.tv_nsec -= 1000000000L; t.tv_sec++; }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
        struct timespec n0;
        clock_gettime(CLOCK_MONOTONIC, &n0);
        const long long ns = (long long)n0.tv_sec * 1000000000LL + n0.tv_nsec;
        const long long periodo_ns = ns - t_prev_ns;
        int32_t jitter = (int32_t)(periodo_ns / 1000 - 1000);
        t_prev_ns = ns;
        if (jitter < 0) jitter = -jitter;
        if (jitter > 500) perdidos++;

        eje_salida_t out[ATP_EJES_MAX];
        for (int k = 0; k < g_n_ejes; k++) {
            carril_t *c = &car[k];
            eje_cmd_t cmd;
            if (eje_cmd_tomar(k, &cmd)) {
                if (cmd.hay_armar) eje_armar(&c->eje, cmd.armar);
                if (cmd.hay_origen)
                    (void)eje_fijar_origen(&c->eje, cmd.origen_rad,
                                           eje_planta_cuentas(&c->planta, c->cfg.cuentas_por_vuelta));
            }
            eje_ref_t ref;
            if (eje_ref_tomar(k, &ref)) eje_ref_nueva(&c->eje, ref.pos, ref.vel, ref.dt_muestras_s);

            eje_entrada_t in = { .cuentas = eje_planta_cuentas(&c->planta, c->cfg.cuentas_por_vuelta),
                                 .estop = false, .corriente_a = NAN };
            eje_paso(&c->eje, &in, &out[k]);
            const double tt = (double)c->eje.tick * 0.001;
            const double par = (k == 0 && g_rafaga[0] >= 0.0 && tt >= g_rafaga[0] && tt < g_rafaga[0] + g_rafaga[1])
                                   ? g_rafaga[2] : 0.0;
            const double u = out[k].habilitar ? (double)out[k].duty * c->cfg.u_pleno : 0.0;
            eje_planta_paso(&c->planta, u, par, 0.001);

            if (c->eje.estado == EJE_ACTIVO) {
                /* en coordenadas del firmware: el angulo de la planta mas el origen declarado (cmd/origen) */
                const double e = fabs((double)c->eje.ref_cmd - (c->planta.theta + (double)c->eje.origen_rad));
                if (e > c->err_max) c->err_max = e;
                c->err_ref += e * e; c->n_seg++;
            }
        }
        struct timespec n1;
        clock_gettime(CLOCK_MONOTONIC, &n1);
        const uint32_t t_lazo_us = (uint32_t)(((long long)n1.tv_sec * 1000000000LL + n1.tv_nsec - ns) / 1000);
        for (int k = 0; k < g_n_ejes; k++) {
            carril_t *c = &car[k];
            tel.pos = c->eje.pos; tel.vel = c->eje.vel; tel.u = c->eje.u; tel.duty = out[k].duty;
            tel.corriente_a = NAN; tel.ref_cmd = c->eje.ref_cmd; tel.ref_edad_s = c->eje.ref_edad_s;
            tel.estado = (uint32_t)c->eje.estado; tel.fallas = c->eje.fallas; tel.ticks = c->eje.tick;
            tel.t_lazo_us = t_lazo_us;
            tel.t_lazo_max_us = tel.t_lazo_us;
            tel.jitter_max_us = jitter;
            tel.ticks_perdidos = perdidos;
            eje_tel_publicar(k, &tel);
        }
        if (g_n_per < MAX_PERIODOS && g_periodos_us && g_tlazo_us) {
            g_periodos_us[g_n_per] = (float)((double)periodo_ns * 1e-3);
            g_tlazo_us[g_n_per] = (float)(((double)(n1.tv_sec - n0.tv_sec) * 1e9 + (double)(n1.tv_nsec - n0.tv_nsec)) * 1e-3);
            g_n_per++;
        }
        ticks_total++;

        if (g_duracion_s > 0.0 &&
            (double)(t.tv_sec - t_ini.tv_sec) + (double)(t.tv_nsec - t_ini.tv_nsec) * 1e-9 >= g_duracion_s)
            g_fin = 1;
    }
    for (int k = 0; k < g_n_ejes; k++) {
        carril_t *c = &car[k];
        printf("eje_host[%s]: fin. estado=%s pos=%.4f deg ref_cmd=%.4f deg  refs_recibidas=%u "
               "reconexiones=%u  error_rms_vs_ref_cmd=%.5f deg (n=%ld)\n",
               g_nombres[k], eje_estado_nombre(c->eje.estado), c->eje.pos * 180.0 / M_PI,
               c->eje.ref_cmd * 180.0 / M_PI, eje_uros_refs_recibidas(), eje_uros_reconexiones(),
               c->n_seg ? sqrt(c->err_ref / (double)c->n_seg) * 180.0 / M_PI : 0.0, c->n_seg);
    }
    escribir_estadisticas(ticks_total, perdidos);
    fflush(stdout);
    _exit(0);       /* el hilo principal esta bloqueado en rclc: se sale de golpe */
    return NULL;
}

int main(int argc, char **argv)
{
    const char *eje_nombre = "az";
    int indice = 0, dominio = 0;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--agente") && i + 1 < argc) {
            char *dp = strchr(argv[++i], ':');
            if (dp) { *dp = 0; snprintf(g_agente_puerto, sizeof g_agente_puerto, "%s", dp + 1); }
            snprintf(g_agente_ip, sizeof g_agente_ip, "%s", argv[i]);
        } else if (!strcmp(argv[i], "--eje") && i + 1 < argc) {
            eje_nombre = argv[++i];
            indice = strcmp(eje_nombre, "el") == 0 ? 1 : 0;
        } else if (!strcmp(argv[i], "--dominio") && i + 1 < argc) {
            dominio = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--duracion") && i + 1 < argc) {
            g_duracion_s = atof(argv[++i]);
        } else if (!strcmp(argv[i], "--armar")) {
            g_armar = 1;
        } else if (!strcmp(argv[i], "--kff")) {
            g_kff = 1;
        } else if (!strcmp(argv[i], "--ejes") && i + 1 < argc) {
            g_n_ejes = atoi(argv[++i]);
            if (g_n_ejes < 1 || g_n_ejes > ATP_EJES_MAX) { fprintf(stderr, "--ejes: 1 o 2\n"); return 2; }
        } else if (!strcmp(argv[i], "--tel-hz") && i + 1 < argc) {
            g_tel_hz = atoi(argv[++i]);
        } else if (!strcmp(argv[i], "--eco")) {
            g_eco = 1;
        } else if (!strcmp(argv[i], "--rt")) {
            g_rt = 1;
        } else if (!strcmp(argv[i], "--stats") && i + 1 < argc) {
            g_stats = argv[++i];
        } else if (!strcmp(argv[i], "--theta0-el") && i + 1 < argc) {
            g_theta0_el_rad = atof(argv[++i]) * M_PI / 180.0;
        } else if (!strcmp(argv[i], "--theta0") && i + 1 < argc) {
            g_theta0_rad = atof(argv[++i]) * M_PI / 180.0;
        } else if (!strcmp(argv[i], "--rafaga") && i + 3 < argc) {
            g_rafaga[0] = atof(argv[++i]); g_rafaga[1] = atof(argv[++i]); g_rafaga[2] = atof(argv[++i]);
        } else {
            fprintf(stderr, "uso: %s [--agente IP:PUERTO] [--eje az|el] [--dominio N] [--armar] "
                            "[--duracion S] [--rafaga T0 DUR NM] [--kff] [--theta0 DEG] [--ejes 1|2] [--theta0-el DEG] "
                            "[--tel-hz N] [--eco] [--rt] [--stats ARCHIVO.json]\n", argv[0]);
            return 2;
        }
    }
    if (g_n_ejes == 1) g_nombres[0] = eje_nombre;      /* un eje: el elegido; dos: az y el */
    signal(SIGINT, en_senal);
    signal(SIGTERM, en_senal);
    setvbuf(stdout, NULL, _IOLBF, 0);

    pthread_t th;
    pthread_create(&th, NULL, hilo_lazo, NULL);

    const eje_uros_cfg_t cfg = {
        .n_ejes = g_n_ejes,
        .eje_nombre = { g_nombres[0], g_nombres[1] }, .eje_indice = { g_n_ejes == 1 ? indice : 0, 1 },
        .eco = g_eco != 0, .domain_id = dominio,
        .tel_hz = g_tel_hz, .diag_hz = 2,
        .opciones = opciones_udp, .ahora_ms = ahora_ms, .dormir_ms = dormir_ms,
    };
    printf("eje_host: %d eje(s) '%s%s%s', agente %s:%s, dominio %d%s\n", g_n_ejes, g_nombres[0],
           g_n_ejes > 1 ? "," : "", g_n_ejes > 1 ? g_nombres[1] : "", g_agente_ip, g_agente_puerto, dominio,
           g_eco ? ", eco de latencia" : "");
    eje_uros_correr(&cfg);
    return 0;
}
