/* Datos compartidos lazo <-> micro-ROS en Linux (mutex POSIX). Misma interfaz
 * que main/compartido_esp.c: ver uros/eje_compartido.h. */
#include <pthread.h>

#include "eje_compartido.h"

static pthread_mutex_t s_mx = PTHREAD_MUTEX_INITIALIZER;

static eje_tel_t s_tel[ATP_EJES_MAX];
static bool s_tel_valida[ATP_EJES_MAX];
static bool s_tel_reiniciar_maximos[ATP_EJES_MAX];
static eje_ref_t s_ref[ATP_EJES_MAX];
static bool s_ref_nueva[ATP_EJES_MAX];
static eje_cmd_t s_cmd[ATP_EJES_MAX];

static bool indice_valido(int e) { return e >= 0 && e < ATP_EJES_MAX; }

void eje_tel_publicar(int e, const eje_tel_t *t)
{
    if (!indice_valido(e)) return;
    pthread_mutex_lock(&s_mx);
    const uint32_t mx_t = s_tel[e].t_lazo_max_us;
    const int32_t mx_j = s_tel[e].jitter_max_us;
    const bool reiniciar = s_tel_reiniciar_maximos[e] || !s_tel_valida[e];
    s_tel[e] = *t;
    if (!reiniciar) {                       /* maximos corridos hasta que alguien lea */
        if (mx_t > s_tel[e].t_lazo_max_us) s_tel[e].t_lazo_max_us = mx_t;
        if (mx_j > s_tel[e].jitter_max_us) s_tel[e].jitter_max_us = mx_j;
    }
    s_tel_reiniciar_maximos[e] = false;
    s_tel_valida[e] = true;
    pthread_mutex_unlock(&s_mx);
}

bool eje_tel_leer(int e, eje_tel_t *t)
{
    if (!indice_valido(e)) return false;
    bool ok;
    pthread_mutex_lock(&s_mx);
    ok = s_tel_valida[e];
    if (ok) {
        *t = s_tel[e];
        s_tel_reiniciar_maximos[e] = true;
    }
    pthread_mutex_unlock(&s_mx);
    return ok;
}

void eje_ref_enviar(int e, const eje_ref_t *r)
{
    if (!indice_valido(e)) return;
    pthread_mutex_lock(&s_mx);
    s_ref[e] = *r;
    s_ref_nueva[e] = true;
    pthread_mutex_unlock(&s_mx);
}

bool eje_ref_tomar(int e, eje_ref_t *r)
{
    if (!indice_valido(e)) return false;
    bool hay;
    pthread_mutex_lock(&s_mx);
    hay = s_ref_nueva[e];
    if (hay) {
        *r = s_ref[e];
        s_ref_nueva[e] = false;
    }
    pthread_mutex_unlock(&s_mx);
    return hay;
}

void eje_cmd_armar(int e, bool armar)
{
    if (!indice_valido(e)) return;
    pthread_mutex_lock(&s_mx);
    s_cmd[e].hay_armar = true;
    s_cmd[e].armar = armar;
    pthread_mutex_unlock(&s_mx);
}

void eje_cmd_origen(int e, float rad)
{
    if (!indice_valido(e)) return;
    pthread_mutex_lock(&s_mx);
    s_cmd[e].hay_origen = true;
    s_cmd[e].origen_rad = rad;
    pthread_mutex_unlock(&s_mx);
}

bool eje_cmd_tomar(int e, eje_cmd_t *c)
{
    if (!indice_valido(e)) return false;
    bool hay;
    pthread_mutex_lock(&s_mx);
    hay = s_cmd[e].hay_armar || s_cmd[e].hay_origen;
    if (hay) {
        *c = s_cmd[e];
        s_cmd[e].hay_armar = false;
        s_cmd[e].hay_origen = false;
    }
    pthread_mutex_unlock(&s_mx);
    return hay;
}
