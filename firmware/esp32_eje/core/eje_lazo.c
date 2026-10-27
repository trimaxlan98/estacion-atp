/* Lazo de un eje: ver eje_lazo.h. C puro; solo <math.h>. */
#include "eje_lazo.h"

#include <math.h>

#define DEG2RAD(x) ((x) * (EJE_PI_F / 180.0f))

/* ------------------------------------------------------------------------
 * Configuracion
 * ---------------------------------------------------------------------- */
void eje_cfg_defecto(eje_cfg_t *c)
{
    c->dt = 0.001f;                    /* 1 kHz (requisito de la mision)               */
    /* ASUMIDO: encoder de 1000 PPR en el motor x4 (cuadratura) x reductor 10:1.
     * Hay que poner el valor de la montura real (docs/CONTROL.md, seccion 8). */
    c->cuentas_por_vuelta = 40000.0f;
    c->signo_enc = 1;
    c->signo_motor = 1;
    /* PID: los valores de pid_controller.py (declare_num de kp, ki, kd, tau). */
    c->kp = 25.0f;
    c->ki = 2.0f;
    c->kd = 8.0f;
    c->tau_d = 0.08f;
    c->kff_v = 0.0f;                   /* apagada: igual que el PID de atp_ros          */
    c->u_max = 8.0f;                   /* N m, u_max de mount_sim.py                    */
    c->u_pleno = 8.0f;                 /* ASUMIDO: 100 % de PWM = u_max; calibrar       */
    c->duty_max = 0.95f;               /* ASUMIDO: el BTS7960 necesita recargar el
                                          bootstrap, no se le pide 100 % continuo       */
    c->vel_max = DEG2RAD(12.0f);       /* ASUMIDO: sobre los 9 deg/s del pase cenital N2 */
    c->extrap_max_s = 0.15f;           /* 3 periodos de la referencia a 20 Hz           */
    c->lim_min = DEG2RAD(-5.0f);       /* ASUMIDO: acimut -5..365 deg; ajustar al eje   */
    c->lim_max = DEG2RAD(365.0f);
    c->lim_margen = DEG2RAD(2.0f);     /* ASUMIDO                                       */
    c->acc_limite = 0.1f;              /* ASUMIDO: sobrepaso del lazo ~ J a/kp = 0.46 deg;
                                          frena 12 deg/s en v^2/(2a) = 12.6 deg          */
    c->t_watchdog_s = 0.5f;            /* src/atp_hw/README.md, punto 4                 */
    c->t_retencion_s = 2.0f;           /* ASUMIDO                                       */
    c->vel_falla = DEG2RAD(30.0f);     /* ASUMIDO: 5x los 6 deg/s del rotor falso       */
    c->salto_max_vel = 5.0f;           /* rad/s: 5 mrad por tick, imposible en la montura */
    c->atasco_frac = 0.9f;
    c->atasco_vel = DEG2RAD(0.5f);
    c->atasco_s = 1.0f;
    c->i_max_a = 0.0f;                 /* sin sensor de corriente                       */
    c->i_max_s = 0.05f;
    c->tau_vel = 0.02f;                /* ASUMIDO: 20 ms filtra la cuantizacion del encoder */
    c->duty_min = 0.0f;
}

int eje_cfg_validar(const eje_cfg_t *c)
{
    if (!(c->dt > 0.0f && c->dt <= 0.05f)) return -1;
    if (!(c->cuentas_por_vuelta > 0.0f)) return -2;
    if (!((c->signo_enc == 1 || c->signo_enc == -1) &&
          (c->signo_motor == 1 || c->signo_motor == -1))) return -3;
    if (!(c->kp >= 0.0f && c->ki >= 0.0f && c->kd >= 0.0f && c->tau_d >= 0.0f)) return -4;
    if (!(c->u_max > 0.0f && c->u_pleno > 0.0f)) return -5;
    if (!(c->duty_max > 0.0f && c->duty_max <= 1.0f)) return -6;
    if (!(c->duty_min >= 0.0f && c->duty_min < c->duty_max)) return -7;
    if (!(c->lim_min < c->lim_max && c->lim_margen >= 0.0f && c->acc_limite > 0.0f)) return -8;
    if (!(c->vel_max > 0.0f && c->vel_falla > 0.0f && c->salto_max_vel > 0.0f)) return -9;
    if (!(c->t_watchdog_s > 0.0f && c->t_retencion_s >= 0.0f)) return -10;
    if (!(c->tau_vel >= 0.0f && c->extrap_max_s >= 0.0f)) return -11;
    return 0;
}

/* ------------------------------------------------------------------------
 * Piezas sueltas
 * ---------------------------------------------------------------------- */
float eje_limitar(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

float eje_slew(float actual, float objetivo, float vmax, float dt)
{
    const float paso = vmax * dt;
    return actual + eje_limitar(objetivo - actual, -paso, paso);
}

float eje_cuentas_a_rad(const eje_cfg_t *c, int32_t cuentas)
{
    return (float)c->signo_enc * (float)cuentas * (2.0f * EJE_PI_F / c->cuentas_por_vuelta);
}

int32_t eje_rad_a_cuentas(const eje_cfg_t *c, float rad)
{
    return (int32_t)lroundf((float)c->signo_enc * rad * c->cuentas_por_vuelta / (2.0f * EJE_PI_F));
}

const char *eje_estado_nombre(eje_estado_t s)
{
    switch (s) {
    case EJE_DESARMADO: return "DESARMADO";
    case EJE_ESPERA:    return "ESPERA";
    case EJE_ACTIVO:    return "ACTIVO";
    case EJE_RETENCION: return "RETENCION";
    case EJE_FALLA:     return "FALLA";
    }
    return "?";
}

/* ------------------------------------------------------------------------
 * PID: pid_controller.py, linea a linea
 *   d_bruta = (e - e_prev)/dt;  alfa = dt/(tau + dt);  d_filt += alfa (d_bruta - d_filt)
 *   u_raw = kp e + ki I + kd d_filt;  u = clip(u_raw, +-u_max)
 *   clamping: no integra si u_raw satura y u_raw*e > 0;  I += e dt
 * ---------------------------------------------------------------------- */
void eje_pid_iniciar(eje_pid_t *p, float kp, float ki, float kd, float tau_d,
                     float dt, float u_max)
{
    p->kp = kp; p->ki = ki; p->kd = kd; p->tau_d = tau_d;
    p->dt = dt; p->u_max = u_max;
    p->integral = 0.0f; p->e_prev = 0.0f; p->d_filt = 0.0f;
}

void eje_pid_reiniciar(eje_pid_t *p, float e0)
{
    p->integral = 0.0f;
    p->e_prev = e0;
    p->d_filt = 0.0f;
}

float eje_pid_paso(eje_pid_t *p, float e, float ff)
{
    const float d_bruta = (e - p->e_prev) / p->dt;
    const float alfa = p->dt / (p->tau_d + p->dt);
    p->d_filt += alfa * (d_bruta - p->d_filt);
    const float u_raw = p->kp * e + p->ki * p->integral + p->kd * p->d_filt + ff;
    const float u = eje_limitar(u_raw, -p->u_max, p->u_max);
    const int sat = fabsf(u_raw) > p->u_max + 1e-12f;
    const int congelar = sat && (u_raw * e > 0.0f);
    if (!congelar) p->integral += e * p->dt;
    p->e_prev = e;
    return u;
}

/* ------------------------------------------------------------------------
 * Lazo del eje
 * ---------------------------------------------------------------------- */
static float u_efectivo(const eje_cfg_t *c)
{
    const float u_duty = c->duty_max * c->u_pleno;   /* lo que el driver puede dar */
    return c->u_max < u_duty ? c->u_max : u_duty;
}

void eje_iniciar(eje_t *e, const eje_cfg_t *c)
{
    *e = (eje_t){0};
    e->cfg = *c;
    e->estado = EJE_DESARMADO;
    eje_pid_iniciar(&e->pid, c->kp, c->ki, c->kd, c->tau_d, c->dt, u_efectivo(c));
    e->ref_edad_s = 1.0e9f;
    if (eje_cfg_validar(c) != 0) {
        e->fallas = EJE_F_CFG;
        e->fallas_vivas = EJE_F_CFG;
        e->estado = EJE_FALLA;
    }
}

void eje_armar(eje_t *e, bool armar)
{
    if (!armar) {
        e->arm_pedido = false;
        if (e->fallas == 0u) e->estado = EJE_DESARMADO;
        return;
    }
    /* la orden se resuelve en el SIGUIENTE eje_paso, con las entradas frescas
     * (p. ej. el paro de emergencia que se acaba de cerrar): ahi se acepta o se rechaza */
    e->arm_pedido = true;
}

bool eje_fijar_origen(eje_t *e, float rad, int32_t cuentas_ahora)
{
    if (e->estado != EJE_DESARMADO && e->estado != EJE_ESPERA && e->estado != EJE_FALLA)
        return false;                                /* con el motor en marcha no se mueve el cero */
    e->origen_rad = rad - eje_cuentas_a_rad(&e->cfg, cuentas_ahora);
    e->pos = rad;
    return true;
}

void eje_ref_nueva(eje_t *e, float pos, float vel, float dt_muestras_s)
{
    if (!isfinite(pos)) return;                      /* un NaN nunca llega al lazo */
    float v = vel;
    if (!isfinite(v)) {
        v = 0.0f;
        if (e->tiene_ref) {
            const float dts = dt_muestras_s > 0.0f
                ? dt_muestras_s
                : (float)(e->tick - e->ref_obj_tick) * e->cfg.dt;
            /* solo se fia de dos muestras entre 5 ms y 1 s; fuera de eso, ni idea */
            if (dts >= 0.005f && dts <= 1.0f) v = (pos - e->ref_obj) / dts;
        }
    }
    /* la velocidad de la referencia no pasa del tope de la montura: si no, la
     * extrapolacion adelantaria un objetivo que el slew no podra seguir */
    v = eje_limitar(v, -e->cfg.vel_max, e->cfg.vel_max);
    e->ref_prev = e->ref_obj;
    e->ref_prev_tick = e->ref_obj_tick;
    e->ref_obj = pos;
    e->ref_obj_vel = v;
    e->ref_obj_tick = e->tick;
    e->tiene_ref = true;
}

static float objetivo_ref(const eje_t *e)
{
    float t = (float)(e->tick - e->ref_obj_tick) * e->cfg.dt;
    if (t > e->cfg.extrap_max_s) t = e->cfg.extrap_max_s;
    const float r = e->ref_obj + e->ref_obj_vel * t;
    return eje_limitar(r, e->cfg.lim_min, e->cfg.lim_max);
}

static void apagar(eje_t *e, eje_salida_t *out)
{
    e->u = 0.0f;
    out->duty = 0.0f;
    out->habilitar = false;
}

void eje_paso(eje_t *e, const eje_entrada_t *in, eje_salida_t *out)
{
    const eje_cfg_t *c = &e->cfg;
    const float dt = c->dt;
    e->tick++;

    /* ---- medida ---- */
    uint32_t vivas = (e->fallas_vivas & EJE_F_CFG);   /* la de configuracion no se va */
    e->pos = eje_cuentas_a_rad(c, in->cuentas) + e->origen_rad;
    if (e->tiene_prev) {
        const int32_t dc = in->cuentas - e->cuentas_prev;
        const float v_bruta = eje_cuentas_a_rad(c, dc) / dt;
        if (fabsf(v_bruta) > c->salto_max_vel) {
            vivas |= EJE_F_ENCODER;                   /* lectura imposible: no se filtra */
        } else {
            const float alfa = dt / (c->tau_vel + dt);
            e->vel += alfa * (v_bruta - e->vel);
        }
    }
    e->cuentas_prev = in->cuentas;
    e->tiene_prev = true;
    e->ref_edad_s = e->tiene_ref ? (float)(e->tick - e->ref_obj_tick) * dt : 1.0e9f;

    /* ---- condiciones de falla (vivas) ---- */
    if (in->estop) vivas |= EJE_F_ESTOP;
    if (e->pos < c->lim_min - c->lim_margen || e->pos > c->lim_max + c->lim_margen)
        vivas |= EJE_F_LIMITE;
    if (fabsf(e->vel) > c->vel_falla) vivas |= EJE_F_VEL;
    if (c->i_max_a > 0.0f && isfinite(in->corriente_a) && fabsf(in->corriente_a) > c->i_max_a)
        e->t_corr++;
    else
        e->t_corr = 0;
    if ((float)e->t_corr * dt >= c->i_max_s && e->t_corr > 0) vivas |= EJE_F_CORR;

    const bool marcha = (e->estado == EJE_ACTIVO || e->estado == EJE_RETENCION);
    const float u_ef = u_efectivo(c);
    if (marcha && fabsf(e->u) > c->atasco_frac * u_ef && fabsf(e->vel) < c->atasco_vel)
        e->t_atasco++;
    else
        e->t_atasco = 0;
    if ((float)e->t_atasco * dt >= c->atasco_s && e->t_atasco > 0) vivas |= EJE_F_ATASCO;

    e->fallas_vivas = vivas;
    if (e->arm_pedido) {
        e->arm_pedido = false;                        /* se consume, aceptada o no */
        if (vivas == 0u && (e->estado == EJE_DESARMADO || e->estado == EJE_FALLA)) {
            e->fallas = 0u;                           /* arma, o rearma tras una falla */
            e->estado = EJE_ESPERA;
        }
    }
    if (vivas != 0u) {
        e->fallas |= vivas;                           /* se engancha */
        e->estado = EJE_FALLA;
    }

    /* ---- maquina de estados ---- */
    switch (e->estado) {
    case EJE_DESARMADO:
    case EJE_FALLA:
        eje_pid_reiniciar(&e->pid, 0.0f);
        apagar(e, out);
        return;
    case EJE_ESPERA:
        eje_pid_reiniciar(&e->pid, 0.0f);
        if (e->tiene_ref && e->ref_edad_s <= c->t_watchdog_s) {
            e->ref_cmd = e->pos;                      /* arranque sin salto: parte de donde esta */
            e->ref_cmd_vel = 0.0f;
            eje_pid_reiniciar(&e->pid, 0.0f);
            e->estado = EJE_ACTIVO;
            break;
        }
        apagar(e, out);
        return;
    case EJE_ACTIVO:
        if (e->ref_edad_s > c->t_watchdog_s) {        /* perdio la referencia: a mantener */
            e->estado = EJE_RETENCION;
            e->t_retencion = 0;
        }
        break;
    case EJE_RETENCION:
        if (e->ref_edad_s <= c->t_watchdog_s) {       /* volvio: retoma desde donde esta */
            e->estado = EJE_ACTIVO;
        } else if ((float)(++e->t_retencion) * dt >= c->t_retencion_s) {
            e->estado = EJE_ESPERA;                   /* mucho tiempo sin referencia: motor libre */
            eje_pid_reiniciar(&e->pid, 0.0f);
            apagar(e, out);
            return;
        }
        break;
    }

    /* ---- referencia comandada ---- */
    float ff = 0.0f;
    if (e->estado == EJE_ACTIVO) {
        const float obj = objetivo_ref(e);
        /* cerca de un limite la referencia frena con perfil v = sqrt(2 a d): asi
         * el eje no llega a el con la velocidad de crucero y el sobrepaso del
         * lazo (J a/kp: el error de seguimiento es proporcional a la aceleracion
         * de la referencia) no pasa del margen de falla */
        float vlim = c->vel_max;
        const float d_lim = (obj >= e->ref_cmd) ? (c->lim_max - e->ref_cmd) : (e->ref_cmd - c->lim_min);
        const float v_frenado = sqrtf(2.0f * c->acc_limite * (d_lim > 0.0f ? d_lim : 0.0f));
        if (v_frenado < vlim) vlim = v_frenado;
        const float nuevo = eje_slew(e->ref_cmd, obj, vlim, dt);
        e->ref_cmd_vel = (nuevo - e->ref_cmd) / dt;
        e->ref_cmd = nuevo;
        ff = c->kff_v * e->ref_cmd_vel;
    } else {                                          /* RETENCION: referencia congelada */
        e->ref_cmd_vel = 0.0f;
    }

    /* ---- PID ---- */
    float u = eje_pid_paso(&e->pid, e->ref_cmd - e->pos, ff);

    /* tope de software: no se empuja mas hacia afuera del limite */
    if ((e->pos >= c->lim_max && u > 0.0f) || (e->pos <= c->lim_min && u < 0.0f)) u = 0.0f;
    e->u = u;

    /* ---- mando -> PWM ---- */
    float duty = u / c->u_pleno;
    if (c->duty_min > 0.0f && fabsf(duty) > 0.002f)
        duty = copysignf(c->duty_min + (1.0f - c->duty_min) * fabsf(duty), duty);
    duty = eje_limitar(duty, -c->duty_max, c->duty_max);
    out->duty = duty * (float)c->signo_motor;
    out->habilitar = true;
}
