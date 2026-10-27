#include "config_eje.h"

#include "sdkconfig.h"

/* pines que solo existen segun la opcion elegida: -1 = no hay */
#ifdef CONFIG_ATP_ENC_PCNT
#define CONFIG_ATP_ENC_A_GPIO_ CONFIG_ATP_ENC_A_GPIO
#define CONFIG_ATP_ENC_B_GPIO_ CONFIG_ATP_ENC_B_GPIO
#else
#define CONFIG_ATP_ENC_A_GPIO_ (-1)
#define CONFIG_ATP_ENC_B_GPIO_ (-1)
#endif
#ifdef CONFIG_ATP_CORR_ADC
#define CORR_R0 CONFIG_ATP_CORR_R_GPIO
#define CORR_L0 CONFIG_ATP_CORR_L_GPIO
#if N_EJES >= 2
#define CORR_R1 CONFIG_ATP_EJE2_CORR_R_GPIO
#define CORR_L1 CONFIG_ATP_EJE2_CORR_L_GPIO
#endif
#else
#define CORR_R0 (-1)
#define CORR_L0 (-1)
#define CORR_R1 (-1)
#define CORR_L1 (-1)
#endif

#define MILLI(x) ((float)(x) * 1.0e-3f)
#define CDEG_A_RAD(x) ((float)(x) * (EJE_PI_F / 18000.0f))

static const pines_eje_t s_pines[N_EJES] = {
    { CONFIG_ATP_ENC_A_GPIO_, CONFIG_ATP_ENC_B_GPIO_, CONFIG_ATP_PWM_A_GPIO, CONFIG_ATP_PWM_B_GPIO,
      CONFIG_ATP_EN_GPIO, CORR_R0, CORR_L0 },
#if N_EJES >= 2
    { CONFIG_ATP_EJE2_ENC_A_GPIO, CONFIG_ATP_EJE2_ENC_B_GPIO, CONFIG_ATP_EJE2_PWM_A_GPIO,
      CONFIG_ATP_EJE2_PWM_B_GPIO, CONFIG_ATP_EJE2_EN_GPIO, CORR_R1, CORR_L1 },
#endif
};

const pines_eje_t *pines_eje(int e) { return &s_pines[e]; }

void config_eje_cargar(int e, eje_cfg_t *c)
{
    eje_cfg_defecto(c);
    c->dt = 0.001f;                                   /* tick de FreeRTOS = 1 ms */
    c->cuentas_por_vuelta = (float)CONFIG_ATP_ENC_CPR;
#ifdef CONFIG_ATP_ENC_INVERTIR
    c->signo_enc = -1;
#endif
#ifdef CONFIG_ATP_MOTOR_INVERTIR
    c->signo_motor = -1;
#endif
    c->kp = MILLI(CONFIG_ATP_KP_MILLI);
    c->ki = MILLI(CONFIG_ATP_KI_MILLI);
    c->kd = MILLI(CONFIG_ATP_KD_MILLI);
    c->tau_d = MILLI(CONFIG_ATP_TAU_D_MS);
    c->kff_v = MILLI(CONFIG_ATP_KFF_V_MILLI);
    c->u_max = MILLI(CONFIG_ATP_U_MAX_MNM);
    c->u_pleno = MILLI(CONFIG_ATP_U_PLENO_MNM);
    c->duty_max = (float)CONFIG_ATP_DUTY_MAX_PCT * 0.01f;
    c->duty_min = (float)CONFIG_ATP_DUTY_MIN_PCT * 0.01f;
    c->vel_max = CDEG_A_RAD(CONFIG_ATP_VEL_MAX_CDEG_S);
    c->acc_limite = CDEG_A_RAD(CONFIG_ATP_ACC_LIMITE_CDEG_S2);
    c->lim_min = CDEG_A_RAD(CONFIG_ATP_LIM_MIN_CDEG);
    c->lim_max = CDEG_A_RAD(CONFIG_ATP_LIM_MAX_CDEG);
    c->lim_margen = CDEG_A_RAD(CONFIG_ATP_LIM_MARGEN_CDEG);
    c->t_watchdog_s = MILLI(CONFIG_ATP_WATCHDOG_MS);
    c->t_retencion_s = MILLI(CONFIG_ATP_RETENCION_MS);
    c->vel_falla = CDEG_A_RAD(CONFIG_ATP_VEL_FALLA_CDEG_S);
#ifdef CONFIG_ATP_CORR_ADC
    c->i_max_a = MILLI(CONFIG_ATP_CORR_MAX_MA);
#else
    c->i_max_a = 0.0f;
#endif
#if N_EJES >= 2
    if (e == 1) {      /* elevacion del banco: lo propio del segundo eje; el resto (ganancias, vigilancia) se comparte */
        c->cuentas_por_vuelta = (float)CONFIG_ATP_EJE2_ENC_CPR;
        c->signo_enc = 1;
        c->signo_motor = 1;
#ifdef CONFIG_ATP_EJE2_ENC_INVERTIR
        c->signo_enc = -1;
#endif
#ifdef CONFIG_ATP_EJE2_MOTOR_INVERTIR
        c->signo_motor = -1;
#endif
        c->u_pleno = MILLI(CONFIG_ATP_EJE2_U_PLENO_MNM);
        c->lim_min = CDEG_A_RAD(CONFIG_ATP_EJE2_LIM_MIN_CDEG);
        c->lim_max = CDEG_A_RAD(CONFIG_ATP_EJE2_LIM_MAX_CDEG);
    }
#else
    (void)e;
#endif
}
