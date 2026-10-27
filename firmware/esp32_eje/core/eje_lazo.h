/* Logica del lazo de UN eje de la montura, en C puro (sin ESP-IDF, sin ROS).
 *
 * La misma fuente corre en tres sitios: el ESP32 (tarea FreeRTOS de 1 kHz),
 * las pruebas del host (ctest) y el nodo micro-ROS sobre Linux. Por eso aqui
 * no hay relojes, hilos ni E/S: quien la llama le da el tiempo en pasos
 * enteros (tick = dt) y las entradas ya leidas, y recibe el mando.
 *
 * Convenciones (las de atp_ros, docs/CONTRATO.md y docs/CONTROL.md):
 *   - angulos en radianes, velocidades en rad/s, par en N m en el EJE DE CARGA;
 *   - el PID es el de pid_controller.py: derivada del error con filtro de
 *     primer orden (tau_d), saturacion y anti-windup por "clamping";
 *   - aritmetica en float: el ESP32 tiene FPU de simple precision (hoja de
 *     datos de Espressif), double iria por software.
 *
 * Concurrencia: NADA aqui es seguro entre hilos. La plataforma pasa la
 * referencia nueva con un buzon y la tarea del lazo la entrega en su tick.
 */
#ifndef EJE_LAZO_H
#define EJE_LAZO_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EJE_PI_F 3.14159265358979323846f

/* ---- PID discreto puro (el de pid_controller.py), reutilizable en el test de vectores */
typedef struct {
    float kp, ki, kd, tau_d, dt, u_max;
    float integral, e_prev, d_filt;
} eje_pid_t;
void  eje_pid_iniciar(eje_pid_t *p, float kp, float ki, float kd, float tau_d,
                      float dt, float u_max);
/* reinicio sin patada: deja e_prev = e0 y la derivada en cero */
void  eje_pid_reiniciar(eje_pid_t *p, float e0);
/* e = ref - medida; `ff` se suma antes de saturar. Devuelve el mando (N m). */
float eje_pid_paso(eje_pid_t *p, float e, float ff);

/* ---- estados ----------------------------------------------------------- */
typedef enum {
    EJE_DESARMADO = 0,  /* driver apagado; espera la orden de armar               */
    EJE_ESPERA    = 1,  /* armado, sin referencia fresca: driver apagado          */
    EJE_ACTIVO    = 2,  /* siguiendo la referencia                                */
    EJE_RETENCION = 3,  /* referencia perdida (watchdog): mantiene la posicion    */
    EJE_FALLA     = 4   /* falla enganchada: driver apagado hasta rearmar         */
} eje_estado_t;

/* ---- fallas (mascara de bits) ------------------------------------------ */
#define EJE_F_ESTOP    0x01u  /* parada de emergencia (GPIO) abierta               */
#define EJE_F_LIMITE   0x02u  /* posicion fuera de los limites +- margen           */
#define EJE_F_VEL      0x04u  /* velocidad medida por encima de vel_falla          */
#define EJE_F_ATASCO   0x08u  /* par casi maximo sostenido sin moverse             */
#define EJE_F_ENCODER  0x10u  /* salto imposible del encoder en un solo tick       */
#define EJE_F_CORR     0x20u  /* sobrecorriente sostenida                          */
#define EJE_F_CFG      0x40u  /* configuracion invalida                            */

/* ---- configuracion ------------------------------------------------------ */
typedef struct {
    float dt;               /* s, periodo del lazo (1 kHz: 0.001)                          */
    /* conversion encoder */
    float cuentas_por_vuelta; /* cuentas por vuelta de la CARGA (PPR x 4 x reduccion)       */
    int   signo_enc;        /* +1 / -1: sentido positivo del encoder                       */
    int   signo_motor;      /* +1 / -1: sentido positivo del mando hacia el driver         */
    /* PID (valores de pid_controller.py) */
    float kp, ki, kd;       /* N m/rad, N m/(rad s), N m s/rad                             */
    float tau_d;            /* s, filtro de la derivada                                    */
    float kff_v;            /* N m s/rad, pre-alimentacion de velocidad (0 = apagada)      */
    float u_max;            /* N m, tope de par del lazo                                   */
    float u_pleno;          /* N m que da el driver al 100 % de PWM (calibracion)          */
    float duty_max;         /* 0..1, tope de PWM (bootstrap del BTS7960: < 1)              */
    /* referencia */
    float vel_max;          /* rad/s, tope de velocidad de la referencia comandada         */
    float extrap_max_s;     /* s, cuanto se extrapola la referencia con su velocidad       */
    float lim_min, lim_max; /* rad, limites de software de la referencia                   */
    float lim_margen;       /* rad, margen de falla fuera de los limites                   */
    float acc_limite;       /* rad/s^2, frenado de la referencia al acercarse a un limite  */
    /* watchdog de referencia */
    float t_watchdog_s;     /* s sin referencia antes de RETENCION (atp_hw: 0.5 s)         */
    float t_retencion_s;    /* s manteniendo posicion antes de apagar el driver            */
    /* proteccion */
    float vel_falla;        /* rad/s, velocidad que enclava falla                          */
    float salto_max_vel;    /* rad/s, |d theta|/dt imposible entre dos ticks               */
    float atasco_frac;      /* fraccion de u_max que cuenta como "par casi maximo"         */
    float atasco_vel;       /* rad/s, por debajo de esto se considera parado               */
    float atasco_s;         /* s de par alto y parado para enclavar falla                  */
    float i_max_a;          /* A, sobrecorriente (<=0: sin sensor)                         */
    float i_max_s;          /* s de sobrecorriente para enclavar falla                     */
    float tau_vel;          /* s, filtro de la velocidad medida                            */
    float duty_min;         /* 0..1, zona muerta del driver/motor (0 = sin compensar)      */
} eje_cfg_t;

/* ---- entrada de cada tick ---------------------------------------------- */
typedef struct {
    int32_t cuentas;        /* cuenta acumulada del encoder (relativa al arranque)         */
    bool    estop;          /* true = parada de emergencia ABIERTA (peligro)               */
    float   corriente_a;    /* A medidos; NAN si no hay sensor                             */
} eje_entrada_t;

/* ---- salida de cada tick ----------------------------------------------- */
typedef struct {
    float duty;             /* -1..1 firmado, ya con signo_motor y topes                   */
    bool  habilitar;        /* true: driver habilitado; false: salidas a cero y EN bajo    */
} eje_salida_t;

/* ---- estado interno (publico para telemetria y pruebas) ----------------- */
typedef struct {
    eje_cfg_t cfg;
    eje_estado_t estado;
    uint32_t fallas;        /* mascara EJE_F_* enganchada                                  */
    uint32_t fallas_vivas;  /* mascara de la condicion ACTUAL (sin enganchar)              */
    uint32_t tick;          /* ticks desde eje_iniciar                                     */
    /* medida */
    float pos;              /* rad (con origen aplicado)                                   */
    float vel;              /* rad/s filtrada                                              */
    float origen_rad;       /* rad que se suman a cuentas*2pi/cpr                          */
    int32_t cuentas_prev;
    bool  tiene_prev;
    /* referencia */
    float ref_obj;          /* rad, ultimo objetivo recibido (sin limitar)                 */
    float ref_obj_vel;      /* rad/s, su velocidad                                         */
    uint32_t ref_obj_tick;  /* tick en que llego                                           */
    uint32_t ref_prev_tick;
    float ref_prev;
    bool  tiene_ref;
    float ref_cmd;          /* rad, referencia comandada (limitada y con tope de velocidad)*/
    float ref_cmd_vel;      /* rad/s                                                       */
    /* PID */
    eje_pid_t pid;
    float u;                /* N m, ultimo mando (con topes)                               */
    /* vigilancia */
    uint32_t t_retencion;   /* ticks en RETENCION                                          */
    uint32_t t_atasco;      /* ticks con par alto y parado                                 */
    uint32_t t_corr;        /* ticks con sobrecorriente                                    */
    bool  arm_pedido;       /* orden de armar por resolver en el proximo paso              */
    float ref_edad_s;       /* s desde la ultima referencia (telemetria)                   */
} eje_t;

/* valores por defecto coherentes con atp_ros (ver eje_lazo.c) */
void eje_cfg_defecto(eje_cfg_t *c);
/* 0 si es valida; si no, un codigo negativo (y eje_iniciar enclava EJE_F_CFG) */
int  eje_cfg_validar(const eje_cfg_t *c);

void eje_iniciar(eje_t *e, const eje_cfg_t *c);

/* Orden de armar/desarmar (la plataforma la recibe de ROS). Desarmar es
 * inmediato. Armar se resuelve en el siguiente eje_paso y solo se acepta sin
 * fallas VIVAS (paro cerrado, dentro de limites...); desde FALLA, armar=true
 * es el REARME. Una orden rechazada no queda pendiente: hay que repetirla. */
void eje_armar(eje_t *e, bool armar);

/* Define la posicion actual como `rad` (referenciado). Solo con el driver
 * apagado (DESARMADO, ESPERA o FALLA); devuelve false si no se acepta. */
bool eje_fijar_origen(eje_t *e, float rad, int32_t cuentas_ahora);

/* Referencia nueva (rad). `vel` = velocidad de la referencia en rad/s o NAN
 * si no se conoce (entonces se estima con la llegada de dos muestras);
 * `dt_muestras_s` > 0: separacion entre las dos muestras segun SUS sellos
 * (mas exacta que la hora de llegada); <= 0: se usa la llegada. */
void eje_ref_nueva(eje_t *e, float pos, float vel, float dt_muestras_s);

/* Un paso del lazo (llamar cada cfg.dt). */
void eje_paso(eje_t *e, const eje_entrada_t *in, eje_salida_t *out);

/* ---- piezas sueltas, probadas por separado ------------------------------ */
float eje_cuentas_a_rad(const eje_cfg_t *c, int32_t cuentas);
int32_t eje_rad_a_cuentas(const eje_cfg_t *c, float rad);
float eje_limitar(float x, float lo, float hi);
/* tope de velocidad: mueve `actual` hacia `objetivo` sin pasar de vmax*dt */
float eje_slew(float actual, float objetivo, float vmax, float dt);

/* nombre de un estado, para el diagnostico */
const char *eje_estado_nombre(eje_estado_t s);

#ifdef __cplusplus
}
#endif
#endif
