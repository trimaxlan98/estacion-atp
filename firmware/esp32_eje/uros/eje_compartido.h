/* Datos que cruzan entre la tarea del lazo (1 kHz) y la tarea micro-ROS.
 *
 * Interfaz unica; la implementacion es de cada plataforma:
 *   - ESP32: main/compartido_esp.c (seccion critica de FreeRTOS, ~1 us)
 *   - Linux: host/compartido_host.c (mutex POSIX)
 * Ninguna funcion bloquea mas que la copia de unas decenas de bytes: la tarea
 * del lazo nunca espera a ROS.
 */
#ifndef EJE_COMPARTIDO_H
#define EJE_COMPARTIDO_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Un lazo puede gobernar hasta ATP_EJES_MAX ejes (banco de escritorio: acimut y
 * elevacion en un ESP32). Todas las funciones llevan el indice del eje `e`
 * (0 = primero, 1 = segundo); con un solo eje se usa siempre e = 0. */
#define ATP_EJES_MAX 2

/* ---- lazo -> ROS: telemetria ---- */
typedef struct {
    float pos, vel;            /* rad, rad/s del eje (con origen)                 */
    float u;                   /* N m, mando aplicado (effort de JointState)      */
    float duty;                /* -1..1                                           */
    float corriente_a;         /* NAN sin sensor                                  */
    float ref_cmd;             /* rad, referencia comandada (tras limites/slew)   */
    float ref_edad_s;          /* s desde la ultima referencia                    */
    uint32_t estado;           /* eje_estado_t                                    */
    uint32_t fallas;           /* mascara EJE_F_*                                 */
    uint32_t ticks;            /* ticks del lazo desde el arranque                */
    uint32_t t_lazo_us;        /* duracion del ultimo tick                        */
    uint32_t t_lazo_max_us;    /* maximo desde la ultima lectura (se reinicia)    */
    int32_t  jitter_max_us;    /* |periodo real - 1000| maximo, idem              */
    uint32_t ticks_perdidos;   /* ticks que no cumplieron el plazo                */
    uint32_t abs_err;          /* errores de lectura del encoder absoluto         */
} eje_tel_t;

void eje_tel_publicar(int e, const eje_tel_t *t);
/* copia la ultima telemetria y reinicia los maximos; false si aun no hay */
bool eje_tel_leer(int e, eje_tel_t *t);

/* ---- ROS -> lazo: referencia y ordenes ---- */
typedef struct {
    float pos;                 /* rad                                             */
    float vel;                 /* rad/s o NAN                                     */
    float dt_muestras_s;       /* separacion entre muestras segun sus sellos, o 0 */
} eje_ref_t;

void eje_ref_enviar(int e, const eje_ref_t *r);
/* toma la referencia pendiente (true) o nada (false) */
bool eje_ref_tomar(int e, eje_ref_t *r);

void eje_cmd_armar(int e, bool armar);
void eje_cmd_origen(int e, float rad);
typedef struct {
    bool hay_armar, armar;
    bool hay_origen; float origen_rad;
} eje_cmd_t;
bool eje_cmd_tomar(int e, eje_cmd_t *c);

#ifdef __cplusplus
}
#endif
#endif
