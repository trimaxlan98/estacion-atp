/* Nodo micro-ROS (rclc) de UNO o DOS ejes. Mismo codigo en el ESP32 y en Linux.
 *
 * Con un eje (ATP_EJES=1) el nodo es `esp32_<eje>` en el espacio `/atp/mcu/<eje>`.
 * Con dos (banco de escritorio) es `esp32_banco` en `/atp/mcu` y los topicos son
 * los MISMOS por eje (`/atp/mcu/az/...` y `/atp/mcu/el/...`): el lado PC no ve la
 * diferencia (mcu_merge, CONTRATO 7).
 *
 * Interfaz ROS 2 (espacio de nombres `/atp/mcu/<eje>`, <eje> = az | el):
 *
 *   suscribe  /atp/reference            sensor_msgs/JointState   (absoluto: el contrato, CONTRATO.md 7)
 *                                       position[i] en rad; i = posicion de <eje> en `name`,
 *                                       o el indice fijo del firmware si `name` viene vacio.
 *                                       velocity[i] opcional (rad/s); sin ella se estima.
 *   suscribe  ~/cmd/enable              std_msgs/Bool            true = armar (rearma tras una falla)
 *   suscribe  ~/cmd/origen              std_msgs/Float64         declara que el eje esta AHORA en ese
 *                                                                angulo (rad); solo con el motor apagado
 *   publica   ~/joint_states            sensor_msgs/JointState   name=[<eje>], position/velocity del
 *                                       encoder, effort = mando en N m (reliable, como /joint_states)
 *   publica   ~/diag                    std_msgs/Float32MultiArray (best effort); ver INDICE_DIAG
 *
 * Opcional (`eco`, apagado en el firmware): publica /atp/mcu/eco (std_msgs/Int64) con el
 * sello de cada /atp/reference recibida, en ns, para medir la latencia de ida y vuelta
 * del enlace (banco HIL, scripts/prueba_hil_lockstep.sh).
 *
 * La reconexion con el agente es automatica y NO detiene el lazo: sin
 * referencia, el watchdog del lazo para el motor.
 */
#ifndef EJE_UROS_H
#define EJE_UROS_H

#include <stdbool.h>
#include <stdint.h>

#include <rcl/rcl.h>

#ifdef __cplusplus
extern "C" {
#endif

/* orden de los campos de ~/diag (Float32MultiArray.data) */
enum {
    DIAG_ESTADO = 0,      /* eje_estado_t: 0 DESARMADO 1 ESPERA 2 ACTIVO 3 RETENCION 4 FALLA */
    DIAG_FALLAS,          /* mascara EJE_F_*                                                 */
    DIAG_POS,             /* rad                                                             */
    DIAG_VEL,             /* rad/s                                                           */
    DIAG_U,               /* N m                                                             */
    DIAG_DUTY,            /* -1..1                                                           */
    DIAG_CORRIENTE,       /* A (NaN sin sensor)                                              */
    DIAG_REF_CMD,         /* rad, referencia comandada                                       */
    DIAG_REF_EDAD,        /* s desde la ultima referencia                                    */
    DIAG_T_LAZO_US,       /* duracion del ultimo tick del lazo                               */
    DIAG_T_LAZO_MAX_US,   /* maximo desde el diag anterior                                   */
    DIAG_JITTER_MAX_US,   /* maximo |periodo - 1 ms| desde el diag anterior                  */
    DIAG_TICKS_PERDIDOS,  /* ticks que no cumplieron el plazo                                */
    DIAG_REFS_RECIBIDAS,  /* mensajes de referencia recibidos                                */
    DIAG_ABS_ERR,         /* errores de lectura del encoder absoluto                         */
    DIAG_UPTIME_S,        /* s desde el arranque del lazo                                    */
    DIAG_N
};

#include "eje_compartido.h"

typedef struct {
    int n_ejes;                 /* 1 | 2 (<= ATP_EJES_MAX)                                  */
    const char *eje_nombre[ATP_EJES_MAX];  /* "az" | "el" (nombre en JointState y topicos)  */
    int eje_indice[ATP_EJES_MAX];          /* 0 | 1: elemento de /atp/reference si `name` viene vacio */
    bool eco;                   /* publica el eco de las referencias (medida de latencia)   */
    int domain_id;              /* ROS_DOMAIN_ID que se manda al agente                     */
    int tel_hz;                 /* joint_states                                             */
    int diag_hz;
    /* opciones de la plataforma: p. ej. direccion UDP del agente en Linux (puede ser NULL) */
    bool (*opciones)(rcl_init_options_t *io);
    uint32_t (*ahora_ms)(void);       /* reloj monotono                                      */
    void (*dormir_ms)(uint32_t ms);
} eje_uros_cfg_t;

/* No retorna: espera al agente, crea las entidades, gira y se reconecta. */
void eje_uros_correr(const eje_uros_cfg_t *c);

/* Contadores para pruebas/diagnostico */
uint32_t eje_uros_refs_recibidas(void);
uint32_t eje_uros_reconexiones(void);

#ifdef __cplusplus
}
#endif
#endif
