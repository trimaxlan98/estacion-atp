/* Nodo micro-ROS de un eje: ver eje_uros.h. */
#include "eje_uros.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <micro_ros_utilities/type_utilities.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/int64.h>

#include "eje_compartido.h"

#define RC_OK(x) ((x) == RCL_RET_OK)
/* limpieza/publicacion sin accion posible si falla (gcc ignora el cast a void de warn_unused_result) */
#define IGNORAR(x) do { rcl_ret_t r_ = (x); (void)r_; } while (0)
#define NUM_MANEJADORES (1 + 2 * ATP_EJES_MAX + 2)   /* /atp/reference + (enable, origen) por eje + 2 temporizadores */
#define PING_CADA_MS 1000u
#define SYNC_CADA_MS 30000u

typedef enum { ESPERANDO_AGENTE, AGENTE_DISPONIBLE, CONECTADO, DESCONECTADO } estado_conexion_t;

/* Todo estatico: un solo nodo por firmware, y asi no hay malloc en el lazo de arranque. */
static const eje_uros_cfg_t *g_cfg;
static rclc_support_t g_support;
static rcl_node_t g_node;
static rcl_publisher_t g_pub_js[ATP_EJES_MAX], g_pub_diag[ATP_EJES_MAX], g_pub_eco;
static rcl_subscription_t g_sub_ref, g_sub_en[ATP_EJES_MAX], g_sub_or[ATP_EJES_MAX];
static rcl_timer_t g_t_tel, g_t_diag;
static rclc_executor_t g_exec;
static rcl_allocator_t g_alloc;

static sensor_msgs__msg__JointState g_js[ATP_EJES_MAX], g_ref;
static std_msgs__msg__Float32MultiArray g_diag[ATP_EJES_MAX];
static std_msgs__msg__Bool g_en[ATP_EJES_MAX];
static std_msgs__msg__Float64 g_or[ATP_EJES_MAX];
static std_msgs__msg__Int64 g_eco;

static uint32_t g_refs = 0, g_reconex = 0;
static int64_t g_ref_stamp_prev_ns = 0;
static bool g_memoria_lista = false;

uint32_t eje_uros_refs_recibidas(void) { return g_refs; }
uint32_t eje_uros_reconexiones(void) { return g_reconex; }

/* ---------------------------------------------------------------- mensajes */

static bool preparar_memoria(void)
{
    if (g_memoria_lista) return true;
    /* Capacidades fijas: 16 caracteres por cadena, 4 elementos por secuencia.
     * /atp/reference trae 2 nombres y 2 posiciones; sobra holgura. */
    micro_ros_utilities_memory_conf_t conf = {0};
    conf.max_string_capacity = 16;
    conf.max_ros2_type_sequence_capacity = 4;
    conf.max_basic_type_sequence_capacity = 4;

    if (!micro_ros_utilities_create_message_memory(
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), &g_ref, conf)) return false;

    for (int e = 0; e < g_cfg->n_ejes; e++) {
        if (!micro_ros_utilities_create_message_memory(
                ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), &g_js[e], conf)) return false;

        micro_ros_utilities_memory_conf_t conf_d = {0};
        conf_d.max_string_capacity = 16;
        conf_d.max_ros2_type_sequence_capacity = 2;
        conf_d.max_basic_type_sequence_capacity = DIAG_N;
        if (!micro_ros_utilities_create_message_memory(
                ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), &g_diag[e], conf_d)) return false;

        /* joint_states: un solo eje por mensaje */
        g_js[e].name.size = 1;
        strncpy(g_js[e].name.data[0].data, g_cfg->eje_nombre[e], g_js[e].name.data[0].capacity - 1);
        g_js[e].name.data[0].size = strlen(g_js[e].name.data[0].data);
        g_js[e].position.size = 1;
        g_js[e].velocity.size = 1;
        g_js[e].effort.size = 1;
        g_js[e].header.frame_id.size = 0;
        g_js[e].header.frame_id.data[0] = '\0';

        g_diag[e].layout.dim.size = 0;
        g_diag[e].layout.data_offset = 0;
        g_diag[e].data.size = DIAG_N;
    }
    g_memoria_lista = true;
    return true;
}

static void sellar(builtin_interfaces__msg__Time *t)
{
    /* hora del agente (PC) si la sesion esta sincronizada; si no, la local */
    const int64_t ns = rmw_uros_epoch_nanos();
    t->sec = (int32_t)(ns / 1000000000LL);
    t->nanosec = (uint32_t)(ns % 1000000000LL);
}

/* ---------------------------------------------------------------- callbacks */

static void cb_referencia(const void *msgin)
{
    const sensor_msgs__msg__JointState *m = (const sensor_msgs__msg__JointState *)msgin;

    /* separacion entre muestras segun SUS sellos (exacta aunque el enlace tenga jitter);
     * la misma para todos los ejes: es un solo mensaje */
    const int64_t ns = (int64_t)m->header.stamp.sec * 1000000000LL + m->header.stamp.nanosec;
    float dt_muestras = 0.0f;
    if (ns > 0 && g_ref_stamp_prev_ns > 0) {
        const double dt = (double)(ns - g_ref_stamp_prev_ns) * 1e-9;
        if (dt > 0.0 && dt < 1.0) dt_muestras = (float)dt;
    }
    g_ref_stamp_prev_ns = ns;
    g_refs++;

    for (int e = 0; e < g_cfg->n_ejes; e++) {
        int idx = g_cfg->eje_indice[e];
        /* si el mensaje trae nombres, manda el nombre: {az, el} u otro orden */
        if (m->name.size > 0) {
            idx = -1;
            for (size_t i = 0; i < m->name.size; i++) {
                if (m->name.data[i].data && strcmp(m->name.data[i].data, g_cfg->eje_nombre[e]) == 0) {
                    idx = (int)i;
                    break;
                }
            }
        }
        if (idx < 0 || (size_t)idx >= m->position.size) continue;
        eje_ref_t r;
        r.pos = (float)m->position.data[idx];
        r.vel = ((size_t)idx < m->velocity.size) ? (float)m->velocity.data[idx] : NAN;
        r.dt_muestras_s = dt_muestras;
        eje_ref_enviar(e, &r);
    }
    if (g_cfg->eco) {                       /* eco para medir la latencia de ida y vuelta */
        g_eco.data = ns;
        IGNORAR(rcl_publish(&g_pub_eco, &g_eco, NULL));
    }
}

/* rclc no pasa contexto a los callbacks: uno por eje */
#define CB_EJE(N) \
    static void cb_enable_##N(const void *msgin) { eje_cmd_armar(N, ((const std_msgs__msg__Bool *)msgin)->data); } \
    static void cb_origen_##N(const void *msgin) { eje_cmd_origen(N, (float)((const std_msgs__msg__Float64 *)msgin)->data); }
CB_EJE(0)
CB_EJE(1)
static const rclc_callback_t CB_ENABLE[ATP_EJES_MAX] = { cb_enable_0, cb_enable_1 };
static const rclc_callback_t CB_ORIGEN[ATP_EJES_MAX] = { cb_origen_0, cb_origen_1 };

/* maximos acumulados entre dos diag (los lee y reinicia eje_tel_leer) */
static uint32_t g_t_max_acum[ATP_EJES_MAX];
static int32_t g_jit_max_acum[ATP_EJES_MAX];
static eje_tel_t g_ultima[ATP_EJES_MAX];

static void guardar_maximos(int e, const eje_tel_t *t)
{
    g_ultima[e] = *t;
    if (t->t_lazo_max_us > g_t_max_acum[e]) g_t_max_acum[e] = t->t_lazo_max_us;
    if (t->jitter_max_us > g_jit_max_acum[e]) g_jit_max_acum[e] = t->jitter_max_us;
}

static void cb_telemetria(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer == NULL) return;
    for (int e = 0; e < g_cfg->n_ejes; e++) {
        eje_tel_t t;
        if (!eje_tel_leer(e, &t)) continue;
        /* eje_tel_leer reinicia los maximos: los de joint_states no se usan, y el
         * diag los lee antes que nadie los pise. Se guardan para el diag. */
        guardar_maximos(e, &t);
        sellar(&g_js[e].header.stamp);
        g_js[e].position.data[0] = (double)t.pos;
        g_js[e].velocity.data[0] = (double)t.vel;
        g_js[e].effort.data[0] = (double)t.u;
        IGNORAR(rcl_publish(&g_pub_js[e], &g_js[e], NULL));
    }
}

static void cb_diag(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer == NULL) return;
    for (int e = 0; e < g_cfg->n_ejes; e++) {
        const eje_tel_t *t = &g_ultima[e];
        float *d = g_diag[e].data.data;
        d[DIAG_ESTADO] = (float)t->estado;
        d[DIAG_FALLAS] = (float)t->fallas;
        d[DIAG_POS] = t->pos;
        d[DIAG_VEL] = t->vel;
        d[DIAG_U] = t->u;
        d[DIAG_DUTY] = t->duty;
        d[DIAG_CORRIENTE] = t->corriente_a;
        d[DIAG_REF_CMD] = t->ref_cmd;
        d[DIAG_REF_EDAD] = t->ref_edad_s;
        d[DIAG_T_LAZO_US] = (float)t->t_lazo_us;
        d[DIAG_T_LAZO_MAX_US] = (float)g_t_max_acum[e];
        d[DIAG_JITTER_MAX_US] = (float)g_jit_max_acum[e];
        d[DIAG_TICKS_PERDIDOS] = (float)t->ticks_perdidos;
        d[DIAG_REFS_RECIBIDAS] = (float)g_refs;
        d[DIAG_ABS_ERR] = (float)t->abs_err;
        d[DIAG_UPTIME_S] = (float)t->ticks * 0.001f;
        g_t_max_acum[e] = 0;
        g_jit_max_acum[e] = 0;
        IGNORAR(rcl_publish(&g_pub_diag[e], &g_diag[e], NULL));
    }
}

/* ---------------------------------------------------------------- entidades */

static void destruir(void)
{
    /* sin comprobar resultados: es limpieza tras perder el agente */
    rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&g_support.context);
    if (rmw_ctx) (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);
    IGNORAR(rclc_executor_fini(&g_exec));
    IGNORAR(rcl_timer_fini(&g_t_tel));
    IGNORAR(rcl_timer_fini(&g_t_diag));
    IGNORAR(rcl_subscription_fini(&g_sub_ref, &g_node));
    for (int e = 0; e < g_cfg->n_ejes; e++) {
        IGNORAR(rcl_subscription_fini(&g_sub_en[e], &g_node));
        IGNORAR(rcl_subscription_fini(&g_sub_or[e], &g_node));
        IGNORAR(rcl_publisher_fini(&g_pub_js[e], &g_node));
        IGNORAR(rcl_publisher_fini(&g_pub_diag[e], &g_node));
    }
    if (g_cfg->eco) IGNORAR(rcl_publisher_fini(&g_pub_eco, &g_node));
    IGNORAR(rcl_node_fini(&g_node));
    IGNORAR(rclc_support_fini(&g_support));
}

/* Nombre de un topico propio de un eje: con un solo eje el nodo YA vive en
 * /atp/mcu/<eje> y el nombre es relativo (como siempre); con varios, el nodo vive en
 * /atp/mcu y se antepone `<eje>/` (el topico final es el mismo). */
static const char *topico(char *buf, size_t n, int e, const char *base)
{
    if (g_cfg->n_ejes == 1) return base;
    snprintf(buf, n, "%s/%s", g_cfg->eje_nombre[e], base);
    return buf;
}

static bool crear(void)
{
    char ns[32];
    char nodo[32];
    if (g_cfg->n_ejes == 1) {
        snprintf(ns, sizeof ns, "/atp/mcu/%s", g_cfg->eje_nombre[0]);
        snprintf(nodo, sizeof nodo, "esp32_%s", g_cfg->eje_nombre[0]);
    } else {
        snprintf(ns, sizeof ns, "/atp/mcu");
        snprintf(nodo, sizeof nodo, "esp32_banco");
    }

    g_alloc = rcl_get_default_allocator();
    rcl_init_options_t io = rcl_get_zero_initialized_init_options();
    if (!RC_OK(rcl_init_options_init(&io, g_alloc))) return false;
    if (!RC_OK(rcl_init_options_set_domain_id(&io, (size_t)g_cfg->domain_id))) return false;
    if (g_cfg->opciones && !g_cfg->opciones(&io)) return false;
    const rcl_ret_t r = rclc_support_init_with_options(&g_support, 0, NULL, &io, &g_alloc);
    IGNORAR(rcl_init_options_fini(&io));
    if (!RC_OK(r)) return false;

    g_node = rcl_get_zero_initialized_node();
    if (!RC_OK(rclc_node_init_default(&g_node, nodo, ns, &g_support))) return false;

    char t[48];
    for (int e = 0; e < g_cfg->n_ejes; e++) {
        if (!RC_OK(rclc_publisher_init_default(
                &g_pub_js[e], &g_node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
                topico(t, sizeof t, e, "joint_states")))) return false;
        if (!RC_OK(rclc_publisher_init_best_effort(
                &g_pub_diag[e], &g_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
                topico(t, sizeof t, e, "diag")))) return false;
    }
    if (g_cfg->eco) {
        if (!RC_OK(rclc_publisher_init_default(
                &g_pub_eco, &g_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
                "/atp/mcu/eco"))) return false;
    }

    /* absoluto: es el topico del contrato, no cuelga del espacio de nombres del eje */
    if (!RC_OK(rclc_subscription_init_default(
            &g_sub_ref, &g_node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
            "/atp/reference"))) return false;
    for (int e = 0; e < g_cfg->n_ejes; e++) {
        if (!RC_OK(rclc_subscription_init_default(
                &g_sub_en[e], &g_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
                topico(t, sizeof t, e, "cmd/enable")))) return false;
        if (!RC_OK(rclc_subscription_init_default(
                &g_sub_or[e], &g_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
                topico(t, sizeof t, e, "cmd/origen")))) return false;
    }

    if (!RC_OK(rclc_timer_init_default2(&g_t_tel, &g_support,
            RCL_MS_TO_NS(1000 / (g_cfg->tel_hz > 0 ? g_cfg->tel_hz : 50)), cb_telemetria, true))) return false;
    if (!RC_OK(rclc_timer_init_default2(&g_t_diag, &g_support,
            RCL_MS_TO_NS(1000 / (g_cfg->diag_hz > 0 ? g_cfg->diag_hz : 2)), cb_diag, true))) return false;

    g_exec = rclc_executor_get_zero_initialized_executor();
    if (!RC_OK(rclc_executor_init(&g_exec, &g_support.context, NUM_MANEJADORES, &g_alloc))) return false;
    if (!RC_OK(rclc_executor_add_subscription(&g_exec, &g_sub_ref, &g_ref, cb_referencia, ON_NEW_DATA))) return false;
    for (int e = 0; e < g_cfg->n_ejes; e++) {
        if (!RC_OK(rclc_executor_add_subscription(&g_exec, &g_sub_en[e], &g_en[e], CB_ENABLE[e], ON_NEW_DATA))) return false;
        if (!RC_OK(rclc_executor_add_subscription(&g_exec, &g_sub_or[e], &g_or[e], CB_ORIGEN[e], ON_NEW_DATA))) return false;
    }
    if (!RC_OK(rclc_executor_add_timer(&g_exec, &g_t_tel))) return false;
    if (!RC_OK(rclc_executor_add_timer(&g_exec, &g_t_diag))) return false;
    return true;
}

/* ---------------------------------------------------------------- bucle */

void eje_uros_correr(const eje_uros_cfg_t *c)
{
    g_cfg = c;
    if (c->n_ejes < 1 || c->n_ejes > ATP_EJES_MAX) {
        printf("eje_uros: n_ejes=%d fuera de 1..%d\n", c->n_ejes, ATP_EJES_MAX);
        for (;;) c->dormir_ms(1000);
    }
    if (!preparar_memoria()) {
        printf("eje_uros: sin memoria para los mensajes\n");
        for (;;) c->dormir_ms(1000);
    }

    estado_conexion_t est = ESPERANDO_AGENTE;
    uint32_t t_ping = 0, t_sync = 0;
    for (;;) {
        switch (est) {
        case ESPERANDO_AGENTE:
            est = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENTE_DISPONIBLE : ESPERANDO_AGENTE;
            if (est == ESPERANDO_AGENTE) c->dormir_ms(400);
            break;
        case AGENTE_DISPONIBLE:
            if (crear()) {
                est = CONECTADO;
                g_reconex++;
                g_ref_stamp_prev_ns = 0;
                (void)rmw_uros_sync_session(1000);
                t_ping = t_sync = c->ahora_ms();
                printf("eje_uros: conectado al agente (#%u)\n", (unsigned)g_reconex);
            } else {
                destruir();
                est = ESPERANDO_AGENTE;
                c->dormir_ms(500);
            }
            break;
        case CONECTADO: {
            IGNORAR(rclc_executor_spin_some(&g_exec, RCL_MS_TO_NS(5)));
            const uint32_t ahora = c->ahora_ms();
            if (ahora - t_sync >= SYNC_CADA_MS) {
                (void)rmw_uros_sync_session(500);
                t_sync = ahora;
            }
            if (ahora - t_ping >= PING_CADA_MS) {
                t_ping = ahora;
                if (RMW_RET_OK != rmw_uros_ping_agent(100, 3)) est = DESCONECTADO;
            }
            break;
        }
        case DESCONECTADO:
            printf("eje_uros: agente perdido; se reintenta (el lazo sigue y el watchdog protege)\n");
            destruir();
            est = ESPERANDO_AGENTE;
            break;
        }
    }
}
