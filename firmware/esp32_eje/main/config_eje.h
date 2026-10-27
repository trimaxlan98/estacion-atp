/* Configuracion del eje desde Kconfig (idf.py menuconfig) hacia eje_cfg_t y tabla de pines. */
#ifndef CONFIG_EJE_H
#define CONFIG_EJE_H

#include "eje_compartido.h"
#include "eje_lazo.h"
#include "sdkconfig.h"

/* Ejes que gobierna este firmware: 1 (el de siempre) o 2 (banco: az y el). */
#ifdef CONFIG_ATP_EJES
#define N_EJES CONFIG_ATP_EJES
#else
#define N_EJES 1
#endif

/* Pines de un eje. Con ATP_EJES=1 solo existe el eje 0, con los pines de siempre. */
typedef struct { int enc_a, enc_b, pwm_a, pwm_b, en, corr_r, corr_l; } pines_eje_t;
const pines_eje_t *pines_eje(int e);

/* Rellena `c` con los CONFIG_ATP_* del eje `e` (valores enteros en milli/centi-unidades). */
void config_eje_cargar(int e, eje_cfg_t *c);

#endif
