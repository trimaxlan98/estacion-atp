/* Planta de un eje en C: la MISMA de atp_ros (J theta'' + b theta' = u + d).
 *
 * Es la rama sin capas opcionales de PlantaMontura.paso (src/atp_ros/atp_ros/
 * planta.py), operacion por operacion:
 *     alpha = (u + par_ext - b w) / J;  w = clip(w + alpha h);  theta += w h
 * (Euler semi-implicito). Se usa en las pruebas del host, en el nodo micro-ROS
 * de Linux y, con CONFIG_ATP_MODO_SIMULADO, dentro del firmware para el
 * "hardware en el lazo sin motor" del plan de src/atp_hw/README.md (punto 7).
 * double: es codigo de prueba, no del lazo de 1 kHz.
 */
#ifndef EJE_PLANTA_H
#define EJE_PLANTA_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double J, b;            /* kg m^2, N m s/rad (model.J_EJE, model.B_EJE)    */
    double w_max;           /* rad/s; <= 0: sin tope (como vel_max_deg_s = 0)  */
    double theta, omega;    /* rad, rad/s de la carga                          */
} eje_planta_t;

void eje_planta_iniciar(eje_planta_t *p, double J, double b, double w_max, double theta0);
void eje_planta_paso(eje_planta_t *p, double u_nm, double par_ext_nm, double h);
/* cuentas de un encoder ideal de `cpr` cuentas por vuelta (redondeo al mas cercano) */
int32_t eje_planta_cuentas(const eje_planta_t *p, double cpr);

#ifdef __cplusplus
}
#endif
#endif
