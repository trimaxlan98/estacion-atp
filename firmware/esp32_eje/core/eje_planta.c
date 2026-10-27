#include "eje_planta.h"

#include <math.h>

void eje_planta_iniciar(eje_planta_t *p, double J, double b, double w_max, double theta0)
{
    p->J = J;
    p->b = b;
    p->w_max = w_max;
    p->theta = theta0;
    p->omega = 0.0;
}

void eje_planta_paso(eje_planta_t *p, double u_nm, double par_ext_nm, double h)
{
    const double alpha = (u_nm + par_ext_nm - p->b * p->omega) / p->J;
    double w = p->omega + alpha * h;
    if (p->w_max > 0.0) {
        if (w > p->w_max) w = p->w_max;
        if (w < -p->w_max) w = -p->w_max;
    }
    p->omega = w;
    p->theta += w * h;
}

int32_t eje_planta_cuentas(const eje_planta_t *p, double cpr)
{
    return (int32_t)llround(p->theta * cpr / (2.0 * M_PI));
}
