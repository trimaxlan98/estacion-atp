"""Guiado consciente de la autoridad del vehiculo y mezclador con prioridad de par (paquete P6). Sin ROS.

Dos ideas, ambas detras de banderas apagadas por defecto (apagadas = los bytes de D1 a D5):

1. **Autoridad de guiado.** El guiado de D5 frena con a = 2 m/s2 y cruza a 5 m/s sea cual sea el cuerpo. La aceleracion horizontal que el
   dron puede dar es `a_h = g tan(inclinacion_max)` (la componente horizontal del empuje con el colectivo que sostiene el peso) y, en
   regimen, el arrastre la compensa a `v_term = sqrt(2 m g tan(inclinacion_max) / (rho CdA))`. En Titan (g = 1.352) `a_h` es 0.947 m/s2:
   un perfil de frenado de 2 m/s2 no se puede cumplir y el dron se pasa de la meta. Con la bandera, el guiado usa
   `a_brake = min(a_brake_base, frac_brake * a_h)` y `v_max = min(v_max, frac_v * v_term)`; el `min` deja la Tierra intacta.
2. **Mezclador con prioridad de par**, inspirado en Faessler, Falanga y Scaramuzza, «Thrust Mixing, Saturation, and Body-Rate Control for
   Accurate Aggressive Quadrotor Flight», RA-L 2(2):476-482, 2017, DOI 10.1109/LRA.2016.2640362 (verificado solo el titulo, DOI y que trata
   la saturacion priorizando las entradas de control; el esquema concreto de ese articulo NO se reproduce aqui). Aqui el colectivo puede cambiar a lo mas `dT_max` para que ningun rotor pida
   menos de 0 N ni mas de `t_max`; los pares se respetan. Con `dT_max` sin tope seria el airmode puro.
"""
import numpy as np

from atp_dron.cuerpos import cuerpo
from atp_dron.dron import PARAMS

FRAC_BRAKE = 0.6        # fraccion de a_h que el perfil de frenado le pide al dron (deja margen para viento, rafagas y sobreimpulso)
FRAC_V = 0.9            # fraccion de v_term a la que se limita la velocidad de crucero
FRAC_ACEL = 0.7         # fraccion de a_h que se le permite pedir al lazo de velocidad (inclinacion pedida = atan(0.7 tan 35) = 26 grados)
KV = 3.0                # ganancia del lazo de velocidad de la cascada (control.Cascada.kv): a_pedida = kv * (v_des - v)


def a_horizontal(g, inclinacion_max_deg):
    """Aceleracion horizontal maxima (m/s2) con el colectivo que sostiene el peso: g tan(inclinacion_max)."""
    return float(g * np.tan(np.radians(inclinacion_max_deg)))


def v_terminal(masa_kg, g, rho, cda_m2, inclinacion_max_deg):
    """Velocidad (m/s) a la que el arrastre cuadratico iguala la fuerza horizontal maxima m g tan(inclinacion_max).
    Formula cerrada: sqrt(2 m g tan(i) / (rho CdA)). Sin atmosfera no hay tope (inf)."""
    if rho <= 0.0 or cda_m2 <= 0.0:
        return float('inf')
    return float(np.sqrt(2.0 * masa_kg * g * np.tan(np.radians(inclinacion_max_deg)) / (rho * cda_m2)))


def limites_autoridad(cuerpo_nombre, dron_params=None, inclinacion_max_deg=35.0, a_brake0=2.0, v_max0=5.0,
                      frac_brake=FRAC_BRAKE, frac_v=FRAC_V, frac_acel=FRAC_ACEL, kv=KV):
    """(a_brake, v_max, informacion) que el guiado debe usar en este cuerpo y con este vehiculo.

    Un salto de `v_des` de 0 a v_max satura la inclinacion al instante (a_pedida = kv * error). La bandera `autoridad_acel` lo evita de
    dos maneras: `'recorte'` deja el error de velocidad en a lo mas `informacion['ev_max_ms']` (= frac_acel * a_h / kv); `'rampa'` (o
    True) hace que `v_des` cambie a lo mas `informacion['a_acel_ms2']` (= frac_acel * a_h) por segundo en el plano horizontal y `informacion['az_acel_ms2']` (= min(frac_acel * a_h, g/2)) en la vertical (bajar solo da ~g). El recorte mantiene un error
    permanente y por eso la integral del lazo de velocidad se infla; la rampa no."""
    c = cuerpo(cuerpo_nombre)
    P = {**PARAMS, **(dron_params or {})}
    a_h = a_horizontal(c['g'], inclinacion_max_deg)
    vt = v_terminal(float(P['masa_kg']), c['g'], c['rho'], float(P['cda_m2']), inclinacion_max_deg)
    a_brake = min(float(a_brake0), frac_brake * a_h)
    v_max = min(float(v_max0), frac_v * vt)
    return a_brake, v_max, {'a_h_ms2': a_h, 'v_term_ms': vt, 'a_brake_ms2': a_brake, 'v_max_ms': v_max,
                             'ev_max_ms': frac_acel * a_h / kv, 'a_acel_ms2': frac_acel * a_h,
                             'az_acel_ms2': min(frac_acel * a_h, 0.5 * c['g'])}


def mezclar_prioridad_par(A_inv, T, tau, t_max, dT_max):
    """Empuje por rotor (4,) para el colectivo `T` y el par `tau` (3,), priorizando el par.

    La salida es `T' * A_inv[:, 0] + A_inv[:, 1:] @ tau` con `T'` el colectivo mas cercano a `T` (a lo mas `dT_max` de distancia) tal que
    todos los rotores quedan en [0, t_max] si eso es posible. Si no alcanza, el par se respeta igual y el recorte de los rotores lo hace
    la planta (`Dron.empuje_a_om`); con `dT_max = 0` es la mezcla de siempre.
    """
    T = float(T)
    base = A_inv[:, 0]
    c = A_inv[:, 1:] @ np.asarray(tau, float)
    lo = float(np.max(-c / base))                       # colectivo minimo con todos los rotores >= 0
    hi = float(np.min((t_max - c) / base))              # colectivo maximo con todos los rotores <= t_max
    if lo <= hi:
        Tn = min(max(T, lo), hi)
    else:                                               # infactible: lo que mas se acerca al centro de la ventana
        Tn = 0.5 * (lo + hi)
    Tn = min(max(Tn, T - dT_max), T + dT_max)
    return Tn * base + c
