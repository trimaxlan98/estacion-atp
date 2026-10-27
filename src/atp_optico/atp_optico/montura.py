"""La montura gruesa de `atp_ros` sin ROS: el MISMO lazo de `reference_generator` + `pid_controller` +
`mount_sim` (PID de 20 Hz con derivada filtrada y anti-windup, planta J theta'' + b theta' = u de
`atp_ros.planta.PlantaMontura` integrada a 2 ms), alimentado con un pase SGP4 real. Da el error de
seguimiento REAL de la montura (contra la verdad, no contra el encoder), que es la perturbacion lenta
del FSM de `atp.py`. Reproduce en el mismo orden que el lockstep (CONTRATO 8):

    tick k: la planta integra [t_(k-1), t_k] con el mando vigente (retencion) -> estado(t_k);
            la referencia de t_k es interp(t_k - t_ini - retardo) ; el PID mide e = ref - estado y da el mando de t_k.

(el retardo `start_delay_s` = 1.5 s mantiene la referencia en AOS mientras arranca el lazo; el error solo
cuenta entre AOS y LOS, como `error_monitor`). La prueba del lockstep compara esta salida con la corrida
ROS (error rms). Fuentes: las del propio atp_ros; ningun parametro nuevo.
"""
import math
from dataclasses import dataclass

import numpy as np

from atp_ros import model, orbit
from atp_ros.planta import PlantaMontura, VientoTurbulento


@dataclass(frozen=True)
class ConfigMontura:
    kp: float = 25.0
    ki: float = 2.0
    kd: float = 8.0
    tau_derivada: float = 0.08
    u_max: float = 8.0
    J: float = model.J_EJE
    b: float = model.B_EJE
    paso_s: float = 0.05              # lockstep y PID a 20 Hz
    substep_s: float = 0.002
    retardo_s: float = 1.5            # start_delay_s del generador
    vel_max_deg_s: float = 0.0        # 0: sin tope de velocidad (como atp_loop por defecto)
    wind_sigma_nm: float = 0.0        # par de viento (Dryden), N m rms; 0: sin viento
    wind_tau_s: float = 5.0
    semilla: int = 7


def perfil_pase(sat='ISS', estacion='cdmx', pase_idx=0, inicio_utc='2026-10-02T15:00:00', horas=72.0,
                mascara_deg=10.0, el_max_min_deg=0.0, dt_s=0.5):
    """Pase SGP4 listo (formato de `orbit.muestrear_pase`) + metadatos."""
    perfil, pase, tle, est = orbit.perfil_tle(sat, estacion, pase_idx, inicio_utc, horas, mascara_deg,
                                              el_max_min_deg, dt_s=dt_s)
    return perfil, pase, tle, est


def simular_montura(perfil, cfg=ConfigMontura()):
    """Corre el lazo de la montura sobre `perfil` (t, az_continuo, el en grados). Devuelve un dict de
    arreglos por tick DENTRO del pase: t (s desde AOS), az/el del satelite (deg), rango (km), vr (km/s),
    e_az/e_el (deg, articulacion), e_pt (deg, separacion en el cielo)."""
    t_p, az_p, el_p = perfil['t'], perfil['az_continuo'], perfil['el']
    dur = float(t_p[-1])
    dt = cfg.paso_s
    planta = PlantaMontura(cfg.J, cfg.b, math.radians(cfg.vel_max_deg_s) if cfg.vel_max_deg_s > 0 else math.inf, cfg.substep_s)
    viento = (VientoTurbulento(cfg.wind_sigma_nm, 'dryden', cfg.wind_tau_s, 10.0, semilla=cfg.semilla + 1000)
              if cfg.wind_sigma_nm > 0 else None)
    alfa = dt / (cfg.tau_derivada + dt)
    integ = np.zeros(2)
    e_prev = np.zeros(2)
    d_filt = np.zeros(2)
    u = np.zeros(2)
    n_sub = int(round(dt / cfg.substep_s))
    out = {k: [] for k in ('t', 'az', 'el', 'rango_km', 'vr_km_s', 'e_az_deg', 'e_el_deg', 'e_pt_deg')}
    k = 0
    while True:
        t_rel = k * dt - cfg.retardo_s
        t = min(max(t_rel, 0.0), dur)
        ref = np.array([math.radians(float(np.interp(t, t_p, az_p))), math.radians(float(np.interp(t, t_p, el_p)))])
        if k == 0:
            planta.enganchar(ref)
        else:
            par = viento.par(k * dt, dt) if viento is not None else 0.0
            for _ in range(n_sub):
                planta.paso(np.clip(u, -cfg.u_max, cfg.u_max), par, cfg.substep_s)
        theta = planta.theta
        e = ref - theta                       # verdad de la carga (truth:=true)
        if 0.0 < t_rel < dur:
            e_deg = np.degrees(e)
            out['t'].append(t)
            out['az'].append(math.degrees(ref[0]))
            out['el'].append(math.degrees(ref[1]))
            out['rango_km'].append(float(np.interp(t, t_p, perfil['d'])))
            out['vr_km_s'].append(float(np.interp(t, t_p, perfil['vr'])))
            out['e_az_deg'].append(float(e_deg[0]))
            out['e_el_deg'].append(float(e_deg[1]))
            out['e_pt_deg'].append(math.hypot(float(e_deg[0]) * math.cos(ref[1]), float(e_deg[1])))
        if t_rel >= dur + 2.0 * dt:
            break
        # PID: mide el encoder (= carga, sin ruido ni cuantizacion por omision) contra la referencia
        d_bruta = (e - e_prev) / dt
        d_filt = d_filt + alfa * (d_bruta - d_filt)
        u_raw = cfg.kp * e + cfg.ki * integ + cfg.kd * d_filt
        u = np.clip(u_raw, -cfg.u_max, cfg.u_max)
        sat = np.abs(u_raw) > cfg.u_max + 1e-12
        congelar = sat & (u_raw * e > 0.0)
        integ = integ + np.where(congelar, 0.0, e * dt)
        e_prev = e
        k += 1
    return {k_: np.array(v) for k_, v in out.items()}


def error_cielo_urad(m, i):
    """Error de la montura en el cielo en el tick i (urad): (e_az cos el, e_el)."""
    return np.array([1e6 * math.radians(m['e_az_deg'][i]) * math.cos(math.radians(m['el'][i])),
                     1e6 * math.radians(m['e_el_deg'][i])])
