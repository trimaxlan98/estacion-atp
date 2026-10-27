"""Atmosfera para un enlace optico tierra-espacio: turbulencia (Hufnagel-Valley),
parametro de Fried, angulo isoplanatico, Rytov y centelleo (con promediado por
apertura), inclinacion del frente de onda (tilt) y *beam wander*, escala
temporal (viento de Bufton, Greenwood) y transmision (Kruse).

Solo numpy/scipy, sin ROS. Cada funcion dice bajo que condicion vale su cifra.
Convenciones: `h` = altura sobre la ESTACION (m) salvo que se diga otra cosa;
`el_deg` = elevacion del satelite; `lam` en metros; todos los angulos en rad
(salvo los `*_urad`). La trayectoria es plano-paralela (sec zeta = 1/sin el):
vale para el >~ 10 deg (a 5 deg el error de la Tierra curva ya es ~10 %).

PERFIL Cn^2(h) (Hufnagel-Valley, Andrews y Phillips cap. 3; Valley 1980):

    Cn2(h) = 0.00594 (v/27)^2 (1e-5 h)^10 exp(-h/1000) + 2.7e-16 exp(-h/1500) + A exp(-h/100)

con v el viento rms de la capa alta (21 m/s en HV 5/7) y A = Cn2 en tierra
(1.7e-14 m^-2/3). "5/7" es el nombre de los dos numeros que reproduce: r0 = 5 cm
y theta0 = 7 urad a 500 nm en el cenit (las pruebas lo verifican). Para una
estacion a `alt_estacion_m` sobre el nivel del mar los dos primeros terminos
(corriente en chorro, troposfera libre) se evaluan a la altura SOBRE EL MAR
(`modo_altitud='asl'`, por defecto: la tropopausa no baja con el terreno) y el
termino de capa limite con la altura sobre la estacion, donde A es la Cn2 LOCAL
en tierra; con `modo_altitud='agl'` todo se mide desde la estacion (el perfil
"se corre" con el terreno; la opcion que adopta el informe del DLR sobre HV a
altitud no nula, `Mata Calvo`, ver docs/OPTICO.md; la diferencia se tabula).

Integrales (A&P cap. 12; Fried 1966; Tyler 1994; Sasiela):
    r0       = [0.423 k^2 sec(zeta) int Cn2 dh]^(-3/5)                   (onda plana)
    theta0   = [2.914 k^2 sec(zeta)^(8/3) int Cn2 h^(5/3) dh]^(-3/5)
    sigma_R^2 = 2.25 k^(7/6) sec(zeta)^(11/6) int Cn2 (h)^(5/6) dh        (Rytov, bajada = onda plana)
    <r_c^2>  = 7.25 W0^(-1/3) int Cn2(z) (R - z)^2 dz                    (beam wander, haz colimado)
"""
import math
from dataclasses import dataclass
from functools import lru_cache

import numpy as np

_trapz = getattr(np, 'trapezoid', None) or np.trapz

LAM_1550 = 1550e-9
LAM_1064 = 1064e-9
LAM_500 = 500e-9


@dataclass(frozen=True)
class Atmosfera:
    cn2_suelo: float = 1.7e-14       # A: Cn2 local en tierra (m^-2/3). HV 5/7
    viento_rms: float = 21.0         # v de la capa alta (m/s). HV 5/7
    alt_estacion_m: float = 0.0      # altura de la estacion sobre el nivel del mar
    modo_altitud: str = 'asl'        # 'asl' | 'agl' (ver docstring del modulo)
    viento_suelo: float = 5.0        # v_g del perfil de Bufton (m/s). SUPUESTO tipico
    L0_m: float = 25.0               # escala externa, solo para f_L del espectro del tilt. SUPUESTO
    visibilidad_km: float = 23.0     # visibilidad meteorologica (aerosol): "cielo despejado" de la OMM
    h_max_m: float = 30000.0         # tope de la integral (sobre la estacion)
    cn2_uniforme: float = 0.0        # > 0: perfil UNIFORME de ese valor (solo para contrastar con las formulas cerradas)


HV57 = Atmosfera()


# ---------------------------------------------------------------- perfil e integrales

def cn2(atm, h_m):
    """Cn2(h) (m^-2/3), h = altura sobre la estacion."""
    h = np.asarray(h_m, dtype=float)
    if atm.cn2_uniforme > 0.0:
        return np.full_like(h, atm.cn2_uniforme)
    ha = h + atm.alt_estacion_m if atm.modo_altitud == 'asl' else h
    return (0.00594 * (atm.viento_rms / 27.0) ** 2 * (1e-5 * ha) ** 10 * np.exp(-ha / 1000.0)
            + 2.7e-16 * np.exp(-ha / 1500.0) + atm.cn2_suelo * np.exp(-h / 100.0))


@lru_cache(maxsize=64)
def _rejilla(atm, n=60001):
    h = np.linspace(0.0, atm.h_max_m, n)
    return h, cn2(atm, h)


@lru_cache(maxsize=512)
def integral(atm, expo=0.0):
    """int_0^H Cn2(h) h^expo dh (regla del trapecio, paso 0.5 m)."""
    h, c = _rejilla(atm)
    return float(_trapz(c * h ** expo, h))


def secante(el_deg):
    return 1.0 / np.sin(np.radians(np.maximum(el_deg, 1.0)))


def masa_de_aire(el_deg):
    """Masa de aire de Kasten y Young (1989): 1/(sin el + 0.50572 (el + 6.07995)^-1.6364);
    incluye curvatura y refraccion medias. En el cenit vale 1 y a 10 deg 5.6 (plano-paralelo 5.76)."""
    el = np.asarray(el_deg, float)
    return 1.0 / (np.sin(np.radians(el)) + 0.50572 * (el + 6.07995) ** -1.6364)


def numero_onda(lam):
    return 2.0 * math.pi / lam


# ---------------------------------------------------------------- Fried, isoplanatico, Rytov

def fried_r0(atm, lam, el_deg=90.0):
    """r0 (m), onda plana (bajada). Escala lam^(6/5) sin(el)^(3/5)."""
    k = numero_onda(lam)
    return (0.423 * k * k * secante(el_deg) * integral(atm, 0.0)) ** (-0.6)


def theta0(atm, lam, el_deg=90.0):
    """Angulo isoplanatico (rad): 0.314 r0/h_bar con h_bar la altura media (5/3) de la turbulencia.
    Escala lam^(6/5) sin(el)^(8/5)."""
    k = numero_onda(lam)
    return (2.914 * k * k * secante(el_deg) ** (8.0 / 3.0) * integral(atm, 5.0 / 3.0)) ** (-0.6)


def rytov(atm, lam, el_deg=90.0):
    """Varianza de Rytov sigma_R^2 de onda plana (bajada); valida como indicador
    del regimen (debil < 1). Para un haz que sube desde tierra a un satelite
    lejano (L >> altura de la turbulencia) el peso (z (L-z)/L)^(5/6) de la onda
    esferica se reduce a z^(5/6): da lo mismo."""
    k = numero_onda(lam)
    return 2.25 * k ** (7.0 / 6.0) * secante(el_deg) ** (11.0 / 6.0) * integral(atm, 5.0 / 6.0)


def altura_efectiva_m(atm):
    """Altura media de la turbulencia ponderada por su aporte al centelleo:
    int Cn2 h^(11/6) dh / int Cn2 h^(5/6) dh (m sobre la estacion)."""
    return integral(atm, 11.0 / 6.0) / integral(atm, 5.0 / 6.0)


def centelleo(atm, lam, el_deg, D_m):
    """Indice de centelleo de una apertura de diametro D (promediado por apertura).

    Modelo de Andrews y Phillips (cap. 8-9; Andrews, Phillips y Hopen 2001): con
    sigma_R^2 de `rytov`, d^2 = k D^2 / (4 L), L = h_efectiva / sin(el) (la
    distancia a la que "vive" el centelleo, `altura_efectiva_m`):

      sigma_lnx^2 = 0.49 sR2 / (1 + 0.65 d2 + 1.11 sR^(12/5))^(7/6)         (gran escala)
      sigma_lny^2 = 0.51 sR2 (1 + 0.69 sR^(12/5))^(-5/6) / (1 + 0.90 d2 + 0.62 d2 sR^(12/5))^(5/6)   (pequena escala)
      sigma_I^2   = exp(sigma_lnx^2 + sigma_lny^2) - 1
      alpha = 1/(exp(sigma_lnx^2) - 1),  beta = 1/(exp(sigma_lny^2) - 1)      (gamma-gamma)

    Con D -> 0 y sR^2 pequeno da sigma_I^2 -> sR^2 (debil). El regimen (para
    elegir distribucion) es 'debil' si sR^2 < 0.3 (log-normal), si no
    'moderado-fuerte' (gamma-gamma)."""
    sR2 = float(rytov(atm, lam, el_deg))
    sR125 = sR2 ** 1.2
    L = altura_efectiva_m(atm) * float(secante(el_deg))
    d2 = numero_onda(lam) * D_m * D_m / (4.0 * L)
    lnx = 0.49 * sR2 / (1.0 + 0.65 * d2 + 1.11 * sR125) ** (7.0 / 6.0)
    lny = (0.51 * sR2 * (1.0 + 0.69 * sR125) ** (-5.0 / 6.0)
           / (1.0 + 0.90 * d2 + 0.62 * d2 * sR125) ** (5.0 / 6.0))
    sI2 = math.expm1(lnx + lny)
    return {'sigma_R2': sR2, 'sigma_lnx2': lnx, 'sigma_lny2': lny, 'sigma_I2': sI2,
            'alpha': 1.0 / math.expm1(lnx), 'beta': 1.0 / math.expm1(lny),
            'd2': d2, 'L_ef_m': L, 'regimen': 'debil' if sR2 < 0.3 else 'moderado-fuerte'}


def promediado_apertura_churnside(atm, lam, el_deg, D_m):
    """Contraste independiente de `centelleo` en regimen debil: factor de
    promediado por apertura A = sigma_I^2(D)/sigma_I^2(0) integrado CAPA POR CAPA
    con la forma de Churnside (1991), A_z = [1 + 1.062 k D^2/(4 L_z)]^(-7/6),
    L_z = h/sin(el): A = int Cn2 h^(5/6) A_z dh / int Cn2 h^(5/6) dh."""
    h, c = _rejilla(atm)
    k = numero_onda(lam)
    hh = np.maximum(h, 1.0)
    Lz = hh * float(secante(el_deg))
    Az = (1.0 + 1.062 * k * D_m * D_m / (4.0 * Lz)) ** (-7.0 / 6.0)
    return float(_trapz(c * hh ** (5.0 / 6.0) * Az, h) / integral(atm, 5.0 / 6.0))


# ---------------------------------------------------------------- tilt, beam wander, Strehl

def tilt_llegada_rad(atm, lam, D_m, el_deg=90.0):
    """rms (rad, UN eje) del angulo de llegada (inclinacion del frente de onda,
    tilt-Z) sobre una apertura D: sigma^2 = 0.182 (D/r0)^(5/3) (lam/D)^2 (Tyler 1994;
    Noll: 0.448 (D/r0)^(5/3) rad^2 de fase * (2/pi)^2 (lam/D)^2). INDEPENDIENTE de
    lam (r0 ~ lam^(6/5)). Kolmogorov (escala externa infinita): CONSERVADOR, la
    escala externa de decenas de m reduce la varianza del orden de 10-30 % a D = 0.4 m."""
    r0 = fried_r0(atm, lam, el_deg)
    return math.sqrt(0.182 * (D_m / r0) ** (5.0 / 3.0)) * lam / D_m


def beam_wander(atm, D_tx_m, el_deg, rango_m, gamma=1.12):
    """Beam wander de un haz que SUBE de tierra a un satelite a `rango_m` (m de
    trayecto oblicuo): <r_c^2> (m^2 en el satelite, los dos ejes) = 7.25 W0^(-1/3)
    int_0^R Cn2(z sin el) (R - z)^2 dz (Andrews y Phillips 2005, cap. 6 y 12; con Cn2
    constante da 2.42 Cn2 L^3 W0^(-1/3)); W0 = D/(2 gamma) es el radio 1/e^2 del haz en
    la apertura. Devuelve {'rc_m', 'sigma_urad'} con sigma el angulo rms de UN eje:
    sqrt(<r_c^2>/2)/R. Haz colimado en regimen debil; sin pre-compensacion del tilt."""
    h, c = _rejilla(atm)
    s = math.sin(math.radians(max(float(el_deg), 1.0)))
    z = h / s
    ok = z < rango_m
    W0 = D_tx_m / (2.0 * gamma)
    I = float(_trapz(np.where(ok, c * (rango_m - z) ** 2, 0.0), z))
    rc2 = 7.25 * W0 ** (-1.0 / 3.0) * I
    return {'rc_m': math.sqrt(rc2), 'sigma_urad': 1e6 * math.sqrt(rc2 / 2.0) / rango_m}


def strehl_corto(D_m, r0_m):
    """Strehl de corta exposicion (sin el tilt): [1 + 0.112 (D/r0)^(5/3)]^(-6/5).
    Interpolacion: a D/r0 pequeno es 1 - 0.134 (D/r0)^(5/3) (varianza de fase sin
    tilt de Noll, Delta_2 = 0.134), a D/r0 grande tiende al (r0/D)^2 de la ley
    de Fried. NO es una formula publicada tal cual: esta documentada como interpolacion."""
    return (1.0 + 0.112 * (D_m / r0_m) ** (5.0 / 3.0)) ** (-1.2)


# ---------------------------------------------------------------- viento y escala temporal

def viento(atm, h_m, el_deg=90.0, slew_rad_s=0.0):
    """Perfil de viento de Bufton con el barrido de la linea de vista:
    v(h) = omega_s h/sin(el) + v_g + 30 exp(-((h_asl - 9400)/4800)^2) (m/s). Suma escalar
    de los tres terminos (Bufton: cota alta)."""
    h = np.asarray(h_m, float)
    ha = h + atm.alt_estacion_m
    return (slew_rad_s * h * secante(el_deg) + atm.viento_suelo
            + 30.0 * np.exp(-(((ha - 9400.0) / 4800.0) ** 2)))


def viento_efectivo(atm, el_deg=90.0, slew_rad_s=0.0):
    """v_eff = [int Cn2 v^(5/3) dh / int Cn2 dh]^(3/5) (m/s)."""
    h, c = _rejilla(atm)
    v = viento(atm, h, el_deg, slew_rad_s)
    return float((_trapz(c * v ** (5.0 / 3.0), h) / integral(atm, 0.0)) ** 0.6)


def viento_tilt(atm, el_deg=90.0, slew_rad_s=0.0):
    """v_tilt = [int Cn2 v^2 dh / int Cn2 dh]^(1/2) (m/s): el que fija la escala temporal del tilt (Tyler)."""
    h, c = _rejilla(atm)
    v = viento(atm, h, el_deg, slew_rad_s)
    return float(math.sqrt(_trapz(c * v ** 2, h) / integral(atm, 0.0)))


def tiempo_coherencia(atm, lam, el_deg=90.0, slew_rad_s=0.0):
    """tau0 = 0.314 r0 / v_eff (s) (Roddier; A&P)."""
    return 0.314 * fried_r0(atm, lam, el_deg) / viento_efectivo(atm, el_deg, slew_rad_s)


def frecuencia_greenwood(atm, lam, el_deg=90.0, slew_rad_s=0.0):
    """f_G = 0.134/tau0 = 0.427 v_eff / r0 (Hz) (Greenwood 1977; 2.31 lam^-6/5 [sec int Cn2 v^5/3]^3/5)."""
    return 0.134 / tiempo_coherencia(atm, lam, el_deg, slew_rad_s)


# ---------------------------------------------------------------- transmision

def tau_cenit(atm, lam):
    """Espesor optico en el cenit (neper) de cielo despejado: aerosol (Kruse: beta =
    (3.912/V)(lam/550nm)^-q /km, escala 1.2 km) + Rayleigh (beta = 0.008569 lam_um^-4 /km, escala
    8 km) + absorcion molecular residual de la ventana (0.02 a 1550 nm y 0.01 a 1064 nm,
    escala 2 km: SUPUESTO, la ventana real depende del vapor de agua). Se corrigen por la altura de
    la estacion (la columna que queda encima)."""
    V = atm.visibilidad_km
    q = 1.6 if V > 50.0 else (1.3 if V > 6.0 else 0.585 * V ** (1.0 / 3.0))
    lam_um = lam * 1e6
    beta_a = 3.912 / V * (lam * 1e9 / 550.0) ** (-q)
    h_km = atm.alt_estacion_m / 1000.0
    t_aer = beta_a * 1.2 * math.exp(-h_km / 1.2)
    t_ray = 0.008569 * lam_um ** -4 * 8.0 * math.exp(-h_km / 8.0)
    t_abs = (0.02 if lam > 1300e-9 else 0.01) * math.exp(-h_km / 2.0)
    return t_aer + t_ray + t_abs


def transmision(atm, lam, el_deg):
    """Transmitancia de la atmosfera despejada: exp(-tau_cenit * masa_de_aire(el))."""
    return float(math.exp(-tau_cenit(atm, lam) * float(masa_de_aire(el_deg))))
