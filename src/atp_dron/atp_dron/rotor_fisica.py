"""Fisica del rotor (P8): coherencia par-potencia, Reynolds, flujo inducido axial/avance y efecto suelo unico.

Todo es "en simulacion, con estos supuestos". Funciones puras (sin estado); `dron.py` las usa detras de banderas apagadas.

Lo que SI esta cerrado (se deriva aqui y se contrasta en las pruebas):
  * momentum 1-D: vh = sqrt(T/(2 rho A)); flujo axial vi(vi+vc) = vh^2 (vc>0 asciende). Rama de ascenso/hover:
    vi = -vc/2 + sqrt((vc/2)^2 + vh^2). Rama de freno de molino (vc <= -2 vh): vi = -vc/2 - sqrt((vc/2)^2 - vh^2).
  * avance (Glauert, disco horizontal): vi sqrt(V^2 + vi^2) = vh^2, con solucion cerrada
    vi^2 = (-V^2 + sqrt(V^4 + 4 vh^4))/2 (se resuelve por Newton y se contrasta con la cerrada).
  * coherencia par-potencia: P_eje = kM T Omega; FoM_implicita = (T^1.5/sqrt(2 rho A)) / (kM T Omega).

Lo que NO esta implementado por falta de fuente leida:
  * estado de anillo de vortice (-2 vh < vc < 0): los coeficientes tipo Johnson NO VERIFICADO; `induccion_axial` lanza
    NotImplementedError en esa banda.
  * la tabla de Perez Perez (NASA Ames, 2020) no se pudo abrir: los exponentes de f(Re) son PARAMETRICOS.
"""
import numpy as np


# ---------------------------------------------------------------- efecto suelo (unico)
def denominador_suelo(z, radio):
    """1 - (R/4z)^2 con z >= R/2 (Cheeseman y Bennett 1955, citada por dron.py; no reverificada aqui)."""
    return 1.0 - (radio / (4.0 * max(z, radio / 2.0))) ** 2


def factor_suelo(z, radio):
    """T/T_inf en efecto suelo (>= 1, tope 1/(1-1/4)=1.333...)."""
    return 1.0 / denominador_suelo(z, radio)


def aplicar_suelo(T, z, radio):
    """Unica ruta de efecto suelo: la usan `Dron._deriv` y `Dron.fuerza_especifica` (misma operacion, mismos bytes que antes)."""
    return T / denominador_suelo(z, radio)


# ---------------------------------------------------------------- cantidad de movimiento
def area_disco(radio):
    return np.pi * radio ** 2


def v_hover(T, rho, radio):
    """Velocidad inducida de hover vh = sqrt(T / (2 rho A))."""
    return np.sqrt(T / (2.0 * rho * area_disco(radio)))


def potencia_ideal(T, rho, radio):
    """Potencia inducida ideal de hover T^1.5 / sqrt(2 rho A) (W)."""
    return T ** 1.5 / np.sqrt(2.0 * rho * area_disco(radio))


def induccion_axial(vc, vh):
    """Velocidad inducida axial por momentum (vc>0 asciende). NotImplementedError dentro del anillo de vortice."""
    if vc >= 0.0:
        return -vc / 2.0 + np.sqrt((vc / 2.0) ** 2 + vh ** 2)
    if vc <= -2.0 * vh:
        return -vc / 2.0 - np.sqrt((vc / 2.0) ** 2 - vh ** 2)
    raise NotImplementedError('estado de anillo de vortice (-2 vh < vc < 0): coeficientes empiricos NO VERIFICADO, '
                              'momentum no es valido aqui')


def glauert_cerrada(V, vh):
    """Solucion cerrada de vi sqrt(V^2+vi^2) = vh^2."""
    return np.sqrt((-V * V + np.sqrt(V ** 4 + 4.0 * vh ** 4)) / 2.0)


def glauert_iterativa(V, vh, tol=1e-14, maxit=100):
    """Newton sobre f(vi) = vi sqrt(V^2+vi^2) - vh^2, desde vi = vh. Devuelve (vi, iteraciones)."""
    vi = float(vh)
    for k in range(1, maxit + 1):
        s = np.sqrt(V * V + vi * vi)
        f = vi * s - vh ** 2
        df = s + vi * vi / s
        nuevo = vi - f / df
        if abs(nuevo - vi) <= tol * max(vh, 1e-300):
            return float(nuevo), k
        vi = nuevo
    return float(vi), maxit


def razon_induccion_avance(V, vh):
    """vi/vh en avance (disco horizontal)."""
    if vh <= 0.0:
        return 1.0
    return float(glauert_cerrada(V, vh) / vh)


def potencia_avance(T, V, rho, radio, fom, om, k_mu=4.65, cda=0.0):
    """Potencia de eje de un rotor en avance a velocidad aerodinamica V (W) y sus tres terminos.

    P = P_i * (vi/vh) + P_0 * (1 + k_mu mu^2) + (cda rho V^3 / 2)/4 por rotor, con
      P_i = T^1.5/sqrt(2 rho A)   (ideal, Glauert modifica solo el factor vi/vh),
      P_0 = P_i (1/FoM - 1)       (perfil: lo que la FoM de hover atribuye a perfil/perdidas; SUPUESTO),
      mu  = V / (Omega R),  k_mu = 4.65 (PARAMETRICO: el valor aparece en una fuente secundaria, el texto de Leishman NO se leyo).
    A V=0 vuelve exactamente a P_i / FoM.
    """
    vh = v_hover(T, rho, radio)
    Pi = potencia_ideal(T, rho, radio)
    P0 = Pi * (1.0 / fom - 1.0)
    mu = V / (om * radio) if om > 0 else 0.0
    ind = Pi * razon_induccion_avance(V, vh)
    perfil = P0 * (1.0 + k_mu * mu * mu)
    par = 0.5 * cda * rho * V ** 3 / 4.0
    return float(ind + perfil + par), {'inducida': float(ind), 'perfil': float(perfil), 'parasita': float(par)}


# ---------------------------------------------------------------- coherencia par-potencia
def fom_implicita(T, om, k_m, rho, radio):
    """FoM que implica el modelo de par: P_ideal / (kM T Omega)."""
    return float(potencia_ideal(T, rho, radio) / (k_m * T * om))


def k_m_coherente(T, om, rho, radio, fom):
    """kM tal que FoM_implicita = fom en el punto (T, om) (hover)."""
    return float(potencia_ideal(T, rho, radio) / (fom * T * om))


# ---------------------------------------------------------------- Reynolds
def reynolds(rho, mu, om, radio, cuerda):
    """Re de punta basado en cuerda: rho * Omega R * c / mu."""
    return rho * om * radio * cuerda / mu


def f_reynolds(re, re_ref=9.0e4, expo=0.2):
    """Factor parametrico min(1, (Re/Re_ref)^n); n por defecto 0.2 (CT) o 0.297 (FM), de ajustar -26 % y -36 % entre
    Re 9e4 y 2e4 (SOLO el extracto del resumen de Perez Perez 2020; tabla NO abierta -> escenario parametrico)."""
    re = np.asarray(re, float)
    return np.minimum(1.0, (re / re_ref) ** expo)


def exponente_desde_caida(caida, re_bajo=2.0e4, re_alto=9.0e4):
    """n tal que (re_bajo/re_alto)^n = 1 - caida."""
    return float(np.log(1.0 - caida) / np.log(re_bajo / re_alto))


# ---------------------------------------------------------------- barridos de diseno (hover con y sin Re)
def escala_minima_hover(rho, g, mu=None, *, masa=1.5, g0=9.80665, empuje_peso=2.2, om_max=1000.0, rho_ref=1.225,
                        radio=0.127, cuerda=0.02, re_ref=9.0e4, expo_kT=0.2, geometria='fija'):
    """`rotor_escala` minima para que el empuje maximo alcance el de hover (T/W >= 1).

    Solo densidad: s = g rho_ref / (empuje_peso g0 rho). Con f(Re) (necesita mu): se resuelve
    s * fT(Re(om_max; s)) = s_solo_densidad por biseccion. `geometria`:
      'fija'     -> R y cuerda no cambian con s (s es un multiplicador abstracto de kT);
      'escalada' -> R y c crecen como s^(1/4) (T ~ rho R^4 con solidez fija; SUPUESTO), asi Re ~ s^(1/2).
    """
    s0 = g * rho_ref / (empuje_peso * g0 * rho)
    if mu is None:
        return float(s0)

    def deficit(s):
        k = s ** 0.25 if geometria == 'escalada' else 1.0
        re = reynolds(rho, mu, om_max, radio * k, cuerda * k)
        return s * float(f_reynolds(re, re_ref, expo_kT)) - s0

    lo, hi = s0, s0
    while deficit(hi) < 0.0:
        hi *= 2.0
        if hi > 1e9:
            return float('inf')
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if deficit(mid) < 0.0 else (lo, mid)
    return float(hi)


def hover_con_re(rho, g, mu, escala, *, masa=1.5, g0=9.80665, empuje_peso=2.2, om_max=1000.0, rho_ref=1.225,
                 radio=0.127, cuerda=0.02, re_ref=9.0e4, expo_kT=0.2, expo_fom=0.297, fom=0.6, eta=0.8, p_av=15.0):
    """Hover con geometria fija: om que da T = m g/4 con T = kT f_T(Re(om)) om^2, y potencia total (W).

    Devuelve None si el hover no se alcanza (T(om_max) < m g/4)."""
    kt = empuje_peso * masa * g0 / 4.0 / om_max ** 2 * (rho / rho_ref) * escala
    T_h = masa * g / 4.0

    def T(om):
        return kt * om * om * float(f_reynolds(reynolds(rho, mu, om, radio, cuerda), re_ref, expo_kT))

    if T(om_max) < T_h:
        return None
    lo, hi = 0.0, om_max
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if T(mid) < T_h else (lo, mid)
    om = hi
    re = reynolds(rho, mu, om, radio, cuerda)
    ff = float(f_reynolds(re, re_ref, expo_fom))
    p = 4.0 * potencia_ideal(T_h, rho, radio) / (fom * ff * eta) + p_av
    return {'om': float(om), 're': float(re), 'f_kT': float(f_reynolds(re, re_ref, expo_kT)), 'f_fom': ff,
            'potencia_w': float(p)}
