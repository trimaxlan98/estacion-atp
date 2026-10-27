"""Cuerpos donde puede volar el dron: gravedad y densidad atmosferica de superficie.

g y densidad (kg/m3): NSSDCA Planetary/Satellite Fact Sheets (Tierra ISA 1.225; Marte ~0.020 con 610 Pa;
Titan ~5.4 en superficie, Fulchignoni et al. 2005, Nature 438; Luna: exosfera, ~0). La viscosidad y el numero de
Reynolds de los rotores NO se modelan: el empuje solo escala con la densidad (T ~ rho w^2), un SUPUESTO que a
Reynolds bajos (Marte) sobrestima el empuje; vale para comparar, no para predecir una mision.
"""
CUERPOS = {
    'tierra': {'nombre_es': 'Tierra', 'g': 9.80665, 'rho': 1.225},
    'marte': {'nombre_es': 'Marte', 'g': 3.71, 'rho': 0.020},
    'titan': {'nombre_es': 'Titan', 'g': 1.352, 'rho': 5.4},
    'luna': {'nombre_es': 'Luna', 'g': 1.62, 'rho': 1e-14},
}
RHO_REF = 1.225

# Viscosidad dinamica por Sutherland (P8; solo se usa con la bandera `f_reynolds`, apagada por defecto).
# mu = mu0 (T/T0)^1.5 (T0+S)/(T+S). Constantes SOLO donde hay fuente leida (extracto de buscador, documento no abierto):
#   aire: mu0=1.716e-5 Pa s, T0=273.15 K, S=110.4 K (CFD-Online, ley de Sutherland);
#   CO2 : mu0=1.370e-5 Pa s, T0=273 K, S=222 K (arXiv 2210.01621, valida ~190-1700 K).
# Titan (N2): NO VERIFICADO -> sin entrada; Luna: sin atmosfera. T_K es SUPUESTO de orden de magnitud (Tierra: ISA 288.15 K).
SUTHERLAND = {
    'tierra': {'mu0': 1.716e-5, 'T0': 273.15, 'S': 110.4, 'T_K': 288.15},
    'marte': {'mu0': 1.370e-5, 'T0': 273.0, 'S': 222.0, 'T_K': 210.0},
}


def viscosidad(nombre, T_K=None):
    """mu (Pa s) del cuerpo, o None si no hay constantes verificadas (Titan, Luna)."""
    s = SUTHERLAND.get(nombre)
    if s is None:
        return None
    T = s['T_K'] if T_K is None else T_K
    return float(s['mu0'] * (T / s['T0']) ** 1.5 * (s['T0'] + s['S']) / (T + s['S']))


def cuerpo(nombre):
    try:
        return CUERPOS[nombre]
    except KeyError:
        raise ValueError(f'cuerpo desconocido: {nombre!r} (hay: {", ".join(sorted(CUERPOS))})')
