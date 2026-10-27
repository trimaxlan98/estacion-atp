"""Cuerpos del sistema solar donde puede rodar el rover (CONTRATO sec. 18).

`PLANETAS[nombre]` -> dict con `g` (m/s2), `suelo` (parametros Bekker-Wong), `densidad_atm` (kg/m3 en la
superficie), `temp_k` (temperatura media de superficie), `retardo_tierra_s` (ida y vuelta TIPICA de una senal
Tierra-cuerpo, para la sec. 19) y `nombre_es`.

Unidades del suelo (Bekker 1969; Wong 2010, "Terramechanics and Off-Road Vehicle Engineering", cap. 2):
    presion bajo la rueda  p = (kc/b + kphi) * z**n    (b = ancho de la rueda en m, z = hundimiento en m)
    kc    N/m^(n+1)      modulo de cohesion
    kphi  N/m^(n+2)      modulo de friccion
    n     -              exponente de hundimiento
    c     Pa             cohesion
    phi_deg  grados      angulo de friccion interna
    k_deslizamiento (K) m  modulo de deformacion cortante de Janosi-Hanamoto: tau = (c + p tan(phi)) (1 - exp(-j/K))

HONESTIDAD DE LOS DATOS. Solo la Tierra (arena seca) y la Luna (regolito, ensayos Apollo) tienen parametros
Bekker publicados que conozco con confianza razonable; los de Marte, Titan y Ceres son SUPUESTOS de orden de
magnitud (suelo marciano tipo arena fina, Titan «arena humeda/organicos blandos», Ceres regolito parecido al
lunar con menos cohesion) y NO estan medidos para este modelo. Cambian el resultado numerico, no la estructura.
Cada valor lleva su fuente o la marca SUPUESTO. Revisar antes de citar una cifra en una tesis.
"""
from copy import deepcopy

# Gravedad superficial (m/s2): fichas de la NASA (NSSDCA Planetary Fact Sheets).
# Titan: NSSDCA Saturn Satellite Fact Sheet. Ceres: Park et al. 2016 (Dawn). Tierra: g0 = 9.80665 (CGPM 1901).
PLANETAS = {
    'tierra': {
        'nombre_es': 'Tierra',
        'g': 9.80665,
        # Arena seca, Wong 2010 (valores clasicos de Wong/Reece): n=1.1, kc=0.99 kN/m^(n+1),
        # kphi=1528.43 kN/m^(n+2), c=1.04 kPa, phi=28 grados, K=0.025 m.
        'suelo': dict(kc=990.0, kphi=1528430.0, n=1.1, c=1040.0, phi_deg=28.0, k_deslizamiento=0.025),
        'densidad_atm': 1.225,      # kg/m3, atmosfera estandar ISA a nivel del mar
        'temp_k': 288.15,           # ISA
        'retardo_tierra_s': 0.0,
    },
    'luna': {
        'nombre_es': 'Luna',
        'g': 1.62,                  # 1.62 m/s2 (NSSDCA Moon Fact Sheet)
        # Regolito lunar, orden de magnitud de Bekker (1969) / Wong 2010: n=1.0, kc=1.4 kPa, kphi=820 kPa/m,
        # c=0.17 kPa, phi=35 grados. K=0.018 m: SUPUESTO (tipico de suelos granulares sueltos, no medido).
        'suelo': dict(kc=1400.0, kphi=820000.0, n=1.0, c=170.0, phi_deg=35.0, k_deslizamiento=0.018),
        'densidad_atm': 1e-14,      # kg/m3: exosfera, ~0 a estos efectos (SUPUESTO de orden de magnitud)
        'temp_k': 250.0,            # media aproximada (dia ~390 K, noche ~100 K, Vasavada 2012)
        'retardo_tierra_s': 2.56,   # 2 x 1.28 s (luz a 384 400 km)
    },
    'marte': {
        'nombre_es': 'Marte',
        'g': 3.71,                  # 3.71 m/s2 (NSSDCA Mars Fact Sheet)
        # SUPUESTO de orden de magnitud: suelo marciano tipo arena fina compacta, mas cohesivo que el lunar
        # (resultados de MER/Spirit: suelo suelto con c ~ 0.1-1 kPa y phi ~ 30-37, Sullivan et al. 2011).
        'suelo': dict(kc=1370.0, kphi=800000.0, n=1.0, c=800.0, phi_deg=33.0, k_deslizamiento=0.02),
        'densidad_atm': 0.020,      # kg/m3 (NSSDCA, presion ~610 Pa)
        'temp_k': 210.0,            # media global aproximada (NSSDCA: 210 K)
        'retardo_tierra_s': 1500.0, # 2 x ~750 s (distancia media ~2.25e8 km); varia de ~360 s a ~2600 s
    },
    'titan': {
        'nombre_es': 'Titan',
        'g': 1.352,                 # 1.352 m/s2 (NSSDCA)
        # SUPUESTO: no hay terramecanica medida. Huygens (Zarnecki et al. 2005, Nature 438) vio una corteza fina
        # sobre material blando tipo arena humeda; se toma un suelo blando, poco cohesivo, con phi bajo.
        'suelo': dict(kc=900.0, kphi=400000.0, n=1.0, c=300.0, phi_deg=26.0, k_deslizamiento=0.03),
        'densidad_atm': 5.4,        # kg/m3 en superficie (Fulchignoni et al. 2005, Huygens HASI)
        'temp_k': 94.0,             # 93.7 K (Fulchignoni et al. 2005)
        'retardo_tierra_s': 9560.0, # 2 x ~4780 s (~9.58 UA con 499 s/UA); varia con la posicion de Saturno
    },
    'ceres': {
        'nombre_es': 'Ceres',
        'g': 0.27,                  # ~0.27-0.28 m/s2 (Park et al. 2016, Nature 537, Dawn)
        # SUPUESTO: regolito de baja cohesion como el lunar; sin mediciones in situ. Con g tan baja el rover
        # casi no carga el suelo: el resultado depende sobre todo de c y de la adhesion, aqui poco fiables.
        'suelo': dict(kc=1000.0, kphi=600000.0, n=1.0, c=100.0, phi_deg=32.0, k_deslizamiento=0.018),
        'densidad_atm': 0.0,
        'temp_k': 168.0,            # SUPUESTO: media aproximada de la superficie (~235 K subsolar)
        'retardo_tierra_s': 2800.0, # 2 x ~1400 s (~2.8 UA); varia de ~1.6 a ~3.6 UA
    },
}


def planeta(nombre):
    """Copia profunda del dict del planeta (para que un experimento no altere el original)."""
    if nombre not in PLANETAS:
        raise KeyError(f'planeta desconocido: {nombre!r} (hay: {", ".join(sorted(PLANETAS))})')
    d = deepcopy(PLANETAS[nombre])
    d['clave'] = nombre
    return d
