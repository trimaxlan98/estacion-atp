"""Validacion de atp_optico.nubes: marginales, correlacion espacial, diversidad (con el contraste de la red alemana)."""
import numpy as np
import pytest

from atp_optico import nubes as N

S2 = [N.Sitio('a', 40.4, -3.7, 0.637), N.Sitio('b', 19.4, -99.1, 0.55)]


def test_marginales_exactas():
    sitios = [N.Sitio('a', 40.4, -3.7, 0.637), N.Sitio('b', 36.8, -2.5, 0.65), N.Sitio('c', 69.6, 19.0, 0.30)]
    c = N.serie_nubes(sitios, 500000, semilla=1)
    for i, s in enumerate(sitios):
        assert float(np.mean(c[i])) == pytest.approx(s.p_claro, abs=0.012)


def test_correlacion_espacial_contra_la_copula_analitica():
    # dos sitios a ~500 km: P(ambos despejados) de la simulacion = Phi_2(.., rho = exp(-d/L))
    a, b = N.Sitio('a', 40.4, -3.7, 0.637), N.Sitio('b', 36.8, -2.5, 0.65)
    d = N.distancia_km(a.lat_deg, a.lon_deg, b.lat_deg, b.lon_deg)
    assert 380 < d < 450
    rho = float(np.exp(-d / 800.0))
    c = N.serie_nubes([a, b], 600000, L_km=800.0, semilla=2)
    ambos = float(np.mean(c[0] & c[1]))
    assert ambos == pytest.approx(N.p_ambas_claras(a.p_claro, b.p_claro, rho), abs=0.012)
    # la diversidad: P(alguna) analitica = p1 + p2 - P(ambas)
    assert N.disponibilidad(c) == pytest.approx(N.disponibilidad_analitica_2(a.p_claro, b.p_claro, rho), abs=0.012)
    # y una correlacion positiva da MENOS diversidad que nubes independientes
    assert N.disponibilidad_analitica_2(a.p_claro, b.p_claro, rho) < N.disponibilidad_independiente([a.p_claro, b.p_claro])


def test_limites_de_la_diversidad():
    s = [N.Sitio('a', 40.0, 0.0, 0.5), N.Sitio('b', 40.0, 0.0, 0.5)]        # el mismo lugar: rho = 1
    assert N.disponibilidad_analitica_2(0.5, 0.5, 1.0 - 1e-9) == pytest.approx(0.5, abs=1e-3)
    assert N.disponibilidad_analitica_2(0.5, 0.5, 0.0) == pytest.approx(0.75, abs=1e-9)
    c = N.serie_nubes(s, 50000, semilla=3)
    assert N.disponibilidad(c) == pytest.approx(0.5, abs=0.02)             # estaciones iguales: sin ganancia
    lejos = [N.Sitio('a', 40.0, -3.0, 0.5), N.Sitio('b', -35.0, 150.0, 0.5)]
    assert N.disponibilidad(N.serie_nubes(lejos, 100000, semilla=3)) == pytest.approx(0.75, abs=0.02)


def test_mas_estaciones_mas_disponibilidad():
    sitios = [N.Sitio('a', 19.4, -99.1, 0.55), N.Sitio('b', 40.4, -3.7, 0.637), N.Sitio('c', -0.2, -78.5, 0.35),
              N.Sitio('d', 69.6, 19.0, 0.30)]
    c = N.serie_nubes(sitios, 200000, semilla=4)
    d1 = N.disponibilidad(c, [0])
    d2 = N.disponibilidad(c, [0, 1])
    d3 = N.disponibilidad(c, [0, 1, 2])
    d4 = N.disponibilidad(c)
    assert d1 < d2 < d3 < d4
    assert d4 <= N.disponibilidad_independiente([s.p_claro for s in sitios]) + 0.01


def test_reproducible_por_semilla():
    a = N.serie_nubes(S2, 2000, semilla=9)
    b = N.serie_nubes(S2, 2000, semilla=9)
    c = N.serie_nubes(S2, 2000, semilla=10)
    assert np.array_equal(a, b) and not np.array_equal(a, c)


def test_persistencia_temporal():
    # tau = 8 h: las rachas despejadas/nubladas duran del orden de horas, no de un paso
    c = N.serie_nubes([N.Sitio('a', 40, 0, 0.5)], 200000, paso_h=1.0, tau_h=8.0, semilla=5)[0]
    cambios = np.mean(c[1:] != c[:-1])
    # AR(1) con umbral en la mediana: P(cambio de signo) = arccos(phi)/pi (Sheppard); sin memoria seria 0.5
    assert cambios == pytest.approx(np.arccos(np.exp(-1.0 / 8.0)) / np.pi, abs=0.01)


def test_contraste_con_la_red_alemana_de_estaciones_opticas():
    """arXiv 2402.13282 (resumen leido): sitios con disponibilidad media 65 % (Almeria), 25 % (Trauen), 34 % (Neubiberg),
    32 % (Neustrelitz) y 83.68 % de disponibilidad de la red. Con nubes independientes saldria 88.2 %; la longitud
    de correlacion L = 500 km de `data/clima_optico.yaml` SE AJUSTO a este unico dato (L ~ 480 km da 83.7 %), asi que
    esta prueba verifica el ajuste y que la cota independiente es una cota, no una validacion independiente."""
    alem = [N.Sitio('almeria', 36.84, -2.46, 0.65), N.Sitio('trauen', 52.90, 10.00, 0.25),
            N.Sitio('neubiberg', 48.08, 11.67, 0.34), N.Sitio('neustrelitz', 53.36, 13.07, 0.32)]
    ind = N.disponibilidad_independiente([s.p_claro for s in alem])
    assert ind == pytest.approx(0.882, abs=0.002)
    c = N.disponibilidad(N.serie_nubes(alem, 300000, L_km=500.0, semilla=6))
    assert abs(c - 0.8368) < 0.02 and c < ind
    # y con L mayor (nubes mas correladas) la diversidad rinde menos
    assert N.disponibilidad(N.serie_nubes(alem, 100000, L_km=1500.0, semilla=6)) < c


def test_clima_del_repositorio():
    sitios, mod = N.cargar_clima()
    assert set(sitios) == {'madrid', 'cdmx', 'tromso', 'ecuador'}
    assert sitios['madrid'].p_claro == pytest.approx(0.637)
    assert sitios['cdmx'].lat_deg == pytest.approx(19.4326)
    assert mod['tau_h'] > 0 and mod['L_km'] > 0
