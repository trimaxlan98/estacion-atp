"""La verdad del mundo: el .npz que lee el generador y la serializacion del modelo."""
import json

import numpy as np

from atp_mision import mundo as W
from atp_ros import od, orbit
from atp_ros import propagador as P


def test_efemeride_de_la_verdad_ida_y_vuelta():
    """El .npz de la verdad es exacto, y el modelo con variacion de densidad (la verdad)
    sobrevive a la serializacion de la OD."""
    import tempfile
    cat = orbit.cargar_catalogo()
    est, _ = orbit.cargar_estaciones()
    with tempfile.TemporaryDirectory() as d:
        w = W.Mundo([orbit.buscar_tle(cat, 'NOAA 19')], {'tromso': est['tromso']},
                    orbit.a_utc('2026-10-03T00:00:00'), carpeta=d)
        ruta = w.archivo_verdad('NOAA 19')
        ef, reloj = od.cargar_efemeride(ruta)
    e0 = w.esc['NOAA 19']
    assert reloj.epoca == e0.reloj.epoca
    t = np.array([1000.0, 40000.5, 80000.25])
    r1, v1 = ef.estado(t)
    r2, v2 = e0.ef.estado(t)
    assert np.array_equal(r1, r2) and np.array_equal(v1, v2)
    m = e0.modelo_verdad
    assert m.rho_var
    m2 = od.modelo_de_dict(json.loads(json.dumps(od.modelo_a_dict(m))))
    assert m2.rho_var == m.rho_var and m2.cd_a_m == m.cd_a_m
    # un modelo sin variacion no agrega la clave: el JSON de la OD de siempre no cambia
    assert 'rho_var' not in od.modelo_a_dict(P.Modelo(grado=2))
