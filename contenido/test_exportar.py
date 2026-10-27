"""Pruebas de exportar.py con corridas sinteticas (sin ROS ni Manim).

    python3 -m unittest discover -s contenido -p 'test_*.py'
"""
import json
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import exportar  # noqa: E402


def muestra(i):
    t = round(0.1 * i, 3)
    e = 0.01 + (0.2 if i == 137 else 0.0)            # un pico aislado
    return {"t": t, "ref_az": 10 + t, "ref_el": 30.0, "az": 10 + t, "el": 30.0,
            "e_az": 0.001, "e_el": 0.002, "e_pt": e, "range_km": 800.0,
            "range_rate_km_s": -3.0, "doppler_hz": 1000.0 + i,
            "doppler_rate_hz_s": 5.0, "l_point_db": 0.0, "cn0_dbhz": 80.0,
            "ebn0_db": 20.0, "margin_db": 15.0, "link_closed": True,
            "u_az": 0.1, "u_el": 0.1}


METRICAS = {"n": 1000, "duration_s": 100.0,
            "pointing": {"rms": 0.02, "p95": 0.03, "max": 0.21,
                         "within_objective_pct": 99.0, "objective_deg": 0.1},
            "axes": {"az": {"rms": 0.01, "max": 0.1}, "el": {"rms": 0.01, "max": 0.1}},
            "link": {"band": "S", "margin_min_db": 14.0, "margin_max_db": 16.0,
                     "closed_pct": 100.0, "doppler_max_hz": 2000.0,
                     "doppler_min_hz": 1000.0, "doppler_rate_max_hz_s": 5.0,
                     "l_point_max_db": 0.0},
            "verdict": "CUMPLE"}


class Base(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.runs = os.path.join(self.tmp.name, "runs")
        self.out = os.path.join(self.tmp.name, "out")

    def corrida(self, nombre="r1", n=1000, metricas=METRICAS, filas=None, escribir_metricas=True):
        d = os.path.join(self.runs, nombre)
        os.makedirs(d)
        filas = filas if filas is not None else [muestra(i) for i in range(n)]
        with open(os.path.join(d, "telemetry.jsonl"), "w") as f:
            for r in filas:
                f.write(json.dumps(r) + "\n")
        if escribir_metricas:
            with open(os.path.join(d, "metrics.json"), "w") as f:
                json.dump(metricas, f)
        return d

    def exporta(self, nombre="r1", **kw):
        return exportar.exportar(nombre, self.runs, self.out, **kw)


class TestPase(Base):
    def test_feliz_y_compacto(self):
        self.corrida()
        destino, ds = self.exporta(max_puntos=200)
        self.assertEqual(len(ds["series"]["t"]), 200)
        self.assertEqual(ds["metricas"], METRICAS)          # copia literal
        self.assertTrue(os.path.isfile(destino))

    def test_conserva_primera_ultima_y_el_pico(self):
        self.corrida()
        _, ds = self.exporta(max_puntos=100)
        s = ds["series"]
        self.assertEqual(s["t"][0], 0.0)
        self.assertEqual(s["t"][-1], 99.9)
        self.assertAlmostEqual(max(s["e_pt"]), 0.21, places=6)   # el pico sobrevive
        self.assertEqual(s["t"], sorted(s["t"]))

    def test_determinista(self):
        self.corrida()
        d1, _ = self.exporta()
        with open(d1, "rb") as f:
            a = f.read()
        d2, _ = self.exporta()
        with open(d2, "rb") as f:
            self.assertEqual(a, f.read())

    def test_falta_metrics(self):
        self.corrida(escribir_metricas=False)
        with self.assertRaisesRegex(exportar.ContratoError, "metrics.json.*no termino"):
            self.exporta()

    def test_metrica_sin_campo(self):
        m = json.loads(json.dumps(METRICAS))
        del m["pointing"]["p95"]
        self.corrida(metricas=m)
        with self.assertRaisesRegex(exportar.ContratoError, "pointing: falta el campo 'p95'"):
            self.exporta()

    def test_sin_link(self):
        m = json.loads(json.dumps(METRICAS))
        del m["link"]
        self.corrida(metricas=m)
        with self.assertRaisesRegex(exportar.ContratoError, "falta el bloque 'link'"):
            self.exporta()

    def test_linea_cortada(self):
        d = self.corrida(n=50)
        with open(os.path.join(d, "telemetry.jsonl"), "a") as f:
            f.write('{"t": 5.1, "ref_az"')
        with self.assertRaisesRegex(exportar.ContratoError, r"telemetry.jsonl:51: .*invalida"):
            self.exporta()

    def test_campo_obligatorio_ausente(self):
        filas = [muestra(i) for i in range(100)]
        for f in filas:
            del f["doppler_hz"]
        self.corrida(filas=filas)
        with self.assertRaisesRegex(exportar.ContratoError, "doppler_hz"):
            self.exporta()

    def test_arranque_incompleto_se_descarta(self):
        filas = [muestra(i) for i in range(200)]
        for f in filas[:3]:                       # antes del primer dato de enlace
            del f["margin_db"]
        self.corrida(filas=filas)
        _, ds = self.exporta()
        self.assertEqual(ds["fuente"]["muestras_completas"], 197)

    def test_tiempo_que_no_crece(self):
        filas = [muestra(i) for i in range(50)]
        filas[20]["t"] = filas[19]["t"]
        self.corrida(filas=filas)
        with self.assertRaisesRegex(exportar.ContratoError, "no crece"):
            self.exporta()

    def test_corrida_inexistente(self):
        with self.assertRaisesRegex(exportar.ContratoError, "no existe la corrida"):
            self.exporta("nada")

    def test_directorio_vacio(self):
        os.makedirs(os.path.join(self.runs, "vacia"))
        with self.assertRaisesRegex(exportar.ContratoError, "no es una corrida terminada"):
            self.exporta("vacia")


class TestCampana(Base):
    def campana(self, **cambios):
        c = {"mode": "lazo", "n": 5, "p50": 0.03, "p95": 0.04, "p99": 1.0, "peor": 2.0,
             "pasa": True, "objective_deg": 0.1, "rms_deg": [0.01, 0.02, 0.03, 0.04, 2.0],
             "el_max_deg": [50, 60, 70, 80, 85]}
        c.update(cambios)
        d = os.path.join(self.runs, "c1")
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, "campaign.json"), "w") as f:
            json.dump(c, f)

    def test_feliz(self):
        self.campana()
        _, ds = self.exporta("c1")
        self.assertEqual(ds["tipo"], "campana")
        self.assertEqual(ds["campana"]["rms_deg"][-1], 2.0)

    def test_n_no_coincide(self):
        self.campana(n=300)
        with self.assertRaisesRegex(exportar.ContratoError, "rms_deg trae 5 valores y n=300"):
            self.exporta("c1")

    def test_falta_campo(self):
        self.campana()
        p = os.path.join(self.runs, "c1", "campaign.json")
        with open(p) as f:
            c = json.load(f)
        del c["p95"]
        with open(p, "w") as f:
            json.dump(c, f)
        with self.assertRaisesRegex(exportar.ContratoError, "falta el campo 'p95'"):
            self.exporta("c1")


if __name__ == "__main__":
    unittest.main()
