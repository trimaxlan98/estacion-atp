"""Escena (a) "El pase": una corrida real de la Estacion en el cielo polar.

La traza de referencia (ambar) y la antena real (cian) avanzan en el tiempo
del pase; el panel muestra las cifras MEDIDAS en ese instante (el, az, rango,
Doppler, margen de enlace) y el error contra la banda del objetivo, que esta
dibujada desde el primer cuadro. Al final, el resumen sale de `metrics.json`.

    ATP_PASE=demo-pase ATP_PLAY_S=22 manim render -ql escenas/pase.py ElPase

Formato del curso 30: sin subtitulos, rotulos cortos, color por rol.
"""
import os

from piezas import *                                        # noqa: F401,F403
from piezas import Cielo, CurvaError, fila

NOMBRE = os.environ.get("ATP_PASE", "demo-pase")
PLAY_S = float(os.environ.get("ATP_PLAY_S", "22"))


class ElPase(Escena):
    def construct(self):
        ds = Corrida(NOMBRE)
        ds.exigir("t", "ref_az", "ref_el", "az", "el", "e_pt", "range_km",
                  "doppler_hz", "margin_db")
        m = ds.metricas
        obj = m["pointing"]["objective_deg"]
        banda = m["link"]["band"]

        titulo = titulo_curso("El pase, medido")
        cielo = Cielo(ds, centro=LEFT * 4.0 + DOWN * 0.15, radio=2.4)
        curva = CurvaError(ds, ancho=5.4, alto=2.0, centro=RIGHT * 3.85 + DOWN * 1.65,
                           objetivo=obj, etiqueta_y="error", etiqueta_x="t")
        tag_obj = tag_junto(curva.traza.banda, f"{obj:g} deg", direccion=UP,
                            buff=0.08, font_size=16, color=C_CALCULO)
        tag_obj.align_to(curva.traza.banda, RIGHT)
        tag_masc = tag_junto(cielo.vista, f"mascara {cielo.el_mascara:.0f} deg",
                             direccion=DOWN, buff=0.34, font_size=16)
        tag_ban = tag_junto(cielo.vista, f"banda {banda}", direccion=DOWN,
                            buff=0.62, font_size=16, color=C_CALCULO)

        # --- entrada: cielo, referencia predicha, banda del objetivo -----
        self.play(FadeIn(titulo, shift=0.2 * DOWN), run_time=0.8)
        self.play(Create(cielo.vista), FadeIn(cielo.mascara), run_time=1.4)
        self.play(Create(cielo.ruta), FadeIn(tag_masc), FadeIn(tag_ban), run_time=1.6)
        self.play(FadeIn(curva.traza.banda), FadeIn(curva.traza.borde_sup),
                  FadeIn(curva.traza.borde_inf), FadeIn(curva.traza.cero),
                  FadeIn(curva.traza.ejes), FadeIn(tag_obj), run_time=1.2)

        # --- el pase: todo cuelga de un reloj ----------------------------
        reloj = ValueTracker(0.0)

        def t_ahora():
            return ds.t0 + reloj.get_value() * (ds.t1 - ds.t0)

        estela_ref = always_redraw(lambda: cielo.estela_ref(t_ahora()))
        estela_ant = always_redraw(lambda: cielo.estela_ant(t_ahora()))
        punto_sat = always_redraw(lambda: Dot(cielo.punto_ref(t_ahora()),
                                              radius=0.075, color=C_SAT))
        punto_ant = always_redraw(lambda: Circle(
            radius=0.17, color=C_CALCULO, stroke_width=3.2).move_to(
                cielo.punto_ant(t_ahora())))
        curva_ahora = always_redraw(lambda: curva.curva(t_ahora()).set_stroke(
            color=C_SAT, width=3.0))
        curva_fuera = always_redraw(lambda: curva.fuera(t_ahora()))
        cabeza = always_redraw(lambda: Dot(curva.cabeza(t_ahora()), radius=0.06,
                                           color=C_SAT))

        banco = banco_mono(19)
        X0, Y0, DY = 1.95, 2.72, 0.36        # celda 0 de la columna y paso de fila

        def panel():
            t = t_ahora()
            e = ds.en("e_pt", t)
            cierra = ds.en("link_closed", t) if "link_closed" in ds.s else True
            lineas = [
                (fila("T+", f"{t:.1f}", "S"), C_TENUE),
                (fila("EL", f"{ds.en('el', t):.2f}", "DEG"), C_CALCULO),
                (fila("AZ", f"{ds.en('az', t):.2f}", "DEG"), C_CALCULO),
                (fila("RANGO", f"{ds.en('range_km', t):.1f}", "KM"), C_CALCULO),
                (fila("DOPPLER", f"{ds.en('doppler_hz', t) / 1000:+.2f}", "KHZ"), C_CALCULO),
                (fila("MARGEN", f"{ds.en('margin_db', t):.1f}", "DB"),
                 C_OK if cierra else C_PELIGRO),
                (fila("ERROR", f"{e:.4f}", "DEG"), C_CALCULO if e <= obj else C_PELIGRO),
            ]
            g = VGroup()
            for j, (tx, col) in enumerate(lineas):
                vigilar(tx, MAX_CIFRA + 1, "panel")
                g.add(banco.linea(tx, col, (X0, Y0 - j * DY)))
            return g

        fondo_panel = Rectangle(
            width=banco.ancho(24) + 0.3, height=7 * DY + 0.2, stroke_width=0,
            fill_color=CODE_BG, fill_opacity=0.78).move_to(
                RIGHT * (X0 + banco.ancho(23) / 2)
                + UP * (Y0 - 3 * DY))
        self.add(fondo_panel)
        cuadro = always_redraw(panel)
        self.add(estela_ref, estela_ant, punto_sat, punto_ant, curva_ahora,
                 curva_fuera, cabeza, cuadro)
        self.cifras.marcar_vivo(ds, self.renderer.time, PLAY_S)
        self.play(reloj.animate.set_value(1.0), run_time=PLAY_S, rate_func=linear)

        # --- el cierre: el resumen sale de metrics.json ------------------
        cif = self.cifras
        filas_res = (
            (fila("RMS", cif.de(ds, "pointing.rms"), "DEG"), C_CALCULO),
            (fila("P95", cif.de(ds, "pointing.p95"), "DEG"), C_SAT),
            (fila("MAX", cif.de(ds, "pointing.max"), "DEG"), C_CALCULO),
            (fila("EN BANDA", cif.de(ds, "pointing.within_objective_pct", "{:.1f}"), "%"), C_CALCULO),
            (fila("MARGEN MIN", cif.de(ds, "link.margin_min_db", "{:.2f}"), "DB"), C_OK),
            (fila("DOPPLER MAX", cif.de(ds, "link.doppler_max_hz", "{:.2f}", 1e-3), "KHZ"), C_CALCULO),
        )
        res = VGroup(*[banco.linea(tx, col, (X0, Y0 - (j + 0.5) * DY))
                       for j, (tx, col) in enumerate(filas_res)])
        ok = m["verdict"] == "CUMPLE"
        veredicto = cifra_pie(m["verdict"], color=C_OK if ok else C_PELIGRO)
        self.play(FadeOut(cuadro), run_time=0.5)
        self.remove(cuadro)
        self.play(FadeIn(res), FadeIn(veredicto, shift=0.2 * UP), run_time=1.0)
        self.wait(4.5)
