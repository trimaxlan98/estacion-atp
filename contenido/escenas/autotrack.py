"""Escena (b) "Program track contra autotrack": dos corridas, mismo pase.

Dos columnas con el MISMO cielo, la MISMA escala y la MISMA banda de 0.1 deg.
A la izquierda la corrida N5a (program track: la antena sigue solo la
efemeride, que va atrasada y con sesgo); a la derecha la N5b (autotrack: el
conical scan centra la antena con la senal). Cada columna lleva el cielo con
sus dos trazas, la "mira" (la antena vista desde el satelite, con el anillo
del objetivo) y el error contra el tiempo.

    ATP_PROGRAM=demo-n5a ATP_AUTOTRACK=demo-n5b ATP_PLAY_S=20 \
        manim render -ql escenas/autotrack.py ProgramContraAutotrack

Toda cifra sale de los datasets; el resumen final, de metrics.json.
"""
import os

from piezas import *                                        # noqa: F401,F403
from piezas import Cielo, CurvaError, polilinea

PROGRAM = os.environ.get("ATP_PROGRAM", "demo-n5a")
AUTOTRACK = os.environ.get("ATP_AUTOTRACK", "demo-n5b")
PLAY_S = float(os.environ.get("ATP_PLAY_S", "20"))


def _rol(ds, esperado):
    """Si el dataset trae params, el rol debe coincidir con `autotrack`."""
    a = ds.params.get("args", {}).get("autotrack")
    if a is not None and str(a).lower() != esperado:
        raise ValueError(f"{ds.id}: se uso como autotrack={esperado} pero la "
                         f"corrida se lanzo con autotrack={a}")


def desvio(ds):
    """Posicion de la antena respecto al satelite en la mira (grados en el
    cielo): -(e_az cos el_verdad, e_el). e = verdad - medida."""
    el_v = ds.s["el"] + ds.s["e_el"]
    return np.column_stack([-ds.s["e_az"] * np.cos(np.radians(el_v)), -ds.s["e_el"]])


class Columna:
    """Una corrida dibujada: cielo, mira y error. Todo cuelga de `t()`."""

    def __init__(self, ds, cx, titulo, y_max, ventana, objetivo, reloj):
        self.ds, self.reloj, self.obj = ds, reloj, objetivo
        self.cielo = Cielo(ds, centro=RIGHT * (cx - 1.8) + UP * 1.2, radio=1.2,
                           font_size=12)
        self.curva = CurvaError(ds, ancho=5.8, alto=1.9, centro=RIGHT * cx + DOWN * 1.5,
                                objetivo=objetivo, y_max=y_max,
                                etiqueta_x="t", etiqueta_y="error")
        # mira: el satelite al centro, la antena desplazada por el error
        self.c_mira = np.array([cx + 1.7, 1.2, 0.0])
        self.r_mira = 1.2
        self.escala = self.r_mira / ventana
        self.d = desvio(ds)
        self.mira = self._mira(ventana)
        self.etq = Text(titulo, font_size=26, color=C_TITULO)
        self.etq.move_to(RIGHT * (cx - 3.25) + UP * 2.92)
        self.etq.shift(RIGHT * self.etq.width / 2)
        self.cx = cx
        self.banco = banco_mono(19)
        self.lectura = None

    def ancla_lectura(self, n_celdas):
        """Celda 0 de una linea de `n_celdas` pegada al borde derecho de la
        columna (cx + 3.3), en la fila de la etiqueta."""
        return (self.cx + 3.3 - self.banco.avance * (n_celdas - 0.5), 2.92)

    def _mira(self, ventana):
        c, r = self.c_mira, self.r_mira
        marco = Circle(radius=r, color=C_EJE, stroke_width=2.4).move_to(c)
        cruz = VGroup(Line(c + LEFT * r, c + RIGHT * r, stroke_width=1.4, color=C_EJE),
                      Line(c + DOWN * r, c + UP * r, stroke_width=1.4, color=C_EJE))
        ro = self.obj * self.escala
        anillo = Circle(radius=ro, color=C_CALCULO, stroke_width=2.2).move_to(c)
        anillo.set_fill(C_CALCULO, opacity=0.14)
        sat = Dot(c, radius=0.06, color=C_SAT)
        tag = Text(f"+-{ventana:.2f} deg", font=FUENTE_HUD, font_size=13,
                   color=C_TENUE).next_to(marco, DOWN, buff=0.10)
        self.anillo, self.sat_mira = anillo, sat
        return VGroup(marco, cruz, anillo, sat, tag)

    def t(self):
        return self.ds.t0 + self.reloj.get_value() * (self.ds.t1 - self.ds.t0)

    def i(self):
        return int(np.clip(self.curva.idx(self.t()), 0, len(self.d) - 1))

    def vivos(self):
        """Mobjects que se redibujan con el reloj."""
        t, ds = self.t, self.ds
        c = self.c_mira

        def antena_mira():
            i = self.i()
            p = c + np.array([self.d[i, 0], self.d[i, 1], 0.0]) * self.escala
            fuera = ds.s["e_pt"][i] > self.obj
            return Circle(radius=0.12, color=C_PELIGRO if fuera else C_CALCULO,
                          stroke_width=3.4).move_to(p)

        def rastro_mira():
            i = max(self.i(), 1)
            pts = c + np.column_stack([self.d[:i + 1], np.zeros(i + 1)]) * self.escala
            return polilinea(pts, C_CALCULO, 1.6, 0.55)

        def lectura():
            e = ds.en("e_pt", t())
            tx = f"ERROR {e:.3f} DEG"
            return self.banco.linea(tx, C_CALCULO if e <= self.obj else C_PELIGRO,
                                    self.ancla_lectura(len(tx)))

        self.lectura = always_redraw(lectura)
        return [
            always_redraw(lambda: self.cielo.estela_ref(t())),
            always_redraw(lambda: self.cielo.estela_ant(t())),
            always_redraw(lambda: Dot(self.cielo.punto_ref(t()), radius=0.06, color=C_SAT)),
            always_redraw(lambda: Circle(radius=0.12, color=C_CALCULO, stroke_width=3.0
                                         ).move_to(self.cielo.punto_ant(t()))),
            always_redraw(rastro_mira),
            always_redraw(antena_mira),
            always_redraw(lambda: self.curva.curva(t()).set_stroke(color=C_SAT, width=3.0)),
            always_redraw(lambda: self.curva.fuera(t())),
            always_redraw(lambda: Dot(self.curva.cabeza(t()), radius=0.055, color=C_SAT)),
            self.lectura,
        ]


class ProgramContraAutotrack(Escena):
    def construct(self):
        a, b = Corrida(PROGRAM), Corrida(AUTOTRACK)
        for ds in (a, b):
            ds.exigir("t", "az", "el", "e_az", "e_el", "e_pt")
        _rol(a, "false")
        _rol(b, "true")
        obj = a.metricas["pointing"]["objective_deg"]
        if b.metricas["pointing"]["objective_deg"] != obj:
            raise ValueError("las dos corridas deben compartir el objetivo")
        # MISMA escala vertical y de mira para las dos: la comparacion es justa
        pico = max(float(a.s["e_pt"].max()), float(b.s["e_pt"].max()))
        y_max = max(pico * 1.25, obj * 1.6)
        ventana = max(pico * 1.25, obj * 1.6)

        reloj = ValueTracker(0.0)
        izq = Columna(a, -3.55, "PROGRAM TRACK", y_max, ventana, obj, reloj)
        der = Columna(b, 3.55, "AUTOTRACK", y_max, ventana, obj, reloj)
        titulo = titulo_curso("Program track contra autotrack")
        tag_ob = [tag_junto(c.curva.traza.banda, f"{obj:g} deg", direccion=DOWN,
                            buff=0.06, font_size=14, color=C_CALCULO) for c in (izq, der)]
        for tg, c in zip(tag_ob, (izq, der)):
            tg.align_to(c.curva.traza.banda, LEFT).shift(RIGHT * 0.1)
        sep = Line(UP * 3.0, DOWN * 2.9, stroke_width=1.2, color=C_EJE)

        # --- entrada: lo mismo en las dos columnas -----------------------
        self.play(FadeIn(titulo, shift=0.2 * DOWN), Create(sep), run_time=0.9)
        for c in (izq, der):
            self.play(FadeIn(c.etq), Create(c.cielo.vista), FadeIn(c.cielo.mascara),
                      FadeIn(c.mira), run_time=1.3)
        self.play(*[Create(c.cielo.ruta) for c in (izq, der)], run_time=1.2)
        tr = [c.curva.traza for c in (izq, der)]
        self.play(*[FadeIn(VGroup(t.banda, t.borde_sup, t.borde_inf, t.cero, t.ejes))
                    for t in tr], *[FadeIn(g) for g in tag_ob], run_time=1.2)

        # --- el pase: las dos corridas, a la vez --------------------------
        vivos = izq.vivos() + der.vivos()
        self.add(*vivos)
        self.cifras.marcar_vivo(a, self.renderer.time, PLAY_S)
        self.cifras.marcar_vivo(b, self.renderer.time, PLAY_S)
        self.play(reloj.animate.set_value(1.0), run_time=PLAY_S, rate_func=linear)

        # --- el resumen sale de metrics.json ------------------------------
        cif = self.cifras
        salida = []
        for ds, c in ((a, izq), (b, der)):
            ok = ds.metricas["verdict"] == "CUMPLE"
            pct = cif.de(ds, "pointing.within_objective_pct", "{:.1f}")
            rms = cif.de(ds, "pointing.rms")
            veredicto = ds.metricas["verdict"]
            col = C_OK if ok else C_PELIGRO
            pie = Text(f"{veredicto}  {pct} % EN BANDA", font=FUENTE_HUD,
                       font_size=22, color=col)
            vigilar(f"{veredicto} {pct} EN BANDA", MAX_CIFRA, "cierre")
            pie.move_to(RIGHT * c.curva.traza.get_center()[0] + DOWN * 3.38)
            tx = f"RMS {rms} DEG"
            r = c.banco.linea(tx, C_CALCULO, c.ancla_lectura(len(tx)))
            salida.append((con_fondo(pie), r))
        self.play(*[FadeOut(c.lectura) for c in (izq, der)], run_time=0.4)
        self.remove(izq.lectura, der.lectura)
        self.play(*[FadeIn(g, shift=0.2 * UP) for p, r in salida for g in (p, r)],
                  run_time=1.0)
        self.wait(5.0)
