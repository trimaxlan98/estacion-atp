"""Escena (c) "La campana": el histograma Monte Carlo de una campana real.

Los N pases de `campaign.json` caen uno a uno en su barra (el error RMS de
cada pase); al final aparecen el p95 (ambar) y el objetivo de 0.1 deg (cian,
la banda de siempre). La cola larga que queda fuera de la escala se cuenta
y se rotula en rojo: un p95 que cumple no dice nada de ella.

    ATP_CAMPANA=demo-campana ATP_PLAY_S=16 \
        manim render -ql escenas/campana.py LaCampana

Toda cifra sale de `campaign.json` (via el dataset).
"""
import os

from piezas import *                                        # noqa: F401,F403

NOMBRE = os.environ.get("ATP_CAMPANA", "demo-campana")
PLAY_S = float(os.environ.get("ATP_PLAY_S", "16"))
BINS = 26
ANCHO, ALTO = 8.0, 3.3


class LaCampana(Escena):
    def construct(self):
        ds = Corrida(NOMBRE)
        c = ds.campana
        if c is None:
            raise ValueError(f"{ds.id}: no es una campana (falta campaign.json)")
        m = np.asarray(c["rms_deg"], dtype=float)
        n = len(m)
        obj = c["objective_deg"]
        x_max = 1.5 * obj          # escala del eje: una vez y media el objetivo
        fuera = int(np.sum(m > x_max))

        titulo = titulo_curso("La campana Monte Carlo")
        h = histograma(m, bins=BINS, ancho=ANCHO, alto=ALTO, umbral=obj,
                       x_max=x_max, font_size=15)
        if abs(h.p95 - c["p95"]) > 1e-9:
            raise ValueError(f"el p95 que dibuja la libreria ({h.p95:.6f}) no es "
                             f"el de campaign.json ({c['p95']:.6f})")
        h.move_to(LEFT * 2.35 + DOWN * 0.45)
        conteo, _ = np.histogram(m, bins=BINS, range=(0.0, x_max))
        tope = max(int(conteo.max()), 1)
        base_y = h.barras[0].get_bottom()[1]
        k = ValueTracker(0)

        def barras():
            kk = int(round(k.get_value()))
            cnt, _ = np.histogram(m[:kk], bins=BINS, range=(0.0, x_max))
            g = VGroup()
            for i, b in enumerate(h.barras):
                r = Rectangle(width=b.width, height=max(ALTO * cnt[i] / tope, 1e-3),
                              stroke_width=0, fill_color=C_CIELO, fill_opacity=0.75)
                r.move_to([b.get_center()[0], base_y, 0], aligned_edge=DOWN)
                g.add(r)
            return g

        banco = banco_mono(21)

        def contador():
            return banco.linea(f"N {int(round(k.get_value()))} DE {n}", C_CALCULO,
                               (3.3, 2.7))

        # --- entrada: ejes y los DOS referentes ya dibujados -------------
        self.play(FadeIn(titulo, shift=0.2 * DOWN), run_time=0.8)
        self.play(FadeIn(h.ejes), run_time=0.8)
        self.play(Create(h.linea_umbral), FadeIn(h.tag_umbral), run_time=1.0)

        # --- los pases caen uno a uno ------------------------------------
        vivas = always_redraw(barras)
        cuenta = always_redraw(contador)
        self.add(vivas, cuenta)
        self.play(k.animate.set_value(n), run_time=PLAY_S, rate_func=smooth)
        self.remove(vivas)
        self.add(h.barras)               # las barras finales de la libreria

        # --- el p95 y la cola ---------------------------------------------
        self.play(Create(h.linea_p95), FadeIn(h.tag_p95), run_time=1.2)
        cif = self.cifras
        panel = VGroup(*[tag_hud(tx, font_size=20, color=col) for tx, col in (
            (f"P50 {cif.de(ds, 'campana.p50')} DEG", C_CALCULO),
            (f"P95 {cif.de(ds, 'campana.p95')} DEG", C_SAT),
            (f"P99 {cif.de(ds, 'campana.p99', '{:.2f}')} DEG", C_PELIGRO),
            (f"PEOR {cif.de(ds, 'campana.peor', '{:.2f}')} DEG", C_PELIGRO),
        )])
        panel.arrange(DOWN, buff=0.22, aligned_edge=LEFT)
        panel.move_to(RIGHT * 4.9 + UP * 1.3)
        panel.set_x(4.9)
        self.play(FadeIn(con_fondo(panel, buff=0.16, opacidad=0.78), shift=0.2 * LEFT),
                  run_time=1.0)
        extremos = VGroup(
            tag_junto(h.ejes, "0", direccion=DOWN, buff=0.12, font_size=14).align_to(
                h.ejes, LEFT),
            tag_junto(h.ejes, f"{x_max:g}", direccion=DOWN, buff=0.12, font_size=14
                      ).align_to(h.ejes, RIGHT).shift(LEFT * 0.9))
        self.add(extremos)
        if fuera:
            borde_der = h.ejes.get_right()[0]
            tag_cola = tag_hud(f"{fuera} > {x_max:g} deg", font_size=17, color=C_PELIGRO)
            tag_cola.move_to([borde_der - tag_cola.width / 2 - 0.62, base_y + 0.6, 0])
            flecha = Arrow(tag_cola.get_right() + RIGHT * 0.08,
                           tag_cola.get_right() + RIGHT * 0.55, buff=0, color=C_PELIGRO,
                           stroke_width=4, max_tip_length_to_length_ratio=0.5)
            self.play(FadeIn(tag_cola), GrowArrow(flecha), run_time=1.0)
        ok = bool(c["pasa"])
        pie = cifra_pie(f"P95 {cif.de(ds, 'campana.p95')} DEG, " + ("PASA" if ok else "NO PASA"),
                        color=C_OK if ok else C_PELIGRO)
        self.play(FadeIn(pie, shift=0.2 * UP), run_time=0.9)
        self.wait(6.0)
