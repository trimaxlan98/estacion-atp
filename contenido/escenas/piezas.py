"""Piezas compartidas por las escenas: cielo con trazas, curva de error que
avanza con el tiempo del pase y tramos fuera de presupuesto.

Todo lo que se dibuja sale de las series de un `Corrida` (dataset); el unico
numero que no viene de una corrida es la geometria de pantalla.
"""
from estilo import *                                        # noqa: F401,F403
from estilo import (C_CALCULO, C_PELIGRO, C_SAT, VMobject, np, mascara_elevacion,
                    vista_polar)

MAX_PUNTOS_CURVA = 400      # tope de traza_error (atp.MUESTRAS_MAX)


def exigir_curva(ds):
    n = len(ds.s["t"])
    if n > MAX_PUNTOS_CURVA:
        raise ValueError(
            f"{ds.ruta.name}: {n} puntos pasan del tope {MAX_PUNTOS_CURVA} de "
            f"traza_error; re-exporta con --max-points {MAX_PUNTOS_CURVA}")


def polilinea(puntos, color, ancho, opacidad=1.0):
    v = VMobject()
    puntos = np.asarray(puntos, dtype=float)
    if len(puntos) < 2:
        puntos = np.vstack([puntos, puntos]) if len(puntos) else np.zeros((2, 3))
    v.set_points_as_corners(puntos)
    v.set_stroke(color=color, width=ancho, opacity=opacidad)
    return v


class Cielo:
    """Cielo polar de una corrida: mascara, traza de referencia (ambar) y
    localizadores de la antena (cian) en el instante `t`."""

    def __init__(self, ds, centro, radio, font_size=15):
        ds.exigir("t", "ref_az", "ref_el", "az", "el")
        self.ds = ds
        self.vista = vista_polar(radio=radio, font_size=font_size)
        self.vista.move_to(centro)
        # la mascara es el primer valor de elevacion de la referencia: lo
        # que la corrida declaro como inicio del pase
        self.el_mascara = float(np.min(ds.s["ref_el"]))
        self.mascara = mascara_elevacion(self.vista, el_min=self.el_mascara)
        # Lo que hay que seguir es la VERDAD (medida + error: el monitor
        # mide contra ella). En un lazo normal coincide con la referencia;
        # en autotrack `ref_*` es el mando corregido, no el satelite.
        if "e_az" in ds.s and "e_el" in ds.s:
            az_v, el_v = ds.s["az"] + ds.s["e_az"], ds.s["el"] + ds.s["e_el"]
        else:
            az_v, el_v = ds.s["ref_az"], ds.s["ref_el"]
        self.pts_ref = np.array([self.vista.punto(a, e)
                                 for a, e in zip(az_v, np.clip(el_v, 0.0, 90.0))])
        self.pts_ant = np.array([self.vista.punto(a, e) for a, e in
                                 zip(ds.s["az"], ds.s["el"])])
        self.ruta = polilinea(self.pts_ref, C_SAT, 2.0, 0.28)

    def _hasta(self, pts, t):
        ds = self.ds
        i = int(np.searchsorted(ds.s["t"], t, side="right"))
        i = int(np.clip(i, 1, len(pts)))
        cola = pts[:i]
        if i < len(pts):
            f = (t - ds.s["t"][i - 1]) / max(ds.s["t"][i] - ds.s["t"][i - 1], 1e-9)
            cola = np.vstack([cola, pts[i - 1] + f * (pts[i] - pts[i - 1])])
        return cola

    def estela_ref(self, t):
        return polilinea(self._hasta(self.pts_ref, t), C_SAT, 4.2)

    def estela_ant(self, t):
        return polilinea(self._hasta(self.pts_ant, t), C_CALCULO, 1.4)

    def punto_ref(self, t):
        return self._hasta(self.pts_ref, t)[-1]

    def punto_ant(self, t):
        return self._hasta(self.pts_ant, t)[-1]


class CurvaError:
    """`traza_error` con la banda siempre dibujada y una curva que avanza
    con `t`; los tramos que pasan del objetivo se pintan de rojo."""

    def __init__(self, ds, ancho, alto, centro, objetivo, y_max=None,
                 etiqueta_x="t", etiqueta_y="error"):
        exigir_curva(ds)
        ds.exigir("t", "e_pt")
        self.ds, self.objetivo = ds, objetivo
        self.traza = traza_error(ds.s["t"], ds.s["e_pt"], ancho=ancho,
                                 alto=alto, umbral=objetivo, y_max=y_max,
                                 muestras=MAX_PUNTOS_CURVA,
                                 etiqueta_x=etiqueta_x, etiqueta_y=etiqueta_y)
        self.traza.move_to(centro)
        self.anclas = np.vstack([self.traza.curva.get_start_anchors(),
                                 self.traza.curva.get_end_anchors()[-1:]])
        if len(self.anclas) != len(ds.s["t"]):
            raise ValueError("traza_error remuestreo la curva: indices desalineados")
        self.y_max = self.traza.y_max

    def idx(self, t):
        ds = self.ds
        return float(np.interp(t, ds.s["t"], np.arange(len(ds.s["t"]))))

    def curva(self, t):
        """Curva ambar hasta `t`."""
        f = self.idx(t) / (len(self.anclas) - 1)
        c = self.traza.curva.copy()
        c.pointwise_become_partial(self.traza.curva, 0.0, float(np.clip(f, 1e-3, 1.0)))
        return c

    def fuera(self, t):
        """Tramos rojos (error > objetivo) hasta `t`."""
        e = self.ds.s["e_pt"]
        hasta = int(self.idx(t))
        g = VGroup()
        i = 0
        while i <= hasta:
            if e[i] > self.objetivo:
                j = i
                while j + 1 <= hasta and e[j + 1] > self.objetivo:
                    j += 1
                a, b = max(i - 1, 0), min(j + 1, hasta)
                g.add(polilinea(self.anclas[a:b + 1], C_PELIGRO, 4.2))
                i = j + 1
            else:
                i += 1
        return g

    def cabeza(self, t):
        i = int(np.floor(self.idx(t)))
        i2 = min(i + 1, len(self.anclas) - 1)
        f = self.idx(t) - i
        return self.anclas[i] + f * (self.anclas[i2] - self.anclas[i])


def fila(etiqueta, valor, unidad, ancho_etq=12, ancho_val=8):
    """Linea de panel de ancho fijo (Space Mono es monoespaciada)."""
    return f"{etiqueta:<{ancho_etq}}{valor:>{ancho_val}} {unidad}"
