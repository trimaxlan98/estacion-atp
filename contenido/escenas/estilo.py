# =====================================================================
# Estacion ATP -> motor de contenido. Bloque de estilo de las escenas.
#
# Hace para estas escenas lo que el `style_block.py` de cada leccion del
# curso 30 (Sistemas ATP) hace para sus clips: fija la identidad visual de
# Co.De Aerospace, la paleta POR ROL y el guardian del FORMATO MUDO. Las
# librerias de dibujo NO se copian: se importan del repo de contenido
# montado en solo lectura (ver contenido/render.sh):
#
#     /workspace/studio/content/manim_extensions   (ATP_LIBS lo cambia)
#
# REGLA: toda cifra que aparece en pantalla sale del dataset
# (contenido/datasets/<id>.json, que a su vez sale de runs/<id>/). Aqui no
# hay un solo numero de resultado escrito a mano; `Cifras` ademas deja un
# registro de cada cifra de resumen (de que campo salio y que texto quedo)
# que `contenido/verificar.py` contrasta con `metrics.json`.
# =====================================================================
import json
import os
import sys
from pathlib import Path

LIBS = os.environ.get("ATP_LIBS", "/workspace/studio/content/manim_extensions")
if LIBS not in sys.path:
    sys.path.insert(0, LIBS)

import numpy as np                                           # noqa: E402
from manim import *                                          # noqa: E402,F401,F403

import code_brand as _code_brand                             # noqa: E402
from code_brand import (CODE_ACCENT, CODE_BG, CODE_INK,      # noqa: E402,F401
                        CODE_MUTED, FUENTE_DISPLAY, FUENTE_HUD, esquinas_hud,
                        marca_agua, registrar_fuentes, titulo_marca)

# --- Tipografia de marca ---------------------------------------------
registrar_fuentes()
Text.set_default(font=FUENTE_DISPLAY)

_TextBase = Text


class Text(_TextBase):                                       # noqa: F811
    """Sombra de Text que descarta los glifos vacios (espacios): Manim
    0.20.1 los deja anclados donde nacio el texto e inflan la caja. Es la
    misma sombra del style_block del curso 30."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.submobjects = [s for s in self.submobjects if s.has_points()]


_code_brand.Text = Text

import apuntado as _ap                                       # noqa: E402
from apuntado import mascara_elevacion, vista_polar          # noqa: E402,F401

_ap.Text = Text

import atp as _atp                                           # noqa: E402
from atp import (C_CALCULO, C_CIELO, C_DATO, C_EJE, C_OK,    # noqa: E402,F401
                 C_PELIGRO, C_SAT, OBJETIVO_DEG, histograma, traza_error)

_atp.Text = Text

config.background_color = CODE_BG

C_TITULO = CODE_INK
C_TENUE = CODE_MUTED
MARGEN_PIE = 0.62

# --- EL GUARDIAN DEL FORMATO MUDO (curso 30, seccion 2) ---------------
MAX_TITULO = 6      # palabras
MAX_CIFRA = 5       # palabras, contando el numero
MAX_TAG = 4


def _palabras(texto):
    return [p for p in str(texto).split() if any(c.isalnum() for c in p)]


def vigilar(texto, maximo, quien):
    """ABORTA el render si un rotulo se convierte en frase."""
    n = len(_palabras(texto))
    if n > maximo:
        raise ValueError(
            f"FORMATO MUDO: {quien} admite {maximo} palabras y le llegaron "
            f"{n}: {texto!r}. La frase va en la narracion, no en pantalla.")
    return texto


def con_fondo(mobjeto, buff=0.14, opacidad=0.82):
    return VGroup(BackgroundRectangle(mobjeto, color=CODE_BG,
                                      fill_opacity=opacidad, buff=buff),
                  mobjeto)


def titulo_curso(texto, font_size=34):
    vigilar(texto, MAX_TITULO, "titulo_curso")
    t = titulo_marca(texto, font_size=font_size, color=C_TITULO)
    if t.width > 7.6:
        t.scale_to_fit_width(7.6)
    t.to_edge(UP, buff=0.42)
    return con_fondo(t)


def tag_hud(texto, font_size=19, color=None):
    """Cifra tecnica en Space Mono (SOLO ASCII: sin acentos ni griegas)."""
    vigilar(texto, MAX_CIFRA, "tag_hud")
    return Text(str(texto), font=FUENTE_HUD, font_size=font_size,
                color=C_CALCULO if color is None else color)


def cifra_pie(texto, font_size=26, color=None):
    """El carril de la cifra (abajo): una medicion con su etiqueta corta."""
    vigilar(texto, MAX_CIFRA, "cifra_pie")
    t = Text(str(texto), font=FUENTE_HUD, font_size=font_size,
             color=C_CALCULO if color is None else color)
    if t.width > config.frame_width - 2.6:
        t.scale_to_fit_width(config.frame_width - 2.6)
    t.to_edge(DOWN, buff=MARGEN_PIE)
    return con_fondo(t)


def tag_junto(mobjeto, texto, direccion=DOWN, buff=0.16, font_size=18,
              color=None):
    """Rotulo de mobiliario (<= 4 palabras), en Space Mono."""
    vigilar(texto, MAX_TAG, "tag_junto")
    t = Text(str(texto), font=FUENTE_HUD, font_size=font_size,
             color=C_TENUE if color is None else color)
    t.next_to(mobjeto, direccion, buff=buff)
    return t


# --- lineas de cifras monoespaciadas, baratas de redibujar --------------
class BancoMono:
    """Banco de glifos de Space Mono (monoespaciada) para cifras EN VIVO.

    Crear un `Text` por cuadro y por cifra (Pango + SVG) es lo que vuelve
    lentos los renders con cifras que cambian. Aqui se compone el Text UNA
    vez y cada linea se arma copiando glifos en una rejilla de ancho fijo.
    """
    CHARS = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789.+-%:/<>="

    def __init__(self, font_size):
        mono = dict(font=FUENTE_HUD, font_size=font_size)
        maestro = Text(self.CHARS, **mono)
        glifos = [g for g in maestro.submobjects if g.has_points()]
        if len(glifos) != len(self.CHARS):
            raise RuntimeError("BancoMono: el Text maestro no trae un glifo por caracter")
        self.avance = (Text("0" * 20, **mono).width - Text("0", **mono).width) / 19.0
        self.x_ref = glifos[0].get_center()[0]
        self.y_ref = maestro.get_center()[1]
        self.glifos = dict(zip(self.CHARS, glifos))
        self.indice = {c: i for i, c in enumerate(self.CHARS)}

    def linea(self, texto, color, ancla):
        """VGroup de glifos con la celda 0 centrada en `ancla` (x) y la
        linea centrada en `ancla` (y). Alineada por celdas: no baila."""
        g = VGroup()
        for i, c in enumerate(str(texto).upper()):
            if c == " ":
                continue
            if c not in self.glifos:
                raise ValueError(f"BancoMono: caracter {c!r} fuera del banco")
            k = self.indice[c]
            glifo = self.glifos[c].copy().set_color(color)
            glifo.shift(RIGHT * (ancla[0] - self.x_ref + (i - k) * self.avance)
                        + UP * (ancla[1] - self.y_ref))
            g.add(glifo)
        return g

    def ancho(self, n_celdas):
        return n_celdas * self.avance


_BANCOS = {}


def banco_mono(font_size=19):
    if font_size not in _BANCOS:
        _BANCOS[font_size] = BancoMono(font_size)
    return _BANCOS[font_size]


class Escena(Scene):
    """Escena con la marca (fondo, esquinas HUD, marca de agua) y el
    registro de cifras que se vuelca a disco al terminar."""

    def setup(self):
        super().setup()
        self.camera.background_color = CODE_BG
        self.add(esquinas_hud(), marca_agua())
        self.cifras = Cifras(type(self).__name__)

    def tear_down(self):
        super().tear_down()
        self.cifras.volcar()


# --- datasets ---------------------------------------------------------
DIR_DATASETS = Path(os.environ.get(
    "ATP_DATASETS", Path(__file__).resolve().parent.parent / "datasets"))
ESQUEMA = "atp-contenido/1"


class Corrida:
    """Un dataset de contenido/datasets/<id>.json (lo escribe exportar.py)."""

    def __init__(self, nombre):
        ruta = Path(nombre)
        if not ruta.suffix:
            ruta = DIR_DATASETS / f"{nombre}.json"
        if not ruta.is_file():
            raise FileNotFoundError(
                f"no existe el dataset {ruta}: exportalo con "
                f"`python3 contenido/exportar.py <id>`")
        self.ruta = ruta
        self.d = json.loads(ruta.read_text(encoding="utf-8"))
        if self.d.get("esquema") != ESQUEMA:
            raise ValueError(f"{ruta}: esquema {self.d.get('esquema')!r}, "
                             f"se esperaba {ESQUEMA!r}")
        self.id = self.d["id"]
        self.s = {k: np.asarray(v, dtype=float if k != "link_closed" else bool)
                  for k, v in self.d.get("series", {}).items()}
        self.metricas = self.d.get("metricas", {})
        self.campana = self.d.get("campana")
        self.params = self.d.get("params", {})

    def exigir(self, *campos):
        faltan = [c for c in campos if c not in self.s]
        if faltan:
            raise ValueError(f"{self.ruta.name}: la escena necesita las series "
                             f"{faltan} y el dataset no las trae")

    @property
    def t0(self):
        return float(self.s["t"][0])

    @property
    def t1(self):
        return float(self.s["t"][-1])

    def en(self, campo, t):
        """Valor de la serie `campo` en el instante `t` (interpola)."""
        if campo == "link_closed":
            i = int(np.searchsorted(self.s["t"], t, side="right")) - 1
            return bool(self.s[campo][max(i, 0)])
        return float(np.interp(t, self.s["t"], self.s[campo]))

    def ruta_metrica(self, ruta):
        """`pointing.rms` -> metrics.json; `campana.p95` -> campaign.json."""
        v = self.d if ruta.split(".")[0] == "campana" else self.metricas
        for k in ruta.split("."):
            v = v[k]
        return v


class Cifras:
    """Registro de las cifras de RESUMEN que una escena rotula: cada una con
    el campo del dataset del que sale y el texto exacto que se dibujo. Se
    vuelca a <media>/cifras/<Escena>.json y `verificar.py` lo contrasta."""

    def __init__(self, escena):
        self.escena = escena
        self.items = []
        self.vivo = []

    def de(self, corrida, ruta, formato="{:.4f}", escala=1.0):
        """Texto de la metrica `ruta` (p. ej. 'pointing.rms') de `corrida`.
        `escala` cambia de unidad al rotular (Hz -> kHz con 1e-3)."""
        valor = corrida.ruta_metrica(ruta)
        texto = formato.format(valor * escala)
        self.items.append({"dataset": corrida.id, "campo": ruta,
                           "valor": valor, "escala": escala,
                           "formato": formato, "texto": texto})
        return texto

    def marcar_vivo(self, corrida, inicio_s, dur_s):
        """Anota cuando (tiempo de VIDEO) corre `corrida` en vivo y cuanto
        dura: permite reconstruir que instante del pase hay en cada cuadro."""
        self.vivo.append({"dataset": corrida.id, "inicio_video_s": round(inicio_s, 3),
                          "dur_s": dur_s, "t0": corrida.t0, "t1": corrida.t1})

    def volcar(self):
        destino = Path(config.media_dir) / "cifras"
        destino.mkdir(parents=True, exist_ok=True)
        (destino / f"{self.escena}.json").write_text(
            json.dumps({"escena": self.escena, "cifras": self.items,
                        "vivo": self.vivo},
                       indent=1, ensure_ascii=False), encoding="utf-8")


def con_signo(x, nd=1):
    return f"{x:+.{nd}f}"
