#!/usr/bin/env python3
"""Graficas con la paleta Co.De Aerospace de cualquier corrida de la Estacion (matplotlib).

  graficas.py <run_dir> <salida> [--titulo T] [--commit C] [--sin-logo]

Reconoce (por los archivos de la corrida, ver `tipo_corrida`):
  lazo  telemetry.jsonl + metrics.json   error de apuntamiento con la banda del objetivo, cielo (az/el),
                                         Doppler y rango, margen de enlace
  adcs  telemetry.jsonl + metrics.json (kind adcs)   error de actitud por modo, velocidad angular, ruedas,
                                         conocimiento del estimador
  red   escena.json + red.jsonl + metricas.json      linea de tiempo de las antenas con sus handovers,
                                         satelites utiles por estacion, error de apuntamiento, KPI
  otra  telemetry.jsonl con otros campos             hasta 6 columnas numericas

Salida: una PNG por grafica (1600x900) y `lamina.png` (1920x1080, resumen con las cifras de metricas).
Corre en el contenedor ros2-jazzy-dev (matplotlib 3.6) o en un host con matplotlib; fuentes de la marca si estan
(FUENTES, MARCA: carpetas; si no, DejaVu). Toda cifra rotulada sale de metrics.json / metricas.json.
"""
import argparse
import json
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt                    # noqa: E402
import numpy as np                                 # noqa: E402
from matplotlib import font_manager as fm          # noqa: E402
from matplotlib.patches import FancyBboxPatch      # noqa: E402

# paleta Co.De Aerospace (marca/paleta.json); `panel` y `rejilla` son tonos derivados del fondo
FONDO, MARINO, PLATA, TENUE = '#080F15', '#0A0E27', '#F2F3F4', '#8E99A6'
CIAN, AMBAR = '#00D9FF', '#F59E0B'
PANEL, REJILLA = '#0E1B26', '#1E2E3B'
CIAN_SUAVE = '#0A6E85'      # cian a ~50 % sobre el fondo, para series secundarias
FUENTES = os.environ.get('FUENTES', os.path.expanduser('~/.local/share/fonts/codeaerospace'))
MARCA = os.environ.get('MARCA', os.path.expanduser('~/Documentos/code-projects/codeaerospace_contenido/marca/derivados'))
F_TIT, F_CUERPO, F_NUM = 'DejaVu Sans', 'DejaVu Sans', 'DejaVu Sans Mono'


def cargar_fuentes():
    global F_TIT, F_CUERPO, F_NUM
    for nombre, fam, attr in (('Rajdhani-SemiBold.ttf', 'Rajdhani', 'F_TIT'), ('Rajdhani-Medium.ttf', 'Rajdhani', None),
                              ('Inter-400.ttf', 'Inter', 'F_CUERPO'), ('Inter-600.ttf', 'Inter', None),
                              ('JetBrainsMono-400.ttf', 'JetBrains Mono', 'F_NUM'),
                              ('JetBrainsMono-700.ttf', 'JetBrains Mono', None)):
        ruta = os.path.join(FUENTES, nombre)
        if os.path.isfile(ruta):
            try:
                fm.fontManager.addfont(ruta)
                if attr:
                    globals()[attr] = fam
            except Exception:
                pass


def estilo():
    cargar_fuentes()
    plt.rcParams.update({
        'figure.facecolor': FONDO, 'savefig.facecolor': FONDO, 'axes.facecolor': PANEL,
        'axes.edgecolor': REJILLA, 'axes.labelcolor': TENUE, 'axes.titlecolor': PLATA,
        'xtick.color': TENUE, 'ytick.color': TENUE, 'text.color': PLATA, 'grid.color': REJILLA,
        'grid.linewidth': 0.7, 'font.family': F_CUERPO, 'font.size': 11, 'axes.titlesize': 14,
        'axes.titleweight': 'normal', 'axes.titlelocation': 'left', 'axes.titlepad': 10,
        'axes.spines.top': False, 'axes.spines.right': False, 'legend.frameon': False,
        'legend.labelcolor': TENUE, 'axes.axisbelow': True, 'lines.solid_capstyle': 'round', 'figure.dpi': 100,
    })


# ---------------------------------------------------------------- lectura
def leer_json(ruta, defecto=None):
    try:
        with open(ruta, encoding='utf-8') as f:
            return json.load(f)
    except Exception:
        return defecto


def leer_jsonl(ruta):
    """Lista de dicts; salta lineas rotas (la ultima puede estar cortada por el flush)."""
    filas = []
    with open(ruta, encoding='utf-8') as f:
        for linea in f:
            try:
                filas.append(json.loads(linea))
            except ValueError:
                pass
    return filas


NOMBRE_CORRIDA = ''


def nombre(run):
    return NOMBRE_CORRIDA or os.path.basename(os.path.normpath(run))


def tipo_corrida(run):
    j = lambda n: os.path.isfile(os.path.join(run, n))   # noqa: E731
    if j('escena.json') and j('red.jsonl') and j('metricas.json'):
        return 'red'
    if j('telemetry.jsonl'):
        m = leer_json(os.path.join(run, 'metrics.json'), {}) or {}
        if m.get('kind') == 'adcs':
            return 'adcs'
        filas1 = leer_jsonl(os.path.join(run, 'telemetry.jsonl'))[:50]
        if filas1 and all(k in filas1[-1] for k in ('t', 'el', 'az', 'e_pt')):
            return 'lazo'
        return 'otra'
    return None


def col(filas, k, defecto=np.nan):
    return np.array([f.get(k, defecto) if f.get(k) is not None else defecto for f in filas], dtype=float)


def bloques_max(t, y, n=1500):
    """Reduce a ~n puntos conservando el maximo de cada bloque (no se pierde ningun pico)."""
    if len(t) <= n:
        return t, y
    k = int(math.ceil(len(t) / n))
    m = len(t) // k * k
    tt = t[:m].reshape(-1, k).mean(axis=1)
    yy = np.nanmax(y[:m].reshape(-1, k), axis=1)
    return tt, yy


def paso(t, y, n=2500):
    if len(t) <= n:
        return t, y
    i = np.linspace(0, len(t) - 1, n).astype(int)
    return t[i], y[i]


# ---------------------------------------------------------------- piezas comunes
def logo_en(fig, x=0.985, y=0.975, alto=0.055):
    ruta = os.path.join(MARCA, 'logo-cian.png')
    if not os.path.isfile(ruta):
        return
    img = plt.imread(ruta)
    h_px, w_px = img.shape[:2]
    W, H = fig.get_size_inches() * fig.dpi
    ancho = alto * H * w_px / h_px / W
    ax = fig.add_axes([x - ancho, y - alto, ancho, alto])
    ax.imshow(img)
    ax.axis('off')


def cabecera(fig, titulo, sub, logo=True):
    fig.text(0.025, 0.955, titulo, fontfamily=F_TIT, fontsize=30, fontweight='semibold', color=PLATA, va='center')
    fig.text(0.026, 0.915, sub, fontsize=12, color=TENUE, va='center')
    if logo:
        logo_en(fig)


def pie(fig, texto):
    fig.text(0.025, 0.018, texto, fontsize=9, color=TENUE, va='center', fontfamily=F_NUM)


def tarjetas(fig, items, y0, alto, x0=0.025, x1=0.975, gap=0.01):
    """Fila de cifras clave. items: [(rotulo, texto, color)]. Coordenadas de la figura."""
    n = len(items)
    ancho = (x1 - x0 - gap * (n - 1)) / n
    for i, (rot, val, color) in enumerate(items):
        x = x0 + i * (ancho + gap)
        fig.add_artist(FancyBboxPatch((x, y0), ancho, alto, boxstyle='round,pad=0,rounding_size=0.006',
                                      transform=fig.transFigure, fc=PANEL, ec=REJILLA, lw=1))
        fig.text(x + 0.012, y0 + alto * 0.70, rot.upper(), fontsize=9.5, color=TENUE, va='center')
        fig.text(x + 0.012, y0 + alto * 0.30, val, fontsize=22, color=color, va='center',
                 fontfamily=F_NUM, fontweight='bold')


def rejilla(ax):
    ax.grid(True, axis='y')
    ax.grid(True, axis='x', alpha=0.4)
    ax.tick_params(length=0)


def guardar(fig, ruta):
    fig.savefig(ruta, dpi=fig.dpi)
    plt.close(fig)
    return ruta


def fig_individual(titulo, sub, pie_txt, dibujar, polar=False, logo=True):
    fig = plt.figure(figsize=(16, 9))
    cabecera(fig, titulo, sub, logo)
    ax = fig.add_axes([0.07, 0.10, 0.90, 0.74], projection='polar' if polar else None)
    dibujar(ax)
    pie(fig, pie_txt)
    return fig


# ---------------------------------------------------------------- lazo
def g_error(ax, t, e, obj, mask_txt=None):
    tt, ee = bloques_max(t, e)
    ax.axhspan(0, obj, color=CIAN, alpha=0.10, lw=0)
    ax.axhline(obj, color=CIAN, lw=1.2, ls=(0, (4, 3)))
    ax.plot(tt, ee, color=CIAN, lw=1.4)
    ax.fill_between(tt, obj, np.maximum(ee, obj), where=ee > obj, color=AMBAR, alpha=0.9, lw=0)
    ax.set_xlabel('tiempo del pase (s)')
    ax.set_ylabel('error de apuntamiento (°)')
    ax.set_title('Error de apuntamiento')
    ax.text(0.99, obj, f' objetivo {obj:g}°', transform=ax.get_yaxis_transform(), ha='right', va='bottom',
            color=CIAN, fontsize=10)
    ax.set_ylim(0, max(float(np.nanmax(ee)) * 1.15, obj * 1.35))
    ax.set_xlim(t[0], t[-1])
    rejilla(ax)


def g_cielo(ax, az, el, ref_az, ref_el, mask):
    ax.set_theta_zero_location('N')
    ax.set_theta_direction(-1)
    rmax = 90.0 if mask is None or mask <= 30 else min(90.0, 90 - mask + 6)   # con mascara alta, solo el casquete
    paso_el = 30 if rmax > 60 else 10 if rmax > 25 else 5
    rs = [r for r in np.arange(0, rmax, paso_el)]
    ax.set_ylim(0, rmax)
    ax.set_yticks(rs)
    ax.set_yticklabels([f'{90 - r:g}°' for r in rs], color=TENUE, fontsize=9)
    ax.set_xticks(np.radians([0, 90, 180, 270]))
    ax.set_xticklabels(['N', 'E', 'S', 'O'], color=TENUE)
    ax.set_facecolor(PANEL)
    ax.grid(True, color=REJILLA)
    ax.spines['polar'].set_color(REJILLA)
    if mask is not None:
        th = np.linspace(0, 2 * np.pi, 200)
        ax.fill_between(th, 90 - mask, 90, color=FONDO, alpha=0.9, lw=0)
        ax.plot(th, np.full_like(th, 90 - mask), color=TENUE, lw=0.8, ls=(0, (3, 3)))
        ax.text(np.radians(135), 90 - mask, f' máscara {mask:.1f}°', color=TENUE, fontsize=8, ha='left', va='bottom')
    ax.plot(np.radians(ref_az), 90 - ref_el, color=AMBAR, lw=2.4, alpha=0.9, label='referencia')
    ax.plot(np.radians(az), 90 - el, color=CIAN, lw=1.3, label='antena')
    ax.text(0.0, 1.0, 'Cielo (az/el)', transform=ax.transAxes, ha='left', va='bottom', fontsize=14, color=PLATA)
    ax.legend(loc='lower left', bbox_to_anchor=(-0.32, -0.02), fontsize=9)


def g_doppler(ax, t, dop_khz, rng):
    ax.plot(t, dop_khz, color=CIAN, lw=1.6, label='Doppler (kHz)')
    ax.set_ylabel('Doppler (kHz)', color=CIAN)
    ax.set_xlabel('tiempo del pase (s)')
    ax.axhline(0, color=REJILLA, lw=1)
    ax.set_title('Doppler y rango')
    ax.set_xlim(t[0], t[-1])
    rejilla(ax)
    if np.isfinite(rng).any():
        ax2 = ax.twinx()
        ax2.plot(t, rng, color=AMBAR, lw=1.4, label='rango (km)')
        ax2.set_ylabel('rango (km)', color=AMBAR)
        ax2.spines['right'].set_visible(True)
        ax2.spines['right'].set_color(REJILLA)
        ax2.tick_params(length=0, colors=TENUE)


def g_margen(ax, t, mar, banda):
    ax.plot(t, mar, color=CIAN, lw=1.6)
    ax.axhline(0, color=AMBAR, lw=1.2, ls=(0, (4, 3)))
    ax.text(0.01, 0, ' enlace abierto bajo 0 dB', transform=ax.get_yaxis_transform(), color=AMBAR, fontsize=9,
            va='bottom')
    ax.fill_between(t, 0, mar, where=mar >= 0, color=CIAN, alpha=0.10, lw=0)
    ax.set_ylabel('margen (dB)')
    ax.set_xlabel('tiempo del pase (s)')
    ax.set_title('Margen de enlace' + (f' · banda {banda}' if banda else ''))
    ax.set_xlim(t[0], t[-1])
    rejilla(ax)


def graficas_lazo(run, salida, titulo, pie_txt, logo):
    filas = [f for f in leer_jsonl(os.path.join(run, 'telemetry.jsonl')) if 'e_pt' in f and 'el' in f]
    if len(filas) < 5:
        raise SystemExit(f'ERROR: telemetry.jsonl tiene solo {len(filas)} muestras completas; no hay curvas que dibujar')
    m = leer_json(os.path.join(run, 'metrics.json'), {}) or {}
    pt, lk = m.get('pointing', {}), m.get('link', {})
    t = col(filas, 't'); e = col(filas, 'e_pt'); az = col(filas, 'az'); el = col(filas, 'el')
    raz = col(filas, 'ref_az'); rel = col(filas, 'ref_el'); dop = col(filas, 'doppler_hz') / 1e3
    rng = col(filas, 'range_km'); mar = col(filas, 'margin_db')
    obj = float(pt.get('objective_deg', 0.1))
    mask = float(np.nanmin(rel)) if np.isfinite(rel).any() else None
    sub = (f'{nombre(run)} · {len(filas)} muestras · {t[-1] - t[0]:.0f} s de pase · '
           f'elevación máx {np.nanmax(el):.1f}°')
    out = []
    out.append(guardar(fig_individual(titulo or 'Error de apuntamiento', sub, pie_txt,
                                      lambda ax: g_error(ax, t, e, obj), logo=logo), f'{salida}/error.png'))
    out.append(guardar(fig_individual('Cielo del pase', sub, pie_txt,
                                      lambda ax: g_cielo(ax, az, el, raz, rel, mask), polar=True, logo=logo),
                       f'{salida}/cielo.png'))
    out.append(guardar(fig_individual('Doppler y rango', sub, pie_txt, lambda ax: g_doppler(ax, t, dop, rng),
                                      logo=logo), f'{salida}/doppler.png'))
    if np.isfinite(mar).any():
        out.append(guardar(fig_individual('Margen de enlace', sub, pie_txt,
                                          lambda ax: g_margen(ax, t, mar, lk.get('band')), logo=logo),
                           f'{salida}/margen.png'))
    # lamina
    fig = plt.figure(figsize=(19.2, 10.8))
    cabecera(fig, titulo or 'Pase: apuntamiento y enlace', sub, logo)
    cum = pt.get('within_objective_pct')
    veredicto = m.get('verdict')
    tarjetas(fig, [
        ('error rms', f'{pt["rms"]:.4f}°' if 'rms' in pt else '—', CIAN),
        ('error p95', f'{pt["p95"]:.4f}°' if 'p95' in pt else '—', CIAN),
        ('error máx', f'{pt["max"]:.4f}°' if 'max' in pt else '—', AMBAR),
        (f'dentro de {obj:g}°', f'{cum:.1f} %' if cum is not None else '—', CIAN),
        ('margen mín', f'{lk["margin_min_db"]:.2f} dB' if 'margin_min_db' in lk else '—', CIAN),
        ('Doppler máx', f'{lk["doppler_max_hz"] / 1e3:.1f} kHz' if 'doppler_max_hz' in lk else '—', CIAN),
        ('veredicto', veredicto or '—', CIAN if veredicto == 'CUMPLE' else AMBAR),
    ], 0.755, 0.105)
    gs = fig.add_gridspec(2, 2, left=0.055, right=0.975, top=0.70, bottom=0.075, wspace=0.16, hspace=0.45)
    g_error(fig.add_subplot(gs[0, 0]), t, e, obj)
    g_cielo(fig.add_subplot(gs[0, 1], projection='polar'), az, el, raz, rel, mask)
    g_doppler(fig.add_subplot(gs[1, 0]), t, dop, rng)
    if np.isfinite(mar).any():
        g_margen(fig.add_subplot(gs[1, 1]), t, mar, lk.get('band'))
    pie(fig, pie_txt)
    out.append(guardar(fig, f'{salida}/lamina.png'))
    return out, {'rms': pt.get('rms'), 'p95': pt.get('p95'), 'veredicto': veredicto}


# ---------------------------------------------------------------- adcs
def graficas_adcs(run, salida, titulo, pie_txt, logo):
    filas = leer_jsonl(os.path.join(run, 'telemetry.jsonl'))
    if len(filas) < 5:
        raise SystemExit(f'ERROR: telemetry.jsonl (ADCS) tiene solo {len(filas)} filas')
    m = leer_json(os.path.join(run, 'metrics.json'), {}) or {}
    t = col(filas, 't'); err = col(filas, 'err_deg'); w = col(filas, 'w_deg_s'); know = col(filas, 'know_deg')
    sig = col(filas, 'sigma_deg')
    h = np.array([np.linalg.norm(f['h_nms']) if f.get('h_nms') else np.nan for f in filas])
    cap = (m.get('wheels') or {}).get('capacity_nms')
    modos = m.get('modes', [])
    nombres = {0: 'detumble', 1: 'nadir', 2: 'seguimiento'}

    def sombrear(ax):
        for md in modos:
            if md.get('mode') == 2:
                ax.axvspan(md['t0'], md['t1'], color=CIAN, alpha=0.08, lw=0)

    def d_err(ax):
        sombrear(ax)
        ax.plot(t, err, color=CIAN, lw=1.4)
        ax.set_title('Error de actitud de la antena (°)')
        ax.set_xlabel('tiempo (s)'); ax.set_ylabel('°'); rejilla(ax)
        thr = (m.get('track') or {}).get('threshold_deg')
        if thr:
            ax.axhline(thr, color=AMBAR, lw=1.2, ls=(0, (4, 3)))
            ax.text(0.99, thr, f' umbral {thr:.2f}°', transform=ax.get_yaxis_transform(), ha='right',
                    va='bottom', color=AMBAR, fontsize=10)

    def d_w(ax):
        sombrear(ax)
        ax.semilogy(t, np.maximum(w, 1e-4), color=CIAN, lw=1.4)
        ax.set_title('Velocidad angular (°/s)'); ax.set_xlabel('tiempo (s)'); rejilla(ax)

    def d_h(ax):
        sombrear(ax)
        ax.plot(t, h * 1e3, color=CIAN, lw=1.4)
        if cap:
            ax.axhline(cap * 1e3, color=AMBAR, lw=1.2, ls=(0, (4, 3)))
            ax.text(0.01, cap * 1e3, ' capacidad de las ruedas', transform=ax.get_yaxis_transform(),
                    color=AMBAR, fontsize=9, va='bottom')
        ax.set_title('Momento de las ruedas (mN·m·s)'); ax.set_xlabel('tiempo (s)'); rejilla(ax)

    def d_k(ax):
        sombrear(ax)
        if np.isfinite(know).any():
            ax.plot(t, np.abs(know), color=CIAN, lw=1.3, label='|error de conocimiento|')
        if np.isfinite(sig).any():
            ax.plot(t, sig, color=AMBAR, lw=1.3, label='1-σ del filtro')
        ax.set_title('Conocimiento de actitud (°)'); ax.set_xlabel('tiempo (s)'); rejilla(ax)
        ax.legend(loc='upper right', fontsize=9)

    sub = f'{nombre(run)} · {len(filas)} pasos de {m.get("paso_s", 1):g} s · {t[-1]:.0f} s'
    out = []
    for nombre, f, tit in (('error', d_err, 'Actitud: error'), ('velocidad', d_w, 'Actitud: velocidad angular'),
                           ('ruedas', d_h, 'Actitud: ruedas'), ('conocimiento', d_k, 'Actitud: conocimiento')):
        out.append(guardar(fig_individual(tit, sub, pie_txt, f, logo=logo), f'{salida}/{nombre}.png'))
    fig = plt.figure(figsize=(19.2, 10.8))
    cabecera(fig, titulo or 'Control de actitud (ADCS)', sub, logo)
    tr, kn, wh = m.get('track') or {}, m.get('knowledge') or {}, m.get('wheels') or {}
    v = m.get('verdict')
    tarjetas(fig, [
        ('seguimiento rms', f'{tr["rms"]:.3f}°' if 'rms' in tr else '—', CIAN),
        ('seguimiento p95', f'{tr["p95"]:.3f}°' if 'p95' in tr else '—', CIAN),
        ('seguimiento máx', f'{tr["max"]:.3f}°' if 'max' in tr else '—', AMBAR),
        ('conocimiento rms', f'{kn["rms_deg"]:.3f}°' if 'rms_deg' in kn else '—', CIAN),
        ('ruedas (frac. de cap.)', f'{wh["frac"] * 100:.0f} %' if 'frac' in wh else '—', CIAN),
        ('veredicto', v or '—', CIAN if v == 'CUMPLE' else AMBAR),
    ], 0.755, 0.105)
    gs = fig.add_gridspec(2, 2, left=0.055, right=0.975, top=0.70, bottom=0.075, wspace=0.16, hspace=0.45)
    for (i, j), f in zip(((0, 0), (0, 1), (1, 0), (1, 1)), (d_err, d_w, d_h, d_k)):
        f(fig.add_subplot(gs[i, j]))
    pie(fig, pie_txt)
    out.append(guardar(fig, f'{salida}/lamina.png'))
    return out, {'verdict': v}


# ---------------------------------------------------------------- generica
def graficas_otra(run, salida, titulo, pie_txt, logo):
    filas = leer_jsonl(os.path.join(run, 'telemetry.jsonl'))
    claves = []
    for k in (filas[-1].keys() if filas else []):
        if k != 't' and isinstance(filas[-1][k], (int, float)) and not isinstance(filas[-1][k], bool):
            claves.append(k)
    claves = claves[:6]
    if len(filas) < 3 or not claves:
        raise SystemExit('ERROR: telemetry.jsonl no tiene columnas numericas que dibujar')
    t = col(filas, 't')
    out = []
    fig = plt.figure(figsize=(19.2, 10.8))
    cabecera(fig, titulo or 'Telemetría de la corrida', f'{nombre(run)} · {len(filas)} filas', logo)
    gs = fig.add_gridspec(2, 3, left=0.055, right=0.975, top=0.84, bottom=0.075, wspace=0.22, hspace=0.42)
    for i, k in enumerate(claves):
        ax = fig.add_subplot(gs[i // 3, i % 3])
        tt, yy = paso(t, col(filas, k))
        ax.plot(tt, yy, color=CIAN, lw=1.3)
        ax.set_title(k); ax.set_xlabel('t (s)'); rejilla(ax)
    pie(fig, pie_txt)
    out.append(guardar(fig, f'{salida}/lamina.png'))
    return out, {'columnas': claves}


# ---------------------------------------------------------------- red
def sep_ang(az1, el1, az2, el2):
    a1, e1, a2, e2 = map(np.radians, (az1, el1, az2, el2))
    c = np.sin(e1) * np.sin(e2) + np.cos(e1) * np.cos(e2) * np.cos(a1 - a2)
    return np.degrees(np.arccos(np.clip(c, -1, 1)))


def graficas_red(run, salida, titulo, pie_txt, logo):
    esc = leer_json(os.path.join(run, 'escena.json'), {})
    met = leer_json(os.path.join(run, 'metricas.json'), {})
    red = leer_jsonl(os.path.join(run, 'red.jsonl'))
    ev = leer_jsonl(os.path.join(run, 'eventos.jsonl')) if os.path.isfile(os.path.join(run, 'eventos.jsonl')) else []
    if len(red) < 3:
        raise SystemExit(f'ERROR: red.jsonl tiene solo {len(red)} ticks; no hay curvas que dibujar')
    t = np.array([r['t'] for r in red], dtype=float)
    ests = [e['id'] for e in esc['estaciones']]
    mask = {e: (met.get('por_estacion', {}).get(e, {}).get('mascara_deg') or 0.0) for e in ests}
    antenas = sorted(red[0]['antenas'].keys())
    # satelites utiles por estacion
    utiles = {e: np.array([sum(1 for s in r['geo'].get(e, []) if s[2] >= mask[e]) for r in red]) for e in ests}
    # error angular por antena (solo en seguimiento)
    err = {}
    for a in antenas:
        v = []
        for r in red:
            d = r['antenas'].get(a, {})
            if d.get('estado') == 'seguimiento' and d.get('ref') and d.get('med'):
                v.append(float(sep_ang(d['ref'][0], d['ref'][1], d['med'][0], d['med'][1])))
            else:
                v.append(np.nan)
        err[a] = np.array(v)
    tol = (met.get('parametros') or {}).get('tol_deg', 0.25)
    res = met.get('resumen', {})
    pe = met.get('por_estacion', {})

    def d_lineas(ax):
        n = len(antenas)
        for i, a in enumerate(antenas):
            y = n - 1 - i
            seg = []
            ant = [r['antenas'].get(a, {}) for r in red]
            ini = 0
            for k in range(1, len(ant) + 1):
                if k == len(ant) or (ant[k].get('sat'), ant[k].get('estado')) != (ant[ini].get('sat'), ant[ini].get('estado')):
                    seg.append((ini, k, ant[ini].get('sat'), ant[ini].get('estado')))
                    ini = k
            for i0, i1, sat, est in seg:
                x0, x1 = t[i0], t[min(i1, len(t) - 1)] if i1 < len(t) else t[-1] + (t[1] - t[0])
                if sat is None or est == 'libre':
                    continue
                col_ = CIAN if est == 'seguimiento' else AMBAR
                ax.broken_barh([(x0, x1 - x0)], (y - 0.32, 0.64), facecolors=col_, alpha=0.85 if est == 'seguimiento' else 0.9)
                if x1 - x0 > (t[-1] - t[0]) * 0.025:
                    ax.text((x0 + x1) / 2, y, sat, ha='center', va='center', fontsize=9, color=FONDO,
                            fontfamily=F_NUM, fontweight='bold')
        for e in ev:
            if e.get('tipo') in ('make-before-break', 'break-before-make'):
                ax.axvline(e['t'], color=AMBAR, lw=0.9, alpha=0.7, ls=(0, (2, 2)))
        ax.set_yticks(range(n)); ax.set_yticklabels(list(reversed(antenas)), fontfamily=F_NUM, fontsize=10)
        ax.set_ylim(-0.7, n - 0.3); ax.set_xlim(t[0], t[-1])
        ax.set_xlabel('tiempo (s)')
        ax.set_title('Quién sirve a cada antena  ·  cian: en seguimiento, ámbar: reapuntando, línea: handover')
        ax.grid(True, axis='x', alpha=0.4); ax.tick_params(length=0)

    def d_utiles(ax):
        for i, e in enumerate(ests):
            ax.step(t, utiles[e], where='post', color=[CIAN, AMBAR, PLATA, TENUE][i % 4], lw=1.6,
                    label=f'{e} (máscara {mask[e]:g}°)')
        ax.set_title('Satélites útiles por estación'); ax.set_xlabel('tiempo (s)'); ax.set_ylabel('satélites')
        ax.set_xlim(t[0], t[-1]); ax.set_ylim(0, max(int(max(u.max() for u in utiles.values())) + 1.5, 2)); rejilla(ax)
        ax.yaxis.get_major_locator().set_params(integer=True)
        ax.legend(loc='upper right', fontsize=9)

    def d_err(ax):
        for i, a in enumerate(antenas):
            tt, yy = paso(t, err[a], 1500)
            ax.plot(tt, yy, lw=1.1, color=[CIAN, AMBAR, PLATA, TENUE, CIAN_SUAVE][i % 5], alpha=0.9, label=a)
        ax.axhline(tol, color=AMBAR, lw=1.1, ls=(0, (4, 3)))
        ax.text(0.01, tol, f' tolerancia {tol:g}°', transform=ax.get_yaxis_transform(), ha='left', va='bottom',
                color=AMBAR, fontsize=9)
        ax.set_ylim(0, tol * 1.35)
        ax.set_title('Error de apuntamiento (en seguimiento)'); ax.set_xlabel('tiempo (s)'); ax.set_ylabel('°')
        ax.set_xlim(t[0], t[-1]); rejilla(ax)
        if len(antenas) <= 6:
            ax.legend(loc='upper right', fontsize=8, ncol=2, bbox_to_anchor=(1.0, 0.93))

    def d_barras(ax):
        x = np.arange(len(ests)); w = 0.38
        serv = [pe.get(e, {}).get('servicio_pct', 0) for e in ests]
        cob = [pe.get(e, {}).get('cobertura_pct', 0) for e in ests]
        b1 = ax.bar(x - w / 2, cob, w, color=CIAN_SUAVE, label='cobertura geométrica')
        b2 = ax.bar(x + w / 2, serv, w, color=CIAN, label='servicio')
        for b in list(b1) + list(b2):
            ax.text(b.get_x() + b.get_width() / 2, b.get_height() + 1, f'{b.get_height():.1f}', ha='center',
                    fontsize=9, color=PLATA, fontfamily=F_NUM)
        ax.set_xticks(x); ax.set_xticklabels(ests); ax.set_ylim(0, 112); ax.set_ylabel('% del tiempo')
        ax.set_title('Servicio y cobertura'); ax.legend(loc='upper right', fontsize=9, ncol=2)
        ax.grid(True, axis='y'); ax.tick_params(length=0)

    def d_huecos(ax):
        h = (met.get('hueco_handover_s') or {}).get('histograma') or []
        if h and any(b['n'] for b in h):
            xs = [(b['desde_s'] + b['hasta_s']) / 2 for b in h]
            ax.bar(xs, [b['n'] for b in h], width=[(b['hasta_s'] - b['desde_s']) * 0.9 for b in h], color=CIAN)
            ax.set_xlabel('hueco de servicio por handover (s)'); ax.set_ylabel('handovers')
        else:
            ax.text(0.5, 0.5, 'sin handovers en la ventana', ha='center', va='center', color=TENUE,
                    transform=ax.transAxes)
        ax.set_title('Hueco por handover'); ax.grid(True, axis='y'); ax.tick_params(length=0)

    p = met.get('parametros', {})
    sub = (f'{nombre(run)} · {res.get("satelites", "?")} satélites · {", ".join(ests)} · '
           f'{res.get("antenas_por_estacion", "?")} antenas por estación · {res.get("politica", "?")}/{res.get("modo", "?")} · '
           f'{res.get("duracion_h", 0) * 3600:.0f} s')
    hu = met.get('hueco_handover_s', {})
    out = []
    out.append(guardar(fig_individual('Servicio y handovers', sub, pie_txt, d_lineas, logo=logo), f'{salida}/servicio.png'))
    out.append(guardar(fig_individual('Satélites útiles', sub, pie_txt, d_utiles, logo=logo), f'{salida}/utiles.png'))
    out.append(guardar(fig_individual('Error de apuntamiento', sub, pie_txt, d_err, logo=logo), f'{salida}/error.png'))
    out.append(guardar(fig_individual('Servicio y cobertura', sub, pie_txt, d_barras, logo=logo), f'{salida}/kpi.png'))
    fig = plt.figure(figsize=(19.2, 10.8))
    cabecera(fig, titulo or 'Red: constelación, handover y servicio', sub, logo)
    pp = res.get('tasa_ping_pong')
    tarjetas(fig, [
        ('handovers/h/estación', f'{res["handovers_por_hora_por_estacion"]:.1f}' if 'handovers_por_hora_por_estacion' in res else '—', CIAN),
        ('servicio medio', f'{res["servicio_medio_pct"]:.1f} %' if 'servicio_medio_pct' in res else '—', CIAN),
        ('cobertura media', f'{res["cobertura_media_pct"]:.1f} %' if 'cobertura_media_pct' in res else '—', CIAN),
        ('hueco medio', f'{hu["media"]:.1f} s' if hu.get('media') is not None else '—', AMBAR),
        ('ping-pong', f'{pp:.3f}' if pp is not None else '0', CIAN),
        ('error rms', f'{res["err_rms_deg_global"]:.4f}°' if 'err_rms_deg_global' in res else '—', CIAN),
    ], 0.775, 0.095)
    n_ant = len(antenas)
    alto_t = min(0.30, 0.05 + 0.03 * n_ant)
    gs = fig.add_gridspec(2, 3, left=0.075, right=0.975, top=0.715, bottom=0.075, wspace=0.22, hspace=0.50,
                          height_ratios=[alto_t / 0.30, 1.0])
    d_lineas(fig.add_subplot(gs[0, :]))
    d_utiles(fig.add_subplot(gs[1, 0]))
    d_err(fig.add_subplot(gs[1, 1]))
    d_barras(fig.add_subplot(gs[1, 2]))
    pie(fig, pie_txt)
    out.append(guardar(fig, f'{salida}/lamina.png'))
    return out, {'handovers_h_est': res.get('handovers_por_hora_por_estacion'),
                 'servicio_pct': res.get('servicio_medio_pct')}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('run')
    ap.add_argument('salida')
    ap.add_argument('--titulo', default='')
    ap.add_argument('--commit', default='')
    ap.add_argument('--sin-logo', action='store_true')
    ap.add_argument('--nombre-corrida', default='', help='nombre a rotular (si el directorio esta montado con otro nombre)')
    a = ap.parse_args(argv)
    global NOMBRE_CORRIDA
    NOMBRE_CORRIDA = a.nombre_corrida
    tipo = tipo_corrida(a.run)
    if tipo is None:
        print(f'ERROR: {a.run} no tiene telemetry.jsonl ni (escena.json + red.jsonl + metricas.json): '
              'graficas dibuja corridas de lazo, ADCS o de red', file=sys.stderr)
        return 2
    os.makedirs(a.salida, exist_ok=True)
    estilo()
    pie_txt = (f'Co.De Aerospace · Estación ATP · corrida {nombre(a.run)}'
               + (f' · commit {a.commit[:9]}' if a.commit else ''))
    f = {'lazo': graficas_lazo, 'adcs': graficas_adcs, 'red': graficas_red, 'otra': graficas_otra}[tipo]
    archivos, resumen = f(a.run, a.salida, a.titulo, pie_txt, not a.sin_logo)
    for r in archivos:
        print('GRAFICA', os.path.basename(r))
    print('RESUMEN', json.dumps({'tipo': tipo, **resumen}, ensure_ascii=False))
    return 0


if __name__ == '__main__':
    sys.exit(main())
