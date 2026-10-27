"""Validacion fuera de muestra de la navegacion del rover (R10): casos, metricas y estadistica. Python puro, determinista.

PRE-REGISTRO (fijado en el commit que introduce este archivo, ANTES de correr ningun caso nuevo; docs/ROVER.md «Resultados R10»).
Pregunta: la navegacion `completa` de R6 (afinada 4 veces sobre los 75 casos de R4: seeds 1-3, ruta [[12,0],[12,8]]) y la `basico`
generalizan a casos que no se usaron para ajustar? Todo es en simulacion, con los suelos supuestos de R4 y la fisica corregida
(`rodadura_en_motor: true`, defecto desde la ronda 8).

CONJUNTOS (disjuntos)
  * ajuste     : los 75 casos de R4/R6 (semillas 1-3, 5 cuerpos x 5 terrenos, ruta fija). Solo aqui se hacen ablaciones y ajustes.
  * validacion : semillas 101-130. 5 cuerpos x 5 regimenes de terreno (rocoso, dunas, rocas, escalones, lateral) x 6 casos = 150
                 casos; cada semilla se usa 5 veces en celdas distintas; la ruta rota entre 4 (`RUTAS`: zigzag largo con 3 giros, diagonal,
                 recta de 18 m «con obstaculo en medio» —el escalon de `escalones` cae en x = 5 y 11—, ida y vuelta). Nunca se ajusta aqui.
  * tercero    : semillas 201-230, mismo diseno. Solo se evalua UNA vez, al final, si se versiona una navegacion nueva.
  Comparaciones por pares: el mismo caso (planeta, terreno, parametros, semilla, ruta) con las dos navegaciones.

METRICAS
  * llegada     : `meta_alcanzada` del controlador (tasa, y diferencia pareada completa - basica).
  * atasco REAL : la metrica del rover cuenta cada episodio de 5 s con |v| < 20 % de la orden, de modo que cada pausa de la maniobra
                  de vaiven de R6 (atras 2.5 s, adelante 3 s) suma un «atasco» aunque el rover se este liberando. Aqui un atasco real es
                  SIN PROGRESO: se mide el progreso a lo largo de la ruta (`progreso`: metas alcanzadas en orden a 1 m + proyeccion sobre el
                  tramo actual, maximo corrido) y hay un episodio cuando pasan `VENTANA_ATASCO_S` = 30 s sin que el progreso maximo
                  suba `UMBRAL_PROGRESO_M` = 0.25 m. 30 s es 6 veces el episodio de 5 s y mas que una maniobra de vaiven (5.5 s por ciclo, 2
                  ciclos = 11 s), un giro en sitio de 180 grados (~5 s) o un arranque; un vaiven que libera al rover sube el progreso y no
                  cuenta, uno que no libera durante 30 s si. Se reportan: corridas con >= 1 episodio, episodios, segundos atascado y
                  `atasco_terminal` (la corrida termina por tiempo sin llegar con un episodio abierto). El contador nativo (`atascos`) se
                  reporta aparte, para comparar con R4/R6.
  * vuelcos     : `vuelcos` de la dinamica (el cuerpo supera 70 grados de inclinacion), tasa por corrida.
  * energia por metro util : energia_wh / progreso maximo (m); tiempo; distancia recorrida / largo de la ruta (ambas, solo en pares
                  donde las DOS navegaciones llegan, para que la comparacion no se mezcle con la tasa de llegada).

ESTADISTICA
  Bootstrap percentil de los casos (pares), 10 000 remuestreos, semilla fija `SEMILLA_BOOTSTRAP`, IC 95 %. Efecto: diferencia de
  proporciones y recuentos discordantes (b, c) con prueba exacta del signo (binomial, dos colas) para lo binario; razon geometrica
  (exp de la media de log-razones) y d_z de Cohen para lo continuo. 150 casos: la semiamplitud del IC de una diferencia de llegada
  es de ~6 puntos si hay ~30 % de pares discordantes; no se corrige por comparaciones multiples (se avisa).

CRITERIOS DE ACEPTACION de `completa` frente a `basico` (fijados antes de ver numeros; lo que no se cumple se reporta tal cual):
  C1 llegada      : limite inferior del IC95 % de (llegada completa - llegada basica) >= -0.05 (no inferioridad con margen de 5 puntos).
                    C1b (informativo): limite inferior > 0 (superioridad).
  C2 vuelcos      : vuelcos de completa <= vuelcos de basica (totales) Y limite superior del IC95 % de la diferencia pareada de
                    tasa de vuelco <= +0.02.
  C3 atasco real  : limite superior del IC95 % de la diferencia pareada de la tasa de corridas con atasco real <= +0.05.
  C4 energia      : (pares donde ambas llegan) limite superior del IC95 % de la razon geometrica de Wh por metro util <= 1.10.
  C5 tiempo       : (pares donde ambas llegan) limite superior del IC95 % de la razon geometrica del tiempo hasta la meta <= 1.15.
  C6 distancia    : (pares donde ambas llegan) limite superior del IC95 % de la razon geometrica de distancia/recta <= 1.10.
  C7 regimenes    : en ninguno de los 5 terrenos ni de los 5 cuerpos el IC95 % de la diferencia de llegada queda entero por debajo de 0
                    (completa significativamente peor; 10 comparaciones, sin correccion).
Si completa incumple C1, C2 o C7 en la validacion se diagnostica con ablaciones SOLO en el conjunto de ajuste y se versiona una
`completa_r10` evaluada una sola vez en el tercero con los mismos criterios (la actual `completa` y `completa_r4` no cambian bytes).
"""
import math

import numpy as np

SEMILLA_BOOTSTRAP = 20261008
N_BOOT = 10000
VENTANA_ATASCO_S = 30.0
UMBRAL_PROGRESO_M = 0.25
RADIO_META_PROGRESO_M = 1.0
V_MAX = 0.5

PLANETAS = ('luna', 'marte', 'titan', 'ceres', 'tierra')
REGIMENES = ('rocoso', 'dunas', 'rocas', 'escalones', 'lateral')
RUTAS = {
    'zigzag': [[8, 0], [8, 8], [16, 8], [16, 0]],     # 32 m, tres giros de 90 grados
    'diagonal': [[14, 14]],                           # 19.8 m, arranca mirando a 45 grados de la meta
    'recta_obstaculo': [[18, 0]],                     # 18 m; en `escalones` cruza dos gradas
    'ida_y_vuelta': [[10, 4], [0, 0]],                # 21.5 m, vuelve al origen
}
ORDEN_RUTAS = ('zigzag', 'diagonal', 'recta_obstaculo', 'ida_y_vuelta')
N_POR_CELDA = 6


def largo_ruta(ruta, origen=(0.0, 0.0)):
    pts = [tuple(origen)] + [tuple(map(float, m)) for m in ruta]
    return float(sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:])))


def _terreno_de(regimen, j):
    """(tipo, terreno_params) de un regimen para el caso j de la celda."""
    if regimen == 'lateral':
        return 'pendiente', {'angulo_deg': (8.0, 12.0)[j % 2], 'direccion_deg': 90.0}
    if regimen == 'escalones':
        return 'escalones', {'altura': (0.06, 0.10)[(j // 2) % 2], 'x_inicio': 5.0, 'separacion': 6.0, 'n': 2}
    return regimen, {}


def casos(semilla_base, n_por_celda=N_POR_CELDA):
    """Los 150 casos de un conjunto fuera de muestra (sin navegacion): lista de cfg de `lazo.correr`.
    Semillas semilla_base .. semilla_base + 29; la celda c y el caso j usan la semilla semilla_base + (c n + j) mod 30 y la ruta
    ORDEN_RUTAS[(c + j) mod 4], de modo que cada celda ve las 4 rutas y cada semilla aparece 5 veces en celdas distintas."""
    out = []
    celdas = [(p, r) for p in PLANETAS for r in REGIMENES]
    for c, (planeta, regimen) in enumerate(celdas):
        for j in range(n_por_celda):
            semilla = semilla_base + (c * n_por_celda + j) % 30
            nombre = ORDEN_RUTAS[(c + j) % len(ORDEN_RUTAS)]
            ruta = RUTAS[nombre]
            tipo, tp = _terreno_de(regimen, j)
            ideal = largo_ruta(ruta) / V_MAX
            out.append({'planeta': planeta, 'terreno': tipo, 'semilla': int(semilla), 'terreno_params': tp, 'ruta': ruta,
                        'duracion_s': round(3.5 * ideal + 20.0, 1), 'v_max': V_MAX, 'estudio': 'val',
                        'regimen': regimen, 'ruta_nombre': nombre})
    return out


def casos_rapidos(semilla_base):
    """Subconjunto para `validar_todo.sh`: 2 cuerpos x 3 regimenes, ruta diagonal, 1 caso (6 casos)."""
    todos = casos(semilla_base)
    out = []
    for p in ('luna', 'marte'):
        for r in ('rocoso', 'escalones', 'lateral'):
            out.append(next(c for c in todos if c['planeta'] == p and c['regimen'] == r and c['ruta_nombre'] == 'diagonal'))
    return out


# ---------------------------------------------------------------------------------------------------------------------------
# Progreso y atasco real
# ---------------------------------------------------------------------------------------------------------------------------
def progreso(filas, ruta, origen=(0.0, 0.0), radio=RADIO_META_PROGRESO_M):
    """Progreso maximo corrido (m) a lo largo de la ruta en cada muestra de `filas`. Las metas se dan por alcanzadas EN ORDEN al pasar
    a `radio` de ellas; entre metas el progreso es el acumulado hasta la anterior + la proyeccion (acotada al tramo) sobre el tramo
    actual. Asi una ruta de ida y vuelta, que se solapa a si misma, no se confunde. -> lista de floats, no decreciente."""
    pts = [tuple(origen)] + [tuple(map(float, m)) for m in ruta]
    segs = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:])]
    cum = [0.0]
    for s in segs:
        cum.append(cum[-1] + s)
    j = 0
    mejor = 0.0
    out = []
    for e in filas:
        x, y = e['x'], e['y']
        while j < len(segs) and math.hypot(pts[j + 1][0] - x, pts[j + 1][1] - y) <= radio:
            j += 1
        if j >= len(segs):
            p = cum[-1]
        else:
            ax, ay = pts[j]
            bx, by = pts[j + 1]
            L = max(segs[j], 1e-9)
            t = ((x - ax) * (bx - ax) + (y - ay) * (by - ay)) / L
            p = cum[j] + max(0.0, min(L, t))
        mejor = max(mejor, p)
        out.append(mejor)
    return out


def atasco_real(tiempos, prog, ventana_s=VENTANA_ATASCO_S, umbral_m=UMBRAL_PROGRESO_M, llego=False, total_m=None):
    """Episodios de atasco SIN PROGRESO: tramos de >= `ventana_s` segundos sin que el progreso maximo suba `umbral_m` sobre el ultimo
    nivel. -> dict(episodios, t_atascado_s, terminal). Un episodio abierto al terminar la corrida es terminal salvo que el rover haya
    llegado (`llego`); el tiempo atascado cuenta todo el tramo sin progreso (incluidos los primeros `ventana_s` s)."""
    nivel = prog[0] if prog else 0.0
    t_nivel = tiempos[0] if len(tiempos) else 0.0
    abierto = False
    episodios = 0
    t_atasc = 0.0
    for t, p in zip(tiempos, prog):
        if p >= nivel + umbral_m:
            if abierto:
                t_atasc += t - t_nivel
            abierto = False
            nivel, t_nivel = p, t
        elif not abierto and t - t_nivel >= ventana_s:
            abierto = True
            episodios += 1
    if abierto and len(tiempos):
        t_atasc += tiempos[-1] - t_nivel
    return {'episodios': episodios, 't_atascado_s': round(t_atasc, 3), 'terminal': bool(abierto and not llego)}


def resumen_corrida(met, filas, ruta):
    """Metricas de validacion de una corrida (a guardar junto a `met`)."""
    tiempos = [e['t'] for e in filas]
    prog = progreso(filas, ruta)
    llego = bool(met.get('meta_alcanzada'))
    total = largo_ruta(ruta)
    ar = atasco_real(tiempos, prog, llego=llego)
    util = prog[-1] if prog else 0.0
    return {'progreso_m': round(util, 4), 'ruta_m': round(total, 4), 'atasco_real_episodios': ar['episodios'],
            'atasco_real_s': ar['t_atascado_s'], 'atasco_terminal': ar['terminal'],
            'wh_por_m_util': round(met['energia_wh'] / util, 6) if util >= 2.0 else None,
            'dist_sobre_recta': round(met['distancia_m'] / total, 4)}


# ---------------------------------------------------------------------------------------------------------------------------
# Estadistica
# ---------------------------------------------------------------------------------------------------------------------------
def bootstrap_ic(x, estad=np.mean, n_boot=N_BOOT, semilla=SEMILLA_BOOTSTRAP, nivel=0.95):
    """(estimador, lo, hi): bootstrap percentil sobre los elementos de x (pares), semilla fija. x vacio -> nan."""
    x = np.asarray(x, dtype=float)
    if x.size == 0:
        return float('nan'), float('nan'), float('nan')
    rng = np.random.default_rng(semilla)
    idx = rng.integers(0, x.size, size=(n_boot, x.size))
    if estad is np.mean:
        est = x[idx].mean(axis=1)
    else:
        est = np.array([estad(x[i]) for i in idx])
    a = (1.0 - nivel) / 2.0
    lo, hi = np.quantile(est, [a, 1.0 - a])
    return float(estad(x)), float(lo), float(hi)


def prueba_signo_exacta(b, c):
    """p de dos colas de la prueba exacta del signo (binomial p = 0.5) con b y c discordantes."""
    n = b + c
    if n == 0:
        return 1.0
    k = min(b, c)
    p = sum(math.comb(n, i) for i in range(0, k + 1)) / 2.0 ** n
    return float(min(1.0, 2.0 * p))


def d_z(d):
    """d de Cohen para diferencias pareadas: media / desviacion tipica (ddof 1). nan si no hay variacion."""
    d = np.asarray(d, dtype=float)
    if d.size < 2:
        return float('nan')
    s = float(np.std(d, ddof=1))
    return float(np.mean(d) / s) if s > 0 else float('nan')


def comparar_binaria(a, b):
    """a, b: 0/1 por caso (misma ordenacion). -> diferencia pareada media (a - b) con IC, recuentos discordantes, p exacta, d_z."""
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    d = a - b
    est, lo, hi = bootstrap_ic(d)
    gana = int(np.sum((a == 1) & (b == 0)))      # solo a
    pierde = int(np.sum((a == 0) & (b == 1)))    # solo b
    return {'n': int(a.size), 'tasa_a': float(a.mean()) if a.size else float('nan'), 'tasa_b': float(b.mean()) if b.size else float('nan'),
            'dif': est, 'ic_lo': lo, 'ic_hi': hi, 'solo_a': gana, 'solo_b': pierde,
            'p_signo': prueba_signo_exacta(gana, pierde), 'd_z': d_z(d)}


def comparar_razon(a, b):
    """a, b > 0 por caso. -> razon geometrica a/b con IC (bootstrap de la media de log-razones), mediana de la razon y d_z de los logs."""
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    if a.size == 0:
        return {'n': 0, 'razon': float('nan'), 'ic_lo': float('nan'), 'ic_hi': float('nan'), 'mediana': float('nan'), 'd_z': float('nan')}
    lr = np.log(a / b)
    est, lo, hi = bootstrap_ic(lr)
    return {'n': int(a.size), 'razon': float(math.exp(est)), 'ic_lo': float(math.exp(lo)), 'ic_hi': float(math.exp(hi)),
            'mediana': float(np.median(a / b)), 'd_z': d_z(lr)}


def clave_caso(cfg):
    """Identidad de un caso, sin la navegacion ni sus ajustes: dos corridas con la misma clave forman un par."""
    return str(sorted((k, repr(v)) for k, v in cfg.items() if k not in ('navegacion', 'nav', 'traccion', 'variante', 'seguridad')))


def emparejar(filas, modo_a, modo_b, variante_a=None, variante_b=None):
    """filas: [{'cfg','met','val'}]. -> lista de (fila_a, fila_b) con el mismo caso. Un modo se identifica por (navegacion, variante)."""
    def ident(f):
        return f['cfg']['navegacion'], f['cfg'].get('variante')
    A = {clave_caso(f['cfg']): f for f in filas if ident(f) == (modo_a, variante_a)}
    B = {clave_caso(f['cfg']): f for f in filas if ident(f) == (modo_b, variante_b)}
    return [(A[k], B[k]) for k in sorted(A) if k in B]


def comparar_modos(pares):
    """Comparacion pareada completa (todas las metricas) de una lista de pares (a, b)."""
    if not pares:
        return {'n': 0}
    lleg_a = [int(a['met']['meta_alcanzada']) for a, _ in pares]
    lleg_b = [int(b['met']['meta_alcanzada']) for _, b in pares]
    vc_a = [int(a['met']['vuelcos'] > 0) for a, _ in pares]
    vc_b = [int(b['met']['vuelcos'] > 0) for _, b in pares]
    at_a = [int(a['val']['atasco_real_episodios'] > 0) for a, _ in pares]
    at_b = [int(b['val']['atasco_real_episodios'] > 0) for _, b in pares]
    ambos = [(a, b) for a, b in pares if a['met']['meta_alcanzada'] and b['met']['meta_alcanzada']]
    en_ambos = [(a, b) for a, b in ambos if a['val']['wh_por_m_util'] and b['val']['wh_por_m_util']]
    return {
        'n': len(pares), 'llegada': comparar_binaria(lleg_a, lleg_b), 'vuelco': comparar_binaria(vc_a, vc_b),
        'vuelcos_a': int(sum(a['met']['vuelcos'] for a, _ in pares)), 'vuelcos_b': int(sum(b['met']['vuelcos'] for _, b in pares)),
        'atasco_real': comparar_binaria(at_a, at_b),
        'atascos_nativos_a': int(sum(a['met']['atascos'] for a, _ in pares)), 'atascos_nativos_b': int(sum(b['met']['atascos'] for _, b in pares)),
        'atasco_real_episodios_a': int(sum(a['val']['atasco_real_episodios'] for a, _ in pares)),
        'atasco_real_episodios_b': int(sum(b['val']['atasco_real_episodios'] for _, b in pares)),
        'atasco_terminal_a': int(sum(a['val']['atasco_terminal'] for a, _ in pares)), 'atasco_terminal_b': int(sum(b['val']['atasco_terminal'] for _, b in pares)),
        'energia_por_m_util': comparar_razon([a['val']['wh_por_m_util'] for a, _ in en_ambos], [b['val']['wh_por_m_util'] for _, b in en_ambos]),
        'tiempo': comparar_razon([a['met']['tiempo_s'] for a, _ in ambos], [b['met']['tiempo_s'] for _, b in ambos]),
        'distancia_sobre_recta': comparar_razon([a['val']['dist_sobre_recta'] for a, _ in ambos], [b['val']['dist_sobre_recta'] for _, b in ambos]),
        'n_ambos_llegan': len(ambos)}


def por_grupo(pares, clave):
    """Comparacion de llegada por grupo (clave: funcion del cfg del primer elemento del par). -> {grupo: comparar_binaria}."""
    g = {}
    for a, b in pares:
        g.setdefault(clave(a['cfg']), []).append((a, b))
    return {k: comparar_binaria([int(a['met']['meta_alcanzada']) for a, _ in v], [int(b['met']['meta_alcanzada']) for _, b in v])
            for k, v in sorted(g.items())}


def evaluar_criterios(comp, regimenes_terreno, regimenes_cuerpo):
    """Criterios C1..C7 del pre-registro. -> lista de dicts {id, texto, cumple, valor}. `comp` = salida de comparar_modos (a = completa, b = basica)."""
    ll, vc, at = comp['llegada'], comp['vuelco'], comp['atasco_real']
    en, ti, di = comp['energia_por_m_util'], comp['tiempo'], comp['distancia_sobre_recta']
    peor = [k for k, v in {**{f'terreno {k}': v for k, v in regimenes_terreno.items()},
                           **{f'cuerpo {k}': v for k, v in regimenes_cuerpo.items()}}.items() if v['ic_hi'] < 0.0]

    def ok(x):
        return bool(x) if x == x else False
    return [
        {'id': 'C1', 'texto': 'llegada: LI del IC95 de (completa - basica) >= -0.05', 'valor': f"{ll['dif']:+.3f} [{ll['ic_lo']:+.3f}, {ll['ic_hi']:+.3f}]", 'cumple': ok(ll['ic_lo'] >= -0.05)},
        {'id': 'C1b', 'texto': 'llegada (informativo): LI > 0 (superioridad)', 'valor': f"LI {ll['ic_lo']:+.3f}", 'cumple': ok(ll['ic_lo'] > 0.0)},
        {'id': 'C2', 'texto': 'vuelcos: totales completa <= basica y LS del IC95 de la dif. de tasa <= +0.02',
         'valor': f"{comp['vuelcos_a']} contra {comp['vuelcos_b']}; dif {vc['dif']:+.3f} [{vc['ic_lo']:+.3f}, {vc['ic_hi']:+.3f}]",
         'cumple': ok(comp['vuelcos_a'] <= comp['vuelcos_b'] and vc['ic_hi'] <= 0.02)},
        {'id': 'C3', 'texto': 'atasco real: LS del IC95 de la dif. de tasa de corridas con atasco <= +0.05', 'valor': f"{at['dif']:+.3f} [{at['ic_lo']:+.3f}, {at['ic_hi']:+.3f}]", 'cumple': ok(at['ic_hi'] <= 0.05)},
        {'id': 'C4', 'texto': 'energia por metro util (ambas llegan): LS de la razon <= 1.10', 'valor': f"x{en['razon']:.3f} [{en['ic_lo']:.3f}, {en['ic_hi']:.3f}] (n={en['n']})", 'cumple': ok(en['ic_hi'] <= 1.10)},
        {'id': 'C5', 'texto': 'tiempo (ambas llegan): LS de la razon <= 1.15', 'valor': f"x{ti['razon']:.3f} [{ti['ic_lo']:.3f}, {ti['ic_hi']:.3f}] (n={ti['n']})", 'cumple': ok(ti['ic_hi'] <= 1.15)},
        {'id': 'C6', 'texto': 'distancia/recta (ambas llegan): LS de la razon <= 1.10', 'valor': f"x{di['razon']:.3f} [{di['ic_lo']:.3f}, {di['ic_hi']:.3f}] (n={di['n']})", 'cumple': ok(di['ic_hi'] <= 1.10)},
        {'id': 'C7', 'texto': 'ningun terreno ni cuerpo con completa significativamente peor en llegada (IC95 entero < 0)', 'valor': ('ninguno' if not peor else ', '.join(peor)), 'cumple': not peor},
    ]
