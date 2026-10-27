"""Operacion del rover con retardo de enlace (R8, CONTRATO sec. 19 «Operacion»; resultados en docs/ROVER.md «Resultados R8»).

Lazo cerrado operador en Tierra <-> enlace <-> rover con autonomia a bordo. Python puro (numpy), determinista, sin ROS.

    from atp_rover.operacion import correr_operacion
    met = correr_operacion({'planeta': 'marte', 'terreno': 'rocas', 'modo': 'plan_sol', 'semilla': 1})

MODOS DE OPERACION (comparables, misma ruta, mismo terreno, misma semilla):
  * `teleop`   (a) teleoperacion directa «mover y esperar»: Tierra manda UN segmento (`segmento_m`, la distancia que cree ver
               libre en las camaras) y espera la telemetria que confirme la llegada antes de mandar el siguiente. Sin alarmas locales.
  * `metas`    (b) metas por pase: en cada telemetria Tierra manda las metas hasta `adelanto_m` (100 m) por delante de la ultima
               posicion recibida; el rover las sigue solo con el SeguidorBasico (cli.py). Sin alarmas locales.
  * `plan_sol` (c) plan de sol: igual, pero compromete la distancia de una jornada de conduccion y el rover lleva alarmas de
               seguridad LOCALES (pendiente/atasco, seguridad.py): para solo y rodea el obstaculo sin esperar a Tierra.
  * `gemelo`   (d) operador con gemelo digital: como (c), mas (i) prediccion: estima donde estara el rover cuando llegue la orden
               (propaga las ordenes en transito con su velocidad nominal, que lleva un error `error_gemelo`) y compromete solo
               lo que el enlace obliga (adelanto = v (2 OWL + 2 reaccion + espera tipica)), (ii) conocimiento del terreno:
               rodea de antemano los obstaculos de pendiente que su MDE conoce (`p_dem`, fraccion de celdas conocidas).
  Los cuatro se pueden retocar con `seguridad_local`, `gemelo_prediccion`, `p_dem`, `adelanto_m`.

QUE ES REAL Y QUE ES SUSTITUTO (leer antes de citar una cifra). La dinamica completa de R1 corre a ~130 pasos/s, es decir ~11 min
de pared por 90 s simulados: no se puede simular un sol (88 775 s) con ella. Por eso:
  * la FISICA de R1 (con `rodadura_en_motor: true`, navegacion `basico`) se corre en micro-corridas de calibracion (`calibrar`) por
    (planeta, terreno, semilla) y de ellas salen la razon de velocidad efectiva, la potencia de marcha y el Wh/m;
  * el rover del lazo operativo es un SUSTITUTO cinematico (monociclo, tic de 1 s) que sigue las metas con el SeguidorBasico real
    de cli.py y usa esa velocidad y esa potencia; NO tiene suspension, deslizamiento ni vuelco fisicos;
  * los PELIGROS son una regla, no fisica: celda con pendiente >= `pend_alarma_deg` (campo de pendientes del Terreno de R1, teselado con
    desplazamiento aleatorio cada 100 m) = obstaculo que el rover local ve a `sensado_m`; zonas blandas ocultas (`bache_por_km`)
    = atasco que solo se detecta por velocidad nula `atasco_s`. Sobre `pend_vuelco_deg` hay una probabilidad `p_vuelco` de vuelco.
    La recuperacion (vaiven) dura `recup_s` y tiene exito con `p_recup`. Todos estos numeros son SUPUESTOS de este modulo;
  * el enlace usa los de R7: retardo de luz y ventanas del rele (Enlace.entrega_*), o un enlace sintetico (OWL constante y ventanas
    periodicas) para los barridos. Sus supuestos son los de comunicacion.py.
Ver docs/ROVER.md «Resultados R8» para lo que NO se probo.
"""
import heapq
import json
import math
import os
from bisect import bisect_right

import numpy as np

from atp_rover import cli
from atp_rover.comunicacion import CUERPOS, Enlace, jd_de_iso
from atp_rover.planetas import planeta as _planeta
from atp_rover.terreno import Terreno

MODOS = ('teleop', 'metas', 'plan_sol', 'gemelo')
LETRA = {'teleop': 'a', 'metas': 'b', 'plan_sol': 'c', 'gemelo': 'd'}
ENLACES = ('ideal', 'retardo', 'ventanas')
P_BASE_W = 20.0                       # consumo fijo del rover (R1, ParametrosRover.p_base_w)
ESPACIO_METAS_M = 10.0                # separacion de las metas del plan
LADO_M, RES_M = 100.0, 0.5            # tesela del campo de pendientes
CALIBRACION_ARCHIVO = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'docs', 'rover_resultados',
                                   'operacion_calibracion.json')

# Valores por defecto de cada modo (se pisan con la configuracion explicita).
MODO_DEF = {
    'teleop': dict(politica='segmentos', seguridad_local=False, gemelo_prediccion=False, p_dem=0.0, adelanto_m=None),
    'metas': dict(politica='plan', seguridad_local=False, gemelo_prediccion=False, p_dem=0.0, adelanto_m=100.0),
    'plan_sol': dict(politica='plan', seguridad_local=True, gemelo_prediccion=False, p_dem=0.0, adelanto_m=None),
    'gemelo': dict(politica='plan', seguridad_local=True, gemelo_prediccion=True, p_dem=0.6, adelanto_m=None),
}

# rocas del estudio: abundancia moderada (Golombek y Rapp 1997 da 0.02-0.1 para Marte) repartidas en toda la tesela
TERRENO_ROCAS = {'abundancia': 0.03, 'radio_rocas': 70.0}

DEFECTOS_OP = {
    'planeta': 'marte', 'terreno': 'llano', 'semilla': 0, 'modo': 'plan_sol',
    'enlace': None,                   # None = 'ventanas' en Marte, 'retardo' en la Luna
    'owl_s': None,                    # retardo de una via [s]; None = el de la fecha (Enlace)
    'intervalo_ventanas_s': None,     # None = ventanas reales del rele; numero = ventanas periodicas sinteticas
    'dur_ventana_s': 600.0, 'rele': None, 'fecha': '2026-10-07', 'lat_deg': None, 'lon_deg': None,
    'n_soles': 3, 'conduccion_h': 2.0, 'v_max': 0.5, 'dt_s': 1.0,
    'p_perdida_cmd': 0.02, 'p_perdida_estado': 0.02,
    'reaccion_operador_s': 10.0, 'segmento_m': 10.0, 'ttl_s': None, 'tel_s': 30.0,
    'bache_por_km': 2.0, 'bache_largo_m': 3.0, 'pend_alarma_deg': 25.0, 'pend_vuelco_deg': 35.0, 'p_vuelco': 0.5,
    'sensado_m': 1.5, 'max_esq': 8, 'atasco_s': 3.0, 'recup_s': 90.0, 'p_recup': 0.6, 'max_recup': 3,
    'error_gemelo': 0.15, 'terreno_params': {},
    'seguridad_local': None, 'gemelo_prediccion': None, 'p_dem': None, 'adelanto_m': None,
    'calibracion': None,              # dict {v_ratio, p_mov_w} o None = tabla de calibracion del archivo
}


# ------------------------------------------------------------------------------------------------------------------
# Calibracion con la fisica de R1
# ------------------------------------------------------------------------------------------------------------------
def calibrar(planeta, terreno, semilla, duracion_s=50.0, v_max=0.5, terreno_params=None, navegacion='basico'):
    """Micro-corrida de la dinamica de R1 (rodadura_en_motor, navegacion basico, ruta recta al este) -> razon de velocidad,
    potencia de marcha y Wh/m. Mide de t = 10 s a la duracion (sin el arranque). ~35 s de pared."""
    from atp_rover.lazo import correr
    cfg = {'planeta': planeta, 'terreno': terreno, 'semilla': semilla, 'ruta': [[400.0, 0.0]], 'duracion_s': duracion_s,
           'v_max': v_max, 'navegacion': navegacion, 'rover': {'rodadura_en_motor': True}}
    if terreno_params:
        cfg['terreno_params'] = terreno_params
    met, filas = correr(cfg)
    f = [r for r in filas if r['t'] >= 10.0]
    v = np.array([r['v'] for r in f])
    p = np.array([r['potencia_w'] for r in f])
    mov = v > 0.05
    d = sum(math.hypot(b['x'] - a['x'], b['y'] - a['y']) for a, b in zip(f[:-1], f[1:]))
    dt = f[-1]['t'] - f[0]['t']
    dw = f[-1]['energia_wh'] - f[0]['energia_wh']
    return {'v_ratio': round(float(np.median(v)) / v_max, 4), 'v_media_ms': round(d / dt, 4),
            'p_mov_w': round(float(p[mov].mean()) if mov.any() else P_BASE_W, 3), 'wh_por_m': round(dw / d, 5) if d > 0.5 else None,
            'dist_m': round(d, 3), 'atascos': met['atascos'], 'vuelcos': met['vuelcos'], 'duracion_s': duracion_s}


def _cal_uno(args):
    pl, te, sem, dur = args
    tp = TERRENO_ROCAS if te == 'rocas' else None
    return f'{pl}|{te}|{sem}', calibrar(pl, te, sem, duracion_s=dur, terreno_params=tp)


def calibrar_tabla(planetas, terrenos, semillas, duracion_s=50.0, trabajadores=4):
    """Tabla {'planeta|terreno|semilla': calibracion} con la dinamica de R1 (procesos en paralelo; cada micro-corrida es independiente)."""
    trabajos = [(p, t, s, duracion_s) for p in planetas for t in terrenos for s in semillas]
    if trabajadores > 1:
        from multiprocessing import Pool
        with Pool(trabajadores) as pool:
            res = pool.map(_cal_uno, trabajos)
    else:
        res = [_cal_uno(j) for j in trabajos]
    return dict(sorted(res))


_CAL = {}


def cargar_calibracion(ruta=None):
    ruta = ruta or CALIBRACION_ARCHIVO
    if ruta not in _CAL:
        try:
            with open(ruta) as f:
                _CAL[ruta] = json.load(f).get('tabla', {})
        except (OSError, ValueError):
            _CAL[ruta] = {}
    return _CAL[ruta]


def calibracion_de(cfg):
    """(dict, origen): lo explicito en cfg, la tabla del archivo (planeta|terreno|semilla) o un valor por defecto marcado."""
    if cfg.get('calibracion'):
        return {**{'v_ratio': 0.85, 'p_mov_w': 45.0}, **cfg['calibracion']}, 'explicita'
    c = cargar_calibracion().get(f"{cfg['planeta']}|{cfg['terreno']}|{cfg['semilla']}")
    if c:
        return c, 'fisica'
    return {'v_ratio': 0.85, 'p_mov_w': 45.0}, 'defecto'


# ------------------------------------------------------------------------------------------------------------------
# Mundo: pendientes del Terreno de R1 teseladas + zonas blandas
# ------------------------------------------------------------------------------------------------------------------
_MAPAS = {}


def _mapa_pendiente(planeta, terreno, semilla, tp):
    clave = (planeta, terreno, int(semilla), json.dumps(tp, sort_keys=True))
    if clave not in _MAPAS:
        pl = _planeta(planeta)
        ter = Terreno(terreno, int(semilla), pl['suelo'], **(tp or {}))
        n = int(LADO_M / RES_M)
        xs = -LADO_M / 2 + RES_M * np.arange(n)
        X, Y = np.meshgrid(xs, xs)
        _MAPAS[clave] = np.asarray(ter.pendiente_deg(X, Y))
    return _MAPAS[clave]


class Mundo:
    """Peligros en (x, y). `roca(x, y)`: pendiente local >= pend_alarma (obstaculo visible). `bache(x, y)`: zona blanda oculta."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.n = int(LADO_M / RES_M)
        sl = _mapa_pendiente(cfg['planeta'], cfg['terreno'], cfg['semilla'], cfg['terreno_params'])
        self.sl = sl.tolist()
        self.lim = cfg['pend_alarma_deg']
        s = int(cfg['semilla'])
        self.conocida = np.random.default_rng([s, 11]).random((self.n, self.n)) < (cfg['p_dem'] or 0.0)
        largo_max = 60000.0
        r = np.random.default_rng([s, 12])
        k = r.poisson(cfg['bache_por_km'] * largo_max / 1000.0)
        self.b_x = np.sort(r.uniform(15.0, largo_max, k)).tolist()
        self.b_y = r.uniform(-1.0, 1.0, k).tolist() if k else []
        self.b_largo, self.b_ancho = cfg['bache_largo_m'], 1.8
        self._sh = {}

    def _celda(self, x, y):
        tx, ty = math.floor((x + 50.0) / LADO_M), math.floor((y + 50.0) / LADO_M)
        sh = self._sh.get((tx, ty))
        if sh is None:
            sh = tuple(int(v) for v in np.random.default_rng([int(self.cfg['semilla']), 13, tx + 10000, ty + 10000]).integers(0, self.n, 2))
            self._sh[(tx, ty)] = sh
        i = (int((x + 50.0 - LADO_M * tx) / RES_M) + sh[0]) % self.n
        j = (int((y + 50.0 - LADO_M * ty) / RES_M) + sh[1]) % self.n
        return j, i

    def pend(self, x, y):
        j, i = self._celda(x, y)
        return self.sl[j][i]

    def roca(self, x, y):
        j, i = self._celda(x, y)
        return self.sl[j][i] >= self.lim

    def roca_conocida(self, x, y):
        j, i = self._celda(x, y)
        return bool(self.conocida[j, i]) and self.sl[j][i] >= self.lim

    def bache(self, x, y):
        k = bisect_right(self.b_x, x) - 1
        for m in (k, k - 1):
            if 0 <= m < len(self.b_x) and x <= self.b_x[m] + self.b_largo and abs(y - self.b_y[m]) < self.b_ancho:
                return True
        return False


# ------------------------------------------------------------------------------------------------------------------
# Enlace de la operacion
# ------------------------------------------------------------------------------------------------------------------
_ENLACES = {}


def _enlace_real(planeta, rele, lat, lon, fecha):
    clave = (planeta, rele, lat, lon, fecha)
    if clave not in _ENLACES:
        _ENLACES[clave] = Enlace(planeta, rele, lat, lon, None, 'omni_helice', jd0=jd_de_iso(fecha))
    return _ENLACES[clave]


class EnlaceOp:
    """cmd(t) -> instante de llegada al rover (o None); tel(t) -> instante de llegada a Tierra (o None). Cuenta perdidas."""

    def __init__(self, cfg, dia_s, rng):
        self.cfg, self.rng, self.dia = cfg, rng, dia_s
        self.tipo = cfg['enlace']
        self.p_cmd, self.p_tel = cfg['p_perdida_cmd'], cfg['p_perdida_estado']
        self.stats = dict(cmd_enviados=0, cmd_perdidos=0, cmd_no_llegan=0, tel_enviadas=0, tel_perdidas=0, tel_no_llegan=0)
        self.enl = None
        self.off = 0.0
        self.w_int = cfg['intervalo_ventanas_s']
        self.w_dur = cfg['dur_ventana_s']
        self.w_fase = 0.0
        pl = cfg['planeta']
        if self.tipo != 'ideal' and (cfg['owl_s'] is None or (self.tipo == 'ventanas' and self.w_int is None)):
            self.enl = _enlace_real(pl, cfg['rele'], cfg['lat_deg'], cfg['lon_deg'], cfg['fecha'])
        self.off = ((int(cfg['semilla']) * 0.37) % 1.0) * dia_s
        self.horizonte = (cfg['n_soles'] + 4) * dia_s
        if self.tipo == 'ideal':
            self.owl = 0.0
        elif cfg['owl_s'] is not None:
            self.owl = float(cfg['owl_s'])
        else:
            self.owl = float(self.enl._owlt(self.off))
        if self.tipo == 'ventanas' and self.w_int:
            self.w_fase = ((int(cfg['semilla']) * 0.61) % 1.0) * self.w_int
        self.lat_cmd = self._latencia_tipica()

    # ventanas sinteticas
    def _abre(self, t):
        """Primer instante >= t con ventana sintetica abierta."""
        k = math.floor((t - self.w_fase) / self.w_int)
        ini = k * self.w_int + self.w_fase
        return t if t < ini + self.w_dur else (k + 1) * self.w_int + self.w_fase

    def _cmd_llega(self, t):
        if self.tipo == 'ideal':
            return t
        if self.enl is not None and (self.tipo == 'ventanas'):
            ta, _ = self.enl.entrega_comando(t + self.off, self.horizonte, dt_s=30.0, modo='ventanas')
            return None if ta is None else ta - self.off
        if self.tipo == 'ventanas':
            return self._abre(t + self.owl)
        return t + self.owl

    def _tel_llega(self, t):
        if self.tipo == 'ideal':
            return t
        if self.enl is not None and self.tipo == 'ventanas':
            ta = self.enl.entrega_telemetria(t + self.off, self.horizonte, dt_s=30.0, modo='ventanas')
            return None if ta is None else ta - self.off
        if self.tipo == 'ventanas':
            return self._abre(t) + self.owl
        return t + self.owl

    def _latencia_tipica(self):
        if self.tipo != 'ventanas':
            return self.owl
        ts = np.linspace(0.0, self.dia, 24, endpoint=False)
        v = [self._cmd_llega(float(t)) for t in ts]
        v = [a - t for a, t in zip(v, ts) if a is not None]
        return float(np.median(v)) if v else self.dia

    def cmd(self, t):
        s = self.stats
        s['cmd_enviados'] += 1
        if self.rng.random() < self.p_cmd:
            s['cmd_perdidos'] += 1
            return None
        a = self._cmd_llega(t)
        if a is None:
            s['cmd_no_llegan'] += 1
        return a

    def tel(self, t):
        s = self.stats
        s['tel_enviadas'] += 1
        if self.rng.random() < self.p_tel:
            s['tel_perdidas'] += 1
            return None
        a = self._tel_llega(t)
        if a is None:
            s['tel_no_llegan'] += 1
        return a


# ------------------------------------------------------------------------------------------------------------------
# Simulacion
# ------------------------------------------------------------------------------------------------------------------
def completar(cfg):
    c = {**DEFECTOS_OP, **cfg}
    if c['modo'] not in MODOS:
        raise ValueError(f'modo desconocido: {c["modo"]!r} (hay: {", ".join(MODOS)})')
    pl = c['planeta']
    if pl not in CUERPOS:
        raise KeyError(f'operacion solo para {", ".join(CUERPOS)} (no {pl!r})')
    if c['enlace'] is None:
        c['enlace'] = 'ventanas' if pl == 'marte' else 'retardo'
    if c['enlace'] not in ENLACES:
        raise ValueError(f'enlace desconocido: {c["enlace"]!r} (hay: {", ".join(ENLACES)})')
    if c['rele'] is None:
        c['rele'] = 'mro' if pl == 'marte' else 'polar_100km'
    if c['lat_deg'] is None:
        c['lat_deg'], c['lon_deg'] = (-4.6, 137.4) if pl == 'marte' else (-85.0, 0.0)
    for k, v in MODO_DEF[c['modo']].items():
        if c.get(k) is None:
            c[k] = v
    c['terreno_params'] = dict(c['terreno_params'] or {})
    if c['terreno'] == 'rocas' and not c['terreno_params']:
        c['terreno_params'] = dict(TERRENO_ROCAS)
    return c


class Operacion:
    def __init__(self, cfg):
        self.c = c = completar(cfg)
        sem = int(c['semilla'])
        self.rng = np.random.default_rng([sem, 21])           # enlace
        self.rng_rec = np.random.default_rng([sem, 22])       # recuperaciones
        self.rng_vuelco = np.random.default_rng([sem, 23])    # vuelcos
        self.dia = CUERPOS[c['planeta']]['dia_s']
        self.cal, self.cal_origen = calibracion_de(c)
        self.vr = float(self.cal['v_ratio'])
        self.p_mov = float(self.cal['p_mov_w'])
        self.v_nom = c['v_max'] * self.vr
        self.mundo = Mundo(c)
        self.link = EnlaceOp(c, self.dia, self.rng)
        self.lat = max(self.link.lat_cmd, 1.0)
        self.t_conduc = c['conduccion_h'] * 3600.0
        self.ev, self._seq = [], 0
        # --- rover
        self.x = self.y = self.th = 0.0
        self.seg = cli.SeguidorBasico([], c['v_max'], 0.5)
        self.n_acc = -1
        self.est = 'normal'            # normal | detect | recup | hold | muerto
        self.t_est = 0.0
        self.intentos = 0
        self.origen_recup = None
        self.lado = 1.0
        self.flag_atasco = False
        self.n_esq, self.x_esq0 = 0, 0.0
        self.t_haz = 0.0
        self.evt = None                # peligro abierto sin respuesta: dict(t, tipo)
        self.n_tel = 0
        # --- Tierra
        self.plan = []
        self.tel_ult = None
        self.t_send = None
        self.t_recup = None
        self.t_decide = -1e18
        self.lado_t = 1.0
        # --- cuentas
        self.m = dict(x_max=0.0, odometro=0.0, ocioso=0.0, atascado=0.0, recup=0.0, energia=0.0, atascos=0, vuelcos=0, caducados=0,
                      aceptados=0, recup_ok=0, obsoleto=0, reac_local=[], reac_tierra=[], reac_local_atasco=[], reac_local_pend=[], sin_reaccion=0, x_fin_sol=[])
        a = c['adelanto_m']
        self.adelanto = a if a is not None else self.v_nom * self.t_conduc * 1.1
        ttl = c['ttl_s']
        self.ttl = ttl if ttl is not None else (2 * self.link.lat_cmd + 120.0 if c['modo'] == 'teleop' else self.dia)
        self.spacing = c['segmento_m'] if c['modo'] == 'teleop' else ESPACIO_METAS_M
        self._precarga()

    # --- eventos
    def push(self, t, tipo, dato=None):
        self._seq += 1
        heapq.heappush(self.ev, (t, self._seq, tipo, dato))

    def en_ventana(self, t):
        s = math.floor(t / self.dia)
        ini = s * self.dia + 0.3 * self.dia
        return ini <= t < ini + self.t_conduc

    def prox_ventana(self, t):
        s = math.floor(t / self.dia)
        ini = s * self.dia + 0.3 * self.dia
        return ini if t < ini else (s + 1) * self.dia + 0.3 * self.dia

    # --- plan de Tierra
    def _extender_plan(self, fin_x, gemelo):
        c = self.c
        x0 = self.plan[-1][0] if self.plan else 0.0
        y0 = self.plan[-1][1] if self.plan else 0.0
        while x0 < fin_x:
            nx = x0 + self.spacing
            if gemelo and c['p_dem'] > 0:
                xs = np.arange(x0, nx + 1e-9, RES_M)
                xh = next((float(a) for a in xs if self.mundo.roca_conocida(float(a), 0.0)), None)
                if xh is not None:
                    # rodea por el lado cuya franja de 4 m (segun el MDE) tiene menos obstaculos conocidos
                    def malos(sg):
                        return sum(self.mundo.roca_conocida(float(a), sg * 4.0) for a in np.arange(xh - 3.0, xh + 4.0 + 1e-9, RES_M))
                    sg = self.lado_t if malos(self.lado_t) <= malos(-self.lado_t) else -self.lado_t
                    self.lado_t = -sg
                    d = sg * 4.0
                    self.plan += [(xh - 3.0, 0.0), (xh - 3.0, d), (xh + 4.0, d), (xh + 4.0, 0.0)]
                    x0 = xh + 4.0
                    continue
            self.plan.append((nx, 0.0))
            x0 = nx

    def _precarga(self):
        c = self.c
        self._extender_plan(self.spacing if c['modo'] == 'teleop' else self.adelanto, c['gemelo_prediccion'] or c['p_dem'] > 0)
        self.seg.ruta = list(self.plan)
        self.n_acc = len(self.plan) - 1
        self.push(0.0, 'tel_gen')

    # --- telemetria del rover
    def _snap(self, t):
        return dict(t=t, x=self.x, y=self.y, n_acc=self.n_acc, idle=self.seg.terminado and self.est == 'normal',
                    atascado=self.flag_atasco,
                    vuelco=self.est == 'muerto')

    def emite_tel(self, t):
        a = self.link.tel(t)
        if a is not None:
            self.push(a, 'tel_rx', self._snap(t))

    # --- eventos de Tierra
    def enviar(self, t, dato):
        a = self.link.cmd(t)
        dato['t_send'] = t
        if a is not None:
            self.push(a, 'cmd_rx', dato)

    def _decide(self, t):
        c, s = self.c, self.tel_ult
        if s is None or s['vuelco']:
            return
        lat = self.lat
        nuevo = self.t_send is None or s['t'] > self.t_send + lat
        if s['atascado']:
            if self.t_recup is None or s['t'] > self.t_recup + lat:
                self.t_recup = t
                self.enviar(t, dict(tipo='recup'))
            return
        n_pend = len(self.plan) - 1 - s['n_acc']
        if c['politica'] == 'segmentos':
            if s['idle'] and n_pend <= 0:
                self.plan.append((s['x'] + c['segmento_m'], 0.0))
                self.t_send = t
                self.enviar(t, dict(tipo='wps', desde=s['n_acc'] + 1, wps=self.plan[s['n_acc'] + 1:]))
            elif n_pend > 0 and nuevo:
                self.t_send = t
                self.enviar(t, dict(tipo='wps', desde=s['n_acc'] + 1, wps=self.plan[s['n_acc'] + 1:]))
            return
        if c['gemelo_prediccion']:
            horizonte = self.v_nom * (2 * self.link.owl + 2 * c['reaccion_operador_s'] + self.lat)
            horizonte = min(max(horizonte, 50.0), self.adelanto)
            fin_plan = self.plan[-1][0]
            v_est = self.v_nom * (1 + c['error_gemelo'] * self._signo_gemelo())
            x_pred = min(fin_plan, s['x'] + v_est * max(0.0, t + self.lat - s['t'])) if not s['idle'] else s['x']
            fin_x = x_pred + horizonte
        else:
            fin_x = s['x'] + self.adelanto
        if self.plan[-1][0] < fin_x - self.spacing / 2:
            self._extender_plan(fin_x, c['gemelo_prediccion'] or c['p_dem'] > 0)
            n_pend = len(self.plan) - 1 - s['n_acc']
            self.t_send = t
            self.enviar(t, dict(tipo='wps', desde=s['n_acc'] + 1, wps=self.plan[s['n_acc'] + 1:]))
        elif n_pend > 0 and nuevo:
            self.t_send = t
            self.enviar(t, dict(tipo='wps', desde=s['n_acc'] + 1, wps=self.plan[s['n_acc'] + 1:]))

    def _signo_gemelo(self):
        return 1.0 if (int(self.c['semilla']) % 2 == 0) else -1.0

    # --- rover: recepcion de ordenes y maniobras
    def _rx_cmd(self, t, d):
        if t - d['t_send'] > self.ttl:
            self.m['caducados'] += 1
            return
        if d['tipo'] == 'recup':
            if self.est in ('detect', 'hold'):
                self._inicia_recup(t, 'tierra')
            else:
                self.m['obsoleto'] += 1
            return
        for k, wp in enumerate(d['wps']):
            if d['desde'] + k > self.n_acc:
                self.seg.ruta.append(tuple(wp))
                self.n_acc = d['desde'] + k
                self.m['aceptados'] += 1

    def _registra_reaccion(self, t, origen):
        if self.evt is not None:
            (self.m['reac_local'] if origen == 'local' else self.m['reac_tierra']).append(t - self.evt['t'])
            if origen == 'local':
                self.m['reac_local_' + ('atasco' if self.evt['tipo'] == 'atasco' else 'pend')].append(t - self.evt['t'])
            self.evt = None

    def _inicia_recup(self, t, origen):
        self._registra_reaccion(t, origen)
        self.est, self.t_est, self.origen_recup = 'recup', t, origen
        if origen == 'tierra':
            self.intentos = 0

    def _lado_libre(self):
        """+1 / -1 (hacia y positivo / negativo) si el rover ve libre de obstaculos `sensado_m` a ese lado; 0 si ninguno."""
        for sg in (self.lado, -self.lado):
            if not any(self.mundo.roca(self.x, self.y + sg * r) for r in np.arange(0.25, self.c['sensado_m'] + 1e-9, 0.25)):
                return sg
        return 0

    def _esquiva(self, forzar=False):
        """Inserta un rodeo lateral de 4 m / 7 m hacia el lado libre. False si no hay lado libre o ya van `max_esq` rodeos sin avanzar."""
        if self.m['x_max'] > self.x_esq0 + 8.0:
            self.n_esq, self.x_esq0 = 0, self.m['x_max']
        sg = self._lado_libre()
        if not forzar and (sg == 0 or self.n_esq >= self.c['max_esq']):
            return False
        sg = sg or self.lado
        self.n_esq += 1
        d = sg * 4.0
        self.lado = -sg
        i = self.seg.i
        xs = self.x + 7.0
        self.seg.ruta[i:i] = [(self.x, self.y + d), (xs, self.y + d), (xs, self.y)]
        while len(self.seg.ruta) > i + 3 and self.seg.ruta[i + 3][0] < xs:     # metas que quedaron atras de la salida del rodeo
            del self.seg.ruta[i + 3]
        return True

    def _paso_rover(self, t, dt):
        c, m = self.c, self.m
        if self.est == 'muerto':
            return
        if self.est == 'detect':
            m['atascado'] += dt
            if t >= self.t_est:
                self.flag_atasco = True
                self.emite_tel(t)
                if c['seguridad_local'] and self.intentos < c['max_recup']:
                    self._inicia_recup(t, 'local')
                else:
                    self.est = 'hold'
            return
        if self.est == 'hold':
            m['atascado'] += dt
            m['energia'] += P_BASE_W * dt / 3600.0
            return
        if self.est == 'recup':
            m['recup'] += dt
            m['energia'] += self.p_mov * dt / 3600.0
            if t - self.t_est >= c['recup_s']:
                if self.rng_rec.random() < c['p_recup']:
                    ang = math.radians(self.th)
                    self.x -= 2.0 * math.cos(ang)
                    self.y -= 2.0 * math.sin(ang)
                    self._esquiva(forzar=True)
                    self.est = 'normal'
                    self.m['recup_ok'] += 1
                    self.flag_atasco = False
                    self.emite_tel(t)
                else:
                    self.intentos += 1
                    if self.origen_recup == 'local' and self.intentos < c['max_recup']:
                        self.t_est = t
                    else:
                        self.est = 'hold'
                        self.evt = dict(t=self.t_haz, tipo='atasco')
                        self.emite_tel(t)
            return
        # normal
        if self.seg.terminado:
            m['ocioso'] += dt
            m['energia'] += P_BASE_W * dt / 3600.0
            return
        e = {'x': self.x, 'y': self.y, 'rumbo_deg': self.th}
        v, w = self.seg.mando(e)
        if self.seg.terminado:
            self.emite_tel(t)
            m['ocioso'] += dt
            return
        v *= self.vr
        ang = math.radians(self.th)
        if c['seguridad_local'] and v > 0:
            ca, sa = math.cos(ang), math.sin(ang)
            if any(self.mundo.roca(self.x + r * ca, self.y + r * sa) for r in np.arange(0.25, c['sensado_m'] + 1e-9, 0.25)):
                if self.evt is None:
                    self.evt = dict(t=t, tipo='pendiente')
                if self._esquiva():
                    self._registra_reaccion(t, 'local')
                else:                      # sin lado libre o demasiados rodeos: modo seguro, espera a Tierra
                    self.est, self.flag_atasco, self.t_haz = 'hold', True, self.evt['t']
                m['energia'] += P_BASE_W * dt / 3600.0
                self.emite_tel(t)
                return
        self.th = (self.th + math.degrees(w * dt) + 180.0) % 360.0 - 180.0
        ang = math.radians(self.th)
        nx, ny = self.x + v * dt * math.cos(ang), self.y + v * dt * math.sin(ang)
        m['odometro'] += math.hypot(nx - self.x, ny - self.y)
        self.x, self.y = nx, ny
        m['energia'] += (self.p_mov if v > 0.02 else P_BASE_W) * dt / 3600.0
        if self.x > m['x_max']:
            m['x_max'] = self.x
        rock = self.mundo.roca(nx, ny)
        if rock or self.mundo.bache(nx, ny):
            m['atascos'] += 1
            if rock and self.mundo.pend(nx, ny) >= c['pend_vuelco_deg'] and self.rng_vuelco.random() < c['p_vuelco']:
                self.est = 'muerto'
                m['vuelcos'] += 1
                self.emite_tel(t)
                return
            self.est, self.t_est = 'detect', t + c['atasco_s']
            self.intentos = 0
            self.t_haz = t
            self.evt = dict(t=t, tipo='atasco')

    # --- bucle
    def correr(self):
        c, m = self.c, self.m
        dt = c['dt_s']
        t_fin = c['n_soles'] * self.dia
        t = 0.0
        sol_ult = 0
        ev = self.ev
        while t < t_fin:
            while ev and ev[0][0] <= t:
                te, _, tipo, d = heapq.heappop(ev)
                if tipo == 'cmd_rx':
                    self._rx_cmd(te, d)
                elif tipo == 'tel_rx':
                    if self.tel_ult is None or d['t'] > self.tel_ult['t']:
                        self.tel_ult = d
                    if te + c['reaccion_operador_s'] - self.t_decide >= 0.9 * c['reaccion_operador_s']:
                        self.push(te + c['reaccion_operador_s'], 'decide')
                elif tipo == 'decide':
                    if te - self.t_decide >= 0.9 * c['reaccion_operador_s']:
                        self.t_decide = te
                        self._decide(te)
                        self.push(te + max(4 * self.lat + 60.0, 120.0), 'decide')
                elif tipo == 'tel_gen':
                    self.emite_tel(te)
                    self.push(te + (c['tel_s'] if self.en_ventana(te) else 600.0), 'tel_gen')
            sol = int(t // self.dia)
            while sol_ult < sol:
                m['x_fin_sol'].append(m['x_max'])
                sol_ult += 1
            if self.est == 'muerto':
                t = t_fin
                break
            if self.en_ventana(t):
                prox = ev[0][0] if ev else t_fin
                if self.est == 'hold' or (self.est == 'normal' and self.seg.terminado):
                    # sin trabajo que simular tic a tic: se salta al proximo evento (el tiempo ocioso/atascado se contabiliza)
                    t2 = min(prox, self.prox_fin_ventana(t), t_fin)
                    t2 = max(t2, t + 1e-6)
                    gasto = t2 - t
                    if self.est == 'hold':
                        m['atascado'] += gasto
                    else:
                        m['ocioso'] += gasto
                    m['energia'] += P_BASE_W * gasto / 3600.0
                    t = t2
                else:
                    self._paso_rover(t, dt)
                    t += dt
            else:
                t = min(ev[0][0] if ev else t_fin, self.prox_ventana(t), t_fin)
        while len(m['x_fin_sol']) < c['n_soles']:
            m['x_fin_sol'].append(m['x_max'])
        return self.metricas()

    def prox_fin_ventana(self, t):
        s = math.floor(t / self.dia)
        return s * self.dia + 0.3 * self.dia + self.t_conduc

    def metricas(self):
        c, m, L = self.c, self.m, self.link.stats
        n = c['n_soles']
        fin = m['x_fin_sol'][:n]
        por_sol = [fin[0]] + [b - a for a, b in zip(fin[:-1], fin[1:])]
        util = m['x_max'] / n

        def med(v):
            return round(float(np.mean(v)), 3) if v else None
        ab = self.evt is not None
        out = dict(
            planeta=c['planeta'], terreno=c['terreno'], semilla=int(c['semilla']), modo=c['modo'], letra=LETRA[c['modo']],
            enlace=c['enlace'], owl_s=round(self.link.owl, 3), lat_cmd_tipica_s=round(self.link.lat_cmd, 1),
            intervalo_ventanas_s=c['intervalo_ventanas_s'], n_soles=n, conduccion_h=c['conduccion_h'],
            seguridad_local=bool(c['seguridad_local']), gemelo_prediccion=bool(c['gemelo_prediccion']), p_dem=c['p_dem'],
            adelanto_m=round(self.adelanto, 1), calibracion=self.cal_origen,
            dist_util_m_sol=round(util, 3), dist_util_por_sol=[round(v, 3) for v in por_sol],
            recorrido_m_sol=round(m['odometro'] / n, 3),
            t_ocioso_s_sol=round(m['ocioso'] / n, 1), t_atascado_s_sol=round(m['atascado'] / n, 1), t_recup_s_sol=round(m['recup'] / n, 1),
            frac_ocioso=round(m['ocioso'] / (n * self.t_conduc), 4),
            cmd_enviados=L['cmd_enviados'], cmd_perdidos=L['cmd_perdidos'], cmd_no_llegan=L['cmd_no_llegan'],
            cmd_caducados=m['caducados'], cmd_aceptados_wps=m['aceptados'], cmd_recup_obsoletos=m['obsoleto'],
            tel_enviadas=L['tel_enviadas'], tel_perdidas=L['tel_perdidas'],
            atascos=m['atascos'], atascos_sol=round(m['atascos'] / n, 2), recuperaciones_ok=m['recup_ok'], vuelcos=m['vuelcos'],
            reaccion_local_s=med(m['reac_local']), reaccion_local_n=len(m['reac_local']),
            reaccion_local_pendiente_s=med(m['reac_local_pend']), reaccion_local_atasco_s=med(m['reac_local_atasco']),
            reaccion_tierra_s=med(m['reac_tierra']), reaccion_tierra_n=len(m['reac_tierra']), peligro_sin_respuesta=int(ab),
            energia_wh_sol=round(m['energia'] / n, 2), wh_por_m_util=round(m['energia'] / m['x_max'], 4) if m['x_max'] > 1 else None)
        return out


def correr_operacion(cfg, carpeta=None):
    """Simula una operacion y devuelve las metricas. Con `carpeta` escribe metrics.json (sort_keys, sin sellos de reloj)."""
    met = Operacion(cfg).correr()
    if carpeta:
        os.makedirs(carpeta, exist_ok=True)
        with open(os.path.join(carpeta, 'metrics.json'), 'w') as f:
            json.dump(met, f, indent=1, sort_keys=True)
            f.write('\n')
    return met


def main(argv=None):
    import argparse
    import sys
    ap = argparse.ArgumentParser(allow_abbrev=False, description='Operacion del rover con retardo de enlace (R8)')
    ap.add_argument('--escena')
    ap.add_argument('--salida', default='/tmp/rover_operacion')
    ap.add_argument('--id', default='operacion')
    a, resto = ap.parse_known_args(argv)
    cfg = {}
    if a.escena:
        import yaml
        with open(a.escena) as f:
            cfg = dict((yaml.safe_load(f).get('args') or {}))
    i = 0
    while i < len(resto):
        r = resto[i]
        i += 1
        if not r.startswith('--'):
            ap.error(f'argumento no reconocido: {r}')
        if '=' in r:
            k, v = r[2:].split('=', 1)
        elif i < len(resto):
            k, v = r[2:], resto[i]
            i += 1
        else:
            ap.error(f'falta el valor de {r}')
        if k not in DEFECTOS_OP:
            ap.error(f'argumento desconocido: --{k} (hay: {", ".join(sorted(DEFECTOS_OP))})')
        try:
            cfg[k] = json.loads(v)
        except ValueError:
            cfg[k] = v
    met = correr_operacion(cfg, os.path.join(a.salida, a.id))
    print('RESUMEN ' + json.dumps(met, sort_keys=True))
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())
