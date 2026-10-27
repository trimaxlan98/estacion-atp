"""Cadena del rover en LOCKSTEP (CONTRATO sec. 8 y 18; docs/ROVER.md «Resultados R9»). Python puro, sin ROS.

Los nodos `rover_dinamica`, `rover_navegador` y `rover_enlace` (modo `lockstep`) son una envoltura delgada de las
clases de aqui; `correr_cadena` ejecuta la MISMA cadena en un solo proceso (sin DDS) y es el equivalente "sin ROS"
con el que se compara la corrida ROS. Sin enlace, la cadena da los mismos bytes que `lazo.correr`.

Semantica (la que fija R9; todo en TIEMPO DE ESCENA t_k = k * 0.1 s, el `t` del estado, que arranca en 0 tras `asentar`):

  1. La planta publica el estado e_k (sello t_k) y espera.
  2. Sin enlace: el navegador responde con el mando (v, w) calculado de e_k y una FICHA {t: t_k, n: 1}. Con enlace: el
     enlace (que recibe e_k) entrega al rover las ordenes cuyo instante de entrega ya llego (<= t_k), en orden de
     entrega, y manda la ficha {t: t_k, n: nº de ordenes}.
  3. El mando de la ficha de t_k rige el intervalo [t_k, t_{k+1}) COMPLETO (retencion de orden cero): la planta integra
     los 10 pasos de 0.01 s con el, muestrea e_{k+1} y confirma en /atp/step_ack el sello del /clock con el que publico e_k.
     Si llegan varias ordenes en la misma ficha rige la ultima; sin ordenes sigue la vigente (al arrancar, (0, 0)).
  4. Con enlace, una orden que el operador manda en t_k (calculada de lo que le llego en t_k) entra al enlace en el
     tick k+1 con su sello de envio t_k; se entrega en el primer tick t >= t_k + retardo (cuantizacion al paso de 0.1 s)
     y, como minimo, un paso despues (en modo `ideal` el retardo es 0 pero la entrega es en t_{k+1}).

Pareo por fichas: un Twist no lleva sello, asi que cada publicador de ordenes/estados pone, DESPUES de ellas, una ficha
en `<topico>_listo` con el sello y el numero de mensajes de ese instante (`n`); el consumidor espera la ficha y sus `n`
mensajes (DDS no ordena entre topicos distintos). Nadie responde dos veces al mismo sello (los duplicados se descartan).
"""
import json
import math
import os

from atp_rover import cli
from atp_rover.lazo import ControlBasico, ControlCompleto, NAVEGACIONES
from atp_rover.planetas import planeta
from atp_rover.rover import DT
from atp_rover.terreno import Terreno

PASO_S = 0.1                       # intervalo de decision y de muestreo (= step_s del reloj)
PASOS = int(round(PASO_S / DT))    # pasos de integracion por tick (10)

# Opciones propias de la cadena (el resto de las claves son las de cli.DEFECTOS). `enlace`: 'ninguno' = navegador -> rover
# directo (equivale a lazo.correr); ideal | retardo | ventanas = operador -> rover_enlace -> rover (modo de FiltroEnlace).
DEFECTOS_LOCK = {
    'enlace': 'ninguno', 'operador': 'navegador', 'guion': [],
    'rele': None, 'lat_deg': None, 'lon_deg': None, 'antena': 'omni_helice', 'el_min_deg': 10.0,
    'fecha': '2026-10-07', 't0_s': 0.0, 'p_perdida_cmd': 0.0, 'p_perdida_estado': 0.0, 'horizonte_dias': 10.0,
    'terreno_enlace': 'llano',
    # `rover` = la dinamica de R1; `cinematica` = monociclo ideal SIN fisica, solo para ejercitar el protocolo y el enlace durante
    # horas de tiempo simulado (la dinamica de R1 integra a ~1 s simulado por s de pared). No sirve para cifras del rover.
    'dinamica': 'rover',
}
# Estacion de cada cuerpo si la escena no da (rele, lat, lon): Gale 4.6 S 137.4 E con MRO; Luna 89 S, 0 E con un orbitador polar.
ENLACE_POR_PLANETA = {'marte': ('mro', -4.6, 137.4), 'luna': ('polar_100km', -89.0, 0.0)}


def cargar_cfg(escena=None, extra=None):
    """cfg completo: DEFECTOS de cli + DEFECTOS_LOCK + `args` del YAML `escena` + `extra` (dict o JSON)."""
    cfg = {**cli.DEFECTOS, **DEFECTOS_LOCK}
    if escena:
        cfg.update(cli.cargar_escena(escena))
    if isinstance(extra, str):
        extra = json.loads(extra or '{}')
    cfg.update(extra or {})
    if cfg['navegacion'] not in NAVEGACIONES:
        raise ValueError(f'navegacion desconocida: {cfg["navegacion"]!r}')
    if cfg['enlace'] not in ('ninguno', 'ideal', 'retardo', 'ventanas'):
        raise ValueError(f'enlace desconocido: {cfg["enlace"]!r}')
    return cfg


# ------------------------------------------------------------------------------------------------------------------
# Fichas
# ------------------------------------------------------------------------------------------------------------------
class Buzon:
    """Mensajes que llegan por un topico + las fichas de ese topico. `sacar()` devuelve (ficha, mensajes) de la primera ficha
    cuyos `n` mensajes ya llegaron, o None. Descarta fichas con sello <= al ultimo atendido (duplicados)."""

    def __init__(self):
        self.items, self.fichas, self.ultimo = [], [], None

    def meter(self, x):
        self.items.append(x)

    def ficha(self, f):
        self.fichas.append(f)

    def sacar(self):
        while self.fichas and self.ultimo is not None and self.fichas[0]['t'] <= self.ultimo:
            self.fichas.pop(0)
        if self.fichas and len(self.items) >= self.fichas[0]['n']:
            f = self.fichas.pop(0)
            n = f['n']
            it = self.items[:n]
            del self.items[:n]
            self.ultimo = f['t']
            return f, it
        return None


# ------------------------------------------------------------------------------------------------------------------
# Planta
# ------------------------------------------------------------------------------------------------------------------
class PlantaLock:
    """El rover (dinamica de R1, RK4 a 0.01 s) avanzado por ticks de 0.1 s con el mando de la ficha."""

    def __init__(self, cfg):
        self.cfg = {**cli.DEFECTOS, **DEFECTOS_LOCK, **cfg}
        self.rov, self.ter, self.pl = cli.construir(self.cfg)
        self.rov.asentar(2.0)
        self.n = int(round(self.cfg['duracion_s'] / DT))
        self.hay_ruta = bool(self.cfg['ruta'])
        self.cmd = (0.0, 0.0)
        self.pasos = 0
        self.e = self.rov.estado()
        self.filas = [self.e]
        self.info = {}
        self.mandos = []           # [{"t","n","v","w"}]: ticks en los que llego al menos una orden
        self.fin = self.n <= 0

    vuelco = False

    def _integrar(self):
        for _ in range(min(PASOS, self.n - self.pasos)):
            self.rov.paso(*self.cmd)
            self.pasos += 1
        self.vuelco = self.rov.vuelco

    def _estado(self):
        return self.rov.estado()

    def estado_txt(self):
        return json.dumps(self.e)

    def aplicar(self, cmds, info=None):
        """Aplica la ficha del tick actual (t = self.e['t']): `cmds` [(v, w), ...] y `info` {'nav': ..., 'enlace': ...}."""
        if info:
            self.info.update(info)
        if cmds:
            self.cmd = (float(cmds[-1][0]), float(cmds[-1][1]))
            self.mandos.append({'t': self.e['t'], 'n': len(cmds), 'v': self.cmd[0], 'w': self.cmd[1]})
        self._integrar()
        if self.pasos % PASOS == 0:
            self.e = self._estado()
            self.filas.append(self.e)
            nav = self.info.get('nav') or {}
            if self.vuelco or (self.hay_ruta and nav.get('terminado')):
                self.fin = True
        if self.pasos >= self.n:
            self.fin = True

    def metricas(self):
        nav = self.info.get('nav') or {}
        met = self.rov.metricas(meta_alcanzada=bool(nav.get('alcanzada', False)), semilla=self.cfg['semilla'])
        met.update(nav.get('extras') or {})
        if self.cfg['enlace'] != 'ninguno':
            met['enlace'] = {'modo': self.cfg['enlace'], **(self.info.get('enlace') or {})}
        return met

    def cerrar(self, carpeta=None):
        met = self.metricas()
        if carpeta:
            os.makedirs(carpeta, exist_ok=True)
            with open(os.path.join(carpeta, 'telemetry.jsonl'), 'w') as f:
                for r in self.filas:
                    f.write(json.dumps(r) + '\n')
            with open(os.path.join(carpeta, 'metrics.json'), 'w') as f:
                json.dump(met, f, indent=2)
                f.write('\n')
            with open(os.path.join(carpeta, 'mandos.jsonl'), 'w') as f:
                for r in self.mandos:
                    f.write(json.dumps(r) + '\n')
        return met


class PlantaCinematica(PlantaLock):
    """Monociclo ideal con el mismo contrato que `PlantaLock` (no es la dinamica de R1): para probar el protocolo y el enlace
    en escalas de horas de tiempo simulado."""

    def __init__(self, cfg):
        self.cfg = {**cli.DEFECTOS, **DEFECTOS_LOCK, **cfg}
        c = self.cfg
        self.x, self.y, self.th = float(c['x0']), float(c['y0']), math.radians(c['rumbo_deg'])
        self.v = self.w = 0.0
        self.dist = 0.0
        self.n = int(round(c['duracion_s'] / DT))
        self.hay_ruta = bool(c['ruta'])
        self.cmd = (0.0, 0.0)
        self.pasos = 0
        self.e = self._estado()
        self.filas = [self.e]
        self.info = {}
        self.mandos = []
        self.fin = self.n <= 0

    def _integrar(self):
        n = min(PASOS, self.n - self.pasos)
        dt = n * DT
        v, w = self.cmd
        th2 = self.th + 0.5 * w * dt
        self.x += v * math.cos(th2) * dt
        self.y += v * math.sin(th2) * dt
        self.th += w * dt
        self.v, self.w = v, w
        self.dist += abs(v) * dt
        self.pasos += n

    def _estado(self):
        return {'t': round(getattr(self, 'pasos', 0) * DT, 6), 'x': round(self.x, 6), 'y': round(self.y, 6),
                'rumbo_deg': round(math.degrees(self.th) % 360.0, 6), 'v': round(getattr(self, 'v', 0.0), 6),
                'w': round(getattr(self, 'w', 0.0), 6)}

    def metricas(self):
        nav = self.info.get('nav') or {}
        met = {'distancia_m': round(self.dist, 6), 'tiempo_s': round(self.pasos * DT, 6),
               'meta_alcanzada': bool(nav.get('alcanzada', False)), 'planeta': self.cfg['planeta'], 'dinamica': 'cinematica',
               'semilla': int(self.cfg['semilla'])}
        met.update(nav.get('extras') or {})
        if self.cfg['enlace'] != 'ninguno':
            met['enlace'] = {'modo': self.cfg['enlace'], **(self.info.get('enlace') or {})}
        return met


def crear_planta(cfg):
    return PlantaCinematica(cfg) if cfg.get('dinamica') == 'cinematica' else PlantaLock(cfg)


# ------------------------------------------------------------------------------------------------------------------
# Navegador y operadores
# ------------------------------------------------------------------------------------------------------------------
class Navegacion:
    """El controlador de `lazo` (basico | completa | completa_r4) con la ruta de la escena: estado -> (v, w) + info para la planta."""

    def __init__(self, cfg):
        self.cfg = cfg = {**cli.DEFECTOS, **DEFECTOS_LOCK, **cfg}
        pl = planeta(cfg['planeta'])
        ter = Terreno(cfg['terreno'], cfg['semilla'], pl['suelo'], **(cfg.get('terreno_params') or {}))
        self.hay_ruta = bool(cfg['ruta'])
        self.ctl = (ControlBasico if cfg['navegacion'] == 'basico' else ControlCompleto)(cfg['ruta'], cfg, ter)

    def decidir(self, e):
        v, w = self.ctl.mando(e) if self.hay_ruta else (self.cfg['v_max'], 0.0)
        info = {'terminado': bool(self.ctl.terminado), 'alcanzada': bool(self.ctl.alcanzada), 'extras': self.ctl.extras()}
        return (float(v), float(w)), info


class OperadorNavegador:
    """Operador en Tierra que corre el navegador con lo que le llega por `estado_tierra` (uno por cada estado entregado)."""

    def __init__(self, cfg):
        self.nav = Navegacion(cfg)

    def responder(self, t, estados):
        cmds, info = [], None
        for txt in estados:
            vw, info = self.nav.decidir(json.loads(txt))
            cmds.append(vw)
        return cmds, ({'nav': info} if info else None)


class OperadorGuion:
    """Operador con un guion [[t, v, w], ...]: en el primer tick t_k >= t manda (v, w) (una vez)."""

    def __init__(self, guion):
        self.g = sorted((float(a), float(b), float(c)) for a, b, c in guion)
        self.i = 0

    def responder(self, t, estados):
        cmds = []
        while self.i < len(self.g) and self.g[self.i][0] <= t + 1e-9:
            cmds.append((self.g[self.i][1], self.g[self.i][2]))
            self.i += 1
        return cmds, None


def crear_operador(cfg):
    return OperadorGuion(cfg['guion']) if cfg.get('operador') == 'guion' else OperadorNavegador(cfg)


# ------------------------------------------------------------------------------------------------------------------
# Enlace
# ------------------------------------------------------------------------------------------------------------------
class EnlaceLock:
    """`FiltroEnlace` (R7) manejado por el tiempo de escena: un `tick` por estado de la planta (ver el docstring del modulo)."""

    def __init__(self, cfg):
        from atp_rover.comunicacion import Enlace, FiltroEnlace, jd_de_iso
        cfg = {**cli.DEFECTOS, **DEFECTOS_LOCK, **cfg}
        rele, lat, lon = ENLACE_POR_PLANETA.get(cfg['planeta'], (None, None, None))
        rele = cfg['rele'] or rele
        lat = lat if cfg['lat_deg'] is None else cfg['lat_deg']
        lon = lon if cfg['lon_deg'] is None else cfg['lon_deg']
        terr = None if cfg['terreno_enlace'] == 'llano' else Terreno(cfg['terreno_enlace'], int(cfg['semilla']))
        # `t0_s`: el instante de la escena t = 0 dentro del calendario del enlace (contactos del rele, giro del cuerpo) que
        # arranca en `fecha`; el filtro trabaja en tiempo de enlace T = t + t0_s. Con t0_s = 0 es el de R7.
        self.t0 = float(cfg['t0_s'])
        self.enlace = Enlace(cfg['planeta'], rele, lat, lon, terr, cfg['antena'], el_min_deg=cfg['el_min_deg'],
                             jd0=jd_de_iso(cfg['fecha']))
        self.filtro = FiltroEnlace(self.enlace, cfg['enlace'], cfg['horizonte_dias'] * self.enlace.cuerpo['dia_s'],
                                   cfg['p_perdida_cmd'], cfg['p_perdida_estado'], int(cfg['semilla']))
        self.t_prev = None
        self.t_enlace = -1.0

    def tick(self, t, estado_txt, cmds_prev=()):
        """t = sello del estado recien llegado. `cmds_prev`: las ordenes que el operador mando en el tick anterior (sello
        `t_prev`). Devuelve (ordenes entregadas al rover, estados_tierra [str JSON], estado del enlace o None, info)."""
        T = t + self.t0
        for vw in cmds_prev:
            self.filtro.cmd(self.t_prev + self.t0, (float(vw[0]), float(vw[1])))
        self.filtro.estado(T, estado_txt)
        cmds, ests = self.filtro.listos(T)
        ests_out = []
        for t_ent, txt in ests:
            d = json.loads(txt)
            d['t_tierra'] = round(t_ent - self.t0, 3)
            ests_out.append(json.dumps(d))
        status = None
        if t - self.t_enlace >= 1.0:
            self.t_enlace = t
            m = self.enlace.muestras([T])
            pc, pe = self.filtro.pendientes()
            status = {'t': round(t, 3), 'visible': bool(m['visible'][0]), 'el_deg': round(float(m['el'][0]), 2),
                      'tasa_bps': float(m['tasa_bps'][0]), 'cmd_pendientes': pc, 'estados_pendientes': pe, **self.filtro.stats}
        self.t_prev = t
        pc, pe = self.filtro.pendientes()
        info = {'cmd_pendientes': pc, 'estados_pendientes': pe, **self.filtro.stats}
        return [vw for _, vw in cmds], ests_out, status, info


# ------------------------------------------------------------------------------------------------------------------
# La cadena completa en un proceso (equivalente sin ROS)
# ------------------------------------------------------------------------------------------------------------------
def correr_cadena(cfg, carpeta=None):
    """Misma cadena que el launch `rover_lockstep.launch.py`, sin DDS. Devuelve (metricas, filas, mandos)."""
    cfg = cargar_cfg(None, cfg)
    pl = crear_planta(cfg)
    if cfg['enlace'] == 'ninguno':
        nav = Navegacion(cfg)
        while not pl.fin:
            vw, info = nav.decidir(json.loads(pl.estado_txt()))
            pl.aplicar([vw], {'nav': info})
    else:
        enl, op = EnlaceLock(cfg), crear_operador(cfg)
        prev, info_prev = [], None
        while not pl.fin:
            txt = pl.estado_txt()
            cmds, ests, _, info_enl = enl.tick(json.loads(txt)['t'], txt, prev)
            info = {'enlace': info_enl}
            if info_prev:
                info.update(info_prev)
            pl.aplicar(cmds, info)
            prev, info_prev = op.responder(json.loads(txt)['t'], ests)
    met = pl.cerrar(carpeta)
    return met, pl.filas, pl.mandos
