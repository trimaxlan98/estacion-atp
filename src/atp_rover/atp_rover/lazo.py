"""Lazo cerrado sin ROS (R4): dinamica de R1 + navegador de R2 (CONTRATO sec. 18, docs/ROVER.md «Resultados R4»).

    from atp_rover.lazo import correr
    met, filas = correr({'planeta': 'marte', 'terreno': 'dunas', 'ruta': [[12, 0]], 'navegacion': 'completa'})

`navegacion`:
  * `basico`   (defecto) el `SeguidorBasico` de cli.py: rumbo proporcional, orden recalculada en cada paso con el ultimo
               estado muestreado. Es exactamente el lazo que tenian las escenas de R1: mismos bytes.
  * `completa` el `Navegador` de R2: A* sobre costo de pendiente/rugosidad -> pure pursuit -> control de traccion ->
               seguridad. Se evalua a 10 Hz (cuando hay estado nuevo) y la orden se mantiene entre muestras.
Interfaz con R2: el `Terreno` de R1 ya cumple el duck-type (`altura`, `normal`); `energia_wh` del estado es energia
CONSUMIDA (se integra la potencia electrica), como asume `seguridad.py`.
"""
import json
import os

from atp_rover import cli
from atp_rover.navegacion import ControlTraccion, Navegador, ParamsNav, ParamsTraccion
from atp_rover.rover import DT
from atp_rover.seguridad import MonitorSeguridad, ParamsSeguridad

NAVEGACIONES = ('basico', 'completa', 'completa_r4', 'completa_r10')
FRENADO_ABORTA_S = 10.0     # tiempo simulado con la seguridad frenando tras el cual se da la corrida por abortada


# Ajustes del navegador para ESTA dinamica (R4). Los de R2 se calibraron con un rover unicycle falso; con la dinamica de R1
# (control de rueda proporcional: el par es proporcional al error de velocidad) una orden de v pequena en cuesta no da par
# suficiente y el rover se queda o retrocede, y un giro con v ~ 0.1 tarda decenas de segundos. Se sobreescriben con cfg['nav'].
NAV_LAZO = {'v_min': 0.3, 'pend_blando_deg': 10.0, 'giro_sitio_deg': 45.0}
# Con el umbral de R2 (0.3) el antideslizamiento recortaba la orden en cualquier cuesta (el deslizamiento de subida de un suelo
# blando ya es 0.3-0.4) y el rover no subia; con 0.5 solo actua ante patinaje real. Se sobreescribe con cfg['traccion'].
TRAC_LAZO = {'umbral_desliz': 0.5}
# R6: la `completa` arregla el diagnostico de R4. `completa_r4` conserva el comportamiento de R4 para comparar.
#  * costo graduado (la pendiente/rugosidad por encima del limite cuesta mas, no prohibe): el crater ya no da `sin_ruta`;
#  * replanifica cada 10 s y al desviarse 2 m;
#  * PI de velocidad sobre la v medida: el control de rueda de R1 es proporcional, asi que reducir la orden en cuesta
#    reducia tambien el par (R4 midio 0.11 m/s con 0.3 mandado en Marte 15 grados); ahora se sube la orden hasta lograr la v deseada;
#  * vaiven (atras-adelante) al detectar atasco y replan evitando ese sitio.
NAV_LAZO_R6 = {**NAV_LAZO, 'graduado': True, 'replan_s': 10.0, 'ki_vel': 0.5, 'kp_vel': 0.5, 'boost_max': 0.6, 'ff_desliz': True, 'acc_max': 1.5, 'frenar_pend': False, 'vaiven': True, 'vaiven_max': 5}
# R10: `completa` de R6 con tres cambios decididos en el conjunto de AJUSTE (docs/ROVER.md «Resultados R10»): alarma de pendiente sobre la huella,
# sin limite de aceleracion de la orden y sin vaiven (la maniobra de R6 acababa en `inalcanzable` en suelo blando donde la basica seguia empujando).
NAV_LAZO_R10 = {**NAV_LAZO_R6, 'pendiente_huella_m': 0.6, 'acc_max': 0.0, 'vaiven': False}
TRAC_LAZO_R6 = {'umbral_desliz': 0.9}   # R6: en este modelo el deslizamiento lo fija la fuerza que pide la cuesta, no la orden: recortar el par no lo baja


class ControlBasico:
    """Adaptador del SeguidorBasico (orden por paso, con el estado muestreado mas reciente)."""

    def __init__(self, ruta, cfg, terreno):
        self.seg = cli.SeguidorBasico(ruta, cfg['v_max'], cfg['radio_meta_m'])
        self.fallo = None

    @property
    def terminado(self):
        return self.seg.terminado

    @property
    def alcanzada(self):
        return bool(self.seg.ruta) and self.seg.terminado

    def mando(self, e):
        return self.seg.mando(e)

    def extras(self):
        return {}


class ControlCompleto:
    """Navegador de R2 por una lista de metas: planifica la siguiente al llegar a la actual."""

    def __init__(self, ruta, cfg, terreno):
        self.ruta = [tuple(map(float, m)) for m in ruta]
        self._modo = cfg['navegacion']
        base = {'completa_r4': NAV_LAZO, 'completa_r10': NAV_LAZO_R10}.get(cfg['navegacion'], NAV_LAZO_R6)
        pn = ParamsNav(v_max=cfg['v_max'], tol_meta=cfg['radio_meta_m'], **{**base, **(cfg.get('nav') or {})})
        ps = ParamsSeguridad(**(cfg.get('seguridad') or {}))
        pt = ParamsTraccion(**{**(TRAC_LAZO if self._modo == 'completa_r4' else TRAC_LAZO_R6), **(cfg.get('traccion') or {})})
        self.nav = Navegador(terreno, pn, MonitorSeguridad(ps), ControlTraccion(pt))
        self.i = 0
        self._plan = -1
        self._e = None
        self._vw = (0.0, 0.0)
        self.fallo = None          # None | 'sin_ruta' | 'seguridad' | 'inalcanzable'
        self.alarmas = {}
        self._t_frena = 0.0

    @property
    def terminado(self):
        return self.fallo is not None or self.i >= len(self.ruta)

    @property
    def alcanzada(self):
        return self.fallo is None and bool(self.ruta) and self.i >= len(self.ruta)

    def mando(self, e):
        if self.terminado:
            return 0.0, 0.0
        if e is self._e:
            return self._vw
        self._e = e
        if self._plan != self.i:
            self.nav.nueva_meta(e, self.ruta[self.i])
            self._plan = self.i
            if self.nav.sin_ruta:
                self.fallo = 'sin_ruta'
                self._vw = (0.0, 0.0)
                return self._vw
        (v, w), alarmas = self.nav.paso(e)
        for a in alarmas:
            self.alarmas[a['tipo']] = self.alarmas.get(a['tipo'], 0) + 1
        if self.nav.inalcanzable:
            self.fallo = 'inalcanzable'
        if self.nav.seg.debe_frenar():
            self._t_frena += 0.1
            if self._t_frena >= FRENADO_ABORTA_S:
                self.fallo = 'seguridad'
        else:
            self._t_frena = 0.0
        if self.nav.llegado:
            self.i += 1
            self._e = None
            return self.mando(e)
        self._vw = (v, w)
        return self._vw

    def extras(self):
        e = {'navegacion': self._modo, 'fallo': self.fallo, 'alarmas': dict(sorted(self.alarmas.items()))}
        if self._modo in ('completa', 'completa_r10'):
            e.update(vaivenes=self.nav.vaivenes, replanes=self.nav.replanes)
        return e


def correr(cfg, carpeta=None):
    """Simula la escena con el lazo elegido. Devuelve (metricas, filas). Con `carpeta` escribe telemetry.jsonl y metrics.json."""
    cfg = {**cli.DEFECTOS, **cfg}
    nav = cfg['navegacion']
    if nav not in NAVEGACIONES:
        raise ValueError(f'navegacion desconocida: {nav!r} (hay: {", ".join(NAVEGACIONES)})')
    rov, ter, pl = cli.construir(cfg)
    rov.asentar(2.0)
    ctl = (ControlBasico if nav == 'basico' else ControlCompleto)(cfg['ruta'], cfg, ter)
    hay_ruta = bool(cfg['ruta'])
    filas = []
    n = int(round(cfg['duracion_s'] / DT))
    e = rov.estado()
    filas.append(e)
    for k in range(1, n + 1):
        v, w = ctl.mando(e) if hay_ruta else (cfg['v_max'], 0.0)
        rov.paso(v, w)
        if k % 10 == 0:
            e = rov.estado()
            filas.append(e)
            if rov.vuelco or (hay_ruta and ctl.terminado):
                break
    met = rov.metricas(meta_alcanzada=ctl.alcanzada, semilla=cfg['semilla'])
    met.update(ctl.extras())
    if carpeta:
        os.makedirs(carpeta, exist_ok=True)
        with open(os.path.join(carpeta, 'telemetry.jsonl'), 'w') as f:
            for r in filas:
                f.write(json.dumps(r) + '\n')
        with open(os.path.join(carpeta, 'metrics.json'), 'w') as f:
            json.dump(met, f, indent=2)
            f.write('\n')
    return met, filas
