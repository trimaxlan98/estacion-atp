"""Orquestador de la mision integrada: un dia de operaciones de una red.

Flujo (docs/MISION.md):

  1. PLAN. El planificador de contactos (`atp_ros.planificador`, optimo) con las
     cargas utiles de cada satelite (`mundo.Carga`): que contactos se hacen.
  2. EJECUCION. Cada contacto asignado es una corrida en lockstep con el pase REAL
     SGP4 de ese satelite sobre esa estacion (`launch/mision_pase.launch.py`), con
     el satelite verdadero distinto de la prediccion (`mundo.Mundo`). El modo de
     seguimiento sale de la banda (X y Ka: autotrack; UHF y S: program track, y con
     receptor SDR en los contactos de calibracion).
  3. OD. Los `n_od` primeros contactos utiles de cada satelite son de CALIBRACION
     (autotrack enganchado: su montura mide al satelite). Con lo que registraron
     sus telemetrias (`od_mision`) se ajusta la orbita y, cuando hay `n_min_od`
     pases y han pasado `latencia_od_s` desde el ultimo, los contactos siguientes de
     ese satelite se vuelan con la orbita estimada (`profile:=od`) en vez del TLE.
  4. CONTABILIDAD. Los GB de cada contacto se recalculan con el C/N0 y el error de
     apuntamiento MEDIDOS en su corrida (`contabilidad`), con el ACM del planificador.
  5. INFORME. Plan contra ejecutado, minutos, GB, perdidas por apuntamiento,
     mejora por la OD (con gemelos: el mismo contacto con el TLE), incidencias.

Los contactos se ejecutan en paralelo con un maximo de concurrencia y un
`ROS_DOMAIN_ID` propio por corrida (`ejecucion`). Un contacto espera a los de
calibracion de su satelite que deben alimentar su efemeride (los demas corren
libres): el informe no depende del orden en que terminen las corridas.

Determinismo: con inicio fijo, la misma configuracion da el mismo `informe.json`
byte a byte (el plan lo es; cada corrida en lockstep lo es; la OD y el ruido del
Doppler usan semillas fijas; ningun tiempo de pared entra al informe).
"""
import concurrent.futures as cf
import datetime as dt
import json
import math
import os
import threading
import time
from dataclasses import asdict, dataclass, field

import numpy as np

from atp_ros import medidas as M
from atp_ros import model, od, orbit
from atp_ros import planificador as pl
from atp_ros import propagador as P

from atp_mision import contabilidad as C
from atp_mision import ejecucion as E
from atp_mision import mundo as W
from atp_mision import od_mision as O

UTC = dt.timezone.utc
ANTICIPO_S = 40.0            # la corrida arranca asi de antes del AOS planificado
BUSQUEDA_H = 0.5             # horas de busqueda del pase en la corrida


def _iso(t):
    return t.astimezone(UTC).strftime('%Y-%m-%dT%H:%M:%S')


def _r(x, nd=4):
    """Redondeo para el informe (None y NaN -> None)."""
    if x is None:
        return None
    x = float(x)
    return None if math.isnan(x) or math.isinf(x) else round(x, nd)


# ---------------------------------------------------------------- configuracion

@dataclass
class Config:
    """Parametros de una mision. Todo lo que cambia el resultado esta aqui y se guarda
    en `mision.json`."""
    sats: tuple = ('ISS', 'NOAA 15', 'NOAA 18', 'NOAA 19', 'NOAA 20', 'NOAA 21',
                   'METEOR-M2 3', 'METEOR-M2 4')
    estaciones: tuple = ('cdmx', 'tromso', 'ecuador', 'madrid')
    inicio: str = '2026-10-02T12:00:00'
    horas: float = 24.0
    n_od: int = 3                      # contactos de calibracion por satelite (0: sin OD)
    n_min_od: int = 3                  # pases de calibracion para usar la orbita estimada
    latencia_od_s: float = 600.0       # del LOS del ultimo pase de la OD a poder usarla
    el_cal_min_deg: float = 15.0       # elevacion util para calibrar
    el_cal_max_deg: float = 85.0       # cerca del cenit el acimut no sirve
    autotrack_bandas: tuple = ('X', 'Ka')
    sdr: str = 'cal'                   # cal | todos | ninguno (solo UHF y S)
    gemelos: str = 'programa'          # programa | todos | ninguno
    concurrencia: int = 3
    margen_acm_db: float = 1.0
    keyhole: str = 'marcar'
    el_min_deg: float = 0.0
    metodo: str = 'auto'
    prioridades: tuple = ()            # (('NOAA 19', 2.0), ...)
    plazo_s: float = 1800.0
    reintentos: int = 1
    semilla: int = 1
    reanudar: bool = False             # reutiliza las corridas ya hechas (no cambia el resultado)

    def a_dict(self):
        d = asdict(self)
        d.pop('reanudar')              # operativo: no entra al informe
        for k in ('sats', 'estaciones', 'autotrack_bandas'):
            d[k] = list(d[k])
        d['prioridades'] = [list(p) for p in d['prioridades']]
        return d


@dataclass
class Contacto:
    """Un contacto del plan y su papel en la mision."""
    id: int
    sat: str
    estacion: str
    aos: dt.datetime
    los: dt.datetime
    el_max_deg: float
    vol_gb_plan: float
    duracion_s: float
    keyhole: bool
    banda: str
    aos_trunc: bool = False            # el pase ya estaba en curso al inicio del horizonte
    los_trunc: bool = False            # el horizonte corta el pase
    rol: str = 'normal'                # cal | normal
    autotrack: bool = False
    sdr: bool = False
    deps: tuple = ()                   # contactos de calibracion que le pueden dar efemeride
    gemelo: bool = False               # tiene gemelo con TLE

    @property
    def nombre(self):
        return f'c{self.id:03d}'


# ---------------------------------------------------------------- la mision

class Mision:
    """Una mision. `ejecutor(args, carpeta) -> dict` corre una corrida (por defecto
    `EjecutorROS`); `plan` (opcional) reutiliza un plan ya calculado."""

    def __init__(self, cfg, carpeta, ejecutor=None, plan=None, log=None, cargas=None):
        self.cfg = cfg
        self.carpeta = carpeta
        self.log = log or (lambda *a: print(*a, flush=True))
        self.ejecutor = ejecutor or E.EjecutorROS(plazo_s=cfg.plazo_s)
        self.t0 = orbit.a_utc(cfg.inicio)
        self.t_fin = self.t0 + dt.timedelta(hours=cfg.horas)
        cat = orbit.cargar_catalogo()
        self.tles = [orbit.buscar_tle(cat, s) for s in cfg.sats]
        todas, _ = orbit.cargar_estaciones()
        self.estaciones = {k: todas[k] for k in cfg.estaciones}
        self.cargas = dict(W.CARGAS_M1)
        self.cargas.update(cargas or {})
        self.plan = plan
        self.mundo = None
        self.contactos = []
        self.res = {}              # clave -> resultado de la corrida
        self.medidas = {}          # id -> Medidas (solo calibracion)
        self.ods = {}              # clave -> info de la OD (cache)
        self._lock = threading.RLock()
        self._ods_lock = {}
        self.od_info = []
        self.sitios = {}           # sat -> lista de Sitio (compartida: Medidas.concatenar)
        self.relojes = {}
        for t in self.tles:
            self.relojes[t.nombre] = M.Reloj(t.epoca)
            self.sitios[t.nombre] = [M.sitio(e) for e in self.estaciones.values()]

    # ------------------------------------------------------------ 1. plan
    def planificar(self):
        """El plan optimo de la red con las cargas utiles de cada satelite."""
        c = self.cfg
        perfiles = {t.nombre: self.carga(t.nombre).perfil_enlace(c.margen_acm_db)
                    for t in self.tles}
        rotor = pl.Rotor(6.0, 3.0, 5.0, c.keyhole)
        prior = {n: float(v) for n, v in c.prioridades}
        defecto = pl.PerfilEnlace(margen_acm_db=c.margen_acm_db)
        vs = pl.construir_ventanas(self.tles, self.estaciones, self.t0, c.horas, defecto,
                                   rotor, perfiles, prior, c.el_min_deg)
        plan = pl.calcular_plan(self.tles, self.estaciones, self.t0, c.horas,
                                enlace=defecto, rotor=rotor, perfiles=perfiles,
                                prioridades=prior, el_min_deg=c.el_min_deg,
                                metodo=c.metodo, ventanas=vs)
        plan['estaciones_lista'] = list(self.estaciones)
        plan['bandas'] = {n: p.banda for n, p in perfiles.items()}
        self.plan = plan
        return plan

    def carga(self, sat):
        return self.cargas.get(sat, W.CARGA_DEFECTO)

    # ------------------------------------------------------------ 2. contactos y papeles
    def preparar(self):
        """Contactos del plan optimo con su papel (calibracion o normal), su modo de
        seguimiento y de que contactos de calibracion depende su efemeride."""
        c = self.cfg
        plan = self.plan
        ids = set(plan['planes']['optimo']['ids']) if 'optimo' in plan['planes'] \
            else set(plan['planes']['codicioso_peso']['ids'])
        cs = []
        for v in plan['ventanas']:
            if v['id'] not in ids:
                continue
            cs.append(Contacto(
                id=v['id'], sat=v['sat'], estacion=v['estacion'],
                aos=orbit.a_utc(v['aos']), los=orbit.a_utc(v['los']),
                el_max_deg=v['el_max_deg'], vol_gb_plan=v['vol_gb'],
                duracion_s=v['duracion_s'], keyhole=bool(v['keyhole']),
                banda=self.carga(v['sat']).banda,
                aos_trunc=bool(v['aos_truncado']), los_trunc=bool(v['los_truncado'])))
        cs.sort(key=lambda x: (x.aos, x.id))
        for sat in {x.sat for x in cs}:
            propios = [x for x in cs if x.sat == sat]
            utiles = [x for x in propios
                      if c.el_cal_min_deg <= x.el_max_deg <= c.el_cal_max_deg and not x.keyhole]
            cal = utiles[:c.n_od] if c.n_od > 0 else []
            for x in cal:
                x.rol = 'cal'
            for x in propios:
                x.autotrack = x.rol == 'cal' or x.banda in c.autotrack_bandas
                x.sdr = (x.banda in ('UHF', 'S')
                         and (c.sdr == 'todos' or (c.sdr == 'cal' and x.rol == 'cal')))
                x.deps = tuple(k.id for k in cal if k.id != x.id
                               and k.los + dt.timedelta(seconds=c.latencia_od_s) <= x.aos)
            for x in propios:
                # el gemelo solo se lanza si el contacto termina volando con la OD
                x.gemelo = bool(x.deps) and (
                    c.gemelos == 'todos' or (c.gemelos == 'programa' and not x.autotrack))
        self.contactos = cs
        return cs

    # ------------------------------------------------------------ 3. mundo
    def crear_mundo(self):
        self.mundo = W.Mundo(self.tles, self.estaciones, self.t_fin + dt.timedelta(hours=1),
                             self.cargas, os.path.join(self.carpeta, 'mundo'))
        return self.mundo

    # ------------------------------------------------------------ argumentos de una corrida
    def _args(self, c, perfil, od_file, carpeta):
        carga = self.carga(c.sat)
        return {
            'sat': c.sat, 'station': c.estacion,
            # un pase ya en curso al inicio del horizonte arranca en su AOS (= el inicio)
            'start_utc': _iso(c.aos if c.aos_trunc else c.aos - dt.timedelta(seconds=ANTICIPO_S)),
            'pass_index': 0, 'tle_search_hours': BUSQUEDA_H,
            'profile': perfil, 'od_file': od_file or '',
            'truth_file': self.mundo.archivo_verdad(c.sat),
            'band': carga.banda, 'eirp_dbw': carga.eirp_dbw,
            'autotrack': bool(c.autotrack),
            'max_corr_deg': round(model.ancho_haz(model.DIAMETRO_PLATO_M,
                                                  model.BANDAS[carga.banda]), 3),
            'sdr': bool(c.sdr), 'sdr_seed': 1,
            'time_scale': 0, 'record_dir': carpeta,
        }

    def _pase_corrida(self, c, args):
        """El pase que vuela la corrida (mismas llamadas que `reference_generator`):
        devuelve (AOS utc, duracion s)."""
        if args['profile'] == 'od':
            _, pase, _, _ = od.perfil_od(args['od_file'], c.estacion, 0, args['start_utc'],
                                         BUSQUEDA_H, None, 0.0)
        else:
            _, pase, _, _ = orbit.perfil_tle(c.sat, c.estacion, 0, args['start_utc'],
                                             BUSQUEDA_H, None, 0.0)
        return pase['aos'], float(pase['duracion_s'])

    # ------------------------------------------------------------ OD
    def _od(self, sat, usados):
        """Orbita estimada con los contactos de calibracion `usados` (ids, ordenados).
        Devuelve la ruta del JSON o None si la OD no se acepta. Cacheada."""
        clave = (sat, tuple(usados))
        lk = self._ods_lock.setdefault(clave, threading.Lock())
        with lk:
            if clave in self.ods:
                return self.ods[clave]['ruta']
            tle = self.tles[[t.nombre for t in self.tles].index(sat)]
            carga = self.carga(sat)
            lista = [self.medidas[i] for i in usados]
            modelo = P.Modelo(grado=2, cd_a_m=carga.bc * carga.p_cal)
            res, info = O.ajustar(tle, self.relojes[sat], lista, carga.banda, modelo)
            info.update({'sat': sat, 'contactos': list(usados)})
            if info['aceptada']:
                ruta = os.path.join(self.carpeta, 'od', f'{W.slug(sat)}_'
                                    f'{"-".join(str(i) for i in usados)}.json')
                os.makedirs(os.path.dirname(ruta), exist_ok=True)
                O.escribir(res, self.relojes[sat], tle, list(self.estaciones), ruta,
                           {'contactos': list(usados), 'canales': info['canales'],
                            'n_medidas': info['n_medidas']})
                info['ruta'] = ruta
            else:
                info['ruta'] = None
            self.ods[clave] = info
            self.log(f"OD {sat} con contactos {list(usados)}: "
                     + ('aceptada' if info['aceptada'] else 'RECHAZADA (' + info['motivo'] + ')')
                     + f", {info['n_medidas']} medidas {'+'.join(info['canales'])}, "
                     f"sigma pos {info['sigma_pos_km']:.3f} km, chi2/dof {info['chi2_dof']:.2f}")
            return info['ruta']

    def _efemeride_para(self, c):
        """(perfil, od_file, usados) con que se vuela `c`: la orbita estimada si hay
        `n_min_od` pases de calibracion con medidas validas y la OD se acepta; si no, TLE."""
        usados = sorted(i for i in c.deps if i in self.medidas)
        if len(usados) >= max(1, self.cfg.n_min_od):
            try:
                ruta = self._od(c.sat, usados)
            except Exception as e:      # noqa: BLE001  si la OD falla, el dia sigue con el TLE
                self.log(f'OD {c.sat} {usados}: fallo ({e!r}); se sigue con el TLE')
                ruta = None
            if ruta:
                return 'od', ruta, usados
        return 'tle', '', usados

    # ------------------------------------------------------------ una corrida
    def _correr(self, c, gemelo=False):
        """Ejecuta un contacto (o su gemelo con TLE) y deja su resultado en `self.res`."""
        clave = (c.id, 'tle' if gemelo else 'real')
        if gemelo:
            perfil, od_file, usados = 'tle', '', []
        else:
            perfil, od_file, usados = self._efemeride_para(c)
        nombre = c.nombre + ('_tle' if gemelo else '')
        carpeta = os.path.join(self.carpeta, 'contactos', nombre)
        args = self._args(c, perfil, od_file, os.path.abspath(carpeta))
        r = {'id': c.id, 'gemelo': gemelo, 'carpeta': os.path.join('contactos', nombre),
             'efemeride': perfil, 'od_file': od_file, 'od_usados': usados, 'incid': [],
             'infra': []}
        os.makedirs(carpeta, exist_ok=True)
        hecha = self.cfg.reanudar and self._hecha(carpeta, args)    # antes de reescribir params.json
        with open(os.path.join(carpeta, 'params.json'), 'w', encoding='utf-8') as f:
            json.dump({k: v for k, v in args.items()}, f, indent=1, sort_keys=True)
        t_ini = time.monotonic()
        out = {'estado': 'fallo'}
        if hecha:
            out = {'estado': 'ok', 'rc': 0}
            r['reanudada'] = True
        for intento in range(0 if r.get('reanudada') else 1 + max(0, self.cfg.reintentos)):
            for f in ('metrics.json', 'telemetry.jsonl'):
                try:
                    os.remove(os.path.join(carpeta, f))
                except OSError:
                    pass
            if intento:       # launch.log guarda solo el ultimo intento; el anterior se conserva aparte
                try:
                    os.replace(os.path.join(carpeta, 'launch.log'),
                               os.path.join(carpeta, f'launch.intento{intento}.log'))
                except OSError:
                    pass
            out = self.ejecutor(args, carpeta)
            r['dominio'] = out.get('dominio')
            if out.get('estado') == 'ok':
                break
            # fallos de la infraestructura (DDS que no empareja, plazo, carga): no entran al
            # informe (no son deterministas); van a ejecucion.json
            r['infra'].append(f"intento {intento + 1}: {out.get('estado')} (rc {out.get('rc')})")
        r['estado'] = out.get('estado', 'fallo')
        r['pared_s'] = round(time.monotonic() - t_ini, 1)
        if r['estado'] == 'ok':
            try:
                self._postprocesar(c, args, carpeta, r)
            except Exception as e:      # noqa: BLE001  un contacto no debe tumbar el dia
                r['estado'] = 'error'
                r['conta'] = None
                r['incid'].append(f'error al procesar la corrida: {e!r}')
        self.log(f"{nombre}: {c.sat} / {c.estacion} {_iso(c.aos)} {r['estado']} "
                 f"[{perfil}{'+autotrack' if c.autotrack else ''}{'+sdr' if c.sdr else ''}] "
                 + (f"{r['conta']['gb']:.3f} GB" if r.get('conta') else '') + f" ({r['pared_s']:.0f} s)")
        with self._lock:
            self.res[clave] = r
        return r

    @staticmethod
    def _hecha(carpeta, args):
        """La corrida de esta carpeta ya termino con ESTOS argumentos (la ejecucion es
        determinista: reutilizarla da el mismo resultado que repetirla)."""
        try:
            with open(os.path.join(carpeta, 'params.json'), encoding='utf-8') as f:
                previos = json.load(f)
        except (OSError, ValueError):
            return False
        return (previos == json.loads(json.dumps(args)) and os.path.exists(
            os.path.join(carpeta, 'metrics.json')) and os.path.exists(
            os.path.join(carpeta, 'telemetry.jsonl')))

    def _postprocesar(self, c, args, carpeta, r):
        filas = C.leer_telemetria(os.path.join(carpeta, 'telemetry.jsonl'))
        carga = self.carga(c.sat)
        # el dia termina en el horizonte: un pase que lo cruza solo cuenta hasta ahi
        # (la ventana del plan esta cortada igual); el resto de la corrida es margen
        cuenta = ([f for f in filas if f['t'] <= c.duracion_s + 0.5] if c.los_trunc else filas)
        r['conta'] = C.contabilizar(cuenta, carga.rs_baud, self.cfg.margen_acm_db)
        r['t_max_s'] = c.duracion_s + 0.5 if c.los_trunc else None
        try:
            with open(os.path.join(carpeta, 'metrics.json'), encoding='utf-8') as f:
                r['metrics'] = json.load(f)
        except (OSError, ValueError):
            r['metrics'] = None
        info, avisos = E.incidencias_de_log(os.path.join(carpeta, 'launch.log'))
        r['enganches'], r['perdidas_enganche'] = info['enganches'], info['perdidas_enganche']
        r['sdr_resumen'] = E.parse_resumen_sdr(info['sdr'])
        r['incid'] += avisos
        if r['conta'] is None:
            r['incid'].append('sin muestras en pase')
        aos, dur = self._pase_corrida(c, args)
        r['aos_corrida'], r['dur_corrida_s'] = aos, dur
        if c.rol == 'cal' and not r['gemelo'] and r['conta'] is not None \
                and not r['enganches']:
            r['incid'].append('calibracion sin medidas validas: el autotrack no engancho '
                              '(los angulos de la montura no miden al satelite)')
        elif (c.rol == 'cal' and not r['gemelo'] and r['conta'] is not None):
            rng = np.random.default_rng([self.cfg.semilla, c.id])
            isit = list(self.estaciones).index(c.estacion)
            med = O.medidas_de_telemetria(filas, self.relojes[c.sat], self.sitios[c.sat], isit,
                                          aos, carga.banda, c.sdr, rng)
            if med is None:
                r['incid'].append('calibracion sin medidas validas (sin enganche del autotrack)')
            else:
                with self._lock:
                    self.medidas[c.id] = med
                r['medidas_od'] = int(len(med))

    # ------------------------------------------------------------ 4. ejecucion
    def ejecutar(self):
        """Corre todos los contactos y sus gemelos en paralelo respetando las
        dependencias de calibracion."""
        cfg = self.cfg
        pendientes = [(c, False) for c in self.contactos]
        hechos = set()
        vuelo = {}
        n = max(1, cfg.concurrencia)
        t0 = time.monotonic()
        with cf.ThreadPoolExecutor(max_workers=n) as pool:
            while pendientes or vuelo:
                for tr in list(pendientes):
                    if len(vuelo) >= n:
                        break
                    c, gem = tr
                    if gem or all((d, 'real') in hechos for d in c.deps):
                        pendientes.remove(tr)
                        vuelo[pool.submit(self._correr, c, gem)] = tr
                if not vuelo:
                    raise RuntimeError('dependencias sin resolver: ' + str(pendientes[:3]))
                hecho, _ = cf.wait(vuelo, return_when=cf.FIRST_COMPLETED)
                for fut in hecho:
                    c, gem = vuelo.pop(fut)
                    r = fut.result()              # propaga una excepcion del orquestador
                    hechos.add((c.id, 'tle' if gem else 'real'))
                    # el gemelo (el mismo contacto con TLE) solo si volo con la OD
                    if not gem and c.gemelo and r['efemeride'] == 'od' and r['estado'] == 'ok':
                        pendientes.append((c, True))
        self.pared_s = time.monotonic() - t0

    def _escribir_ejecucion(self):
        """`ejecucion.json`: lo operativo de esta ejecucion (tiempo de pared, dominios,
        reintentos). NO es parte del informe: cambia de una ejecucion a otra."""
        filas = []
        for (cid, tipo), r in sorted(self.res.items()):
            filas.append({'contacto': f'c{cid:03d}' + ('_tle' if tipo == 'tle' else ''),
                          'estado': r['estado'], 'pared_s': r.get('pared_s'),
                          'dominio': r.get('dominio'), 'reanudada': bool(r.get('reanudada')),
                          'infra': r['infra']})
        with open(os.path.join(self.carpeta, 'ejecucion.json'), 'w') as f:
            json.dump({'pared_s': round(getattr(self, 'pared_s', 0.0), 1),
                       'concurrencia': self.cfg.concurrencia, 'corridas': filas}, f, indent=1)

    # ------------------------------------------------------------ 5. informe
    def _evaluar_efemeride(self, c, r):
        """Error de la efemeride que uso el contacto, contra la VERDAD (solo informe)."""
        if 'aos_corrida' not in r:
            return None
        ef = None
        if r['efemeride'] == 'od':
            ef = self.mundo.efemeride_orbita(c.sat, r['od_file'])
        e = self.mundo.error_efemeride(c.sat, c.estacion, r['aos_corrida'], r['dur_corrida_s'], ef)
        return {k: _r(e[k], 4) for k in ('aos', 'max', 'rms', 'p95')}

    def informe(self):
        cfg = self.cfg
        plan = self.plan
        optimo = plan['planes'].get('optimo') or plan['planes']['codicioso_peso']
        filas = []
        for c in self.contactos:
            r = self.res.get((c.id, 'real'))
            filas.append(self._fila(c, r))
        ejec = [f for f in filas if f['estado'] == 'ok' and f.get('gb') is not None]
        tot = {
            'contactos_planificados': len(self.contactos),
            'contactos_ejecutados': len(ejec),
            'contactos_fallidos': len(self.contactos) - len(ejec),
            'minutos_plan': _r(sum(c.duracion_s for c in self.contactos) / 60.0, 2),
            'minutos': _r(sum(f['minutos'] for f in ejec), 2),
            'gb_plan': _r(sum(c.vol_gb_plan for c in self.contactos), 3),
            'gb': _r(sum(f['gb'] for f in ejec), 3),
            'gb_sin_perdida_apunt': _r(sum(f['gb_sin_apunt'] for f in ejec), 3),
            'perdida_apunt_gb': _r(sum(f['perdida_apunt_gb'] for f in ejec), 3),
        }
        tot['gb_otras_diferencias'] = _r(
            sum(f['gb_sin_apunt'] for f in ejec) - sum(
                f['gb_plan'] for f in ejec), 3)
        pl_resumen = optimo['resumen']
        out = {
            'mision': {
                'inicio_utc': _iso(self.t0) + 'Z', 'horas': cfg.horas,
                'estaciones': list(self.estaciones), 'satelites': [t.nombre for t in self.tles],
                'config': cfg.a_dict(),
                'cargas': {t.nombre: asdict(self.carga(t.nombre)) for t in self.tles},
                'plan_algoritmo': optimo['algoritmo'], 'plan_optimo_probado': bool(
                    optimo.get('optimo_probado')),
            },
            'plan': {k: pl_resumen[k] for k in ('contactos', 'minutos', 'gb')},
            'totales': tot,
            'por_satelite': self._agrupar(filas, 'sat'),
            'por_estacion': self._agrupar(filas, 'estacion'),
            'contactos': filas,
            'od': self._resumen_od(),
            'mejora_od': self._mejora_od(filas),
            'incidencias': self._incidencias(filas),
        }
        out['verificacion'] = self._verificar(filas, tot)
        return out

    def _fila(self, c, r):
        f = {'id': c.id, 'contacto': c.nombre, 'sat': c.sat, 'estacion': c.estacion,
             'aos': _iso(c.aos) + 'Z', 'los': _iso(c.los) + 'Z', 'banda': c.banda,
             'el_max_deg': _r(c.el_max_deg, 2), 'rol': c.rol,
             'seguimiento': 'autotrack' if c.autotrack else 'program',
             'sdr': c.sdr, 'keyhole': c.keyhole,
             'minutos_plan': _r(c.duracion_s / 60.0, 2), 'gb_plan': _r(c.vol_gb_plan, 4)}
        if r is None:
            f['estado'] = 'no ejecutado'
            return f
        f['estado'] = r['estado']
        f['truncado_horizonte'] = bool(c.aos_trunc or c.los_trunc)
        f['t_max_s'] = r.get('t_max_s')
        f['efemeride'] = r['efemeride']
        f['od_contactos'] = r['od_usados'] if r['efemeride'] == 'od' else []
        f['carpeta'] = r['carpeta']
        f['incidencias'] = list(r['incid'])
        if r['estado'] != 'ok' or not r.get('conta'):
            return f
        k = r['conta']
        f.update({
            'minutos': _r(k['minutos'], 2), 'gb': _r(k['gb'], 4),
            'gb_sin_apunt': _r(k['gb_sin_apunt'], 4),
            'perdida_apunt_gb': _r(k['perdida_apunt_gb'], 4),
            'apunt_rms_deg': _r(k['apunt_rms_deg'], 4), 'apunt_p95_deg': _r(k['apunt_p95_deg'], 4),
            'apunt_max_deg': _r(k['apunt_max_deg'], 4),
            'l_point_max_db': _r(k['l_point_max_db'], 3),
            'cn0_min_dbhz': _r(k['cn0_min_dbhz'], 2), 'cn0_max_dbhz': _r(k['cn0_max_dbhz'], 2),
            'modcod_max': k['modcod_max'], 'enlace_cerrado_pct': _r(k['enlace_cerrado_pct'], 1),
            'enganches': r.get('enganches', 0), 'perdidas_enganche': r.get('perdidas_enganche', 0),
        })
        if r.get('sdr_resumen'):
            f['sdr_resumen'] = {a: _r(b, 3) for a, b in r['sdr_resumen'].items()}
        if 'medidas_od' in r:
            f['medidas_od'] = r['medidas_od']
        ev = self._evaluar_efemeride(c, r)
        if ev:
            f['err_efemeride_deg'] = ev
        return f

    @staticmethod
    def _agrupar(filas, clave):
        out = {}
        for f in filas:
            if f['estado'] != 'ok' or f.get('gb') is None:
                continue
            a = out.setdefault(f[clave], {'contactos': 0, 'minutos': 0.0, 'gb': 0.0,
                                          'gb_plan': 0.0, 'perdida_apunt_gb': 0.0})
            a['contactos'] += 1
            a['minutos'] += f['minutos']
            a['gb'] += f['gb']
            a['gb_plan'] += f['gb_plan']
            a['perdida_apunt_gb'] += f['perdida_apunt_gb']
        return {k: {a: (round(b, 4) if isinstance(b, float) else b) for a, b in v.items()}
                for k, v in sorted(out.items())}

    def _resumen_od(self):
        out = []
        for (sat, usados), info in sorted(self.ods.items()):
            e = {k: info[k] for k in ('sat', 'contactos', 'n_medidas', 'canales', 'aceptada',
                                      'motivo', 'epoca_utc')}
            for k, nd in (('chi2_dof', 3), ('sigma_pos_km', 4), ('desplazamiento_vs_tle_km', 3),
                          ('sigma_ang_deg', 4)):
                e[k] = _r(info[k], nd)
            # error REAL de posicion de la orbita estimada (contra la verdad), en la
            # epoca de cada contacto que la uso: solo informe
            if info.get('ruta'):
                ef = self.mundo.efemeride_orbita(sat, info['ruta'])
                t_ep = orbit.a_utc(info['epoca_utc'])
                e['error_pos_epoca_km'] = _r(self.mundo.error_posicion_km(sat, ef, t_ep), 3)
                e['error_pos_tle_epoca_km'] = _r(self.mundo.error_posicion_km(
                    sat, self.mundo.esc[sat].sgp4, t_ep), 3)
            out.append(e)
        return out

    def _mejora_od(self, filas):
        """Mejora por la OD: los contactos volados con orbita estimada contra su gemelo
        (el mismo contacto con TLE) y, para todos, el error de efemeride analitico."""
        por_id = {f['id']: f for f in filas}
        pares, od_efem = [], []
        for c in self.contactos:
            f = por_id[c.id]
            if f['estado'] != 'ok' or f.get('gb') is None or f['efemeride'] != 'od':
                continue
            g = self.res.get((c.id, 'tle'))
            e_od = f.get('err_efemeride_deg')
            e_tle = None
            if g is not None and g.get('conta'):
                ev = self._evaluar_efemeride(c, g)
                e_tle = ev
                pares.append({
                    'id': c.id, 'sat': c.sat, 'estacion': c.estacion,
                    'seguimiento': f['seguimiento'],
                    'gb_od': f['gb'], 'gb_tle': _r(g['conta']['gb'], 4),
                    'delta_gb': _r(f['gb'] - g['conta']['gb'], 4),
                    'apunt_rms_od_deg': f['apunt_rms_deg'],
                    'apunt_rms_tle_deg': _r(g['conta']['apunt_rms_deg'], 4),
                    'apunt_max_od_deg': f['apunt_max_deg'],
                    'apunt_max_tle_deg': _r(g['conta']['apunt_max_deg'], 4),
                    'err_efemeride_od_deg': e_od, 'err_efemeride_tle_deg': e_tle})
            if e_od:
                od_efem.append((c, e_od))
        out = {'contactos_con_od': len([f for f in filas if f.get('efemeride') == 'od']),
               'contactos_con_gemelo': len(pares), 'pares': pares}
        if pares:
            gb_od = sum(p['gb_od'] for p in pares)
            gb_tle = sum(p['gb_tle'] for p in pares)
            out['gb_con_od'] = _r(gb_od, 3)
            out['gb_con_tle'] = _r(gb_tle, 3)
            out['delta_gb'] = _r(gb_od - gb_tle, 3)
            out['delta_pct'] = _r(100.0 * (gb_od - gb_tle) / gb_tle, 2) if gb_tle > 0 else None
            for k in ('apunt_rms', 'apunt_max'):
                out[f'{k}_od_mediana_deg'] = _r(float(np.median(
                    [p[f'{k}_od_deg'] for p in pares])), 4)
                out[f'{k}_tle_mediana_deg'] = _r(float(np.median(
                    [p[f'{k}_tle_deg'] for p in pares])), 4)
            med = lambda xs: _r(float(np.median(xs)), 4) if xs else None      # noqa: E731
            out['err_efemeride_max_od_mediana_deg'] = med(
                [p['err_efemeride_od_deg']['max'] for p in pares if p['err_efemeride_od_deg']])
            out['err_efemeride_max_tle_mediana_deg'] = med(
                [p['err_efemeride_tle_deg']['max'] for p in pares if p['err_efemeride_tle_deg']])
        return out

    @staticmethod
    def _incidencias(filas):
        out = []
        for f in filas:
            if f['estado'] != 'ok':
                out.append({'contacto': f['contacto'], 'sat': f['sat'], 'tipo': 'no se ejecuto',
                            'detalle': f['estado']})
            for s in f.get('incidencias', []):
                out.append({'contacto': f['contacto'], 'sat': f['sat'], 'tipo': 'corrida',
                            'detalle': s})
            if f.get('perdidas_enganche'):
                out.append({'contacto': f['contacto'], 'sat': f['sat'],
                            'tipo': 'autotrack', 'detalle':
                            f"{f['perdidas_enganche']} perdidas de enganche"})
            if f['estado'] == 'ok' and f.get('autotrack_sin_enganche'):
                out.append({'contacto': f['contacto'], 'sat': f['sat'], 'tipo': 'autotrack',
                            'detalle': 'sin enganche'})
            if f['estado'] == 'ok' and f['seguimiento'] == 'autotrack' and not f.get('enganches'):
                out.append({'contacto': f['contacto'], 'sat': f['sat'], 'tipo': 'autotrack',
                            'detalle': 'el autotrack no engancho: program track'})
            if f['estado'] == 'ok' and f.get('enlace_cerrado_pct', 100.0) < 100.0:
                out.append({'contacto': f['contacto'], 'sat': f['sat'], 'tipo': 'enlace',
                            'detalle': f"enlace cerrado {f['enlace_cerrado_pct']} % del pase"})
        return out

    def _verificar(self, filas, tot):
        """Los totales deben ser la suma de las corridas: GB y minutos recalculados
        desde cero con `contabilizar_a_mano` sobre la telemetria de cada carpeta."""
        gb = mi = 0.0
        n = 0
        for f in filas:
            if f['estado'] != 'ok' or f.get('gb') is None:
                continue
            ruta = os.path.join(self.carpeta, f['carpeta'], 'telemetry.jsonl')
            filas_t = C.leer_telemetria(ruta)
            if f.get('t_max_s') is not None:
                filas_t = [x for x in filas_t if x['t'] <= f['t_max_s']]
            h = C.contabilizar_a_mano(filas_t, self.carga(f['sat']).rs_baud,
                                      self.cfg.margen_acm_db)
            if h is None:
                continue
            gb += h['gb']
            mi += h['minutos']
            n += 1
        return {'corridas_releidas': n,
                'gb_releido': _r(gb, 3), 'minutos_releidos': _r(mi, 2),
                'cuadra_gb': abs(round(gb, 3) - tot['gb']) <= 0.002 + 1e-4 * abs(tot['gb']),
                'cuadra_minutos': abs(round(mi, 2) - tot['minutos']) <= 0.05}

    # ------------------------------------------------------------ todo
    def correr(self):
        os.makedirs(self.carpeta, exist_ok=True)
        t_ini = time.monotonic()
        if self.plan is None:
            self.log('planificando...')
            self.planificar()
        with open(os.path.join(self.carpeta, 'plan.json'), 'w') as f:
            json.dump(self.plan, f, sort_keys=True)
        self.preparar()
        n_cal = sum(1 for c in self.contactos if c.rol == 'cal')
        self.log(f"plan: {len(self.contactos)} contactos ({n_cal} de calibracion, "
                 f"{sum(1 for c in self.contactos if c.gemelo)} gemelos con TLE)")
        self.crear_mundo()
        with open(os.path.join(self.carpeta, 'mision.json'), 'w') as f:
            json.dump(self.cfg.a_dict(), f, indent=1, sort_keys=True)
        self.ejecutar()
        self._escribir_ejecucion()
        inf = self.informe()
        with open(os.path.join(self.carpeta, 'informe.json'), 'w') as f:
            json.dump(inf, f, sort_keys=True, indent=1)
        from atp_mision import informe as I
        txt = I.texto(inf)
        with open(os.path.join(self.carpeta, 'informe.txt'), 'w') as f:
            f.write(txt)
        m = I.metricas(inf)
        with open(os.path.join(self.carpeta, 'metrics.json'), 'w') as f:
            json.dump(m, f, indent=1, sort_keys=True)
        self.log(txt)
        self.log(f'-> {self.carpeta}  ({time.monotonic() - t_ini:.0f} s de pared)')
        return inf
