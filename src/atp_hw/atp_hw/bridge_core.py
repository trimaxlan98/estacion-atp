"""Nucleo del puente a rotctld: hilo con conexion, reconexion con backoff,
ordenes (banda muerta + tasa maxima) y lectura de posicion. Sin ROS, para
poder probarlo contra `fake_rotctld` en pytest.

Todo el E/S bloqueante vive en UN hilo de trabajo: un rotor lento o un
rotctld colgado no bloquea los callbacks de ROS ni la publicacion.

Estados:
  DISCONNECTED  sin conexion con rotctld (reintenta con backoff 0.5 s -> 5 s)
  WAIT_REF      conectado, sin referencia todavia
  PREPOSITION   mandando el punto de AOS; espera a que el rotor llegue
  TRACKING      siguiendo la referencia (el pase corre)
La referencia de reference_generator se congela en AOS hasta que alguien
publica /atp/torque: el nodo ROS lo hace al entrar en TRACKING.
"""
import collections
import threading
import time
from dataclasses import dataclass

from atp_hw import protocol as pr
from atp_hw import rotor_logic as rl
from atp_hw.rotctl_client import RotctldClient


@dataclass
class BridgeConfig:
    host: str = '127.0.0.1'
    port: int = 4533
    timeout_s: float = 1.0
    az_min: float = 0.0
    az_max: float = 360.0
    el_min: float = 0.0
    el_max: float = 90.0
    az_res_deg: float = 0.1       # banda muerta = resolucion del rotor
    el_res_deg: float = 0.1
    cmd_rate_hz: float = 5.0      # tasa maxima de `P`
    read_rate_hz: float = 10.0    # tasa de `p`
    backoff_min_s: float = 0.5
    backoff_max_s: float = 5.0
    ref_timeout_s: float = 2.0    # sin referencia -> `S` (stop)
    preposition: bool = True      # ir primero al punto de AOS y esperar
    ready_tol_deg: float = 0.5
    ready_hold_s: float = 1.0
    preposition_timeout_s: float = 180.0
    az_prefer: str = 'low'        # low | high | near (primera orden)
    allow_flip: bool = True
    query_limits: bool = True     # intersecar limites con \dump_state
    stop_on_exit: bool = True


class BridgeCore:
    def __init__(self, cfg, log=None):
        self.cfg = cfg
        self._log = log or (lambda lvl, msg: print(f'[{lvl}] {msg}', flush=True))
        self.client = RotctldClient(cfg.host, cfg.port, cfg.timeout_s)
        self.limiter = rl.RateLimiter(cfg.cmd_rate_hz)
        self.az_min, self.az_max = cfg.az_min, cfg.az_max
        self.el_min, self.el_max = cfg.el_min, cfg.el_max
        self.state = 'DISCONNECTED'
        self.halted = False
        self.last_cmd = None            # (az, el) ultima orden aceptada
        self.last_rotor = None          # (az, el) ultima lectura
        self.readings = collections.deque(maxlen=2000)   # (t_mono, az, el)
        self.stats = collections.Counter()
        self._ref = None                # (az_c, el, t_mono)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._halt_pending = False
        self._thread = None
        self._limits_failed = False
        self._ref_stopped = False
        self._prepos_t0 = None
        self._ok_since = None
        self._rprt_seen = collections.Counter()
        self._state_before_loss = None

    # ---- API para el nodo ROS (cualquier hilo) -----------------------------
    def log(self, lvl, msg):
        self._log(lvl, msg)

    def set_reference(self, az_c_deg, el_deg):
        with self._lock:
            self._ref = (float(az_c_deg), float(el_deg), time.monotonic())
        self.stats['refs_received'] += 1

    def pop_readings(self):
        out = []
        while True:
            try:
                out.append(self.readings.popleft())
            except IndexError:
                return out

    def set_halt(self, halt):
        """Parada de emergencia: envia `S` y deja de mandar posiciones hasta
        set_halt(False)."""
        halt = bool(halt)
        if halt and not self.halted:
            self._halt_pending = True
        if not halt and self.halted:
            self.last_cmd = None
            self.log('info', 'halt liberado: se reanudan las ordenes')
        self.halted = halt

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name='rotctld_bridge_io')
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=self.cfg.timeout_s * 3 + 1.0)

    def summary(self):
        s = self.stats
        return (f'estado={self.state} ordenes={s["cmds_sent"]} '
                f'rechazadas={s["rejected"]} banda_muerta={s["skipped_deadband"]} '
                f'refs={s["refs_received"]} lecturas={s["reads"]} '
                f'flips={s["flips"]} flips_bloqueados={s["flips_blocked"]} '
                f'el_recortada={s["el_clamped"]} az_recortado={s["az_clamped"]} '
                f'reconexiones={s["connects"] - 1 if s["connects"] else 0} '
                f'desconexiones={s["disconnects"]} timeouts_ref={s["ref_timeouts"]}')

    # ---- hilo de trabajo ---------------------------------------------------
    def _run(self):
        cfg = self.cfg
        backoff = cfg.backoff_min_s
        read_dt = 1.0 / cfg.read_rate_hz
        next_read = 0.0
        try:
            while not self._stop.is_set():
                if not self.client.connected:
                    if not self._connect():
                        self._stop.wait(backoff)
                        backoff = min(backoff * 2.0, cfg.backoff_max_s)
                        continue
                    backoff = cfg.backoff_min_s
                now = time.monotonic()
                try:
                    if self._halt_pending:
                        self._halt_pending = False
                        self.client.stop()
                        self.stats['halts'] += 1
                        self.log('warn', 'HALT: enviado S al rotor')
                    if now >= next_read:
                        self._do_read(now)
                        next_read = max(next_read + read_dt, now) \
                            if next_read else now + read_dt
                    if self.limiter.ready(now):
                        self._do_command(now)
                except pr.RotctldConnectionError as e:
                    self.stats['disconnects'] += 1
                    self._state_before_loss = self.state
                    self.state = 'DISCONNECTED'
                    self.log('warn', f'conexion con rotctld perdida: {e}')
                    self.client.close()
                    continue
                except pr.RotctldError as e:
                    self.stats['proto_errors'] += 1
                    self.log('warn', f'respuesta rara de rotctld: {e}')
                self._stop.wait(0.005)
        finally:
            if self.cfg.stop_on_exit and self.client.connected:
                try:
                    self.client.stop()
                    self.log('info', 'S enviado al salir')
                except pr.RotctldError:
                    pass
            self.client.close()

    def _connect(self):
        try:
            self.client.connect()
        except OSError as e:
            self.stats['connect_fail'] += 1
            if self.stats['connect_fail'] in (1, 10) or \
                    self.stats['connect_fail'] % 60 == 0:
                self.log('warn', f'sin rotctld en {self.cfg.host}:{self.cfg.port} '
                                 f'({e}); reintento con backoff')
            return False
        self.stats['connects'] += 1
        if self.cfg.query_limits and not self._limits_failed:
            try:
                lim = self.client.get_limits()
            except pr.RotctldConnectionError:
                # rotctld sin \dump_state: se cierra (resincroniza) y se
                # reconecta sin volver a preguntar.
                self._limits_failed = True
                self.log('info', 'rotctld no soporta \\dump_state; se usan los limites configurados')
                return False
            self._apply_limits(lim)
        self.last_cmd = None            # tras reconectar se vuelve a mandar
        # tras una caida en pleno pase se sigue en TRACKING; en cualquier
        # otro caso se vuelve a empezar (WAIT_REF -> PREPOSITION)
        self.state = 'TRACKING' if self._state_before_loss == 'TRACKING' else 'WAIT_REF'
        self.log('info', f'conectado a rotctld {self.cfg.host}:{self.cfg.port}')
        return True

    def _apply_limits(self, lim):
        cfg = self.cfg
        # un backend que declara max <= min (rango sin informar) se ignora
        if not lim or lim.get('max_az', 1) <= lim.get('min_az', 0) or \
                lim.get('max_el', 1) <= lim.get('min_el', 0):
            return
        self.az_min = max(cfg.az_min, lim.get('min_az', cfg.az_min))
        self.az_max = min(cfg.az_max, lim.get('max_az', cfg.az_max))
        self.el_min = max(cfg.el_min, lim.get('min_el', cfg.el_min))
        self.el_max = min(cfg.el_max, lim.get('max_el', cfg.el_max))
        if (self.az_min, self.az_max, self.el_min, self.el_max) != \
                (cfg.az_min, cfg.az_max, cfg.el_min, cfg.el_max):
            self.log('warn', f'limites recortados por el rotor: az '
                             f'[{self.az_min:g},{self.az_max:g}] el '
                             f'[{self.el_min:g},{self.el_max:g}]')

    def _do_read(self, now):
        az, el = self.client.get_position()
        self.stats['reads'] += 1
        self.last_rotor = (az, el)
        self.readings.append((now, az, el))
        self._update_state(now)

    def _update_state(self, now):
        if self.state != 'PREPOSITION' or self.last_cmd is None:
            return
        cfg = self.cfg
        tol_az = max(cfg.ready_tol_deg, cfg.az_res_deg)
        tol_el = max(cfg.ready_tol_deg, cfg.el_res_deg)
        az, el = self.last_rotor
        if abs(az - self.last_cmd[0]) <= tol_az and abs(el - self.last_cmd[1]) <= tol_el:
            if self._ok_since is None:
                self._ok_since = now
            if now - self._ok_since >= cfg.ready_hold_s:
                self._enter_tracking(f'rotor en AOS (az={az:.2f} el={el:.2f}) tras '
                                     f'{now - self._prepos_t0:.1f} s')
                return
        else:
            self._ok_since = None
        if now - self._prepos_t0 > cfg.preposition_timeout_s:
            self.stats['preposition_timeout'] += 1
            self._enter_tracking('TIMEOUT de preposicionamiento: el rotor no llego '
                                 'al punto de AOS; se sigue de todos modos', 'error')

    def _enter_tracking(self, why, lvl='info'):
        self.state = 'TRACKING'
        self.stats['tracking_started'] += 1
        self.log(lvl, f'TRACKING: {why}')

    def _do_command(self, now):
        cfg = self.cfg
        if self.halted:
            return
        with self._lock:
            ref = self._ref
        if ref is None:
            return
        az_c, el, t_ref = ref
        if now - t_ref > cfg.ref_timeout_s:
            if not self._ref_stopped:
                self._ref_stopped = True
                self.stats['ref_timeouts'] += 1
                self.last_cmd = None
                self.client.stop()
                self.log('warn', f'sin referencia hace > {cfg.ref_timeout_s:g} s '
                                 f'(fin de pase?): S enviado al rotor')
            return
        self._ref_stopped = False
        if self.last_cmd is not None:
            anchor = self.last_cmd[0]
        elif cfg.az_prefer == 'near' and self.last_rotor is not None:
            anchor = self.last_rotor[0]
        else:
            anchor = None
        az_cmd, st = rl.map_azimuth(az_c, anchor, self.az_min, self.az_max,
                                    cfg.az_prefer)
        if st == 'clamped':
            self.stats['az_clamped'] += 1
        el_cmd, clipped = rl.clamp_elevation(el, self.el_min, self.el_max)
        if clipped:
            self.stats['el_clamped'] += 1
        cmd = (az_cmd, el_cmd)
        if not rl.needs_update(cmd, self.last_cmd, cfg.az_res_deg, cfg.el_res_deg):
            self.stats['skipped_deadband'] += 1
            return
        if rl.is_flip(az_cmd, self.last_cmd[0] if self.last_cmd else None):
            if not cfg.allow_flip:
                self.stats['flips_blocked'] += 1
                self.limiter.mark(now)
                return
            self.stats['flips'] += 1
            self.log('warn', f'FLIP de acimut: {self.last_cmd[0]:.1f} -> {az_cmd:.1f} '
                             f'deg; el rotor desenrosca y se pierde el satelite')
        self.limiter.mark(now)
        try:
            self.client.set_position(az_cmd, el_cmd)
        except pr.RotctldRPRT as e:
            self.stats['rejected'] += 1
            self._rprt_seen[e.code] += 1
            if self._rprt_seen[e.code] in (1, 20, 200):
                self.log('warn', f'rotctld rechazo P {az_cmd:.2f} {el_cmd:.2f}: {e} '
                                 f'(x{self._rprt_seen[e.code]})')
            return
        self.last_cmd = cmd
        self.stats['cmds_sent'] += 1
        if self.state == 'WAIT_REF':
            self._prepos_t0 = now
            if cfg.preposition:
                self.state = 'PREPOSITION'
                self.log('info', f'preposicion: P {az_cmd:.2f} {el_cmd:.2f}')
            else:
                self._enter_tracking('sin preposicion')
