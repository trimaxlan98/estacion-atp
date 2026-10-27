"""Gobernador de recursos (docs/APP.md 3): UNA autoridad decide que corre.

Corridas del catalogo y renders piden turno aqui. Cada tarea tiene una clase
(`ligero | pesado | gazebo | manim`) con un cupo y un umbral de carga (1 min del host):

* Si la clase ya tiene su cupo lleno (o se llego al tope global `max_concurrent`, o no queda
  pantalla Xephyr libre) la tarea espera en `en_cola`.
* Si hay cupo pero la carga del host supera el umbral de la clase, espera en
  `esperando_carga` y se reevalua cada `recheck_s` (10 s) o en cuanto otra tarea termina.
* Al admitirla recibe un `Lease` con (si lo pidio) un numero de pantalla del pool `:80-:83`
  (el script del render arranca el Xephyr; el gobernador solo reparte numeros).

El gobernador convive con `scripts/vigilante_host.sh` (que detiene los contenedores ROS si la
carga pasa de 12): no lo sustituye ni lo desactiva; sus umbrales (4-10) estan por debajo del suyo.
La lectura de carga es una funcion inyectable para probarlo sin depender de la maquina.
"""
import asyncio
import itertools
import os
import random
import time

from station.config import CLASES
from station.db import now_iso


def leer_carga():
    """Carga media de 1 min del host (en el contenedor con red host es la del host)."""
    try:
        return os.getloadavg()[0]
    except OSError:
        return 0.0


class Ticket:
    def __init__(self, clase, pantalla, on_state, label):
        self.clase, self.pantalla, self.on_state, self.label = clase, pantalla, on_state, label
        self.fut = asyncio.get_running_loop().create_future()
        self.estado = None              # 'cola' | 'carga': por que espera ahora mismo
        self.desde = time.monotonic()
        self.desde_utc = now_iso()


class Lease:
    """Turno concedido: se devuelve con `Governor.release`."""

    def __init__(self, gov, clase, pantalla, label):
        self.gov, self.clase, self.pantalla, self.label = gov, clase, pantalla, label
        self.released = False
        self.desde_utc = now_iso()


class Governor:
    def __init__(self, settings, load_reader=None):
        self.s = settings
        self.load_reader = load_reader or leer_carga
        self.corriendo = {c: 0 for c in CLASES}
        self.leases = []                       # concedidos y sin devolver
        self.waiting = []                      # Tickets en orden de llegada (FIFO)
        self.pantallas_libres = list(range(settings.pantalla_min, settings.pantalla_max + 1))
        self.dominios = set()                  # ROS_DOMAIN_ID en uso (pool 100-199)
        self._seq = itertools.count(1)

    # --------------------------------------------------------------- carga
    def carga(self):
        try:
            return float(self.load_reader())
        except Exception:  # noqa: BLE001  - sin lectura no se bloquea: el vigilante sigue ahi
            return 0.0

    def total(self):
        return sum(self.corriendo.values())

    # ------------------------------------------------------------ admision
    def _motivo(self, clase, pantalla, carga_fn):
        """None si se puede admitir; si no 'cola' (cupo/tope/pantalla) o 'carga'."""
        if self.total() >= self.s.max_concurrent:
            return 'cola'
        if self.corriendo[clase] >= self.s.cupos[clase]:
            return 'cola'
        if pantalla and not self.pantallas_libres:
            return 'cola'
        return 'carga' if carga_fn() >= self.s.umbrales[clase] else None

    def _pump(self):
        """Admite lo que se pueda, en orden de llegada, y avisa a cada tarea por que espera."""
        memo = []

        def carga_fn():
            if not memo:
                memo.append(self.carga())
            return memo[0]

        for t in list(self.waiting):
            motivo = self._motivo(t.clase, t.pantalla, carga_fn)
            if motivo is None:
                self.waiting.remove(t)
                lease = self._conceder(t.clase, t.pantalla, t.label)
                memo.clear()        # lo recien admitido cuenta: se vuelve a leer la carga
                if not t.fut.done():
                    t.fut.set_result(lease)
            elif motivo != t.estado:
                t.estado = motivo
                if t.on_state:
                    try:
                        t.on_state(motivo)
                    except Exception:  # noqa: BLE001
                        pass

    def _conceder(self, clase, pantalla, label):
        self.corriendo[clase] += 1
        pant = f':{self.pantallas_libres.pop(0)}' if pantalla else None
        lease = Lease(self, clase, pant, label)
        self.leases.append(lease)
        return lease

    def try_acquire(self, clase, pantalla=False, label=''):
        """Admision sin esperar (para el trabajador del host, que reclama trabajos: APP 4.1). Devuelve
        `(lease, None)` o `(None, 'cola'|'carga')`. Respeta el orden de llegada: si hay tareas del
        backend esperando en la misma clase, no se les adelanta."""
        if clase not in self.s.cupos:
            raise ValueError(f'clase desconocida: {clase}')
        if any(t.clase == clase for t in self.waiting):
            return None, 'cola'
        motivo = self._motivo(clase, pantalla, self.carga)
        if motivo is not None:
            return None, motivo
        return self._conceder(clase, pantalla, label), None

    async def acquire(self, clase, pantalla=False, on_state=None, label=''):
        """Espera el turno. `on_state('cola'|'carga')` se llama cuando cambia el motivo de la
        espera. Si la tarea se cancela mientras espera, no queda nada retenido."""
        if clase not in self.s.cupos:
            raise ValueError(f'clase desconocida: {clase}')
        t = Ticket(clase, pantalla, on_state, label)
        self.waiting.append(t)
        self._pump()
        try:
            while not t.fut.done():
                try:
                    await asyncio.wait_for(asyncio.shield(t.fut), self.s.recheck_s)
                except asyncio.TimeoutError:
                    self._pump()
            return t.fut.result()
        except asyncio.CancelledError:
            if t in self.waiting:
                self.waiting.remove(t)
            if t.fut.done() and not t.fut.cancelled() and t.fut.exception() is None:
                self.release(t.fut.result())          # ya concedido: se devuelve
            raise

    def release(self, lease):
        if lease is None or lease.released:
            return
        lease.released = True
        self.corriendo[lease.clase] -= 1
        if lease in self.leases:
            self.leases.remove(lease)
        if lease.pantalla:
            n = int(lease.pantalla[1:])
            if n not in self.pantallas_libres:
                self.pantallas_libres.append(n)
                self.pantallas_libres.sort()
        self._pump()

    # ---------------------------------------------------- ROS_DOMAIN_ID
    def alloc_domain(self):
        libres = [d for d in range(self.s.domain_min, self.s.domain_max + 1)
                  if d not in self.dominios]
        if not libres:
            raise RuntimeError('sin ROS_DOMAIN_ID libres en el pool')
        d = random.choice(libres)      # al azar: menos choques con procesos ajenos
        self.dominios.add(d)
        return d

    def free_domain(self, d):
        self.dominios.discard(d)

    # ----------------------------------------------------------- estado
    def snapshot(self):
        """Colas, cupos y pantallas para `GET /api/sistema`."""
        carga = self.carga()
        clases = {}
        for c in CLASES:
            esp = [t for t in self.waiting if t.clase == c]
            clases[c] = {'cupo': self.s.cupos[c], 'corriendo': self.corriendo[c],
                         'umbral_carga': self.s.umbrales[c],
                         'en_cola': sum(t.estado != 'carga' for t in esp),
                         'esperando_carga': sum(t.estado == 'carga' for t in esp),
                         'admite_ahora': (self.total() < self.s.max_concurrent
                                          and self.corriendo[c] < self.s.cupos[c]
                                          and carga < self.s.umbrales[c])}
        ahora = time.monotonic()
        return {
            'tope_global': self.s.max_concurrent,
            'corriendo_total': self.total(),
            'clases': clases,
            'corriendo': [{'clase': ls.clase, 'tarea': ls.label, 'pantalla': ls.pantalla,
                           'desde': ls.desde_utc} for ls in self.leases],
            'esperando': [{'clase': t.clase, 'tarea': t.label,
                           'motivo': 'carga' if t.estado == 'carga' else 'cola',
                           'espera_s': round(ahora - t.desde, 1), 'desde': t.desde_utc}
                          for t in self.waiting],
            'pantallas': {'pool': [f':{n}' for n in range(self.s.pantalla_min, self.s.pantalla_max + 1)],
                          'libres': [f':{n}' for n in self.pantallas_libres]},
            'dominios_ros_en_uso': sorted(self.dominios),
            'reintentos': self.s.reintentos,
            'recheck_s': self.s.recheck_s,
        }


def muerte_por_senal(rc):
    """True si el proceso murio por una senal que no mando la Estacion (OOM, el vigilante que
    detiene el contenedor, un `kill` ajeno): -N si lo mato la senal N; 128+N si lo reporta un
    envoltorio (bash `docker run` -> 137/143). Un error propio del programa (rc > 0) no cuenta."""
    return rc is not None and (rc < 0 or rc in (129, 131, 137, 143))
