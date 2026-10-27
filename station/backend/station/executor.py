"""Ciclo de vida comun de una tarea gobernada (corrida del catalogo o trabajo de render).

    pedir turno al gobernador  ->  lanzar el subprocess (grupo propio)  ->  esperar / matar
        ->  clasificar la salida  ->  (muerte por senal externa) interrumpida + reintento con
            espera creciente  |  estado final

`Manager` (corridas) y `RenderManager` (renders) heredan de `GovernedExecutor` y solo aportan los
ganchos que cambian: la fila, el comando, el estado final. Asi las dos comparten una sola
politica de espera, cancelacion, timeout y reintentos.
"""
import asyncio
import logging
import os
import signal

from station.governor import muerte_por_senal

log = logging.getLogger('station.executor')

ESPERA_MAX_S = 600.0


class GovernedExecutor:
    def __init__(self, settings, governor):
        self.s = settings
        self.gov = governor
        self.tasks = {}                 # id -> asyncio.Task
        self.procs = {}                 # id -> Process
        self.fase = {}                  # id -> 'espera' | 'corre' | 'espera_reintento'
        self.cancelled = set()          # ids cancelados por el usuario
        self.timed_out = set()

    # ------------------------------------------------------------------ ganchos
    def _get(self, tid):                                  # fila o None
        raise NotImplementedError

    def _clase(self, row):
        raise NotImplementedError

    def _quiere_pantalla(self, row):
        return False

    def _label(self, tid):
        return tid

    def _estado_espera(self, tid, motivo):                # 'cola' | 'carga'
        raise NotImplementedError

    def _arranca(self, tid, domain, pantalla):            # -> fila actualizada (estado corriendo)
        raise NotImplementedError

    def _lanzamiento(self, tid, row, domain, pantalla):   # -> (argv, env, cwd, log_path)
        raise NotImplementedError

    def _timeout(self, row):
        return row['timeout_s']

    def _resultado_ok(self, tid, rc):                     # True si el resultado existe (por archivos)
        return False

    def _final(self, tid, rc, senal):                     # estado final (done/error/interrumpida...)
        raise NotImplementedError

    def _fallo(self, tid, ex):                            # excepcion interna -> estado error
        raise NotImplementedError

    def _interrumpida(self, tid, intento, espera, rc):    # estado transitorio antes de reintentar
        raise NotImplementedError

    def _reintenta(self, tid, intento):                   # limpia lo parcial y vuelve a la cola
        raise NotImplementedError

    def _marcar_cancelada(self, tid):                     # cancelada antes de correr
        raise NotImplementedError

    # ------------------------------------------------------------ ciclo de vida
    def lanzar(self, tid):
        self.tasks[tid] = asyncio.create_task(self._ciclo(tid))

    async def _ciclo(self, tid):
        intento = 0
        try:
            while True:
                row = self._get(tid)
                if row is None:
                    return
                self.fase[tid] = 'espera'
                lease = await self.gov.acquire(
                    self._clase(row), self._quiere_pantalla(row),
                    on_state=lambda m, t=tid: self._estado_espera(t, m), label=self._label(tid))
                rc = None
                try:
                    self.fase[tid] = 'corre'
                    rc = await self._ejecutar(tid, lease)
                finally:
                    self.gov.release(lease)
                senal = (muerte_por_senal(rc) and tid not in self.cancelled
                         and tid not in self.timed_out and not self._resultado_ok(tid, rc))
                if senal and intento < self.s.reintentos:
                    intento += 1
                    espera = min(self.s.reintento_espera_s * 2 ** (intento - 1), ESPERA_MAX_S)
                    self._interrumpida(tid, intento, espera, rc)
                    self.fase[tid] = 'espera_reintento'
                    await asyncio.sleep(espera)
                    self._reintenta(tid, intento)
                    continue
                self._final(tid, rc, senal)
                return
        except asyncio.CancelledError:
            # cancelada mientras esperaba turno o la espera del reintento: la fila ya la marco
            # `cancel()`; si esta aqui por apagado, el arranque siguiente la marca interrumpida
            return
        except Exception as ex:  # noqa: BLE001
            log.exception('tarea %s fallo', tid)
            self._fallo(tid, ex)
        finally:
            self.procs.pop(tid, None)
            self.tasks.pop(tid, None)
            self.fase.pop(tid, None)
            self.cancelled.discard(tid)
            self.timed_out.discard(tid)

    async def _ejecutar(self, tid, lease):
        domain = self.gov.alloc_domain()
        try:
            self._arranca(tid, domain, lease.pantalla)
            row = self._get(tid)
            argv, env, cwd, logp = self._lanzamiento(tid, row, domain, lease.pantalla)
            with open(logp, 'ab') as logf:
                proc = await asyncio.create_subprocess_exec(
                    *argv, stdout=logf, stderr=asyncio.subprocess.STDOUT,
                    stdin=asyncio.subprocess.DEVNULL, env=env, cwd=cwd, start_new_session=True)
                self.procs[tid] = proc
                if tid in self.cancelled:        # cancelada mientras arrancaba
                    await self._kill_group(proc)
                try:
                    await asyncio.wait_for(proc.wait(), self._timeout(row))
                except asyncio.TimeoutError:
                    self.timed_out.add(tid)
                    await self._kill_group(proc)
                else:
                    # barrido: nodos que sobrevivan al lanzador no deben quedar vivos
                    try:
                        os.killpg(proc.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
            return proc.returncode
        finally:
            self.procs.pop(tid, None)
            self.gov.free_domain(domain)

    async def _kill_group(self, proc):
        """SIGINT al grupo (ros2 launch cierra limpio), SIGKILL si no sale, y al
        final un barrido SIGKILL por si quedaron nodos huerfanos."""
        try:
            os.killpg(proc.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            await asyncio.wait_for(proc.wait(), self.s.kill_grace_s)
        except asyncio.TimeoutError:
            pass
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        await proc.wait()

    async def cancelar(self, tid):
        """Cancela una tarea no final. True si se cancelo."""
        task = self.tasks.get(tid)
        fase = self.fase.get(tid)
        if fase == 'corre' or tid in self.procs:
            self.cancelled.add(tid)
            proc = self.procs.get(tid)
            if proc:
                await self._kill_group(proc)
            if task:
                try:
                    await asyncio.wait_for(asyncio.shield(task), self.s.kill_grace_s + 10)
                except asyncio.TimeoutError:
                    pass
            return True
        # esperando turno, esperando el reintento o aun sin tarea: no hay proceso que matar
        self._marcar_cancelada(tid)
        if task:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
        return True

    async def detener(self):
        for tid in list(self.procs):
            self.cancelled.add(tid)
            await self._kill_group(self.procs[tid])
        for tid, t in list(self.tasks.items()):
            if self.fase.get(tid) != 'corre':
                t.cancel()
        if self.tasks:
            await asyncio.gather(*self.tasks.values(), return_exceptions=True)
