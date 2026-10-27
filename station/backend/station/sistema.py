"""`GET /api/sistema` (docs/APP.md 3 y 5): carga, memoria, colas, cupos y lo que hizo el vigilante.

El vigilante (`scripts/vigilante_host.sh`) corre en el HOST y registra con `logger -t atp-vigilante`.
Desde el contenedor `journalctl -t atp-vigilante` solo funciona si el diario del host esta montado;
si no se puede leer, `vigilante` es `null` (no es un error). Las lecturas son funciones
inyectables para probarlas sin depender de la maquina.
"""
import os
import re
import subprocess
import time

JOURNAL_CMD = ('journalctl', '-t', 'atp-vigilante', '--no-pager', '-o', 'short-iso', '-n')
LINEA_RE = re.compile(r'^(\d{4}-\d{2}-\d{2}T\S+)\s+\S+\s+atp-vigilante(?:\[\d+\])?:\s*(.*)$')
SIN_DIARIO = ('No journal files were found', 'no journal files', 'Permission denied',
              'Failed to', 'not seeing messages')


def leer_cargas():
    """(1 min, 5 min, 15 min) del host."""
    try:
        return tuple(round(x, 2) for x in os.getloadavg())
    except OSError:
        return (0.0, 0.0, 0.0)


def leer_memoria(path='/proc/meminfo'):
    """MiB de /proc/meminfo: total, disponible, usada (%) y swap. {} si no se puede leer."""
    campos = {}
    try:
        with open(path, encoding='ascii') as f:
            for ln in f:
                k, _, v = ln.partition(':')
                partes = v.split()
                if partes:
                    campos[k] = int(partes[0])      # kB
    except (OSError, ValueError):
        return {}
    total = campos.get('MemTotal', 0)
    disp = campos.get('MemAvailable', campos.get('MemFree', 0))
    swap_t, swap_l = campos.get('SwapTotal', 0), campos.get('SwapFree', 0)
    return {'total_mb': round(total / 1024), 'disponible_mb': round(disp / 1024),
            'usada_pct': round(100.0 * (total - disp) / total, 1) if total else None,
            'swap_total_mb': round(swap_t / 1024), 'swap_usada_mb': round((swap_t - swap_l) / 1024)}


def leer_vigilante(n=15, runner=subprocess.run):
    """Ultimas `n` lineas del vigilante, o None si el diario no se puede leer."""
    try:
        r = runner([*JOURNAL_CMD, str(int(n))], capture_output=True, text=True, timeout=3)
    except (OSError, subprocess.SubprocessError):
        return None
    if r.returncode != 0:
        return None
    salida, err = r.stdout or '', r.stderr or ''
    if any(m in salida or m in err for m in SIN_DIARIO[:2]):
        return None
    lineas = []
    for ln in salida.splitlines():
        if not ln.strip() or ln.startswith('-- '):
            continue
        m = LINEA_RE.match(ln)
        lineas.append({'t': m.group(1), 'texto': m.group(2)} if m else {'t': None, 'texto': ln.strip()})
    sobrecargas = [x for x in lineas if 'SOBRECARGA' in x['texto']]
    return {'lineas': lineas, 'ultima_sobrecarga': sobrecargas[-1] if sobrecargas else None}


class Vigilancia:
    """Lee el diario del vigilante con una cache corta (no lanzar journalctl en cada peticion)."""

    def __init__(self, n=15, reader=leer_vigilante, ttl_s=10.0):
        self.n, self.reader, self.ttl = n, reader, ttl_s
        self._t, self._val = -1e9, None

    def leer(self):
        if time.monotonic() - self._t > self.ttl:
            try:
                self._val = self.reader(self.n)
            except Exception:  # noqa: BLE001
                self._val = None
            self._t = time.monotonic()
        return self._val


def snapshot(gov, db, vigilancia, cargas=leer_cargas, memoria=leer_memoria):
    l1, l5, l15 = cargas()
    s = gov.snapshot()
    nproc = os.cpu_count() or 1
    return {
        'carga': {'l1': l1, 'l5': l5, 'l15': l15, 'hilos': nproc,
                  'umbral_vigilante_estimado': round(1.5 * nproc, 1),
                  'leida_por_gobernador': round(gov.carga(), 2)},
        'memoria': memoria(),
        'gobernador': s,
        'colas': {c: {'en_cola': v['en_cola'], 'esperando_carga': v['esperando_carga'],
                      'corriendo': v['corriendo']} for c, v in s['clases'].items()},
        'cupos': {c: {'cupo': v['cupo'], 'umbral_carga': v['umbral_carga'],
                      'admite_ahora': v['admite_ahora']} for c, v in s['clases'].items()},
        'pantallas': s['pantallas'],
        'activas': {'corridas': db.count_active(), 'renders': db.count_active_jobs()},
        'vigilante': vigilancia.leer(),
    }
