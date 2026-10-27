"""Lectura de las series de una corrida y flujo SSE en vivo.

No hay sse-starlette en la imagen: el SSE es un StreamingResponse con
`text/event-stream` que hace tail de telemetry.jsonl y launch.log. Cada
conexion repite la corrida desde el principio (sample/log), asi que el
cliente se reconecta sin perder datos y sin Last-Event-ID.
"""
import asyncio
import csv
import io
import json
import time

from station.config import es_final

# columnas del CSV: las del CONTRATO 2, en ese orden
CSV_FIELDS = ['t', 'ref_az', 'ref_el', 'az', 'el', 'e_az', 'e_el', 'e_pt',
              'range_km', 'range_rate_km_s', 'doppler_hz', 'doppler_rate_hz_s',
              'l_point_db', 'cn0_dbhz', 'ebn0_db', 'margin_db', 'link_closed',
              'u_az', 'u_el']
# telemetria de una corrida ADCS (CONTRATO 2.1): los arreglos salen como `<campo>_<i>`
ADCS_CSV_FIELDS = (['t', 'mode', 'w_deg_s'] + [f'q_{i}' for i in range(4)]
                   + [f'q_est_{i}' for i in range(4)] + [f'h_nms_{i}' for i in range(3)]
                   + ['eclipse', 'el_deg', 'range_km', 'in_pass']
                   + [f'u_w_nm_{i}' for i in range(3)] + [f'm_am2_{i}' for i in range(3)]
                   + ['err_deg', 'know_deg', 'sigma_deg'])
LOG_TAIL_BYTES = 2_000_000
POLL_S = 0.25
PING_S = 15.0


def read_telemetry(run_dir, every=1):
    p = run_dir / 'telemetry.jsonl'
    if not p.exists():
        return []
    out = []
    with open(p, 'r', encoding='utf-8', errors='replace') as f:
        for i, line in enumerate(f):
            if i % every:
                continue
            try:
                out.append(json.loads(line))
            except ValueError:
                pass            # linea a medio escribir (corrida en curso)
    return out


def read_log(run_dir):
    p = run_dir / 'launch.log'
    if not p.exists():
        return ''
    size = p.stat().st_size
    with open(p, 'rb') as f:
        if size > LOG_TAIL_BYTES:
            f.seek(size - LOG_TAIL_BYTES)
        return f.read().decode('utf-8', errors='replace')


def _flat(row):
    out = {}
    for k, v in row.items():
        if isinstance(v, list):
            out.update({f'{k}_{i}': x for i, x in enumerate(v)})
        else:
            out[k] = v
    return out


def to_csv(samples):
    adcs = bool(samples) and 'mode' in samples[0]
    buf = io.StringIO()
    w = csv.DictWriter(buf, fieldnames=ADCS_CSV_FIELDS if adcs else CSV_FIELDS, extrasaction='ignore')
    w.writeheader()
    w.writerows([_flat(r) for r in samples] if adcs else samples)
    return buf.getvalue()


def sse(event, data):
    return f'event: {event}\ndata: {json.dumps(data, separators=(",", ":"))}\n\n'


class Tail:
    """Lee lineas nuevas de un archivo que crece, sin repetir ni partir lineas."""

    def __init__(self, path):
        self.path, self.off = path, 0

    def read(self, final=False):
        try:
            with open(self.path, 'rb') as f:
                f.seek(self.off)
                data = f.read()
        except FileNotFoundError:
            return []
        cut = data.rfind(b'\n') + 1
        if final and cut < len(data):      # cierre: la ultima linea sin '\n' tambien cuenta
            cut = len(data)
        self.off += cut
        text = data[:cut].decode('utf-8', errors='replace')
        return [ln for ln in text.splitlines() if ln.strip()]


async def event_stream(db, run_dir, run_id):
    tele, log = Tail(run_dir / 'telemetry.jsonl'), Tail(run_dir / 'launch.log')
    last_status, last_ping = None, time.monotonic()
    while True:
        # el estado se lee ANTES que los archivos: si ya es terminal, los
        # archivos estan completos y esta es la ultima pasada
        run = db.get_run(run_id)
        if run is None:                    # borrada mientras se escuchaba
            return
        status = run['status']
        final = es_final(status, run['finished_at'])
        out = []
        if status != last_status and not final:
            out.append(sse('status', {'status': status}))
            last_status = status
        for ln in tele.read(final):
            try:
                out.append(sse('sample', json.loads(ln)))
            except ValueError:
                pass
        for ln in log.read(final):
            out.append(sse('log', {'line': ln}))
        if final:
            out.append(sse('status', {'status': status}))
            out.append(sse('done', {'metrics': run['metrics']}))
        if out:
            yield ''.join(out)
            last_ping = time.monotonic()
        elif time.monotonic() - last_ping > PING_S:
            yield ': ping\n\n'
            last_ping = time.monotonic()
        if final:
            return
        await asyncio.sleep(POLL_S)


async def render_event_stream(db, job_id, salida):
    """SSE de un trabajo de render: `estado` ({estado, error, reintentos}), `log` ({line}),
    `progreso` ({progreso} 0-1, cuando el script imprime `PROGRESO <0-1>`) y, al terminar, `hecho`
    ({estado, error, media}). Como el de las corridas, repite el trabajo desde el principio en
    cada conexion (una reconexion no pierde datos)."""
    from station.renders import PROGRESO_RE
    log = Tail(salida / 'render.log')
    last, last_ping = None, time.monotonic()
    while True:
        # el estado se lee ANTES que el log: si ya es final, el log esta completo
        j = db.get_job(job_id)
        if j is None:                      # borrado mientras se escuchaba
            return
        estado = j['estado']
        final = es_final(estado, j['finished_at'])
        out = []
        if estado != last:
            out.append(sse('estado', {'estado': estado, 'error': j['error'],
                                      'reintentos': j['reintentos']}))
            last = estado
        for ln in log.read(final):
            out.append(sse('log', {'line': ln}))
            m = PROGRESO_RE.match(ln.strip())
            if m:
                out.append(sse('progreso', {'progreso': min(max(float(m.group(1)), 0.0), 1.0)}))
        if final:
            out.append(sse('hecho', {'estado': estado, 'error': j['error'], 'media': j['media']}))
        if out:
            yield ''.join(out)
            last_ping = time.monotonic()
        elif time.monotonic() - last_ping > PING_S:
            yield ': ping\n\n'
            last_ping = time.monotonic()
        if final:
            return
        await asyncio.sleep(POLL_S)
