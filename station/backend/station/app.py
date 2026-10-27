"""API REST de la Estacion (CONTRATO 6 + docs/APP.md 5). Un solo worker uvicorn: la cola y los
procesos viven en memoria, asi que no se puede escalar a varios workers."""
import asyncio
import json
import re
from contextlib import asynccontextmanager
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional

from fastapi import APIRouter, FastAPI, HTTPException, Query, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import PlainTextResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from station import experiments, mcap, medios, renders, sistema, stream
from station.config import (CLASES, CONTACT_RE, EXPERIMENT_ID_RE, JOB_ID_RE, RENDER_ID_RE, RUN_ID_RE,
                            es_final, Settings)
from station.db import DB
from station.governor import Governor
from station.renders import RenderManager
from station.runner import Manager, default_command

SSE_HEADERS = {'Cache-Control': 'no-cache', 'X-Accel-Buffering': 'no'}


class RunRequest(BaseModel):
    experiment_id: str
    args: Optional[Dict[str, Any]] = None


class ReclamarRequest(BaseModel):
    trabajador: str
    clases: Optional[List[str]] = None


class LatidoRequest(BaseModel):
    progreso: Optional[float] = None
    log: Optional[List[str]] = None


class FinRequest(BaseModel):
    exit_code: int
    senal: Optional[Any] = None


TRABAJADOR_RE = re.compile(r'^[A-Za-z0-9_.@-]{1,64}$')
MAX_LINEAS_LATIDO, MAX_LINEA = 2000, 8000


class RenderRequest(BaseModel):
    render_id: str
    args: Optional[Dict[str, Any]] = None


def create_app(settings=None, command=default_command, load_reader=None, journal_reader=None):
    """`load_reader` (() -> carga de 1 min) y `journal_reader` (n -> dict|None) se inyectan en las
    pruebas para no depender de la maquina; por defecto se lee /proc/loadavg y journalctl."""
    s = settings or Settings()
    db = DB(s.db_path)
    gov = Governor(s, load_reader)
    mgr = Manager(s, db, command, gov)
    rmgr = RenderManager(s, db, mgr.run_dir, gov)
    vigilancia = sistema.Vigilancia(s.vigilante_lineas, journal_reader or sistema.leer_vigilante)

    @asynccontextmanager
    async def lifespan(_app):
        await mgr.start()            # marca como interrumpido lo que quedo de la ejecucion anterior
        await rmgr.start()
        yield
        await rmgr.stop()
        await mgr.stop()
        db.close()

    app = FastAPI(title='Estacion ATP', lifespan=lifespan)
    app.state.settings, app.state.db, app.state.manager = s, db, mgr
    app.state.renders, app.state.governor = rmgr, gov
    app.add_middleware(CORSMiddleware, allow_methods=['*'], allow_headers=['*'],
                       allow_origins=['http://localhost:5173', 'http://127.0.0.1:5173'])
    api = APIRouter(prefix='/api')

    # ---------------------------------------------------------------- ayudas
    def get_exp(exp_id):
        if not EXPERIMENT_ID_RE.match(exp_id):
            raise HTTPException(404, 'experimento no encontrado')
        exp = experiments.load_catalog(s.experiments_dir).get(exp_id)
        if exp is None:
            raise HTTPException(404, 'experimento no encontrado')
        return exp

    def get_run(run_id):
        # el formato estricto del id es la defensa contra path traversal
        if not RUN_ID_RE.match(run_id):
            raise HTTPException(404, 'corrida no encontrada')
        r = db.get_run(run_id)
        if r is None:
            raise HTTPException(404, 'corrida no encontrada')
        return r

    def get_job(job_id):
        if not JOB_ID_RE.match(job_id):
            raise HTTPException(404, 'trabajo de render no encontrado')
        j = db.get_job(job_id)
        if j is None:
            raise HTTPException(404, 'trabajo de render no encontrado')
        return j

    def read_json(path):
        try:
            return json.loads(path.read_text(encoding='utf-8'))
        except (OSError, ValueError):
            return None

    def summary(r):
        return {k: r[k] for k in ('run_id', 'experiment_id', 'title', 'status',
                                  'created_at', 'finished_at', 'args', 'metrics')}

    def job_view(j, detalle=False):
        d = {k: j[k] for k in ('job_id', 'run_id', 'render_id', 'title', 'estado', 'args', 'clase',
                               'pantalla', 'ros_domain_id', 'reintentos', 'exit_code', 'error',
                               'created_at', 'started_at', 'finished_at', 'trabajador')}
        d['ejecutor'] = (j['spec'] or {}).get('ejecutor') or 'backend'
        salida = rmgr.salida(j['run_id'], j['job_id'])
        if j['estado'] == 'hecho':
            d['progreso'] = 1.0
        elif j['estado'] == 'corriendo':
            d['progreso'] = renders.ultimo_progreso(salida / 'render.log')
        else:
            d['progreso'] = None
        d['miniatura'] = renders.miniatura_de(j['job_id'], j['media']) if j['media'] else None
        if detalle:
            d['media'] = j['media']
            d['duracion_estimada_s'] = (j['spec'] or {}).get('duracion_estimada_s')
        return d

    def artefactos(r):
        spec = r.get('spec') or {}
        cat = None if spec.get('artefactos') else experiments.load_catalog(s.experiments_dir)
        return renders.artefactos_de(r, mgr.run_dir(r['run_id']), cat)

    # ------------------------------------------------------------------ rutas
    @api.get('/health')
    def health():
        return {'ok': True, 'ros_distro': s.ros_distro or None,
                'active_runs': mgr.active_count(), 'active_renders': db.count_active_jobs()}

    @api.get('/experiments')
    def list_experiments():
        return list(experiments.load_catalog(s.experiments_dir).values())

    @api.get('/experiments/{exp_id}')
    def one_experiment(exp_id: str):
        return get_exp(exp_id)

    @api.post('/runs', status_code=201)
    async def create_run(req: RunRequest):
        exp = get_exp(req.experiment_id)
        try:
            args = experiments.validate_args(exp, req.args)
        except experiments.ArgError as ex:
            raise HTTPException(422, str(ex))
        return {'run_id': mgr.submit(exp, args), 'status': 'queued'}

    @api.get('/runs')
    def list_runs(experiment_id: Optional[str] = None,
                  limit: int = Query(50, ge=1, le=500)):
        if experiment_id is not None and not EXPERIMENT_ID_RE.match(experiment_id):
            return []
        return [summary(r) for r in db.list_runs(experiment_id, limit)]

    @api.get('/runs/{run_id}')
    def run_detail(run_id: str):
        r = get_run(run_id)
        d = mgr.run_dir(run_id)
        kind = experiments.kind_of_run(r)
        out = dict(summary(r), started_at=r['started_at'], ros_domain_id=r['ros_domain_id'],
                   exit_code=r['exit_code'], error=r['error'], timeout_s=r['timeout_s'],
                   launch=r['launch'], kind=kind, clase=r.get('clase'),
                   reintentos=r.get('reintentos') or 0)
        if out['metrics'] is None:
            out['metrics'] = (read_json(d / (r['spec'].get('metricas') or 'metrics.json'))
                              if kind == 'proceso' else read_json(d / 'metrics.json'))
        out['campaign'] = read_json(d / 'campaign.json')
        out['plan'] = read_json(d / 'plan.json')
        out['mision'] = (read_json(d / 'informe.json') if kind == 'mision' else None)
        out['artefactos'] = artefactos(r)
        return out

    @api.delete('/runs/{run_id}')
    def delete_run(run_id: str):
        r = get_run(run_id)
        if not es_final(r['status'], r['finished_at']):
            raise HTTPException(409, 'la corrida esta activa: cancelela primero')
        if any(not es_final(j['estado'], j['finished_at']) for j in db.list_jobs(run_id, None, 1000)):
            raise HTTPException(409, 'la corrida tiene renders activos: cancelelos primero')
        mgr.delete(run_id)
        return {'deleted': run_id}

    @api.post('/runs/{run_id}/cancel')
    async def cancel_run(run_id: str):
        get_run(run_id)
        if not await mgr.cancel(run_id):
            raise HTTPException(409, 'la corrida ya termino')
        return {'run_id': run_id, 'status': db.get_run(run_id)['status']}

    @api.get('/runs/{run_id}/telemetry')
    def telemetry(run_id: str, every: int = Query(1, ge=1, le=10000)):
        get_run(run_id)
        return stream.read_telemetry(mgr.run_dir(run_id), every)

    def contact_dir(run_id, contacto):
        # `kind: mision`: cada contacto es una corrida en runs/<id>/contactos/cNNN[_tle]/
        r = get_run(run_id)
        if experiments.kind_of_run(r) != 'mision' or not CONTACT_RE.match(contacto):
            raise HTTPException(404, 'contacto no encontrado')
        d = mgr.run_dir(run_id) / 'contactos' / contacto
        if not d.is_dir():
            raise HTTPException(404, 'contacto no encontrado')
        return d

    @api.get('/runs/{run_id}/contactos/{contacto}')
    def mission_contact(run_id: str, contacto: str):
        d = contact_dir(run_id, contacto)
        return {'contacto': contacto, 'params': read_json(d / 'params.json'),
                'metrics': read_json(d / 'metrics.json')}

    @api.get('/runs/{run_id}/contactos/{contacto}/telemetry')
    def mission_contact_telemetry(run_id: str, contacto: str,
                                  every: int = Query(1, ge=1, le=10000)):
        return stream.read_telemetry(contact_dir(run_id, contacto), every)

    @api.get('/runs/{run_id}/contactos/{contacto}/log', response_class=PlainTextResponse)
    def mission_contact_log(run_id: str, contacto: str):
        return stream.read_log(contact_dir(run_id, contacto))

    @api.get('/runs/{run_id}/log', response_class=PlainTextResponse)
    def run_log(run_id: str):
        get_run(run_id)
        return stream.read_log(mgr.run_dir(run_id))

    @api.get('/runs/{run_id}/export.csv')
    def export_csv(run_id: str):
        get_run(run_id)
        body = stream.to_csv(stream.read_telemetry(mgr.run_dir(run_id)))
        return Response(body, media_type='text/csv', headers={
            'Content-Disposition': f'attachment; filename="{run_id}.csv"'})

    @api.get('/runs/{run_id}/stream')
    def run_stream(run_id: str):
        get_run(run_id)
        return StreamingResponse(
            stream.event_stream(db, mgr.run_dir(run_id), run_id),
            media_type='text/event-stream', headers=SSE_HEADERS)

    # ------------------------------------------------- escenas (APP 5) y renders (APP 4)
    @api.get('/escenas')
    def escenas():
        """El catalogo de `/experiments` enriquecido: area, recursos, artefactos y
        la miniatura del ultimo medio de una corrida de la escena (null si no hay)."""
        minis = {}
        for j in db.list_jobs(estado='hecho', limit=500):       # mas nuevos primero
            run = db.get_run(j['run_id'])
            if run and run['experiment_id'] not in minis and j['media']:
                m = renders.miniatura_de(j['job_id'], j['media'])
                if m:
                    minis[run['experiment_id']] = m
        out = []
        for e in experiments.load_catalog(s.experiments_dir).values():
            out.append(dict(e, area=e.get('area'),
                            recursos=experiments.clase_de(e), artefactos=e.get('artefactos') or [],
                            miniatura=minis.get(e['id'])))
        return out

    @api.get('/renders/catalogo')
    def render_catalogo():
        return list(renders.load_render_catalog(s.render_dir).values())

    @api.post('/runs/{run_id}/renders', status_code=201)
    async def create_render(run_id: str, req: RenderRequest):
        run = get_run(run_id)
        if not RENDER_ID_RE.match(req.render_id):
            raise HTTPException(404, 'render no encontrado')
        render = renders.load_render_catalog(s.render_dir).get(req.render_id)
        if render is None:
            raise HTTPException(404, 'render no encontrado')
        try:
            args = experiments.validate_args(render, req.args)
        except experiments.ArgError as ex:
            raise HTTPException(422, str(ex))
        if run['status'] != 'done':
            raise HTTPException(409, f'la corrida no esta terminada (estado {run["status"]})')
        ok, motivo = renders.cumple_requiere(render['requiere'], run, mgr.run_dir(run_id),
                                             experiments.load_catalog(s.experiments_dir))
        if not ok:
            raise HTTPException(409, motivo)
        return {'job_id': rmgr.submit(render, run, args), 'estado': 'en_cola'}

    @api.get('/renders')
    def list_renders(run_id: Optional[str] = None, estado: Optional[str] = None,
                     limit: int = Query(50, ge=1, le=500)):
        if run_id is not None and not RUN_ID_RE.match(run_id):
            return []
        return [job_view(j) for j in db.list_jobs(run_id, estado, limit)]

    # ---- trabajador del host (APP 4.1): reclamar / latido / fin
    @api.post('/renders/reclamar')
    async def reclamar_render(req: ReclamarRequest):
        if not TRABAJADOR_RE.match(req.trabajador):
            raise HTTPException(422, 'trabajador invalido')
        if req.clases is not None and (not req.clases or any(c not in CLASES for c in req.clases)):
            raise HTTPException(422, f'clases debe ser una lista no vacia de {CLASES}')
        job = rmgr.reclamar(req.trabajador, req.clases)
        if job is None:
            return Response(status_code=204)
        return job

    def get_host_job(job_id):
        j = get_job(job_id)
        if (j['spec'] or {}).get('ejecutor') != 'host':
            raise HTTPException(409, 'el trabajo no es de ejecutor host')
        return j

    @api.post('/renders/{job_id}/latido')
    async def latido_render(job_id: str, req: LatidoRequest):
        get_host_job(job_id)
        if req.progreso is not None and not 0.0 <= req.progreso <= 1.0:
            raise HTTPException(422, 'progreso debe estar entre 0 y 1')
        if req.log is not None and (len(req.log) > MAX_LINEAS_LATIDO
                                    or any(len(x) > MAX_LINEA for x in req.log)):
            raise HTTPException(422, 'log demasiado grande')
        lineas = [x.replace('\r', '').replace('\n', ' ') for x in (req.log or [])]
        cancelar = rmgr.latido(job_id, req.progreso, lineas)
        return {'cancelar': cancelar, 'estado': db.get_job(job_id)['estado']}

    @api.post('/renders/{job_id}/fin')
    async def fin_render(job_id: str, req: FinRequest):
        get_host_job(job_id)
        paso, dato = rmgr.fin_host(job_id, req.exit_code, req.senal)
        if paso == 'validar':
            j = await asyncio.to_thread(rmgr.cerrar_host, job_id, req.exit_code, dato)
        else:
            j = db.get_job(job_id)
        return {'job_id': job_id, 'estado': j['estado'], 'error': j['error'], 'reintentos': j['reintentos']}

    @api.get('/renders/{job_id}')
    def render_detail(job_id: str):
        return job_view(get_job(job_id), detalle=True)

    @api.post('/renders/{job_id}/cancel')
    async def cancel_render(job_id: str):
        get_job(job_id)
        if not await rmgr.cancel(job_id):
            raise HTTPException(409, 'el trabajo ya termino')
        return {'job_id': job_id, 'estado': db.get_job(job_id)['estado']}

    @api.delete('/renders/{job_id}')
    def delete_render(job_id: str):
        j = get_job(job_id)
        if not es_final(j['estado'], j['finished_at']):
            raise HTTPException(409, 'el trabajo esta activo: cancelelo primero')
        rmgr.delete(job_id)
        return {'deleted': job_id}

    @api.get('/renders/{job_id}/stream')
    def render_stream(job_id: str):
        j = get_job(job_id)
        return StreamingResponse(
            stream.render_event_stream(db, job_id, rmgr.salida(j['run_id'], job_id)),
            media_type='text/event-stream', headers=SSE_HEADERS)

    @api.get('/medios')
    def list_medios(tipo: Optional[str] = None, escena: Optional[str] = None,
                    limit: int = Query(100, ge=1, le=1000)):
        if escena is not None and not EXPERIMENT_ID_RE.match(escena):
            return []
        return renders.galeria(db, tipo, escena, limit)

    @api.api_route('/medios/{job_id}/{ruta:path}', methods=['GET', 'HEAD'])
    def get_medio(job_id: str, ruta: str, request: Request, descargar: bool = False):
        j = get_job(job_id)
        p = medios.confinar(rmgr.salida(j['run_id'], job_id), ruta)
        return medios.servir(p, request.headers.get('range'), descargar, request.method)

    # ----------------------------------------------- artefactos declarados de una corrida (C4a)
    @api.api_route('/runs/{run_id}/artefactos/{ruta:path}', methods=['GET', 'HEAD'])
    def get_artefacto(run_id: str, ruta: str, request: Request, descargar: bool = False):
        """Sirve un artefacto DECLARADO por la escena (`artefactos` del YAML, p. ej. `topologia_isl.jsonl`, `rutas.jsonl`):
        el visor y la app leen asi los archivos de una corrida `proceso` sin que el backend conozca su formato. Solo lo que la
        escena declara y existe (el resto de la carpeta, p. ej. params.json, no se expone); con soporte de `Range`."""
        r = get_run(run_id)
        if ruta not in {a['ruta'] for a in artefactos(r) if a['existe']}:
            raise HTTPException(404, 'artefacto no encontrado')
        p = medios.confinar(mgr.run_dir(run_id), ruta)
        return medios.servir(p, request.headers.get('range'), descargar, request.method)

    # ----------------------------------------------- MCAP y sistema
    @api.get('/runs/{run_id}/export.mcap')
    def export_mcap(run_id: str):
        r = get_run(run_id)
        d = mgr.run_dir(run_id)
        if not mcap.archivos_disponibles(d):
            raise HTTPException(404, 'la corrida no tiene series (telemetry.jsonl / red.jsonl)')
        try:
            base = datetime.strptime(r['created_at'], '%Y-%m-%dT%H:%M:%SZ').replace(
                tzinfo=timezone.utc).timestamp()
        except ValueError:
            base = 0.0
        return StreamingResponse(mcap.generar(d, base), media_type='application/octet-stream',
                                 headers={'Content-Disposition': f'attachment; filename="{run_id}.mcap"'})

    @api.get('/sistema')
    def get_sistema():
        return sistema.snapshot(gov, db, vigilancia)

    app.include_router(api)
    # el frontend construido (equipo B) se sirve en / si existe
    if s.frontend_dist.is_dir():
        app.mount('/', StaticFiles(directory=str(s.frontend_dist), html=True),
                  name='frontend')
    return app
