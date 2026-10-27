#!/usr/bin/env python3
"""Exporta una corrida de la Estacion (runs/<id>/) como dataset para Manim.

Python puro (solo biblioteca estandar): corre en el host, en el contenedor ROS
y en el contenedor de Manim. Valida el contrato (docs/CONTRATO.md 1-4) y falla
con un mensaje claro si falta algo; no rellena ni inventa datos.

    python3 contenido/exportar.py demo-n2
    python3 contenido/exportar.py runs/20261002-101500-ab12cd --id mi-pase
    python3 contenido/exportar.py demo-n2 demo-n5a demo-n5b demo-campana

Salida: contenido/datasets/<id>.json, determinista (mismas corridas, mismos
bytes), compacto (series en columnas, submuestreadas) y versionable.

Submuestreo: la telemetria es de 10 Hz (5 400 muestras en un pase de 9 min).
Se reduce a `--max-points` conservando SIEMPRE el pico de error de cada
bloque (decimacion por maximo), de modo que ni el maximo ni las salidas de
la banda de 0.1 grados se pierden al dibujar. Cada punto lleva su propio `t`;
las escenas interpolan por tiempo y no suponen paso uniforme.

Las metricas (`metrics.json`) y la campana (`campaign.json`) se copian TAL
CUAL: son la fuente de toda cifra de resumen que se rotule en pantalla.
"""
import argparse
import json
import math
import os
import sys

ESQUEMA = "atp-contenido/1"

# Campos de cada muestra (CONTRATO 2). OBLIGATORIOS: sin ellos no hay escena.
OBLIGATORIOS = ("t", "ref_az", "ref_el", "az", "el", "e_pt", "range_km",
                "doppler_hz", "margin_db")
# Opcionales: se exportan solo si TODAS las filas utiles los traen.
OPCIONALES = ("e_az", "e_el", "range_rate_km_s", "doppler_rate_hz_s",
              "l_point_db", "cn0_dbhz", "ebn0_db", "u_az", "u_el")
# Decimales por campo (suficientes para 0.001 deg de error y 0.01 Hz)
DECIMALES = {"t": 3, "range_km": 3, "range_rate_km_s": 4, "doppler_hz": 2,
             "doppler_rate_hz_s": 2, "margin_db": 3, "cn0_dbhz": 3,
             "ebn0_db": 3, "l_point_db": 5, "u_az": 4, "u_el": 4}
DEC_DEFECTO = 5

METRICAS_PUNTO = ("rms", "p95", "max", "within_objective_pct", "objective_deg")
METRICAS_LINK = ("band", "margin_min_db", "margin_max_db", "closed_pct",
                 "doppler_max_hz", "doppler_min_hz", "doppler_rate_max_hz_s",
                 "l_point_max_db")
CAMPANA_CAMPOS = ("mode", "n", "p50", "p95", "p99", "peor", "pasa",
                  "objective_deg", "rms_deg")


class ContratoError(Exception):
    """La corrida no cumple el contrato: el mensaje dice que falta y donde."""


def _leer_json(ruta, que):
    try:
        with open(ruta, encoding="utf-8") as f:
            return json.load(f)
    except FileNotFoundError:
        raise ContratoError(f"falta {ruta} ({que})") from None
    except json.JSONDecodeError as e:
        raise ContratoError(f"{ruta} no es JSON valido ({e}): {que}") from None


def _numero(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) \
        and math.isfinite(v)


def _exigir_numeros(obj, campos, donde):
    if not isinstance(obj, dict):
        raise ContratoError(f"{donde}: se esperaba un objeto, llego {type(obj).__name__}")
    for c in campos:
        if c == "band":
            if not isinstance(obj.get(c), str) or not obj[c]:
                raise ContratoError(f"{donde}: falta el campo '{c}'")
        elif c not in obj:
            raise ContratoError(f"{donde}: falta el campo '{c}'")
        elif not _numero(obj[c]):
            raise ContratoError(f"{donde}.{c}: no es un numero finito ({obj[c]!r})")


def _resolver(arg, runs_dir):
    """Acepta una ruta a la corrida o su id dentro de `runs_dir`."""
    if os.path.isdir(arg):
        return os.path.normpath(arg)
    cand = os.path.join(runs_dir, arg)
    if os.path.isdir(cand):
        return os.path.normpath(cand)
    raise ContratoError(f"no existe la corrida '{arg}' (ni como ruta ni en {runs_dir}/)")


# ---------------------------------------------------------------------------
# telemetria
# ---------------------------------------------------------------------------
def _leer_telemetria(ruta):
    if not os.path.isfile(ruta):
        raise ContratoError(f"falta {ruta} (el grabador no escribio telemetria)")
    filas = []
    with open(ruta, encoding="utf-8") as f:
        for i, linea in enumerate(f, 1):
            linea = linea.strip()
            if not linea:
                continue
            try:
                fila = json.loads(linea)
            except json.JSONDecodeError as e:
                raise ContratoError(
                    f"{ruta}:{i}: linea JSON invalida ({e}); "
                    f"¿la corrida se corto a medio escribir?") from None
            if not isinstance(fila, dict):
                raise ContratoError(f"{ruta}:{i}: la muestra no es un objeto")
            filas.append(fila)
    if not filas:
        raise ContratoError(f"{ruta} esta vacio (0 muestras)")
    return filas


def _filas_utiles(filas, ruta):
    """Filas con todos los campos obligatorios; el resto (arranque, antes del
    primer dato de enlace o de montura) se descarta y se cuenta."""
    utiles, vistos = [], {c: 0 for c in OBLIGATORIOS}
    for fila in filas:
        for c in OBLIGATORIOS:
            if c in fila:
                vistos[c] += 1
        if all(c in fila and _numero(fila[c]) for c in OBLIGATORIOS):
            utiles.append(fila)
    if not utiles:
        ausentes = [c for c, n in vistos.items() if n == 0]
        raise ContratoError(
            f"{ruta}: ninguna muestra trae todos los campos obligatorios "
            f"{list(OBLIGATORIOS)}; nunca aparecen: {ausentes or 'ninguno (valores no numericos)'}")
    if len(utiles) < 0.9 * len(filas):
        raise ContratoError(
            f"{ruta}: solo {len(utiles)} de {len(filas)} muestras estan completas "
            f"(se exige >= 90 %); campos con huecos: "
            f"{[c for c, n in vistos.items() if n < len(filas)]}")
    # tiempo estrictamente creciente (el grabador no repite t)
    for a, b in zip(utiles, utiles[1:]):
        if b["t"] <= a["t"]:
            raise ContratoError(
                f"{ruta}: el tiempo no crece (t={a['t']} seguido de t={b['t']})")
    for fila in utiles:
        if not -1.0 <= fila["ref_el"] <= 91.0 or not -1.0 <= fila["el"] <= 91.0:
            raise ContratoError(f"{ruta}: elevacion fuera de [0, 90] grados en t={fila['t']}")
        if fila["e_pt"] < 0:
            raise ContratoError(f"{ruta}: e_pt negativo en t={fila['t']} (es una magnitud)")
    return utiles


def _decimar(filas, max_puntos):
    """Reduce a <= max_puntos conservando el maximo de e_pt de cada bloque.

    Siempre se queda la primera y la ultima muestra. Orden temporal intacto.
    """
    n = len(filas)
    if n <= max_puntos:
        return list(range(n))
    bloques = max_puntos - 2                   # + primera + ultima
    elegidos = {0, n - 1}
    interior = n - 2
    for b in range(bloques):
        ini = 1 + (b * interior) // bloques
        fin = 1 + ((b + 1) * interior) // bloques
        if fin <= ini:
            continue
        i = max(range(ini, fin), key=lambda k: filas[k]["e_pt"])
        elegidos.add(i)
    return sorted(elegidos)


def _series(filas, max_puntos, ruta):
    opc = [c for c in OPCIONALES
           if all(c in f and _numero(f[c]) for f in filas)]
    idx = _decimar(filas, max_puntos)
    cols = list(OBLIGATORIOS) + opc
    series = {}
    for c in cols:
        d = DECIMALES.get(c, DEC_DEFECTO)
        series[c] = [round(float(filas[i][c]), d) for i in idx]
    if all("link_closed" in f for f in filas):
        series["link_closed"] = [bool(filas[i]["link_closed"]) for i in idx]
    return series, len(idx)


# ---------------------------------------------------------------------------
# metricas y campana
# ---------------------------------------------------------------------------
def _validar_metricas(m, ruta):
    if not isinstance(m, dict):
        raise ContratoError(f"{ruta}: se esperaba un objeto")
    if m.get("verdict") == "SIN DATOS" or m.get("n", 0) == 0:
        raise ContratoError(f"{ruta}: la corrida termino SIN DATOS (n=0)")
    for c in ("n", "duration_s"):
        if not _numero(m.get(c)):
            raise ContratoError(f"{ruta}: falta '{c}' numerico")
    _exigir_numeros(m.get("pointing"), METRICAS_PUNTO, f"{ruta}: pointing")
    ax = m.get("axes")
    if not isinstance(ax, dict) or not all(
            isinstance(ax.get(e), dict) and _numero(ax[e].get("rms")) and _numero(ax[e].get("max"))
            for e in ("az", "el")):
        raise ContratoError(f"{ruta}: falta axes.az / axes.el con rms y max")
    if "link" not in m:
        raise ContratoError(f"{ruta}: falta el bloque 'link' (el enlace no publico /atp/link)")
    _exigir_numeros(m["link"], METRICAS_LINK, f"{ruta}: link")
    if m.get("verdict") not in ("CUMPLE", "NO CUMPLE"):
        raise ContratoError(f"{ruta}: verdict invalido ({m.get('verdict')!r})")


def _validar_campana(c, ruta):
    if not isinstance(c, dict):
        raise ContratoError(f"{ruta}: se esperaba un objeto")
    for k in CAMPANA_CAMPOS:
        if k not in c:
            raise ContratoError(f"{ruta}: falta el campo '{k}'")
    for k in ("n", "p50", "p95", "p99", "peor", "objective_deg"):
        if not _numero(c[k]):
            raise ContratoError(f"{ruta}.{k}: no es un numero finito ({c[k]!r})")
    r = c["rms_deg"]
    if not isinstance(r, list) or not r or not all(_numero(x) for x in r):
        raise ContratoError(f"{ruta}: rms_deg debe ser una lista no vacia de numeros")
    if len(r) != int(c["n"]):
        raise ContratoError(f"{ruta}: rms_deg trae {len(r)} valores y n={c['n']}")
    if "el_max_deg" in c and len(c["el_max_deg"]) != len(r):
        raise ContratoError(f"{ruta}: el_max_deg y rms_deg difieren de longitud")


# ---------------------------------------------------------------------------
def exportar(arg, runs_dir="runs", salida="contenido/datasets", max_puntos=400,
             ident=None):
    """Valida la corrida `arg` y escribe su dataset. Devuelve la ruta."""
    ruta = _resolver(arg, runs_dir)
    ident = ident or os.path.basename(ruta.rstrip(os.sep))
    tiene_tele = os.path.isfile(os.path.join(ruta, "telemetry.jsonl"))
    tiene_camp = os.path.isfile(os.path.join(ruta, "campaign.json"))
    if not tiene_tele and not tiene_camp:
        raise ContratoError(
            f"{ruta}/ no tiene telemetry.jsonl ni campaign.json: no es una corrida terminada")

    ds = {"esquema": ESQUEMA, "id": ident}
    p = os.path.join(ruta, "params.json")
    if os.path.isfile(p):
        ds["params"] = _leer_json(p, "parametros de la corrida")

    if tiene_tele:
        ds["tipo"] = "pase"
        mpath = os.path.join(ruta, "metrics.json")
        if not os.path.isfile(mpath):
            raise ContratoError(
                f"falta {mpath}: la corrida no termino (el grabador lo escribe al "
                f"recibir /atp/pass_done). ¿Se cancelo o hubo timeout? Revisa launch.log")
        metricas = _leer_json(mpath, "metricas del pase")
        _validar_metricas(metricas, mpath)
        tpath = os.path.join(ruta, "telemetry.jsonl")
        filas = _leer_telemetria(tpath)
        utiles = _filas_utiles(filas, tpath)
        series, n_out = _series(utiles, max_puntos, tpath)
        ds["metricas"] = metricas
        ds["fuente"] = {"muestras_telemetria": len(filas),
                        "muestras_completas": len(utiles),
                        "muestras_dataset": n_out,
                        "t_inicio_s": utiles[0]["t"], "t_fin_s": utiles[-1]["t"]}
        ds["series"] = series
    if tiene_camp:
        cpath = os.path.join(ruta, "campaign.json")
        camp = _leer_json(cpath, "resultado de la campana")
        _validar_campana(camp, cpath)
        ds["tipo"] = "pase+campana" if tiene_tele else "campana"
        ds["campana"] = camp

    os.makedirs(salida, exist_ok=True)
    destino = os.path.join(salida, f"{ident}.json")
    with open(destino, "w", encoding="utf-8") as f:
        json.dump(ds, f, ensure_ascii=False, separators=(",", ":"), sort_keys=True)
        f.write("\n")
    return destino, ds


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("corridas", nargs="+", help="id o ruta de runs/<id>")
    ap.add_argument("--runs-dir", default="runs", help="donde buscar ids (def. runs)")
    ap.add_argument("--out", default="contenido/datasets", help="carpeta de salida")
    ap.add_argument("--max-points", type=int, default=400,
                    help="muestras por serie tras submuestrear (def. 400, tope de traza_error)")
    ap.add_argument("--id", dest="ident", help="nombre del dataset (solo con una corrida)")
    a = ap.parse_args(argv)
    if a.ident and len(a.corridas) > 1:
        ap.error("--id solo vale con una corrida")
    if a.max_points < 10:
        ap.error("--max-points debe ser >= 10")
    codigo = 0
    for c in a.corridas:
        try:
            destino, ds = exportar(c, a.runs_dir, a.out, a.max_points, a.ident)
        except ContratoError as e:
            print(f"ERROR [{c}]: {e}", file=sys.stderr)
            codigo = 1
            continue
        kb = os.path.getsize(destino) / 1024.0
        fu = ds.get("fuente")
        det = (f"{fu['muestras_telemetria']} -> {fu['muestras_dataset']} muestras"
               if fu else f"campana n={ds['campana']['n']}")
        print(f"OK {destino}  ({ds['tipo']}, {det}, {kb:.0f} KB)")
    return codigo


if __name__ == "__main__":
    sys.exit(main())
