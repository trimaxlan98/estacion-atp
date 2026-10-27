#!/usr/bin/env python3
"""Comprueba que las cifras de resumen rotuladas en pantalla salen de la corrida.

Cada escena vuelca en contenido/media/cifras/<Escena>.json las cifras de
resumen que dibujo (campo del dataset, valor, formato y texto exacto). Aqui
se contrasta, por cada una:

  1. el texto dibujado == formato(valor del dataset * escala)
  2. el valor del dataset == el valor en runs/<id>/metrics.json (o campaign.json)
     leido de nuevo del disco, o sea la fuente original, sin pasar por exportar.py
  3. el dataset completo de metricas es identico al archivo de la corrida

Las cifras EN VIVO (el, az, rango, Doppler, margen, error en cada instante) no
pasan por este registro: salen de las series del dataset interpoladas al
tiempo del cuadro; se comprueban con `fotogramas.sh` + la telemetria (ver
contenido/README.md).

    python3 contenido/verificar.py            # todas las escenas renderizadas
    python3 contenido/verificar.py LaCampana
    python3 contenido/verificar.py --t 14.0 ElPase     # que deberia verse en el segundo 14
Sale con codigo 1 si algo no coincide.

Con `--t` (segundo del VIDEO) se reconstruye el instante del pase que hay en
ese cuadro y se imprimen las cifras en vivo leidas de runs/<id>/telemetry.jsonl
(la fuente original, a 10 Hz, no el dataset submuestreado): se comparan a ojo
con el fotograma de `fotogramas.sh`. Tolerancia: el dataset conserva el pico
de error de cada bloque, asi que ERROR puede diferir de la muestra exacta;
el resto (el, az, rango, Doppler, margen) coincide a la cifra rotulada.
"""
import json
import os
import sys

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(AQUI)


def _fuente(corrida_id, campo):
    """Valor de `campo` leido de runs/<id>/ (la fuente original)."""
    raiz = campo.split(".")[0]
    archivo = "campaign.json" if raiz == "campana" else "metrics.json"
    ruta = os.path.join(RAIZ, "runs", corrida_id, archivo)
    with open(ruta, encoding="utf-8") as f:
        v = json.load(f)
    claves = campo.split(".")[1:] if raiz == "campana" else campo.split(".")
    for k in claves:
        v = v[k]
    return v, ruta


def en_vivo(carpeta, nombre, seg):
    with open(os.path.join(carpeta, f"{nombre}.json"), encoding="utf-8") as f:
        reg = json.load(f)
    for v in reg.get("vivo", []):
        a = min(max((seg - v["inicio_video_s"]) / v["dur_s"], 0.0), 1.0)
        t = v["t0"] + a * (v["t1"] - v["t0"])
        with open(os.path.join(RAIZ, "runs", v["dataset"], "telemetry.jsonl"),
                  encoding="utf-8") as f:
            filas = [json.loads(x) for x in f if x.strip()]
        filas = [r for r in filas if all(k in r for k in ("az", "el", "e_pt", "margin_db"))]
        r = min(filas, key=lambda r: abs(r["t"] - t))
        print(f"== {nombre} video t={seg:g} s -> {v['dataset']} t_pase={t:.2f} s "
              f"(muestra de telemetria t={r['t']:.2f})")
        print(f"   T+ {t:.1f} S | EL {r['el']:.2f} | AZ {r['az']:.2f} | RANGO {r['range_km']:.1f} KM"
              f" | DOPPLER {r['doppler_hz'] / 1000:+.2f} KHZ | MARGEN {r['margin_db']:.1f} DB"
              f" | ERROR {r['e_pt']:.4f} DEG")
    return 0


def main(argv):
    if argv and argv[0] == "--t":
        carpeta = os.path.join(AQUI, "media", "cifras")
        return en_vivo(carpeta, argv[2], float(argv[1]))
    carpeta = os.path.join(AQUI, "media", "cifras")
    if not os.path.isdir(carpeta):
        print("no hay media/cifras: renderiza primero (contenido/render.sh)", file=sys.stderr)
        return 2
    nombres = argv or sorted(n[:-5] for n in os.listdir(carpeta) if n.endswith(".json"))
    fallos = total = 0
    for nombre in nombres:
        with open(os.path.join(carpeta, f"{nombre}.json"), encoding="utf-8") as f:
            reg = json.load(f)
        print(f"== {reg['escena']}")
        for it in reg["cifras"]:
            total += 1
            with open(os.path.join(AQUI, "datasets", f"{it['dataset']}.json"),
                      encoding="utf-8") as f:
                ds = json.load(f)
            raiz = it["campo"].split(".")[0]
            v = ds if raiz == "campana" else ds["metricas"]
            for k in it["campo"].split("."):
                v = v[k]
            esperado = it["formato"].format(v * it["escala"])
            origen, ruta = _fuente(it["dataset"], it["campo"])
            ok_texto = esperado == it["texto"]
            ok_fuente = origen == v
            ok = ok_texto and ok_fuente
            fallos += not ok
            print(f"  {'OK ' if ok else 'MAL'} {it['dataset']:<13} {it['campo']:<32} "
                  f"pantalla={it['texto']:<9} runs/{os.path.relpath(ruta, os.path.join(RAIZ, 'runs'))}="
                  f"{origen!r}" + ("" if ok else f"   (esperado {esperado})"))
    # el bloque completo de metricas del dataset es el de la corrida
    for d in sorted({i["dataset"] for n in nombres for i in
                     json.load(open(os.path.join(carpeta, f"{n}.json")))["cifras"]}):
        with open(os.path.join(AQUI, "datasets", f"{d}.json"), encoding="utf-8") as f:
            ds = json.load(f)
        for clave, archivo in (("metricas", "metrics.json"), ("campana", "campaign.json")):
            if clave in ds:
                with open(os.path.join(RAIZ, "runs", d, archivo), encoding="utf-8") as f:
                    igual = json.load(f) == ds[clave]
                fallos += not igual
                print(f"  {'OK ' if igual else 'MAL'} {d}: bloque '{clave}' del dataset == runs/{d}/{archivo}")
    print(f"{total} cifras de resumen, {fallos} discrepancias")
    return 1 if fallos else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
