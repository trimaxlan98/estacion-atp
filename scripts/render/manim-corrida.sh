#!/usr/bin/env bash
# manim-corrida: video de Manim (marca Co.De Aerospace) a partir de una corrida de la Estacion, con cifras
# MEDIDAS: el puente de contenido/ (exportar.py -> dataset -> escena -> contenido/render.sh).
#   scripts/render/manim-corrida.sh <run_dir> <salida> [<pantalla>] [--escena auto|pase|campana]
#                                   [--calidad l|m|h] [--play_s N] [--cpus 2]
#
# <run_dir>: corrida de lazo (telemetry.jsonl + metrics.json: escena `pase`, "El pase") o de campana
# (campaign.json: escena `campana`, "La campana"). `auto` elige por los archivos. Calidad: l 480p15,
# m 720p30 (por defecto), h 1080p60. La escena `autotrack` compara DOS corridas y no es de este render.
# Necesita docker, la imagen codeaerospace_contenido-manim y el repo hermano codeaerospace_contenido
# (CONTENIDO_REPO, ver contenido/README.md); sin pantalla (la pantalla se ignora). El dataset se exporta a una
# carpeta temporal del trabajo: contenido/datasets no se toca.
# Salida: <escena>.mp4 (principal), cifras.json (cada cifra rotulada y de que campo sale), miniatura.jpg, media.json.
ID=manim-corrida
source "$(dirname "${BASH_SOURCE[0]}")/_comun.sh"
comun_args "$@"

tipo=$(python3 scripts/render/ayuda.py tipo-corrida "$RUN")
defecto escena auto
defecto calidad m
defecto cpus 2
case ${A[escena]} in
  auto) case $tipo in lazo) A[escena]=pase ;; campana) A[escena]=campana ;;
          *) fallo "la corrida no cumple el requisito de $ID: debe ser un lazo (telemetry.jsonl + metrics.json) o una campana (campaign.json); esta es: $tipo ($RUN)" 2 ;; esac ;;
  pase) [ "$tipo" = lazo ] || fallo "la escena pase exige una corrida de lazo (telemetry.jsonl + metrics.json); esta es: $tipo ($RUN)" 2 ;;
  campana) [ "$tipo" = campana ] || fallo "la escena campana exige una corrida de campana (campaign.json); esta es: $tipo ($RUN)" 2 ;;
  *) fallo "escena desconocida '${A[escena]}' (pase | campana | auto)" 2 ;;
esac
case ${A[calidad]} in l) CARPETA=480p15 ;; m) CARPETA=720p30 ;; h) CARPETA=1080p60 ;;
  *) fallo "--calidad debe ser l, m o h" 2 ;; esac
case ${A[escena]} in pase) CLASE=ElPase; ENV_CORRIDA=ATP_PASE ;; campana) CLASE=LaCampana; ENV_CORRIDA=ATP_CAMPANA ;; esac
[ -n "${A[play_s]:-}" ] && { [[ ${A[play_s]} =~ ^[0-9]+(\.[0-9]+)?$ ]] || fallo "--play_s debe ser un numero" 2; }
REPO_CONT=${CONTENIDO_REPO:-$HOME/Documentos/code-projects/codeaerospace_contenido}
[ -d "$REPO_CONT" ] || fallo "no encuentro el repo de contenido ($REPO_CONT): defina CONTENIDO_REPO (ver contenido/README.md)" 2
docker image inspect "${IMAGEN:-codeaerospace_contenido-manim}" >/dev/null 2>&1 ||
  fallo "falta la imagen docker ${IMAGEN:-codeaerospace_contenido-manim} (se construye en el repo codeaerospace_contenido)" 2

trabajo_crear
DS_ID=$(printf '%s' "$(basename "$RUN")" | tr -c 'A-Za-z0-9_.-' '_')
mkdir -p "$TRABAJO/datasets" "$TRABAJO/media"
progreso 0.03
# 1. dataset compacto y validado (falla con un mensaje que dice que falta y donde)
python3 contenido/exportar.py "$RUN" --out "$TRABAJO/datasets" --id "$DS_ID" || fallo "la corrida no cumple el contrato de exportar.py (ver arriba)" 2
progreso 0.08

# 2. render; el avance se estima por tiempo (Manim no informa el total de animaciones)
( t0=$(date +%s); while sleep 5; do
    awk -v t=$(( $(date +%s) - t0 )) 'BEGIN{printf "PROGRESO %.2f\n", 0.1 + 0.8 * (1 - exp(-t / 90))}'
  done ) &
TICK=$!
esperar_carga 5
export "$ENV_CORRIDA=$DS_ID"
[ -n "${A[play_s]:-}" ] && export ATP_PLAY_S=${A[play_s]}
ejecutar env MEDIA_DIR="$TRABAJO/media" DATASETS_DIR="$TRABAJO/datasets" NOMBRE_CONTENEDOR="$NOMBRE" CPUS="${A[cpus]}" \
  bash contenido/render.sh -q "${A[calidad]}" "${A[escena]}"
rc=$?
kill "$TICK" 2>/dev/null; wait "$TICK" 2>/dev/null
VIDEO_ORIG="$TRABAJO/media/videos/${A[escena]}/$CARPETA/$CLASE.mp4"
[ $rc -eq 0 ] && [ -s "$VIDEO_ORIG" ] || fallo "Manim no genero $CLASE.mp4 (codigo $rc)" 1

mv "$VIDEO_ORIG" "$SALIDA/${A[escena]}.mp4"
PRINCIPAL="${A[escena]}.mp4"; MINI_DE=$PRINCIPAL
if [ -s "$TRABAJO/media/cifras/$CLASE.json" ]; then
  cp "$TRABAJO/media/cifras/$CLASE.json" "$SALIDA/cifras.json"; ARCHIVOS+=(cifras.json)
fi
NOTAS="escena $CLASE ($CARPETA) del dataset exportado de la corrida $(basename "$RUN"); cada cifra de resumen rotulada sale literal de metrics.json/campaign.json (registro en cifras.json)"
exit 0
