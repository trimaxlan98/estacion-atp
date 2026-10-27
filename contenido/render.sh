#!/usr/bin/env bash
# Renderiza las escenas de la Estacion con la imagen de Manim del motor de
# contenido (codeaerospace_contenido-manim: Manim + LaTeX + ffmpeg).
#
#   contenido/render.sh [-q l|m|h] [-m] escena...        escenas: pase autotrack campana todo
#   contenido/render.sh -q l pase autotrack
#   contenido/render.sh -q m -m campana                  (-m: guarda la miniatura en contenido/muestras/)
#
# Montajes (el repo de contenido y los datasets van en SOLO LECTURA; no se copia
# ninguna libreria):
#   <repo de contenido>   -> /workspace   (ro)  librerias: studio/content/manim_extensions
#   contenido/            -> /atp         (ro)  escenas/ y datasets/
#   contenido/media/      -> /media       (rw)  unica salida; ignorada en git
#
# El contenedor corre sin red, con el sistema de archivos en solo lectura (salvo
# /tmp y /media) y como tu usuario, para que media/ no quede de root.
#
# Variables utiles (se pasan a las escenas): ATP_PASE, ATP_PLAY_S, ATP_PROGRAM,
# ATP_AUTOTRACK, ATP_CAMPANA. CONTENIDO_REPO cambia la ruta del repo hermano.
#
# Uso no interactivo (lo lanza scripts/render/manim-corrida.sh; sin estas variables todo es como antes):
#   MEDIA_DIR=<dir>           salida de Manim (def. contenido/media); debe existir o poder crearse
#   DATASETS_DIR=<dir>        datasets extra, montados en solo lectura en /atp_datasets (ATP_DATASETS
#                             apunta ahi): permite renderizar una corrida sin guardar su dataset
#   NOMBRE_CONTENEDOR=<n>     nombre del contenedor (def. sin nombre; se le añade -<escena>)
set -euo pipefail

AQUI="$(cd "$(dirname "$0")" && pwd)"
REPO="${CONTENIDO_REPO:-$HOME/Documentos/code-projects/codeaerospace_contenido}"
IMAGEN="${IMAGEN:-codeaerospace_contenido-manim}"
CPUS="${CPUS:-2}"

calidad=l; muestra=0; escenas=()
while [ $# -gt 0 ]; do
  case $1 in
    -q) calidad=$2; shift 2 ;;
    -m) muestra=1; shift ;;
    -h|--help) sed -n '2,19p' "$0"; exit 0 ;;
    *) escenas+=("$1"); shift ;;
  esac
done
[ ${#escenas[@]} -gt 0 ] || { echo "falta la escena (pase autotrack campana todo)" >&2; exit 2; }
[ "${escenas[0]}" = todo ] && escenas=(pase autotrack campana)
case $calidad in l) carpeta=480p15 ;; m) carpeta=720p30 ;; h) carpeta=1080p60 ;;
  *) echo "calidad: l, m o h" >&2; exit 2 ;; esac

MEDIA="${MEDIA_DIR:-$AQUI/media}"
mkdir -p "$MEDIA" "$AQUI/muestras"
[ -d "$REPO" ] || { echo "no encuentro el repo de contenido: $REPO" >&2; exit 1; }

docker_base=(docker run --rm --network none --read-only --cpus "$CPUS"
  --user "$(id -u):$(id -g)" -e HOME=/tmp --tmpfs /tmp:size=1g,mode=1777
  -v "$REPO":/workspace:ro -v "$AQUI":/atp:ro -v "$MEDIA":/media)
if [ -n "${DATASETS_DIR:-}" ]; then
  docker_base+=(-v "$(realpath "$DATASETS_DIR")":/atp_datasets:ro)
  export ATP_DATASETS=/atp_datasets
fi

env_atp=()
for v in $(compgen -e | grep '^ATP_' || true); do env_atp+=(-e "$v=${!v}"); done

archivo_de() { case $1 in pase) echo pase.py ;; autotrack) echo autotrack.py ;;
  campana) echo campana.py ;; *) echo "escena desconocida: $1" >&2; return 1 ;; esac; }
clase_de() { case $1 in pase) echo ElPase ;; autotrack) echo ProgramContraAutotrack ;;
  campana) echo LaCampana ;; esac; }

for e in "${escenas[@]}"; do
  archivo=$(archivo_de "$e"); clase=$(clase_de "$e")
  echo "=== $e ($clase, -q$calidad)"
  inicio=$(date +%s)
  nombre=(); [ -n "${NOMBRE_CONTENEDOR:-}" ] && nombre=(--name "$NOMBRE_CONTENEDOR-$e")
  "${docker_base[@]}" "${nombre[@]}" "${env_atp[@]}" -e PYTHONPATH=/atp/escenas -w /atp/escenas "$IMAGEN" \
    manim render -q"$calidad" --media_dir /media --disable_caching --progress_bar none \
      "$archivo" "$clase" 2>&1 | grep -vE '^\s*$' | tail -n 25
  video="$MEDIA/videos/${archivo%.py}/$carpeta/$clase.mp4"
  [ -f "$video" ] || { echo "ERROR: no se genero $video" >&2; exit 1; }
  dur=$("${docker_base[@]}" "$IMAGEN" ffprobe -v error -show_entries format=duration \
        -of csv=p=0 "/media/videos/${archivo%.py}/$carpeta/$clase.mp4")
  echo "OK $video  duracion ${dur%.*}.${dur#*.} s  ($(( $(date +%s) - inicio )) s de render)"
  if [ "$muestra" = 1 ]; then
    # miniatura: un fotograma al 92 % de la duracion (ya con el resumen en pantalla)
    t=$(python3 -c "print(round(float('$dur')*0.92, 2))")
    "${docker_base[@]}" "$IMAGEN" ffmpeg -v error -y -ss "$t" \
      -i "/media/videos/${archivo%.py}/$carpeta/$clase.mp4" -frames:v 1 "/media/muestra-$e.png"
    cp "$MEDIA/muestra-$e.png" "$AQUI/muestras/$e.png"
    echo "   miniatura: contenido/muestras/$e.png (t=$t s)"
  fi
done
