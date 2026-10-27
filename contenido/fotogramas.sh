#!/usr/bin/env bash
# Extrae fotogramas PNG de un video renderizado (con el ffmpeg de la imagen de
# Manim, no hace falta ffmpeg en el host). Sirve para comprobar a ojo que las
# cifras en pantalla coinciden con metrics.json.
#
#   contenido/fotogramas.sh media/videos/pase/480p15/ElPase.mp4 5 12.5 30
#
# Deja media/fotogramas/<video>-t<seg>.png (media/ esta ignorada en git).
set -euo pipefail
AQUI="$(cd "$(dirname "$0")" && pwd)"
IMAGEN="${IMAGEN:-codeaerospace_contenido-manim}"
video=$1; shift
[ -f "$AQUI/../$video" ] && video="$AQUI/../$video"
[ -f "$video" ] || video="$AQUI/$video"
[ -f "$video" ] || { echo "no existe $1" >&2; exit 1; }
rel=$(realpath --relative-to="$AQUI/media" "$video")
base=$(basename "$video" .mp4)
mkdir -p "$AQUI/media/fotogramas"
for t in "$@"; do
  docker run --rm --network none --user "$(id -u):$(id -g)" -e HOME=/tmp \
    -v "$AQUI/media":/media "$IMAGEN" \
    ffmpeg -v error -y -ss "$t" -i "/media/$rel" -frames:v 1 "/media/fotogramas/$base-t$t.png"
  echo "contenido/media/fotogramas/$base-t$t.png"
done
