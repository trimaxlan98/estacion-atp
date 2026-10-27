#!/usr/bin/env bash
# graficas: curvas clave de una corrida (matplotlib, paleta Co.De Aerospace) y una lamina resumen 1920x1080.
#   scripts/render/graficas.sh <run_dir> <salida> [<pantalla>] [--titulo "texto"]
#
# <run_dir>: cualquier corrida con telemetry.jsonl (lazo, ADCS u otra) o una corrida de red con
# escena.json + red.jsonl + metricas.json. La pantalla no se usa (se acepta por uniformidad).
# Salida: lamina.png (principal, 1920x1080), una PNG por grafica (1600x900), miniatura.jpg, media.json.
# matplotlib: del host si lo tiene; si no, del contenedor ros2-jazzy-dev (sin red, solo lectura de la
# corrida; la marca y las fuentes se montan en solo lectura).
ID=graficas
source "$(dirname "${BASH_SOURCE[0]}")/_comun.sh"
comun_args "$@"

tipo=$(python3 scripts/render/ayuda.py tipo-corrida "$RUN")
case $tipo in
  lazo|adcs) requiere_archivo telemetry.jsonl "telemetria"; requiere_archivo metrics.json "metricas (la corrida no termino)" ;;
  red) requiere_archivo metricas.json "metricas de la red (la corrida no termino)" ;;
  *) [ -s "$RUN/telemetry.jsonl" ] ||
       fallo "la corrida no cumple el requisito de $ID: necesita telemetry.jsonl o una corrida de red con escena.json + red.jsonl + metricas.json; esta es: $tipo ($RUN)" 2 ;;
esac
defecto titulo ""
COMMIT=$(git rev-parse HEAD 2>/dev/null)
FUENTES=${FUENTES:-$HOME/.local/share/fonts/codeaerospace}
MARCA=${MARCA:-$HOME/Documentos/code-projects/codeaerospace_contenido/marca/derivados}
progreso 0.05

if python3 -c 'import matplotlib' 2>/dev/null; then
  ejecutar env FUENTES="$FUENTES" MARCA="$MARCA" python3 scripts/render/graficas.py "$RUN" "$SALIDA" \
    --titulo "${A[titulo]}" --commit "$COMMIT" --nombre-corrida "$(basename "$RUN")"
else
  ejecutar docker run --rm --name "$NOMBRE" --network none --cpus 2 --memory 3g --user "$(id -u):$(id -g)" \
    -e HOME=/tmp -e MPLCONFIGDIR=/tmp/mpl -e FUENTES=/fuentes -e MARCA=/marca --tmpfs /tmp:size=512m,mode=1777 \
    --entrypoint python3 \
    -v "$RAIZ/scripts/render":/render:ro -v "$RUN":/corrida:ro -v "$SALIDA":/salida \
    -v "$FUENTES":/fuentes:ro -v "$MARCA":/marca:ro \
    ros2-jazzy-dev /render/graficas.py /corrida /salida --titulo "${A[titulo]}" --commit "$COMMIT" --nombre-corrida "$(basename "$RUN")"
fi
rc=$?
[ $rc -eq 0 ] || fallo "graficas.py fallo (codigo $rc)" 1
progreso 0.9
[ -s "$SALIDA/lamina.png" ] || fallo "no se genero lamina.png" 1
PRINCIPAL=lamina.png; MINI_DE=lamina.png
for f in "$SALIDA"/*.png; do
  b=$(basename "$f"); [ "$b" = lamina.png ] || ARCHIVOS+=("$b")
done
NOTAS="corrida de tipo $tipo; ${#ARCHIVOS[@]} graficas individuales (1600x900) y la lamina 1920x1080; cifras de metrics.json/metricas.json"
exit 0
