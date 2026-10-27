#!/usr/bin/env bash
# gz-demo: la antena siguiendo a un satelite en Gazebo, 1080p con los rotulos de Co.De Aerospace.
#   scripts/render/gz-demo.sh <run_dir> <salida> [<pantalla>] [--el_max N] [--mask N] [--band S] [--h_km N]
#                             [--az_culmination N] [--res 1920x1080] [--lp_threads 3]
#
# <run_dir>: corrida de lazo (telemetry.jsonl + metrics.json). Gazebo REPITE el pase con la geometria de esa
# corrida (el_max y mask de su params.json o, si faltan, el maximo de `el` y el minimo de `ref_el` de su
# telemetria; la banda de su metrics.json): no reproduce su telemetria, vuelve a simularla con fisica.
# Los --args pisan lo deducido. <pantalla>: :80 a :83 (por defecto la primera libre); nunca :77/:78.
# Salida: demo.mp4 (principal), eval.json (error de apuntamiento medido en Gazebo), miniatura.jpg, media.json.
# Reutiliza scripts/gz_demo.sh (variables SALIDA, PANTALLA, NOMBRE, DOMINIO_ROS, PROGRESO, DUR_EST).
ID=gz-demo
source "$(dirname "${BASH_SOURCE[0]}")/_comun.sh"
comun_args "$@"

tipo=$(python3 scripts/render/ayuda.py tipo-corrida "$RUN")
[ "$tipo" = lazo ] || fallo "la corrida no cumple el requisito de $ID: debe ser un lazo (telemetry.jsonl + metrics.json de atp_loop/atp_autotrack); esta es: $tipo ($RUN)" 2
requiere_archivo telemetry.jsonl "telemetria del lazo"
requiere_archivo metrics.json "metricas del lazo (la corrida no termino)"

read -r DEL DMASK DBAND DHKM DUR < <(python3 scripts/render/ayuda.py demo-params "$RUN" --sh)
[ "$DEL" != - ] && defecto el_max "$DEL" || defecto el_max 85
[ "$DMASK" != - ] && defecto mask "$DMASK" || defecto mask 45
defecto band "$DBAND"
defecto h_km "$DHKM"
defecto az_culmination 140
defecto res 1920x1080
defecto lp_threads "${LP_NUM_THREADS:-3}"
for k in el_max mask h_km az_culmination; do
  [[ ${A[$k]} =~ ^-?[0-9]+(\.[0-9]+)?$ ]] || fallo "--$k debe ser un numero (recibi '${A[$k]}')" 2
done
[[ ${A[res]} =~ ^[0-9]+x[0-9]+$ ]] || fallo "--res debe ser ANCHOxALTO (recibi '${A[res]}')" 2
python3 - "${A[el_max]}" "${A[mask]}" <<'PY' || fallo "el pase no supera la mascara (el_max ${A[el_max]} <= mask ${A[mask]})" 2
import sys
sys.exit(0 if float(sys.argv[1]) > float(sys.argv[2]) else 1)
PY
# duracion estimada del pase si cambiaron los argumentos
DUR=$(python3 - "${A[h_km]}" "${A[el_max]}" "${A[mask]}" <<'PY'
import sys
sys.path.insert(0, 'src/atp_ros')
try:
    from atp_ros import model
    print(round(model.duracion_pase(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])), 1))
except Exception:
    print(140)
PY
)

pantalla_tomar
anfitrion
esperar_carga 4
trabajo_crear
progreso 0.01
echo "$ID: pase el_max=${A[el_max]} mask=${A[mask]} banda ${A[band]} (~${DUR} s) en $PANTALLA, contenedor $NOMBRE"

ejecutar env DISPLAY="$ANF" SALIDA="$TRABAJO_REL" PANTALLA="$PANTALLA" NOMBRE="$NOMBRE" DOMINIO_ROS="${ROS_DOMAIN_ID:-83}" \
  PROGRESO=1 DUR_EST="$DUR" RES="${A[res]}" LP_NUM_THREADS="${A[lp_threads]}" \
  bash scripts/gz_demo.sh "el_max:=${A[el_max]}" "mask:=${A[mask]}" "band:=${A[band]}" \
  "h_km:=${A[h_km]}" "az_culmination:=${A[az_culmination]}"
rc=$?
trabajo_guardar_logs launch.log
REND=$(cat "$TRABAJO/renderer.txt" 2>/dev/null)   # software o GPU (scripts/gz_gpu.sh)

[ -s "$TRABAJO/demo.mp4" ] || fallo "gz_demo no genero demo.mp4 (codigo $rc; ver $SALIDA/logs/launch.log)" 1
mv "$TRABAJO/demo.mp4" "$SALIDA/demo.mp4"
PRINCIPAL=demo.mp4; MINI_DE=demo.mp4
if [ -s "$TRABAJO/eval.json" ]; then
  mv "$TRABAJO/eval.json" "$SALIDA/eval.json"; ARCHIVOS+=(eval.json)
  NOTAS=$(python3 - "$SALIDA/eval.json" "${A[el_max]}" "${A[mask]}" "${A[band]}" <<'PY'
import json, sys
e = json.load(open(sys.argv[1]))
print(f"pase el_max {sys.argv[2]} deg, mascara {sys.argv[3]} deg, banda {sys.argv[4]}: error de apuntamiento medido en Gazebo "
      f"rms {e['rms']:.4f} deg, p95 {e['p95']:.4f}, max {e['max']:.4f}, dentro de 0.1 deg {e['dentro_pct']:.1f} %"
      + (' (PARCIAL: el pase no termino)' if e.get('parcial') else ''))
PY
)
  NOTAS="$NOTAS; render OpenGL: ${REND:-desconocido}"
  if python3 -c 'import json,sys; sys.exit(0 if json.load(open(sys.argv[1])).get("parcial") else 1)' "$SALIDA/eval.json"; then
    ERROR="el pase quedo incompleto (el vigilante del host u otro corte detuvo el contenedor): reintentable"
    exit 143
  fi
else
  NOTAS="sin eval.json (el contenedor termino antes de medir); render OpenGL: ${REND:-desconocido}"
fi
exit 0
