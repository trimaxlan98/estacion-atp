#!/usr/bin/env bash
# Rover planetario en Gazebo Harmonic con video, en el HOST (equipo R3; docs/ROVER.md §Resultados R3):
#   bash scripts/gz_rover.sh                          # marte, terreno rocoso, ruta demo, 50 s simulados
#   PLANETA=luna TERRENO=crater bash scripts/gz_rover.sh
#   PLANETA=titan TERRENO=dunas RUTA=giro DUR=30 bash scripts/gz_rover.sh
#   VIDEO=0 bash scripts/gz_rover.sh                  # sin Xephyr ni grabacion: ventana en tu pantalla
#   RES=1920x1080 bash scripts/gz_rover.sh            # otra resolucion (por defecto 1280x720)
# PLANETA: luna | marte | titan | ceres | tierra (gravedad y colores de cielo/suelo del mundo)
# TERRENO: llano | rocoso | dunas | crater | pendiente (heightmap generado, determinista por SEMILLA)
# MODO:    diffdrive (por defecto: Gazebo simula el rover con DiffDrive/skid-steer y la ruta RUTA se
#                    publica en /rover/cmd_vel por ros_gz_bridge) |
#          estado    (Gazebo solo dibuja lo que diga /rover/estado de la dinamica de R1, CONTRATO §18) |
#                    (con ESTADO_REPETIR=<telemetria.jsonl dentro del repo> reproduce ese archivo en vez de escuchar ROS)
#          externo   (otro nodo publica /rover/cmd_vel; este script solo graba DUR s)
# Salida en runs/gz_rover/<planeta>_<terreno>/ (fuera de git): rover.mp4 (con rotulos), rover_crudo.mp4,
# mundo.sdf + terreno.png/color.png, telemetria.jsonl (formato /rover/estado), resumen.json (tiempos) y logs.
#
# Corre por SOFTWARE (llvmpipe) por omision; con GPU=1 usa la GPU (scripts/gz_gpu.sh; renderer en <salida>/renderer.txt), en una pantalla Xephyr :79. El tiempo
# de pared de una corrida es varias veces la duracion simulada: resumen.json lo mide (rtf = simulado /
# pared). Si el vigilante del host (scripts/vigilante_host.sh) detiene el contenedor por carga, el video
# queda con lo grabado hasta entonces.
#
# Uso no interactivo: SALIDA, PANTALLA (:80...), NOMBRE (contenedor y particion de gz-transport),
# DOMINIO_ROS, PROGRESO=1 (imprime `PROGRESO <0-1>`).
cd "$(dirname "$0")/.."
PLANETA=${PLANETA:-marte}
TERRENO=${TERRENO:-rocoso}
SEMILLA=${SEMILLA:-1}
MODO=${MODO:-diffdrive}
RUTA=${RUTA:-demo}
DUR=${DUR:-50}
RES=${RES:-1280x720}
SALIDA=${SALIDA:-runs/gz_rover/${PLANETA}_${TERRENO}}
NOMBRE=${NOMBRE:-gz_rover}
VD=${PANTALLA:-:79}
MARCA=${MARCA:-$HOME/Documentos/code-projects/codeaerospace_contenido/marca/derivados}
FUENTES=${FUENTES:-$HOME/.local/share/fonts/codeaerospace}
case $SALIDA in /*) SALIDA=$(realpath -m --relative-to="$PWD" "$SALIDA") ;; esac
case $SALIDA in ..|../*) echo "gz_rover: SALIDA debe estar dentro del repo (el contenedor lo monta en /ws)" >&2; exit 2 ;; esac
progreso() { [ -n "${PROGRESO:-}" ] && echo "PROGRESO $1"; return 0; }
mkdir -p "$SALIDA"
rm -f "$SALIDA"/{rover.mp4,rover_crudo.mp4,go,fin_ok,listo,resumen.json,telemetria.jsonl,conducir.log,gz.log}

# espera a que la carga del host baje (el vigilante mata contenedores con load > 12; aqui se pide < 4)
for i in $(seq 1 $(( ${MAX_ESPERA_MIN:-90} * 12 ))); do
  awk -v l="$(cut -d' ' -f1 /proc/loadavg)" 'BEGIN{exit !(l>4)}' || { ok=1; break; }
  [ $((i % 12)) -eq 1 ] && echo "gz_rover: carga alta ($(cut -d' ' -f1 /proc/loadavg)), espero..."
  sleep 5
done
[ -n "$ok" ] || { echo "gz_rover: la carga del host no bajo de 4; no lanzo"; exit 3; }

# mundo + terreno + modelo + GUI (sin Gazebo); el contenedor ve el repo en /ws
progreso 0.02
python3 src/atp_rover/gazebo/generar.py "$SALIDA" --planeta "$PLANETA" --terreno "$TERRENO" --semilla "$SEMILLA" \
  --modo "$([ "$MODO" = estado ] && echo estado || echo diffdrive)" --prefijo-uri "file:///ws/$SALIDA" --res "$RES" || exit 1
progreso 0.05

xhost +local:docker >/dev/null 2>&1 || true
if [ "${VIDEO:-1}" = "1" ]; then
  pkill -f "Xephyr $VD -screen" 2>/dev/null; sleep 0.5
  Xephyr "$VD" -screen "$RES" -ac -br -noreset -title "gz_rover ($RES)" >/dev/null 2>&1 &
  xpid=$!
  for _ in $(seq 1 50); do [ -S "/tmp/.X11-unix/X${VD#:}" ] && break; sleep 0.2; done
  DISPLAY_GZ=$VD
else
  DISPLAY_GZ="${DISPLAY:-:0}"
fi
limpiar() {
  [ -n "$ffpid" ] && kill -INT "$ffpid" 2>/dev/null
  [ -n "$xpid" ] && kill "$xpid" 2>/dev/null
  docker rm -f "$NOMBRE" >/dev/null 2>&1
}
trap limpiar EXIT
trap 'exit 143' TERM INT

docker rm -f "$NOMBRE" >/dev/null 2>&1
t_ini=$(date +%s)
source scripts/gz_gpu.sh
gz_gpu_preparar "$DISPLAY_GZ" "$SALIDA" "${LP_NUM_THREADS:-3}"   # GPU=1: Mesa d3d12 (WSL2) o NVIDIA; aborta si cae a software
export GZ_RENDERER
"${GZ_COMPOSE[@]}" run -d --rm --name "$NOMBRE" \
  -e DISPLAY="$DISPLAY_GZ" -e QT_X11_NO_MITSHM=1 "${GZ_GL[@]}" -e ROS_DOMAIN_ID="${DOMINIO_ROS:-74}" \
  -e GZ_PARTITION="$NOMBRE" -e GZ_SALIDA="/ws/$SALIDA" -e MUNDO="rover_$PLANETA" -e MODO="$MODO" \
  -e RUTA="$RUTA" -e DUR="$DUR" -e ESTADO_REPETIR="${ESTADO_REPETIR:-}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  gz bash /ws/scripts/gz_rover_contenedor.sh >/dev/null
echo "gz_rover: lanzado ($PLANETA, $TERRENO, semilla $SEMILLA, modo $MODO, ruta $RUTA, $DUR s sim) en $DISPLAY_GZ; salida en $SALIDA/"

corriendo() { docker ps --format '{{.Names}}' | grep -qx "$NOMBRE"; }

if [ "${VIDEO:-1}" = "1" ]; then
  progreso 0.08
  for _ in $(seq 1 900); do                    # Gazebo + GUI en software: hasta 15 min
    corriendo || break
    [ -e "$SALIDA/listo" ] && break
    sleep 1
  done
  if corriendo && [ -e "$SALIDA/listo" ]; then
    sleep 3                                    # que la camara de seguimiento y el primer cuadro esten dibujados
    progreso 0.2
    echo "gz_rover: grabando $RES -> $SALIDA/rover_crudo.mp4"
    nice -n 10 ffmpeg -y -loglevel error -f x11grab -draw_mouse 0 -framerate 20 -video_size "$RES" \
      -i "$DISPLAY_GZ" -c:v libx264 -preset ultrafast -crf 20 -pix_fmt yuv420p \
      "$SALIDA/rover_crudo.mp4" </dev/null &
    ffpid=$!
    for _ in $(seq 1 100); do [ -s "$SALIDA/rover_crudo.mp4" ] && break; sleep 0.05; done
    t_graba=$(date +%s)
    touch "$SALIDA/go"                         # arranca la ruta: el video ya esta corriendo
    while corriendo && [ ! -e "$SALIDA/fin_ok" ]; do
      sleep 2
      progreso "$(awk -v t=$(( $(date +%s) - t_graba )) -v d="$DUR" 'BEGIN{p=0.2+0.6*t/(d*2.5); printf "%.2f", (p>0.8?0.8:p)}')"
    done
    sleep 1
    t_fin=$(date +%s)
    while corriendo; do sleep 2; done
    kill -INT "$ffpid" 2>/dev/null; wait "$ffpid" 2>/dev/null; ffpid=''
  else
    echo "gz_rover: el contenedor termino antes de estar listo; sin video"
  fi
else
  until [ -e "$SALIDA/listo" ] || ! corriendo; do sleep 1; done
  t_graba=$(date +%s); touch "$SALIDA/go"
  while corriendo && [ ! -e "$SALIDA/fin_ok" ]; do sleep 2; done
  t_fin=$(date +%s)
fi
while corriendo; do sleep 2; done
progreso 0.88

# tiempos: pared vs simulado (el render por software es lento; el reloj simulado lo pone Gazebo)
python3 - "$SALIDA" "$t_ini" "${t_graba:-0}" "${t_fin:-0}" "$DUR" "$PLANETA" "$TERRENO" "$MODO" "$RES" <<'PY'
import json, os, sys
d, t_ini, t_gr, t_fin, dur, pl, te, modo, res = sys.argv[1:10]
t_ini, t_gr, t_fin, dur = int(t_ini), int(t_gr), int(t_fin), float(dur)
pared = (t_fin - t_gr) if t_fin and t_gr else None
tel = os.path.join(d, 'telemetria.jsonl')
filas = [json.loads(s) for s in open(tel)] if os.path.exists(tel) else []
sim = (filas[-1]['t'] - filas[0]['t']) if len(filas) > 1 else None
dist = 0.0
for a, b in zip(filas, filas[1:]):
    dist += ((b['x'] - a['x']) ** 2 + (b['y'] - a['y']) ** 2) ** 0.5
r = {'planeta': pl, 'terreno': te, 'modo': modo, 'res': res, 'render': os.environ.get('GZ_RENDERER', 'software (llvmpipe)'),
     'dur_pedida_sim_s': dur, 'sim_medido_s': sim, 'pared_ruta_s': pared,
     'pared_total_s': (t_fin - t_ini) if t_fin else None,
     'rtf': round(sim / pared, 3) if sim and pared else None, 'distancia_m': round(dist, 2),
     'muestras': len(filas)}
json.dump(r, open(os.path.join(d, 'resumen.json'), 'w'), indent=1)
print('resumen:', json.dumps(r))
open(os.path.join(d, 'rotulo_titulo.txt'), 'w', encoding='utf-8').write(
    f"ROVER · {pl.upper()} · TERRENO {te.upper()}")
g = {'luna': 1.62, 'marte': 3.71, 'titan': 1.35, 'ceres': 0.28, 'tierra': 9.81}[pl]
# el rotulo dice con que se dibujo de verdad (el renderer completo queda en resumen.json y renderer.txt)
via = 'software' if any(s in r['render'].lower() for s in ('llvmpipe', 'softpipe', 'swrast')) else 'GPU'
cif = f"g = {g} m/s2   modo {modo}" + (f"   {dist:.1f} m recorridos" if modo != 'estado' else '') + (f"   {sim:.0f} s simulados en {pared} s de pared ({via})" if sim and pared else '')
open(os.path.join(d, 'rotulo_cifras.txt'), 'w', encoding='utf-8').write(cif)
PY

# --- rotulos: recorta la barra de Gazebo y pone cabecera (logo + titulo) y las cifras ---
if [ -f "$SALIDA/rover_crudo.mp4" ]; then
  W=${RES%x*}; H=${RES#*x}; BARRA=48; CAB=76
  F="$FUENTES/Rajdhani-SemiBold.ttf"; FB="$FUENTES/Rajdhani-Medium.ttf"
  logo=(); logof=""
  [ -f "$MARCA/logo-cian.png" ] && { logo=(-i "$MARCA/logo-cian.png"); logof="[1:v]scale=-1:60[lg];"; }
  if [ -n "$logof" ]; then ovl="[v0][lg]overlay=W-w-28:8[v1]"; else ovl="[v0]null[v1]"; fi
  ffmpeg -y -loglevel error -i "$SALIDA/rover_crudo.mp4" "${logo[@]}" -filter_complex \
    "[0:v]crop=$((W - 4)):$((H - CAB)):3:$BARRA,pad=$W:$H:2:$CAB:color=0x080F15[v00];${logof}\
[v00]drawtext=fontfile=$F:textfile=$SALIDA/rotulo_titulo.txt:expansion=none:fontcolor=0xF2F3F4:fontsize=40:x=32:y=16,\
drawbox=x=0:y=$((H - 58)):w=$W:h=58:color=0x080F15@0.78:t=fill,\
drawtext=fontfile=$FB:textfile=$SALIDA/rotulo_cifras.txt:expansion=none:fontcolor=0x00D9FF:fontsize=26:x=28:y=$((H - 42))[v0];$ovl" \
    -map "[v1]" -c:v libx264 -preset medium -crf 18 -pix_fmt yuv420p "$SALIDA/rover.mp4" </dev/null
fi
progreso 0.97
echo "--- resultado ---"
[ -f "$SALIDA/rover.mp4" ] && echo "video: $SALIDA/rover.mp4 ($(du -h "$SALIDA/rover.mp4" | cut -f1))"
true
