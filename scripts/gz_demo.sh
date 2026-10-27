#!/usr/bin/env bash
# Animacion en Gazebo: la antena siguiendo al satelite, con la paleta Co.De Aerospace. Corre en el HOST:
#   bash scripts/gz_demo.sh                    # pase al cenit, mascara 45 deg (139 s de pase), 1080p
#   bash scripts/gz_demo.sh el_max:=60 mask:=20   # cualquier argumento de atp_gz.launch.py
#   VIDEO=0 bash scripts/gz_demo.sh            # sin grabar (ventana en tu pantalla, sin Xephyr)
#   RES=1280x720 bash scripts/gz_demo.sh       # otra resolucion de la pantalla virtual
# Salida en runs/gz_demo/ (fuera de git): eval.json (error de apuntamiento medido), demo.mp4 (con
# cabecera y cifras) y demo_crudo.mp4 (lo grabado, sin rotulos).
#
# Gazebo se dibuja en una pantalla virtual Xephyr de RES (por defecto 1920x1080) para no depender del
# tamano de tu monitor; se ve como una ventana mas. Render por software (llvmpipe) o, con GPU=1, por la
# GPU (scripts/gz_gpu.sh; renderer en <salida>/renderer.txt). Si el vigilante del host (scripts/vigilante_host.sh) detiene el contenedor por carga,
# eval.json queda con "parcial": true y lo medido hasta ese momento.
#
# Uso no interactivo (lo lanza scripts/render/gz-demo.sh; sin estas variables todo es como antes):
#   SALIDA=runs/<x>/media/<job>/_t   carpeta de salida (relativa al repo, o absoluta DENTRO del repo:
#                                    el contenedor ve el repo en /ws); por defecto runs/gz_demo
#   PANTALLA=:80                     pantalla Xephyr propia (por defecto :77)
#   NOMBRE=atp-render-gz-demo-<job>  nombre del contenedor (por defecto gz_demo); tambien es su
#                                    particion de gz-transport, para no mezclarse con otro trabajo
#   DOMINIO_ROS=83                   dominio ROS (por defecto el del servicio gz, 82)
#   PROGRESO=1 DUR_EST=<s>           imprime `PROGRESO <0-1>` (DUR_EST: duracion estimada del pase, s)
cd "$(dirname "$0")/.."
SALIDA=${SALIDA:-runs/gz_demo}
NOMBRE=${NOMBRE:-gz_demo}
VD=${PANTALLA:-:77}
case $SALIDA in /*) SALIDA=$(realpath -m --relative-to="$PWD" "$SALIDA") ;; esac
case $SALIDA in ..|../*) echo "gz_demo: SALIDA debe estar dentro del repo (el contenedor lo monta en /ws)" >&2; exit 2 ;; esac
progreso() { [ -n "${PROGRESO:-}" ] && echo "PROGRESO $1"; return 0; }
RES=${RES:-1920x1080}
MARCA=${MARCA:-$HOME/Documentos/code-projects/codeaerospace_contenido/marca/derivados}
FUENTES=${FUENTES:-$HOME/.local/share/fonts/codeaerospace}
mkdir -p "$SALIDA"
rm -f "$SALIDA/eval.json" "$SALIDA/demo.mp4" "$SALIDA/demo_crudo.mp4"
xhost +local:docker >/dev/null 2>&1 || true

# valores por defecto, salvo los que el usuario pase
args=("$@")
tiene() { for a in "${args[@]}"; do [[ "$a" == "$1:="* ]] && return 0; done; return 1; }
tiene el_max || args+=("el_max:=85")
tiene mask || args+=("mask:=45")
tiene eval_file || args+=("eval_file:=/ws/$SALIDA/eval.json")
if [ "${VIDEO:-1}" = "1" ]; then tiene gui_res || args+=("gui_res:=$RES"); else tiene gui_res || args+=("gui_res:=1200x700"); fi

if [ "${VIDEO:-1}" = "1" ]; then
  pkill -f "Xephyr $VD -screen" 2>/dev/null; sleep 0.5
  Xephyr "$VD" -screen "$RES" -ac -br -noreset -title "gz_demo ($RES)" >/dev/null 2>&1 &
  xpid=$!
  for _ in $(seq 1 50); do [ -S "/tmp/.X11-unix/X${VD#:}" ] && break; sleep 0.2; done
  export DISPLAY_GZ=$VD
else
  export DISPLAY_GZ="${DISPLAY:-:0}"
fi
limpiar() {
  [ -n "$ffpid" ] && kill -INT "$ffpid" 2>/dev/null
  [ -n "$xpid" ] && kill "$xpid" 2>/dev/null
  docker rm -f "$NOMBRE" >/dev/null 2>&1
}
trap limpiar EXIT
trap 'exit 143' TERM INT

docker rm -f "$NOMBRE" >/dev/null 2>&1
dominio=(); [ -n "${DOMINIO_ROS:-}" ] && dominio=(-e "ROS_DOMAIN_ID=$DOMINIO_ROS")
particion=(); [ "$NOMBRE" != gz_demo ] && particion=(-e "GZ_PARTITION=$NOMBRE")
source scripts/gz_gpu.sh
gz_gpu_preparar "$DISPLAY_GZ" "$SALIDA" "${LP_NUM_THREADS:-4}"
"${GZ_COMPOSE[@]}" run -d --rm --name "$NOMBRE" "${dominio[@]}" "${particion[@]}" \
  -e DISPLAY="$DISPLAY_GZ" -e QT_X11_NO_MITSHM=1 "${GZ_GL[@]}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw gz bash -c '
  source /opt/ros/jazzy/setup.bash; cd /ws
  # --cmake-force-configure: si `station` (imagen sin ros2_control) compilo antes el workspace compartido, atp_control
  # quedo vacio y su cache de CMake no vuelve a buscar ros2_control (faltaria atp_gz.launch.py)
  colcon build --packages-select atp_description atp_control --cmake-force-configure 2>&1 | tail -2
  source install/setup.bash
  ros2 launch atp_control atp_gz.launch.py headless:=false escena:=true '"${args[*]}"' \
    > /ws/'"$SALIDA"'/launch.log 2>&1' >/dev/null
echo "gz_demo: lanzado (${args[*]}) en $DISPLAY_GZ; log en $SALIDA/launch.log"

corriendo() { docker ps --format '{{.Names}}' | grep -qx "$NOMBRE"; }
progreso 0.05

if [ "${VIDEO:-1}" = "1" ]; then
  # espera a que el lazo cierre (arranca el pase) y que la escena tenga ventana
  for _ in $(seq 1 300); do
    corriendo || break
    grep -q "arranca el pase" "$SALIDA/launch.log" 2>/dev/null && break
    sleep 1
  done
  if corriendo; then
    sleep 2
    progreso 0.2
    echo "gz_demo: grabando $RES -> $SALIDA/demo_crudo.mp4"
    nice -n 10 ffmpeg -y -loglevel error -f x11grab -draw_mouse 0 -framerate 20 -video_size "$RES" \
      -i "$DISPLAY_GZ" -c:v libx264 -preset ultrafast -crf 20 -pix_fmt yuv420p \
      "$SALIDA/demo_crudo.mp4" </dev/null &
    ffpid=$!
    t0=$(date +%s)
    while corriendo; do
      sleep 2
      # el pase corre en tiempo real: avance estimado con la duracion del pase (DUR_EST), tope 0.8
      [ -n "${DUR_EST:-}" ] && progreso "$(awk -v t=$(( $(date +%s) - t0 )) -v d="$DUR_EST" 'BEGIN{p=0.2+0.6*t/d; printf "%.2f", (p>0.8?0.8:p)}')"
    done
    kill -INT "$ffpid" 2>/dev/null; wait "$ffpid" 2>/dev/null; ffpid=''
    progreso 0.85
  else
    echo "gz_demo: el contenedor termino antes de empezar el pase; sin video"
  fi
fi
while corriendo; do sleep 2; done
progreso 0.88

# --- rotulos: recorta la barra de Gazebo y pone cabecera (logo + titulo) y las cifras medidas ---
if [ -f "$SALIDA/demo_crudo.mp4" ]; then
  W=${RES%x*}; H=${RES#*x}; BARRA=48; CAB=76
  python3 - "$SALIDA" "$@" <<'PY'
import json, os, sys
d = sys.argv[1]
try:
    e = json.load(open(os.path.join(d, 'eval.json')))
    cifras = (f"error de apuntamiento  rms {e['rms']:.4f}°   p95 {e['p95']:.4f}°   máx {e['max']:.4f}°   "
              f"dentro de 0.1°: {e['dentro_pct']:.1f} %" + ("   (parcial)" if e.get('parcial') else ''))
except Exception:
    cifras = 'error de apuntamiento: sin datos'
open(os.path.join(d, 'rotulo_cifras.txt'), 'w', encoding='utf-8').write(cifras)
open(os.path.join(d, 'rotulo_titulo.txt'), 'w', encoding='utf-8').write('SEGUIMIENTO ATP · ANTENA Y SATÉLITE')
PY
  F="$FUENTES/Rajdhani-SemiBold.ttf"; FB="$FUENTES/Rajdhani-Medium.ttf"
  logo=(); logof=""
  [ -f "$MARCA/logo-cian.png" ] && { logo=(-i "$MARCA/logo-cian.png"); logof="[1:v]scale=-1:60[lg];"; }
  if [ -n "$logof" ]; then
    ovl="[v0][lg]overlay=W-w-28:8[v1]"
  else
    ovl="[v0]null[v1]"
  fi
  ffmpeg -y -loglevel error -i "$SALIDA/demo_crudo.mp4" "${logo[@]}" -filter_complex \
    "[0:v]crop=$((W - 4)):$((H - CAB)):3:$BARRA,pad=$W:$H:2:$CAB:color=0x080F15[v00];${logof}\
[v00]drawtext=fontfile=$F:textfile=$SALIDA/rotulo_titulo.txt:expansion=none:fontcolor=0xF2F3F4:fontsize=40:x=32:y=16,\
drawbox=x=0:y=$((H - 58)):w=$W:h=58:color=0x080F15@0.78:t=fill,\
drawtext=fontfile=$FB:textfile=$SALIDA/rotulo_cifras.txt:expansion=none:fontcolor=0x00D9FF:fontsize=28:x=28:y=$((H - 42))[v0];$ovl" \
    -map "[v1]" -c:v libx264 -preset medium -crf 18 -pix_fmt yuv420p "$SALIDA/demo.mp4" </dev/null
fi

progreso 0.97
echo "--- resultado ---"
grep -E "EVAL" "$SALIDA/launch.log" | sed 's/^.*\] //' | tail -1
[ -f "$SALIDA/eval.json" ] && echo "eval.json: $(tr -d '\n ' < "$SALIDA/eval.json")"
[ -f "$SALIDA/demo.mp4" ] && echo "video: $SALIDA/demo.mp4 ($(du -h "$SALIDA/demo.mp4" | cut -f1))"
true
