#!/usr/bin/env bash
# Genera las corridas reales que alimentan las escenas (Sprint 6).
# Usa el motor ROS (contenedor ros2-jazzy-dev) con ROS_DOMAIN_ID=75 y deja cada
# corrida en runs/<nombre>/ (telemetry.jsonl, metrics.json o campaign.json).
# Las corridas son deterministas (lockstep): repetirlas da las mismas cifras.
# Uso: contenido/corridas.sh [nombre...]   (sin argumentos: todas)
set -euo pipefail
cd "$(dirname "$0")/.."
todas="demo-n2 demo-pase demo-n5a demo-n5b demo-campana"
sel=${*:-$todas}

# params.json con la misma forma que escribe el backend de la Estacion
# (CONTRATO 1): {experiment_id, launch, args, ros_domain_id, created_at}
escribir_params() {   # escribir_params <nombre> <experiment_id> <launch> [k:=v ...]
  python3 - "$@" <<'PY'
import datetime, json, sys
nombre, exp, launch, *kv = sys.argv[1:]
args = dict(a.split(":=", 1) for a in kv)
args = {k: (float(v) if v.replace(".", "", 1).replace("-", "", 1).isdigit() else v)
        for k, v in args.items()}
args = {k: (int(v) if isinstance(v, float) and v.is_integer() else v) for k, v in args.items()}
ruta = f"runs/{nombre}/params.json"
json.dump({"experiment_id": exp, "launch": launch, "args": args, "ros_domain_id": 75,
           "created_at": datetime.datetime.now().isoformat(timespec="seconds")},
          open(ruta, "w"), indent=1)
PY
}

lanzar() {   # lanzar <nombre> <experiment_id> <launch> [args...]
  local nombre=$1 exp=$2 launch=$3; shift 3
  rm -rf "runs/$nombre"
  echo "=== $nombre: ros2 launch atp_ros $launch $* record_dir:=/ws/runs/$nombre"
  docker compose run --rm -T -e ROS_DOMAIN_ID=75 ros bash -c \
    "source /opt/ros/jazzy/setup.bash && cd /ws && source install/setup.bash && \
     timeout 900 ros2 launch atp_ros $launch $* record_dir:=/ws/runs/$nombre" \
    2>&1 | grep -E "RESULTADO|ENLACE|CAMPANA|ENGANCHE|Traceback|ERROR|metrics.json" || true
  escribir_params "$nombre" "$exp" "$launch" "$@"
  ls "runs/$nombre"
}

# compilar una vez (idempotente)
docker compose run --rm -T -e ROS_DOMAIN_ID=75 ros bash -c \
  'source /opt/ros/jazzy/setup.bash && cd /ws && colcon build 2>&1 | tail -3'

for n in $sel; do
  case $n in
    # N2: pase casi cenital, solo el tramo sobre 70 grados (51 s)
    demo-n2)     lanzar "$n" n2-keyhole-cenital atp_loop.launch.py el_max:=85 mask:=70 time_scale:=0 ;;
    # pase completo en banda S: el que muestra el cielo entero (mascara 5)
    demo-pase)   lanzar "$n" adhoc-pase-completo-s atp_loop.launch.py el_max:=60 mask:=5 time_scale:=0 band:=S ;;
    # N5a / N5b: Ka, efemeride 0.1 s atrasada y sesgo; sin y con autotrack
    demo-n5a)    lanzar "$n" n5-autotrack-ka atp_autotrack.launch.py autotrack:=false time_scale:=0 ;;
    demo-n5b)    lanzar "$n" n5-autotrack-ka atp_autotrack.launch.py autotrack:=true time_scale:=0 ;;
    # campana Monte Carlo con lazo (experimento n4c-montecarlo-lazo)
    demo-campana) lanzar "$n" n4c-montecarlo-lazo campaign_run.launch.py mode:=lazo n:=300 seed:=2029 \
        sigma_wind_nm:=0.010 sigma_noise_deg:=0.02 bias_max_deg:=0.03 \
        latency_max_s:=0.045 kp:=25 mask_deg:=5 vel_max_deg_s:=6 ;;
    *) echo "corrida desconocida: $n" >&2; exit 2 ;;
  esac
done
