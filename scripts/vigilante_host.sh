#!/usr/bin/env bash
# Vigilante del HOST: si la carga o la memoria se disparan, para los contenedores de
# trabajo ROS (ros/gz/station/m1*_run...) ANTES de que la maquina deje de responder.
# Corre en el host, no en el contenedor:
#   ./scripts/vigilante_host.sh            (primer plano)
# Como servicio: ver scripts/atp-vigilante.service
# Ajustes por entorno: VIG_LOAD (def. 1.5 x hilos), VIG_MEM_MIN_MB (def. 800),
#                      VIG_CICLOS (def. 3 lecturas seguidas), VIG_PERIODO (def. 5 s)
hilos=$(nproc)
umbral_load=${VIG_LOAD:-$(awk -v n="$hilos" 'BEGIN{printf "%.1f", n*1.5}')}
mem_min=${VIG_MEM_MIN_MB:-800}
ciclos=${VIG_CICLOS:-3}
periodo=${VIG_PERIODO:-5}
mal=0
log() { logger -t atp-vigilante "$*" 2>/dev/null; echo "$(date '+%F %T') $*"; }
log "activo: load>${umbral_load} o MemAvailable<${mem_min} MB durante ${ciclos} lecturas"
while sleep "$periodo"; do
  load=$(cut -d' ' -f1 /proc/loadavg)
  mem=$(awk '/MemAvailable/{printf "%d", $2/1024}' /proc/meminfo)
  if awk -v l="$load" -v u="$umbral_load" 'BEGIN{exit !(l>u)}' || [ "$mem" -lt "$mem_min" ]; then
    mal=$((mal+1))
  else
    mal=0
  fi
  if [ "$mal" -ge "$ciclos" ]; then
    log "SOBRECARGA load=${load} mem=${mem}MB: deteniendo contenedores ROS"
    # solo imagenes del proyecto; no toca otros contenedores (nginx, etc.)
    docker ps -q --filter ancestor=ros2-jazzy-dev --filter ancestor=ros2-jazzy-gz \
      | xargs -r docker stop -t 10
    mal=0
  fi
done
