#!/usr/bin/env bash
# Parte comun de los envoltorios `scripts/render/<id>.sh` (se carga con `source`; ver docs/RENDER.md).
#
# Contrato de un envoltorio:   scripts/render/<id>.sh <run_dir> <salida> [<pantalla>] [--clave valor ...]
#   * escribe sus medios en <salida>/ y SIEMPRE un media.json valido (tambien si falla: estado "error"),
#   * imprime `PROGRESO <0-1>` a medida que avanza,
#   * codigos de salida: 0 hecho · 2 uso o la corrida no cumple el requisito · 3 la carga del host no
#     bajo · 4 sin pantalla libre en el pool :80-:83 · 143 interrumpido (SIGTERM o el vigilante corto el
#     contenedor: el backend lo reconoce como muerte por senal y reintenta; media.json trae lo alcanzado, estado "parcial") · otro: fallo.
#
# Un envoltorio hace:   ID=<id>; source "$(dirname "$0")/_comun.sh"; comun_args "$@"; defecto k v ...; ...
# y al final deja PRINCIPAL, ARCHIVOS, MINI_DE y NOTAS; el trap de salida escribe el media.json.

RAIZ=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
cd "$RAIZ" || exit 1
INICIO=$(date +%s)
declare -A A=()          # --clave valor (guiones -> guiones bajos)
ARGV_ORIG=("$@")
RUN="" SALIDA="" PANTALLA=""
ESTADO=ok ERROR="" NOTAS="" PRINCIPAL="" MINI_DE="" MINI_T=-1
ARCHIVOS=()
HIJO="" TRABAJO="" NOMBRE=""

progreso() { echo "PROGRESO $1"; }

fallo() {   # fallo "mensaje" [codigo]
  ERROR="$1"; ESTADO=error
  echo "ERROR: $1" >&2
  exit "${2:-2}"
}

defecto() {   # defecto <clave> <valor>: solo si el usuario no la dio
  [ -n "${A[$1]+x}" ] || A[$1]="$2"
}

comun_args() {
  [ $# -ge 2 ] || { sed -n '2,/^ID=/{/^ID=/!p}' "$0" | sed 's/^# \{0,1\}//' >&2
                    echo "uso: $0 <run_dir> <salida> [<pantalla>] [--clave valor ...]" >&2; exit 2; }
  RUN=$1; SALIDA=$2; shift 2
  mkdir -p "$SALIDA" || { echo "ERROR: no puedo crear $SALIDA" >&2; exit 2; }
  SALIDA=$(cd "$SALIDA" && pwd)
  [ -d "$RUN" ] && RUN=$(cd "$RUN" && pwd)
  if [ $# -gt 0 ] && [[ $1 == :* ]]; then PANTALLA=$1; shift; fi
  while [ $# -gt 0 ]; do
    case $1 in
      --*=*) k=${1%%=*}; k=${k#--}; A[${k//-/_}]=${1#*=}; shift ;;
      --*) k=${1#--}; k=${k//-/_}
           if [ $# -ge 2 ] && [[ $2 != --* ]]; then A[$k]=$2; shift 2; else A[$k]=true; shift; fi ;;
      *) fallo "argumento inesperado: $1 (uso: <run_dir> <salida> [<pantalla>] [--clave valor ...])" 2 ;;
    esac
  done
  JOBTAG=$(printf '%s' "${RENDER_JOB_ID:-${JOB_ID:-$(basename "$SALIDA")}}" | tr -c 'A-Za-z0-9_.-' '_' | cut -c1-40)-$$
  NOMBRE="atp-render-$ID-$JOBTAG"
  trap 'comun_final' EXIT
  trap 'matar_hijo; exit 143' TERM INT
  [ -d "$RUN" ] || fallo "la corrida no existe: $RUN" 2
}

matar_hijo() {
  [ -n "$HIJO" ] && { kill -TERM "$HIJO" 2>/dev/null; wait "$HIJO" 2>/dev/null; }
  HIJO=""
}

ejecutar() {   # ejecutar cmd args...: en segundo plano para poder cancelar con TERM
  "$@" &
  HIJO=$!
  wait "$HIJO"; local rc=$?
  HIJO=""
  return $rc
}

esperar_carga() {   # esperar_carga <umbral>: la carga a 1 min debe ser <= umbral (como el gobernador)
  [ -n "${SIN_ESPERA_CARGA:-}" ] && return 0
  local i max=$(( ${MAX_ESPERA_MIN:-90} * 12 ))
  for i in $(seq 1 "$max"); do
    awk -v l="$(cut -d' ' -f1 /proc/loadavg)" -v u="$1" 'BEGIN{exit (l>u) ? 0 : 1}' || { INICIO=$(date +%s); return 0; }   # duracion_pared_s no cuenta la espera
    [ $((i % 12)) -eq 1 ] && echo "$ID: carga alta ($(cut -d' ' -f1 /proc/loadavg) > $1), espero..."
    sleep 5
  done
  fallo "la carga del host no bajo de $1 en ${MAX_ESPERA_MIN:-90} min; no lanzo" 3
}


anfitrion() {   # DISPLAY del escritorio real (el que Xephyr necesita como padre); NO el de la pantalla virtual
  # el backend/trabajador fija DISPLAY=<pantalla del pool>: Xephyr no puede abrir su propio display como padre
  if [ -n "${DISPLAY_ANFITRION:-}" ]; then ANF=$DISPLAY_ANFITRION
  elif [ -n "${DISPLAY:-}" ] && [ "$DISPLAY" != "$PANTALLA" ]; then ANF=$DISPLAY
  else ANF=:0; fi
}

pantalla_tomar() {   # toma una pantalla del pool :80-:83 (cerrojo por pantalla); nunca :77/:78
  local p
  local cand=(:80 :81 :82 :83)
  [ -n "$PANTALLA" ] && cand=("$PANTALLA")
  for p in "${cand[@]}"; do
    [[ $p =~ ^:[0-9]+$ ]] || fallo "pantalla invalida: $p (use :80 a :83)" 2
    case $p in :77|:78) [ -n "${RENDER_PANTALLAS_FIJAS:-}" ] ||
      fallo "la pantalla $p es de los scripts interactivos (:77 gz_demo); use :80 a :83" 2 ;; esac
    exec 9>"/tmp/atp-render-pantalla-${p#:}.lock"
    if flock -n 9; then PANTALLA=$p; return 0; fi
  done
  fallo "ninguna pantalla libre del pool (${cand[*]})" 4
}

trabajo_crear() {   # carpeta de trabajo DENTRO del repo (el contenedor lo monta en /ws); sale relativa
  case $SALIDA in
    "$RAIZ"/*) TRABAJO=$SALIDA/_trabajo ;;
    *) TRABAJO=$RAIZ/runs/_render/$JOBTAG ;;
  esac
  rm -rf "$TRABAJO"; mkdir -p "$TRABAJO"
  TRABAJO_REL=$(realpath --relative-to="$RAIZ" "$TRABAJO")
}

trabajo_guardar_logs() {   # los logs utiles del trabajo pasan a <salida>/logs/
  [ -n "$TRABAJO" ] && [ -d "$TRABAJO" ] || return 0
  mkdir -p "$SALIDA/logs"
  local f
  for f in "$@"; do [ -f "$TRABAJO/$f" ] && cp "$TRABAJO/$f" "$SALIDA/logs/$f"; done
  return 0
}

requiere_archivo() {   # requiere_archivo <archivo> "que es"
  [ -s "$RUN/$1" ] || fallo "la corrida no cumple el requisito de $ID: falta $1 ($2) en $RUN" 2
}

comun_final() {
  local rc=$?
  trap - EXIT
  matar_hijo
  [ -n "${TICK:-}" ] && kill "$TICK" 2>/dev/null
  [ -n "$TRABAJO" ] && [ "${CONSERVAR_TRABAJO:-}" != 1 ] && rm -rf "$TRABAJO"
  if [ -n "$SALIDA" ] && [ -d "$SALIDA" ]; then
    if [ $rc -eq 143 ] && [ "$ESTADO" = ok ]; then ESTADO=parcial; fi
    if [ $rc -ne 0 ] && [ $rc -ne 143 ] && [ "$ESTADO" = ok ]; then
      ESTADO=error; ERROR=${ERROR:-"el render termino con codigo $rc"}
    fi
    local args_a=() k
    for k in "${!A[@]}"; do args_a+=("$k" "${A[$k]}"); done
    local cmd_json
    cmd_json=$(python3 -c 'import json,sys; print(json.dumps(["bash","scripts/render/"+sys.argv[1]+".sh"]+sys.argv[2:]))' "$ID" "${ARGV_ORIG[@]}")
    local extra=()
    for k in "${ARCHIVOS[@]}"; do extra+=(--archivo "$k"); done
    [ -n "$PRINCIPAL" ] && extra+=(--principal "$PRINCIPAL")
    [ -n "$MINI_DE" ] && extra+=(--miniatura-de "$MINI_DE" --mini-t "$MINI_T")
    [ -n "$ERROR" ] && extra+=(--error "$ERROR")
    python3 scripts/render/media_json.py --salida "$SALIDA" --render "$ID" --run "$RUN" \
      --inicio "$INICIO" --comando-json "$cmd_json" \
      --args-json "$(python3 scripts/render/ayuda.py args-json "${args_a[@]}")" \
      --notas "$NOTAS" --estado "$ESTADO" "${extra[@]}" >&2 || echo "ERROR: no pude escribir media.json" >&2
    [ "$ESTADO" = ok ] && [ $rc -eq 0 ] && progreso 1
  fi
  exit $rc
}
