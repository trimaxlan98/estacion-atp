#!/usr/bin/env bash
# Refresca los TLE de data/tle/ desde CelesTrak y regenera data/tle/README.md
# con la epoca de cada objeto.
# Uso (en el host o en el contenedor): bash scripts/actualizar_tle.sh
# CelesTrak pide no repetir la misma consulta mas de una vez cada ~2 h: los
# TLE se actualizan pocas veces al dia, no hace falta insistir.
# Si una descarga falla se conserva el archivo anterior (no se pisa con basura).
cd "$(dirname "$0")/.." || exit 1
dir=data/tle
mkdir -p "$dir"
base='https://celestrak.org/NORAD/elements/gp.php'

bajar() {   # bajar <archivo> <query>   (grupo completo)
  local tmp; tmp=$(mktemp)
  if curl -fsS --max-time 30 "$base?$2&FORMAT=tle" -o "$tmp" \
     && [ "$(grep -c '^1 ' "$tmp")" -ge 1 ]; then
    mv "$tmp" "$dir/$1"; echo "ok   $1 ($(grep -c '^1 ' "$dir/$1") objetos)"
  else
    rm -f "$tmp"; echo "FALLO $1 (se conserva el anterior)" >&2
  fi
}

bajar_lista() {   # bajar_lista <archivo> <catnr...>  (una consulta por objeto)
  local arch=$1; shift
  local tmp; tmp=$(mktemp)
  for n in "$@"; do
    curl -fsS --max-time 30 "$base?CATNR=$n&FORMAT=tle" 2>/dev/null >> "$tmp"
    sleep 1
  done
  if [ "$(grep -c '^1 ' "$tmp")" -ge 1 ]; then
    mv "$tmp" "$dir/$arch"; echo "ok   $arch ($(grep -c '^1 ' "$dir/$arch") objetos)"
  else
    rm -f "$tmp"; echo "FALLO $arch (se conserva el anterior)" >&2
  fi
}

bajar stations.tle 'GROUP=stations'
# meteorologicos polares: NOAA 15/18/19 (APT 137 MHz), NOAA 20/21, METEOR-M2 3/4
bajar_lista meteo.tle 25338 28654 33591 43013 54234 57166 59051
# CubeSats y radioaficion: FUNcube-1 (AO-73), CO-57
bajar_lista cubesats.tle 39444 27848

# README con la epoca de cada TLE (generada, nunca escrita a mano)
python3 scripts/tle_readme.py
