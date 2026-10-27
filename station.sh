#!/usr/bin/env bash
# Levanta la Estacion (backend + frontend construido) en http://127.0.0.1:8090
# Uso: ./station.sh          -> en primer plano (Ctrl+C la detiene)
#      ./station.sh -d       -> en segundo plano (luego: docker compose stop station)
cd "$(dirname "$0")"
docker compose up station "$@"
