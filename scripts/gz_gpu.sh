#!/usr/bin/env bash
# Render de Gazebo por software (por omision, como siempre) o con GPU (GPU=1). Lo carga
# scripts/gz_demo.sh con `source`; define:
#   GZ_COMPOSE  (docker compose [-f docker-compose.yml -f docker-compose.gpu*.yml])  para `"${GZ_COMPOSE[@]}" run ... gz`
#   GZ_GL       las -e de OpenGL para ese `run` (llvmpipe por software; vacio con GPU: las pone el compose de GPU)
#   GZ_RENDERER el renderer OpenGL que vera Gazebo (lo escribe tambien en <salida>/renderer.txt)
# gz_gpu_preparar <display> <salida> <lp_threads>: con GPU=1 comprueba con glxinfo DENTRO del contenedor que el
# renderer no sea por software y, si lo es, sale con 2 (no graba un video "con GPU" que en realidad no la uso).
#   WSL2 (existe /dev/dxg): Mesa d3d12 sobre la GPU de Windows (docker-compose.gpu-wsl.yml), validado.
#   Linux nativo: NVIDIA Container Toolkit (docker-compose.gpu.yml), SIN PROBAR.
gz_gpu_preparar() {
  local disp=$1 salida=$2 lp=$3
  # el servicio gz pide cpus: ${ATP_CPUS:-5}; Docker rechaza un tope mayor que los nucleos del host (como run.sh)
  if [ -z "${ATP_CPUS:-}" ] && [ "$(nproc)" -lt 5 ]; then export ATP_CPUS=$(nproc); fi
  GZ_COMPOSE=(docker compose)
  if [ "${GPU:-0}" != 1 ]; then
    GZ_GL=(-e LIBGL_ALWAYS_SOFTWARE=1 -e GALLIUM_DRIVER=llvmpipe -e LP_NUM_THREADS="$lp")
    GZ_RENDERER="software (llvmpipe, $lp hilos)"
  else
    if [ -e /dev/dxg ]; then
      GZ_COMPOSE+=(-f docker-compose.yml -f docker-compose.gpu-wsl.yml)
    else
      GZ_COMPOSE+=(-f docker-compose.yml -f docker-compose.gpu.yml)
    fi
    GZ_GL=()
    GZ_RENDERER=$("${GZ_COMPOSE[@]}" run --rm --no-deps -T -e DISPLAY="$disp" -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
      gz bash -c 'glxinfo -B 2>&1 | sed -n "s/^OpenGL renderer string: //p"' 2>/dev/null | tail -1)
    case $GZ_RENDERER in
      ''|*llvmpipe*|*softpipe*|*swrast*|*SwiftShader*)
        echo "gz_gpu: ERROR: GPU=1 pero el renderer OpenGL del contenedor es '${GZ_RENDERER:-ninguno}' (por software o sin pantalla)" >&2
        exit 2 ;;
    esac
  fi
  echo "gz_gpu: renderer OpenGL: $GZ_RENDERER"
  [ -n "$salida" ] && printf '%s\n' "$GZ_RENDERER" > "$salida/renderer.txt"
  return 0
}
