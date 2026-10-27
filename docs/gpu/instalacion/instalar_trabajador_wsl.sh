#!/usr/bin/env bash
# Instala el trabajador de render del host (scripts/render/trabajador_host.py) como servicio systemd DE USUARIO
# en WSL2 (o cualquier Linux), sin editar scripts/render/atp-render.service: las rutas de esta maquina van en un
# drop-in (~/.config/systemd/user/atp-render.service.d/local.conf).
#
#   bash docs/gpu/instalacion/instalar_trabajador_wsl.sh [--url http://127.0.0.1:8090] [--clases ligero,manim]
#                                                        [--contenido ~/codeaerospace_contenido] [--gpu]
#
# Por defecto solo la clase `ligero` (graficas). `--contenido <repo>` (el repo codeaerospace_contenido, con su imagen
# codeaerospace_contenido-manim ya construida: `docker compose build manim` en ese repo) fija CONTENIDO_REPO, anade la
# clase `manim` y usa sus fuentes de marca en graficas. `--gpu` anade la clase `gazebo` con GPU=1 (scripts/gz_gpu.sh:
# en WSL2, OpenGL por Mesa d3d12 sobre la GPU de Windows) e instala Xephyr; necesita la imagen ros2-jazzy-gz
# (`docker compose build gz`). Sin --gpu, `gazebo` se puede pedir igual con --clases y renderiza por software.
# Correr como el usuario normal (no root); usa sudo para apt.
set -euo pipefail
URL=http://127.0.0.1:8090
CLASES=ligero
CONTENIDO=
CONGPU=0
while [ $# -gt 0 ]; do
  case $1 in
    --gpu) CONGPU=1; shift ;;
    --url) URL=$2; shift 2 ;;
    --clases) CLASES=$2; shift 2 ;;
    --contenido) CONTENIDO=$(cd "$2" && pwd); shift 2 ;;
    *) echo "argumento desconocido: $1" >&2; exit 2 ;;
  esac
done
REPO=$(cd "$(dirname "$0")/../../.." && pwd)
[ -f "$REPO/scripts/render/trabajador_host.py" ] || { echo "no encuentro el repo en $REPO" >&2; exit 2; }
case $REPO in /mnt/*) echo "aviso: el repo esta en $REPO (montaje de Windows): clonalo dentro de WSL (CRLF y 9p lento)" >&2 ;; esac
EXTRA_ENV=
if [ -n "$CONTENIDO" ]; then
  [ -d "$CONTENIDO/studio/content/manim_extensions" ] || { echo "$CONTENIDO no parece codeaerospace_contenido (falta studio/content/manim_extensions)" >&2; exit 2; }
  docker image inspect codeaerospace_contenido-manim >/dev/null 2>&1 ||
    echo "aviso: falta la imagen codeaerospace_contenido-manim (cd $CONTENIDO && docker compose build manim)" >&2
  case ",$CLASES," in *,manim,*) ;; *) CLASES="$CLASES,manim" ;; esac
  EXTRA_ENV="Environment=CONTENIDO_REPO=$CONTENIDO
Environment=FUENTES=$CONTENIDO/studio/content/manim_extensions/fonts"
fi
if [ "$CONGPU" = 1 ]; then
  docker image inspect ros2-jazzy-gz >/dev/null 2>&1 || echo "aviso: falta la imagen ros2-jazzy-gz (docker compose build gz)" >&2
  case ",$CLASES," in *,gazebo,*) ;; *) CLASES="$CLASES,gazebo" ;; esac
  EXTRA_ENV="$EXTRA_ENV
Environment=GPU=1"
fi
case ",$CLASES," in *,gazebo,*) sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -qq xserver-xephyr >/dev/null ;; esac

# dependencias del host: matplotlib (graficas.py usa el del host antes que el contenedor), ffmpeg/ffprobe (miniatura y
# metadatos de media.json)
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y -qq python3-matplotlib python3-numpy ffmpeg >/dev/null

UNIDAD=~/.config/systemd/user
mkdir -p "$UNIDAD/atp-render.service.d"
cp "$REPO/scripts/render/atp-render.service" "$UNIDAD/"
# en WSLg el escritorio es :0 sin XAUTHORITY; ExecStart= vacio borra el de la unidad antes de poner el nuestro
cat > "$UNIDAD/atp-render.service.d/local.conf" <<EOF
[Service]
WorkingDirectory=$REPO
ExecStart=
ExecStart=/usr/bin/python3 $REPO/scripts/render/trabajador_host.py --url $URL --clases $CLASES
Environment=DISPLAY=:0
UnsetEnvironment=XAUTHORITY
$EXTRA_ENV
EOF
systemctl --user daemon-reload
systemctl --user enable atp-render.service
systemctl --user restart atp-render.service    # `enable --now` no reinicia uno ya activo: el drop-in nuevo no se aplicaria
sudo loginctl enable-linger "$USER"     # que arranque con WSL aunque no haya una terminal abierta
sleep 2
systemctl --user --no-pager --lines=5 status atp-render.service || true
echo "Listo. Log: journalctl --user -u atp-render -f"
