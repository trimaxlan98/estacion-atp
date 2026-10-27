#!/usr/bin/env bash
# Pruebas del lazo del ESP32 en el host (C puro + ctest) y comprobacion de que
# los vectores de referencia siguen siendo los que genera atp_ros.
# Dentro del contenedor ROS:  ./run.sh bash firmware/esp32_eje/scripts/pruebas_host.sh
# (o desde scripts/validar_todo.sh, que ya corre en /ws). ~5 s. Sale con
# codigo distinto de 0 si algo falla.
set -uo pipefail
RAIZ="$(cd "$(dirname "$0")/../../.." && pwd)"
cd "$RAIZ"
SALIDA="${TMPDIR:-/tmp}/eje_pruebas_build"
rm -rf "$SALIDA"
estado=0
echo "--- vectores: regenerar con atp_ros y comparar con los del repo"
PYTHONPATH=src/atp_ros python3 firmware/esp32_eje/test/gen_vectores.py --comprobar || estado=1
echo "--- compilar (C99, -Wall -Wextra -Wconversion -Werror)"
cmake -S firmware/esp32_eje/test -B "$SALIDA" -G "Unix Makefiles" > /dev/null || exit 1
cmake --build "$SALIDA" 2>&1 | grep -E "warning|error|Error" && estado=1
echo "--- ctest"
ctest --test-dir "$SALIDA" -V > "$SALIDA/ctest.log" 2>&1 || estado=1
grep -E "^[0-9]+: (\[|  |[0-9]+ comprobaciones)" "$SALIDA/ctest.log" | sed 's/^[0-9]*: //'
grep -E "tests passed|Test +#" "$SALIDA/ctest.log"
exit $estado
