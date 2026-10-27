#!/usr/bin/env bash
# Valida la escalera N0-N4 dentro del contenedor. Uso: ./run.sh bash scripts/validar.sh [nivel...]
# (sin set -u: los setup.bash de ROS usan variables sin definir)
cd /ws
source /opt/ros/jazzy/setup.bash
colcon build 2>&1 | tail -2
source install/setup.bash
niveles=${*:-"test n1 n2 n4a n4b n4c s4 n5a n5b c1 det"}
filtro='RESULTADO|ENLACE|CAMPANA|Traceback|ERROR|sin confirmar|respaldo'
for n in $niveles; do
  echo "=================== $n"
  case $n in
    test) python3 -m pytest -q src/atp_ros/test 2>&1 | tail -3 ;;
    n1)  timeout 300 ros2 launch atp_ros atp_loop.launch.py profile:=ramp time_scale:=0 2>&1 | grep -E "$filtro" ;;
    n2)  timeout 300 ros2 launch atp_ros atp_loop.launch.py el_max:=85 mask:=70 time_scale:=0 2>&1 | grep -E "$filtro" ;;
    # N4a: pase oblicuo COMPLETO de 7 min (el que antes no cabia en tiempo real), banda S
    n4a) timeout 600 ros2 launch atp_ros atp_loop.launch.py el_max:=30 mask:=5 time_scale:=0 band:=S 2>&1 | grep -E "$filtro" ;;
    # N4b: el mismo pase cenital en Ka con viento: el haz de 0.23 deg no perdona
    n4b) timeout 600 ros2 launch atp_ros atp_loop.launch.py el_max:=85 mask:=20 time_scale:=0 band:=Ka gust_nm:=3.0 gust_start:=60 2>&1 | grep -E "$filtro" ;;
    # S4: pase REAL de la ISS sobre CDMX con SGP4 + TLE (data/tle)
    s4)  timeout 600 ros2 launch atp_ros atp_loop.launch.py profile:=tle sat:=ISS station:=cdmx pass_index:=0 start_utc:=2026-10-02T15:00:00 time_scale:=0 band:=S 2>&1 | grep -E "$filtro|pase real" ;;
    # N4c: Monte Carlo como accion ROS, analitico y con lazo
    n4c)
      ros2 run atp_ros campaign_server > /tmp/campaign.log 2>&1 &
      pid=$!; sleep 3
      ros2 action send_goal /atp/run_campaign atp_interfaces/action/RunCampaign "{mode: analitico, n: 2000}" 2>&1 | grep -E "p95_deg|meets|Goal finished"
      ros2 action send_goal --feedback /atp/run_campaign atp_interfaces/action/RunCampaign "{mode: lazo, n: 300}" 2>&1 | grep -E "p95_deg|meets|Goal finished|done: 300" | head -4
      kill $pid; sleep 1; grep CAMPANA /tmp/campaign.log ;;
    # N5: adquisicion y seguimiento por RF. Ka con efemeride 0.1 s atrasada y sesgo
    # mecanico: el program track solo no cabe en el haz de 0.23 deg; el autotrack si
    n5a) timeout 600 ros2 launch atp_ros atp_autotrack.launch.py autotrack:=false time_scale:=0 2>&1 | grep -E "$filtro" ;;
    n5b) timeout 600 ros2 launch atp_ros atp_autotrack.launch.py autotrack:=true time_scale:=0 2>&1 | grep -E "$filtro|ENGANCHE" ;;
    # C1 (control avanzado): el mismo pase cenital N2 con LQR integral + pre-alimentacion
    c1)  timeout 300 ros2 launch atp_ros atp_loop.launch.py el_max:=85 mask:=70 time_scale:=0 controller:=lqr 2>&1 | grep -E "$filtro" ;;
    # C2: LQR + Kalman con planta realista (backlash 0.05, Coulomb+Stribeck, encoder 14 bits), error REAL
    c2)  timeout 300 ros2 launch atp_ros atp_loop.launch.py el_max:=85 mask:=70 time_scale:=0 controller:=lqr estimator:=kalman backlash_deg:=0.05 coulomb_nm:=0.05 stribeck_nm:=0.02 encoder_bits:=14 truth:=true 2>&1 | grep -E "$filtro" ;;
    # Determinismo del lockstep: la misma corrida dos veces, CON grabador, debe dar
    # telemetry.jsonl y metrics.json identicos byte a byte (CONTRATO 8).
    # det = ambos; det_lazo = solo el lazo N2; det_n5 = solo el autotrack N5
    det|det_lazo|det_n5)
      lanzs=("atp_loop.launch.py el_max:=85 mask:=70 time_scale:=0" "atp_autotrack.launch.py autotrack:=true")
      [ "$n" = det_lazo ] && lanzs=("${lanzs[0]}")
      [ "$n" = det_n5 ] && lanzs=("${lanzs[1]}")
      for caso in "${lanzs[@]}"; do
        rm -rf /tmp/det; for i in 1 2; do
          timeout 900 ros2 launch atp_ros $caso record_dir:=/tmp/det/$i > /tmp/det_$i.log 2>&1
        done
        if cmp -s /tmp/det/1/telemetry.jsonl /tmp/det/2/telemetry.jsonl && cmp -s /tmp/det/1/metrics.json /tmp/det/2/metrics.json; then
          echo "DETERMINISTA ($caso): $(wc -l < /tmp/det/1/telemetry.jsonl) filas identicas"
        else echo "NO DETERMINISTA ($caso): difieren"; fi
      done ;;
    # Mision integrada (equipo P, docs/MISION.md; NO estan en la lista por defecto).
    # mision_test: la orquestacion con corridas simuladas (sin ROS). mision: una mision CORTA de
    # verdad (3 h, 2 estaciones, ISS en S + NOAA 20 en X: 5 contactos en lockstep, SDR, autotrack,
    # OD y un gemelo con TLE) DOS veces; los dos informes deben ser identicos byte a byte.
    mision_test) python3 -m pytest -q src/atp_mision/test 2>&1 | tail -3 ;;
    mision)
      rm -rf /tmp/mision
      for i in 1 2; do
        timeout 3000 python3 scripts/mision.py --sats "ISS,NOAA 20" --estaciones cdmx,madrid --horas 3 \
          --n-od 2 --n-min-od 2 --el-cal-min 8 --latencia-od 300 --concurrencia "${MISION_CONC:-3}" --id m$i --salida /tmp/mision > /tmp/mision_$i.log 2>&1
        grep -E 'Planificado|^contactos|^GB |Mejora por la OD|GB con OD|apuntamiento rms|Incidencias|Verificacion|Traceback' /tmp/mision_$i.log
      done
      if cmp -s /tmp/mision/m1/informe.json /tmp/mision/m2/informe.json; then
        echo "DETERMINISTA (mision): informe.json identico, $(wc -c < /tmp/mision/m1/informe.json) bytes"
      else echo "NO DETERMINISTA (mision): los informes difieren"; fi ;;
    # OD (equipo N, docs/OD.md; NO esta en la lista por defecto): el lazo de la orbita en ROS.
    # Pase 1: ISS sobre CDMX; od_estimator ajusta la orbita con angulos + rango + Doppler sellados.
    # Pase 2 (otro dia): el generador usa ESA orbita (profile:=od) y el estimador la encadena como a priori.
    od)
      rm -rf /tmp/od; mkdir -p /tmp/od
      timeout 900 ros2 launch atp_ros atp_loop.launch.py profile:=tle sat:=ISS station:=cdmx pass_index:=0 start_utc:=2026-10-02T15:00:00 time_scale:=0 band:=S od:=true od_range:=true od_doppler:=true od_out:=/tmp/od/1 record_dir:=/tmp/od/rec1 2>&1 | grep -E "$filtro|OD lotes|pase real"
      timeout 900 ros2 launch atp_ros atp_loop.launch.py profile:=od od_prior:=od od_file:=/tmp/od/1/od.json station:=cdmx pass_index:=0 start_utc:=2026-10-03T00:00:00 time_scale:=0 band:=S od:=true od_range:=true od_doppler:=true od_out:=/tmp/od/2 record_dir:=/tmp/od/rec2 2>&1 | grep -E "$filtro|OD lotes|ORBITA ESTIMADA"
      python3 scripts/od_comparar.py /tmp/od/1/od.json --sat ISS --station cdmx --start-utc 2026-10-03T00:00:00
      python3 scripts/od_comparar.py /tmp/od/2/od.json --sat ISS --station cdmx --start-utc 2026-10-04T00:00:00 ;;
    # Enlace optico tierra-espacio (equipo Y, docs/OPTICO.md; NO esta en la lista por defecto).
    # optico_test: las pruebas de atp_optico (~2 min). optico: pruebas + cifras clave del estudio (r0, centelleo, tasa vs
    # elevacion) + el ATP optico en LOCKSTEP ROS (ISS sobre CDMX, el pase de 78 deg, solo el tramo sobre 50 deg) DOS veces: optico.json y
    # optico_serie.jsonl deben ser identicos byte a byte, y coincidir con la corrida sin ROS (`atp_optico.escenario`).
    optico_test) python3 -m pytest -q src/atp_optico/test 2>&1 | tail -3 ;;
    optico)
      python3 -m pytest -q src/atp_optico/test 2>&1 | tail -3
      python3 scripts/optico_estudio.py clave
      bash "$0" optico_ros ;;
    # optico_ros: solo el lockstep ROS dos veces + la comparacion con la corrida sin ROS (~8 min)
    optico_ros)
      rm -rf /tmp/optico; mkdir -p /tmp/optico
      A="sat:=ISS station:=cdmx pass_index:=0 start_utc:=2026-10-02T15:00:00 tle_mask:=50 time_scale:=0"
      for i in 1 2; do
        timeout 900 ros2 launch atp_optico optico.launch.py $A salida:=/tmp/optico/r$i > /tmp/optico_$i.log 2>&1
        grep -E 'RESUMEN OPTICO|ATP residual|ENLACE con FSM|Traceback|ERROR' /tmp/optico_$i.log | sed 's/^.*optico_atp\]: //'
      done
      python3 -m atp_optico.escenario --station cdmx --pase_idx 0 --mascara_deg 50 --salida /tmp/optico/offline | sed -n 2,4p
      if cmp -s /tmp/optico/r1/optico.json /tmp/optico/r2/optico.json && cmp -s /tmp/optico/r1/optico_serie.jsonl /tmp/optico/r2/optico_serie.jsonl; then
        echo "DETERMINISTA (optico): optico.json y $(wc -l < /tmp/optico/r1/optico_serie.jsonl) filas de serie identicas en 2 corridas ROS"
      else echo "NO DETERMINISTA (optico): difieren"; fi
      if cmp -s /tmp/optico/r1/optico_serie.jsonl /tmp/optico/offline/optico_serie.jsonl; then echo "IDENTICO a la corrida sin ROS (optico_serie.jsonl, byte a byte)"; else echo "la serie ROS difiere de la corrida sin ROS (ver abajo)"; fi
      python3 scripts/optico_estudio.py comparar /tmp/optico/r1 /tmp/optico/offline ;;
    # Rover en LOCKSTEP ROS (equipo R9, docs/ROVER.md «Resultados R9»; NO esta en la lista por defecto). rover_test: las pruebas de la
    # cadena sin ROS (~1.5 min; el pytest completo de atp_rover ya corre en validar_todo). rover: la MISMA escena corta con ROS (sim_clock
    # + rover_dinamica + rover_navegador) a time_scale 0 y 2, con navegacion basica y completa, mas la cadena operador -> rover_enlace
    # -> rover (Luna, retardo de luz, navegador en Tierra) y la ventana de MRO en Marte (planta cinematica, ~45 min simulados); cada
    # corrida repite la escena SIN ROS (scripts/rover_lockstep.py) y compara telemetry.jsonl, metrics.json y mandos.jsonl byte a
    # byte; ademas time_scale 0 y 2 deben dar los mismos bytes entre si (~8 min).
    rover_test) python3 -m pytest -q src/atp_rover/test/test_lockstep.py 2>&1 | tail -3 ;;
    rover)
      rm -rf /tmp/rover_ls; mkdir -p /tmp/rover_ls; D=${ROVER_DOMINIO:-160}; ok=1
      L=(python3 scripts/rover_lockstep.py --salida /tmp/rover_ls --dominio $D)
      for nav in basico completa; do
        for ts in 0 2; do
          "${L[@]}" --id ${nav}_x$ts --planeta luna --terreno rocoso --semilla 7 --ruta "[[4,0],[4,3]]" --duracion_s 12 --v_max 0.5 \
            --navegacion $nav --time_scale $ts > /tmp/rover_ls/${nav}_x$ts.log 2>&1
          grep -E 'EQUIVALENCIA|Traceback|ERROR' /tmp/rover_ls/${nav}_x$ts.log | sed "s/^/[$nav x$ts] /"
        done
        for f in telemetry.jsonl metrics.json mandos.jsonl; do
          cmp -s /tmp/rover_ls/${nav}_x0/$f /tmp/rover_ls/${nav}_x2/$f || { ok=0; echo "DIFIERE $f entre time_scale 0 y 2 ($nav)"; }
        done
      done
      "${L[@]}" --id marte --planeta marte --terreno dunas --semilla 3 --ruta "[[5,0]]" --duracion_s 12 --v_max 0.4 --navegacion basico \
        > /tmp/rover_ls/marte.log 2>&1; grep -E 'EQUIVALENCIA|Traceback|ERROR' /tmp/rover_ls/marte.log | sed 's/^/[marte dunas] /'
      "${L[@]}" --id enlace_luna --planeta luna --terreno llano --semilla 1 --duracion_s 8 --enlace retardo --operador guion \
        --guion "[[1.0,0.3,0.0],[4.0,0.0,0.0]]" > /tmp/rover_ls/enlace_luna.log 2>&1
      grep -E 'EQUIVALENCIA|Traceback|ERROR' /tmp/rover_ls/enlace_luna.log | sed 's/^/[enlace luna] /'
      echo "ordenes que llegan al rover (t de escena, enviadas en 1.0 y 4.0 s): $(python3 -c "import json;print([r['t'] for r in map(json.loads, open('/tmp/rover_ls/enlace_luna/mandos.jsonl'))])")"
      "${L[@]}" --id enlace_luna_nav --planeta luna --terreno llano --semilla 1 --ruta "[[1.5,0]]" --duracion_s 10 --v_max 0.5 --navegacion basico \
        --enlace retardo --operador navegador > /tmp/rover_ls/enlace_luna_nav.log 2>&1
      grep -E 'EQUIVALENCIA|Traceback|ERROR' /tmp/rover_ls/enlace_luna_nav.log | sed 's/^/[navegador en Tierra, luna] /'
      echo "primera orden del navegador en Tierra llega al rover (t de escena; 2 x 1.28 s de retardo): $(python3 -c "import json;print(json.loads(open('/tmp/rover_ls/enlace_luna_nav/mandos.jsonl').readline())['t'])")"
      "${L[@]}" --id ventanas_marte --planeta marte --dinamica cinematica --enlace ventanas --t0_s 117599 --duracion_s 2760 --operador guion \
        --guion "[[1.0,0.3,0.0]]" > /tmp/rover_ls/ventanas.log 2>&1; grep -E 'EQUIVALENCIA|Traceback|ERROR' /tmp/rover_ls/ventanas.log | sed 's/^/[ventanas marte] /'
      echo "orden enviada en 1.0 s llega (t de escena): $(python3 -c "import json;print([r['t'] for r in map(json.loads, open('/tmp/rover_ls/ventanas_marte/mandos.jsonl'))])")"
      for d in basico_x0 completa_x0 marte enlace_luna enlace_luna_nav ventanas_marte; do
        grep -q '^  "identico": true' /tmp/rover_ls/$d/equivalencia.json || { ok=0; echo "DIFIERE con y sin ROS: $d"; }
      done
      if [ $ok = 1 ]; then echo "DETERMINISTA (rover): telemetry, metrics y mandos identicos con y sin ROS en 6 escenas (8 corridas ROS), y entre time_scale 0 y 2"
      else echo "NO DETERMINISTA (rover): ver arriba"; fi ;;
    # Dron en LOCKSTEP ROS (D5, docs/DRON.md «D5»; NO esta en la lista por defecto). dron_test: las pruebas de la cadena sin ROS y de los
    # nodos sobre un rclpy en memoria (~10 s). dron: la MISMA escena con ROS de verdad (sim_clock + dron_dinamica + dron_navegador) a
    # time_scale 0 y 2, sin y con sensores (ESKF a bordo); cada corrida repite la escena SIN ROS (scripts/dron_lockstep.py) y compara
    # telemetry.jsonl, metrics.json y setpoints.jsonl byte a byte; ademas time_scale 0 y 2 deben dar los mismos bytes entre si.
    dron_test) python3 -m pytest -q src/atp_dron/test/test_lockstep.py 2>&1 | tail -3 ;;
    dron)
      rm -rf /tmp/dron_ls; mkdir -p /tmp/dron_ls; D=${DRON_DOMINIO:-170}; ok=1
      L=(python3 scripts/dron_lockstep.py --salida /tmp/dron_ls --dominio $D)
      for sens in false true; do
        for ts in 0 2; do
          "${L[@]}" --id s${sens}_x$ts --ruta "[[10,0,5],[10,8,5]]" --duracion_s 20 --viento "[2,0,0]" --rafaga_sigma 1.0 --semilla 3 \
            --sensores $sens --time_scale $ts > /tmp/dron_ls/s${sens}_x$ts.log 2>&1
          grep -E 'EQUIVALENCIA|Traceback|ERROR' /tmp/dron_ls/s${sens}_x$ts.log | sed "s/^/[sensores $sens x$ts] /"
        done
        for f in telemetry.jsonl metrics.json setpoints.jsonl; do
          cmp -s /tmp/dron_ls/s${sens}_x0/$f /tmp/dron_ls/s${sens}_x2/$f || { ok=0; echo "DIFIERE $f entre time_scale 0 y 2 (sensores $sens)"; }
        done
      done
      for d in sfalse_x0 sfalse_x2 strue_x0 strue_x2; do
        grep -q '^  "identico": true' /tmp/dron_ls/$d/equivalencia.json || { ok=0; echo "DIFIERE con y sin ROS: $d"; }
      done
      if [ $ok = 1 ]; then echo "DETERMINISTA (dron): telemetry, metrics y setpoints identicos con y sin ROS, sin y con sensores (4 corridas ROS), y entre time_scale 0 y 2"
      else echo "NO DETERMINISTA (dron): ver arriba"; fi ;;
  esac
done
