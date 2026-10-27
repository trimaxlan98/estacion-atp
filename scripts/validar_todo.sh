#!/usr/bin/env bash
# Validacion completa (compila, todas las suites de pruebas y la escalera).
# Uso: ./run.sh bash scripts/validar_todo.sh
source /opt/ros/jazzy/setup.bash; cd /ws
echo "######## build"; colcon build 2>&1 | tail -3; source install/setup.bash
echo "######## pytest atp_ros"; python3 -m pytest -q src/atp_ros/test 2>&1 | tail -3
echo "######## pytest atp_hw"; python3 -m pytest -q src/atp_hw/test 2>&1 | tail -3
echo "######## pytest atp_mcu"; python3 -m pytest -q src/atp_mcu/test 2>&1 | tail -3
echo "######## firmware ESP32: lazo en C + vectores de atp_ros (host, ~5 s)"; bash firmware/esp32_eje/scripts/pruebas_host.sh 2>&1 | grep -E "comprobaciones|tests passed|DIFIERE|FAIL"
echo "######## requisitos de hardware (equipo S): tablas sin SGP4 + el HIL en lockstep se corre aparte (prueba_hil_lockstep.sh, necesita Docker)"; python3 scripts/requisitos_hw.py --rapido > /dev/null && echo "requisitos_hw.py --rapido: OK"
echo "######## pytest sat_adcs"; python3 -m pytest -q src/sat_adcs/test 2>&1 | tail -3
echo "######## pytest sat_isl"; python3 -m pytest -q src/sat_isl/test 2>&1 | tail -3
echo "######## pytest atp_sdr"; python3 -m pytest -q src/atp_sdr/test 2>&1 | tail -3
echo "######## pytest atp_mision"; python3 -m pytest -q src/atp_mision/test 2>&1 | tail -3
echo "######## pytest atp_optico (equipo Y: enlace optico tierra-espacio; el lockstep ROS y el determinismo: validar.sh optico)"; python3 -m pytest -q src/atp_optico/test 2>&1 | tail -3
echo "######## pytest atp_rover COMPLETO (R1-R8: dinamica, navegacion, enlace, operacion, validacion; ~12 min; una sola corrida: antes se repetia por equipo)"; python3 -m pytest -q src/atp_rover/test 2>&1 | tail -3
echo "######## estudio del enlace del rover R7 (--rapido; ~45 s)"; python3 scripts/estudio_enlace_rover.py --rapido > /dev/null && echo "estudio_enlace_rover.py --rapido: OK"
echo "######## estudio de operacion del rover R8 (--rapido; ~2 min, usa la calibracion guardada)"; python3 scripts/estudio_operacion_rover.py --rapido --trabajadores 4 > /dev/null && echo "estudio_operacion_rover.py --rapido: OK"
echo "######## estudio del rover R4 (--rapido: navegacion completa vs basica en 2 planetas; ~4 min)"; python3 scripts/estudio_rover.py --rapido --trabajadores 4 > /dev/null && echo "estudio_rover.py --rapido: OK"
echo "######## validacion fuera de muestra de la navegacion del rover R10 (--rapido: 6 casos nuevos, basico y completa; ~3 min)"; python3 scripts/estudio_navegacion_r10.py --rapido --trabajadores 4 > /dev/null && echo "estudio_navegacion_r10.py --rapido: OK"
echo "######## validacion del rover R5(--rapido: convergencia, Sobol pequeno, literatura, invariancias; ~6 min)"; python3 scripts/validacion_rover.py --rapido --trabajadores 4 > /dev/null && echo "validacion_rover.py --rapido: OK"
echo "######## pytest station";(cd station/backend && python3 -m pytest -q 2>&1 | tail -5)
echo "######## escalera"; bash scripts/validar.sh n1 n2 n4a n4b n4c s4 n5a n5b c1 c2 det rover 2>&1 | grep -v "Summary\|Finished\|Starting"
