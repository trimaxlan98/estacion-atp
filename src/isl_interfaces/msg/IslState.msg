# Estado de la planta del enlace optico entre dos satelites (A y B), sellado con el
# instante /clock en que se calculo (lockstep, CONTRATO.md sec. 8). Lo publica
# isl_sim; lo consumen isl_controller (solo los SENSORES) e isl_monitor (todo).
# Angulos en urad.
std_msgs/Header header
float64 t_sim_s              # s desde el inicio del escenario; 0 mientras la planta no arranca

# --- sensores de los terminales: lo unico que debe usar el controlador ---
uint8 modo                   # modo aplicado en el paso (como IslCmd.MODO_*)
bool det_a                   # la camara de A detecto la baliza de B en este paso
bool det_b                   # la camara de B detecto la baliza de A en este paso
float64[2] cen_a_urad        # centroide medido por A: aparente - eje de recepcion (con ruido)
float64[2] cen_b_urad
bool qpd_ok                  # los dos QPD estan dentro de su rango y con senal (seguimiento valido)
float64 pr_a_dbm             # potencia media recibida por A / por B en el paso (-200: sin luz)
float64 pr_b_dbm

# --- verdad de la simulacion: solo para el monitor ---
bool visible                 # el rayo libra la Tierra + 100 km
float64 rango_km
float64 tasa_los_urad_s      # giro de la linea de vista
float64 adelanto_urad        # point-ahead 2 v_perp / c
float64 theta_h_urad         # semiangulo 1/e^2 equivalente del haz de comunicacion
float64 ber                  # peor sentido, media del paso
float64 tasa_bps             # tasa neta de datos si el enlace esta cerrado, 0 si no
bool cerrado
float64 perdida_db           # perdida por apuntamiento media del paso
float64[] err_a_urad         # error radial de seguimiento de A, 1 de cada 10 muestras de 10 kHz
float64[] err_b_urad
float64 err_a_ms             # media de los cuadrados (urad^2) de todas las muestras del paso
float64 err_b_ms
float64 fsm_y_max_urad       # posicion maxima del FSM en el paso
float64 nea_urad             # ruido equivalente en angulo del QPD, por muestra
int32 busq_k                 # pasos de barrido dados
int32 barridos               # barridos completos sin deteccion
bool enganchado              # los lazos de seguimiento estan cerrados
