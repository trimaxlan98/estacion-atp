# Mando del controlador del enlace optico, sellado con el MISMO instante del estado
# al que responde (lockstep): la planta lo confirma en /atp/step_ack.
# Angulos en urad; las bases 2D son las del plano normal a dir_a / dir_b.
std_msgs/Header header
float64 t_sim_s
uint8 MODO_ESPERA=0
uint8 MODO_BUSQUEDA=1        # A barre con la baliza, B mira
uint8 MODO_APRETON=2         # B apunto a A y emite su baliza
uint8 MODO_SEGUIMIENTO=3     # lazos QPD + FSM + montura cerrados, enlace de datos
uint8 modo
float64[3] dir_a             # eje de recepcion de A (aparente de B, con el error de efemerides y actitud), TEME
float64[3] dir_b
float64[2] steer_a           # centroide medido: se suma al apuntamiento al entrar en SEGUIMIENTO
float64[2] steer_b           # idem al entrar en APRETON
float64[2] pa_a              # adelanto mandado (point-ahead 2 v_perp / c) de cada terminal
float64[2] pa_b
float64 pitch_urad           # paso de la espiral de barrido
float64 rmax_urad            # radio maximo del barrido (3.5 sigma de la incertidumbre)
float64 dwell_s              # duracion de cada paso del barrido
float64 tasa_bps             # tasa de datos pedida (0: sin datos)
float64 sigma_u_urad         # incertidumbre que el controlador supone (telemetria)
