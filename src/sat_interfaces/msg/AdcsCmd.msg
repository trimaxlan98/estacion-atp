# Mando del controlador ADCS, sellado con el MISMO instante del estado al que
# responde (lockstep): la planta lo confirma en /atp/step_ack.
std_msgs/Header header
float64 t_sim_s
uint8 MODE_DETUMBLE=0
uint8 MODE_NADIR=1
uint8 MODE_TRACK_GS=2
uint8 mode
float64[3] wheel_torque_nm   # par de MOTOR de cada rueda (el cuerpo recibe el opuesto)
float64[3] mtq_dipole_am2    # dipolo de cada magnetopar

# --- telemetria del estimador (para el monitor) ---
bool est_valid
float64[4] q_est             # cuerpo -> inercial, [w, x, y, z]
float64[3] bias_est_rad_s
float64[3] sigma_att_rad     # 1-sigma de cada eje de actitud segun la covarianza
