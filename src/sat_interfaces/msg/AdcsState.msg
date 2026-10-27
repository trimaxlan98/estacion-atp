# Estado de la planta ADCS, sellado con el instante /clock en que se calculo
# (lockstep, CONTRATO.md sec. 8). Lo publica adcs_sim; lo consumen
# adcs_controller (solo los SENSORES) y adcs_monitor (todo, incluida la verdad).
std_msgs/Header header
float64 t_sim_s              # s desde el inicio del escenario; 0 mientras la planta no arranca

# --- sensores: lo unico que debe usar el controlador ---
float64[3] mag_body_t        # magnetometro, T, ejes del cuerpo
float64[3] gyro_rad_s        # giroscopio (con sesgo y ruido), rad/s
float64[3] sun_body          # vector unitario al Sol en el cuerpo (si sun_valid)
bool sun_valid               # false en eclipse
float64[3] wheel_h_nms       # tacometros: momento axial de cada rueda, N m s

# --- verdad de la simulacion: solo para el monitor ---
float64[4] q_true            # cuerpo -> inercial TEME, [w, x, y, z]
float64[3] w_true_rad_s      # velocidad angular del cuerpo (en el cuerpo)
float64[3] gyro_bias_true    # sesgo real del giroscopio, rad/s
bool eclipse
