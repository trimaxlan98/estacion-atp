# Doppler y presupuesto de enlace de bajada en un instante.
std_msgs/Header header
string band
float64 f0_hz
float64 doppler_hz           # fd = -vr/c * f0
float64 doppler_rate_hz_s
float64 theta3_deg           # ancho de haz a media potencia del plato
float64 pointing_error_deg
float64 l_point_db           # 12 (e/theta3)^2
float64 fspl_db
float64 g_t_db
float64 cn0_dbhz
float64 ebn0_db
float64 margin_db            # Eb/N0 - Eb/N0 requerido
bool link_closed
