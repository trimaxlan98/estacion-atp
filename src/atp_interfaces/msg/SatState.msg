# Estado geometrico del satelite visto desde la estacion.
std_msgs/Header header
float64 az_deg              # acimut continuo (desenroscado)
float64 el_deg
float64 range_km            # rango oblicuo
float64 range_rate_km_s     # + = se aleja
float64 t_since_aos_s
float64 pass_duration_s
