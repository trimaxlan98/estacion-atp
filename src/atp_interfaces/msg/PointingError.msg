# Error de apuntamiento de UN instante (sello = instante del lockstep).
# Solo se publica entre AOS y LOS (CONTRATO 8).
std_msgs/Header header
float64 e_az_deg
float64 e_el_deg
float64 e_pt_deg          # separacion angular real: hypot(e_az cos el, e_el)
