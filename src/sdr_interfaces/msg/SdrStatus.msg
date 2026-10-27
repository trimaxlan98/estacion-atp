# Estado del receptor SDR (cadena de RF a nivel de muestras IQ) tras procesar
# el bloque IQ de UN paso del lockstep. Sello = instante del paso (CONTRATO 8).
# Los campos medidos valen NaN mientras el receptor no esta siguiendo.
std_msgs/Header header
string band
string mode                  # portadora | bpsk
float64 f0_hz
float64 fs_hz                # tasa de muestreo del bloque IQ
uint32 n_samples             # muestras IQ de este bloque
float64 doppler_true_hz      # Doppler verdadero (media de las epocas del bloque)
float64 doppler_est_hz       # frecuencia del NCO del receptor (misma media)
float64 doppler_err_hz       # est - verdad
float64 cn0_budget_dbhz      # C/N0 del presupuesto de enlace (con desapuntamiento)
float64 cn0_est_dbhz         # C/N0 medido sobre la senal IQ: M2M4 (<= 45 dB-Hz) o espectral
float64 cn0_m2m4_dbhz        # M2M4 sobre las muestras post-integracion (NaN si no sigue)
float64 cn0_spec_dbhz        # potencia de banda ancha menos piso espectral
float64 amp_est              # amplitud IQ medida (piso de ruido = 1 por muestra)
float64 lock_metric          # cos(2 fase) compensado en ruido, 1 = enganchado
uint8 state                  # 0 BUSQUEDA, 1 SEGUIMIENTO, 2 ENGANCHADO
bool locked                  # state == 2
float64 ber                  # solo modo bpsk: BER acumulada (NaN si no aplica)
float64 proc_ms              # CPU del hilo que costo el bloque, en ms (no determinista)
