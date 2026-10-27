"""Modelo fisico de ATP, portado de codeaerospace_contenido/studio/content/
manim_extensions/atp.py (curso 30, Sistemas ATP). Solo numpy, sin ROS, para
que se pueda probar y reutilizar fuera de los nodos.

Geometria esferica honesta (orbita circular, Tierra esferica). NO es SGP4.
Cada funcion dice bajo que condicion vale su cifra.
"""
import numpy as np

R_TIERRA_KM = 6371.0
MU_TIERRA = 398600.0          # km^3/s^2
C_LUZ_M_S = 299792458.0
K_BOLTZMANN_DB = 228.6        # -10 log10(k)
H_LEO_KM = 550.0
MASCARA_DEG = 5.0
J_EJE = 2.0                   # kg m^2, inercia reflejada al eje
B_EJE = 0.5                   # N m s/rad
DIAMETRO_PLATO_M = 3.0
T_SISTEMA_K = 150.0
OBJETIVO_DEG = 0.1            # presupuesto de apuntamiento del curso

BANDAS = {                    # nombre -> frecuencia de bajada (Hz)
    'UHF': 437e6,
    'S': 2.2e9,
    'X': 8.4e9,
    'Ka': 30e9,
}


# ---------------------------------------------------------------- orbita

def velocidad_circular(h_km):
    """Rapidez orbital (km/s) de una orbita CIRCULAR."""
    return float(np.sqrt(MU_TIERRA / (R_TIERRA_KM + float(h_km))))


def periodo_orbital(h_km):
    a = R_TIERRA_KM + float(h_km)
    return float(2.0 * np.pi * np.sqrt(a ** 3 / MU_TIERRA))


def angulo_central(h_km, el_deg):
    el = np.radians(np.asarray(el_deg, dtype=float))
    razon = R_TIERRA_KM / (R_TIERRA_KM + float(h_km))
    lam = np.arccos(np.clip(razon * np.cos(el), -1.0, 1.0)) - el
    return np.degrees(lam)


def elevacion_de_angulo_central(h_km, lam_deg):
    lam = np.radians(np.asarray(lam_deg, dtype=float))
    r = R_TIERRA_KM + float(h_km)
    num = np.cos(lam) - R_TIERRA_KM / r
    den = np.sin(lam)
    el = np.where(den > 1e-12,
                  np.arctan2(num, np.where(den > 1e-12, den, 1.0)), np.pi / 2.0)
    return np.degrees(el)


def rango_oblicuo(h_km, el_deg):
    lam = np.radians(angulo_central(h_km, el_deg))
    r = R_TIERRA_KM + float(h_km)
    d2 = R_TIERRA_KM ** 2 + r ** 2 - 2.0 * R_TIERRA_KM * r * np.cos(lam)
    return np.sqrt(np.maximum(d2, 0.0))


def tasa_acimut(h_km, el_max_deg):
    """Velocidad de acimut (grados/s) EN LA CULMINACION de un pase que
    culmina a `el_max_deg`: el 1/cos(el) que diverge en el cenit."""
    el = float(el_max_deg)
    omega = np.degrees(velocidad_circular(h_km) / float(rango_oblicuo(h_km, el)))
    return float(omega / max(np.cos(np.radians(el)), 1e-9))


def elevacion_keyhole(h_km=H_LEO_KM, vel_max_deg_s=6.0):
    """Elevacion de culminacion a partir de la cual la demanda de acimut
    supera el tope del rotor: por encima, la montura PIERDE el satelite.
    A 550 km con un rotor de 6 deg/s: ~82.5 deg."""
    lo, hi = 0.0, 89.999
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if tasa_acimut(h_km, mid) < vel_max_deg_s else (lo, mid)
    return float(0.5 * (lo + hi))


def duracion_pase(h_km, el_max_deg=90.0, mascara_deg=MASCARA_DEG):
    """Duracion (s) AOS->LOS. Cenital a 550 km con mascara 5: 9.8 min."""
    lam_mask = float(angulo_central(h_km, mascara_deg))
    lam_min = float(angulo_central(h_km, el_max_deg))
    if lam_min >= lam_mask:
        return 0.0
    cos_u = np.cos(np.radians(lam_mask)) / np.cos(np.radians(lam_min))
    u_max = np.degrees(np.arccos(np.clip(cos_u, -1.0, 1.0)))
    return float(2.0 * u_max / 360.0 * periodo_orbital(h_km))


def perfil_pase(h_km=H_LEO_KM, el_max_deg=70.0, mascara_deg=MASCARA_DEG,
                az_culminacion_deg=140.0, n=1200):
    """Pase entero muestreado: t (s desde AOS), el, az, az_continuo (grados),
    d (km), vr (km/s, + = se aleja)."""
    h_km = float(h_km)
    lam_min = float(angulo_central(h_km, el_max_deg))
    lam_mask = float(angulo_central(h_km, mascara_deg))
    if lam_min >= lam_mask:
        raise ValueError(f'un pase que culmina a {el_max_deg} deg no supera '
                         f'la mascara de {mascara_deg} deg')
    cos_u = np.cos(np.radians(lam_mask)) / np.cos(np.radians(lam_min))
    u_max = np.arccos(np.clip(cos_u, -1.0, 1.0))
    w_orb = 2.0 * np.pi / periodo_orbital(h_km)
    u = np.linspace(-u_max, u_max, int(max(16, n)))
    t = (u + u_max) / w_orb
    lam_r = np.arccos(np.clip(np.cos(np.radians(lam_min)) * np.cos(u), -1, 1))
    el = elevacion_de_angulo_central(h_km, np.degrees(lam_r))
    d = rango_oblicuo(h_km, el)
    sin_lam_min = max(np.sin(np.radians(lam_min)), 1e-12)
    psi = np.degrees(np.arctan(np.tan(u) / sin_lam_min))
    az = (float(az_culminacion_deg) + psi) % 360.0
    az_cont = np.degrees(np.unwrap(np.radians(az)))
    dt = np.gradient(t)
    return {
        't': t, 'el': el, 'az': az, 'az_continuo': az_cont, 'd': d,
        'vr': np.gradient(d) / dt,
        'az_pto': np.gradient(az_cont) / dt, 'el_pto': np.gradient(el) / dt,
        'duracion': float(t[-1]), 'd_min': float(np.min(d)),
    }


def elmax_aleatorio(n, h_km=H_LEO_KM, mascara_deg=MASCARA_DEG, rng=None):
    """Elevacion maxima de `n` pases AL AZAR. lam_min ~ U(0, lam_mascara):
    los pases altos son raros (mediana ~22 deg), como en una estacion real."""
    rng = np.random.default_rng() if rng is None else rng
    lam_mask = float(angulo_central(h_km, mascara_deg))
    lam_min = rng.uniform(0.0, lam_mask, int(n))
    return np.asarray(elevacion_de_angulo_central(h_km, lam_min), dtype=float)


# ---------------------------------------------------------------- Doppler

def doppler_hz(vr_km_s, f0_hz):
    """fd = -vr/c f0 (no relativista; vr = 2.5e-5 c, error despreciable)."""
    return -np.asarray(vr_km_s, dtype=float) * 1000.0 / C_LUZ_M_S * float(f0_hz)


def tasa_doppler_cenit(h_km=H_LEO_KM, f0_hz=437e6):
    """Tasa maxima de Doppler (Hz/s), en la culminacion de un pase CENITAL.

    Con la estacion en la superficie (radio R) y el satelite en orbita
    circular (radio R+h, velocidad angular w), el rango cumple
    r^2 = (R+h)^2 + R^2 - 2R(R+h)cos(wt), y en el cenit
        r'' = R (R+h) w^2 / h = (v^2/h) * R/(R+h).
    La version "plana" v^2/h (que usaba esta funcion y el curso 30) olvida la
    curvatura y da (R+h)/R de mas: +8.6 % a 550 km. Hallazgo del equipo de RF
    (atp_sdr), que mide la tasa sobre el perfil del pase. A 437 MHz y 550 km:
    ~141 Hz/s (la plana daba 153)."""
    h = float(h_km)
    v = velocidad_circular(h) * 1000.0
    a_r = v * v / (h * 1000.0) * R_TIERRA_KM / (R_TIERRA_KM + h)
    return float(a_r / C_LUZ_M_S * float(f0_hz))


# ---------------------------------------------------------------- enlace

def ancho_haz(diametro_m=DIAMETRO_PLATO_M, f_hz=2.2e9):
    """theta3dB ~ 70 lambda/D (aprox. de ingenieria, D >> lambda).
    Plato 3 m: S 3.18 deg, Ka 0.233 deg."""
    return float(70.0 * (C_LUZ_M_S / float(f_hz)) / float(diametro_m))


def perdida_apuntamiento(error_deg, theta3_deg):
    """L = 12 (e/theta3)^2 dB (lobulo gaussiano). En e = theta3/2 da 3 dB."""
    return 12.0 * (np.asarray(error_deg, dtype=float) / float(theta3_deg)) ** 2


def ganancia_plato(diametro_m=DIAMETRO_PLATO_M, f_hz=2.2e9, eta=0.6):
    """G = eta (pi D/lambda)^2 en dBi. 3 m, 2.2 GHz, eta 0.6: 34.6 dBi."""
    lam = C_LUZ_M_S / float(f_hz)
    return float(10.0 * np.log10(float(eta) * (np.pi * float(diametro_m) / lam) ** 2))


def fspl_db(d_km, f_hz):
    """Perdida de espacio libre: 92.45 + 20log d_km + 20log f_GHz."""
    return (92.45 + 20.0 * np.log10(np.asarray(d_km, dtype=float))
            + 20.0 * np.log10(float(f_hz) / 1e9))


def g_sobre_t(diametro_m=DIAMETRO_PLATO_M, f_hz=2.2e9, eta=0.6,
              t_sys_k=T_SISTEMA_K):
    """G/T (dB/K). 3 m, 2.2 GHz, eta 0.6, 150 K: 12.8 dB/K."""
    return float(ganancia_plato(diametro_m, f_hz, eta)
                 - 10.0 * np.log10(float(t_sys_k)))


def presupuesto_enlace(d_km, error_deg, f_hz=2.2e9, eirp_dbw=9.0,
                       perdidas_db=3.0, tasa_bps=1e6, ebn0_req_db=10.5,
                       diametro_m=DIAMETRO_PLATO_M, eta=0.6,
                       t_sys_k=T_SISTEMA_K):
    """C/N0 = EIRP - FSPL - L_otras - L_point + G/T + 228.6 [dB-Hz].

    Por defecto: CubeSat de banda S (2 W + parche 6 dBi = 9 dBW) bajando a
    1 Mbps en QPSK (10.5 dB de Eb/N0 para BER 1e-6). `d_km` es el rango
    OBLICUO, no la altitud. Acepta escalares o arrays."""
    th3 = ancho_haz(diametro_m, f_hz)
    # La gaussiana solo vale en el lobulo principal. Fuera de el la antena
    # recibe por lobulos laterales (~0 dBi): la perdida no puede superar la
    # ganancia del plato, o el modelo inventaria miles de dB.
    l_point = np.minimum(perdida_apuntamiento(error_deg, th3),
                         ganancia_plato(diametro_m, f_hz, eta))
    fspl = fspl_db(d_km, f_hz)
    gt = g_sobre_t(diametro_m, f_hz, eta, t_sys_k)
    cn0 = float(eirp_dbw) - fspl - float(perdidas_db) - l_point + gt + K_BOLTZMANN_DB
    ebn0 = cn0 - 10.0 * np.log10(float(tasa_bps))
    return {'theta3_deg': th3, 'l_point_db': l_point, 'fspl_db': fspl,
            'g_t_db': gt, 'cn0_dbhz': cn0, 'ebn0_db': ebn0,
            'margen_db': ebn0 - float(ebn0_req_db)}


# ---------------------------------------------------------------- Monte Carlo

def percentiles(muestras, objetivo=OBJETIVO_DEG):
    """p50/p95/p99/peor. El criterio de aceptacion es p95 < objetivo: la
    mediana describe el pase tipico e ignora al uno de cada veinte."""
    m = np.asarray(muestras, dtype=float)
    p95 = float(np.percentile(m, 95))
    return {'p50': float(np.percentile(m, 50)), 'p95': p95,
            'p99': float(np.percentile(m, 99)), 'peor': float(np.max(m)),
            'pasa': bool(p95 < objetivo)}


def incertidumbre_percentil(n, p=0.95):
    """+- puntos percentiles de un percentil estimado con n corridas
    (binomial). N=500 -> ~1 punto; cuadruplicar N solo la duplica."""
    n = float(n)
    return float(100.0 * np.sqrt(n * p * (1.0 - p)) / n)


def campana_analitica(n=500, sigma_viento_nm=0.010, sigma_ruido_deg=0.02,
                      sesgo_max_deg=0.03, latencia_max_s=0.045, kp=25.0,
                      h_km=H_LEO_KM, mascara_deg=MASCARA_DEG,
                      vel_max_deg_s=6.0, semilla=2029):
    """Campana por CUADRATURA de terminos fisicos (curso 30, L3.2):
    viento tau/kp, latencia v*dt (la que fabrica la cola), ruido y sesgo
    uniforme. La demanda de acimut se acota al tope del rotor. Rapida:
    miles de corridas en milisegundos. Devuelve rms por corrida (grados)."""
    rng = np.random.default_rng(int(semilla))
    n = int(max(1, n))
    el_max = elmax_aleatorio(n, h_km, mascara_deg, rng)
    demanda = np.minimum([tasa_acimut(h_km, e) for e in el_max],
                         float(vel_max_deg_s))
    viento = np.degrees(np.abs(rng.normal(0.0, sigma_viento_nm, n)) / float(kp))
    latencia = demanda * rng.uniform(0.0, latencia_max_s, n)
    ruido = np.abs(rng.normal(0.0, sigma_ruido_deg, n))
    sesgo = rng.uniform(-sesgo_max_deg, sesgo_max_deg, n)
    rms = np.sqrt(viento ** 2 + ruido ** 2 + sesgo ** 2 + latencia ** 2)
    return {'rms': rms, 'el_max': el_max}


def campana_lazo(n=200, sigma_viento_nm=0.010, sigma_ruido_deg=0.02,
                 sesgo_max_deg=0.03, latencia_max_s=0.045, kp=25.0, ki=2.0,
                 kd=8.0, u_max=8.0, vel_max_deg_s=6.0, h_km=H_LEO_KM,
                 mascara_deg=MASCARA_DEG, J=J_EJE, b=B_EJE, f_lazo=20.0,
                 dt=0.01, rafaga_s=2.0, semilla=2029, progreso=None):
    """Campana SIMULANDO el lazo de cada pase: PID digital con derivada
    filtrada y anti-windup + montura J/b con tope de par y de velocidad,
    en los dos ejes, para `n` pases al azar a la vez (vectorizado: el coste
    depende del pase mas largo, no de n).

    Perturbaciones por corrida: sesgo de encoder uniforme (constante),
    ruido de encoder gaussiano por muestra, una rafaga de viento de
    `rafaga_s` s a mitad de pase con par gaussiano, y latencia como
    referencia retrasada. El tope de velocidad hace que los pases casi
    cenitales PIERDAN el satelite en el keyhole: es fisico, no un fallo.

    Metrica: RMS del error de apuntamiento real en el cielo,
    sqrt((e_az cos el)^2 + e_el^2), sobre todo el pase. `progreso(i, n)`
    se llama cada ~5 % del tiempo simulado.
    """
    rng = np.random.default_rng(int(semilla))
    n = int(max(1, n))
    el_max = elmax_aleatorio(n, h_km, mascara_deg, rng)
    # un pase rasante por debajo de mascara+0.5 no tiene arco util
    el_max = np.maximum(el_max, mascara_deg + 0.5)
    az_cul = rng.uniform(0.0, 360.0, n)
    perfiles = [perfil_pase(h_km, e, mascara_deg, a, n=600)
                for e, a in zip(el_max, az_cul)]
    dur = np.array([p['duracion'] for p in perfiles])
    lat = rng.uniform(0.0, latencia_max_s, n)
    sesgo = np.radians(rng.uniform(-sesgo_max_deg, sesgo_max_deg, (2, n)))
    par_rafaga = rng.normal(0.0, sigma_viento_nm, (2, n))
    t_rafaga = dur * rng.uniform(0.2, 0.8, n)
    ruido = np.radians(float(sigma_ruido_deg))

    paso = max(1, int(round(1.0 / (f_lazo * dt))))
    dt_lazo = paso * dt
    n_pasos = int(np.ceil(dur.max() / dt)) + 1
    tick_t = np.arange(0, n_pasos, paso) * dt
    # referencias a la cadencia del lazo, retrasadas por la latencia
    ref = np.zeros((2, n, len(tick_t)))
    ref_true = np.zeros((2, n, len(tick_t)))
    for k, p in enumerate(perfiles):
        tt = np.minimum(tick_t, p['t'][-1])
        td = np.maximum(tt - lat[k], 0.0)
        for eje, clave in ((0, 'az_continuo'), (1, 'el')):
            ref_true[eje, k] = np.radians(np.interp(tt, p['t'], p[clave]))
            ref[eje, k] = np.radians(np.interp(td, p['t'], p[clave]))

    theta = ref_true[:, :, 0].copy()
    omega = np.zeros((2, n))
    integ = np.zeros((2, n))
    e_prev = np.zeros((2, n))
    d_filt = np.zeros((2, n))
    u = np.zeros((2, n))
    w_max = np.radians(vel_max_deg_s)
    alfa = dt_lazo / (0.08 + dt_lazo)
    suma_e2 = np.zeros(n)
    cuenta = np.zeros(n)
    aviso = max(1, len(tick_t) // 20)

    for j, t in enumerate(tick_t):
        vivo = t <= dur
        medida = theta + sesgo + rng.normal(0.0, ruido, (2, n))
        e = ref[:, :, j] - medida
        d_filt += alfa * ((e - e_prev) / dt_lazo - d_filt)
        u_raw = kp * e + ki * integ + kd * d_filt
        u = np.clip(u_raw, -u_max, u_max)
        congelar = (np.abs(u_raw) > u_max) & (u_raw * e > 0.0)
        integ += np.where(congelar, 0.0, e * dt_lazo)
        e_prev = e
        # error real (sin sesgo ni ruido: lo que ve el satelite)
        e_true = ref_true[:, :, j] - theta
        e_pt = np.hypot(e_true[0] * np.cos(ref_true[1, :, j]), e_true[1])
        suma_e2 += np.where(vivo, np.degrees(e_pt) ** 2, 0.0)
        cuenta += vivo
        for _ in range(paso):
            en_rafaga = (t >= t_rafaga) & (t < t_rafaga + rafaga_s)
            par = u + np.where(en_rafaga, par_rafaga, 0.0)
            omega += (par - b * omega) / J * dt
            np.clip(omega, -w_max, w_max, out=omega)
            theta += omega * dt
        if progreso is not None and j % aviso == 0:
            progreso(j, len(tick_t))

    rms = np.sqrt(suma_e2 / np.maximum(cuenta, 1))
    return {'rms': rms, 'el_max': el_max}


# ---------------------------------------------------------------- autotrack

def perdida_cruce_conscan(r_deg, theta3_deg):
    """Perdida de cruce (dB) de un conical scan de radio `r_deg`: la que paga
    el enlace por mirar SIEMPRE un poco al lado. 12 (r/theta3)^2; con
    r = 0.2 theta3 son 0.48 dB."""
    return float(12.0 * (float(r_deg) / float(theta3_deg)) ** 2)


def potencia_conscan(dx, dy, fase, r_deg, theta3_deg):
    """Potencia relativa (dB) que recibe un alimentador que nuta en un
    circulo de radio `r_deg` cuando el blanco esta en (dx, dy) grados
    (cross-elevacion, elevacion) respecto al eje mecanico. Lobulo
    gaussiano: valido mientras el blanco este dentro del haz principal."""
    fx = float(r_deg) * np.cos(fase)
    fy = float(r_deg) * np.sin(fase)
    return -12.0 * ((dx - fx) ** 2 + (dy - fy) ** 2) / float(theta3_deg) ** 2


def discriminador_conscan(p_db, fase, r_deg, theta3_deg):
    """Estima el desapuntamiento (dx, dy) en grados a partir de las muestras
    de potencia de UN numero entero de vueltas del scan.

    Con el lobulo gaussiano en dB la parte que oscila a la frecuencia del
    scan es exactamente (24 r / theta3^2)(dx cos f + dy sin f), asi que
        dx = theta3^2 / (12 r) * <(P - <P>) cos f>
    y lo mismo con sin para dy. Exacto en el modelo; con ruido, el error
    del estimador baja como 1/sqrt(muestras)."""
    p = np.asarray(p_db, dtype=float)
    f = np.asarray(fase, dtype=float)
    pc = p - p.mean()
    k = float(theta3_deg) ** 2 / (12.0 * float(r_deg))
    return float(k * np.mean(pc * np.cos(f))), float(k * np.mean(pc * np.sin(f)))
