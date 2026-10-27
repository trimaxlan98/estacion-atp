"""Fisica del control de actitud (ADCS) de un CubeSat 3U. Solo numpy, sin ROS.

Es el segmento ESPACIO de la Estacion ATP: aqui vive todo lo que se puede
probar sin simular el reloj (cuaterniones, dinamica, ambiente, sensores,
estimacion y control). Los nodos de `sat_adcs` solo ponen esto en lockstep.

CONVENCIONES (valen para todo el paquete)
  Cuaternion q = [w, x, y, z], ESCALAR PRIMERO, Hamilton (i*j = k), unitario.
  q representa la rotacion CUERPO -> INERCIAL: v_I = R(q) v_B, con R = q_a_dcm(q).
  Cinematica: dq/dt = 1/2 q (x) [0, w_B], con w_B la velocidad angular del
  cuerpo respecto al inercial EXPRESADA en el cuerpo. El error entre una
  actitud q y una referencia q_ref es dq = q_ref^-1 (x) q (rotacion cuerpo ->
  marco de referencia); su parte vectorial es ~ dtheta/2, con dtheta el
  angulo de error expresado en el cuerpo.
  Marco inercial = TEME (el de SGP4 en `atp_ros.orbit`). Cuerpo: +Z es el
  eje largo del 3U y el BORE-SIGHT de la antena; +X, +Y las caras de 10 cm.
  LVLH de referencia: +Z = nadir, +Y = - normal de la orbita, +X ~ velocidad.
  Unidades SI salvo posiciones/velocidades orbitales (km, km/s, como orbit.py).

QUE ES CADA MODELO Y DONDE DEJA DE VALER esta en el docstring de cada funcion.
Resumen de lo que NO es:
  * El campo geomagnetico a bordo es por omision un DIPOLO inclinado (IGRF
    grado 1), NO IGRF. La "verdad" de la simulacion es IGRF-13 completo (grado
    13, solo campo principal). Ver `error_modelo_campo`.
  * La atmosfera es exponencial por capas: la densidad real varia x10 con la
    actividad solar. El par aerodinamico es de placas planas (Cd supuesto).
  * No hay modelo termico, ni flexibilidad, ni sloshing, ni dinamica del
    rotor de las ruedas (se modela su momento axial).
"""
import math
from dataclasses import dataclass, field

import numpy as np

from atp_ros import orbit
from sat_adcs.igrf13_datos import IGRF13_2020

MU_M3_S2 = 3.986004418e14          # constante gravitatoria terrestre (WGS-84/EGM)
R_TIERRA_KM = 6378.137             # radio ecuatorial WGS-84 (el de orbit.py)
A_GEOMAG_KM = 6371.2               # radio de referencia de IGRF (NO el ecuatorial)
OMEGA_TIERRA = orbit.OMEGA_TIERRA  # rad/s
UA_KM = 149597870.7
EPS = 1e-12


# ====================================================================== algebra

def skew(v):
    """Matriz [v x] tal que [v x] u = v x u."""
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


def cruz(a, b):
    """a x b de dos 3-vectores, ~10x mas rapido que np.cross (que domina el
    perfil de la planta porque se llama miles de veces por paso)."""
    return np.array([a[1] * b[2] - a[2] * b[1],
                     a[2] * b[0] - a[0] * b[2],
                     a[0] * b[1] - a[1] * b[0]])


def unitario(v):
    """v/|v|; exige |v| > 0."""
    n = float(np.linalg.norm(v))
    if n < EPS:
        raise ValueError('vector nulo no tiene direccion')
    return np.asarray(v, dtype=float) / n


# ====================================================================== cuaterniones

def q_norm(q):
    """Normaliza. Un q casi nulo es un error de programa, no se arregla."""
    n = float(np.linalg.norm(q))
    if n < EPS:
        raise ValueError('cuaternion nulo')
    return np.asarray(q, dtype=float) / n


def q_mult(a, b):
    """Producto de Hamilton a (x) b, escalar primero. Componer rotaciones:
    R(a (x) b) = R(a) R(b). No es conmutativo."""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                     w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def q_conj(q):
    """Conjugado = inverso para un cuaternion unitario."""
    return np.array([q[0], -q[1], -q[2], -q[3]])


def q_a_dcm(q):
    """Matriz de rotacion cuerpo -> inercial R(q) de un cuaternion unitario:
    R = (w^2 - v.v) I + 2 v v^T + 2 w [v x]. Ortogonal propia (det +1)."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def dcm_a_q(R):
    """Matriz de rotacion -> cuaternion (metodo de Shepperd: parte de la
    componente mas grande para no dividir por algo pequeno). Con w >= 0."""
    t = np.trace(R)
    if t >= max(R[0, 0], R[1, 1], R[2, 2]):
        w = 0.5 * math.sqrt(1.0 + t)
        k = 0.25 / w
        q = np.array([w, (R[2, 1] - R[1, 2]) * k, (R[0, 2] - R[2, 0]) * k,
                      (R[1, 0] - R[0, 1]) * k])
    else:
        a = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
        b, c = (a + 1) % 3, (a + 2) % 3
        s = 2.0 * math.sqrt(1.0 + R[a, a] - R[b, b] - R[c, c])
        v = np.zeros(3)
        v[a] = 0.25 * s
        v[b] = (R[b, a] + R[a, b]) / s
        v[c] = (R[c, a] + R[a, c]) / s
        q = np.array([(R[c, b] - R[b, c]) / s, *v])
    q = q_norm(q)
    return q if q[0] >= 0.0 else -q


def q_de_rotvec(theta):
    """Vector de rotacion theta (rad, eje x angulo) -> cuaternion. Exacta;
    serie de Taylor para |theta| < 1e-6 (evita 0/0)."""
    th = float(np.linalg.norm(theta))
    if th < 1e-6:
        return q_norm(np.array([1.0, *(0.5 * np.asarray(theta))]))
    s = math.sin(0.5 * th) / th
    return np.array([math.cos(0.5 * th), *(s * np.asarray(theta))])


def q_a_rotvec(q):
    """Cuaternion -> vector de rotacion (rad) con |theta| <= pi (camino
    corto: q y -q dan lo mismo)."""
    q = q if q[0] >= 0.0 else -q
    v = q[1:]
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        return 2.0 * v
    return 2.0 * math.atan2(n, q[0]) * v / n


def q_de_eje_angulo(eje, ang):
    """Rotacion de `ang` rad alrededor del eje (no necesita ser unitario)."""
    return q_de_rotvec(unitario(eje) * float(ang))


def q_rotar(q, v):
    """v_I = R(q) v_B sin construir la matriz."""
    return q_a_dcm(q) @ np.asarray(v, dtype=float)


def q_error(q_ref, q):
    """dq = q_ref^-1 (x) q, con signo tal que w >= 0 (camino corto).
    Cuerpo -> referencia: 2*dq[1:] ~ angulo de error en el cuerpo."""
    dq = q_mult(q_conj(q_ref), q)
    return dq if dq[0] >= 0.0 else -dq


def angulo_entre(u, v):
    """Angulo (rad) entre dos vectores, estable cerca de 0 y de pi."""
    u, v = np.asarray(u, dtype=float), np.asarray(v, dtype=float)
    return math.atan2(float(np.linalg.norm(np.cross(u, v))), float(u @ v))


def q_aleatorio(rng):
    """Cuaternion uniforme en SO(3) (Shoemake)."""
    u1, u2, u3 = rng.random(3)
    return np.array([math.sqrt(1 - u1) * math.sin(2 * math.pi * u2),
                     math.sqrt(1 - u1) * math.cos(2 * math.pi * u2),
                     math.sqrt(u1) * math.sin(2 * math.pi * u3),
                     math.sqrt(u1) * math.cos(2 * math.pi * u3)])


# ====================================================================== satelite

@dataclass
class Satelite:
    """Parametros fisicos. Todo lo que no tiene fuente esta marcado SUPUESTO.

    `A` (3 x N): ejes de las N ruedas expresados en el cuerpo. `J` incluye el
    momento de inercia transversal de las ruedas (convencion estandar); el
    momento de cada rueda `h` es su momento angular AXIAL absoluto.
    """
    J: np.ndarray
    A: np.ndarray
    h_max: float                  # Nms por rueda
    tau_max: float                # Nm por rueda
    m_max: float                  # A m^2 por eje de magnetopar
    masa: float
    dims: tuple                   # (dx, dy, dz) m
    cg: np.ndarray                # desplazamiento del CG respecto al centro geometrico, m
    m_res: np.ndarray             # dipolo magnetico residual en el cuerpo, A m^2
    cd: float = 2.2               # coeficiente de arrastre de placa plana
    nombre: str = 'CubeSat 3U'

    def __post_init__(self):
        # caras de la caja para el par aerodinamico: normal, area, brazo desde el CG
        dx, dy, dz = self.dims
        n, area, arm = [], [], []
        for eje, a, medio in ((0, dy * dz, dx / 2), (1, dx * dz, dy / 2), (2, dx * dy, dz / 2)):
            for signo in (1.0, -1.0):
                v = np.zeros(3)
                v[eje] = signo
                n.append(v)
                area.append(a)
                arm.append(v * medio - self.cg)
        self._caras = (np.array(n), np.array(area), np.array(arm))

    @property
    def n_ruedas(self):
        return self.A.shape[1]

    @property
    def J_inv(self):
        return np.linalg.inv(self.J)


def cubesat_3u(asimetria=0.02):
    """CubeSat 3U de referencia.

    Fuentes y SUPUESTOS (cada cifra dice de donde sale):
      * 3U = 100 x 100 x 340.5 mm y <= 4.0 kg: CubeSat Design Specification
        (Cal Poly) rev. 13. Se usa 0.34 m y 4.0 kg (techo de la masa: SUPUESTO).
      * Inercia: caja homogenea, I = m/12 (a^2 + b^2) (calculada). La masa de un
        CubeSat real no es homogenea: se suma un acoplamiento (productos de
        inercia) del `asimetria` = 2 % de Ixx, SUPUESTO tipico del reparto
        interno, para que los ejes de inercia no coincidan con los del cuerpo.
      * Ruedas: 3 ortogonales con la hoja de datos de CubeSpace CubeWheel Small
        (par maximo 0.23 mNm, momento 1.77 mNms a 10 000 rpm). Verificado
        en la hoja del fabricante (satnow/CubeSpace), 2026-10.
      * Magnetopares: 0.2 A m^2 por eje, la cifra de la placa ISISpace iMTQ
        (verificada en la hoja del fabricante); los CubeTorquer estan en el
        mismo orden de magnitud.
      * CG desplazado (1, 0.5, 2) cm del centro geometrico: SUPUESTO dentro de
        la tolerancia del CDS (+-2 cm en X, Y; +-7 cm en Z).
      * Dipolo magnetico residual (0.01, -0.005, 0.008) A m^2: SUPUESTO, orden
        de magnitud tipico de un CubeSat sin compensar.
    """
    dx, dy, dz = 0.1, 0.1, 0.34
    m = 4.0
    J = np.diag([m / 12 * (dy ** 2 + dz ** 2), m / 12 * (dx ** 2 + dz ** 2),
                 m / 12 * (dx ** 2 + dy ** 2)])
    Ixx = J[0, 0]
    acople = asimetria * Ixx * np.array([[0.0, 1.0, 0.5], [1.0, 0.0, 0.3],
                                         [0.5, 0.3, 0.0]])
    return Satelite(J=J + acople, A=np.eye(3), h_max=1.77e-3, tau_max=0.23e-3,
                    m_max=0.2, masa=m, dims=(dx, dy, dz),
                    cg=np.array([0.01, 0.005, 0.02]),
                    m_res=np.array([0.01, -0.005, 0.008]))


# ====================================================================== dinamica

def estado_inicial(q, w, h_w):
    """Vector de estado y = [q(4), w(3), h_w(N)]."""
    return np.concatenate([q_norm(q), np.asarray(w, float), np.asarray(h_w, float)])


def momento_total_cuerpo(y, sat):
    """H = J w + A h_w (momento angular total, en el cuerpo)."""
    return sat.J @ y[4:7] + sat.A @ y[7:]


def energia_rotacional(y, sat):
    """T = 1/2 w.J w. Se conserva sin pares externos si el momento de las
    ruedas es constante (u = 0): d(T)/dt = w.(-w x H) = 0 porque w.(w x H) = 0."""
    w = y[4:7]
    return 0.5 * float(w @ sat.J @ w)


def f_rigido(y, sat, u_w, tau_ext):
    """Derivada del estado de un cuerpo rigido con N ruedas de reaccion.

        dq/dt = 1/2 q (x) [0, w]
        J dw/dt = tau_ext - w x (J w + A h) - A u_w
        dh/dt = u_w

    `u_w` (N) es el par que el MOTOR aplica a cada rueda (el cuerpo recibe el
    opuesto). `tau_ext`: par externo en el cuerpo (3,) o una funcion
    tau_ext(q, w) -> (3,). Vale para cuerpo rigido con J constante en el
    cuerpo; los productos de inercia del rotor se absorben en J."""
    q, w, h = y[:4], y[4:7], y[7:]
    tau = tau_ext(q, w) if callable(tau_ext) else tau_ext
    dq = 0.5 * q_mult(q, np.array([0.0, *w]))
    dw = sat.J_inv @ (tau - cruz(w, sat.J @ w + sat.A @ h) - sat.A @ u_w)
    return np.concatenate([dq, dw, u_w])


def rk4(f, y, h):
    """Un paso de Runge-Kutta de 4o orden de paso fijo h: error local O(h^5),
    global O(h^4) (se verifica en test_fisica). `f(y) -> dy`."""
    k1 = f(y)
    k2 = f(y + 0.5 * h * k1)
    k3 = f(y + 0.5 * h * k2)
    k4 = f(y + h * k3)
    return y + h / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)


def saturar_ruedas(u_cmd, h, sat, dt):
    """Par REAL de cada rueda: recorta al par maximo y, dentro del paso `dt`,
    al momento maximo (la rueda no puede pasar de +-h_max aunque el mando lo
    pida: |h + u dt| <= h_max). Mando retenido (ZOH) durante dt."""
    u = np.clip(np.asarray(u_cmd, float), -sat.tau_max, sat.tau_max)
    lo = (-sat.h_max - h) / dt
    hi = (sat.h_max - h) / dt
    return np.clip(u, lo, hi)


def propagar(y, sat, u_cmd, tau_ext, dt, h_int=0.25):
    """Avanza `dt` s con RK4 en subpasos de <= `h_int`, con el mando de ruedas
    retenido (ZOH) y saturado en par y momento. `tau_ext(q, w)` (o un vector)
    se evalua con el AMBIENTE congelado durante `dt`: para ambiente que
    varia, el llamador avanza en tramos cortos (la planta lo hace a 1 s, y el
    campo gira ~2e-3 rad por segundo, asi que el error es ~1e-3 relativo
    en el par). El cuaternion se renormaliza tras cada subpaso (la deriva
    de norma de RK4 es O(h^5)). Devuelve (estado nuevo, par de rueda aplicado)."""
    n = max(1, int(math.ceil(dt / h_int - 1e-9)))
    h = dt / n
    y = np.array(y, float)
    u = saturar_ruedas(u_cmd, y[7:], sat, dt)
    for _ in range(n):
        y = rk4(lambda s: f_rigido(s, sat, u, tau_ext), y, h)
        y[:4] = q_norm(y[:4])
    return y, u


# ====================================================================== ambiente: orbita de prueba

def r_circular(a_km, inc_rad, t_s, raan=0.0, u0=0.0):
    """Posicion y velocidad (TEME aprox., km, km/s) de una orbita CIRCULAR
    kepleriana. Solo para pruebas y estimaciones: no incluye J2."""
    n = math.sqrt(MU_M3_S2 * 1e-9 / a_km ** 3)
    u = u0 + n * t_s
    cu, su, ci, si, cr, sr = (math.cos(u), math.sin(u), math.cos(inc_rad),
                              math.sin(inc_rad), math.cos(raan), math.sin(raan))
    r = a_km * np.array([cr * cu - sr * su * ci, sr * cu + cr * su * ci, su * si])
    v = a_km * n * np.array([-cr * su - sr * cu * ci, -sr * su + cr * cu * ci, cu * si])
    return r, v


# ====================================================================== ambiente: perturbaciones

def par_gradiente_gravedad(J, q, r_km):
    """Par de gradiente de gravedad en el cuerpo: tau = 3 mu/r^3 (c x J c), con
    c la direccion unitaria geocentro -> satelite expresada en el cuerpo
    (forma estandar, Wie, 'Space Vehicle Dynamics and Control'; c y -c dan
    lo mismo). Vale para una Tierra puntual (sin J2) y J constante; su orden de
    magnitud es 3 n^2 dI ~ 1e-7 N m a 550 km con dI ~ 0.035 kg m^2 (se
    calcula en test_fisica)."""
    r = np.asarray(r_km, float) * 1e3
    rn = float(np.linalg.norm(r))
    rb = q_a_dcm(q).T @ (r / rn)
    return 3.0 * MU_M3_S2 / rn ** 3 * cruz(rb, J @ rb)


# Atmosfera exponencial por capas: rho = rho0 exp(-(h-h0)/H). Tabla de
# Vallado (Fundamentals of Astrodynamics, tabla 8-4) RECORDADA de memoria, no
# verificada contra el libro en esta sesion; lo unico que la respalda es que
# las capas empalman entre si a < 1 % (test_atmosfera: si una cifra estuviera
# mal, el salto se veria). Sirve para el orden de magnitud del par
# aerodinamico, no para predecir el decaimiento. (h0 km, rho0 kg/m^3, H km)
CAPAS_ATMOSFERA = [
    (300.0, 2.418e-11, 53.628), (350.0, 9.518e-12, 53.298), (400.0, 3.725e-12, 58.515),
    (450.0, 1.585e-12, 60.828), (500.0, 6.967e-13, 63.822), (600.0, 1.454e-13, 71.835),
    (700.0, 3.614e-14, 88.667), (800.0, 1.170e-14, 124.64), (900.0, 5.245e-15, 181.05),
]


def densidad_atmosfera(h_km):
    """Densidad (kg/m^3) por capas exponenciales (ver CAPAS_ATMOSFERA). Fuera de
    300-900 km extrapola la capa mas cercana. Incertidumbre real: x2-x10."""
    cap = CAPAS_ATMOSFERA[0]
    for c in CAPAS_ATMOSFERA:
        if h_km >= c[0]:
            cap = c
    return cap[1] * math.exp(-(h_km - cap[0]) / cap[2])


def par_aerodinamico(sat, q, r_km, v_km_s, rho=None):
    """Par aerodinamico de PLACAS PLANAS en el cuerpo.

    Cada una de las 6 caras de la caja expuesta al viento (n.v > 0) recibe
    F = -1/2 rho Cd A (n.v^) |v|^2 v^ aplicada en su centro; el brazo es el
    centro de la cara menos el CG. Viento relativo = v - w_tierra x r
    (atmosfera que co-rota). Sin sustentacion, sin sombras entre caras ni
    reemision: error tipico de un factor 2 en la fuerza. El valor sale de
    la densidad (incertidumbre x2-x10), asi que solo sirve como orden de
    magnitud: ~1e-8 N m a 550 km con el CG a 2 cm."""
    r = np.asarray(r_km, float)
    v_rel = (np.asarray(v_km_s, float) - np.array([-OMEGA_TIERRA * r[1], OMEGA_TIERRA * r[0], 0.0])) * 1e3
    h_km = float(np.linalg.norm(r)) - R_TIERRA_KM
    rho = densidad_atmosfera(h_km) if rho is None else rho
    vb = q_a_dcm(q).T @ v_rel
    vn = float(np.linalg.norm(vb))
    vh = vb / vn
    n, area, arm = sat._caras
    cos = n @ vh
    w = np.where(cos > 0.0, -0.5 * rho * sat.cd * area * cos * vn ** 2, 0.0)   # F_i = w_i vh
    return cruz(w @ arm, vh)


def par_magnetico(m_body, q, B_eci_T):
    """tau = m x B, con m en el cuerpo (A m^2) y B inercial (T) pasado al cuerpo."""
    return cruz(m_body, q_a_dcm(q).T @ np.asarray(B_eci_T, float))


# ====================================================================== ambiente: campo geomagnetico

NMAX_IGRF = 13
_COEF_ORDEN = sorted(IGRF13_2020)


def anio_decimal(jd_ut1):
    """Fecha juliana -> ano decimal (aprox. de 365.25 d; basta para la SV)."""
    return 2000.0 + (float(jd_ut1) - 2451545.0) / 365.25


def _legendre(nmax, c, s):
    """Funciones de Legendre de Schmidt semi-normalizadas P_n^m(cos th), su
    derivada respecto a th (dP) y Q = P/sin th (para B_phi sin dividir por 0 en
    los polos), n <= nmax. Recurrencias estandar:
        P_nn = sqrt((2n-1)/(2n)) sin th P_(n-1)(n-1),  P_11 = sin th
        P_nm = [(2n-1) cos th P_(n-1)m - sqrt((n-1+m)(n-1-m)) P_(n-2)m] / sqrt((n+m)(n-m))
    y las derivadas de las mismas. Verificadas contra scipy.special.lpmv y
    contra diferencias finitas del potencial (test_fisica)."""
    P = np.zeros((nmax + 2, nmax + 2))
    dP = np.zeros_like(P)
    Q = np.zeros_like(P)
    P[0, 0] = 1.0
    for n in range(1, nmax + 1):
        for m in range(0, n + 1):
            if m == n:
                if n == 1:
                    P[1, 1], dP[1, 1], Q[1, 1] = s, c, 1.0
                else:
                    k = math.sqrt((2.0 * n - 1.0) / (2.0 * n))
                    P[n, n] = k * s * P[n - 1, n - 1]
                    dP[n, n] = k * (c * P[n - 1, n - 1] + s * dP[n - 1, n - 1])
                    Q[n, n] = k * P[n - 1, n - 1]
            else:
                den = math.sqrt((n + m) * (n - m))
                a = (2.0 * n - 1.0)
                b = math.sqrt(max((n - 1.0 + m) * (n - 1.0 - m), 0.0))
                P[n, m] = (a * c * P[n - 1, m] - b * P[n - 2, m]) / den
                dP[n, m] = (a * (-s * P[n - 1, m] + c * dP[n - 1, m]) - b * dP[n - 2, m]) / den
                Q[n, m] = (a * c * Q[n - 1, m] - b * Q[n - 2, m]) / den
    return P, dP, Q


def _angulos(r_ecef_km):
    r_km = np.asarray(r_ecef_km, float)
    r = float(np.linalg.norm(r_km))
    th = math.acos(max(-1.0, min(1.0, float(r_km[2]) / r)))
    return r, th, math.atan2(float(r_km[1]), float(r_km[0]))


def campo_geomagnetico(r_ecef_km, anio, nmax=NMAX_IGRF):
    """Campo B (nT) en ECEF por armonicos esfericos de Schmidt hasta grado
    `nmax` (1..13) con los coeficientes IGRF-13 (`igrf13_datos`) llevados a
    `anio` con la variacion secular:

        V = a sum_n (a/r)^(n+1) sum_m (g cos m phi + h sin m phi) P_n^m(cos th)
        B = -grad V  ->  Br, Bth, Bph  (th = colatitud GEOCENTRICA, a = 6371.2 km)

    nmax=1 es el DIPOLO INCLINADO centrado: el modelo de a bordo (barato:
    3 coeficientes) y NO es IGRF. nmax=13 es IGRF-13 completo, la 'verdad' de la
    simulacion: IGRF es el campo PRINCIPAL (interno); no incluye el campo de
    corteza residual (grados > 13), el externo (magnetosfera, ionosfera:
    decenas de nT en calma, cientos en tormenta) ni su error de ajuste
    (~decenas de nT). Valido para r > a; la extrapolacion de la variacion
    secular mas alla de 2025 (fin de IGRF-13) deriva decenas de nT en 2026-27.
    Verificacion: B = -grad V por diferencias finitas y valores de anclaje
    (SAA ~23 000 nT en superficie) en test_fisica."""
    r, th, ph = _angulos(r_ecef_km)
    c, s = math.cos(th), math.sin(th)
    P, dP, Q = _legendre(int(nmax), c, s)
    dt = anio - 2020.0
    br = bt = bp = 0.0
    ra = A_GEOMAG_KM / r
    fac = [0.0] + [ra ** (n + 2) for n in range(1, int(nmax) + 1)]
    cm = [math.cos(m * ph) for m in range(int(nmax) + 1)]
    sm = [math.sin(m * ph) for m in range(int(nmax) + 1)]
    for (n, m) in _COEF_ORDEN:
        if n > nmax:
            continue
        g0, h0, gs, hs = IGRF13_2020[(n, m)]
        g, h = g0 + gs * dt, h0 + hs * dt
        f = fac[n]
        gc = g * cm[m] + h * sm[m]
        br += (n + 1) * f * gc * P[n, m]
        bt -= f * gc * dP[n, m]
        bp += f * m * (g * sm[m] - h * cm[m]) * Q[n, m]
    er = np.array([s * math.cos(ph), s * math.sin(ph), c])
    et = np.array([c * math.cos(ph), c * math.sin(ph), -s])
    ep = np.array([-math.sin(ph), math.cos(ph), 0.0])
    return br * er + bt * et + bp * ep


def potencial_geomagnetico(r_ecef_km, anio, nmax=NMAX_IGRF):
    """V (nT km) del mismo modelo, para verificar B = -grad V."""
    r, th, ph = _angulos(r_ecef_km)
    P, _, _ = _legendre(int(nmax), math.cos(th), math.sin(th))
    dt = anio - 2020.0
    v = 0.0
    for (n, m) in _COEF_ORDEN:
        if n > nmax:
            continue
        g0, h0, gs, hs = IGRF13_2020[(n, m)]
        g, h = g0 + gs * dt, h0 + hs * dt
        v += (A_GEOMAG_KM * (A_GEOMAG_KM / r) ** (n + 1)
              * (g * math.cos(m * ph) + h * math.sin(m * ph)) * P[n, m])
    return v


def campo_eci_T(r_eci_km, jd, fr, nmax=NMAX_IGRF):
    """Campo (T) en el marco inercial TEME en la posicion r_eci (km) y el
    instante (jd, fr) UTC. Rota a ECEF con GMST (el de orbit.py)."""
    th = float(orbit.gmst_rad(jd, fr))
    c, s = math.cos(th), math.sin(th)
    r = np.asarray(r_eci_km, float)
    r_ecef = np.array([c * r[0] + s * r[1], -s * r[0] + c * r[1], r[2]])
    b = campo_geomagnetico(r_ecef, anio_decimal(jd + fr), nmax) * 1e-9
    return np.array([c * b[0] - s * b[1], s * b[0] + c * b[1], b[2]])


def error_modelo_campo(a_km=6978.0, inc_deg=97.5, nmax_bordo=1, n=180, anio=2026.75):
    """Error del modelo de a bordo (grado `nmax_bordo`) respecto a la verdad
    (IGRF-13 grado 13) a lo largo de UNA orbita circular de radio a_km e
    inclinacion inc_deg (muestreada en n puntos, con la Tierra congelada: el
    error depende de la geometria, y a bordo se evalua girando con la Tierra).

    Devuelve dict: rms_rel (|dB|/|B| rms), max_rel, rms_deg y max_deg (angulo
    entre ambos campos), rms_nT. Es la cifra que fija el ruido de medida
    del filtro: se CALCULA, no se escribe."""
    inc = math.radians(inc_deg)
    rel, ang, nt = [], [], []
    for i in range(n):
        u = 2 * math.pi * i / n
        r = a_km * np.array([math.cos(u), math.sin(u) * math.cos(inc),
                             math.sin(u) * math.sin(inc)])
        bv = campo_geomagnetico(r, anio, NMAX_IGRF)
        bm = campo_geomagnetico(r, anio, nmax_bordo)
        rel.append(np.linalg.norm(bv - bm) / np.linalg.norm(bv))
        ang.append(math.degrees(angulo_entre(bm, bv)))
        nt.append(np.linalg.norm(bv - bm))
    rel, ang, nt = map(np.array, (rel, ang, nt))
    return {'rms_rel': float(np.sqrt(np.mean(rel ** 2))), 'max_rel': float(rel.max()),
            'rms_deg': float(np.sqrt(np.mean(ang ** 2))), 'max_deg': float(ang.max()),
            'rms_nT': float(np.sqrt(np.mean(nt ** 2)))}


# ====================================================================== ambiente: Sol

def sol_eci_km(jd_ut1):
    """Posicion del Sol (km, marco ecuador/equinoccio medios de la fecha, que
    se toma como TEME) con el algoritmo 29 de Vallado (Almanaque
    Astronomico de baja precision): ~0.01 deg hasta 2050. Se desprecia la
    diferencia TEME - MOD (< 0.4 deg en 2026), que solo mueve unos segundos
    el instante del eclipse."""
    T = (float(jd_ut1) - 2451545.0) / 36525.0
    lm = 280.460 + 36000.771 * T
    M = math.radians(357.5291092 + 35999.05034 * T)
    lam = math.radians(lm + 1.914666471 * math.sin(M) + 0.019994643 * math.sin(2 * M))
    eps = math.radians(23.439291 - 0.0130042 * T)
    r = 1.000140612 - 0.016708617 * math.cos(M) - 0.000139589 * math.cos(2 * M)
    return UA_KM * r * np.array([math.cos(lam), math.cos(eps) * math.sin(lam),
                                 math.sin(eps) * math.sin(lam)])


def en_eclipse(r_sat_km, r_sol_km):
    """Eclipse con sombra CILINDRICA: el satelite esta en la sombra si esta
    detras de la Tierra (r.s < 0) y a menos de R_T del eje Tierra-Sol. Ignora la
    penumbra (cuya duracion es de ~10 s en LEO) y el aplanamiento."""
    s = unitario(r_sol_km)
    r = np.asarray(r_sat_km, float)
    along = float(r @ s)
    return bool(along < 0.0 and np.linalg.norm(r - along * s) < R_TIERRA_KM)


# ====================================================================== sensores

def medir_magnetometro(B_body_T, sigma_T, rng, bias_T=None):
    """B_medido = B + bias + ruido blanco N(0, sigma^2) por eje. Sin errores
    de escala ni de no-ortogonalidad (SUPUESTO: calibrado) ni campo propio de
    los magnetopares (en vuelo se apagan para medir)."""
    b = np.asarray(B_body_T, float) + rng.normal(0.0, sigma_T, 3)
    return b if bias_T is None else b + bias_T


class Giroscopo:
    """Giroscopo de 3 ejes con ruido de velocidad (ARW) y sesgo en paseo
    aleatorio (RRW), el modelo de Farrenkopf:

        w_m = w + b + eta_v,   db/dt = eta_u
        var(eta_v) = sigma_v^2 / dt (+ sigma_u^2 dt / 12),  var(b_{k+1}-b_k) = sigma_u^2 dt

    con dt el periodo de muestreo. Es lo que el MEKF espera (consistente con
    su Q). No modela error de escala, desalineacion ni dependencia de la
    temperatura. sigma_v en rad/s^0.5, sigma_u en rad/s^1.5."""

    def __init__(self, sigma_v, sigma_u, bias0, rng):
        self.sv, self.su = float(sigma_v), float(sigma_u)
        self.b = np.array(bias0, float)
        self.rng = rng

    def medir(self, w_true, dt):
        b_ant = self.b.copy()
        self.b = self.b + self.rng.normal(0.0, self.su * math.sqrt(dt), 3)
        ruido = self.rng.normal(0.0, math.sqrt(self.sv ** 2 / dt + self.su ** 2 * dt / 12.0), 3)
        return np.asarray(w_true, float) + 0.5 * (b_ant + self.b) + ruido


def medir_sol(s_body, sigma_rad, rng):
    """Vector unitario al Sol con error angular N(0, sigma^2) por eje
    transversal (angulo pequeno: vale para sigma < ~5 deg). Cobertura de 4 pi
    (SUPUESTO: 6 sensores, uno por cara)."""
    return _perturbar_unitario(unitario(s_body), sigma_rad, rng)


def _perturbar_unitario(s, sigma, rng):
    """s' = R(dtheta) s con dtheta perpendicular a s, de desviacion sigma por
    componente transversal."""
    d = rng.normal(0.0, sigma, 3)
    d -= s * float(s @ d)
    return unitario(s + np.cross(d, s))


# ====================================================================== estimacion

def triad(b1, r1, b2, r2):
    """TRIAD (Black 1964): actitud a partir de dos vectores observados en el
    cuerpo (b1, b2) y conocidos en el inercial (r1, r2). El PRIMERO es el
    exacto (se alinea perfectamente); el segundo solo fija el giro. Devuelve
    q cuerpo->inercial. Falla (ValueError) si los vectores son casi paralelos
    (|sin| < 1e-3). El error crece como sigma_2/sin(angulo entre vectores)."""
    b1, b2, r1, r2 = (unitario(x) for x in (b1, b2, r1, r2))
    if np.linalg.norm(np.cross(b1, b2)) < 1e-3 or np.linalg.norm(np.cross(r1, r2)) < 1e-3:
        raise ValueError('vectores casi paralelos: TRIAD mal condicionado')
    tb = np.column_stack([b1, unitario(np.cross(b1, b2)),
                          np.cross(b1, unitario(np.cross(b1, b2)))])
    ti = np.column_stack([r1, unitario(np.cross(r1, r2)),
                          np.cross(r1, unitario(np.cross(r1, r2)))])
    R = ti @ tb.T           # R b = r  (cuerpo -> inercial)
    return dcm_a_q(R)


def _series_phi(wn, dt):
    """Coeficientes (a, b, c) de Phi11 = I - a [w x] + b [w x]^2 y
    Phi12 = -I dt + c1 [w x] - c2 [w x]^2 con x = |w| dt; estables para x -> 0."""
    x = wn * dt
    if x < 1e-3:
        s_x = dt * (1 - x ** 2 / 6 + x ** 4 / 120)                  # sin(x)/|w|
        c1 = dt ** 2 * (0.5 - x ** 2 / 24 + x ** 4 / 720)           # (1-cos x)/|w|^2
        c2 = dt ** 3 * (1 / 6 - x ** 2 / 120 + x ** 4 / 5040)       # (x - sin x)/|w|^3
    else:
        s_x = math.sin(x) / wn
        c1 = (1 - math.cos(x)) / wn ** 2
        c2 = (x - math.sin(x)) / wn ** 3
    return s_x, c1, c2


class Mekf:
    """Filtro de Kalman extendido MULTIPLICATIVO de 6 estados (Lefferts,
    Markley y Shuster 1982; Markley y Crassidis 2014, cap. 7): el error de
    actitud dtheta (3, en el cuerpo, q = q^ (x) dq) y el error de sesgo del
    giroscopo db = b - b^.

    Propagacion: q^ <- q^ (x) q(w^ dt), w^ = w_m - b^ (exacta para w^ constante en
    el paso); P <- Phi P Phi^T + Qd con Phi = exp(F dt) EXACTA para
    F = [[-[w^ x], -I], [0, 0]] y Qd de primer orden (valido si |w| dt << 1;
    a 1 Hz y 0.1 rad/s el error relativo es ~|w|dt/2 en los terminos cruzados).

    Actualizacion con un vector unitario medido en el cuerpo y su direccion
    inercial conocida: y = A(q^)^T r, H = [[y x], 0], R = sigma^2 I, forma de
    Joseph. Tras la correccion se hace el 'reset' q^ <- q^ (x) q(d_theta); se
    desprecia el Jacobiano del reset (I - [d_theta/2 x]), O(d_theta)."""

    def __init__(self, q0, b0, sigma_att0, sigma_b0, sigma_v, sigma_u):
        self.q = q_norm(q0)
        self.b = np.array(b0, float)
        self.P = np.diag([sigma_att0 ** 2] * 3 + [sigma_b0 ** 2] * 3)
        self.sv, self.su = float(sigma_v), float(sigma_u)
        self.w = np.zeros(3)

    def propagar(self, w_m, dt):
        w = np.asarray(w_m, float) - self.b
        self.w = w
        wn = float(np.linalg.norm(w))
        self.q = q_norm(q_mult(self.q, q_de_rotvec(w * dt)))
        W = skew(w)
        s_x, c1, c2 = _series_phi(wn, dt)
        I3 = np.eye(3)
        phi = np.eye(6)
        phi[:3, :3] = I3 - s_x * W + c1 * (W @ W)
        phi[:3, 3:] = -I3 * dt + c1 * W - c2 * (W @ W)
        sv2, su2 = self.sv ** 2, self.su ** 2
        Q = np.zeros((6, 6))
        Q[:3, :3] = (sv2 * dt + su2 * dt ** 3 / 3.0) * I3
        Q[:3, 3:] = -(su2 * dt ** 2 / 2.0) * I3
        Q[3:, :3] = Q[:3, 3:]
        Q[3:, 3:] = su2 * dt * I3
        self.P = phi @ self.P @ phi.T + Q
        self.P = 0.5 * (self.P + self.P.T)

    def actualizar_vector(self, y_cuerpo, r_inercial, sigma_rad):
        """Correccion con un vector unitario. Devuelve (innovacion, S) para
        calcular el NIS."""
        y = unitario(y_cuerpo)
        yh = q_a_dcm(self.q).T @ unitario(r_inercial)
        H = np.zeros((3, 6))
        H[:, :3] = skew(yh)
        R = (sigma_rad ** 2) * np.eye(3)
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        dx = K @ (y - yh)
        IKH = np.eye(6) - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)
        self.q = q_norm(q_mult(self.q, q_de_rotvec(dx[:3])))
        self.b = self.b + dx[3:]
        return y - yh, S

    def sigma_actitud(self):
        """Desviacion tipica 1-sigma de cada eje de actitud (rad)."""
        return np.sqrt(np.diag(self.P)[:3])


def nees(q_est, b_est, P, q_true, b_true):
    """Error de estimacion normalizado al cuadrado de un MEKF:
    e = [dtheta; db], NEES = e^T P^-1 e (chi-cuadrado con 6 grados de libertad si
    el filtro es consistente y el error pequeno)."""
    dth = q_a_rotvec(q_error(q_est, q_true))
    e = np.concatenate([dth, np.asarray(b_true) - np.asarray(b_est)])
    return float(e @ np.linalg.solve(P, e))


_Z = {0.995: 2.5758293035489, 0.975: 1.959963984540054, 0.005: -2.5758293035489,
      0.025: -1.959963984540054}


def chi2_cuantil(p, k):
    """Cuantil p de chi-cuadrado con k grados de libertad, aproximacion de
    Wilson-Hilferty: k (1 - 2/(9k) + z_p sqrt(2/(9k)))^3. Error relativo
    < 0.3 % para k >= 30 y p entre 0.005 y 0.995 (se cruza con scipy en las
    pruebas si esta instalado). p debe ser 0.005, 0.025, 0.975 o 0.995."""
    z = _Z[p]
    a = 2.0 / (9.0 * k)
    return k * (1.0 - a + z * math.sqrt(a)) ** 3


# ====================================================================== control

def ganancia_bdot(J_min, B_rms_T, tau_s):
    """Ganancia k de m = -k dB/dt tal que el amortiguamiento promedio tenga
    constante de tiempo `tau_s` sobre el eje de MENOR inercia.

    Derivacion: con w grande frente a la velocidad orbital, dB/dt ~ -w x B en el
    cuerpo; tau = m x B = k (w x B) x B = -k B^2 [w - (w.B^)B^] (amortigua la
    componente de w perpendicular a B con tasa k B^2/J). Igualando k B^2 / J_min
    = 1/tau: k = J_min/(tau B^2). Por eso a gran velocidad el dipolo se
    SATURA (m = k w B > m_max) y la bajada es lineal en el tiempo, no
    exponencial. Vale mientras la dinamica de B por la orbita sea lenta
    frente a w (w >> 2 n)."""
    return float(J_min) / (float(tau_s) * float(B_rms_T) ** 2)


def b_rms_dipolo_T(a_km, anio):
    """|B| eficaz (T) del dipolo a radio a_km promediado sobre una esfera:
    <|B|^2> = B_ec^2 (A/r)^6 <1 + 3 sin^2 lat> = 2 B_ec^2 (A/r)^6, de modo que
    B_rms = sqrt(2) B_ec (A/r)^3, con B_ec = sqrt(g10^2 + g11^2 + h11^2) el
    campo ecuatorial del dipolo (coeficientes IGRF-13 llevados a `anio`)."""
    dt = anio - 2020.0
    g10, _, s10, _ = IGRF13_2020[(1, 0)]
    g11, h11, s11, sh11 = IGRF13_2020[(1, 1)]
    b_ec = math.sqrt((g10 + s10 * dt) ** 2 + (g11 + s11 * dt) ** 2 + (h11 + sh11 * dt) ** 2) * 1e-9
    return math.sqrt(2.0) * b_ec * (A_GEOMAG_KM / a_km) ** 3


class Bdot:
    """Ley B-dot para detumbling con magnetopares: m = -k dB/dt, saturada por
    eje, con dB/dt por diferencia hacia atras y filtro de primer orden
    (constante `tau_f`). Solo usa el magnetometro. Simplificacion: el
    magnetometro se lee mientras el magnetopar actua (en vuelo se alternan
    medida y actuacion: 'duty cycle'), que es lo que falta para hardware."""

    def __init__(self, k, m_max, tau_f=3.0):
        self.k, self.m_max, self.tau_f = float(k), float(m_max), float(tau_f)
        self.b_ant = None
        self.bdot = np.zeros(3)

    def paso(self, b_meas, dt):
        b = np.asarray(b_meas, float)
        if self.b_ant is not None:
            raw = (b - self.b_ant) / dt
            a = dt / (self.tau_f + dt)
            self.bdot = self.bdot + a * (raw - self.bdot)
        self.b_ant = b.copy()
        return np.clip(-self.k * self.bdot, -self.m_max, self.m_max)


def control_actitud(q, w, h_w, q_ref, w_ref, a_ref, sat, wn, zeta, w_lim):
    """Par deseado en el CUERPO (3,) para seguir (q_ref, w_ref, a_ref).

    Ley PD en cuaterniones con limite de velocidad (Wie, cap. 7), con
    desacople de Euler y prealimentacion de la referencia:

        dq = q_ref^-1 (x) q ,  w_e = w - R(dq)^T w_ref
        tau = w x (J w + A h) + J (R(dq)^T a_ref - w_e x R(dq)^T w_ref)
              - 2 zeta wn J [ w_e + sat(wn dq_v / zeta, w_lim) ]

    En pequeno: J dw_e/dt = -2 zeta wn J w_e - 2 wn^2 J dq_v, es decir
    dtheta'' + 2 zeta wn dtheta' + wn^2 dtheta = 0 (dq_v = dtheta/2): la
    dinamica es la de un oscilador de frecuencia wn y amortiguamiento zeta. El
    recorte `sat` (norma <= w_lim) evita que un error grande pida una
    velocidad que las ruedas no pueden absorber (w_lim ~ h_max / J).
    `w_ref`, `a_ref` en el marco de la referencia."""
    dq = q_error(q_ref, q)
    Rt = q_a_dcm(dq).T
    wr = Rt @ w_ref
    ar = Rt @ a_ref
    we = w - wr
    p = wn * dq[1:] / zeta
    n = float(np.linalg.norm(p))
    if n > w_lim:
        p = p * (w_lim / n)
    return (np.cross(w, sat.J @ w + sat.A @ h_w) + sat.J @ (ar - np.cross(we, wr))
            - 2.0 * zeta * wn * (sat.J @ (we + p)))


def asignar_ruedas(tau_cuerpo, sat):
    """Par de motor por rueda que da `tau_cuerpo` en el cuerpo: el cuerpo recibe
    -A u, asi que u = -pinv(A) tau (minima norma si hay mas de 3 ruedas)."""
    return -np.linalg.pinv(sat.A) @ np.asarray(tau_cuerpo, float)


def desaturar(B_body_T, h_cuerpo, tau_s, m_max):
    """Dipolo para descargar las ruedas: m = -(1/tau) (B x h)/|B|^2.

    Derivacion: el par magnetico es tau_m = m x B = -(1/tau_d) h_perp, la
    componente de h perpendicular a B; las ruedas la absorben bajando su
    momento con constante de tiempo tau_d. La componente de h paralela a B no
    se puede descargar en ese instante (la orbita gira B y la expone despues).
    Se satura por eje a m_max."""
    B = np.asarray(B_body_T, float)
    b2 = float(B @ B)
    if b2 < EPS:
        return np.zeros(3)
    m = -np.cross(B, np.asarray(h_cuerpo, float)) / (float(tau_s) * b2)
    return np.clip(m, -m_max, m_max)


def error_apuntamiento_deg(q, eje_cuerpo, dir_inercial):
    """Angulo (deg) entre el eje del cuerpo (p. ej. el bore-sight +Z) y una
    direccion inercial. Es el error de apuntamiento de la antena."""
    return math.degrees(angulo_entre(q_a_dcm(q) @ np.asarray(eje_cuerpo, float),
                                     dir_inercial))
