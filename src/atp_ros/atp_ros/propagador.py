"""Propagador numerico de orbita para la Estacion: gravedad central + zonales
J2 (J3, J4 opcionales) + arrastre atmosferico, con RK4 de paso fijo,
Dormand-Prince 5(4) adaptativo y matriz de transicion de estado (STM).

Solo numpy (sin ROS, sin scipy), para probarlo y reutilizarlo fuera de los
nodos. Es el motor del "gemelo" de la orbita: la verdad de los experimentos
de determinacion de orbita (`od.py`) y el modelo dinamico del estimador.

MARCO (decision documentada). Las ecuaciones se escriben en TEME (True
Equator, Mean Equinox) de la epoca: el mismo marco en que SGP4 entrega el
estado y en que `orbit.py` calcula GMST. Asi la comparacion con SGP4 y la
cadena TEME -> ECEF -> ENU de `orbit.py` no necesitan ninguna rotacion
intermedia. TEME NO es inercial: rota respecto de J2000 por precesion y
nutacion (~50 arcsec/anio). Tratarlo como inercial equivale a ignorar una
fuerza ficticia de Coriolis 2 W x v ~ 1e-10 km/s^2 (frente a 8e-3 de la
gravedad); en una orbita se traduce en girar el plano orbital como un todo,
~15 m en 3 dias de LEO. Se desprecia: es 2-3 ordenes menor que los errores
que se estudian (km). El eje de J2 es el z de TEME (polo verdadero de la
epoca): ahi si es exacto durante el arco de dias que se propaga.

FUERZAS (cada una dice lo que desprecia):
  * Gravedad central y zonales J2, J3, J4 (Vallado 2013, Eq. 8-30..8-31,
    derivadas del potencial zonal). Desprecia los armonicos tesserales y de
    orden superior (~1e-6 de la gravedad central, 10-100 m/dia en LEO).
  * Arrastre: a = -1/2 (Cd A/m) rho |v_rel| v_rel con v_rel = v - w x r
    (atmosfera que co-rota con la Tierra). Desprecia los vientos
    (~100 m/s sobre 7.6 km/s: ~1 %), la variacion diurna y con la actividad
    solar (el factor `rho_factor` y el estado `p` la absorben en gran parte).
  * No incluye presion de radiacion, tercer cuerpo ni mareas (decenas de
    metros por dia en LEO a ~400 km).
Atmosfera: exponencial por capas, tabla de Vallado y McClain (2013), Tabla
8-4 (basada en CIRA-72); ver `ATMOSFERA_VALLADO`.

La derivada de la aceleracion (para la STM) se obtiene por PASO COMPLEJO
(Squire y Trapp 1998), exacta a precision de maquina y valida para CUALQUIER
combinacion de fuerzas: todo lo de `aceleracion` es analitico en el plano
complejo (solo sqrt, exp y productos; las decisiones discretas, como la capa
de la atmosfera, usan la parte real). Las pruebas lo contrastan con
diferencias finitas.

Unidades: km, s, rad; densidad en kg/m3; Cd A/m en m2/kg.
"""
import math
from dataclasses import dataclass, field, replace

import numpy as np

# ---------------------------------------------------------------- constantes

OMEGA_TIERRA = 7.2921158553e-5   # rad/s, igual que orbit.OMEGA_TIERRA


@dataclass(frozen=True)
class Gravedad:
    """Campo gravitatorio: mu (km3/s2), radio ecuatorial (km), J2, J3, J4."""
    nombre: str
    mu: float
    re: float
    j2: float
    j3: float
    j4: float


# Valores de sgp4.earth_gravity (las pruebas los cotejan con la libreria):
# WGS72 es lo que SGP4 espera de un TLE; WGS84 es el de la Estacion (orbit.py).
WGS72 = Gravedad('wgs72', 398600.8, 6378.135, 0.001082616,
                 -0.00000253881, -0.00000165597)
WGS84 = Gravedad('wgs84', 398600.5, 6378.137, 0.00108262998905,
                 -0.00000253215306, -0.00000161098761)
F_APLANAMIENTO = 1.0 / 298.257223563     # WGS84, para la altitud geodesica


# ---------------------------------------------------------------- atmosfera

# Vallado y McClain, "Fundamentals of Astrodynamics and Applications", 4a ed.
# (2013), Tabla 8-4: atmosfera exponencial por capas, rho = rho0 exp(-(h-h0)/H),
# con (h0 km, rho0 kg/m3, H km) para cada capa. Basada en CIRA-72, es una
# atmosfera MEDIA: puede errar por un factor ~2 segun la actividad solar y la
# hora local (a 400 km, de 1e-12 a 1e-11 kg/m3). Las pruebas verifican la
# continuidad entre capas (la tabla se transcribio; una cifra mal copiada
# romperia la continuidad) y la contrastan con la US Standard Atmosphere 1976.
ATMOSFERA_VALLADO = (
    (0.0, 1.225, 7.249), (25.0, 3.899e-2, 6.349), (30.0, 1.774e-2, 6.682),
    (40.0, 3.972e-3, 7.554), (50.0, 1.057e-3, 8.382), (60.0, 3.206e-4, 7.714),
    (70.0, 8.770e-5, 6.549), (80.0, 1.905e-5, 5.799), (90.0, 3.396e-6, 5.382),
    (100.0, 5.297e-7, 5.877), (110.0, 9.661e-8, 7.263), (120.0, 2.438e-8, 9.473),
    (130.0, 8.484e-9, 12.636), (140.0, 3.845e-9, 16.149),
    (150.0, 2.070e-9, 22.523), (180.0, 5.464e-10, 29.740),
    (200.0, 2.789e-10, 37.105), (250.0, 7.248e-11, 45.546),
    (300.0, 2.418e-11, 53.628), (350.0, 9.518e-12, 53.298),
    (400.0, 3.725e-12, 58.515), (450.0, 1.585e-12, 60.828),
    (500.0, 6.967e-13, 63.822), (600.0, 1.454e-13, 71.835),
    (700.0, 3.614e-14, 88.667), (800.0, 1.170e-14, 124.64),
    (900.0, 5.245e-15, 181.05), (1000.0, 3.019e-15, 268.00),
)


class AtmosferaExp:
    """Densidad exponencial por capas. Vale entre 0 y ~1500 km (por encima de
    1000 km se extrapola la ultima capa; ahi el arrastre es despreciable).
    Acepta arrays reales o complejos (la capa se elige con la parte real, para
    el paso complejo de la STM)."""

    def __init__(self, tabla=ATMOSFERA_VALLADO):
        t = np.asarray(tabla, dtype=float)
        self.h0, self.rho0, self.esc = t[:, 0], t[:, 1], t[:, 2]

    def densidad(self, h_km):
        h = np.asarray(h_km)
        i = np.clip(np.searchsorted(self.h0, np.real(h), side='right') - 1,
                    0, len(self.h0) - 1)
        return self.rho0[i] * np.exp(-(h - self.h0[i]) / self.esc[i])


ATMOSFERA = AtmosferaExp()


# ---------------------------------------------------------------- modelo

@dataclass(frozen=True)
class Modelo:
    """Fuerzas del propagador.

    grado: 0 = solo gravedad central, 2 = + J2, 3 = + J3, 4 = + J4.
    arrastre: si se incluye; `cd_a_m` es el coeficiente balistico NOMINAL
    (Cd A / m, m2/kg) y `rho_factor` escala la densidad de la atmosfera
    (incertidumbre del modelo de atmosfera). El estado puede llevar ademas un
    factor `p` que multiplica a `cd_a_m` y se estima en la OD."""
    grav: Gravedad = WGS84
    grado: int = 2
    arrastre: bool = True
    cd_a_m: float = 0.0105     # ISS: ~2.2 * 1600 m2 / 335000 kg = 0.0105
    rho_factor: float = 1.0
    atm: AtmosferaExp = field(default=ATMOSFERA, compare=False, repr=False)
    omega_atm: float = OMEGA_TIERRA
    # Variacion temporal de la densidad (SOLO para generar una "verdad" mas
    # realista que un factor constante): rho *= 1 + sum amp sin(2 pi t/T + fase)
    # con terminos (amp, periodo_s, fase_rad), t en segundos desde la epoca de
    # integracion. Solo `derivadas` (propagacion de estado) la aplica; la
    # aceleracion vectorizada y el jacobiano/STM del estimador usan el modelo
    # SIN variacion: es error de modelo a proposito.
    rho_var: tuple = ()

    def con(self, **kw):
        return replace(self, **kw)


# ---------------------------------------------------------------- aceleracion

def _legendre(n, s):
    """(P_n(s), P_n'(s)) para n = 2, 3, 4 (Vallado, polinomios de Legendre)."""
    if n == 2:
        return 0.5 * (3.0 * s * s - 1.0), 3.0 * s
    if n == 3:
        return 0.5 * (5.0 * s ** 3 - 3.0 * s), 0.5 * (15.0 * s * s - 3.0)
    if n == 4:
        return ((35.0 * s ** 4 - 30.0 * s * s + 3.0) / 8.0,
                (140.0 * s ** 3 - 60.0 * s) / 8.0)
    raise ValueError(n)


def aceleracion_gravedad(r, grav, grado):
    """Aceleracion (km/s2) de la gravedad central + zonales hasta `grado`
    (2, 3 o 4), r de forma (N, 3) en km. Del potencial
    U_n = -mu J_n Re^n P_n(z/r) / r^(n+1):
        a_x = mu J_n Re^n x r^-(n+3) [(n+1) P_n + s P_n']
        a_z = -mu J_n Re^n r^-(n+2) [-(n+1) s P_n + (1-s^2) P_n']
    (y como x). Para n = 2 reproduce la forma clasica de Vallado Eq. 8-31.
    Vale para r > Re; desprecia los demas armonicos."""
    x, y, z = r[:, 0], r[:, 1], r[:, 2]
    r2 = x * x + y * y + z * z
    rn = np.sqrt(r2)
    mu = grav.mu
    f0 = -mu / (r2 * rn)
    a = np.stack([f0 * x, f0 * y, f0 * z], axis=1)
    s = z / rn
    for n, jn in ((2, grav.j2), (3, grav.j3), (4, grav.j4)):
        if n > grado:
            break
        pn, dpn = _legendre(n, s)
        k = mu * jn * grav.re ** n
        fxy = k * rn ** (-(n + 3)) * ((n + 1) * pn + s * dpn)
        fz = -k * rn ** (-(n + 2)) * (-(n + 1) * s * pn + (1.0 - s * s) * dpn)
        a = a + np.stack([fxy * x, fxy * y, fz], axis=1)
    return a


def altitud_aprox(r, grav):
    """Altitud (km) sobre el elipsoide, primer orden en el aplanamiento:
    h = r - Re (1 - f sin^2 phi), sin phi = z/r (latitud geocentrica). Error
    < ~0.1 km en LEO frente a la altitud geodesica exacta (irrelevante para
    una atmosfera de escala ~50 km)."""
    r2 = np.sum(r * r, axis=1)
    rn = np.sqrt(r2)
    s2 = r[:, 2] ** 2 / r2
    return rn - grav.re * (1.0 - F_APLANAMIENTO * s2)


def aceleracion_arrastre(r, v, p, modelo):
    """Arrastre (km/s2): -1/2 B rho |v_rel| v_rel, B = cd_a_m * p (m2/kg),
    rho en kg/m3, v_rel (km/s) = v - w x r. El factor 1e3 convierte
    (m2/kg)(kg/m3)(km/s)^2 en km/s2. `p` es un array (N,)."""
    vrel = np.stack([v[:, 0] + modelo.omega_atm * r[:, 1],
                     v[:, 1] - modelo.omega_atm * r[:, 0], v[:, 2]], axis=1)
    vn = np.sqrt(np.sum(vrel * vrel, axis=1))     # sin conjugar: paso complejo
    rho = modelo.rho_factor * modelo.atm.densidad(altitud_aprox(r, modelo.grav))
    k = -0.5 * modelo.cd_a_m * p * rho * vn * 1e3
    return vrel * k[:, None]


def aceleracion(r, v, p, modelo):
    """Aceleracion total (km/s2). r, v de forma (N, 3), p de forma (N,)."""
    a = aceleracion_gravedad(r, modelo.grav, modelo.grado)
    if modelo.arrastre:
        a = a + aceleracion_arrastre(r, v, p, modelo)
    return a


def acel1(r, v, modelo, p=1.0):
    """Aceleracion de UN estado (arrays (3,)) -> (3,)."""
    return aceleracion(np.asarray(r, float)[None], np.asarray(v, float)[None],
                       np.array([float(p)]), modelo)[0]


# ---------------------------------------------------------------- derivadas

def aceleracion_escalar(modelo):
    """Version en Python puro (floats) de `aceleracion` para UN estado real:
    devuelve g(x, y, z, vx, vy, vz, p) -> (ax, ay, az). Es ~20 veces mas
    rapida que la vectorizada para N = 1 (la sobrecarga de numpy domina), y es
    la que usa `derivadas` en la propagacion de la verdad. Las pruebas exigen
    que coincida con `aceleracion` a precision de maquina."""
    gr = modelo.grav
    mu, re, grado = gr.mu, gr.re, modelo.grado
    zon = [(n, mu * jn * re ** n) for n, jn in ((2, gr.j2), (3, gr.j3), (4, gr.j4))
           if n <= grado]
    atm = modelo.atm
    h0, rho0, esc = list(atm.h0), list(atm.rho0), list(atm.esc)
    nh = len(h0)
    w = modelo.omega_atm
    cd = modelo.cd_a_m
    rf = modelo.rho_factor
    arr = modelo.arrastre
    f = F_APLANAMIENTO

    def g(x, y, z, vx, vy, vz, p, fm=1.0):
        r2 = x * x + y * y + z * z
        rn = math.sqrt(r2)
        f0 = -mu / (r2 * rn)
        ax, ay, az = f0 * x, f0 * y, f0 * z
        if zon:
            s = z / rn
            for n, k in zon:
                if n == 2:
                    pn, dpn = 0.5 * (3.0 * s * s - 1.0), 3.0 * s
                elif n == 3:
                    pn, dpn = 0.5 * (5.0 * s ** 3 - 3.0 * s), 0.5 * (15.0 * s * s - 3.0)
                else:
                    pn = (35.0 * s ** 4 - 30.0 * s * s + 3.0) / 8.0
                    dpn = (140.0 * s ** 3 - 60.0 * s) / 8.0
                fxy = k * rn ** (-(n + 3)) * ((n + 1) * pn + s * dpn)
                fz = -k * rn ** (-(n + 2)) * (-(n + 1) * s * pn + (1.0 - s * s) * dpn)
                ax += fxy * x
                ay += fxy * y
                az += fz
        if arr:
            ux, uy = vx + w * y, vy - w * x
            vn = math.sqrt(ux * ux + uy * uy + vz * vz)
            h = rn - re * (1.0 - f * z * z / r2)
            i = 0
            while i + 1 < nh and h0[i + 1] <= h:
                i += 1
            rho = rf * rho0[i] * math.exp(-(h - h0[i]) / esc[i])
            kk = -0.5 * cd * p * fm * rho * vn * 1e3
            ax += kk * ux
            ay += kk * uy
            az += kk * vz
        return ax, ay, az
    return g


def derivadas(modelo):
    """f(t, y) para y = [r(3), v(3)] o [r, v, p] (7 elementos), estado REAL
    (usa la aceleracion escalar rapida). Aplica `modelo.rho_var` (variacion
    temporal de la densidad) si la hay; t son los segundos del integrador."""
    g = aceleracion_escalar(modelo)
    var = [(a, 2.0 * math.pi / T, ph) for a, T, ph in modelo.rho_var]

    def f(t, y):
        n = len(y)
        fm = 1.0
        for a, w, ph in var:
            fm += a * math.sin(w * t + ph)
        ax, ay, az = g(y[0], y[1], y[2], y[3], y[4], y[5], y[6] if n > 6 else 1.0, fm)
        out = np.empty(n)
        out[0:3] = y[3:6]
        out[3] = ax
        out[4] = ay
        out[5] = az
        if n > 6:
            out[6] = 0.0
        return out
    return f


_H_CS = 1e-30     # paso complejo: sin cancelacion, el error es O(h^2) ~ 0


def jacobiano_aceleracion(modelo, r, v, p=1.0, con_p=False):
    """(a, A): aceleracion (3,) y matriz A = d(r', v')/d(r, v[, p]) de
    6x6 (o 6x7 con `con_p`), por paso complejo, en UNA llamada vectorizada."""
    m = 7 if con_p else 6
    R = np.tile(np.asarray(r, float), (m + 1, 1)).astype(complex)
    V = np.tile(np.asarray(v, float), (m + 1, 1)).astype(complex)
    P = np.full(m + 1, float(p), dtype=complex)
    for j in range(3):
        R[1 + j, j] += 1j * _H_CS
        V[4 + j, j] += 1j * _H_CS
    if con_p:
        P[7] += 1j * _H_CS
    a = aceleracion(R, V, P, modelo)
    A = np.zeros((6, m))
    A[0:3, 3:6] = np.eye(3)
    A[3:6, :] = (a[1:m + 1].imag / _H_CS).T
    return a[0].real, A


def jacobiano_escalar(modelo):
    """Version analitica y en Python puro de `jacobiano_aceleracion`:
    devuelve J(x, y, z, vx, vy, vz, p) -> (a(3), dadr(3x3), dadv(3x3),
    dadp(3)) como arrays de numpy. Deriva a mano las tres fuerzas: gradiente
    de la central y de los zonales (a_x = x f_xy(r, s), a_z = f_z(r, s), con
    s = z/r) y el del arrastre (incluida d rho / d r por la altitud). Es ~8
    veces mas rapida que el paso complejo; las pruebas la contrastan con este
    a precision de maquina."""
    gr = modelo.grav
    mu, re, grado = gr.mu, gr.re, modelo.grado
    zon = [(n, mu * jn * re ** n) for n, jn in ((2, gr.j2), (3, gr.j3), (4, gr.j4))
           if n <= grado]
    atm = modelo.atm
    h0, rho0, esc = list(atm.h0), list(atm.rho0), list(atm.esc)
    nh = len(h0)
    w, cd, rf, arr, fl = (modelo.omega_atm, modelo.cd_a_m, modelo.rho_factor,
                          modelo.arrastre, F_APLANAMIENTO)

    def jac(x, y, z, vx, vy, vz, p):
        r2 = x * x + y * y + z * z
        rn = math.sqrt(r2)
        r3 = r2 * rn
        f0 = -mu / r3
        c3 = 3.0 * mu / (r3 * r2)
        a = [f0 * x, f0 * y, f0 * z]
        # central: -mu (I / r^3 - 3 r r^T / r^5)
        J = [[f0 + c3 * x * x, c3 * x * y, c3 * x * z],
             [c3 * x * y, f0 + c3 * y * y, c3 * y * z],
             [c3 * x * z, c3 * y * z, f0 + c3 * z * z]]
        if zon:
            s = z / rn
            s2 = s * s
            for n, k in zon:
                if n == 2:
                    pn, d1, d2 = 0.5 * (3.0 * s2 - 1.0), 3.0 * s, 3.0
                elif n == 3:
                    pn, d1, d2 = (0.5 * (5.0 * s ** 3 - 3.0 * s),
                                  0.5 * (15.0 * s2 - 3.0), 15.0 * s)
                else:
                    pn = (35.0 * s ** 4 - 30.0 * s2 + 3.0) / 8.0
                    d1 = (140.0 * s ** 3 - 60.0 * s) / 8.0
                    d2 = (420.0 * s2 - 60.0) / 8.0
                F = (n + 1) * pn + s * d1
                dF = (n + 2) * d1 + s * d2
                G = (n + 1) * s * pn - (1.0 - s2) * d1
                dG = (n + 1) * pn + (n + 3) * s * d1 - (1.0 - s2) * d2
                fxy = k * rn ** (-(n + 3)) * F
                fz = k * rn ** (-(n + 2)) * G
                kx = -k * rn ** (-(n + 5)) * ((n + 3) * F + s * dF)       # d fxy / dx / x
                kz = k * rn ** (-(n + 4)) * (-(n + 3) * s * F + (1.0 - s2) * dF)
                gx = -k * rn ** (-(n + 4)) * ((n + 2) * G + s * dG)       # d fz / dx / x
                gz = k * rn ** (-(n + 3)) * (-(n + 2) * s * G + (1.0 - s2) * dG)
                a[0] += fxy * x
                a[1] += fxy * y
                a[2] += fz
                J[0][0] += fxy + x * x * kx
                J[0][1] += x * y * kx
                J[0][2] += x * kz
                J[1][0] += x * y * kx
                J[1][1] += fxy + y * y * kx
                J[1][2] += y * kz
                J[2][0] += gx * x
                J[2][1] += gx * y
                J[2][2] += gz
        Jv = [[0.0] * 3 for _ in range(3)]
        dp = [0.0, 0.0, 0.0]
        if arr:
            ux, uy, uz = vx + w * y, vy - w * x, vz
            un = math.sqrt(ux * ux + uy * uy + uz * uz)
            h = rn - re * (1.0 - fl * z * z / r2)
            i = 0
            while i + 1 < nh and h0[i + 1] <= h:
                i += 1
            rho = rf * rho0[i] * math.exp(-(h - h0[i]) / esc[i])
            kap = -0.5 * cd * p * 1e3
            kr = kap * rho
            ad = [kr * un * ux, kr * un * uy, kr * un * uz]
            a[0] += ad[0]
            a[1] += ad[1]
            a[2] += ad[2]
            dp = [ad[0] / p, ad[1] / p, ad[2] / p]
            u = (ux, uy, uz)
            # d(|u| u)/du = |u| I + u u^T / |u|
            B = [[kr * ((un if i_ == j_ else 0.0) + u[i_] * u[j_] / un)
                  for j_ in range(3)] for i_ in range(3)]
            Jv = B
            # du/dr: ux depende de y (+w), uy de x (-w)
            for i_ in range(3):
                J[i_][0] += -w * B[i_][1]
                J[i_][1] += w * B[i_][0]
            # d rho / d r = -rho / H dh/dr
            r4 = r2 * r2
            dh = (x / rn - 2.0 * re * fl * z * z * x / r4,
                  y / rn - 2.0 * re * fl * z * z * y / r4,
                  z / rn + 2.0 * re * fl * z * (r2 - z * z) / r4)
            gk = kap * un * (-rho / esc[i])
            for i_ in range(3):
                for j_ in range(3):
                    J[i_][j_] += gk * u[i_] * dh[j_]
        return np.array(a), np.array(J), np.array(Jv), np.array(dp)
    return jac


def derivadas_stm(modelo, con_p=False):
    """f(t, y) para y = [x(n), Phi(n*n)], n = 6 (o 7 con el factor de arrastre
    `p`): ecuaciones variacionales Phi' = A Phi, Phi(t0) = I, con A del
    jacobiano analitico (`jacobiano_escalar`)."""
    n = 7 if con_p else 6
    jac = jacobiano_escalar(modelo)

    def f(_t, y):
        a, dr, dv, dp = jac(y[0], y[1], y[2], y[3], y[4], y[5], y[6] if con_p else 1.0)
        Phi = y[n:].reshape(n, n)
        out = np.empty_like(y)
        out[0:3] = y[3:6]
        out[3:6] = a
        dPhi = out[n:].reshape(n, n)       # vista: se escribe en `out`
        dPhi[0:3] = Phi[3:6]
        dPhi[3:6] = dr @ Phi[0:3] + dv @ Phi[3:6]
        if con_p:
            out[6] = 0.0
            dPhi[3:6] += np.outer(dp, Phi[6])
            dPhi[6] = 0.0
        return out
    return f


_I3 = np.eye(3)


def y0_stm(x0):
    """Estado inicial de las ecuaciones variacionales: [x0, I]."""
    x0 = np.asarray(x0, float)
    return np.concatenate([x0, np.eye(len(x0)).ravel()])


# ---------------------------------------------------------------- integradores

def rk4_paso(f, t, y, h):
    k1 = f(t, y)
    k2 = f(t + h / 2, y + h / 2 * k1)
    k3 = f(t + h / 2, y + h / 2 * k2)
    k4 = f(t + h, y + h * k3)
    return y + h / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)


# Dormand-Prince 5(4) (Dormand y Prince 1980); FSAL.
_DP_A = (
    (),
    (1 / 5,),
    (3 / 40, 9 / 40),
    (44 / 45, -56 / 15, 32 / 9),
    (19372 / 6561, -25360 / 2187, 64448 / 6561, -212 / 729),
    (9017 / 3168, -355 / 33, 46732 / 5247, 49 / 176, -5103 / 18656),
    (35 / 384, 0.0, 500 / 1113, 125 / 192, -2187 / 6784, 11 / 84),
)
_DP_C = (0.0, 1 / 5, 3 / 10, 4 / 5, 8 / 9, 1.0, 1.0)
_DP_B5 = _DP_A[6] + (0.0,)
_DP_B4 = (5179 / 57600, 0.0, 7571 / 16695, 393 / 640, -92097 / 339200,
          187 / 2100, 1 / 40)


def dp45_paso(f, t, y, h, k1):
    """Un paso de Dormand-Prince. Devuelve (y5, error, k7); k7 = f(t+h, y5)
    sirve de k1 del paso siguiente (FSAL)."""
    ks = [k1]
    for i in range(1, 7):
        yi = y + h * sum(a * k for a, k in zip(_DP_A[i], ks))
        ks.append(f(t + _DP_C[i] * h, yi))
    y5 = y + h * sum(b * k for b, k in zip(_DP_B5[:6], ks[:6]))
    # ks[6] = f(t+h, y5): es el k7 (FSAL)
    err = h * sum((b5 - b4) * k for b5, b4, k in zip(_DP_B5, _DP_B4, ks))
    return y5, err, ks[6]


def propagar(f, y0, t0, ts, metodo='dp45', h=10.0, rtol=1e-9, atol=1e-9,
             h_max=120.0, atol_vec=None):
    """Integra y' = f(t, y) desde (t0, y0) y devuelve el estado en cada
    instante de `ts` (monotono, creciente o decreciente), array (len(ts), n).

    metodo 'rk4': paso fijo `h` (se reparte parejo en cada intervalo).
    metodo 'dp45': adaptativo con tolerancia mixta atol + rtol |y| por
    componente (norma infinito) y paso maximo `h_max`; aterriza exacto en
    cada elemento de `ts`. Con rtol = atol = 1e-9 (defecto) el error de
    posicion en LEO es del orden de milimetros por hora y ~1 cm por dia (las
    pruebas lo comparan contra RK4 con h pequeno y contra la conservacion de
    energia: deriva relativa < 1e-8 en un dia). `atol_vec` permite
    tolerancias absolutas distintas por componente."""
    y = np.asarray(y0, float).copy()
    t = float(t0)
    ts = np.atleast_1d(np.asarray(ts, float))
    out = np.empty((len(ts), len(y)))
    atol = np.asarray(atol if atol_vec is None else atol_vec, float)
    k1, h_nat = None, h
    for idx, tf in enumerate(ts):
        d = tf - t
        if d == 0.0:
            out[idx] = y
            continue
        if metodo == 'rk4':
            n = max(1, int(math.ceil(abs(d) / h - 1e-9)))
            hh = d / n
            for _ in range(n):
                y = rk4_paso(f, t, y, hh)
                t += hh
            t = tf
        elif metodo == 'dp45':
            sgn = 1.0 if d > 0 else -1.0
            if k1 is None:
                k1 = f(t, y)
                h_nat = h
            if h_nat * sgn < 0:
                h_nat = -h_nat
            while (tf - t) * sgn > 1e-9:
                hh = h_nat
                if (t + hh - tf) * sgn > 0:
                    hh = tf - t            # aterriza en tf; h_nat no cambia
                y5, err, k7 = dp45_paso(f, t, y, hh, k1)
                esc = atol + rtol * np.maximum(np.abs(y), np.abs(y5))
                e = float(np.max(np.abs(err) / esc))
                fac = min(5.0, max(0.2, 0.9 * (e ** -0.2 if e > 1e-10 else 5.0)))
                if e <= 1.0 or abs(hh) < 1e-6:
                    t += hh
                    y, k1 = y5, k7
                    if hh == h_nat or fac < 1.0:
                        h_nat = hh * fac
                else:
                    h_nat = hh * fac
                if abs(h_nat) > h_max:
                    h_nat = sgn * h_max
            t = tf
        else:
            raise ValueError(f'metodo desconocido: {metodo}')
        out[idx] = y
    return out


# ---------------------------------------------------------------- utilidades

def energia(modelo_grav, r, v):
    """Energia especifica (km2/s2) con gravedad central: v^2/2 - mu/r."""
    r, v = np.atleast_2d(r), np.atleast_2d(v)
    return 0.5 * np.sum(v * v, axis=1) - modelo_grav.mu / np.sqrt(np.sum(r * r, axis=1))


def elementos_clasicos(r, v, mu):
    """(a km, e, i rad, RAAN rad, arg. perigeo rad, anomalia verdadera rad)
    de un estado cartesiano, arrays (N, 3). Vale para e > ~1e-4 (el perigeo y
    la anomalia no estan definidos en una orbita circular) y orbitas no
    ecuatoriales. Vallado, Algoritmo 9."""
    r, v = np.atleast_2d(r), np.atleast_2d(v)
    rn = np.linalg.norm(r, axis=1)
    h = np.cross(r, v)
    hn = np.linalg.norm(h, axis=1)
    nodo = np.stack([-h[:, 1], h[:, 0], np.zeros(len(r))], axis=1)
    nn = np.linalg.norm(nodo, axis=1)
    rv = np.sum(r * v, axis=1)
    ev = ((np.sum(v * v, axis=1) - mu / rn)[:, None] * r - rv[:, None] * v) / mu
    e = np.linalg.norm(ev, axis=1)
    a = 1.0 / (2.0 / rn - np.sum(v * v, axis=1) / mu)
    i = np.arccos(h[:, 2] / hn)
    raan = np.arctan2(nodo[:, 1], nodo[:, 0])
    cosw = np.sum(nodo * ev, axis=1) / (nn * e)
    w = np.arccos(np.clip(cosw, -1, 1))
    w = np.where(ev[:, 2] < 0, 2 * np.pi - w, w)
    cosnu = np.sum(ev * r, axis=1) / (e * rn)
    nu = np.arccos(np.clip(cosnu, -1, 1))
    nu = np.where(rv < 0, 2 * np.pi - nu, nu)
    return a, e, i, raan, w, nu


def tasas_seculares_j2(a, e, i, grav=WGS84):
    """(RAAN', w') en rad/s por J2 (Vallado 2013, Eq. 9-41 y 9-42):
        RAAN' = -3/2 J2 (Re/p)^2 n cos i
        w'    =  3/4 J2 (Re/p)^2 n (4 - 5 sin^2 i)
    con p = a (1 - e^2), n = sqrt(mu/a^3) los MEDIOS. Vale a primer orden en
    J2 (el error relativo es O(J2) ~ 1e-3) y e < ~0.3."""
    p = a * (1.0 - e * e)
    n = math.sqrt(grav.mu / a ** 3)
    k = grav.j2 * (grav.re / p) ** 2 * n
    return -1.5 * k * math.cos(i), 0.75 * k * (4.0 - 5.0 * math.sin(i) ** 2)


# ---------------------------------------------------------------- efemeride

class Efemeride:
    """Trayectoria muestreada (t en s desde una epoca de referencia, r, v, a)
    con interpolacion de Hermite QUINTICA: la posicion, la velocidad (como
    derivada exacta del polinomio) y la aceleracion en los nudos son
    continuas. Error ~ (n h)^6 R: con rejilla de 30 s en LEO, < 1 mm.
    Sirve para evaluar la orbita en miles de instantes sin reintegrar."""

    def __init__(self, t, r, v, a):
        self.t, self.r, self.v, self.a = (np.asarray(x, float) for x in (t, r, v, a))

    @classmethod
    def desde_modelo(cls, modelo, y0, t0, ts, p=1.0, **kw):
        """Integra desde (t0, y0 = [r, v]) y guarda la rejilla `ts`."""
        ts = np.asarray(ts, float)
        Y = propagar(derivadas(modelo), np.asarray(y0, float), t0, ts, **kw)
        r, v = Y[:, 0:3], Y[:, 3:6]
        a = aceleracion(r, v, np.full(len(ts), float(p)), modelo)
        return cls(ts, r, v, a)

    def estado(self, tq):
        """(r, v) en los instantes `tq` (dentro de la rejilla)."""
        tq = np.atleast_1d(np.asarray(tq, float))
        i = np.clip(np.searchsorted(self.t, tq, side='right') - 1, 0, len(self.t) - 2)
        h = (self.t[i + 1] - self.t[i])[:, None]
        s = ((tq - self.t[i])[:, None]) / h
        s2, s3, s4, s5 = s * s, s ** 3, s ** 4, s ** 5
        h0 = 1 - 10 * s3 + 15 * s4 - 6 * s5
        h1 = s - 6 * s3 + 8 * s4 - 3 * s5
        h2 = 0.5 * s2 - 1.5 * s3 + 1.5 * s4 - 0.5 * s5
        h3 = 10 * s3 - 15 * s4 + 6 * s5
        h4 = -4 * s3 + 7 * s4 - 3 * s5
        h5 = 0.5 * s3 - s4 + 0.5 * s5
        d0 = -30 * s2 + 60 * s3 - 30 * s4
        d1 = 1 - 18 * s2 + 32 * s3 - 15 * s4
        d2 = s - 4.5 * s2 + 6 * s3 - 2.5 * s4
        d3 = 30 * s2 - 60 * s3 + 30 * s4
        d4 = -12 * s2 + 28 * s3 - 15 * s4
        d5 = 1.5 * s2 - 4 * s3 + 2.5 * s4
        r0, v0, a0 = self.r[i], self.v[i], self.a[i]
        r1, v1, a1 = self.r[i + 1], self.v[i + 1], self.a[i + 1]
        r = h0 * r0 + h1 * h * v0 + h2 * h * h * a0 + h3 * r1 + h4 * h * v1 + h5 * h * h * a1
        v = (d0 * r0 / h + d1 * v0 + d2 * h * a0 + d3 * r1 / h + d4 * v1 + d5 * h * a1)
        return r, v
