"""Control avanzado de la montura (sin ROS, solo numpy): Riccati, LQR con
accion integral, margenes, filtro de Kalman y campana Monte Carlo por
controlador.

Portado y ampliado de codeaerospace_contenido/.../atp.py (curso 30, leccion
3.1: `lqr`, `lqr_doble_integrador`, `margenes`, `margen_fase_con_retardo`,
`controlabilidad`) para que la Estacion calcule lo que el curso explica.
No se usa scipy (no esta garantizado en la imagen): Riccati continua por la
matriz hamiltoniana y Riccati discreta por el algoritmo de duplicacion
(SDA, Chu, Fan, Lin y Wang 2004). Cada funcion dice bajo que condicion vale.

Convenciones: ejes en radianes; el eje es J theta'' + b theta' = u + d, con
x = [theta, omega]. Los lazos digitales corren a `Ts` (20 Hz en la Estacion)
con retencion de orden cero.
"""
import math

import numpy as np

from atp_ros import model
from atp_ros.planta import PlantaMontura, VientoTurbulento, cuantizar


# ---------------------------------------------------------------- algebra

def expm(M, terminos=24):
    """Exponencial de matriz por escalamiento y cuadrado con Taylor.
    Valida para matrices pequenas y bien escaladas (aqui <= 6x6, ||M Ts|| ~ 1);
    error relativo ~1e-15."""
    M = np.asarray(M, dtype=float)
    n = M.shape[0]
    s = max(0, int(math.ceil(math.log2(max(np.linalg.norm(M, np.inf), 1e-30)))) + 1)
    X = M / (2.0 ** s)
    E = np.eye(n)
    T = np.eye(n)
    for k in range(1, terminos):
        T = T @ X / k
        E = E + T
    for _ in range(s):
        E = E @ E
    return E


def matrices_eje(J=model.J_EJE, b=model.B_EJE):
    """Un eje x = [theta, omega]: A = [[0,1],[0,-b/J]], B = [0, 1/J]^T.
    Con J = 2, b = 0.5: B = [0, 0.5]^T, como en la leccion 3.1."""
    J, b = float(J), float(b)
    return (np.array([[0.0, 1.0], [0.0, -b / J]]), np.array([[0.0], [1.0 / J]]))


def discretizar_zoh(A, B, Ts):
    """Discretizacion exacta con retencion de orden cero (la del DAC de la
    montura): Ad = e^{A Ts}, Bd = int_0^Ts e^{A s} ds B, via la exponencial
    de [[A, B],[0, 0]] Ts. Exacta si u es constante en el periodo (el lazo
    de la Estacion mantiene el mando entre muestras)."""
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float).reshape(A.shape[0], -1)
    n, m = A.shape[0], B.shape[1]
    M = np.zeros((n + m, n + m))
    M[:n, :n] = A
    M[:n, n:] = B
    E = expm(M * float(Ts))
    return E[:n, :n], E[:n, n:]


def discretizar_ruido(A, Qc, Ts):
    """Covarianza discreta de proceso Qd para dx = A x dt + dw con densidad
    espectral Qc (Van Loan 1978): de la exponencial de
    [[-A, Qc],[0, A']] Ts. Valida para ruido blanco continuo."""
    A = np.asarray(A, dtype=float)
    n = A.shape[0]
    M = np.zeros((2 * n, 2 * n))
    M[:n, :n] = -A
    M[:n, n:] = Qc
    M[n:, n:] = A.T
    E = expm(M * float(Ts))
    Ad = E[n:, n:].T
    Qd = Ad @ E[:n, n:]
    return Ad, (Qd + Qd.T) / 2.0


def controlabilidad(A, B):
    """Matriz [B AB] y su rango/determinante. El eje del curso da det = -0.25
    y rango 2: controlable, se pueden colocar polos donde se quiera."""
    A, B = np.asarray(A, float), np.asarray(B, float).reshape(-1, 1)
    C = np.hstack([B, A @ B])
    return {'matriz': C, 'rango': int(np.linalg.matrix_rank(C)),
            'det': float(np.linalg.det(C))}


# ---------------------------------------------------------------- Riccati

def lqr(A, B, Q, R):
    """LQR CONTINUO: resuelve A'P + PA - P B R^-1 B' P + Q = 0 y devuelve
    (K, P), K = R^-1 B' P, por autovectores del semiplano izquierdo de la
    hamiltoniana (como `lqr` de atp.py). Valido si (A,B) estabilizable y
    (Q^1/2,A) detectable; no se debe usar con polos en el eje imaginario de
    la hamiltoniana (Q con una direccion no detectable)."""
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float).reshape(A.shape[0], -1)
    Q = np.asarray(Q, dtype=float)
    R = np.atleast_2d(np.asarray(R, dtype=float))
    n = A.shape[0]
    Rinv = np.linalg.inv(R)
    H = np.block([[A, -B @ Rinv @ B.T], [-Q, -A.T]])
    vals, vecs = np.linalg.eig(H)
    est = np.argsort(vals.real)[:n]
    U = vecs[:, est]
    P = np.real(U[n:, :] @ np.linalg.inv(U[:n, :]))
    P = (P + P.T) / 2.0
    return Rinv @ B.T @ P, P


def lqr_doble_integrador(q, r):
    """Forma CERRADA del LQR de un doble integrador con Q = diag(q, 0):
    k1 = sqrt(q/r), k2 = sqrt(2 k1), wn = (q/r)^(1/4), zeta = 1/sqrt(2)
    para cualquier q y r (lo que el PID buscaba a tientas)."""
    k1 = float(np.sqrt(float(q) / float(r)))
    k2 = float(np.sqrt(2.0 * k1))
    wn = float(np.sqrt(k1))
    zeta = float(k2 / (2.0 * wn))
    return {'k1': k1, 'k2': k2, 'wn': wn, 'zeta': zeta,
            't_est': t_establecimiento(zeta, wn)}


def t_establecimiento(zeta, wn, banda=0.02):
    """t_s = -ln(banda)/(zeta wn): aproximacion clasica de un segundo orden
    subamortiguado (zeta = 0.707, wn = 3.16 -> 1.75 s al 2 %)."""
    z, w = float(zeta), float(wn)
    if z <= 0 or w <= 0:
        return float('inf')
    return float(-np.log(float(banda)) / (z * w))


def dare(A, B, Q, R, tol=1e-13, max_iter=100):
    """Ecuacion de Riccati DISCRETA X = A'X(I + G X)^-1 A + Q, G = B R^-1 B',
    por el algoritmo de duplicacion estructurado (SDA): convergencia
    cuadratica, sin autovalores. Devuelve X (simetrica).

    Valida si (A,B) es estabilizable y (Q^1/2,A) detectable; admite A con
    polos en |z| = 1 (integrador) siempre que Q los penalice. Lanza
    ArithmeticError si no converge."""
    A = np.asarray(A, dtype=float)
    B = np.asarray(B, dtype=float).reshape(A.shape[0], -1)
    Q = np.asarray(Q, dtype=float)
    R = np.atleast_2d(np.asarray(R, dtype=float))
    n = A.shape[0]
    I = np.eye(n)
    Ak, Gk, Hk = A.copy(), B @ np.linalg.solve(R, B.T), Q.copy()
    for _ in range(max_iter):
        W = np.linalg.solve(I + Gk @ Hk, I)         # (I + G H)^-1
        An = Ak @ W @ Ak
        Gn = Gk + Ak @ W @ Gk @ Ak.T
        Hn = Hk + Ak.T @ Hk @ W @ Ak
        dif = np.max(np.abs(Hn - Hk))
        Ak, Gk, Hk = An, (Gn + Gn.T) / 2.0, (Hn + Hn.T) / 2.0
        if dif <= tol * max(1.0, np.max(np.abs(Hk))):
            return Hk
    raise ArithmeticError('dare: no converge')


def lqr_discreto(Ad, Bd, Q, R):
    """LQR discreto u = -K x: K = (R + B'XB)^-1 B'XA con X = dare(...).
    Devuelve (K, X)."""
    Bd = np.asarray(Bd, dtype=float).reshape(np.asarray(Ad).shape[0], -1)
    R = np.atleast_2d(np.asarray(R, dtype=float))
    X = dare(Ad, Bd, Q, R)
    K = np.linalg.solve(R + Bd.T @ X @ Bd, Bd.T @ X @ np.asarray(Ad, float))
    return K, X


def aumentar_integral(Ad, Bd, Ts):
    """Estado aumentado z = [theta, omega, zeta] con zeta+ = zeta + Ts theta
    (integral del error de posicion, referencia ya restada: z son DESVIOS).
    Aa = [[Ad, 0],[Ts C, 1]], Ba = [Bd; 0]."""
    n = Ad.shape[0]
    Aa = np.zeros((n + 1, n + 1))
    Aa[:n, :n] = Ad
    Aa[n, 0] = float(Ts)
    Aa[n, n] = 1.0
    Ba = np.vstack([np.asarray(Bd, float).reshape(n, 1), [[0.0]]])
    return Aa, Ba


# Pesos por defecto del LQR de la montura (rad, rad/s, rad s, N m): ver
# docs/CONTROL.md, "Sintonia". Bryson: q ~ 1/(desvio admisible)^2.
LQR_DEFECTO = {'q_theta': 1.0e4, 'q_omega': 1.0, 'q_int': 1.0e3, 'r': 1.0}


def lqr_integral(J=model.J_EJE, b=model.B_EJE, Ts=0.05,
                 q_theta=None, q_omega=None, q_int=None, r=None):
    """LQR DISCRETO con accion integral sobre [theta, omega, integral e].

    Discretiza el eje con retencion de orden cero a `Ts`, aumenta con el
    integrador y resuelve la Riccati discreta. Devuelve un dict con
    `K` (1x3 = [k_theta, k_omega, k_int]), las matrices `Ad, Bd, Aa, Ba`,
    el lazo cerrado `Acl = Aa - Ba K` y sus `polos` (deben estar dentro del
    circulo unidad). Ley (en desvios): u = u_ff - K [theta - r, omega - rdot,
    zeta]. Valido para la planta lineal J, b sin saturar."""
    d = dict(LQR_DEFECTO)
    for k, v in (('q_theta', q_theta), ('q_omega', q_omega), ('q_int', q_int), ('r', r)):
        if v is not None:
            d[k] = float(v)
    A, B = matrices_eje(J, b)
    Ad, Bd = discretizar_zoh(A, B, Ts)
    Aa, Ba = aumentar_integral(Ad, Bd, Ts)
    Q = np.diag([d['q_theta'], d['q_omega'], d['q_int']])
    K, X = lqr_discreto(Aa, Ba, Q, d['r'])
    Acl = Aa - Ba @ K
    return {'K': K, 'P': X, 'Ad': Ad, 'Bd': Bd, 'Aa': Aa, 'Ba': Ba, 'Acl': Acl,
            'polos': np.linalg.eigvals(Acl), 'Q': Q, 'R': d['r'], 'Ts': float(Ts)}


# ---------------------------------------------------------------- margenes

def _margenes_desde_L(w, L):
    """Margenes de un lazo abierto L(jw) muestreado en `w` (rad/s), por
    cruces: el margen de fase en cada |L| = 1 (el menor manda), el de
    ganancia en cada cruce del eje real negativo (hacia ARRIBA si |L| < 1,
    hacia ABAJO si |L| > 1: un lazo con doble integrador es condicionalmente
    estable y tambien se desestabiliza al BAJAR la ganancia)."""
    mag = np.abs(L)
    mag_db = 20.0 * np.log10(mag)
    ang = np.degrees(np.angle(L))
    mf, wc, mf_all = float('inf'), float('nan'), []
    for i in np.where(np.diff(np.sign(mag_db)) != 0)[0]:
        fr = -mag_db[i] / (mag_db[i + 1] - mag_db[i])
        wci = float(w[i] + fr * (w[i + 1] - w[i]))
        Lc = L[i] + fr * (L[i + 1] - L[i])
        pm = (180.0 + np.degrees(np.angle(Lc))) % 360.0
        pm = pm - 360.0 if pm > 180.0 else pm
        mf_all.append((float(pm), wci))
        if abs(pm) < abs(mf):
            mf, wc = float(pm), wci
    mg_up, mg_dn, w_pi = float('inf'), float('inf'), float('nan')
    im, re = L.imag, L.real
    for i in np.where((np.sign(im[:-1]) != np.sign(im[1:])) & (re[:-1] < 0.0))[0]:
        fr = im[i] / (im[i] - im[i + 1])
        m = abs(L[i] + fr * (L[i + 1] - L[i]))
        if m < 1.0:
            if -20.0 * np.log10(m) < mg_up:
                mg_up, w_pi = float(-20.0 * np.log10(m)), float(w[i] + fr * (w[i + 1] - w[i]))
        else:
            mg_dn = min(mg_dn, float(20.0 * np.log10(m)))
    # retardo que lleva el lazo al limite: PM/wc del cruce mas restrictivo
    ret = [np.radians(p) / c for p, c in mf_all if p > 0 and c > 0]
    return {'margen_ganancia_db': mg_up, 'margen_ganancia_baja_db': mg_dn,
            'margen_fase_deg': mf, 'wc': wc, 'w_pi': w_pi,
            'margen_retardo_s': float(min(ret)) if ret else float('inf'),
            'w': w, 'mag_db': mag_db, 'fase_deg': ang}


def margenes(A, B, K, w=None):
    """Margenes de ganancia (dB) y fase (grados) del lazo abierto continuo
    L(s) = K (sI - A)^-1 B, barriendo frecuencia (como `margenes` de atp.py).
    Valido para realimentacion de estado con la B de la planta; un LQR
    continuo con R diagonal garantiza MG infinito y MF >= 60 grados
    (Anderson y Moore 1990), y aqui se comprueba, no se cita."""
    A = np.asarray(A, float)
    B = np.asarray(B, float).reshape(A.shape[0], -1)
    K = np.asarray(K, float).reshape(1, -1)
    w = np.logspace(-2, 3, 4000) if w is None else np.asarray(w, float)
    I = np.eye(A.shape[0])
    L = np.array([(K @ np.linalg.solve(1j * wi * I - A, B))[0, 0] for wi in w])
    mag_db = 20.0 * np.log10(np.abs(L))
    fase = np.degrees(np.unwrap(np.angle(L)))
    mf, wc = float('nan'), float('nan')
    idx = np.where(np.diff(np.sign(mag_db)))[0]
    if len(idx):
        i0 = idx[0]
        frac = -mag_db[i0] / (mag_db[i0 + 1] - mag_db[i0])
        wc = float(w[i0] + frac * (w[i0 + 1] - w[i0]))
        mf = float(180.0 + fase[i0] + frac * (fase[i0 + 1] - fase[i0]))
    mg = float('inf')
    cr = np.where(np.diff(np.sign(fase + 180.0)))[0]
    if len(cr):
        i0 = cr[0]
        frac = (-180.0 - fase[i0]) / (fase[i0 + 1] - fase[i0])
        mg = float(-(mag_db[i0] + frac * (mag_db[i0 + 1] - mag_db[i0])))
    return {'margen_ganancia_db': mg, 'margen_fase_deg': mf, 'wc': wc,
            'w': w, 'mag_db': mag_db, 'fase_deg': fase}


def margen_fase_con_retardo(A, B, K, retardo_s):
    """Margen de fase (grados) tras anadir un retardo puro `retardo_s`
    (muestreo + computo + bus): resta wc * retardo en el cruce. Es lo que se
    come el margen al bajar la frecuencia del lazo."""
    m = margenes(A, B, K)
    if not np.isfinite(m['wc']):
        return m['margen_fase_deg']
    return float(m['margen_fase_deg'] - np.degrees(m['wc'] * float(retardo_s)))


def margenes_discreto(Aa, Ba, K, Ts, n_w=6000):
    """Margenes del lazo DIGITAL L(z) = K (zI - Aa)^-1 Ba en z = e^{jwTs}
    (lazo roto a la entrada de la planta, incluido el integrador del
    estado aumentado), hasta la frecuencia de Nyquist. Devuelve
    margen_ganancia_db (hacia arriba; inf si no hay cruce),
    margen_ganancia_baja_db (hacia abajo: el lazo es condicionalmente
    estable), margen_fase_deg, wc, `margen_retardo_s` = MF/wc (retardo puro
    que lleva el lazo al limite) y `Ms` = max |1/(1+L)| (pico de la sensibilidad; <= 2 es buena
    robustez). Valido para el lazo de estados REALES sin estimador: con
    Kalman (LQG) estas garantias no se heredan (Doyle 1978)."""
    Aa = np.asarray(Aa, float)
    Ba = np.asarray(Ba, float).reshape(Aa.shape[0], -1)
    K = np.asarray(K, float).reshape(1, -1)
    w = np.logspace(-3, np.log10(np.pi / float(Ts)), int(n_w))
    I = np.eye(Aa.shape[0])
    L = np.array([(K @ np.linalg.solve(np.exp(1j * wi * Ts) * I - Aa, Ba))[0, 0]
                  for wi in w])
    m = _margenes_desde_L(w, L)
    m['Ms'] = float(np.max(1.0 / np.abs(1.0 + L)))
    return m


# ---------------------------------------------------------------- Kalman

# Densidades espectrales por defecto del Kalman de la montura (supuesto,
# ver docs/CONTROL.md): aceleracion no modelada y deriva del par de sesgo.
KALMAN_DEFECTO = {'q_acc': 1.0e-4, 'q_par': 1.0e-2}


def diseno_kalman(J=model.J_EJE, b=model.B_EJE, Ts=0.05, sigma_med_rad=1e-4,
                  bits=0, q_acc=None, q_par=None):
    """Modelo de un eje para el filtro de Kalman, x = [theta, omega, d] con
    d = par de perturbacion/sesgo (N m, camina al azar): friccion,
    gravedad, viento lento...

        theta' = omega;  omega' = (u + d - b omega)/J + w_a;  d' = w_d

    `q_acc` [(rad/s^2)^2 s] es la densidad de la aceleracion no modelada y
    `q_par` [(N m)^2/s] la de la deriva de d. La medida es theta con ruido
    gaussiano `sigma_med_rad` MAS la cuantizacion de `bits` bits
    (varianza q^2/12 de un cuantizador uniforme; valido si el error de
    cuantizacion es ~uniforme e incorrelado: cierto cuando la senal cruza
    varios escalones por muestra o hay ruido de al menos ~q/2). Devuelve
    dict con Ad, Bd, Qd, R, H."""
    J, b = float(J), float(b)
    d = dict(KALMAN_DEFECTO)
    if q_acc is not None:
        d['q_acc'] = float(q_acc)
    if q_par is not None:
        d['q_par'] = float(q_par)
    A = np.array([[0.0, 1.0, 0.0], [0.0, -b / J, 1.0 / J], [0.0, 0.0, 0.0]])
    B = np.array([[0.0], [1.0 / J], [0.0]])
    Qc = np.diag([0.0, d['q_acc'], d['q_par']])
    Ad, Bd = discretizar_zoh(A, B, Ts)
    _, Qd = discretizar_ruido(A, Qc, Ts)
    q = 2.0 * math.pi / 2 ** int(bits) if bits else 0.0
    # piso de 1e-6 rad: con ruido y bits nulos R = 0 haria al filtro ciego a su modelo
    R = max(float(sigma_med_rad), 1e-6) ** 2 + q * q / 12.0
    return {'Ad': Ad, 'Bd': Bd, 'Qd': Qd, 'R': R, 'H': np.array([[1.0, 0.0, 0.0]]),
            'A': A, 'B': B, 'Ts': float(Ts)}


class FiltroKalman:
    """Kalman discreto de [theta, omega, d] para uno o muchos ejes a la vez.

    `x` tiene forma (3, ...) (por eje y/o por pase); la covarianza P (3,3)
    es COMUN: no depende de los datos (mismo modelo y misma R en todos),
    asi que se propaga una vez. Forma de Joseph (simetrica, semidefinida).

    Orden en el lazo en lockstep: en el paso k llega la medida y_k (estado
    tras integrar con u_{k-1}); `paso(y_k, u_{k-1})` predice con u_{k-1} y
    corrige con y_k; el mando u_k se calcula con la estimacion corregida.
    """

    def __init__(self, diseno, y0, forma=(2,), sigma_p0=(2e-3, 0.05, 0.1)):
        self.Ad, self.Bd = diseno['Ad'], diseno['Bd'][:, 0]
        self.Qd, self.R = diseno['Qd'], diseno['R']
        self.H = diseno['H'][0]
        self.forma = tuple(forma)
        self.x = np.zeros((3,) + self.forma)
        self.x[0] = y0
        self.P = np.diag(np.asarray(sigma_p0, float) ** 2)
        self.K = None

    def paso(self, y, u_prev=0.0):
        """Predice con el mando previo y corrige con la medida `y`.
        Devuelve la estimacion x (3, ...)."""
        sh = (3,) + (1,) * len(self.forma)
        self.x = np.tensordot(self.Ad, self.x, axes=(1, 0)) \
            + self.Bd.reshape(sh) * np.asarray(u_prev)
        self.P = self.Ad @ self.P @ self.Ad.T + self.Qd
        S = float(self.H @ self.P @ self.H + self.R)
        K = (self.P @ self.H) / S
        self.x = self.x + K.reshape(sh) * (np.asarray(y) - self.x[0])
        IKH = np.eye(3) - np.outer(K, self.H)
        self.P = IKH @ self.P @ IKH.T + self.R * np.outer(K, K)
        self.P = (self.P + self.P.T) / 2.0
        self.K = K
        return self.x

    def nees(self, x_verdadero):
        """Error de estimacion normalizado (x - x_hat)' P^-1 (x - x_hat) por
        muestra; su promedio debe rondar la dimension (3) si el filtro es
        consistente."""
        e = np.asarray(x_verdadero) - self.x
        e = e.reshape(3, -1)
        return np.einsum('in,in->n', e, np.linalg.solve(self.P, e))


class EstimadorDerivada:
    """Estimador SIN Kalman: theta_hat = medida y omega_hat = derivada
    filtrada de primer orden de la medida (la misma que usa el PID, tau
    0.08 s). El LQR sin filtro lo usa; amplifica la cuantizacion del
    encoder (q/Ts de ruido en omega) y retrasa tau."""

    def __init__(self, y0, Ts, tau=0.08, forma=(2,)):
        self.Ts = float(Ts)
        self.alfa = self.Ts / (float(tau) + self.Ts)
        self.y_prev = np.array(y0, dtype=float)
        self.w = np.zeros(self.y_prev.shape)
        self.x = np.zeros((3,) + tuple(forma))
        self.x[0] = y0

    def paso(self, y, u_prev=0.0):
        y = np.asarray(y, dtype=float)
        self.w = self.w + self.alfa * ((y - self.y_prev) / self.Ts - self.w)
        self.y_prev = y.copy()
        self.x = np.stack([y, self.w, np.zeros_like(y)])
        return self.x


def crear_estimador(kalman, J, b, Ts, y0, sigma_med_rad, bits=0, forma=(2,),
                    **kw):
    """Estimador del lazo: `FiltroKalman` si `kalman`, si no
    `EstimadorDerivada`. Ambos exponen `.x` = [theta_h, omega_h, d_h] y
    `.paso(y, u_prev)`. `kw` = q_acc, q_par del Kalman."""
    if kalman:
        return FiltroKalman(diseno_kalman(J, b, Ts, sigma_med_rad, bits, **kw),
                            y0, forma)
    return EstimadorDerivada(y0, Ts, 0.08, forma)


# ---------------------------------------------------------------- lazos

class DiferenciadorRef:
    """Velocidad y aceleracion de la referencia a partir de sus muestras a
    `Ts`, con diferencias hacia atras de 2.o orden (causales, error O(Ts^2)):
        v = (3 r_k - 4 r_{k-1} + r_{k-2}) / (2 Ts)
        a = (r_k - 2 r_{k-1} + r_{k-2}) / Ts^2
    Valido para referencias suaves y SIN saltos (un salto de referencia
    produce un pulso de a); los dos primeros pasos devuelven 0."""

    def __init__(self, Ts):
        self.Ts = float(Ts)
        self.r1 = self.r2 = None
        self.n = 0

    def paso(self, r):
        r = np.asarray(r, dtype=float)
        if self.n >= 2:
            v = (3.0 * r - 4.0 * self.r1 + self.r2) / (2.0 * self.Ts)
            a = (r - 2.0 * self.r1 + self.r2) / self.Ts ** 2
        else:
            v = np.zeros_like(r)
            a = np.zeros_like(r)
        self.r2, self.r1 = self.r1, r.copy()
        self.n += 1
        return v, a


class LazoLQR:
    """Ley de control del LQR con accion integral (una o muchas copias).

        u = clip( u_ff - K [theta_h - r, omega_h - rdot, zeta] - c d_h )
        u_ff = J rddot + b rdot            (pre-alimentacion de la referencia)
        zeta+ = zeta + Ts (theta_h - r)    (integral, con anti-windup por
                                            clamping como el PID)

    `c` = 1 compensa ademas el par de sesgo d_h que estima el Kalman (la
    integral solo lo hace despacio); con EstimadorDerivada d_h = 0.
    Valido sin saturar (K diseno lineal); con saturacion el clamping evita el
    windup pero no hay garantia global.
    """

    def __init__(self, diseno, J, b, u_max, forma=(2,), antiwindup=True,
                 usar_ff=True, usar_dhat=True):
        self.K = np.asarray(diseno['K'], float).reshape(3)
        self.Ts = float(diseno['Ts'])
        self.J, self.b, self.u_max = float(J), float(b), float(u_max)
        self.aw, self.ff, self.dh = bool(antiwindup), bool(usar_ff), bool(usar_dhat)
        self.zeta = np.zeros(forma)
        self.u = np.zeros(forma)

    def calcular(self, est, ref, ref_v, ref_a):
        """`est` = (3, ...) [theta_h, omega_h, d_h]; devuelve el mando u."""
        th, w, d = est
        e_p = th - ref
        e_v = w - (ref_v if self.ff else 0.0)
        u_ff = (self.J * ref_a + self.b * ref_v) if self.ff else 0.0
        u_raw = u_ff - (self.K[0] * e_p + self.K[1] * e_v + self.K[2] * self.zeta)
        if self.dh:
            u_raw = u_raw - d
        self.u = np.clip(u_raw, -self.u_max, self.u_max)
        sat = np.abs(u_raw) > self.u_max + 1e-12
        # clamping: no integra si el mando esta saturado y el error empuja mas
        congelar = self.aw & sat & (u_raw * (-e_p) > 0.0)
        self.zeta = self.zeta + np.where(congelar, 0.0, e_p * self.Ts)
        return self.u


# ---------------------------------------------------------------- campana

CONTROLADORES = ('pid', 'lqr', 'lqr_kf')


def campana_lazo_ctrl(controlador='pid', n=200, sigma_viento_nm=0.010,
                      sigma_ruido_deg=0.02, sesgo_max_deg=0.03,
                      latencia_max_s=0.045, kp=25.0, ki=2.0, kd=8.0, u_max=8.0,
                      vel_max_deg_s=6.0, h_km=model.H_LEO_KM,
                      mascara_deg=model.MASCARA_DEG, J=model.J_EJE,
                      b=model.B_EJE, f_lazo=20.0, dt=0.01, rafaga_s=2.0,
                      semilla=2029, lqr=None, kalman=None, ff=True,
                      coulomb_nm=0.0, stribeck_nm=0.0, stribeck_vel_deg_s=0.5,
                      backlash_deg=0.0, backlash_motor_frac=0.5,
                      unbalance_nm=0.0, encoder_bits=0, viento_sigma_nm=0.0,
                      viento_modelo='dryden', viento_tau_s=5.0, progreso=None):
    """`model.campana_lazo` con ELECCION DE CONTROLADOR y planta realista.

    controlador: 'pid' (el de la Estacion), 'lqr' (LQR con integral y
    derivada filtrada como estimador) o 'lqr_kf' (LQR + Kalman de
    [theta, omega, par de sesgo]). Mismas perturbaciones, mismas semillas y
    mismos n pases al azar para los tres: la comparacion es pareada.
    Con los parametros de planta por defecto y `controlador='pid'` el
    resultado es IDENTICO al de `model.campana_lazo` (se prueba).

    Extras de planta (todos apagados por defecto; ver `planta.py`): friccion
    de Coulomb/Stribeck, backlash, desbalance gravitatorio (eje 1),
    cuantizacion del encoder, y viento turbulento con espectro
    (`viento_sigma_nm` > 0, Dryden o Davenport) ADEMAS de la rafaga.

    Metrica: RMS del error de apuntamiento real en el cielo
    sqrt((e_az cos el)^2 + e_el^2) con el angulo REAL de la carga (no la
    medida). Devuelve {'rms', 'el_max'}.
    """
    if controlador not in CONTROLADORES:
        raise ValueError(f'controlador desconocido: {controlador}')
    rng = np.random.default_rng(int(semilla))
    n = int(max(1, n))
    el_max = model.elmax_aleatorio(n, h_km, mascara_deg, rng)
    el_max = np.maximum(el_max, mascara_deg + 0.5)
    az_cul = rng.uniform(0.0, 360.0, n)
    perfiles = [model.perfil_pase(h_km, e, mascara_deg, a, n=600)
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
    ref = np.zeros((2, n, len(tick_t)))
    ref_true = np.zeros((2, n, len(tick_t)))
    for k, p in enumerate(perfiles):
        tt = np.minimum(tick_t, p['t'][-1])
        td = np.maximum(tt - lat[k], 0.0)
        for eje, clave in ((0, 'az_continuo'), (1, 'el')):
            ref_true[eje, k] = np.radians(np.interp(tt, p['t'], p[clave]))
            ref[eje, k] = np.radians(np.interp(td, p['t'], p[clave]))

    w_max = np.radians(vel_max_deg_s)
    planta = PlantaMontura(
        J, b, w_max, dt, coulomb_nm=coulomb_nm, stribeck_nm=stribeck_nm,
        v_stribeck=math.radians(stribeck_vel_deg_s),
        backlash_rad=math.radians(backlash_deg), frac_motor=backlash_motor_frac,
        mgd_nm=unbalance_nm)
    planta.enganchar(ref_true[:, :, 0])
    viento = None
    if viento_sigma_nm > 0.0:
        viento = VientoTurbulento(viento_sigma_nm, viento_modelo, viento_tau_s,
                                  forma=(2, n), semilla=int(semilla) + 1000)
    par_v = 0.0

    integ = np.zeros((2, n))
    e_prev = np.zeros((2, n))
    d_filt = np.zeros((2, n))
    u = np.zeros((2, n))
    alfa = dt_lazo / (0.08 + dt_lazo)
    suma_e2 = np.zeros(n)
    cuenta = np.zeros(n)
    aviso = max(1, len(tick_t) // 20)

    est = lazo = dif = u_prev = None
    if controlador != 'pid':
        dis = lqr_integral(J, b, dt_lazo, **(lqr or {}))
        lazo = LazoLQR(dis, J, b, u_max, forma=(2, n), usar_ff=ff,
                       usar_dhat=(controlador == 'lqr_kf'))
        dif = DiferenciadorRef(dt_lazo)

    for j, t in enumerate(tick_t):
        vivo = t <= dur
        enc = planta.angulo_encoder('load')
        medida = enc + sesgo + rng.normal(0.0, ruido, (2, n))
        if encoder_bits:
            medida = cuantizar(medida, encoder_bits)
        if controlador == 'pid':
            e = ref[:, :, j] - medida
            d_filt += alfa * ((e - e_prev) / dt_lazo - d_filt)
            u_raw = kp * e + ki * integ + kd * d_filt
            u = np.clip(u_raw, -u_max, u_max)
            congelar = (np.abs(u_raw) > u_max) & (u_raw * e > 0.0)
            integ += np.where(congelar, 0.0, e * dt_lazo)
            e_prev = e
        else:
            if j == 0:
                est = crear_estimador(controlador == 'lqr_kf', J, b, dt_lazo,
                                      medida, ruido, encoder_bits, (2, n),
                                      **(kalman or {}))
                estimado = est.x
            else:
                estimado = est.paso(medida, u_prev)
            rv, ra = dif.paso(ref[:, :, j])
            u = lazo.calcular(estimado, ref[:, :, j], rv, ra)
            u_prev = u
        # error real (angulo REAL de la carga, sin sesgo ni ruido)
        th = planta.theta
        e_true = ref_true[:, :, j] - th
        e_pt = np.hypot(e_true[0] * np.cos(ref_true[1, :, j]), e_true[1])
        suma_e2 += np.where(vivo, np.degrees(e_pt) ** 2, 0.0)
        cuenta += vivo
        if viento is not None:
            par_v = viento.par(t, dt_lazo)
        for _ in range(paso):
            en_rafaga = (t >= t_rafaga) & (t < t_rafaga + rafaga_s)
            par = np.where(en_rafaga, par_rafaga, 0.0)
            if viento is not None:
                par = par + par_v
            planta.paso(u, par, dt)
        if progreso is not None and j % aviso == 0:
            progreso(j, len(tick_t))

    rms = np.sqrt(suma_e2 / np.maximum(cuenta, 1))
    return {'rms': rms, 'el_max': el_max}


def comparar_controladores(controladores=CONTROLADORES, **kw):
    """Corre `campana_lazo_ctrl` para cada controlador con las mismas
    semillas y devuelve {nombre: percentiles(...)} (p50/p95/p99/peor/pasa
    contra el objetivo de 0.1 grados) mas el vector `rms` y `el_max`."""
    out = {}
    for c in controladores:
        r = campana_lazo_ctrl(controlador=c, **kw)
        pc = model.percentiles(r['rms'])
        pc['rms'], pc['el_max'] = r['rms'], r['el_max']
        out[c] = pc
    return out
