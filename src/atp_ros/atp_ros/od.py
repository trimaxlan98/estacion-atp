"""Determinacion de orbita (OD) con las mediciones de la propia estacion.

Solo numpy (sin ROS, sin scipy). Cierra el lazo de la orbita: acimut y
elevacion de la montura (y, si hay receptor, rango y Doppler) -> estado del
satelite -> prediccion de los pases siguientes -> referencia de la antena.

Piezas:
  * `ajuste_lotes`: minimos cuadrados ponderados por lotes (Gauss-Newton con
    amortiguamiento de Levenberg-Marquardt) sobre el propagador de
    `propagador.py`, estimando el estado (r, v) en la epoca del primer dato y,
    opcionalmente, el factor del coeficiente balistico `p`. Con prior
    opcional (el TLE). Entrega covarianza formal, residuos normalizados y
    chi2 por grado de libertad.
  * `ekf`: filtro de Kalman extendido secuencial (actualizacion escalar por
    canal, forma de Joseph) con ruido de proceso de aceleracion blanca.
  * `monte_carlo_nees`: consistencia del estimador (NEES medio ~ dimension).
  * Utilidades de escenario: efemerides, pases, error de apuntamiento y el
    perfil de pase con el formato de `orbit.muestrear_pase` para que
    `reference_generator` lo use (`perfil_od`).

Supuestos que el lector debe conocer (ver docs/OD.md):
  * El modelo dinamico del estimador es el del propagador (J2.. + arrastre
    exponencial); un modelo mas pobre que la "verdad" es la situacion normal,
    y el error de modelo NO esta en la covarianza formal (por eso se mide
    ademas el error real contra la verdad).
  * Linealizacion: Gauss-Newton converge si la estimacion inicial esta a
    unos km del estado real (un TLE de dias, ~1-10 km, lo cumple). Con SOLO
    angulos y un pase corto el rango es casi inobservable: sin prior la
    covarianza explota en esa direccion (se reporta, no se oculta).
  * Datos independientes y sin sesgo (ruido blanco gaussiano). Los sesgos
    sistematicos de la montura, la refraccion y el tiempo de luz se discuten
    en `medidas.py`.
"""
import datetime as dt
import json
import math
from dataclasses import dataclass, field

import numpy as np

from atp_ros import medidas as M
from atp_ros import orbit
from atp_ros import propagador as P

UTC = dt.timezone.utc

INT_STM = dict(metodo='dp45', rtol=1e-8, atol=1e-8, h=20.0, h_max=100.0)   # STM en la OD
INT_VERDAD = dict(metodo='rk4', h=15.0)  # integracion de la verdad


def _escala(n):
    """Escala de las incognitas (km, km/s, adimensional) para condicionar."""
    return np.array([1.0, 1.0, 1.0, 1e-3, 1e-3, 1e-3, 0.1][:n])


# ---------------------------------------------------------------- escenario

class EfemerideSGP4:
    """SGP4 con la misma interfaz `estado(t)` que `propagador.Efemeride`."""

    def __init__(self, tle, reloj):
        self.tle, self.reloj = tle, reloj

    def estado(self, t):
        jd, fr = self.reloj.jdfr(t)
        r, v = orbit.propagar_teme(self.tle, jd, fr)
        return r, v


def estado_tle(tle, reloj=None):
    """Estado TEME (r km, v km/s) de SGP4 en la epoca del TLE, y el `Reloj`
    con esa epoca. Es el "conocido a priori" cuando el TLE es reciente."""
    reloj = reloj or M.Reloj(tle.epoca)
    r, v = orbit.propagar_teme(tle, *reloj.jdfr(0.0))
    return np.r_[r[0], v[0]], reloj


def efemeride(modelo, x, t0, t_ini, t_fin, dt_s=30.0, metodo=None):
    """`Efemeride` (rejilla de `dt_s`) integrando desde (t0, x) hasta t_fin.
    x = [r, v] o [r, v, p]. Si t_ini < t0 no se integra hacia atras: la
    rejilla empieza en t0."""
    p = float(x[6]) if len(x) > 6 else 1.0
    t_ini = max(t_ini, t0)
    n = int(math.ceil((t_fin - t_ini) / dt_s)) + 1
    ts = t_ini + dt_s * np.arange(n)
    kw = dict(metodo or INT_VERDAD)
    return P.Efemeride.desde_modelo(modelo, np.asarray(x[:6], float), t0, ts, p=p, **kw)


def pases_efemeride(ef, reloj, sit, t_ini, t_fin, mascara_deg=None, paso_s=20.0):
    """Pases sobre la estacion en [t_ini, t_fin] (s del reloj) para cualquier
    objeto con `estado(t)`. AOS/LOS por biseccion (0.01 s), elevacion maxima
    en rejilla de 1 s. Devuelve dicts con aos, los, t_el_max, el_max_deg,
    duracion_s y los booleanos `truncado` (el pase toca el borde de la
    ventana). Valido como la efemeride (la mascara por defecto es la de la
    estacion); un pase de menos de `paso_s` puede no detectarse."""
    mask = math.radians(sit.est.mascara_deg if mascara_deg is None else mascara_deg)
    n = int(math.ceil((t_fin - t_ini) / paso_s)) + 1
    tg = np.minimum(t_ini + paso_s * np.arange(n), t_fin)

    def el(t):
        r, v = ef.estado(t)
        return M.observables(reloj, sit, t, r, v)['el']

    sobre = np.nan_to_num(el(tg) - mask, nan=-1.0) >= 0.0

    def cruce(a, b):
        fa = el(np.array([a]))[0] - mask
        for _ in range(40):
            m = 0.5 * (a + b)
            fm = el(np.array([m]))[0] - mask
            if (fm > 0) == (fa > 0):
                a, fa = m, fm
            else:
                b = m
            if b - a < 0.01:
                break
        return 0.5 * (a + b)

    pases, i = [], 0
    while i < n:
        if not sobre[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and sobre[j + 1]:
            j += 1
        aos = tg[0] if i == 0 else cruce(tg[i - 1], tg[i])
        los = tg[-1] if j == n - 1 else cruce(tg[j], tg[j + 1])
        trunc = (i == 0, j == n - 1)
        i = j + 1
        if los - aos < 1.0:
            continue
        td = np.arange(aos, los + 1.0, 1.0)
        e = el(td)
        k = int(np.argmax(e))
        pases.append({'aos': float(aos), 'los': float(los), 'duracion_s': float(los - aos),
                      't_el_max': float(td[k]), 'el_max_deg': float(math.degrees(e[k])),
                      'aos_truncado': trunc[0], 'los_truncado': trunc[1]})
    return pases


def error_apuntamiento(ef_est, ef_ver, reloj, sit, t):
    """Desapuntamiento (deg) de una antena que apunta a la efemeride estimada
    cuando el satelite esta en la verdad: angulo entre las dos direcciones
    de la estacion, en cada instante `t`."""
    r1, v1 = ef_est.estado(t)
    r2, v2 = ef_ver.estado(t)
    a = M.observables(reloj, sit, t, r1, v1)
    b = M.observables(reloj, sit, t, r2, v2)
    return M.separacion_deg(a['az'], a['el'], b['az'], b['el'])


def resumen_error_pase(ef_est, ef_ver, reloj, sit, pase, dt_s=1.0):
    """Estadisticos del desapuntamiento sobre un pase: en AOS, maximo, rms y
    p95 (deg), y el error a lo largo de la pista (deg, firmado por la
    proyeccion sobre el movimiento del satelite no se calcula: solo el
    modulo)."""
    t = np.arange(pase['aos'], pase['los'] + 1e-9, dt_s)
    e = error_apuntamiento(ef_est, ef_ver, reloj, sit, t)
    return {'aos': float(e[0]), 'max': float(e.max()),
            'rms': float(np.sqrt(np.mean(e ** 2))), 'p95': float(np.percentile(e, 95)),
            'medio': float(e.mean())}


def medidas_de_pase(ef_ver, reloj, sitios, isit, pase, dt_s=5.0, sigmas=None, rng=None,
                    canales=M.CANALES, el_min_deg=0.0, el_max_deg=88.0,
                    sesgo_ang_deg=(0.0, 0.0)):
    """Medidas simuladas de un pase de la verdad cada `dt_s` s, solo con la
    elevacion entre `el_min_deg` y `el_max_deg` (cerca del cenit el acimut
    no esta definido y el jacobiano explota)."""
    t = np.arange(pase['aos'], pase['los'] + 1e-9, dt_s)
    r, v = ef_ver.estado(t)
    el = np.degrees(M.observables(reloj, sitios[isit], t, r, v)['el'])
    ok = (el >= el_min_deg) & (el <= el_max_deg)
    return M.generar(reloj, sitios, isit, t[ok], r[ok], v[ok], sigmas, rng, canales,
                     sesgo_ang_deg)


# ---------------------------------------------------------------- ajuste por lotes

@dataclass
class Resultado:
    """Solucion de la OD. `x` = [r(3) km, v(3) km/s (, p)] en la epoca `t0`
    (s desde el reloj); `P` su covarianza formal."""
    x: np.ndarray
    P: np.ndarray
    t0: float
    modelo: P.Modelo
    estima_p: bool
    iteraciones: int
    convergio: bool
    costo: float
    n_obs: int
    chi2_dof: float
    rms_norm: dict = field(default_factory=dict)      # rms de residuos normalizados por canal
    rms_fisico: dict = field(default_factory=dict)    # rms de residuos en unidades fisicas
    residuos: dict = field(default_factory=dict)      # canal -> (t, residuo normalizado)
    sv_norm: np.ndarray = None                        # valores singulares de la info normalizada
    historial: list = field(default_factory=list)

    @property
    def sigma(self):
        return np.sqrt(np.diag(self.P))

    def sigma_pos_km(self):
        """Raiz de la traza de la covarianza de posicion (km)."""
        return float(math.sqrt(np.trace(self.P[:3, :3])))


def _sigmas(sigmas):
    sg = dict(M.SIGMAS_DEFECTO)
    sg.update(sigmas or {})
    return sg


def _construir(med, x, t0, modelo, estima_p, sg, integ):
    """Residuos (pre-ajuste), matriz de diseno H (en unidades de x) y pesos
    para el estado `x` en `t0`. Devuelve (z, H, w, canal, t) con una fila por
    medida escalar valida."""
    n = len(x)
    ts = np.unique(med.t)
    f = P.derivadas_stm(modelo, estima_p)
    Y = P.propagar(f, P.y0_stm(x), t0, ts, **integ)
    idx = np.searchsorted(ts, med.t)
    Xt = Y[idx, :n]
    Phi = Y[idx, n:].reshape(len(med), n, n)
    z, H, w, can, tt = [], [], [], [], []
    for isit, sit in enumerate(med.sitios):
        sel = np.where(med.isit == isit)[0]
        if len(sel) == 0:
            continue
        r, v, t = Xt[sel, 0:3], Xt[sel, 3:6], med.t[sel]
        h = M.observables(med.reloj, sit, t, r, v)
        for c in M.CANALES:
            y = med.val[c][sel]
            ok = np.isfinite(y)
            if not ok.any():
                continue
            J = M.jacobiano(med.reloj, sit, t[ok], r[ok], v[ok], (c,))[:, 0, :]   # (m, 6)
            res = y[ok] - h[c][ok]
            if c == 'az':
                res = M.envolver(res)
            Hc = np.einsum('mi,mij->mj', J, Phi[sel][ok][:, :6, :])
            z.append(res)
            H.append(Hc)
            w.append(np.full(ok.sum(), 1.0 / sg[c]))
            can.append(np.full(ok.sum(), c))
            tt.append(t[ok])
    return (np.concatenate(z), np.vstack(H), np.concatenate(w),
            np.concatenate(can), np.concatenate(tt))


def ajuste_lotes(med, x0, t0, modelo, sigmas=None, estima_p=False, prior=None,
                 max_iter=12, tol=1e-7, integ=None, lm0=0.0, limites_p=(0.2, 3.0)):
    """Minimos cuadrados por lotes del estado (r, v[, p]) en `t0`.

    `limites_p`: cotas fisicas del factor de arrastre `p` (se recorta en cada
    paso). Sin ellas, con datos muy precisos y un modelo imperfecto, el ajuste
    usa `p` como comodin y lo lleva a valores absurdos (medido: p = 6 y
    negativos en el EKF) que empeoran la prediccion.

    med: `Medidas`; x0: estimacion inicial [r, v] (o [r, v, p]); t0 <= todos
    los t de `med`. `sigmas`: dict canal -> sigma (SIGMAS_DEFECTO). `prior`:
    (x_bar, P_bar) con las dimensiones del estado estimado, o None.
    Minimiza J = sum((y - h(x))/sigma)^2 + (x - x_bar)^T P_bar^-1 (x - x_bar)
    con Gauss-Newton escalado y amortiguamiento de Levenberg-Marquardt: si el
    costo sube se multiplica el amortiguamiento por 10 y se reintenta.
    Converge cuando el paso escalado y el cambio de costo son < `tol`.

    Valido si la estimacion inicial esta en la cuenca de atraccion (unos km
    para LEO) y el modelo `modelo` describe la dinamica; la covarianza es la
    FORMAL (inversa de la informacion) y solo cubre el ruido de medicion y el
    prior, no el error de modelo."""
    sg = _sigmas(sigmas)
    integ = integ or INT_STM
    n = 7 if estima_p else 6
    x = np.asarray(x0, float).copy()
    if len(x) < n:
        x = np.r_[x, 1.0]
    x = x[:n]
    S = _escala(n)
    if prior is not None:
        xb, Pb = np.asarray(prior[0], float)[:n], np.asarray(prior[1], float)[:n, :n]
        Pbi = np.linalg.inv(Pb)
    else:
        xb, Pbi = None, np.zeros((n, n))

    def costo(z, w, x):
        c = float(np.sum((z * w) ** 2))
        if xb is not None:
            d = x - xb
            c += float(d @ Pbi @ d)
        return c

    z, H, w, can, tt = _construir(med, x, t0, modelo, estima_p, sg, integ)
    c = costo(z, w, x)
    lam = float(lm0)
    hist = [c]
    convergio = False
    it = 0
    for it in range(1, max_iter + 1):
        Ht = (H * w[:, None]) * S[None, :]
        zt = z * w
        Pbi_s = Pbi * np.outer(S, S)
        N = Ht.T @ Ht + Pbi_s
        g = Ht.T @ zt
        if xb is not None:
            g = g - S * (Pbi @ (x - xb))
        aceptado = False
        for _ in range(12):
            A = N + lam * np.diag(np.diag(N))
            try:
                d = np.linalg.solve(A, g)
            except np.linalg.LinAlgError:
                d = np.linalg.lstsq(A, g, rcond=1e-12)[0]
            xn = x + S * d
            if estima_p:
                xn[6] = min(max(xn[6], limites_p[0]), limites_p[1])
            zn, Hn, wn, can_n, tt_n = _construir(med, xn, t0, modelo, estima_p, sg, integ)
            cn = costo(zn, wn, xn)
            if cn <= c * (1 + 1e-12):
                aceptado = True
                break
            lam = max(lam * 10.0, 1e-3)
        if not aceptado:
            break
        paso = float(np.max(np.abs(d)))
        dc = abs(c - cn) / max(c, 1.0)
        x, z, H, w, can, tt, c = xn, zn, Hn, wn, can_n, tt_n, cn
        lam = lam / 10.0 if lam > 1e-6 else 0.0
        hist.append(c)
        if paso < tol and dc < tol:
            convergio = True
            break
        if dc < 1e-12:
            convergio = True
            break
    # covarianza formal en el punto final
    Ht = (H * w[:, None]) * S[None, :]
    Nn = Ht.T @ Ht + Pbi * np.outer(S, S)
    sv = np.linalg.svd(Ht, compute_uv=False)
    try:
        Ps = np.linalg.inv(Nn)
    except np.linalg.LinAlgError:
        Ps = np.linalg.pinv(Nn, rcond=1e-14)
    Pcov = Ps * np.outer(S, S)
    Pcov = 0.5 * (Pcov + Pcov.T)
    nz = z * w
    dof = max(len(z) - n, 1)
    rms_n, rms_f, res = {}, {}, {}
    for cn_ in M.CANALES:
        m = can == cn_
        if m.any():
            rms_n[cn_] = float(np.sqrt(np.mean(nz[m] ** 2)))
            rms_f[cn_] = float(np.sqrt(np.mean(z[m] ** 2)))
            res[cn_] = (tt[m], nz[m])
    return Resultado(x=x, P=Pcov, t0=float(t0), modelo=modelo, estima_p=estima_p,
                     iteraciones=it, convergio=convergio, costo=c, n_obs=len(z),
                     chi2_dof=float(np.sum(nz ** 2) / dof), rms_norm=rms_n,
                     rms_fisico=rms_f, residuos=res, sv_norm=sv, historial=hist)


def prior_de_tle(x_tle, edad_dias, sigma_pos_km_dia=2.0, sigma_pos0_km=1.0,
                 estima_p=False, sigma_p=0.5, sigma_deriva_km_dia=2.0, mu=P.WGS84.mu):
    """Prior (x_bar, P_bar) para un estado derivado de un TLE de `edad_dias`.

    Dos partes, ambas supuestos documentados (docs/OD.md):
      1. Diagonal "laxa": sigma de posicion = hypot(1 km, 2 km/dia * edad) (el
         piso de ~1 km en la epoca y el 1-3 km/dia que documenta orbit.py para
         LEO, valor central) y sigma de velocidad = (v/r) * sigma_pos (un
         error de posicion a lo largo de la pista es un desfase de fase, que
         gira el vector velocidad el mismo angulo: 1 km = 1.1 m/s).
      2. Un termino de informacion sobre la ENERGIA orbital: el TLE conoce el
         semieje (el periodo) mucho mejor que lo que la diagonal sugiere. Un
         error de semieje da_ produce una deriva de fase 1.5 n da_ (km/s):
         pedir que esa deriva sea del orden de `sigma_deriva_km_dia` (2 km/dia)
         por max(1, edad en dias) (el error de semieje que deja el arrastre
         mal modelado crece con la edad) fija sigma_a = 14 m * max(1, edad) en
         LEO (2 km/dia = 14 m de semieje; la ISS decae ~70 m/dia y el 25 %
         de error de densidad da ~17 m por dia de edad). Se anade como
         P^-1 = D^-1 + g g^T / sigma_E^2 con g = dE/dx = [mu r/r^3, v] y
         sigma_E = mu sigma_a / (2 a^2). Sin este termino, una OD con SOLO
         angulos de un pase (que mide mal el periodo: 1 m/s de velocidad a lo
         largo de la pista son 250 km/dia de deriva) arruina la prediccion.
         `sigma_deriva_km_dia=None` lo omite.
    Ignora el resto de la correlacion posicion-velocidad. `sigma_p` es la
    incertidumbre relativa del factor de arrastre."""
    xb = np.asarray(x_tle, float)[:6]
    sp = math.hypot(sigma_pos0_km, sigma_pos_km_dia * abs(edad_dias))
    rn = float(np.linalg.norm(xb[:3]))
    vn = float(np.linalg.norm(xb[3:]))
    sv = vn / rn * sp
    d = np.array([sp] * 3 + [sv] * 3)
    info = np.diag(1.0 / d ** 2)
    if sigma_deriva_km_dia:
        a = 1.0 / (2.0 / rn - vn * vn / mu)
        n = math.sqrt(mu / a ** 3)
        sig_a = (sigma_deriva_km_dia * max(1.0, abs(edad_dias)) / 86400.0) / (1.5 * n)
        sig_e = mu * sig_a / (2.0 * a * a)
        g = np.r_[mu * xb[:3] / rn ** 3, xb[3:]]
        info = info + np.outer(g, g) / sig_e ** 2
    Pm = np.linalg.inv(info)
    if estima_p:
        Pb = np.zeros((7, 7))
        Pb[:6, :6] = Pm
        Pb[6, 6] = sigma_p ** 2
        return np.r_[xb, 1.0], Pb
    return xb, Pm


# ---------------------------------------------------------------- EKF

@dataclass
class ResultadoEKF:
    x: np.ndarray
    P: np.ndarray
    t: float
    modelo: P.Modelo
    estima_p: bool
    nis: np.ndarray             # innovacion normalizada al cuadrado, por medida escalar
    canal: np.ndarray
    t_nis: np.ndarray
    historial_pos_sigma: np.ndarray = None   # (t, sigma_pos) tras cada instante

    @property
    def t0(self):
        return self.t

    @property
    def sigma(self):
        return np.sqrt(np.diag(self.P))

    def sigma_pos_km(self):
        return float(math.sqrt(np.trace(self.P[:3, :3])))


def ekf(med, x0, P0, t0, modelo, sigmas=None, estima_p=False, q_acc=3e-21,
        integ=None, q_p=0.0, limites_p=(0.2, 3.0), iter_med=3):
    """EKF secuencial sobre las medidas en orden de tiempo, desde (t0, x0,
    P0). Prediccion con la STM del propagador y ruido de proceso de aceleracion
    blanca de densidad espectral `q_acc` (km^2/s^3). El valor "fisico"
    sigma_a^2 tau = 3e-17 (sigma_a = 7.5e-11 km/s^2, el 25 % de la aceleracion
    de arrastre de la ISS, y tau = 5600 s) resulto 4 veces PEOR en el
    experimento (docs/OD.md): el error de arrastre es un sesgo correlacionado a
    lo largo de dias, no ruido blanco, y el ruido blanco borra la informacion
    de velocidad de los pases anteriores. El defecto 3e-21 se ajusto en ese
    experimento (optimista para otros escenarios); con 0 el EKF reproduce los
    lotes. `q_p` es el paseo aleatorio del factor de arrastre (1/s). La
    actualizacion es escalar por canal (Joseph, iterada `iter_med` veces),
    valida porque el ruido de los canales es independiente.
    Linealiza alrededor de la propia estimacion: puede divergir si el prior
    esta a mas de unos km. Devuelve el estado en el ULTIMO instante."""
    sg = _sigmas(sigmas)
    integ = integ or INT_STM
    n = 7 if estima_p else 6
    x = np.asarray(x0, float)[:n].copy()
    if len(x) < n:
        x = np.r_[x, 1.0]
    Pm = np.asarray(P0, float)[:n, :n].copy()
    f = P.derivadas_stm(modelo, estima_p)
    t_prev = float(t0)
    nis, can, tn, hs = [], [], [], []
    med = med.ordenar()
    ts = np.unique(med.t)
    for tk in ts:
        if tk > t_prev:
            Y = P.propagar(f, P.y0_stm(x), t_prev, [tk], **integ)[0]
            x, Phi = Y[:n].copy(), Y[n:].reshape(n, n)
            dt_ = tk - t_prev
            Q = np.zeros((n, n))
            q3, q2, q1 = dt_ ** 3 / 3.0, dt_ ** 2 / 2.0, dt_
            for i in range(3):
                Q[i, i] = q_acc * q3
                Q[i, i + 3] = Q[i + 3, i] = q_acc * q2
                Q[i + 3, i + 3] = q_acc * q1
            if estima_p:
                Q[6, 6] = q_p * dt_
            Pm = Phi @ Pm @ Phi.T + Q
            t_prev = tk
        for k in np.where(med.t == tk)[0]:
            sit = med.sitios[med.isit[k]]
            for c in M.CANALES:
                y = med.val[c][k]
                if not np.isfinite(y):
                    continue
                # actualizacion ITERADA (IEKF, Jazwinski): re-linealiza h alrededor de la
                # estimacion actualizada `iter_med` veces. Con un a priori a 5-14 km
                # (un TLE de dias) la linealizacion unica de la medida sesga el estado.
                x_pred = x.copy()
                xi = x.copy()
                for it in range(max(1, int(iter_med))):
                    h = M.observables(med.reloj, sit, [tk], xi[None, 0:3], xi[None, 3:6])[c][0]
                    J = M.jacobiano(med.reloj, sit, [tk], xi[None, 0:3], xi[None, 3:6], (c,))[0, 0]
                    Hr = np.zeros(n)
                    Hr[:6] = J
                    res = y - h
                    if c == 'az':
                        res = float(M.envolver(res))
                    res = res - float(Hr @ (x_pred - xi))
                    s = float(Hr @ Pm @ Hr) + sg[c] ** 2
                    K = Pm @ Hr / s
                    xi = x_pred + K * res
                    if it == 0:
                        nis.append(res * res / s)
                x = xi
                if estima_p:
                    x[6] = min(max(x[6], limites_p[0]), limites_p[1])
                IKH = np.eye(n) - np.outer(K, Hr)
                Pm = IKH @ Pm @ IKH.T + sg[c] ** 2 * np.outer(K, K)
                can.append(c)
                tn.append(tk)
        hs.append((tk, math.sqrt(max(np.trace(Pm[:3, :3]), 0.0))))
    return ResultadoEKF(x, 0.5 * (Pm + Pm.T), t_prev, modelo, estima_p, np.array(nis),
                        np.array(can), np.array(tn), np.array(hs))


def propagar_resultado(res, t_fin, dt_s=30.0, metodo=None):
    """`Efemeride` de la orbita estimada (`Resultado` o `ResultadoEKF`) desde
    su epoca hasta `t_fin` (s). Sirve para predecir pases. El error de la
    prediccion crece con el modelo dinamico, no con la covarianza formal."""
    return efemeride(res.modelo, res.x, res.t0, res.t0, t_fin, dt_s, metodo)


# ---------------------------------------------------------------- consistencia

def nees(error, Pcov):
    """Error normalizado al cuadrado de la estimacion: e^T P^-1 e. Con un
    estimador consistente tiene media = dim(e) (chi2)."""
    e = np.asarray(error, float)
    return float(e @ np.linalg.solve(Pcov, e))


def chi2_limites(dof, n=1, nivel=0.95):
    """Limites (inferior, superior) de dos colas del promedio de `n` variables
    chi2(`dof`) independientes, por la aproximacion de Wilson-Hilferty (error
    < 1 % para dof >= 3). Para decidir si un NEES medio es compatible con la
    dimension."""
    z = {0.90: 1.6449, 0.95: 1.9600, 0.99: 2.5758}[nivel]

    def cuantil(zz):
        k = dof * n
        a = 2.0 / (9.0 * k)
        return k * (1 - a + zz * math.sqrt(a)) ** 3 / n
    return cuantil(-z), cuantil(z)


# ---------------------------------------------------------------- serializacion

def modelo_a_dict(m):
    d = {'grav': m.grav.nombre, 'grado': m.grado, 'arrastre': m.arrastre,
         'cd_a_m': m.cd_a_m, 'rho_factor': m.rho_factor}
    if m.rho_var:                 # solo la "verdad" de un escenario la lleva
        d['rho_var'] = [[float(x) for x in t] for t in m.rho_var]
    return d


def modelo_de_dict(d):
    grav = {'wgs84': P.WGS84, 'wgs72': P.WGS72}[d['grav']]
    return P.Modelo(grav=grav, grado=int(d['grado']), arrastre=bool(d['arrastre']),
                    cd_a_m=float(d['cd_a_m']), rho_factor=float(d['rho_factor']),
                    rho_var=tuple(tuple(t) for t in d.get('rho_var', ())))


def guardar_efemeride(ef, epoca_utc, ruta, nombre=''):
    """Guarda una `propagador.Efemeride` (t en s desde `epoca_utc`, r, v, a) en un
    .npz exacto: es la "verdad" de un escenario que el generador de referencia
    (`truth_file`) lee sin reintegrar. `epoca_utc`: datetime UTC del t = 0."""
    np.savez(ruta, t=ef.t, r=ef.r, v=ef.v, a=ef.a, nombre=np.array(str(nombre)),
             epoca_utc=np.array(epoca_utc.astimezone(UTC).strftime('%Y-%m-%dT%H:%M:%S.%f')))


def cargar_efemeride(ruta):
    """Inverso de `guardar_efemeride`: devuelve (Efemeride, Reloj)."""
    with np.load(ruta) as z:
        ef = P.Efemeride(z['t'], z['r'], z['v'], z['a'])
        epoca = orbit.a_utc(str(z['epoca_utc']))
    return ef, M.Reloj(epoca)


def resultado_json(res, reloj, nombre='', norad=None, estaciones=(), extra=None):
    """Resultado de la OD como dict serializable (formato del archivo de la
    orbita estimada que lee `reference_generator`): estado TEME y covarianza
    en la epoca `epoca_utc`."""
    t0 = res.t0
    d = {
        'version': 1, 'satelite': nombre, 'norad': norad,
        'epoca_utc': reloj.utc(t0).astimezone(UTC).strftime('%Y-%m-%dT%H:%M:%S.%f') + 'Z',
        'marco': 'TEME', 'unidades': 'km, km/s',
        'x': [float(v) for v in res.x], 'P': [[float(v) for v in fila] for fila in res.P],
        'sigma_pos_km': float(math.sqrt(np.trace(res.P[:3, :3]))),
        'estima_p': bool(res.estima_p), 'modelo': modelo_a_dict(res.modelo),
        'estaciones': list(estaciones),
    }
    if hasattr(res, 'n_obs'):
        d.update({'n_obs': int(res.n_obs), 'iteraciones': int(res.iteraciones),
                  'convergio': bool(res.convergio), 'chi2_dof': float(res.chi2_dof),
                  'rms_norm': res.rms_norm})
    d.update(extra or {})
    return d


def orbita_de_json(d):
    """dict (o ruta a JSON) -> (x, modelo, reloj, t0=0, estima_p). El reloj
    tiene como epoca la del estado."""
    if isinstance(d, str):
        with open(d) as f:
            d = json.load(f)
    reloj = M.Reloj(orbit.a_utc(d['epoca_utc']))
    return (np.asarray(d['x'], float), modelo_de_dict(d['modelo']), reloj,
            bool(d.get('estima_p', False)))


def prior_de_orbita(orbita, reloj, t_s, q_acc=3e-21, integ=None):
    """Prior para encadenar pases: el estado y la covarianza de una orbita
    estimada antes (dict de `resultado_json`, o ruta) llevados al instante
    `t_s` (s del `reloj` destino) con la STM del propagador, mas el ruido de
    proceso de aceleracion blanca `q_acc` (km^2/s^3, ver `ekf`) que cubre el
    error de modelo acumulado. Devuelve (x, P, estima_p) con las dimensiones
    del estado guardado (6 o 7). Vale hacia adelante en el tiempo (t_s
    posterior a la epoca de la orbita) y mientras el modelo del OD siga siendo
    razonable: dias, no semanas."""
    if isinstance(orbita, str):
        with open(orbita) as f:
            orbita = json.load(f)
    x, modelo, rl, estp = orbita_de_json(orbita)
    n = 7 if estp else 6
    Pm = np.asarray(orbita['P'], float)[:n, :n]
    t_p = reloj.segundos(rl.epoca)
    if t_s < t_p:
        raise ValueError('la orbita previa es POSTERIOR al instante pedido')
    Y = P.propagar(P.derivadas_stm(modelo, estp), P.y0_stm(x[:n]), t_p, [t_s],
                   **(integ or INT_STM))[0]
    Phi = Y[n:].reshape(n, n)
    dt_ = t_s - t_p
    Q = np.zeros((n, n))
    for i in range(3):
        Q[i, i] = q_acc * dt_ ** 3 / 3.0
        Q[i, i + 3] = Q[i + 3, i] = q_acc * dt_ ** 2 / 2.0
        Q[i + 3, i + 3] = q_acc * dt_
    return Y[:n].copy(), Phi @ Pm @ Phi.T + Q, estp


def perfil_de_efemeride(ef, reloj, sit, pase, dt_s=0.5):
    """Pase muestreado con el MISMO formato que `orbit.muestrear_pase` (t
    desde AOS, az, az_continuo, el en deg, d km, vr km/s, az_pto, el_pto
    deg/s, duracion, d_min, inicio_utc), para servir de referencia."""
    dur = float(pase['duracion_s'])
    n = max(16, int(math.ceil(dur / dt_s)) + 1)
    t = np.linspace(0.0, dur, n)
    r, v = ef.estado(pase['aos'] + t)
    o = M.observables(reloj, sit, pase['aos'] + t, r, v)
    az = np.degrees(o['az'])
    az_c = np.degrees(np.unwrap(o['az']))
    el = np.degrees(o['el'])
    return {'t': t, 'az': az, 'az_continuo': az_c, 'el': el, 'd': o['rng'],
            'vr': o['rr'], 'az_pto': np.gradient(az_c, t), 'el_pto': np.gradient(el, t),
            'duracion': dur, 'd_min': float(np.min(o['rng'])),
            'inicio_utc': reloj.utc(pase['aos'])}


def perfil_od(orbita, estacion=None, pase_idx=0, inicio_utc=None, horas=72.0,
              mascara_deg=None, el_max_min_deg=0.0, dt_s=0.5,
              archivo_estaciones=None):
    """Como `orbit.perfil_tle` pero con la ORBITA ESTIMADA (dict o ruta del
    JSON de `resultado_json`) propagada con el propagador numerico: elige el
    `pase_idx`-esimo pase desde `inicio_utc` (por defecto, la epoca de la
    orbita) y lo muestrea. Devuelve (perfil, pase, orbita_dict, estacion);
    `pase` tiene aos/los/t_el_max como datetimes UTC, mas duracion_s y
    el_max_deg. Lanza LookupError si no hay tantos pases. Valido mientras la
    prediccion lo sea: el error crece con los dias desde la epoca de la OD."""
    if isinstance(orbita, str):
        with open(orbita) as f:
            orbita = json.load(f)
    x, modelo, reloj, _ = orbita_de_json(orbita)
    est = orbit.buscar_estacion(estacion, archivo_estaciones)
    sit = M.sitio(est)
    t_ini = max(0.0, reloj.segundos(orbit.a_utc(inicio_utc))) if inicio_utc else 0.0
    t_fin = t_ini + float(horas) * 3600.0
    ef = efemeride(modelo, x, 0.0, 0.0, t_fin + 60.0, 30.0)
    pases = [p for p in pases_efemeride(ef, reloj, sit, t_ini, t_fin, mascara_deg)
             if p['el_max_deg'] >= el_max_min_deg]
    if not 0 <= int(pase_idx) < len(pases):
        raise LookupError(f'orbita estimada sobre {est.nombre}: pase_index={pase_idx} pero '
                          f'solo hay {len(pases)} pases en {horas:g} h')
    p = pases[int(pase_idx)]
    perfil = perfil_de_efemeride(ef, reloj, sit, p, dt_s)
    pase = {'aos': reloj.utc(p['aos']), 'los': reloj.utc(p['los']),
            't_el_max': reloj.utc(p['t_el_max']), 'duracion_s': p['duracion_s'],
            'el_max_deg': p['el_max_deg'], 'aos_truncado': p['aos_truncado'],
            'rango_min_km': perfil['d_min'],
            'az_aos_deg': float(perfil['az'][0]), 'az_los_deg': float(perfil['az'][-1])}
    return perfil, pase, orbita, est
