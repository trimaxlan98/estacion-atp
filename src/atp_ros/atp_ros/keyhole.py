"""Estrategias de keyhole para la montura Az/El del seguidor satelital (sin ROS).

El problema: en un pase casi cenital el acimut del satelite
da un giro de ~180 grados en unos segundos. Su velocidad pico es
`model.tasa_acimut(h, el_max)`, que diverge como 1/cos(el_max); cuando supera
el tope del rotor (`model.elevacion_keyhole`: 82.5 deg a 550 km con 6 deg/s) la
montura que solo "sigue la referencia" se queda atras y pierde el satelite.

Una estrategia de keyhole convierte la direccion verdadera del satelite
(az_continuo, el) en la ORDEN a las articulaciones (az', el'). El error de
apuntamiento SIEMPRE se mide contra la direccion verdadera, en el cielo.

  ingenua      orden = direccion verdadera (comportamiento de siempre).
  anticipada   orden = trayectoria de articulaciones que MINIMIZA el error
               maximo de apuntamiento en el cielo con tope de velocidad y de
               aceleracion (programa lineal, ver `planificar`): el acimut
               empieza a girar ANTES de la culminacion.
  sobre_cenit  montura con elevacion hasta 180 deg (`el_montura_max_deg`): se
               cruza el cenit sin el giro de 180 deg del acimut (el' pasa de
               90 a 180 - el). Por pase se elige el mejor entre cruzar o
               seguir directo segun el error planificado.

Metodo de `planificar` (justificacion). Sea e(t) el error en el cielo. Cerca del
cenit un error de acimut D vale ~ D cos(el): el acimut puede ir "atrasado"
muchos grados sin que el haz lo note. Se busca la orden que minimiza
eps = max_t sqrt(e_az^2 cos^2 el + e_el^2), acotada por
    |dx/dt| <= v_max,  |d2x/dt2| <= a_max  (cada articulacion),
que es un problema CONVEXO: con la cota de componentes
    |az' - az| cos(el) <= eps,  |el' - el| <= eps
resulta un PROGRAMA LINEAL en (az', el', eps) (en grados), resuelto con HiGHS
(scipy.optimize.linprog). eps es el maximo de las COMPONENTES del error y, como
sqrt(a^2 + b^2) >= max(a, b), es una COTA INFERIOR del error en el cielo que
logra cualquier controlador con esos topes (igual a el cuando una sola
componente manda, que es lo habitual). Es el optimo global de ese problema (no
una heuristica); el lazo real (PID + planta) se mide aparte contra el. Un pequeno termino L1 en el objetivo hace
que la orden siga a la verdad donde no hace falta separarse.

Contraste independiente (sin LP, sin SGP4): `contraste_analitico` usa el
modelo PLANAR cerca del cenit (pase recto a distancia m = 90 - el_max del
cenit, velocidad angular v/h) y el algoritmo de alcanzabilidad por intervalos
(`eps_tubo_planar`, solo velocidad). Para el cruce del cenit el error minimo es
exactamente m: la montura PASA por el cenit y el satelite siempre esta a >= m.

Los limites del modelo estan en cada funcion; todo es numpy + scipy.
"""
import math

import numpy as np

from atp_ros import model

ESTRATEGIAS = ('ingenua', 'anticipada', 'sobre_cenit')

# ---------------------------------------------------------------- haces

# Bandas del seguidor. VHF/UHF (CubeSat tipico, 437 MHz) llevan una Yagi; S y X,
# el plato de 3 m del resto del proyecto.
BANDAS_SEGUIDOR = {'VHF': 145e6, 'UHF': 437e6, 'S': 2.2e9, 'X': 8.4e9}
GANANCIA_YAGI_DBI = 15.0      # Yagi cruzada de ~15 dBi (supuesto: 13-17 elementos)
ETA_YAGI = 0.6                # misma eficiencia que el plato (model.ganancia_plato)


def diametro_equivalente_yagi(f_hz, ganancia_dbi=GANANCIA_YAGI_DBI, eta=ETA_YAGI):
    """Apertura circular equivalente D (m) de una antena de ganancia G a f:
    G = eta (pi D / lambda)^2  ->  D = lambda sqrt(G / eta) / pi. Con 15 dBi,
    D = 2.31 lambda (1.59 m a 437 MHz)."""
    lam = model.C_LUZ_M_S / float(f_hz)
    g = 10.0 ** (float(ganancia_dbi) / 10.0)
    return float(lam * math.sqrt(g / float(eta)) / math.pi)


def ancho_haz_banda(banda):
    """theta_3dB (grados) de la antena que usa cada banda: 70 lambda / D con el
    plato de 3 m en S (3.18 deg) y X (0.83 deg), y con la apertura equivalente
    de una Yagi de 15 dBi en VHF/UHF (30.3 deg). La Yagi es un SUPUESTO: una
    Yagi corta (10 dBi) da ~55 deg y una larga (18 dBi) ~21 deg."""
    f = BANDAS_SEGUIDOR[banda]
    if banda in ('S', 'X'):
        return model.ancho_haz(model.DIAMETRO_PLATO_M, f)
    return model.ancho_haz(diametro_equivalente_yagi(f), f)


def semihaz_3db(banda):
    """Error de apuntamiento (grados) en el que el haz pierde 3 dB: theta_3dB/2
    (`model.perdida_apuntamiento`). Es el criterio de "fuera del haz"."""
    return 0.5 * ancho_haz_banda(banda)


# ---------------------------------------------------------------- geometria

def _unit(az_deg, el_deg):
    az, el = np.radians(az_deg), np.radians(el_deg)
    return np.stack([np.cos(el) * np.sin(az), np.cos(el) * np.cos(az), np.sin(el)], axis=-1)


def a_cielo(az_art_deg, el_art_deg):
    """Articulaciones (az', el') -> direccion en el cielo (az, el) con el en
    [0, 90]. Una montura con el' > 90 (sobre el cenit) apunta a
    (az' + 180, 180 - el'); con el' <= 90 es la identidad. `az` sale con el
    valor de az'(+180), sin envolver."""
    az = np.asarray(az_art_deg, dtype=float)
    el = np.asarray(el_art_deg, dtype=float)
    flip = el > 90.0
    return np.where(flip, az + 180.0, az), np.where(flip, 180.0 - el, el)


def separacion_deg(az1_deg, el1_deg, az2_deg, el2_deg):
    """Distancia angular (grados) entre dos direcciones (az, el): exacta en
    toda la esfera (atan2 de |a x b| y a . b), no necesita angulos pequenos."""
    a, b = _unit(az1_deg, el1_deg), _unit(az2_deg, el2_deg)
    cr = np.linalg.norm(np.cross(a, b), axis=-1)
    return np.degrees(np.arctan2(cr, np.sum(a * b, axis=-1)))


def error_apuntamiento(az_ver, el_ver, az_art, el_art):
    """Error en el cielo (grados) de la montura (articulaciones az', el') con
    respecto a la direccion verdadera. Lo mismo que mide `error_monitor` con
    `esferico:=true`."""
    az_m, el_m = a_cielo(az_art, el_art)
    return separacion_deg(az_ver, el_ver, az_m, el_m)


# ---------------------------------------------------------------- planificacion

def _resolver_lp(tp, ax, ay, w, vel, acel, el_lim, mu=1e-3):
    """Programa lineal descrito arriba. Variables [x (N), y (N), ux (N), uy (N),
    eps]. Minimiza eps + mu/N * sum(ux + uy) con ux >= w|x - ax|, uy >= |y - ay|.
    Devuelve (x, y, eps) en grados, o None si es infactible."""
    from scipy.optimize import linprog
    from scipy.sparse import coo_matrix, vstack, identity, diags

    n = len(tp)
    dt = float(tp[1] - tp[0])
    nv = 4 * n + 1
    iE = 4 * n
    f = lambda a: coo_matrix(a)                              # noqa: E731
    z = coo_matrix((n, n))
    I = identity(n, format='coo')
    W = diags(w, format='coo')
    col_eps = coo_matrix((np.full(n, -1.0), (np.arange(n), np.zeros(n, int))), shape=(n, 1))
    zc = coo_matrix((n, 1))
    bloques, rhs = [], []

    def agregar(blk, b):
        bloques.append(blk.tocsr())
        rhs.append(np.asarray(b, dtype=float))

    # cota por componente: w|x-ax| <= eps ; |y-ay| <= eps
    from scipy.sparse import hstack
    agregar(hstack([W, z, z, z, col_eps]), w * ax)
    agregar(hstack([-W, z, z, z, col_eps]), -w * ax)
    agregar(hstack([z, I, z, z, col_eps]), ay)
    agregar(hstack([z, -I, z, z, col_eps]), -ay)
    # holguras L1:  w(x-ax) - ux <= 0 ...
    agregar(hstack([W, z, -I, z, zc]), w * ax)
    agregar(hstack([-W, z, -I, z, zc]), -w * ax)
    agregar(hstack([z, I, z, -I, zc]), ay)
    agregar(hstack([z, -I, z, -I, zc]), -ay)
    # velocidad: |x_{k+1} - x_k| <= vel dt
    D1 = diags([-np.ones(n - 1), np.ones(n - 1)], [0, 1], shape=(n - 1, n), format='coo')
    z1 = coo_matrix((n - 1, n))
    zc1 = coo_matrix((n - 1, 1))
    lv = np.full(n - 1, vel * dt)
    for sgn in (1.0, -1.0):
        agregar(hstack([sgn * D1, z1, z1, z1, zc1]), lv)
        agregar(hstack([z1, sgn * D1, z1, z1, zc1]), lv)
    # aceleracion: |x_{k+1} - 2 x_k + x_{k-1}| <= acel dt^2
    D2 = diags([np.ones(n - 2), -2.0 * np.ones(n - 2), np.ones(n - 2)], [0, 1, 2],
               shape=(n - 2, n), format='coo')
    z2 = coo_matrix((n - 2, n))
    zc2 = coo_matrix((n - 2, 1))
    la = np.full(n - 2, acel * dt * dt)
    for sgn in (1.0, -1.0):
        agregar(hstack([sgn * D2, z2, z2, z2, zc2]), la)
        agregar(hstack([z2, sgn * D2, z2, z2, zc2]), la)
    A = vstack(bloques).tocsr()
    b = np.concatenate(rhs)
    c = np.zeros(nv)
    c[2 * n:4 * n] = mu / n
    c[iE] = 1.0
    cotas = [(None, None)] * n + [(0.0, float(el_lim))] * n + [(0.0, None)] * (2 * n + 1)
    cotas[0] = (float(ax[0]), float(ax[0]))                  # parte apuntando al AOS
    cotas[n] = (float(ay[0]), float(ay[0]))
    res = linprog(c, A_ub=A, b_ub=b, bounds=cotas, method='highs')
    if res.status != 0:
        return None
    x = res.x
    return x[:n], x[n:2 * n], float(x[iE])


def planificar(t, az_deg, el_deg, estrategia='anticipada', vel_max_deg_s=6.0,
               acel_max_deg_s2=20.0, el_montura_max_deg=90.0, dt_plan_s=0.25,
               frac_vel=1.0):
    """Orden de articulaciones para un pase.

    t (s), az_deg (CONTINUO, desenroscado), el_deg: direccion verdadera,
    muestreada con paso <= ~0.1 s cerca del cenit (si no, el propio objetivo
    esta submuestreado: es la fisica del keyhole, no un error del metodo).
    Devuelve un dict: az_cmd, el_cmd (sobre la rejilla `t`), eps_deg (el error
    maximo PLANIFICADO, cota inferior de lo que logra cualquier lazo), aplica
    (False si la estrategia no puede usarse: sobre_cenit con la montura
    limitada), modo ('directo' | 'sobre_cenit'), motivo.

    Topes: vel_max_deg_s * frac_vel y acel_max_deg_s2 en cada articulacion;
    el' <= el_montura_max_deg. `frac_vel` = 1: el optimo usa TODA la velocidad
    del rotor (con 0.9 el error planificado a 85 grados sube de 0.92 a 1.32:
    es muy sensible); el tope de aceleracion casi no cambia el optimo (el giro
    se reparte durante decenas de segundos antes de la culminacion) salvo que
    sea < ~0.1 deg/s2. La primera orden es la direccion del AOS (la montura
    parte apuntando ahi)."""
    if estrategia not in ESTRATEGIAS:
        raise ValueError(f'estrategia desconocida: {estrategia!r} (hay {ESTRATEGIAS})')
    t = np.asarray(t, dtype=float)
    az = np.asarray(az_deg, dtype=float)
    el = np.asarray(el_deg, dtype=float)
    base = {'az_cmd': az.copy(), 'el_cmd': el.copy(), 'eps_deg': 0.0, 'aplica': True,
            'modo': 'directo', 'motivo': ''}
    if estrategia == 'ingenua':
        base['motivo'] = 'la orden es la direccion verdadera'
        return base

    n = int(math.ceil((t[-1] - t[0]) / dt_plan_s)) + 1
    tp = t[0] + np.arange(n) * dt_plan_s
    ax = np.interp(tp, t, az)
    ay = np.interp(tp, t, el)
    w = np.maximum(np.cos(np.radians(ay)), 1e-3)
    vel = float(vel_max_deg_s) * float(frac_vel)

    def resolver(ax_, ay_, w_, el_lim):
        r = _resolver_lp(tp, ax_, ay_, w_, vel, float(acel_max_deg_s2), el_lim)
        return r

    directo = resolver(ax, ay, w, 90.0)
    candidatos = [('directo', directo, None)]
    if estrategia == 'sobre_cenit':
        el_fin = float(np.min(el[-3:]))
        if float(el_montura_max_deg) < 180.0 - el_fin:
            base.update(aplica=False, motivo=(
                f'la montura llega a {el_montura_max_deg:g} deg de elevacion y el cruce '
                f'pide {180.0 - el_fin:.1f}: la estrategia no aplica'))
            return base
        kc = int(np.argmax(ay))
        sgn = 1.0 if az[-1] >= az[0] else -1.0
        ax_f = ax.copy()
        ay_f = ay.copy()
        ax_f[kc:] = ax[kc:] - 180.0 * sgn
        ay_f[kc:] = 180.0 - ay[kc:]
        # el peso es el coseno de la elevacion del CIELO (ay), no el de la articulacion
        w_f = w
        candidatos.append(('sobre_cenit', resolver(ax_f, ay_f, w_f, float(el_montura_max_deg)),
                           (ax_f, ay_f)))
    validos = [(m, r) for m, r, _ in candidatos if r is not None]
    if not validos:
        base.update(aplica=False, motivo='el programa lineal es infactible con esos topes')
        return base
    modo, (x, y, eps) = min(validos, key=lambda mr: mr[1][2])
    if estrategia == 'anticipada':
        modo = 'directo'
    base.update(az_cmd=np.interp(t, tp, x), el_cmd=np.interp(t, tp, y), eps_deg=eps,
                modo=modo, motivo='ok')
    base['eps_directo_deg'] = None if directo is None else float(directo[2])
    return base


# ---------------------------------------------------------------- referencia: montura X-Y

def a_montura_xy(az_deg, el_deg):
    """Direccion (az, el) -> angulos de una montura X-Y con el eje primario
    horizontal hacia el NORTE: p = (E, N, U) = (cos y sin x, sin y, cos y cos x),
    x = atan2(E, U) (giro sobre el eje norte), y = asin(N) (eje secundario). La
    singularidad esta donde cos y = 0, es decir en el horizonte al norte (y al
    sur): el keyhole se desplaza al horizonte, que la mascara de elevacion ya
    descarta. SOLO REFERENCIA (analisis del documento): no hay lazo ni montura
    X-Y en el banco."""
    p = _unit(np.asarray(az_deg, float), np.asarray(el_deg, float))
    x = np.degrees(np.unwrap(np.arctan2(p[..., 0], p[..., 2])))
    y = np.degrees(np.arcsin(np.clip(p[..., 1], -1.0, 1.0)))
    return x, y


def tasa_max_articulaciones(t, az_deg, el_deg, montura='azel'):
    """Velocidad maxima (deg/s) de cada articulacion para seguir EXACTAMENTE
    el pase: Az/El (az', el') o X-Y (x, y). La demanda que tendria que
    cubrir el rotor sin ninguna estrategia."""
    if montura == 'azel':
        a, b = np.asarray(az_deg, float), np.asarray(el_deg, float)
    elif montura == 'xy':
        a, b = a_montura_xy(az_deg, el_deg)
    else:
        raise ValueError("montura debe ser 'azel' o 'xy'")
    return (float(np.max(np.abs(np.gradient(a, t)))), float(np.max(np.abs(np.gradient(b, t)))))


# ---------------------------------------------------------------- contraste independiente

def pase_planar(el_max_deg, h_km, mascara_deg=5.0, dt=0.05, n_sigma=None):
    """Pase RECTO en el plano tangente (aproximacion cerca del cenit, SIN
    SGP4 ni geometria esferica): el satelite cruza a distancia angular
    m = 90 - el_max del cenit con velocidad angular constante Omega = v/h
    (rad/s; en el cenit es exacta: la velocidad es perpendicular a la visual).
    Devuelve t, az (continuo, deg), el (deg) y m. Valido para zenith angle
    < ~30 deg (a mas, la tasa angular real cae por la curvatura): es lo que
    importa al keyhole; fuera de ahi la montura no tiene problema."""
    h = float(h_km)
    omega = model.velocidad_circular(h) / h               # rad/s
    m = math.radians(90.0 - float(el_max_deg))
    zmax = math.radians(30.0) if n_sigma is None else float(n_sigma)
    s_max = math.sqrt(max(zmax ** 2 - m ** 2, 0.0))
    t = np.arange(-s_max / omega, s_max / omega + dt, dt)
    x, y = omega * t, m
    zen = np.hypot(x, y)
    az = np.degrees(np.unwrap(np.arctan2(x, y)))          # medido desde el punto de cruce
    el = 90.0 - np.degrees(zen)
    return {'t': t - t[0], 'az': az - az[0], 'el': el, 'm_deg': math.degrees(m)}


def seguidor_limitado(t, objetivo, vel_max):
    """Persecucion ideal con tope de velocidad: x_{k+1} = x_k + clip(obj_{k+1} -
    x_k, +-v dt). Es lo que hace la montura "ingenua" con ganancia infinita:
    el mejor caso de seguir la referencia con un rotor de v_max."""
    x = np.empty_like(objetivo)
    x[0] = objetivo[0]
    dt = np.diff(t)
    for k in range(len(t) - 1):
        paso = vel_max * dt[k]
        x[k + 1] = x[k] + np.clip(objetivo[k + 1] - x[k], -paso, paso)
    return x


def eps_tubo_planar(t, az, el, vel_max, tol=1e-4):
    """Error minimo (grados) de una montura de VELOCIDAD limitada (sin tope de
    aceleracion) que siga a (az, el): biseccion sobre eps y, para cada eps,
    alcanzabilidad por intervalos hacia adelante (el acimut debe quedar en
    [az - eps/(sqrt2 cos el), az + ...] y moverse <= v dt). SIN programa
    lineal: contraste independiente de `planificar` con acel_max grande.
    La elevacion se supone seguida exactamente (su tasa es chica)."""
    az = np.asarray(az, float)
    w = np.maximum(np.cos(np.radians(el)), 1e-3)
    dt = np.diff(t)

    def factible(eps):
        anc = eps / w
        lo = hi = az[0]
        for k in range(len(t) - 1):
            lo = max(lo - vel_max * dt[k], az[k + 1] - anc[k + 1])
            hi = min(hi + vel_max * dt[k], az[k + 1] + anc[k + 1])
            if lo > hi + 1e-12:
                return False
        return True

    a, b = 0.0, 90.0
    for _ in range(80):
        if b - a < tol:
            break
        mid = 0.5 * (a + b)
        a, b = (a, mid) if factible(mid) else (mid, b)
    return float(b)


def contraste_analitico(el_max_deg, h_km=model.H_LEO_KM, vel_max_deg_s=6.0, dt=0.05):
    """Cifras analiticas / semianaliticas con las que se contrasta la medicion:

    * el_keyhole_deg      `model.elevacion_keyhole` (rotor satura en la culminacion)
    * demanda_az_deg_s    `model.tasa_acimut` (1/cos(el_max))
    * satura              demanda > tope del rotor
    * eps_ingenua_deg     error de la persecucion ideal con tope de velocidad
                          (planar; la montura "ingenua" ideal)
    * eps_anticipada_deg  error minimo con tope de velocidad (planar, tubo)
    * eps_cenit_deg       cruce del cenit: la montura pasa por el cenit y el
                          satelite siempre esta a >= m = 90 - el_max: eps = m
    Todo en el modelo planar de `pase_planar` (valido cerca del cenit)."""
    p = pase_planar(el_max_deg, h_km, dt=dt)
    x_ing = seguidor_limitado(p['t'], p['az'], vel_max_deg_s)
    e_ing = error_apuntamiento(p['az'], p['el'], x_ing, p['el'])
    return {
        'el_max_deg': float(el_max_deg),
        'el_keyhole_deg': model.elevacion_keyhole(h_km, vel_max_deg_s),
        'demanda_az_deg_s': model.tasa_acimut(h_km, el_max_deg),
        'satura': bool(model.tasa_acimut(h_km, el_max_deg) > vel_max_deg_s),
        'eps_ingenua_deg': float(np.max(e_ing)),
        'eps_anticipada_deg': eps_tubo_planar(p['t'], p['az'], p['el'], vel_max_deg_s),
        'eps_cenit_deg': float(p['m_deg']),
    }


def umbral_analitico(estrategia, banda, h_km=model.H_LEO_KM, vel_max_deg_s=6.0, dt=0.1):
    """Elevacion maxima (grados) a partir de la cual el error ANALITICO (modelo
    planar de `contraste_analitico`) supera el semihaz de -3 dB de la banda, es
    decir, donde la estrategia empieza a sacar al satelite del haz; None si no
    ocurre antes de 89.99. Para `sobre_cenit` es 90 - semihaz (el cruce del
    cenit cuesta exactamente la distancia al cenit), siempre que el rotor
    alcance. Es el contraste de la elevacion de perdida."""
    sh = semihaz_3db(banda)
    if estrategia == 'sobre_cenit':
        return float(90.0 - sh)
    clave = {'ingenua': 'eps_ingenua_deg', 'anticipada': 'eps_anticipada_deg'}[estrategia]
    if contraste_analitico(89.99, h_km, vel_max_deg_s, dt)[clave] < sh:
        return None
    lo, hi = model.elevacion_keyhole(h_km, vel_max_deg_s), 89.99
    for _ in range(25):
        mid = 0.5 * (lo + hi)
        if contraste_analitico(mid, h_km, vel_max_deg_s, dt)[clave] < sh:
            lo = mid
        else:
            hi = mid
    return float(0.5 * (lo + hi))


# ---------------------------------------------------------------- lazo (banco sin ROS)

def simular_lazo(t, az_cmd, el_cmd, vel_max_deg_s=6.0, kp=25.0, ki=2.0, kd=8.0,
                 tau_d=0.08, u_max=8.0, dt=0.05, h_planta=0.002, **planta_kw):
    """El MISMO lazo que el lockstep, sin ROS: PID digital a 20 Hz con derivada
    filtrada y anti-windup por clamping (`pid_controller.py`) sobre
    `planta.PlantaMontura` (J, b, tope de par y de velocidad), con el mando
    retenido entre pasos. La montura parte alineada con la primera orden.
    `t` debe cubrir el pase; devuelve (tk, az_art, el_art) en grados a la
    cadencia del lazo. Determinista (sin azar salvo que se pase ruido)."""
    from atp_ros.planta import PlantaMontura
    tk = np.arange(0.0, float(t[-1]) + 1e-9, dt)
    ref = np.stack([np.interp(tk, t, az_cmd), np.interp(tk, t, el_cmd)])
    pl = PlantaMontura(model.J_EJE, model.B_EJE, math.radians(vel_max_deg_s)
                       if vel_max_deg_s > 0 else math.inf, h_planta, **planta_kw)
    pl.enganchar(np.radians(ref[:, 0]))
    integ = np.zeros(2)
    d_filt = np.zeros(2)
    e_prev = np.zeros(2)
    alfa = dt / (tau_d + dt)
    n_sub = int(round(dt / h_planta))
    sal = np.zeros((2, len(tk)))
    for k in range(len(tk)):
        meas = pl.theta.copy()
        sal[:, k] = meas
        e = np.radians(ref[:, k]) - meas
        d_filt += alfa * ((e - e_prev) / dt - d_filt)
        u_raw = kp * e + ki * integ + kd * d_filt
        u = np.clip(u_raw, -u_max, u_max)
        congelar = (np.abs(u_raw) > u_max + 1e-12) & (u_raw * e > 0.0)
        integ += np.where(congelar, 0.0, e * dt)
        e_prev = e
        for _ in range(n_sub):
            pl.paso(u, 0.0, h_planta)
    return tk, np.degrees(sal[0]), np.degrees(sal[1])


# ---------------------------------------------------------------- metricas

def metricas(t, e_pt, bandas=('VHF', 'UHF', 'S', 'X'), margen_db=None):
    """Metricas de un pase a partir del error de apuntamiento e_pt(t) (grados).

    rms, max; por banda: `fuera_s` (tiempo con e > theta_3dB/2: el haz pierde
    mas de 3 dB), `fuera_pct` y `max_sobre_haz` (max e / semihaz); y, si hay
    `margen_db` (margen de enlace de S en cada instante), `datos_perdidos_pct`
    = % del pase con margen < 0 (el enlace no cierra). Integra con la regla
    del rectangulo sobre la rejilla (paso medio de t)."""
    t = np.asarray(t, float)
    e = np.asarray(e_pt, float)
    dt = np.gradient(t) if len(t) > 1 else np.ones(1)
    dur = float(np.sum(dt))
    out = {'rms_deg': float(np.sqrt(np.mean(e ** 2))), 'max_deg': float(np.max(e)),
           'duracion_s': dur, 'bandas': {}}
    for b in bandas:
        sh = semihaz_3db(b)
        fuera = e > sh
        out['bandas'][b] = {'semihaz_deg': sh, 'fuera_s': float(np.sum(dt[fuera])),
                            'fuera_pct': float(100.0 * np.sum(dt[fuera]) / dur),
                            'max_sobre_haz': float(np.max(e) / sh)}
    if margen_db is not None:
        m = np.asarray(margen_db, float)
        out['datos_perdidos_pct'] = float(100.0 * np.sum(dt[m < 0.0]) / dur)
    return out


def margen_enlace_s(d_km, e_pt):
    """Margen de enlace de S (dB) en cada instante con el presupuesto por
    defecto del proyecto (`model.presupuesto_enlace`: CubeSat 9 dBW, 1 Mbps,
    QPSK, plato de 3 m): mismo calculo que el nodo link_budget."""
    return model.presupuesto_enlace(d_km, e_pt, f_hz=model.BANDAS['S'])['margen_db']


def evaluar_pase(perfil, estrategia, vel_max_deg_s=6.0, acel_max_deg_s2=20.0,
                 el_montura_max_deg=90.0, bandas=('VHF', 'UHF', 'S', 'X'), **plan_kw):
    """Planifica + lazo + metricas de UN pase (banco sin ROS). `perfil` es el
    dict de `model.perfil_pase` / `orbit.muestrear_pase` (t, az_continuo, el, d)."""
    plan = planificar(perfil['t'], perfil['az_continuo'], perfil['el'], estrategia,
                      vel_max_deg_s, acel_max_deg_s2, el_montura_max_deg, **plan_kw)
    if not plan['aplica']:
        return {'aplica': False, 'motivo': plan['motivo'], 'estrategia': estrategia}
    tk, az_a, el_a = simular_lazo(perfil['t'], plan['az_cmd'], plan['el_cmd'], vel_max_deg_s)
    az_v = np.interp(tk, perfil['t'], perfil['az_continuo'])
    el_v = np.interp(tk, perfil['t'], perfil['el'])
    d = np.interp(tk, perfil['t'], perfil['d'])
    e = error_apuntamiento(az_v, el_v, az_a, el_a)
    m = metricas(tk, e, bandas, margen_enlace_s(d, e))
    m.update(aplica=True, estrategia=estrategia, modo=plan['modo'],
             eps_plan_deg=plan['eps_deg'])
    return m


# ---------------------------------------------------------------- pases SGP4 reales

def estacion_bajo_traza(tle, t_centro, gamma_rad, nombre='keyhole', mascara_deg=5.0):
    """Estacion terrena desplazada `gamma_rad` (angulo central, perpendicular a
    la traza) del punto subsatelite en `t_centro`. gamma = 0 deja al satelite
    pasando por el cenit; crece -> el_max baja. Es una ESTACION A MEDIDA para
    obtener, con el SGP4 y el TLE reales, pases de la elevacion maxima que se
    pide (los pases cenitales son raros sobre una estacion dada)."""
    from atp_ros import orbit
    jd, fr = orbit.a_jd(t_centro)
    r, v = orbit.propagar_ecef(tle, np.array([jd]), np.array([fr]))
    r, v = r[0], v[0]
    u = r / np.linalg.norm(r)
    vt = v - np.dot(v, u) * u
    vt /= np.linalg.norm(vt)
    nrm = np.cross(u, vt)
    d = math.cos(gamma_rad) * u + math.sin(gamma_rad) * nrm
    p = d * model.R_TIERRA_KM
    lat, lon, _ = orbit.ecef_a_geodetica(p.reshape(1, 3))
    return orbit.Estacion(nombre=nombre, lat_deg=float(np.ravel(lat)[0]),
                          lon_deg=float(np.ravel(lon)[0]), alt_m=0.0,
                          mascara_deg=mascara_deg)


def pase_real(tle, t_centro, el_max_objetivo_deg, dt_s=0.05, mascara_deg=5.0, ventana_s=1500.0):
    """Pase SGP4 real del `tle` cuya elevacion maxima es `el_max_objetivo_deg`
    (+-0.05): coloca la estacion bajo la traza por biseccion sobre gamma.
    Devuelve (perfil, pase, estacion). `perfil` con paso `dt_s` (0.05 s: la
    cadencia del lazo; con 0.5 s el acimut cenital queda submuestreado)."""
    import datetime as dt
    from atp_ros import orbit
    t0 = t_centro - dt.timedelta(seconds=ventana_s / 2)

    def pase_de(gam):
        est = estacion_bajo_traza(tle, t_centro, gam, mascara_deg=mascara_deg)
        ps = orbit.predecir_pases(tle, est, t0, ventana_s / 3600.0, mascara_deg)
        if not ps:
            return est, None
        return est, max(ps, key=lambda p: p['el_max_deg'])

    lo, hi = 0.0, math.radians(20.0)
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        _, p = pase_de(mid)
        elm = -1.0 if p is None else p['el_max_deg']
        if abs(elm - el_max_objetivo_deg) < 0.02:
            break
        if elm > el_max_objetivo_deg:
            lo = mid
        else:
            hi = mid
    est, pase = pase_de(mid)
    if pase is None:
        raise LookupError('no hay pase en la ventana')
    return orbit.muestrear_pase(tle, est, pase, dt_s), pase, est
