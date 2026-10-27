"""Dinamica de un rover de seis ruedas (CONTRATO sec. 18): cuerpo rigido 6-DOF + contacto por rueda + Bekker-Wong.

Estado (19): pos ENU [x,y,z] m, cuaternion cuerpo->mundo [w,x,y,z], velocidad lineal en el mundo, velocidad angular en
el cuerpo, y la velocidad de giro de las 6 ruedas. Integrador RK4 de paso fijo DT = 0.01 s (el mando se mantiene
constante en el paso). Sin azar: mismas entradas, mismos bytes.

Cuerpo: eje x adelante, y a la izquierda, z arriba. Ruedas 0..2 a la izquierda (delante, medio, atras), 3..5 a la derecha.

Contacto por rueda (sobre la normal local n del heightmap):
  * normal: resorte-amortiguador sobre la penetracion de una esfera de radio Rw (Fn >= 0, nunca tira);
  * tracción: Bekker-Wong simplificada. Hundimiento z = (Fn / (b sqrt(D) kp))^(1/(n+1/2)), longitud de contacto
    L = sqrt(D z), kp = kc/b + kphi. Empuje con deslizamiento s:
        T(s) = (c b L + Fn tan(phi)) (1 - K/(s L) (1 - exp(-s L/K)))        (Janosi-Hanamoto integrado, Wong 2010)
  * resistencia por compactacion Rc = b kp z^(n+1)/(n+1) (siempre contra el avance);
  * lateral: Coulomb regularizado, Fy = -Flat tanh(vy/VS) acotado por el circulo de friccion con Fx.

SUSPENSION (R6, `ParametrosRover.suspension`):
  * `simple` (DEFECTO; bytes identicos a R1/R4): la suspension rocker-bogie se representa por un resorte-amortiguador por rueda
    sobre puntos fijos del chasis (ver SIMPLIFICACIONES).
  * `rocker_bogie`: cinematica real de los tres grados de libertad pasivos. Por lado, el balancin gira sobre el chasis (pivote en
    x_pivote_balancin) con el angulo theta; el bogie cuelga del extremo trasero del balancin con el angulo phi (relativo al balancin)
    y lleva las ruedas media y trasera; la rueda delantera va en el extremo delantero del balancin. La BARRA DIFERENCIAL liga los
    dos balancines: theta_izq = +delta, theta_der = -delta (el chasis queda a la media de ambos). Estados extra: delta, phi_izq,
    phi_der (rad). Mecanismo sin masa: la fuerza y el par sobre el chasis son la suma de las fuerzas en los cubos (en sus posiciones
    reales) menos el par de los motores, y cada articulacion se mueve a la velocidad Q/c_art (sobreamortiguada, c_art N m s/rad) con
    Q = fuerza generalizada de los contactos (equilibrio de momentos en el pivote). Con esto la carga se reparte sola: con la
    geometria por defecto (balancin 2:1) cada rueda lleva ~1/6 del peso en llano. Ademas el contacto pasa a ser por ENVOLVENTE
    (`contacto='auto'`): se muestrea el perfil del terreno a lo largo del disco de la rueda y se toma el punto de mayor penetracion,
    con la normal del contacto hacia el cubo; asi una roca o un escalon se «ve» de lado y la rueda trepa por el borde, no solo por
    debajo del cubo.

SIMPLIFICACIONES (asumidas, ver docs/ROVER.md): la suspension rocker-bogie se representa por un resorte-amortiguador
por rueda sobre puntos fijos del chasis (equivale a un reparto de carga casi igual; no hay cinematica de balancin ni
colision del chasis con el suelo); el hundimiento de la rueda NO desplaza su centro; la rueda es una esfera para el
contacto; no hay giro de las ruedas delanteras (viraje por deslizamiento, skid steer); el limite del deslizamiento
lateral por tanh produce una deriva lenta en reposo sobre pendiente (regularizacion, VS).
"""
from dataclasses import dataclass, field, fields

import numpy as np

DT = 0.01            # paso fijo de integracion (s)
EPS_V = 0.3          # m/s: piso de la velocidad en el denominador del deslizamiento (regulariza el reposo)
VS = 0.1             # m/s: ancho de la regularizacion de la friccion lateral
V_ROD = 0.1          # m/s: ancho de la regularizacion de la resistencia a la rodadura
ANGULO_VUELCO_DEG = 70.0   # inclinacion del eje z del cuerpo respecto a la vertical que se da por vuelco


@dataclass
class ParametrosRover:
    masa: float = 150.0          # kg
    inercia: tuple = (10.0, 20.0, 26.0)   # kg m2 (caja 1.2 x 0.8 x 0.4 m)
    radio_rueda: float = 0.25    # m
    ancho_rueda: float = 0.20    # m
    inercia_rueda: float = 1.0   # kg m2 (rueda + rotor reflejado por la reduccion)
    x_ruedas: tuple = (0.5, 0.0, -0.5)   # m (delante, medio, atras) respecto al centro de masa
    y_ruedas: float = 0.40       # m (semivia)
    dz_cubo: float = 0.15        # m: el cubo de la rueda esta dz por debajo del centro de masa
    k_susp: float = 20000.0      # N/m por rueda
    c_susp: float = 990.0        # N s/m por rueda (~0.7 critico)
    rodadura_en_motor: bool = True  # R6/ronda 8: True (defecto) = el motor vence la resistencia por compactacion (fisica corregida); False = comportamiento historico R1/R4 (solo para reproducir cifras antiguas)
    kw: float = 20.0             # N m s/rad: ganancia del control de velocidad de rueda
    tau_max: float = 60.0        # N m por rueda
    cd_a: float = 0.8            # m2: Cd*A para la resistencia del aire
    eta: float = 0.7             # eficiencia mecanica->electrica (SUPUESTO)
    r_joule: float = 0.02        # W/(N m)^2: perdidas en el cobre (SUPUESTO)
    p_base_w: float = 20.0       # W de consumo fijo (computador, radio) (SUPUESTO)
    # --- R6: suspension rocker-bogie real (solo con suspension='rocker_bogie'; en 'simple' se ignoran)
    suspension: str = 'simple'   # 'simple' (defecto, R1) | 'rocker_bogie'
    contacto: str = 'auto'       # 'punto' | 'envolvente' | 'auto' (= punto con 'simple', envolvente con 'rocker_bogie')
    x_pivote_balancin: float = 0.0    # m respecto al CM (con 0.0, reparto 1/3 por rueda en llano)
    z_pivote_balancin: float = 0.05   # m respecto al CM
    x_pivote_bogie: float = -0.25     # m respecto al CM (entre la rueda media y la trasera)
    z_pivote_bogie: float = -0.05     # m respecto al CM
    lim_balancin_deg: float = 30.0    # tope del angulo del balancin (SUPUESTO)
    lim_bogie_deg: float = 40.0       # tope del angulo del bogie (SUPUESTO)
    k_tope: float = 20000.0           # N m/rad de los topes de las articulaciones
    c_art: float = 250.0              # N m s/rad: amortiguamiento de las articulaciones (sobreamortiguadas)
    n_muestras: int = 13              # puntos del perfil del disco para el contacto por envolvente
    ancho_via: float = field(init=False)

    def __post_init__(self):
        self.ancho_via = 2 * self.y_ruedas
        if self.suspension not in ('simple', 'rocker_bogie'):
            raise ValueError(f'suspension desconocida: {self.suspension!r} (hay: simple, rocker_bogie)')
        if self.contacto not in ('punto', 'envolvente', 'auto'):
            raise ValueError(f'contacto desconocido: {self.contacto!r} (hay: punto, envolvente, auto)')

    @classmethod
    def desde_dict(cls, d=None):
        d = dict(d or {})
        validos = {f.name for f in fields(cls) if f.init}
        malos = set(d) - validos
        if malos:
            raise ValueError(f'parametros de rover desconocidos: {sorted(malos)}')
        for k in ('inercia', 'x_ruedas'):
            if k in d:
                d[k] = tuple(d[k])
        return cls(**d)


def _quat_mat(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _quat_mul(a, b):
    return np.array([a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
                     a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
                     a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
                     a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]])


def _cross(a, b):
    return np.stack([a[..., 1] * b[..., 2] - a[..., 2] * b[..., 1],
                     a[..., 2] * b[..., 0] - a[..., 0] * b[..., 2],
                     a[..., 0] * b[..., 1] - a[..., 1] * b[..., 0]], axis=-1)


def _empuje(s, cAb, fn_tan, K, L):
    """T(s) de Wong para s en (0, 1]; serie para s pequeno (evita 0/0)."""
    x = s * L / K
    f = np.where(x < 1e-4, x / 2.0, 1.0 - (1.0 - np.exp(-x)) / np.where(x < 1e-4, 1.0, x))
    return (cAb + fn_tan) * f


class Rover:
    """Un rover sobre un `Terreno` en un `planeta` (dict de planetas.py)."""

    def __init__(self, planeta, terreno, params=None, x=0.0, y=0.0, rumbo_deg=0.0):
        self.pl = planeta
        self.ter = terreno
        if terreno.suelo_base is None:
            terreno.suelo_base = dict(planeta['suelo'])
        self.p = params if isinstance(params, ParametrosRover) else ParametrosRover.desde_dict(params)
        p = self.p
        self.g = float(planeta['g'])
        self.rho = float(planeta['densidad_atm'])
        self.hubs = np.array([[xx, sy * p.y_ruedas, -p.dz_cubo] for sy in (1.0, -1.0) for xx in p.x_ruedas])
        self.rb = p.suspension == 'rocker_bogie'
        self.envolvente = p.contacto == 'envolvente' or (p.contacto == 'auto' and self.rb)
        if self.rb:
            self._geom_rb()
        self.I = np.array(p.inercia, dtype=float)
        self.n_pasos = 0
        # actitud inicial alineada con la normal del terreno y rumbo dado
        h, n = terreno.altura_normal(np.array([x]), np.array([y]))
        n = n[0]
        yaw = np.radians(rumbo_deg)
        fx = np.array([np.cos(yaw), np.sin(yaw), 0.0])
        fx = fx - n * fx.dot(n)
        fx /= np.linalg.norm(fx)
        ly = _cross(n, fx)
        R = np.stack([fx, ly, n], axis=1)
        q = self._mat_quat(R)
        d0 = p.masa * self.g / (6.0 * p.k_susp)
        pos = np.array([x, y, h[0]]) + n * (p.radio_rueda + p.dz_cubo - d0)
        self.s = np.concatenate([pos, q, np.zeros(3), np.zeros(3), np.zeros(6)] + ([np.zeros(3)] if self.rb else []))
        self.margen_min = 90.0
        self.cmd = (0.0, 0.0)
        self.vuelco = False
        self._reset_acum()
        self._ultimo = None

    # ---- utilidades -----------------------------------------------------------------------------------------
    @staticmethod
    def _mat_quat(R):
        tr = np.trace(R)
        if tr > 0:
            s = 2.0 * np.sqrt(tr + 1.0)
            q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
        else:
            i = int(np.argmax(np.diag(R)))
            j, k = (i + 1) % 3, (i + 2) % 3
            s = 2.0 * np.sqrt(R[i, i] - R[j, j] - R[k, k] + 1.0)
            q = [0.0] * 4
            q[0] = (R[k, j] - R[j, k]) / s
            q[1 + i] = 0.25 * s
            q[1 + j] = (R[j, i] + R[i, j]) / s
            q[1 + k] = (R[k, i] + R[i, k]) / s
        q = np.array(q)
        return q / np.linalg.norm(q)

    def _reset_acum(self):
        self.energia_ws = 0.0
        self.distancia = 0.0
        self.suma_desl = 0.0
        self.n_desl = 0
        self.pend_max = 0.0
        self.atascos = 0
        self._t_atasco = 0.0
        self._en_atasco = False
        self._xy_prev = self.s[:2].copy()
        self.n_pasos = 0
        self.margen_min = 90.0

    @property
    def t(self):
        return self.n_pasos * DT

    def targets(self, v, w):
        """Velocidad angular objetivo de las 6 ruedas para (v m/s, w rad/s), viraje diferencial."""
        p = self.p
        vl = v - w * p.y_ruedas
        vr = v + w * p.y_ruedas
        return np.array([vl] * 3 + [vr] * 3) / p.radio_rueda

    # ---- cinematica rocker-bogie (R6) -----------------------------------------------------------------------
    def _geom_rb(self):
        """Brazos del mecanismo en reposo (sistema del cuerpo), a partir de las posiciones de reposo de los cubos."""
        p = self.p
        self._arm = []
        for lado, sy in enumerate((1.0, -1.0)):
            piv = np.array([p.x_pivote_balancin, sy * p.y_ruedas, p.z_pivote_balancin])
            bog = np.array([p.x_pivote_bogie, sy * p.y_ruedas, p.z_pivote_bogie])
            h = self.hubs[3 * lado:3 * lado + 3]
            self._arm.append((piv, h[0] - piv, bog - piv, h[1] - bog, h[2] - bog))

    @staticmethod
    def _ry(a):
        c, sn = np.cos(a), np.sin(a)
        return np.array([[c, 0.0, sn], [0.0, 1.0, 0.0], [-sn, 0.0, c]]), np.array([[-sn, 0.0, c], [0.0, 0.0, 0.0], [-c, 0.0, -sn]])

    def _hubs_rb(self, q):
        """q = [delta, phi_izq, phi_der] -> (hubs (6,3) en el cuerpo, J (6,3,3) con dhub/dq_k = J[:, :, k])."""
        hubs = np.zeros((6, 3))
        J = np.zeros((6, 3, 3))
        for lado, sg in enumerate((1.0, -1.0)):
            piv, af, ab, am, ar = self._arm[lado]
            th = sg * q[0]
            ph = q[1 + lado]
            R1, D1 = self._ry(th)
            R2, D2 = self._ry(th + ph)
            b = piv + R1 @ ab
            i = 3 * lado
            hubs[i] = piv + R1 @ af
            hubs[i + 1] = b + R2 @ am
            hubs[i + 2] = b + R2 @ ar
            dth_f = D1 @ af
            dth_b = D1 @ ab
            J[i, :, 0] = sg * dth_f
            J[i + 1, :, 0] = sg * (dth_b + D2 @ am)
            J[i + 2, :, 0] = sg * (dth_b + D2 @ ar)
            J[i + 1, :, 1 + lado] = D2 @ am
            J[i + 2, :, 1 + lado] = D2 @ ar
        return hubs, J

    def _contacto_env(self, hub, R):
        """Contacto por envolvente: -> (pen, n (6,3), cxy (6,2)). Sobre la normal puntual (la de siempre) solo se impone el punto del perfil
        del disco si penetra mas de 1 cm que ella (evita el ruido de muestreo en terreno liso)."""
        p = self.p
        Rw = p.radio_rueda
        h, n = self.ter.altura_normal(hub[:, 0], hub[:, 1])
        d = np.einsum('ij,ij->i', n, hub - np.column_stack([hub[:, 0], hub[:, 1], h]))
        pen = Rw - d
        bx = R[:, 0]
        fh = np.array([bx[0], bx[1]])
        fh = fh / max(np.linalg.norm(fh), 1e-9)
        dl = np.linspace(-Rw, Rw, p.n_muestras)
        px = hub[:, 0][None, :] + dl[:, None] * fh[0]
        py = hub[:, 1][None, :] + dl[:, None] * fh[1]
        pz = self.ter.altura(px, py)
        vec = np.stack([hub[:, 0][None, :] - px, hub[:, 1][None, :] - py, hub[:, 2][None, :] - pz], axis=-1)
        dist = np.linalg.norm(vec, axis=-1)
        k = np.argmax(Rw - dist, axis=0)
        w = np.arange(6)
        pen_e = Rw - dist[k, w]
        usar = pen_e > pen + 0.01
        ne = vec[k, w] / np.maximum(dist[k, w], 1e-9)[:, None]
        n2 = np.where(usar[:, None], ne, n)
        pen2 = np.where(usar, pen_e, pen)
        cxy = np.where(usar[:, None], np.column_stack([px[k, w], py[k, w]]), hub[:, :2])
        return pen2, n2, cxy

    def margen_vuelco(self):
        """Margen de estabilidad al vuelco (grados): angulo que habria que inclinar el rover alrededor del borde mas
        comprometido del poligono de apoyo (casco convexo de los puntos de contacto) hasta que el CM quede sobre ese borde.
        90 si no hay un poligono (menos de 3 contactos no colineales); negativo si el CM ya esta fuera del poligono."""
        u = self._u1 if getattr(self, '_u1', None) is not None else self._ultimo
        pc = u.get('pc')
        if pc is None:
            return None
        pts = pc[u['fn'] > 1.0]
        if len(pts) < 3:
            return 0.0
        P = sorted(map(tuple, pts[:, :2]))

        def cruz(o, a, b):
            return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
        bajo = []
        for q in P:
            while len(bajo) >= 2 and cruz(bajo[-2], bajo[-1], q) <= 0:
                bajo.pop()
            bajo.append(q)
        alto = []
        for q in reversed(P):
            while len(alto) >= 2 and cruz(alto[-2], alto[-1], q) <= 0:
                alto.pop()
            alto.append(q)
        casco = bajo[:-1] + alto[:-1]
        if len(casco) < 3:
            return 0.0
        idx = [int(np.argmin(np.linalg.norm(pts[:, :2] - np.array(q), axis=1))) for q in casco]
        cm = self.s[:3]
        cen = pts.mean(axis=0)
        mejor = 90.0
        for a, b in zip(idx, idx[1:] + idx[:1]):
            A, B = pts[a], pts[b]
            eje = B - A
            ln = np.linalg.norm(eje)
            if ln < 1e-6:
                continue
            eje = eje / ln
            up = np.array([0.0, 0.0, 1.0])
            e1 = up - eje * (up @ eje)
            e1 /= np.linalg.norm(e1)
            e2 = np.cross(eje, e1)
            out = np.array([B[1] - A[1], -(B[0] - A[0]), 0.0])         # normal horizontal (derecha del borde)
            if out @ (cen - A) > 0:
                out = -out                                            # que apunte hacia fuera del poligono
            r = cm - A
            a1 = r @ e1
            a2o = (r @ e2) * np.sign(e2 @ out)
            mejor = min(mejor, float(np.degrees(np.arctan2(-a2o, a1))))
        return mejor

    # ---- dinamica -------------------------------------------------------------------------------------------
    def _fuerzas(self, s, om_t):
        p = self.p
        pos, q, v, wb, om = s[0:3], s[3:7], s[7:10], s[10:13], s[13:19]
        q = q / np.linalg.norm(q)
        R = _quat_mat(q)
        w_w = R @ wb
        if self.rb:
            hubs_b, J = self._hubs_rb(s[19:22])
            rh = hubs_b @ R.T
        else:
            rh = self.hubs @ R.T                   # (6,3) mundo, desde el CM
        hub = pos + rh
        if self.envolvente:
            pen, n, cxy = self._contacto_env(hub, R)
        else:
            h, n = self.ter.altura_normal(hub[:, 0], hub[:, 1])
            d = np.einsum('ij,ij->i', n, hub - np.column_stack([hub[:, 0], hub[:, 1], h]))
            pen = p.radio_rueda - d
        vh = v + _cross(w_w, rh)
        vn = np.einsum('ij,ij->i', n, vh)
        cont = pen > 0.0
        fn = np.where(cont, np.maximum(0.0, p.k_susp * pen - p.c_susp * vn), 0.0)
        bx = R[:, 0]
        f = bx[None, :] - np.einsum('j,ij->i', bx, n)[:, None] * n
        f /= np.linalg.norm(f, axis=1, keepdims=True)
        lat = _cross(n, f)
        vx = np.einsum('ij,ij->i', vh, f)
        vy = np.einsum('ij,ij->i', vh, lat)
        so = self.ter.suelo_arr(cxy[:, 0], cxy[:, 1]) if self.envolvente else self.ter.suelo_arr(hub[:, 0], hub[:, 1])
        b, D, Rw = p.ancho_rueda, 2 * p.radio_rueda, p.radio_rueda
        kp = so['kc'] / b + so['kphi']
        nn = so['n']
        fn_s = np.maximum(fn, 1e-6)
        z = (fn_s / (b * np.sqrt(D) * kp)) ** (1.0 / (nn + 0.5))
        L = np.sqrt(D * z)
        tanphi = np.tan(np.radians(so['phi_deg']))
        cAb = so['c'] * b * L
        fmax = cAb + fn * tanphi
        uw = om * Rw
        den = np.maximum(np.maximum(np.abs(uw), np.abs(vx)), EPS_V)
        sl = np.clip((uw - vx) / den, -1.0, 1.0)
        T = _empuje(np.abs(sl), cAb, fn * tanphi, so['k_deslizamiento'], L)
        Rc = b * kp * z ** (nn + 1.0) / (nn + 1.0)
        fx = np.sign(sl) * T - Rc * np.tanh(vx / V_ROD)
        fx = np.clip(fx, -fmax, fmax)
        fy_lim = np.sqrt(np.maximum(fmax ** 2 - fx ** 2, 0.0))
        fy = -fy_lim * np.tanh(vy / VS)
        act = cont & (fn > 0.0)
        fx = np.where(act, fx, 0.0)
        if p.rodadura_en_motor:
            # R6 (hallazgo de R5): el motor debe vencer TAMBIEN la resistencia por compactacion. El empuje T mueve al cuerpo menos Rc, pero
            # el par de la rueda es (T + Rc) Rw: antes ese par usaba fx = T - Rc y Rc ayudaba a girar la rueda en vez de frenarla.
            fx_par = np.where(act, np.clip(np.sign(sl) * T, -fmax, fmax) + Rc * np.tanh(vx / V_ROD), 0.0)
        else:
            fx_par = fx
        fy = np.where(act, fy, 0.0)
        F = fn[:, None] * n + fx[:, None] * f + fy[:, None] * lat
        rc = rh - Rw * n
        tau = np.clip(p.kw * (om_t - om), -p.tau_max, p.tau_max)
        eje = R[:, 1]
        Ftot = F.sum(axis=0) + np.array([0.0, 0.0, -p.masa * self.g])
        if self.rho > 0:
            Ftot = Ftot - 0.5 * self.rho * p.cd_a * np.linalg.norm(v) * v
        Ttot = _cross(rc, F).sum(axis=0) - np.sum(tau - fx_par * Rw) * eje
        self._ultimo = dict(fx=fx, fy=fy, fn=fn, sl=np.where(act, sl, 0.0), tau=tau, om=om, z=z, n_c=n[:, 2].mean())
        if self.rb:
            # fuerza generalizada de cada articulacion: F_i . dhub_i/dq (en el cuerpo) menos el par de reaccion de los motores
            Fb = F @ R                               # F en el cuerpo (R.T @ F por fila)
            Q = np.einsum('ij,ijk->k', Fb, J)
            Q = Q - np.array([tau[0] - tau[3] + (tau[1] + tau[2]) - (tau[4] + tau[5]), tau[1] + tau[2], tau[4] + tau[5]])
            q = s[19:22]
            lb, lg = np.radians(p.lim_balancin_deg), np.radians(p.lim_bogie_deg)
            Q = Q - p.k_tope * np.array([max(q[0] - lb, 0.0) + min(q[0] + lb, 0.0),
                                         max(q[1] - lg, 0.0) + min(q[1] + lg, 0.0),
                                         max(q[2] - lg, 0.0) + min(q[2] + lg, 0.0)])
            self._ultimo['dq'] = Q / p.c_art
            self._ultimo['pc'] = hub - p.radio_rueda * n
            nm = (n * (fn > 1.0)[:, None]).sum(axis=0)
            self._ultimo['pend'] = float(np.degrees(np.arccos(np.clip(nm[2] / max(np.linalg.norm(nm), 1e-9), -1, 1)))) if nm.any() else 0.0
        return Ftot, Ttot, R, tau, fx_par

    def _deriv(self, s, om_t):
        p = self.p
        Ftot, Ttot, R, tau, fx = self._fuerzas(s, om_t)
        q = s[3:7]
        wb = s[10:13]
        dq = 0.5 * _quat_mul(q, np.concatenate([[0.0], wb]))
        dwb = (R.T @ Ttot - _cross(wb, self.I * wb)) / self.I
        dom = (tau - fx * p.radio_rueda - 0.02 * s[13:19]) / p.inercia_rueda
        if self.rb:
            return np.concatenate([s[7:10], dq, Ftot / p.masa, dwb, dom, self._ultimo['dq']])
        return np.concatenate([s[7:10], dq, Ftot / p.masa, dwb, dom])

    def paso(self, v_cmd=None, w_cmd=None):
        """Avanza DT con el mando (v m/s, w rad/s); sin argumentos repite el ultimo."""
        if v_cmd is not None:
            self.cmd = (float(v_cmd), float(w_cmd or 0.0))
        if self.vuelco:
            self.n_pasos += 1
            return
        om_t = self.targets(*self.cmd)
        s = self.s
        k1 = self._deriv(s, om_t)
        self._u1 = self._ultimo     # contacto al inicio del paso: base de las metricas y del estado publicado
        k2 = self._deriv(s + 0.5 * DT * k1, om_t)
        k3 = self._deriv(s + 0.5 * DT * k2, om_t)
        k4 = self._deriv(s + DT * k3, om_t)
        self.s = s + DT / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
        self.s[3:7] /= np.linalg.norm(self.s[3:7])
        self.n_pasos += 1
        self._acumular()

    def _acumular(self):
        p = self.p
        u = self._u1
        tau, om = u['tau'], u['om']
        pw = p.p_base_w + float(np.sum(np.maximum(0.0, tau * om) / p.eta + p.r_joule * tau ** 2))
        self.potencia_w = pw
        self.energia_ws += pw * DT
        R = _quat_mat(self.s[3:7])
        if np.degrees(np.arccos(np.clip(R[2, 2], -1, 1))) > ANGULO_VUELCO_DEG:
            self.vuelco = True
            self.s[7:13] = 0.0
        if self.n_pasos % 10 == 0:
            xy = self.s[:2]
            self.distancia += float(np.linalg.norm(xy - self._xy_prev))
            self._xy_prev = xy.copy()
            sl = self._u1['sl']
            self.suma_desl += float(np.mean(np.abs(sl)))
            self.n_desl += 1
            self.pend_max = max(self.pend_max, self._u1['pend'] if self.rb else float(self.ter.pendiente_deg(self.s[0], self.s[1])))
            if self.rb:
                m = self.margen_vuelco()
                if m is not None:
                    self.margen_min = min(self.margen_min, m)
            vcmd = abs(self.cmd[0])
            vel = float(R[:, 0] @ self.s[7:10])
            if vcmd > 0.1 and abs(vel) < 0.2 * vcmd:
                self._t_atasco += 10 * DT
            else:
                self._t_atasco = 0.0
                self._en_atasco = False
            if self._t_atasco >= 5.0 and not self._en_atasco:
                self._en_atasco = True
                self.atascos += 1

    @property
    def atascado(self):
        return self._en_atasco

    def asentar(self, dur_s=2.0):
        """Deja caer el rover sobre sus resortes sin mando y reinicia reloj y acumuladores (t = 0 despues)."""
        for _ in range(int(round(dur_s / DT))):
            self.paso(0.0, 0.0)
        self.s[7:13] = 0.0
        self.s[13:19] = 0.0
        self.vuelco = False
        self._reset_acum()

    # ---- salida ---------------------------------------------------------------------------------------------
    def estado(self):
        """El dict `estado` del CONTRATO sec. 18 (floats redondeados a 6 decimales)."""
        s = self.s
        R = _quat_mat(s[3:7])
        if self._ultimo is None:
            self._fuerzas(s, self.targets(*self.cmd))
        u = self._u1 if getattr(self, '_u1', None) is not None else self._ultimo
        r = lambda a: [round(float(x), 6) for x in a]
        rum = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
        cab = np.degrees(np.arcsin(np.clip(R[2, 0], -1, 1)))
        ala = np.degrees(np.arctan2(R[2, 1], R[2, 2]))
        e = {'t': round(self.t, 6), 'x': round(float(s[0]), 6), 'y': round(float(s[1]), 6),
             'z': round(float(s[2]), 6), 'rumbo_deg': round(float(rum), 6), 'cabeceo_deg': round(float(cab), 6),
             'alabeo_deg': round(float(ala), 6), 'v': round(float(R[:, 0] @ s[7:10]), 6),
             'w': round(float(s[12]), 6),
             'pendiente_deg': round(float(u['pend'] if self.rb else self.ter.pendiente_deg(s[0], s[1])), 6),
             'deslizamiento': r(u['sl']), 'par_nm': r(u['tau']),
             'potencia_w': round(float(getattr(self, 'potencia_w', self.p.p_base_w)), 6),
             'energia_wh': round(self.energia_ws / 3600.0, 6), 'atascado': bool(self._en_atasco)}
        if self.rb:      # campos opcionales (CONTRATO sec. 18): solo con suspension rocker_bogie; en 'simple' el estado no cambia
            q = s[19:22]
            e['balancin_deg'] = r(np.degrees([q[0], -q[0]]))
            e['bogie_deg'] = r(np.degrees(q[1:3]))
            e['carga_n'] = r(u['fn'])
            m = self.margen_vuelco()
            if m is not None:
                e['margen_vuelco_deg'] = round(float(m), 6)
        return e

    def energia_mecanica(self):
        """Energia cinetica + potencial del chasis y las ruedas (J), para la prueba de caida libre."""
        p = self.p
        return (0.5 * p.masa * float(self.s[7:10] @ self.s[7:10]) + p.masa * self.g * float(self.s[2])
                + 0.5 * float(np.sum(self.I * self.s[10:13] ** 2)))

    def metricas(self, meta_alcanzada=False, semilla=None):
        m = self._metricas_base(meta_alcanzada, semilla)
        if self.rb:      # claves opcionales: solo con rocker_bogie
            m['suspension'] = 'rocker_bogie'
            m['margen_vuelco_min_deg'] = round(self.margen_min, 6)
        return m

    def _metricas_base(self, meta_alcanzada=False, semilla=None):
        return {'distancia_m': round(self.distancia, 6), 'tiempo_s': round(self.t, 6),
                'energia_wh': round(self.energia_ws / 3600.0, 6),
                'deslizamiento_medio': round(self.suma_desl / max(self.n_desl, 1), 6),
                'pendiente_max_deg': round(self.pend_max, 6), 'atascos': int(self.atascos),
                'vuelcos': int(self.vuelco), 'meta_alcanzada': bool(meta_alcanzada),
                'planeta': self.pl.get('clave', self.pl.get('nombre_es', '')), 'terreno': self.ter.tipo,
                'semilla': int(self.ter.semilla if semilla is None else semilla)}
