"""Fisica de la montura (sin ROS): J theta'' + b theta' = u + perturbaciones.

Modulo puro (solo numpy) que comparten el nodo `mount_sim` y las campanas
Monte Carlo de `control.py`: una sola implementacion de la fisica, vale para
los dos ejes (arreglos de forma (2,)) y para n pases a la vez ((2, n)).

Capas OPCIONALES de realismo; con los valores por defecto (todas apagadas) el
paso es EXACTAMENTE el de siempre (`alpha = (u + par_ext - b w)/J`,
semi-implicito), operacion por operacion:

* friccion de Coulomb + Stribeck (con adherencia, tipo Karnopp),
* backlash del reductor: dos cuerpos (motor y carga) con holgura entre ellos,
* desbalance gravitatorio en elevacion: par -m g d cos(el),
* cuantizacion del encoder (bits sobre 360 grados),
* viento como par con ESPECTRO (Dryden o Davenport), con semilla.

Cada modelo dice abajo bajo que condicion vale. Todo par/inercia esta en el
EJE DE CARGA (inercia reflejada), como en `model.py`.
"""
import math

import numpy as np


# ---------------------------------------------------------------- friccion

def friccion_stribeck(w, coulomb_nm, stribeck_nm, v_stribeck, signo=None):
    """Par de friccion (N m) a velocidad `w` (rad/s): curva de Stribeck
    F(w) = [Fc + Fe exp(-(w/vs)^2)] sgn(w), con Fe = Fs - Fc el exceso de la
    friccion estatica sobre la de Coulomb. Para |w| grande vale Fc; en
    w -> 0 vale Fs = Fc + Fe. `signo` fuerza el signo (se usa a velocidad
    casi nula, donde sgn(w) no esta definido).

    Modelo clasico (Armstrong-Helouvry, Dupont y Canudas de Wit 1994,
    "A survey of models, analysis tools and compensation methods for the
    control of machines with friction", Automatica 30(7)). Los parametros
    de la montura (Fc, Fe, vs) NO estan medidos: son un supuesto.
    """
    w = np.asarray(w, dtype=float)
    s = np.sign(w) if signo is None else signo
    return (coulomb_nm + stribeck_nm * np.exp(-(w / v_stribeck) ** 2)) * s


def _karnopp(w, neto, J, coulomb, exceso, vs, veps, h):
    """Un paso de velocidad de un cuerpo con friccion de Coulomb + Stribeck
    por el metodo de Karnopp: si |w| < veps (banda de velocidad cero) y el
    par neto (sin friccion) no supera la estatica Fs = Fc + Fe, el cuerpo se
    queda pegado (w = 0); si la supera, arranca. Evita el temblor numerico
    de sgn(w) a paso fijo. Devuelve (w_nuevo, aceleracion efectiva)."""
    lento = np.abs(w) < veps
    fs = coulomb + exceso
    pega = lento & (np.abs(neto) <= fs)
    signo = np.where(lento, np.sign(neto), np.sign(w))
    fric = friccion_stribeck(w, coulomb, exceso, vs, signo)
    w_n = np.where(pega, 0.0, w + (neto - fric) / J * h)
    return w_n, (w_n - w) / h


# ---------------------------------------------------------------- viento

def davenport_psd(f_hz, v10_m_s=10.0, kappa=0.005):
    """Espectro de Davenport de la velocidad del viento (m^2/s^2 por Hz):
    S(f) = 4 kappa V10^2 x^2 / (f (1 + x^2)^(4/3)), x = 1200 f / V10.
    Davenport (1961), "The spectrum of horizontal gustiness near the ground
    in high winds", Q.J.R. Meteorol. Soc. 87. `kappa` es el coeficiente de
    arrastre de la superficie (0.005 terreno abierto; Simiu y Scanlan,
    "Wind effects on structures"). La varianza total es 6 kappa V10^2."""
    f = np.asarray(f_hz, dtype=float)
    x = 1200.0 * f / float(v10_m_s)
    return 4.0 * kappa * v10_m_s ** 2 * x ** 2 / (f * (1.0 + x ** 2) ** (4.0 / 3.0))


def dryden_psd(f_hz, sigma=1.0, tau_s=5.0):
    """Espectro de Dryden longitudinal (unilateral, por Hz) de un proceso de
    Gauss-Markov de primer orden: S(f) = 4 sigma^2 tau / (1 + (2 pi f tau)^2),
    con tau = L/V (escala de turbulencia sobre velocidad media).
    MIL-HDBK-1797 / MIL-F-8785C. Integra a sigma^2."""
    f = np.asarray(f_hz, dtype=float)
    return 4.0 * sigma ** 2 * tau_s / (1.0 + (2.0 * np.pi * f * tau_s) ** 2)


class VientoTurbulento:
    """Par de viento aleatorio con espectro (N m), uno por eje (y por pase).

    Supuesto declarado: el par es PROPORCIONAL a la fluctuacion de velocidad
    (linealizando 1/2 rho Cd A r (V + u')^2 en torno a V, el coeficiente
    es rho Cd A r V) y se normaliza para que su RMS sea `sigma_nm`; solo la
    FORMA espectral viene de la fuente. La amplitud real de una rafaga en un
    plato de 3 m es de decenas de N m (`par_viento` de atp.py, 78 N m a
    10 m/s), muy por encima del J = 2 kg m^2 didactico de la montura: por eso
    `sigma_nm` es un parametro y no se deduce de la velocidad.

    modelo 'dryden': AR(1) exacto, x_{k+1} = phi x_k + sigma sqrt(1-phi^2) w,
        phi = exp(-dt/tau). Costo O(1) por paso. Valido para dt cualquiera.
    modelo 'davenport': suma de `n_armonicos` senoides con fases al azar y
        amplitudes sqrt(2 S(f) df) normalizadas, f en [f_min, f_max] (log).
        Valido en esa banda: fuera de ella el espectro se trunca.

    El par se mantiene constante entre llamadas (retencion de orden cero a
    la cadencia del reloj, 20 Hz): es lento frente al lazo (tau >= 1 s).
    """

    def __init__(self, sigma_nm, modelo='dryden', tau_s=5.0, v10_m_s=10.0,
                 kappa=0.005, forma=(2,), semilla=11, n_armonicos=48,
                 f_min_hz=0.002, f_max_hz=2.0):
        self.sigma = float(sigma_nm)
        self.modelo = str(modelo)
        self.tau = float(tau_s)
        self.rng = np.random.default_rng(int(semilla))
        self.forma = tuple(forma)
        self.x = self.rng.normal(0.0, self.sigma, self.forma)   # estado estacionario
        if self.modelo == 'davenport':
            f = np.logspace(np.log10(f_min_hz), np.log10(f_max_hz), int(n_armonicos))
            df = np.gradient(f)
            s = davenport_psd(f, v10_m_s, kappa) * df
            self.f = f
            self.amp = self.sigma * np.sqrt(2.0 * s / s.sum())
            self.fase = self.rng.uniform(0.0, 2.0 * np.pi, (len(f),) + self.forma)
        elif self.modelo != 'dryden':
            raise ValueError(f'modelo de viento desconocido: {modelo}')

    def par(self, t_s, dt_s):
        """Par (N m) vigente tras avanzar `dt_s` hasta el instante `t_s`."""
        if self.sigma <= 0.0:
            return np.zeros(self.forma)
        if self.modelo == 'dryden':
            phi = math.exp(-float(dt_s) / self.tau)
            self.x = phi * self.x + self.sigma * math.sqrt(1.0 - phi * phi) \
                * self.rng.normal(0.0, 1.0, self.forma)
            return self.x
        arg = 2.0 * np.pi * self.f.reshape((-1,) + (1,) * len(self.forma)) * t_s + self.fase
        return np.tensordot(self.amp, np.cos(arg), axes=(0, 0))


# ---------------------------------------------------------------- planta

def cuantizar(theta_rad, bits):
    """Cuantiza un angulo (rad) a un encoder de `bits` bits sobre 360 grados
    (mid-tread: redondeo al paso mas cercano). 14 bits: 0.022 grados."""
    q = 2.0 * math.pi / float(2 ** int(bits))
    return q * np.round(np.asarray(theta_rad) / q)


class PlantaMontura:
    """Dos ejes (o 2 x n pases) de J theta'' + b theta' = u + perturbacion.

    `theta`, `omega`: angulo y velocidad de la CARGA (rad, rad/s). Con
    `backlash_rad > 0` aparecen ademas `theta_m`, `omega_m` (motor) y
    `lado` (0 libre, +1/-1 contacto).

    Condiciones de validez de las capas opcionales:
    * friccion: Karnopp con banda `v_eps`; exige h * Fc / J << v_eps (con
      h = 2 ms, Fc < 1 N m y J = 2 se cumple con v_eps = 1e-3 rad/s).
    * backlash: modelo hibrido de dos cuerpos (Nordin y Gutman 2002,
      "Controlling mechanical systems with backlash - a survey",
      Automatica 38). J se reparte `frac_motor` / (1 - frac_motor) entre
      motor y carga (igual b); el mando `u` actua en el motor; viento,
      gravedad y friccion actuan en la CARGA. El impacto al cerrar la
      holgura es de restitucion `restitucion` (0 = plastico). Se detecta el
      contacto una vez por subpaso `h`: exige h pequeno frente al tiempo
      de cruce de la holgura.
    * gravedad: par -mgd cos(el) en el eje 1 (elevacion), el 0 no lo siente.
    """

    def __init__(self, J, b, w_max=math.inf, h=0.002, coulomb_nm=0.0,
                 stribeck_nm=0.0, v_stribeck=0.01, v_eps=1e-3,
                 backlash_rad=0.0, frac_motor=0.5, restitucion=0.0,
                 mgd_nm=0.0):
        self.J, self.b = float(J), float(b)
        self.w_max = float(w_max)
        self.h = float(h)
        self.fc, self.fe = float(coulomb_nm), float(stribeck_nm)
        self.vs, self.veps = float(v_stribeck), float(v_eps)
        self.bl_rad = float(backlash_rad)
        self.fm = float(frac_motor)
        self.e = float(restitucion)
        self.mgd = float(mgd_nm)
        self.avanzada = bool(self.fc > 0 or self.fe > 0 or self.bl_rad > 0
                             or self.mgd != 0.0)
        self.theta = None
        self.omega = None
        self.theta_m = self.omega_m = self.lado = None

    def enganchar(self, theta0):
        self.theta = np.array(theta0, dtype=float)
        self.omega = np.zeros_like(self.theta)
        if self.bl_rad > 0.0:
            self.theta_m = self.theta.copy()
            self.omega_m = np.zeros_like(self.theta)
            self.lado = np.zeros(self.theta.shape)

    def angulo_encoder(self, lado_encoder='load'):
        """Angulo que mide el encoder: en la carga (lazo cerrado sobre lo que
        apunta) o en el motor (lo que mide un encoder detras del reductor)."""
        if lado_encoder == 'motor' and self.theta_m is not None:
            return self.theta_m
        return self.theta

    def separacion(self):
        """Holgura instantanea theta_m - theta (rad); None sin backlash."""
        if self.theta_m is None:
            return None
        return self.theta_m - self.theta

    # ------------------------------------------------------------ pasos
    def paso(self, u, par_ext, h):
        """Avanza `h` s con el mando `u` (N m, en el motor) y el par externo
        `par_ext` (escalar o arreglo, en la carga)."""
        if not self.avanzada:
            # camino original, operacion por operacion (resultados identicos)
            alpha = (u + par_ext - self.b * self.omega) / self.J
            self.omega = np.clip(self.omega + alpha * h, -self.w_max, self.w_max)
            self.theta += self.omega * h
            return
        ext = par_ext + self._gravedad()
        if self.bl_rad > 0.0:
            self._paso_backlash(u, ext, h)
        else:
            neto = u + ext - self.b * self.omega
            w, _ = _karnopp(self.omega, neto, self.J, self.fc, self.fe,
                            self.vs, self.veps, h)
            self.omega = np.clip(w, -self.w_max, self.w_max)
            self.theta += self.omega * h

    def _gravedad(self):
        g = np.zeros_like(self.theta)
        if self.mgd != 0.0:
            g[1] = -self.mgd * np.cos(self.theta[1])
        return g

    def _paso_backlash(self, u, ext, h):
        J, fm = self.J, self.fm
        Jm, Jl = J * fm, J * (1.0 - fm)
        bm, bl = self.b * fm, self.b * (1.0 - fm)
        dh = 0.5 * self.bl_rad
        wl, wm, lado = self.omega, self.omega_m, self.lado
        cont = lado != 0.0
        # cuerpo rigido (en contacto): inercia J, velocidad comun wl
        neto_r = u + ext - self.b * wl
        w_r, a_r = _karnopp(wl, neto_r, J, self.fc, self.fe, self.vs, self.veps, h)
        tau_c = u - bm * wl - Jm * a_r            # par motor -> carga
        sale = cont & (lado * tau_c < 0.0)
        # libre: el motor gira solo; la carga queda a su friccion y su par
        wm_f = wm + (u - bm * wm) / Jm * h
        wl_f, _ = _karnopp(wl, ext - bl * wl, Jl, self.fc, self.fe,
                           self.vs, self.veps, h)
        wm_n = np.clip(np.where(cont, w_r, wm_f), -self.w_max, self.w_max)
        wl_n = np.clip(np.where(cont, w_r, wl_f), -self.w_max, self.w_max)
        thm = self.theta_m + wm_n * h
        thl = self.theta + wl_n * h
        # en contacto el cuerpo es rigido: la carga queda a lado*holgura/2
        thl = np.where(cont, thm - lado * dh, thl)
        # cierre de holgura entre cuerpos libres
        d = thm - thl
        golpe = (~cont) & (np.abs(d) >= dh)
        if np.any(golpe):
            s = np.sign(d)
            exceso = np.where(golpe, d - s * dh, 0.0)
            thm = thm - exceso * Jl / J
            thl = thl + exceso * Jm / J
            vr = wm_n - wl_n
            cierra = golpe & (s * vr > 0.0)
            vc = (Jm * wm_n + Jl * wl_n) / J
            wm_i = vc - self.e * Jl / J * vr
            wl_i = vc + self.e * Jm / J * vr
            wm_n = np.where(cierra, wm_i, wm_n)
            wl_n = np.where(cierra, wl_i, wl_n)
            lado = np.where(golpe, s, lado)
        lado = np.where(sale, 0.0, lado)
        self.theta_m, self.theta = thm, thl
        self.omega_m, self.omega = wm_n, wl_n
        self.lado = lado
