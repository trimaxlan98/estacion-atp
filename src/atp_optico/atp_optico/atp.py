"""ATP de la estacion optica: montura gruesa (el error de seguimiento que MIDE la montura de
`atp_ros`) + espejo de direccion fina (FSM) con su lazo a 10 kHz, y el tilt atmosferico
(el jitter de llegada de la turbulencia) como perturbacion. Metrica: el error residual en
urad frente al ancho de haz de recepcion y la fraccion del pase con enlace cerrado.

ARQUITECTURA (la clasica de un terminal de lasercom de tierra; el lazo fino es el de `sat_isl`)

    d = (satelite - montura) + tilt de llegada + vibracion de la torre      [urad, por eje]
    e = d - y  (+ ruido del sensor de cuadrante)  -->  C(z)  -->  FSM P(z)  -->  y
    C(z) = Ki Ts/(1 - z^-1) (integrador),  P(s) = wn^2/(s^2 + 2 zeta wn s + wn^2) con retencion y retardo.

Se REUTILIZAN de `sat_isl.seguimiento`: el ruido del cuadrante (`ruido_qpd_urad`), la
conversion ZOH y las perturbaciones modales; y de `sat_isl.terminal.Lazo` los parametros del
lazo (10 kHz, FSM de 2 kHz, cruce a 400 Hz, +-1 mrad). Se ADAPTA la planta: aqui no hay
"descarga" del FSM a un cardan rapido (lazo de tipo 2 de sat_isl); la montura gruesa de la
estacion es la de `atp_ros` (programa de efemerides, ~0.5 Hz) y su error entra al FSM como
perturbacion lenta, asi que el lazo es de tipo 1 (S = 1/(1+L), L = G_f) y el FSM carga con
TODO el error de la montura: debe caber en su recorrido (`carrera`), si no el enganche se pierde.

EL TILT ATMOSFERICO tiene el rms de `atmosfera.tilt_llegada_rad` y un espectro de la forma
de Conan et al. (1995): plano bajo f_L = v_tilt/L0, f^(-2/3) hasta f_c = 0.3 v_tilt/D y f^(-17/3)
despues (ver `espectro_tilt`); se genera con un banco de filtros pasa bajos de 3.er orden de esquinas
logaritmicas cuyas varianzas salen de un ajuste no negativo a ese espectro (`BancoTilt`), de modo que el
rms y la forma se cumplen y la perturbacion sigue siendo estacionaria ruido-blanco-filtrado (lfilter
con estado continuo: la corrida es reproducible por semilla).
"""
import dataclasses
import math
from dataclasses import dataclass, field
from functools import lru_cache

import numpy as np
from scipy import signal
from scipy.optimize import nnls

from atp_optico import atmosfera as A
from atp_optico import enlace as L
from sat_isl import enlace as E
from sat_isl import seguimiento as S
from sat_isl import terminal as T

_trapz = getattr(np, 'trapezoid', None) or np.trapz


# ---------------------------------------------------------------- lazo fino (sin descarga)

class LazoFino:
    """Lazo del FSM sin cardan de descarga: L = G_f = Ki Ts/(1 - z^-1) z^-D P(z); Ki fija |L(f_c)| = 1.
    Misma discretizacion (ZOH) que `sat_isl.seguimiento.LazoLineal`; e = S d - T n, y = G_f (e + n)."""

    def __init__(self, lazo):
        self.lazo = lazo
        Ts = self.Ts = 1.0 / lazo.fs_hz
        wn = 2 * math.pi * lazo.fsm_fn_hz
        bp, ap = S._zoh([wn * wn], [1.0, 2 * lazo.fsm_zeta * wn, wn * wn], Ts)
        D = int(round(lazo.retardo_muestras))
        self.ag = np.convolve(np.array([1.0, -1.0]), ap)
        bg1 = np.concatenate([np.zeros(D), np.convolve(np.array([Ts]), bp)])
        wc = 2 * math.pi * lazo.f_cruce_hz
        self.Ki = 1.0 / abs(self._resp(bg1, self.ag, np.array([wc]))[0])
        self.bg = self.Ki * bg1
        den = S._suma(self.ag, self.bg)
        self.bS, self.aS = self.ag, den
        self.bT, self.aT = self.bg, den

    def _resp(self, b, a, w):
        z = np.exp(1j * np.asarray(w, float) * self.Ts)
        k = np.arange(max(len(b), len(a)))
        zi = z[:, None] ** (-k[None, :])
        return (zi[:, :len(b)] @ b) / (zi[:, :len(a)] @ a)

    def S(self, f_hz):
        return self._resp(self.bS, self.aS, 2 * math.pi * np.atleast_1d(f_hz))

    def T(self, f_hz):
        return self._resp(self.bT, self.aT, 2 * math.pi * np.atleast_1d(f_hz))

    def G(self, f_hz):
        return self._resp(self.bg, self.ag, 2 * math.pi * np.atleast_1d(f_hz))

    def estable(self):
        return bool(np.all(np.abs(np.roots(self.aS)) < 1.0))

    def margenes(self, n=40000):
        f = np.logspace(0.0, math.log10(0.499 * self.lazo.fs_hz), n)
        Lf = self.G(f)
        mag = np.abs(Lf)
        idx = np.where((mag[:-1] >= 1.0) & (mag[1:] < 1.0))[0]
        i = int(idx[-1])
        ph = np.unwrap(np.angle(Lf[i:]))
        ph = ph - 2 * math.pi * math.ceil(ph[0] / (2 * math.pi))
        out = {'f_cruce_hz': float(f[i]), 'pm_deg': float(180.0 + math.degrees(ph[0]))}
        k180 = np.where((ph[:-1] > -math.pi) & (ph[1:] <= -math.pi))[0]
        out['gm_db'] = float(-20 * math.log10(mag[i + int(k180[0])])) if len(k180) else None
        out['ms'] = float(np.max(np.abs(1.0 / (1.0 + Lf))))
        return out


@lru_cache(maxsize=8)
def lazo_fino(lazo):
    return LazoFino(lazo)


LAZO_FSM = T.Lazo(f_descarga_hz=0.0, nombre_grueso='montura atp_ros (sin descarga)')


# ---------------------------------------------------------------- espectro del tilt y su banco

def espectro_tilt(f, f_L, f_c):
    """Forma (sin normalizar) de la PSD del tilt de un eje: plano bajo f_L, f^(-2/3) entre f_L y f_c,
    f^(-17/3) sobre f_c (Conan, Rousset y Madec 1995, forma asintotica). Suave: (1+(f/f_L)^2)^(-1/3)
    (1+(f/f_c)^2)^(-5/2)."""
    f = np.asarray(f, float)
    return (1.0 + (f / f_L) ** 2) ** (-1.0 / 3.0) * (1.0 + (f / f_c) ** 2) ** (-2.5)


def _rejilla_log(fs_hz, n=120001):
    return np.logspace(-4.0, math.log10(0.5 * fs_hz), n)


class BancoTilt:
    """N filtros de 3.er orden (tres polos reales iguales, `y = p y + (1-p) x` en cascada) con esquinas
    f_i = f_min 10^(i/3) y varianza unitaria cada uno; `pesos(f_c, f_L)` da las varianzas no negativas
    (suman 1) que ajustan `espectro_tilt` normalizado."""
    F_MIN = 0.1
    N = 14

    def __init__(self, fs_hz):
        self.fs = fs_hz
        self.f_i = self.F_MIN * 10.0 ** (np.arange(self.N) / 3.0)
        self.b, self.a, self.gan = [], [], []
        wgrid = 2.0 * math.pi * _rejilla_log(fs_hz) / fs_hz
        for f in self.f_i:
            p = math.exp(-2.0 * math.pi * f / fs_hz)
            b = np.array([(1.0 - p) ** 3])
            a = np.array([1.0, -3.0 * p, 3.0 * p * p, -p ** 3])
            _, H = signal.freqz(b, a, worN=wgrid)
            var1 = float(_trapz(np.abs(H) ** 2, wgrid) / math.pi)
            self.b.append(b)
            self.a.append(a)
            self.gan.append(1.0 / math.sqrt(var1))
        self._fg = np.logspace(-2.0, math.log10(0.499 * fs_hz), 500)
        w = 2.0 * math.pi * self._fg / fs_hz
        # PSD unilateral de varianza unitaria de cada componente
        self._P = np.array([2.0 / fs_hz * g * g * np.abs(signal.freqz(b, a, worN=w)[1]) ** 2
                            for b, a, g in zip(self.b, self.a, self.gan)]).T
        self._cache = {}

    def pesos(self, f_c, f_L):
        clave = (round(math.log10(f_c) * 12.0), round(math.log10(f_L) * 12.0))
        if clave not in self._cache:
            fc, fl = 10.0 ** (clave[0] / 12.0), 10.0 ** (clave[1] / 12.0)
            s = espectro_tilt(self._fg, fl, fc)
            s = s / _trapz(s, self._fg)
            # error ponderado por f (varianza por decada): minimiza integral (P w - s)^2 f^2 d ln f
            Wt = self._fg[:, None] * self._P
            w, _ = nnls(Wt, self._fg * s)
            self._cache[clave] = w / w.sum()
        return self._cache[clave]

    def psd(self, f_hz, w):
        fi = np.atleast_1d(f_hz)
        om = 2.0 * math.pi * fi / self.fs
        tot = np.zeros(len(fi))
        for wi, b, a, g in zip(w, self.b, self.a, self.gan):
            tot += wi * 2.0 / self.fs * g * g * np.abs(signal.freqz(b, a, worN=om)[1]) ** 2
        return tot


@lru_cache(maxsize=4)
def banco_tilt(fs_hz):
    return BancoTilt(fs_hz)


class CanalTilt:
    """Un eje de tilt: estado continuo de los N filtros. `bloque(n, rng, w)` devuelve un bloque de
    n muestras de varianza unitaria total (suma de las componentes con varianzas `w`)."""

    def __init__(self, banco):
        self.banco = banco
        self.zi = [np.zeros(3) for _ in range(banco.N)]

    def bloque(self, n, rng, w):
        ruido = rng.standard_normal((self.banco.N, n))
        out = np.zeros(n)
        for i in range(self.banco.N):
            if w[i] <= 0.0:
                self.zi[i] = signal.lfilter(self.banco.b[i], self.banco.a[i], ruido[i], zi=self.zi[i])[1]
                continue
            x, self.zi[i] = signal.lfilter(self.banco.b[i], self.banco.a[i], ruido[i] * self.banco.gan[i],
                                           zi=self.zi[i])
            out += math.sqrt(w[i]) * x
        return out


# ---------------------------------------------------------------- montura cerrada sobre el sensor (analitico)

def sensibilidad_montura(f_hz, kp=25.0, ki=2.0, kd=8.0, tau=0.08, J=2.0, b=0.5):
    """|S| continuo de la montura de `atp_ros` si su PID se cerrara sobre el error OPTICO en vez de sobre el encoder
    (la alternativa sin FSM): L = C(s)/(J s^2 + b s), C = kp + ki/s + kd s/(1 + tau s) (ignora el muestreo de 20 Hz)."""
    s = 2j * math.pi * np.asarray(f_hz, float)
    C = kp + ki / s + kd * s / (1.0 + tau * s)
    return 1.0 / (1.0 + C / (J * s * s + b * s))


def residuo_montura_cerrada_urad(sigma_tilt_urad, f_c_hz, f_L_hz, fs_hz=10000.0):
    """rms (urad, un eje) del tilt atmosferico tras la montura cerrada sobre el sensor optico: sigma sqrt(int PSD |S_m|^2 df).
    Sirve de linea base 'sin FSM pero con camara': la montura (0.5 Hz) no sigue el tilt, que vive entre f_L y f_c."""
    b = banco_tilt(fs_hz)
    w = b.pesos(f_c_hz, f_L_hz)
    f = np.logspace(-2, math.log10(0.499 * fs_hz), 40000)
    return float(sigma_tilt_urad * math.sqrt(_trapz(b.psd(f, w) * np.abs(sensibilidad_montura(f)) ** 2, f)))


def residuo_fsm_analitico_urad(sigma_tilt_urad, f_c_hz, f_L_hz, nea_urad=0.0, lazo=None, fs_hz=10000.0):
    """rms (urad, un eje) del error del FSM por tilt + ruido del sensor, calculado SIN simular:
    sqrt(sigma^2 int PSD_tilt |S|^2 df + NEA^2 (2/fs) int |T|^2 df) (el sesgo y la vibracion no entran)."""
    lin = lazo_fino(lazo or LAZO_FSM)
    b = banco_tilt(fs_hz)
    w = b.pesos(f_c_hz, f_L_hz)
    f = np.logspace(-2, math.log10(0.499 * fs_hz), 40000)
    v = sigma_tilt_urad ** 2 * _trapz(b.psd(f, w) * np.abs(lin.S(f)) ** 2, f)
    v += nea_urad ** 2 * 2.0 / fs_hz * _trapz(np.abs(lin.T(f)) ** 2, f)
    return float(math.sqrt(v))


# ---------------------------------------------------------------- configuracion

@dataclass(frozen=True)
class ConfigAtp:
    paso_s: float = 0.05             # lockstep (CONTRATO 8)
    lazo: object = LAZO_FSM
    semilla: int = 7
    # sensor de seguimiento (QPD o camara de cuadrante): sobre la propia senal de bajada
    qpd_fraccion: float = 0.1        # fraccion de la potencia recibida que va al sensor. SUPUESTO
    qpd_w_urad: float = 50.0         # radio de la mancha en el sensor (>= el seeing lam/r0). SUPUESTO
    qpd_nep_w_rthz: float = 1.1e-13  # el de sat_isl.terminal (APD M = 10 + TIA). SUPUESTO
    # fuentes de error propias de la estacion
    vibracion_urad: float = 0.5      # torre/pilar por viento, rms por eje, modo de 6 Hz. SUPUESTO
    vibracion_hz: float = 6.0
    modelo_apunt_urad: float = 89.0   # residuo del modelo de apuntamiento tras calibrar con el Sol: 0.0072 deg rms en el cielo = 126 urad = 89 urad por eje; constante en el pase
    ncpa_urad: float = 0.5           # error de camino no comun sensor-detector, rms por eje (constante en el pase). SUPUESTO
    l0_m: float = 25.0               # escala externa (solo fija f_L del espectro del tilt)
    burn_s: float = 20.0             # calentamiento del banco de tilt antes del pase
    p_out: float = 1e-2              # outage admitido para elegir la tasa
    carrera_urad: float = 1000.0     # +- recorrido del FSM en espacio del cielo (el de sat_isl.terminal.Lazo)


def _modo_vibracion(fs_hz, f_hz, zeta=0.05):
    w = 2 * math.pi * f_hz
    b, a = S._zoh([w * w], [1.0, 2 * zeta * w, w * w], 1.0 / fs_hz)
    wgrid = 2.0 * math.pi * _rejilla_log(fs_hz, 60001) / fs_hz
    _, H = signal.freqz(b, a, worN=wgrid)
    return b, a, 1.0 / math.sqrt(float(_trapz(np.abs(H) ** 2, wgrid) / math.pi))


# ---------------------------------------------------------------- motor del ATP (un paso del lockstep)

class AtpOptico:
    """Estado del ATP optico entre un paso y el siguiente. `paso(...)` integra los 10 kHz del intervalo
    (t_prev, t] con el error de montura medido en t (interpolado linealmente desde el paso previo)
    y devuelve las cifras del paso. Los sorteos van siempre en el mismo orden: reproducible por semilla."""

    def __init__(self, cfg, sat, est, atm):
        self.cfg, self.sat, self.est, self.atm = cfg, sat, est, atm
        lz = cfg.lazo
        self.n = int(round(cfg.paso_s * lz.fs_hz))
        self.lin = lazo_fino(lz)
        self.banco = banco_tilt(lz.fs_hz)
        self.rng = np.random.default_rng(cfg.semilla)
        self.canales = [CanalTilt(self.banco) for _ in range(2)]
        self.vib = _modo_vibracion(lz.fs_hz, cfg.vibracion_hz)
        self.zi_vib = [np.zeros(2), np.zeros(2)]
        self.zi_S = [np.zeros(max(len(self.lin.bS), len(self.lin.aS)) - 1) for _ in range(2)]
        self.zi_T = [np.zeros_like(self.zi_S[0]) for _ in range(2)]
        self.zi_G = [np.zeros(max(len(self.lin.bg), len(self.lin.ag)) - 1) for _ in range(2)]
        self.ncpa = cfg.ncpa_urad * self.rng.choice([-1.0, 1.0], 2)      # magnitud fija, signo al azar
        self.modelo = cfg.modelo_apunt_urad * self.rng.standard_normal(2)   # el cielo no coincide con el encoder
        self.err_prev = None
        self.cielo_prev = None
        self.sigma_prev = est.sigma_apunt_urad        # error residual del paso anterior (para la potencia causal)
        # calentamiento del banco de tilt con espectro del primer paso (esquinas tipicas)
        self._calentar()
        self.k = 0

    def _calentar(self):
        w = self.banco.pesos(30.0, 1.0)
        n_blk = 2000
        for _ in range(int(self.cfg.burn_s * self.cfg.lazo.fs_hz / n_blk)):
            for ax in range(2):
                self.canales[ax].bloque(n_blk, self.rng, w)

    # ---- enlace de un paso
    def _enlace(self, el, rango_km, sigma_urad):
        """Potencia media, tasa elegida con outage <= p_out y outage a esa tasa (bajada), con el error
        residual `sigma_urad` (rms por eje)."""
        r = _evaluar_cache(self.sat, self.est, self.atm, round(el * 4.0) / 4.0, round(rango_km, 0),
                           _cuant_sigma(sigma_urad), self.cfg.p_out)
        return r

    def paso(self, t, az_deg, el_deg, rango_km, err_urad):
        """Un paso. `err_urad`: error de montura en el cielo (2 ejes: acimut*cos(el), elevacion) en t."""
        cfg, lz = self.cfg, self.cfg.lazo
        n = self.n
        err = np.asarray(err_urad, float)
        prev = err if self.err_prev is None else self.err_prev
        # ---- velocidad de barrido de la linea de vista (rad/s) a partir del paso anterior
        if self.cielo_prev is None:
            slew = 0.0
        else:
            daz = (az_deg - self.cielo_prev[0]) * math.cos(math.radians(el_deg))
            slew = math.radians(math.hypot(daz, el_deg - self.cielo_prev[1])) / cfg.paso_s
        self.cielo_prev = (az_deg, el_deg)
        # ---- perturbacion
        ramp = (np.arange(1, n + 1) / n)[None, :]
        d_mount = prev[:, None] + (err - prev)[:, None] * ramp
        sig_tilt = 1e6 * A.tilt_llegada_rad(self.atm, self.sat.lam_m, self.est.D_m, el_deg)
        v_t = A.viento_tilt(self.atm, el_deg, slew)
        f_c = 0.3 * v_t / self.est.D_m
        f_L = v_t / cfg.l0_m
        w = self.banco.pesos(f_c, f_L)
        d_tilt = np.empty((2, n))
        d_vib = np.empty((2, n))
        for ax in range(2):
            d_tilt[ax] = sig_tilt * self.canales[ax].bloque(n, self.rng, w)
            b, a, g = self.vib
            x, self.zi_vib[ax] = signal.lfilter(b, a, self.rng.standard_normal(n) * g * cfg.vibracion_urad,
                                                zi=self.zi_vib[ax])
            d_vib[ax] = x
        d = d_mount + d_tilt + d_vib + self.modelo[:, None]
        # ---- sensor de seguimiento: ruido por la potencia recibida (la del paso anterior: causal)
        p_rx = _evaluar_cache(self.sat, self.est, self.atm, round(el_deg * 4.0) / 4.0, round(rango_km, 0),
                              _cuant_sigma(self.sigma_prev), cfg.p_out)['p_rx_w']
        nea = S.ruido_qpd_urad(p_rx, self.sat.lam_m, lz.fs_hz, cfg.qpd_w_urad, cfg.qpd_fraccion,
                               cfg.qpd_nep_w_rthz)
        ruido = nea * self.rng.standard_normal((2, n))
        # ---- lazo
        e = np.empty((2, n))
        y = np.empty((2, n))
        for ax in range(2):
            eS, self.zi_S[ax] = signal.lfilter(self.lin.bS, self.lin.aS, d[ax], zi=self.zi_S[ax])
            eT, self.zi_T[ax] = signal.lfilter(self.lin.bT, self.lin.aT, ruido[ax], zi=self.zi_T[ax])
            e[ax] = eS - eT
            y[ax], self.zi_G[ax] = signal.lfilter(self.lin.bg, self.lin.ag, e[ax] + ruido[ax], zi=self.zi_G[ax])
        self.err_prev = err
        # ---- cifras del paso (urad): el error que ve el receptor es el del lazo mas el camino no comun
        e_fsm = e + self.ncpa[:, None]
        e_sin = d + self.ncpa[:, None]
        s_fsm = math.sqrt(float(np.mean(e_fsm ** 2)))            # rms por eje (promedio de los dos)
        s_sin = math.sqrt(float(np.mean(e_sin ** 2)))
        y_max = float(np.max(np.abs(y)))
        sujeto = y_max <= cfg.carrera_urad
        out = {'t': t, 'el': el_deg, 'rango_km': rango_km, 'slew_mrad_s': 1e3 * slew, 'nea_urad': nea,
               'sig_tilt_urad': sig_tilt, 'f_c_hz': f_c,
               'rms_montura_urad': math.sqrt(float(np.mean(d_mount ** 2))),
               'modelo_urad': float(np.hypot(*self.modelo)),
               'rms_tilt_urad': math.sqrt(float(np.mean(d_tilt ** 2))),
               'rms_d_urad': math.sqrt(float(np.mean(d ** 2))),
               'sigma_fsm_urad': s_fsm, 'sigma_sin_urad': s_sin, 'y_max_urad': y_max, 'enganchado': bool(sujeto),
               'e_fsm_sq': float(np.sum(e_fsm ** 2)), 'e_sin_sq': float(np.sum(e_sin ** 2)), 'n_muestras': int(2 * n)}
        # enlace: con FSM (si el FSM esta enganchado: si se satura no corrige nada)
        s_ef = s_fsm if sujeto else s_sin
        out['con_fsm'] = self._enlace(el_deg, rango_km, s_ef)
        out['sin_fsm'] = self._enlace(el_deg, rango_km, s_sin)
        self.sigma_prev = s_ef
        self.k += 1
        return out


def _cuant_sigma(s):
    """Cuantiza sigma (urad) en pasos de ~8 % (cache del enlace; el error es < 0.3 dB de acoplo)."""
    return math.exp(round(math.log(max(s, 0.05)) * 12.0) / 12.0)


@lru_cache(maxsize=200000)
def _evaluar_cache(sat, est, atm, el, rango_km, sigma, p_out):
    est2 = dataclasses.replace(est, sigma_apunt_urad=sigma)
    r = L.evaluar_enlace(sat, est2, atm, el, rango_km, sigma, p_out, con_margen=False)
    return {'p_rx_w': r['presupuesto']['p_rx_w'], 'p_rx_dbm': r['p_rx_dbm'], 'tasa_bps': r['tasa_bps'],
            'outage': r['outage'], 'sigma_I2': r['sigma_I2'], 'regimen': r['regimen'], 'util_bps': _util(sat, est, r)}


def _util(sat, est, r):
    if r['tasa_bps'] <= 0.0:
        return 0.0
    return est.modem.tasa_util(est.modem.tasas()[r['idx_tasa']], r['presupuesto']['p_rx_w'], sat.lam_m) \
        if est.modem.tipo == 'ppm' else est.modem.tasa_util(r['tasa_bps'])


# ---------------------------------------------------------------- medidor

class Medidor:
    """Acumula los pasos y entrega el RESUMEN (deterministico: todo en formato fijo)."""

    def __init__(self, cfg, est):
        self.cfg, self.est = cfg, est
        self.filas = []

    def muestra(self, r):
        self.filas.append(r)

    def resumen_dict(self):
        f = self.filas
        if not f:
            return {}
        dt = self.cfg.paso_s
        n = np.array([r['n_muestras'] for r in f], float)
        sq_f = np.array([r['e_fsm_sq'] for r in f])
        sq_s = np.array([r['e_sin_sq'] for r in f])
        sf = np.array([r['sigma_fsm_urad'] for r in f])
        ss = np.array([r['sigma_sin_urad'] for r in f])
        eng = np.array([r['enganchado'] for r in f])

        def modo(clave):
            out = np.array([r[clave]['outage'] for r in f])
            tasa = np.array([r[clave]['tasa_bps'] for r in f])
            util = np.array([r[clave]['util_bps'] for r in f])
            cerrado = tasa > 0.0
            if clave == 'con_fsm':
                cerrado = cerrado & eng
            gb = float(np.sum(util * (1.0 - out) * cerrado * dt)) / 8e9
            return {'cerrado_pct': 100.0 * float(np.mean(cerrado)), 'gb': gb,
                    'tasa_media_gbps': 1e-9 * float(np.mean(tasa * cerrado)), 'outage_medio': float(np.mean(out[cerrado])) if cerrado.any() else 1.0}
        theta = self.est.campo_urad
        return {
            'n_pasos': len(f), 'duracion_s': dt * len(f),
            'rms_montura_urad': float(np.sqrt(np.mean([r['rms_montura_urad'] ** 2 for r in f]))),
            'rms_tilt_urad': float(np.sqrt(np.mean([r['rms_tilt_urad'] ** 2 for r in f]))),
            'rms_con_fsm_urad': float(math.sqrt(np.sum(sq_f) / np.sum(n))),
            'rms_sin_fsm_urad': float(math.sqrt(np.sum(sq_s) / np.sum(n))),
            'p95_con_fsm_urad': float(np.percentile(sf, 95)),
            'p95_sin_fsm_urad': float(np.percentile(ss, 95)),
            'enganchado_pct': 100.0 * float(np.mean(eng)),
            'y_max_urad': float(max(r['y_max_urad'] for r in f)),
            'campo_rx_urad': theta,
            'rms_sobre_campo': float(math.sqrt(np.sum(sq_f) / np.sum(n))) / theta,
            'nea_max_urad': float(max(r['nea_urad'] for r in f)),
            'con_fsm': modo('con_fsm'), 'sin_fsm': modo('sin_fsm'),
        }

    def resumen(self):
        r = self.resumen_dict()
        if not r:
            return ['RESUMEN OPTICO: sin muestras']
        c, s = r['con_fsm'], r['sin_fsm']
        return [
            f"RESUMEN OPTICO n={r['n_pasos']} dur={r['duracion_s']:.1f} s | error de montura rms={r['rms_montura_urad']:.1f} urad, "
            f"tilt rms={r['rms_tilt_urad']:.2f} urad",
            f"ATP residual (por eje) con FSM: rms={r['rms_con_fsm_urad']:.2f} p95={r['p95_con_fsm_urad']:.2f} urad "
            f"(= {r['rms_sobre_campo']:.3f} del campo de {r['campo_rx_urad']:.0f} urad) | sin FSM: rms={r['rms_sin_fsm_urad']:.1f} "
            f"p95={r['p95_sin_fsm_urad']:.1f} urad | FSM enganchado {r['enganchado_pct']:.1f} % (|y| max {r['y_max_urad']:.0f} de {self.cfg.carrera_urad:.0f} urad)",
            f"ENLACE con FSM: cerrado {c['cerrado_pct']:.1f} % del pase, {c['gb']:.2f} GB, tasa media {c['tasa_media_gbps']:.2f} Gb/s | "
            f"sin FSM: cerrado {s['cerrado_pct']:.1f} %, {s['gb']:.2f} GB"]
