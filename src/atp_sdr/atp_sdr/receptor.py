"""Receptor de portadora a nivel IQ: adquisicion, seguimiento FLL/PLL, enganche y C/N0.

Solo numpy, sin ROS. Estructura de un receptor GNSS/SDR de portadora (Kaplan &
Hegarty, Understanding GPS/GNSS 3.a ed., cap. 5 y 8; Stephens & Thomas 1995):

1. `adquirir_fft`: FFT del bloque (con relleno de ceros) y busqueda del pico en
   la ventana de Doppler esperada; umbral CFAR por probabilidad de falsa alarma.
2. `SeguidorPortadora`: un NCO continuo mezcla la entrada, integra y descarga
   cada `t_int` (una "epoca"), y un lazo digital de 2.o o 3.er orden, con ayuda
   de un FLL durante la captura, corrige fase y frecuencia del NCO. La salida es
   la frecuencia del NCO: la ESTIMACION DEL DOPPLER del receptor.
3. Estimadores sobre las muestras post-integracion (`p`): C/N0 por el metodo de
   momentos M2M4 (Pauluzzi & Beaulieu 2000) y por NWPR (Van Dierendonck 1992),
   y el detector de enganche cos(2 phi).

Convenciones: fase y frecuencia en rad y Hz; el error de fase del discriminador
es positivo cuando la portadora VERDADERA va por delante del NCO.
"""
import cmath
import math

import numpy as np

ZETA = 1.0 / math.sqrt(2.0)
DOS_PI = 2.0 * math.pi


# ---------------------------------------------------------------- lazos digitales

def coef_lazo(orden, bn_hz):
    """Coeficientes (c0, c1, c2) y wn (rad/s) del filtro de lazo continuo
    F(s) = c2 + c1/s + c0/s^2 (c0 = 0 en 2.o orden) para un ancho de banda de
    ruido B_L = `bn_hz` (Hz).

    2.o orden, zeta = 0.707: wn = B_L / 0.53, c1 = wn^2, c2 = 2 zeta wn.
    3.er orden, a3 = 1.1, b3 = 2.4 (Ward; Kaplan & Hegarty, cap. 5): wn = B_L /
    0.7845, c0 = wn^3, c1 = a3 wn^2, c2 = b3 wn. El polinomio caracteristico del
    lazo cerrado es s^2 + c2 s + c1 (2.o) o s^3 + c2 s^2 + c1 s + c0 (3.o); la
    integral numerica de |H|^2 da B_L = 0.7838 wn (el 0.7845 de la fuente)."""
    if orden == 2:
        wn = 2.0 * bn_hz / (ZETA + 1.0 / (4.0 * ZETA))
        return 0.0, wn * wn, 2.0 * ZETA * wn, wn
    if orden == 3:
        wn = bn_hz / 0.7845
        return wn ** 3, 1.1 * wn * wn, 2.4 * wn, wn
    raise ValueError('el orden del lazo es 2 o 3')


def _reducir_pi(a):
    """Lleva un angulo a (-pi/2, pi/2]: discriminador de 2 cuadrantes (Costas)."""
    if a > 0.5 * math.pi:
        return a - math.pi
    if a <= -0.5 * math.pi:
        return a + math.pi
    return a


class SeguidorPortadora:
    """NCO + integracion y descarga + lazo FLL-asistido/PLL.

    Parametros:
      fs_hz        tasa de muestreo de la entrada
      t_int_s      tiempo de integracion = periodo del lazo (s). fs*t_int debe
                   ser entero. Condicion de estabilidad digital: B_L * t_int
                   <~ 0.2; la sinc de la integracion pierde 0.9 dB con un
                   error de frecuencia de 0.44/t_int
      orden        2 o 3 (el 3.o sigue rampas de frecuencia, el 2.o no)
      bn_hz        B_L del PLL (Hz)
      bn_fll_hz    ancho de banda del FLL de ayuda (Hz); su ganancia es 4 B_FLL
      disc         'pll' (atan2: portadora sin datos), 'costas' (atan: portadora
                   con modulacion binaria, ambiguedad de 180 grados) o 'cuadrado'
                   (la epoca se integra ELEVADA AL CUADRADO: quita la BPSK)
      modo         'pll' (PLL con FLL de ayuda) o 'fll' (solo FLL de 2.o orden,
                   no engancha la fase: lo usa la 1.a etapa de la cadena BPSK)
      asistencia   'adaptativa' (FLL activo hasta que el detector declara
                   enganche), 'siempre' o 'nunca'
      n_ventana    epocas de la ventana del detector de enganche y del C/N0

    El NCO mantiene fase continua; la frecuencia de la epoca k se calcula con
    la epoca k-1 (un retardo de computo)."""

    def __init__(self, fs_hz, t_int_s=1e-3, orden=3, bn_hz=50.0, bn_fll_hz=10.0,
                 disc='pll', modo='pll', asistencia='adaptativa', n_ventana=100):
        self.fs = float(fs_hz)
        self.t = float(t_int_s)
        self.n_ep = int(round(self.fs * self.t))
        if abs(self.n_ep - self.fs * self.t) > 1e-6:
            raise ValueError('fs * t_int debe ser entero')
        self.orden = int(orden)
        self.bn = float(bn_hz)
        self.disc = disc
        self.modo = modo
        self.asistencia = asistencia
        self.n_vent = int(n_ventana)
        self.c0, self.c1, self.c2, self.wn = coef_lazo(self.orden, self.bn)
        self.wf = 4.0 * float(bn_fll_hz)               # ganancia del FLL (1/s)
        # FLL de 2.o orden (modo 'fll'): s^2 + kf1 s + kf2
        wnf = 2.0 * float(bn_fll_hz) / (ZETA + 1.0 / (4.0 * ZETA))
        self.kf1, self.kf2 = 2.0 * ZETA * wnf, wnf * wnf
        self._n = np.arange(self.n_ep, dtype=float)
        self.reset()

    def reset(self, f_hz=0.0, fase=0.0):
        """Reinicia el NCO en `f_hz` y `fase` (rad) y borra los lazos."""
        self.fase = float(fase)
        self.w_cmd = DOS_PI * float(f_hz)      # rad/s
        self.i1 = self.w_cmd
        self.i2 = 0.0
        self.p_prev = None
        self.enganchado = False
        self.hist = np.zeros(0, dtype=complex)
        self.siguiendo = False

    def iniciar(self, f_hz, fase=0.0):
        """Arranca el seguimiento desde una estimacion (p. ej. la de la FFT)."""
        self.reset(f_hz, fase)
        self.siguiendo = True

    @property
    def f_hz(self):
        return self.w_cmd / DOS_PI

    def procesar(self, x, rotada=False):
        """Procesa `x` (len multiplo de fs*t_int). Devuelve un dict con arreglos
        por epoca: 'f_hz' (frecuencia del NCO en la epoca), 'fase_mid' (fase del
        NCO a mitad de epoca, rad), 'p' (muestra post-integracion; con disc
        'cuadrado', la integral del cuadrado), 'th' (error de fase del
        discriminador), y, si `rotada`, 'rot': la entrada mezclada con el NCO."""
        n = self.n_ep
        nep = len(x) // n
        xe = np.asarray(x)[:nep * n].reshape(nep, n)
        f_out = np.empty(nep)
        mid = np.empty(nep)
        p_out = np.empty(nep, dtype=complex)
        th_out = np.empty(nep)
        rot = np.empty((nep, n), dtype=complex) if rotada else None
        t, nn = self.t, self._n
        inv_n = 1.0 / n
        fs_rad = DOS_PI / self.fs
        cuad = self.disc == 'cuadrado'
        pll = self.modo == 'pll'
        atan2 = math.atan2
        fase, w_cmd, i1, i2 = self.fase, self.w_cmd, self.i1, self.i2
        c0, c1, c2, wf = self.c0, self.c1, self.c2, self.wf
        kf1, kf2 = self.kf1, self.kf2
        p_prev = self.p_prev
        orden3 = self.orden == 3
        r = np.empty(n, dtype=complex)
        cexp, cumprod = cmath.exp, np.cumprod
        for k in range(nep):
            w = w_cmd * (1.0 / self.fs)              # rad/muestra
            # NCO por recurrencia: r[m] = exp(-j fase) z^m con z = exp(-j w);
            # cumprod evita una exponencial compleja por muestra (3-4 veces mas
            # rapido); el error crece como m * eps ~ 1e-13 en 2000 muestras
            r[0] = cexp(-1j * fase)
            r[1:] = cexp(-1j * w)
            cumprod(r, out=r)
            y = xe[k] * r
            if rotada:
                rot[k] = y
            if cuad:
                p = complex(np.dot(y, y)) * inv_n
            else:
                p = complex(y.sum()) * inv_n
            f_out[k] = w_cmd / DOS_PI
            mid[k] = fase + w * 0.5 * (n - 1)
            p_out[k] = p
            # discriminadores
            if cuad:
                th = 0.5 * atan2(p.imag, p.real)
            elif self.disc == 'costas':
                th = _reducir_pi(atan2(p.imag, p.real))
            else:
                th = atan2(p.imag, p.real)
            if p_prev is None:
                ef = 0.0
            else:
                q = p * p_prev.conjugate()
                ang = atan2(q.imag, q.real)
                if cuad:
                    ang *= 0.5
                elif self.disc == 'costas':
                    ang = _reducir_pi(ang)
                ef = ang / t                          # rad/s
            p_prev = p
            th_out[k] = th
            # filtro de lazo
            if pll:
                asiste = (self.asistencia == 'siempre'
                          or (self.asistencia == 'adaptativa' and not self.enganchado))
                if orden3:
                    i2 += c0 * th * t
                i1 += (c1 * th + i2 + (wf * ef if asiste else 0.0)) * t
                w_nuevo = c2 * th + i1
            else:
                i2 += kf2 * ef * t
                i1 += (kf1 * ef + i2) * t
                w_nuevo = i1
            fase = (fase + w * n) % DOS_PI
            w_cmd = w_nuevo
        self.fase, self.w_cmd, self.i1, self.i2 = fase, w_cmd, i1, i2
        self.p_prev = p_prev
        self.hist = np.concatenate([self.hist, p_out])[-self.n_vent:]
        out = {'f_hz': f_out, 'fase_mid': mid, 'p': p_out, 'th': th_out}
        if rotada:
            out['rot'] = rot.reshape(-1)
        return out

    # ------------------------------------------------------ enganche y C/N0

    def evaluar(self, umbral_alto=0.6, umbral_bajo=0.3, ruido_prompt=None):
        """Detector de enganche y C/N0 sobre la ventana de las ultimas
        `n_ventana` epocas. Devuelve dict(metrica, cn0_dbhz, amp, enganchado).

        metrica = <I^2 - Q^2> / S: en lazo enganchado vale E[cos(2 phi)] =
        exp(-2 sigma^2) ~ 1 SIN sesgo por ruido (el ruido se cancela en
        I^2 - Q^2), y cae a ~0 si la fase gira. S es la potencia de senal de la
        ventana: <I^2 + Q^2> - N si se conoce el ruido de una muestra
        post-integracion (`ruido_prompt` = E|n|^2, p. ej. el piso espectral
        sigma^2 / muestras por epoca: es lo que se usa en el nodo y vale hasta
        C/N0 bajos), o la de M2M4 si no (peor por debajo de ~30 dB-Hz).
        Histeresis: engancha por encima de `umbral_alto` (sigma ~ 29 grados),
        suelta por debajo de `umbral_bajo` (sigma ~ 44 grados). Solo valido
        con la ventana llena y disc 'pll'/'costas'."""
        p = self.hist
        if len(p) < self.n_vent:
            return {'metrica': float('nan'), 'cn0_dbhz': float('nan'),
                    'amp': float('nan'), 'enganchado': self.enganchado}
        s, nz = m2m4(p)
        if ruido_prompt is not None:
            s_det = float(np.mean(p.real ** 2 + p.imag ** 2)) - float(ruido_prompt)
            s_det = s_det if s_det > 0.05 * float(ruido_prompt) else 0.0
        else:
            s_det = s
        if not (s_det > 0.0):
            met = 0.0
        else:
            met = float(np.mean(p.real ** 2 - p.imag ** 2)) / s_det
        if met > umbral_alto:
            self.enganchado = True
        elif met < umbral_bajo:
            self.enganchado = False
        cn0 = (10.0 * math.log10(s / nz / self.t)
               if s > 0.0 and nz > 0.0 else float('nan'))
        return {'metrica': met, 'cn0_dbhz': cn0,
                'amp': math.sqrt(s) if s > 0.0 else float('nan'),
                'enganchado': self.enganchado}


# ---------------------------------------------------------------- estimadores

def m2m4(p):
    """Momentos M2 = E|p|^2, M4 = E|p|^4 de muestras complejas de una senal de
    envolvente CONSTANTE en ruido gaussiano complejo circular: M2 = S + N,
    M4 = S^2 + 4 S N + 2 N^2, asi que S = sqrt(2 M2^2 - M4) y N = M2 - S.
    Devuelve (S, N); S = nan si 2 M2^2 <= M4 (ruido y varianza del estimador
    ganan a la senal). Insesgado de forma asintotica; no necesita fase ni
    frecuencia (Pauluzzi & Beaulieu 2000, ec. 14-15, caso complejo)."""
    a2 = np.abs(p) ** 2
    m2 = float(a2.mean())
    m4 = float((a2 * a2).mean())
    d = 2.0 * m2 * m2 - m4
    if d <= 0.0:
        return float('nan'), float('nan')
    s = math.sqrt(d)
    return s, m2 - s


def cn0_m2m4(p, t_int_s):
    """C/N0 (dB-Hz) por M2M4 de las muestras post-integracion `p` de `t_int_s`
    segundos: el ruido de una integracion de T segundos tiene varianza N0/T
    con amplitud unitaria, asi que C/N0 = (S/N) / T. Valido con la portadora
    ya barrida (envolvente constante), S/N por muestra >~ -3 dB y unos
    cientos de muestras; fuera de ese rango el estimador se sesga."""
    s, nz = m2m4(np.asarray(p))
    if not (s > 0.0 and nz > 0.0):
        return float('nan')
    return 10.0 * math.log10(s / nz / float(t_int_s))


def cn0_nwpr(p, t_int_s, m=20):
    """C/N0 (dB-Hz) por potencia de banda estrecha/ancha (Van Dierendonck et
    al. 1992): grupos de `m` muestras, NBP = |sum p|^2, WBP = sum |p|^2,
    mu = <NBP/WBP> y C/N0 = (mu - 1)/(m - mu)/T. Exige la fase del lazo
    enganchada (muestras de fase constante en cada grupo)."""
    p = np.asarray(p)
    g = len(p) // m
    if g < 1:
        return float('nan')
    q = p[:g * m].reshape(g, m)
    nbp = np.abs(q.sum(axis=1)) ** 2
    wbp = (np.abs(q) ** 2).sum(axis=1)
    mu = float(np.mean(nbp / wbp))
    if not (1.0 < mu < m):
        return float('nan')
    return 10.0 * math.log10((mu - 1.0) / (m - mu) / float(t_int_s))


_HANN = {}


def _hann(n):
    """Ventana de Hann de n puntos, en cache (se pide una vez por bloque)."""
    if n not in _HANN:
        _HANN[n] = np.hanning(n)
    return _HANN[n]


def cn0_espectral(x, fs_hz, f_hz=None, sigma2=None):
    """C/N0 (dB-Hz) por potencia de banda ancha menos piso de ruido: la
    potencia total del bloque, <|x|^2> = C + sigma^2, y el piso sigma^2 leido del
    espectro (mediana de |X_k|^2 con ventana de Hann / ln 2 / sum(w^2): robusta a
    la portadora y a las bandas laterales mientras ocupen < ~30 % de los bins).
    C/N0 = (<|x|^2> - sigma^2) fs / sigma^2, con N0 = sigma^2 / fs.

    Es el complemento de M2M4 a C/N0 ALTO: cuando S/N por muestra post-
    integracion pasa de ~25 dB, M2M4 es una resta de cantidades casi iguales,
    pero aqui el error es el de la mediana (~1.3 % con 12 500 bins = 0.06 dB).
    Valido para C/N0 >~ 45 dB-Hz con bloques de 50 ms; por debajo, <|x|^2> casi
    no se distingue del piso y el error sube (promediar bloques). Devuelve
    (cn0_dbhz, amplitud) con amplitud = sqrt(C) en unidades de la entrada."""
    x = np.asarray(x)
    if sigma2 is None:
        sigma2 = piso_ruido(x, fs_hz, f_hz)
    p_tot = float(np.mean(x.real ** 2 + x.imag ** 2))
    c = p_tot - sigma2
    if c <= 0.0 or sigma2 <= 0.0:
        return float('nan'), float('nan')
    return 10.0 * math.log10(c * fs_hz / sigma2), math.sqrt(c)


def piso_ruido(x, fs_hz, f_hz=None):
    """Varianza de ruido por muestra (sigma^2, potencia compleja) leida del
    espectro del bloque `x`: mediana de |X_k|^2 con ventana de Hann /
    ln 2 / sum(w^2) (la potencia de un bin de ruido es exponencial: mediana =
    ln 2 por la media). Se lee en la mayor potencia de 2 <= len(x) (FFT rapida;
    con 4 096-32 768 bins el error de la mediana es 1.7-0.6 %). Si se da la
    frecuencia de la portadora `f_hz`, solo cuenta la MITAD del espectro mas
    lejana de ella: las faldas del filtro de transmision (RRC truncado, ~-45
    dB) subirian el piso un 10 % cerca de la banda."""
    m = 1 << int(math.log2(len(x)))
    w = _hann(m)
    pw = np.abs(np.fft.fft(x[-m:] * w)) ** 2
    if f_hz is not None:
        k0 = (f_hz / fs_hz) * m
        d = np.abs((np.arange(m) - k0 + m / 2.0) % m - m / 2.0)
        pw = pw[d > m / 4.0]
    return float(np.median(pw)) / math.log(2.0) / float(np.sum(w * w))


# ---------------------------------------------------------------- adquisicion

def adquirir_fft(x, fs_hz, ventana_hz, cuadrado=False, pfa=1e-3, relleno=4):
    """Busca la portadora en la ventana de Doppler `ventana_hz = (f_min, f_max)`
    con una FFT de `relleno`*len(x) puntos (relleno de ceros) y la interpolacion
    parabolica del pico (log-potencia, 3 bins). Con `cuadrado=True` busca el
    pico de x^2 (que quita la BPSK) en 2*ventana y devuelve f/2.

    Deteccion CFAR: la potencia de un bin de ruido es exponencial, asi que con
    M bins independientes en la ventana el umbral para una falsa alarma `pfa`
    es gamma = ln(M_eff / pfa) veces la media del ruido, que se estima con la
    MEDIANA de todo el espectro / ln 2 (robusta al pico). Resolucion de
    frecuencia 1/T con T = len(x)/fs (sin interpolar). Valido si el Doppler
    cambia menos de ~1 bin (tasa * T^2 < 1) durante la ventana; si no, el pico
    se reparte y se pierde ganancia.
    Devuelve dict(f_hz, snr_db (pico/ruido), umbral_db, detectado)."""
    x = np.asarray(x)
    n = len(x)
    z = x * x if cuadrado else x
    nfft = 1 << int(math.ceil(math.log2(relleno * n)))
    pw = np.abs(np.fft.fftshift(np.fft.fft(z, nfft))) ** 2
    fr = np.fft.fftshift(np.fft.fftfreq(nfft, 1.0 / fs_hz))
    lo, hi = ventana_hz
    if cuadrado:
        lo, hi = 2.0 * lo, 2.0 * hi
    idx = np.nonzero((fr >= lo) & (fr <= hi))[0]
    j = idx[int(np.argmax(pw[idx]))]
    ruido = float(np.median(pw)) / math.log(2.0)
    # Con relleno de ceros la envolvente continua del espectro tiene ~2.8 veces
    # mas "intentos" independientes que los bins sin rellenar (medido con 10^4
    # corridas de ruido: la tasa de falsas alarmas sale 2.8-3.2 veces la de
    # diseno si no se corrige; con relleno 1 sale 1.0 veces)
    m_indep = max(2.0, (hi - lo) * n / fs_hz * (2.8 if relleno > 1 else 1.0))
    gamma = math.log(m_indep / pfa)
    f = fr[j]
    if 0 < j < nfft - 1 and pw[j - 1] > 0 and pw[j + 1] > 0 and pw[j] > 0:
        a, b, c = (math.log(pw[j - 1]), math.log(pw[j]), math.log(pw[j + 1]))
        den = a - 2.0 * b + c
        if den < 0:
            f += 0.5 * (a - c) / den * (fr[1] - fr[0])
    if cuadrado:
        f *= 0.5
    razon = float(pw[j]) / ruido
    return {'f_hz': float(f), 'snr_db': 10.0 * math.log10(razon),
            'umbral_db': 10.0 * math.log10(gamma), 'detectado': bool(razon > gamma)}


def ancho_banda_discreto(orden, bn_hz, t_int_s, n=20000):
    """B_L EFECTIVO (Hz) del lazo digital de `SeguidorPortadora`: simula el lazo
    lineal (un retardo de computo, integracion y descarga a mitad de epoca,
    integradores de Euler) con una entrada impulsiva y calcula
    B_L = sum(h^2) / (2 T). Es el ancho de banda con el que hay que comparar la
    varianza medida: sigma^2 = B_L / (C/N0) (1 + 1/(2 T C/N0)) (Kaplan &
    Hegarty, eq. 5.? del PLL con atan2). Con B_L T = 0.02 supera al nominal en
    un 4-5 %; con 0.1, en un 27 % (3.er orden)."""
    c0, c1, c2, _ = coef_lazo(orden, bn_hz)
    t = float(t_int_s)
    i1 = i2 = fase = w = 0.0
    suma = 0.0
    for k in range(int(n)):
        m = fase + w * t / 2.0
        suma += m * m                       # entrada: impulso en k = 0
        e = (1.0 if k == 0 else 0.0) - m
        i2 += c0 * e * t
        i1 += (c1 * e + i2) * t
        fase += w * t
        w = c2 * e + i1
    return suma / (2.0 * t)


def varianza_fase_por_oscilador(orden, bn_hz, t_int_s, lw_hz, n=20000):
    """Varianza (rad^2) del error de fase del lazo cuando el oscilador del
    TRANSMISOR/receptor tiene ruido de frecuencia blanco de ancho de linea
    `lw_hz` (caminata aleatoria de fase, var(dphi) = 2 pi lw T por epoca).
    La respuesta del error a un escalon de fase unitario es g_k (lazo lineal,
    las mismas ecuaciones que `ancho_banda_discreto`) y, como la fase es la
    suma de incrementos blancos, var(e) = 2 pi lw T sum g_k^2. Valido en el
    rango lineal (sigma < ~0.5 rad) y con C/N0 alto (sin ruido de medida).
    Referencia: Rubiola 2009, cap. 1; Gardner, Phaselock Techniques, cap. 6."""
    c0, c1, c2, _ = coef_lazo(orden, bn_hz)
    t = float(t_int_s)
    i1 = i2 = fase = w = 0.0
    suma = 0.0
    for k in range(int(n)):
        m = fase + w * t / 2.0
        e = 1.0 - m                          # escalon de fase en k = 0
        suma += e * e
        i2 += c0 * e * t
        i1 += (c1 * e + i2) * t
        fase += w * t
        w = c2 * e + i1
    return 2.0 * math.pi * float(lw_hz) * t * suma
