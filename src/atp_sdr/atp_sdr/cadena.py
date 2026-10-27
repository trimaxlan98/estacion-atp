"""Cadena de RF completa por pasos de 50 ms: beacon IQ -> receptor -> medidas.

Une `senal` (generador del beacon con el Doppler, el C/N0 y los fenomenos del
oscilador), `receptor` (adquisicion FFT, seguimiento FLL/PLL, enganche) y
`bpsk` (filtro adaptado, Gardner, Costas). Es la MISMA clase que usa el nodo
ROS `sdr_receiver` y las pruebas sobre pases: un paso = un bloque IQ.

    cad = CadenaRF('S', 'portadora', semilla=3)
    for t, d_km, vr, err in pase:          # cada 50 ms
        m = cad.paso(t, d_km, vr, err)     # dict con Doppler/C/N0/enganche
    print(cad.resumen())

Solo numpy y `atp_ros.model` (presupuesto de enlace), sin ROS.
"""
import math
import time

import numpy as np

from atp_ros import model
from atp_sdr.bpsk import ReceptorBPSK, alinear_ber
from atp_sdr.receptor import (SeguidorPortadora, adquirir_fft, cn0_espectral,
                              piso_ruido)
from atp_sdr.senal import Beacon, doppler_de_vr, ventana_doppler_hz

BUSQUEDA, SEGUIMIENTO, ENGANCHADO = 0, 1, 2
NOMBRES_ESTADO = ('BUSQUEDA', 'SEGUIMIENTO', 'ENGANCHADO')
# B_L del PLL por banda (Hz): la jerk del Doppler crece con f0, y con ella el
# error de arrastre del 3.er orden (theta = jerk / wn^3)
BN_DEFECTO = {'UHF': 10.0, 'S': 30.0, 'X': 60.0, 'Ka': 100.0}
NAN = float('nan')


def fs_para(f0_hz, modo='portadora', h_km=550.0, margen_hz=0.0):
    """Tasa de muestreo (multiplo de 50 kHz) que contiene la ventana de Doppler
    de la banda: 2.5 veces el semiancho para la portadora; 4.2 veces para BPSK,
    porque la busqueda por cuadrado duplica la frecuencia."""
    v = ventana_doppler_hz(f0_hz, h_km, margen_hz)
    k = 2.5 if modo == 'portadora' else 4.2
    return max(50e3, math.ceil(k * v / 50e3) * 50e3)


class CadenaRF:
    """Generador + receptor de un beacon por pasos de `dt_s`.

    Parametros principales (resto: presupuesto de enlace de `model`):
      banda            'UHF' | 'S' | 'X' | 'Ka' (frecuencia de bajada de model.BANDAS)
      modo             'portadora' (tono, PLL con atan2) o 'bpsk'
      semilla          determinismo: misma semilla, mismas muestras y mismo resumen
      fs_hz            None: la que pide la banda (ver `fs_para`)
      orden, bn_hz     lazo de portadora (3.er orden; B_L por defecto segun la banda)
      t_int_s          integracion y periodo del lazo (1 ms)
      offset_lo_hz, lw_lo_hz, deriva_lo_hz_s   defectos del oscilador local
      delta_cn0_db     degrada el C/N0 del presupuesto (para probar el umbral)
      tasa_simbolos    Rs en modo bpsk (12.5 ksps: ~600 simbolos por bloque)
      h_km             altitud para la ventana de Doppler esperada
      ventana_hz       semiancho de la busqueda (None: la del pase LEO de h_km;
                       con `fd_pred_hz` en `paso`, el del residuo de la efemeride)
    """

    def __init__(self, banda='S', modo='portadora', semilla=1, dt_s=0.05,
                 fs_hz=None, orden=3, bn_hz=None, bn_fll_hz=10.0, t_int_s=1e-3,
                 offset_lo_hz=0.0, lw_lo_hz=0.0, deriva_lo_hz_s=0.0,
                 delta_cn0_db=0.0, eirp_dbw=9.0, otras_perdidas_db=3.0,
                 dish_m=model.DIAMETRO_PLATO_M, eta=0.6, tsys_k=model.T_SISTEMA_K,
                 tasa_simbolos=12.5e3, h_km=550.0, pfa=1e-3, f0_hz=None,
                 margen_ventana_hz=None, ventana_hz=None):
        self.banda = banda
        self.modo = modo
        self.f0 = float(f0_hz) if f0_hz else model.BANDAS[banda]
        self.dt = float(dt_s)
        self.t_int = float(t_int_s)
        self.delta_cn0 = float(delta_cn0_db)
        self.kw = dict(f_hz=self.f0, eirp_dbw=float(eirp_dbw),
                       perdidas_db=float(otras_perdidas_db), diametro_m=float(dish_m),
                       eta=float(eta), t_sys_k=float(tsys_k))
        margen = (abs(float(offset_lo_hz)) + 0.5e-6 * self.f0
                  if margen_ventana_hz is None else float(margen_ventana_hz))
        self.ventana = (float(ventana_hz) if ventana_hz
                        else ventana_doppler_hz(self.f0, h_km, margen))
        self.fs = float(fs_hz) if fs_hz else fs_para(self.f0, modo, h_km, margen)
        self.n_blk = int(round(self.fs * self.dt))
        self.pfa = float(pfa)
        self.beacon = Beacon(self.fs, modo, tasa_simbolos=tasa_simbolos,
                             offset_lo_hz=offset_lo_hz, deriva_lo_hz_s=deriva_lo_hz_s,
                             lw_lo_hz=lw_lo_hz, semilla=semilla)
        bn = float(bn_hz) if bn_hz else BN_DEFECTO.get(banda, 30.0)
        if modo == 'portadora':
            self.seg = SeguidorPortadora(self.fs, self.t_int, orden, bn, bn_fll_hz,
                                         disc='pll', modo='pll')
            self.rx = None
        else:
            self.rx = ReceptorBPSK(self.fs, tasa_simbolos, t_int_s=self.t_int,
                                   bn_fll_hz=max(bn_fll_hz, 20.0))
            self.seg = self.rx.etapa1
        self.n_ep = self.seg.n_ep
        # ventana de la adquisicion: el Doppler no debe moverse mas de ~1 bin
        # (tasa * T^2 < 1) y no puede pasar del bloque
        tasa_max = model.tasa_doppler_cenit(h_km, self.f0)
        t_acq = min(self.dt, 1.0 / math.sqrt(max(tasa_max, 1.0)))
        self.n_acq = max(self.n_ep, int(t_acq * self.fs) // self.n_ep * self.n_ep)
        self.estado = BUSQUEDA
        self._fd_prev = None
        self._pred_prev = None
        self._sin_enganche = 0
        # estadisticas del pase
        self.n_bloques = 0
        self.n_enganchado = 0
        self.n_perdidas = 0
        self.n_adquisiciones = 0
        self.t_gen = 0.0
        self.t_rx = 0.0
        self.muestras = 0
        self._e2 = 0.0               # suma de errores^2 por epoca, enganchado
        self._e1 = 0.0
        self._n_e = 0
        self._emax = 0.0
        self._dcn0 = []              # (est - presupuesto), enganchado
        self._ult_t = None
        self._rx_eng = False
        self.huecos = 0
        self._ber_desfase = None
        self._ber_err = 0
        self._ber_n = 0
        self._ber_inv = False
        self._bits_vistos = 0

    # ------------------------------------------------------------ un paso

    def paso(self, t_s, range_km, vr_km_s, err_deg, extra_db=0.0, fd_pred_hz=None):
        """Genera el bloque IQ del paso (Doppler de `vr_km_s`, C/N0 del
        presupuesto con el rango y el desapuntamiento `err_deg`, mas `extra_db`
        de ganancia/perdida adicional, p. ej. la modulacion del conical scan) y
        lo procesa. Devuelve un dict con las medidas del bloque (ver la clave de
        cada una en `sdr_interfaces/SdrStatus`).

        `fd_pred_hz`: Doppler PREDICHO por la efemeride con que la estacion
        sintoniza su oscilador local (como hace una estacion real): a banda
        base solo queda el residuo fd - fd_pred y la ventana de busqueda puede
        ser mucho mas angosta (fs menor). `doppler_est_hz` lo suma de vuelta, de
        modo que sigue siendo el Doppler completo."""
        c0 = time.thread_time()
        if self._ult_t is not None and t_s - self._ult_t > 1.5 * self.dt:
            self.huecos += int(round((t_s - self._ult_t) / self.dt)) - 1
        self._ult_t = t_s
        fd = float(doppler_de_vr(vr_km_s, self.f0))
        pred = 0.0 if fd_pred_hz is None else float(fd_pred_hz)
        bb = fd - pred                              # Doppler a banda base
        bb_ini = bb if self._fd_prev is None else self._fd_prev
        pred_ini = pred if self._pred_prev is None else self._pred_prev
        self._fd_prev, self._pred_prev = bb, pred
        lb = model.presupuesto_enlace(range_km, err_deg, **self.kw)
        cn0_b = float(lb['cn0_dbhz']) + self.delta_cn0 + float(extra_db)
        x = self.beacon.bloque(self.n_blk, bb_ini, bb, cn0_b)
        c1 = time.thread_time()

        n_ep = self.n_ep
        enganchado_antes = self.estado == ENGANCHADO
        ep0 = 0
        f_ant = self.seg.f_hz if self.seg.siguiendo else None
        sigma2 = piso_ruido(x, self.fs, f_ant)      # piso de ruido leido del espectro
        if not self.seg.siguiendo:
            acq = adquirir_fft(x[:self.n_acq], self.fs, (-self.ventana, self.ventana),
                               cuadrado=self.modo == 'bpsk', pfa=self.pfa)
            if acq['detectado']:
                self.n_adquisiciones += 1
                if self.rx is not None:
                    self.rx.iniciar(acq['f_hz'])
                else:
                    self.seg.iniciar(acq['f_hz'])
                ep0 = self.n_acq // n_ep
                self._sin_enganche = 0
        f_est = f_ver = NAN
        met = cn0_m = NAN
        # amplitud y C/N0 espectrales: no dependen del lazo, valen aunque no
        # se este siguiendo (el autotrack por amplitud los usa)
        cn0_s, amp_s = cn0_espectral(x, self.fs, f_ant, sigma2)
        if self.seg.siguiendo:
            if self.rx is not None:
                r = self.rx.procesar(x[ep0 * n_ep:])
            else:
                r = self.seg.procesar(x[ep0 * n_ep:])
            nep = len(r['f_hz'])
            # Doppler verdadero a mitad de cada epoca procesada (mismo modelo
            # lineal que uso el generador dentro del bloque)
            dur = self.n_blk / self.fs
            tm = (np.arange(ep0, ep0 + nep) * n_ep + 0.5 * (n_ep - 1)) / self.fs
            f_true = bb_ini + (bb - bb_ini) * tm / dur
            # el error incluye el desplazamiento del oscilador, si lo hay:
            # un receptor sin calibrar lo cuenta como Doppler
            err = r['f_hz'] - f_true
            f_est = float(np.mean(r['f_hz'])) + float(np.mean(
                pred_ini + (pred - pred_ini) * tm / dur))
            f_ver = float(np.mean(f_true)) + float(np.mean(
                pred_ini + (pred - pred_ini) * tm / dur))
            ev = self._evaluar(sigma2)
            met = ev['metrica']
            cn0_m = ev['cn0_dbhz']
            self.estado = ENGANCHADO if ev['enganchado'] else SEGUIMIENTO
            if enganchado_antes and self.estado == ENGANCHADO:
                self._e2 += float(np.sum(err * err))
                self._e1 += float(np.sum(err))
                self._n_e += nep
                self._emax = max(self._emax, float(np.max(np.abs(err))))
            if enganchado_antes and self.estado != ENGANCHADO:
                self.n_perdidas += 1
                self._reiniciar_busqueda()
            elif self.estado == SEGUIMIENTO:
                self._sin_enganche += 1
                if self._sin_enganche > 40:          # 2 s sin enganchar: a buscar
                    self._reiniciar_busqueda()
            else:
                self._sin_enganche = 0
            if self.estado == ENGANCHADO:
                self._dcn0.append(self._cn0_final(cn0_m, cn0_s) - cn0_b)
        else:
            self.estado = BUSQUEDA
        if self.rx is not None and self.seg.siguiendo:
            self._ber_actualizar()
        self.n_bloques += 1
        if self.estado == ENGANCHADO:
            self.n_enganchado += 1
        self.muestras += self.n_blk
        c2 = time.thread_time()
        self.t_gen += c1 - c0
        self.t_rx += c2 - c1
        cn0_est = self._cn0_final(cn0_m, cn0_s) if self.seg.siguiendo else cn0_s
        return {
            'estado': self.estado, 'locked': self.estado == ENGANCHADO,
            'doppler_true_hz': f_ver, 'doppler_est_hz': f_est,
            'doppler_err_hz': f_est - f_ver,
            'cn0_budget_dbhz': cn0_b, 'cn0_est_dbhz': cn0_est,
            'cn0_m2m4_dbhz': cn0_m, 'cn0_spec_dbhz': cn0_s,
            'amp_est': amp_s, 'lock_metric': met, 'n_samples': self.n_blk,
            'ber': self.ber(), 'proc_ms': (c2 - c0) * 1e3,
            'fd_true_fin_hz': fd, 'l_point_db': float(lb['l_point_db']),
        }

    # ------------------------------------------------------------ auxiliares

    def _evaluar(self, sigma2):
        """Enganche y C/N0 de la ventana. El ruido de una muestra post-integracion
        sale del piso espectral (sigma^2 / muestras por epoca; en BPSK, por
        simbolo), asi que el detector no se sesga a C/N0 bajo."""
        if self.rx is not None:
            ev = self.rx.evaluar(ruido_simbolo=sigma2 / self.rx.sps)
            # el Costas es el que engancha: histeresis sobre su metrica
            m = ev['metrica']
            if not math.isnan(m):
                if m > 0.6:
                    self._rx_eng = True
                elif m < 0.3:
                    self._rx_eng = False
            ev['enganchado'] = self._rx_eng
            return ev
        return self.seg.evaluar(ruido_prompt=sigma2 / self.n_ep)

    def _reiniciar_busqueda(self):
        self.seg.reset()
        if self.rx is not None:
            self.rx.z = np.zeros(0, dtype=complex)     # la metrica se rehace con simbolos nuevos
        self._rx_eng = False
        self._sin_enganche = 0
        self.estado = BUSQUEDA

    @staticmethod
    def _cn0_final(cn0_m2m4, cn0_esp):
        """M2M4 si C/N0 bajo (<= 45 dB-Hz); si no, el espectral."""
        if not math.isnan(cn0_m2m4) and cn0_m2m4 <= 45.0:
            return cn0_m2m4
        return cn0_esp if not math.isnan(cn0_esp) else cn0_m2m4

    def _ber_actualizar(self):
        """BER acumulada: tras 3000 simbolos se fija el desfase y la polaridad
        (correlacion con los bits del generador) y a partir de ahi se cuentan
        los errores de cada bloque nuevo contra los bits transmitidos."""
        rx = self.rx
        tx = self.beacon.bits_tx()
        bits = rx.bits_rx()
        if self._ber_desfase is None:
            if rx.n_simbolos < 3000:
                return
            res = alinear_ber(tx, bits, n_sync=1000, descartar=2000)
            if res['n'] < 1 or abs(res['ber'] - 0.5) < 0.1:
                return
            self._ber_desfase, self._ber_inv = res['desfase'], res['invertido']
            self._bits_vistos = 2000
        i0 = self._bits_vistos
        i1 = min(len(bits), len(tx) - self._ber_desfase)
        if i1 <= i0:
            return
        r = bits[i0:i1]
        t = tx[i0 + self._ber_desfase:i1 + self._ber_desfase]
        if self._ber_inv:
            r = 1 - r
        self._ber_err += int(np.sum(r != t))
        self._ber_n += int(i1 - i0)
        self._bits_vistos = i1

    def ber(self):
        return self._ber_err / self._ber_n if self._ber_n else NAN

    # ------------------------------------------------------------ resultado

    def resumen(self):
        """Cifras del pase: Doppler (rms, sesgo, max), C/N0 medido contra
        presupuesto, fraccion enganchada y rendimiento. Solo cuenta lo que pasa
        con el receptor ENGANCHADO desde el bloque anterior (el bloque de la
        adquisicion y la captura no son seguimiento)."""
        n = max(self.n_bloques, 1)
        rms = math.sqrt(self._e2 / self._n_e) if self._n_e else NAN
        dc = np.array(self._dcn0) if self._dcn0 else np.zeros(0)
        dc = dc[~np.isnan(dc)]                  # M2M4 puede dar nan a C/N0 muy bajo
        seg = self.t_gen + self.t_rx
        return {
            'banda': self.banda, 'modo': self.modo, 'f0_hz': self.f0, 'fs_hz': self.fs,
            'bloques': self.n_bloques, 'huecos': self.huecos,
            'enganchado_pct': 100.0 * self.n_enganchado / n,
            'perdidas': self.n_perdidas, 'adquisiciones': self.n_adquisiciones,
            'doppler_rms_hz': rms,
            'doppler_sesgo_hz': self._e1 / self._n_e if self._n_e else NAN,
            'doppler_max_hz': self._emax if self._n_e else NAN,
            'cn0_sesgo_db': float(dc.mean()) if len(dc) else NAN,
            'cn0_rms_db': float(math.sqrt(np.mean(dc ** 2))) if len(dc) else NAN,
            'ber': self.ber(), 'ber_n': self._ber_n,
            'muestras_por_s_rx': self.muestras / self.t_rx if self.t_rx else NAN,
            'muestras_por_s_total': self.muestras / seg if seg else NAN,
            'ms_por_bloque_gen': 1e3 * self.t_gen / n,
            'ms_por_bloque_rx': 1e3 * self.t_rx / n,
        }


def simular_pase(cadena, perfil, t_ini=0.0, t_fin=None, err_deg=0.02):
    """Recorre un pase (`model.perfil_pase` u `orbit.perfil_tle`) con la
    cadena, paso a paso de `cadena.dt` segundos, interpolando rango y velocidad
    radial como lo hace el reference_generator. `err_deg`: error de
    apuntamiento constante (o funcion f(t) -> deg). Devuelve la lista de
    dicts de `CadenaRF.paso`, con la clave 't' agregada."""
    t_fin = float(perfil['t'][-1]) if t_fin is None else float(t_fin)
    out = []
    k = int(round(t_ini / cadena.dt))
    while k * cadena.dt <= t_fin + 1e-9:
        t = k * cadena.dt
        d = float(np.interp(t, perfil['t'], perfil['d']))
        vr = float(np.interp(t, perfil['t'], perfil['vr']))
        e = err_deg(t) if callable(err_deg) else float(err_deg)
        m = cadena.paso(t, d, vr, e)
        m['t'] = t
        out.append(m)
        k += 1
    return out


def main(argv=None):
    """`ros2 run atp_sdr sdr_offline --band Ka --el-max 60`: un pase completo
    sin ROS (misma cadena) y su resumen."""
    import argparse
    ap = argparse.ArgumentParser(description=main.__doc__)
    ap.add_argument('--band', default='S')
    ap.add_argument('--mode', default='portadora')
    ap.add_argument('--el-max', type=float, default=60.0)
    ap.add_argument('--mask', type=float, default=5.0)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--t-ini', type=float, default=0.0)
    ap.add_argument('--t-fin', type=float, default=None)
    ap.add_argument('--err-deg', type=float, default=0.02)
    ap.add_argument('--delta-cn0-db', type=float, default=0.0)
    ap.add_argument('--offset-lo-hz', type=float, default=0.0)
    ap.add_argument('--lw-lo-hz', type=float, default=0.0)
    a = ap.parse_args(argv)
    perfil = model.perfil_pase(550.0, a.el_max, a.mask)
    cad = CadenaRF(a.band, a.mode, a.seed, delta_cn0_db=a.delta_cn0_db,
                   offset_lo_hz=a.offset_lo_hz, lw_lo_hz=a.lw_lo_hz)
    simular_pase(cad, perfil, a.t_ini, a.t_fin, a.err_deg)
    for k, v in cad.resumen().items():
        print(f'{k}: {v}')


if __name__ == '__main__':
    main()
