"""Terminal optico de un satelite: parametros y presets.

Arquitectura gruesa/fina (curso 30 sec. 5, y la de todo terminal operativo):
  * montura GRUESA: cardan (o el propio ADCS) que apunta con la efemeride y
    sigue la linea de vista (~mrad/s) con ancho de banda de ~1 Hz;
  * espejo de apuntamiento FINO (FSM): estrecho en recorrido (+-1 mrad) y
    rapido (cruce del lazo de cientos de Hz), cerrado sobre un sensor de
    cuadrante (QPD) que ve el haz recibido;
  * camara de ADQUISICION de campo ancho y baliza (haz mas ancho que el de
    comunicacion) para el barrido inicial.

Cada cifra dice su fuente o SUPUESTO. Los presets 'LCT_*' sirven para
contrastar el modelo de enlace con los terminales publicados de Tesat.
"""
from dataclasses import dataclass, field, replace

from sat_isl.enlace import GAMMA_KLEIN, Apd, Homodino


@dataclass(frozen=True)
class Lazo:
    """Lazo de seguimiento fino + grueso (ver `seguimiento.py`)."""
    fs_hz: float = 10_000.0          # muestreo del lazo fino (QPD / FPGA). SUPUESTO tipico
    fsm_fn_hz: float = 2_000.0       # resonancia del FSM (piezo/bobina movil, 1-3 kHz). SUPUESTO
    fsm_zeta: float = 0.3            # amortiguamiento del FSM. SUPUESTO
    fsm_carrera_urad: float = 1_000.0  # +-recorrido en espacio del haz. SUPUESTO tipico (+-1 mrad)
    f_cruce_hz: float = 400.0        # cruce de ganancia objetivo del lazo fino
    retardo_muestras: float = 1.0    # computo + transporte, ademas de la retencion (0.5)
    # montura gruesa que descarga al FSM
    f_descarga_hz: float = 1.5       # ancho de banda del lazo FSM -> montura (cardan)
    cardan_fn_hz: float = 30.0       # servo del cardan (2.o orden). SUPUESTO
    cardan_zeta: float = 0.7
    nombre_grueso: str = 'cardan'


LAZO_CARDAN = Lazo()
# El propio ADCS como montura gruesa: servo ~ wn del PD de sat_adcs (0.1 rad/s ~ 0.016 Hz)
# y descarga del FSM aun mas lenta (0.005 Hz): el FSM carga con casi todo. SUPUESTO.
LAZO_ADCS = replace(LAZO_CARDAN, f_descarga_hz=0.005, cardan_fn_hz=0.05, cardan_zeta=0.9,
                    nombre_grueso='adcs')


@dataclass(frozen=True)
class Terminal:
    nombre: str = 'ISL-1550 (supuesto)'
    lam_m: float = 1550e-9
    D_m: float = 0.05                # apertura (tx = rx)
    p_tx_w: float = 1.0              # potencia optica de comunicacion
    gamma: float = GAMMA_KLEIN       # radio de apertura / radio del haz (Klein y Degnan)
    eta_tx: float = 0.5              # eficiencia optica de transmision. SUPUESTO (-3 dB)
    eta_rx: float = 0.4              # recepcion + acoplo a fibra monomodo. SUPUESTO (-4 dB)
    detector: object = field(default_factory=Homodino)
    # sensor de seguimiento (QPD de APD): ver seguimiento.ruido_qpd_urad
    qpd_fraccion: float = 0.1        # fraccion de la potencia recibida que va al QPD. SUPUESTO
    qpd_w_urad: float = 100.0        # radio 1/e^2 de la mancha en el QPD (en angulo, defocalizada)
    qpd_nep_w_rthz: float = 1.1e-13  # NEP de entrada (APD M = 10 + TIA 1 pA/rtHz). SUPUESTO
    qpd_eta: float = 0.8
    cal_point_ahead_urad: float = 0.5   # error de calibracion del adelanto, 1 sigma. SUPUESTO
    # baliza de adquisicion
    p_baliza_w: float = 0.5
    theta_baliza_urad: float = 300.0    # semiangulo 1/e^2 de la baliza. SUPUESTO (el optimo se calcula en adquisicion.py)
    eta_baliza: float = 0.5
    # camara de adquisicion (InGaAs): SUPUESTOS tipicos
    cam_dwell_s: float = 0.005       # tiempo de integracion/estabilizacion por paso de barrido
    cam_ruido_e: float = 150.0       # ruido de lectura, e- rms
    cam_oscuro_e_s: float = 5e4      # corriente de oscuridad, e-/s
    cam_fondo_e_s: float = 1e5       # fondo (resplandor, limbo) en el campo, e-/s. SUPUESTO
    cam_pfa: float = 1e-6            # probabilidad de falsa alarma por paso
    cam_eta: float = 0.8
    eta_rx_cam: float = 0.6          # transmision del camino telescopio -> camara. SUPUESTO
    cam_centroide_urad: float = 3.0  # error del centroide de la baliza (1 sigma por eje). SUPUESTO
    # barrido
    t_asiento_cardan_s: float = 0.05  # asentamiento si el barrido sale del recorrido del FSM
    sigma_cardan_urad: float = 50.0   # exactitud absoluta del cardan (codificador). SUPUESTO
    lazo: Lazo = field(default_factory=Lazo)


# ---------------- terminales publicados, solo para contrastar el enlace --------
# Tesat LCT de TerraSAR-X / NFIRE (2008): 1064 nm, telescopio de 125 mm, 0.7 W
# opticos, BPSK homodino a 5.625 Gb/s, BER mejor que 1e-9, enlaces de ~5000 km
# (Smutny et al., Proc. SPIE 7199, 2009 y Gregory et al., Opt. Eng. 51(3), 2012;
# cifras leidas en resumenes/fichas, ver README). Las perdidas opticas y la
# penalizacion del receptor NO son publicas en lo que se pudo leer: SUPUESTOS.
LCT_TSX_NFIRE = Terminal(nombre='Tesat LCT (TSX-NFIRE)', lam_m=1064e-9, D_m=0.125, p_tx_w=0.7,
                         eta_tx=0.5, eta_rx=0.45, detector=Homodino(penalizacion_db=5.0))
# Tesat LCT 135 (Alphasat/EDRS): 135 mm, 1.8 Gb/s sobre 45 000 km (LEO-GEO).
# Los 2.2 W opticos son de memoria de la hoja del fabricante (SIN VERIFICAR).
LCT_135 = Terminal(nombre='Tesat LCT 135 (EDRS)', lam_m=1064e-9, D_m=0.135, p_tx_w=2.2,
                   eta_tx=0.5, eta_rx=0.45, detector=Homodino(penalizacion_db=5.0))

# Terminal del escenario: clase nanosatelite (SUPUESTOS), con detector APD o coherente
ISL_COHERENTE = Terminal()
ISL_APD = replace(ISL_COHERENTE, nombre='ISL-1550 APD (supuesto)', detector=Apd())
