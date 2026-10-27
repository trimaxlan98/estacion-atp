"""Logica pura (sin ROS) de la fusion de ejes del microcontrolador.

Cada ESP32 gobierna UN eje y publica en `/atp/mcu/<eje>/joint_states` un
JointState con un solo nombre. La Estacion (error_monitor, grabador, ...)
espera `/joint_states` con name=['az', 'el'] (CONTRATO.md, seccion 7): aqui se
combinan. Los indices y mascaras del diagnostico son los de
firmware/esp32_eje/uros/eje_uros.h (enum DIAG_*) y core/eje_lazo.h (EJE_F_*).
"""

ESTADOS = ('DESARMADO', 'ESPERA', 'ACTIVO', 'RETENCION', 'FALLA')

FALLAS = (
    (0x01, 'ESTOP'),       # paro de emergencia abierto
    (0x02, 'LIMITE'),      # posicion fuera de limites +- margen
    (0x04, 'VEL'),         # sobrevelocidad
    (0x08, 'ATASCO'),      # par casi maximo sostenido sin moverse
    (0x10, 'ENCODER'),     # salto imposible del encoder
    (0x20, 'CORR'),        # sobrecorriente
    (0x40, 'CFG'),         # configuracion invalida
)

INDICE_DIAG = {
    'estado': 0, 'fallas': 1, 'pos': 2, 'vel': 3, 'u': 4, 'duty': 5,
    'corriente': 6, 'ref_cmd': 7, 'ref_edad': 8, 't_lazo_us': 9,
    't_lazo_max_us': 10, 'jitter_max_us': 11, 'ticks_perdidos': 12,
    'refs_recibidas': 13, 'abs_err': 14, 'uptime_s': 15,
}


def nombres_fallas(mascara):
    """Nombres de los bits de falla activos en la mascara (entero o float)."""
    m = int(mascara)
    return [n for b, n in FALLAS if m & b]


def decodificar_diag(data):
    """Float32MultiArray.data -> dict legible; None si el arreglo es corto."""
    if len(data) < len(INDICE_DIAG):
        return None
    d = {k: float(data[i]) for k, i in INDICE_DIAG.items()}
    e = int(d['estado'])
    d['estado_nombre'] = ESTADOS[e] if 0 <= e < len(ESTADOS) else f'?{e}'
    d['fallas_nombres'] = nombres_fallas(d['fallas'])
    return d


class Fusion:
    """Combina los ejes en un solo JointState de [az, el].

    * `ejes`: orden de salida (por defecto az, el).
    * `simulados`: ejes SIN microcontrolador (banco con un solo ESP32). Su
      valor sale de la referencia, o sea que su error es 0 por construccion:
      sirve para probar un eje, no para medir apuntamiento.
    * Un eje REAL cuyo ultimo mensaje es mas viejo que `caduca_s` NO se
      rellena: `combinar` devuelve None y no se publica nada, para que quien
      consume vea que falta el dato en vez de una posicion congelada.
    """

    def __init__(self, ejes=('az', 'el'), simulados=(), caduca_s=0.5):
        self.ejes = tuple(ejes)
        self.simulados = frozenset(simulados)
        self.caduca_s = float(caduca_s)
        self.ultimo = {}          # eje -> (pos, vel, effort, t_recepcion)
        self.ref = {}             # eje -> (pos, t_recepcion)

    def medida(self, eje, pos, vel, effort, t):
        self.ultimo[eje] = (float(pos), float(vel), float(effort), float(t))

    def referencia(self, eje, pos, t):
        self.ref[eje] = (float(pos), float(t))

    def combinar(self, ahora):
        """-> (nombres, posiciones, velocidades, esfuerzos) o None si falta algun eje real."""
        pos, vel, eff = [], [], []
        for e in self.ejes:
            if e in self.simulados:
                if e not in self.ref:
                    return None
                p = self.ref[e][0]
                pos.append(p); vel.append(0.0); eff.append(0.0)
                continue
            if e not in self.ultimo:
                return None
            p, v, f, t = self.ultimo[e]
            if ahora - t > self.caduca_s:
                return None
            pos.append(p); vel.append(v); eff.append(f)
        return list(self.ejes), pos, vel, eff

    def faltan(self, ahora):
        """Ejes reales sin dato o con dato caducado (para el aviso en el log)."""
        malos = []
        for e in self.ejes:
            if e in self.simulados:
                continue
            if e not in self.ultimo or ahora - self.ultimo[e][3] > self.caduca_s:
                malos.append(e)
        return malos
