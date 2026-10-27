"""Seguridad del rover (R2): alarmas de vuelco, pendiente, atasco y energia. Python puro.

Una alarma se emite al activarse y se repite cada `repetir_s` mientras siga activa (anti-spam).
Formato (CONTRATO sec. 18): {"t":..., "tipo":"vuelco|atasco|pendiente|energia", "detalle":...}.
`vuelco` salta por |cabeceo| o |alabeo| > vuelco_deg o, si el estado trae `margen_vuelco_deg` (R6, suspension rocker_bogie), por un
margen de estabilidad menor que `margen_vuelco_deg` (el CM cerca de salirse del poligono de apoyo, que depende del terreno y la carga).
`energia_wh` del estado se interpreta como energia CONSUMIDA; la alarma salta al pasar `energia_max_wh`.
"""
from dataclasses import dataclass

TIPOS = ('vuelco', 'pendiente', 'atasco', 'energia')
FRENAN = ('vuelco', 'pendiente', 'energia')   # el atasco no frena: se avisa y se reduce par


@dataclass
class ParamsSeguridad:
    vuelco_deg: float = 35.0       # |cabeceo| o |alabeo| mayor
    margen_vuelco_deg: float = 8.0  # R6: margen de estabilidad (estado `margen_vuelco_deg`, rocker_bogie) por debajo de esto = vuelco inminente
    pendiente_deg: float = 25.0
    atasco_v: float = 0.02         # |v| por debajo de esto ...
    atasco_orden_v: float = 0.1    # ... con una orden de al menos esto ...
    atasco_s: float = 3.0          # ... durante este tiempo
    energia_max_wh: float = 1e9
    repetir_s: float = 5.0


class MonitorSeguridad:
    def __init__(self, params=None):
        self.p = params or ParamsSeguridad()
        self.activas = set()
        self._ult = {}
        self._t_atasco = None

    def evaluar(self, estado, v_orden=0.0):
        """-> lista de dicts de alarma a publicar ahora. `self.activas` queda con los tipos vigentes."""
        p = self.p
        t = estado.get('t', 0.0)
        cond = {}
        cab, ala = estado.get('cabeceo_deg'), estado.get('alabeo_deg')
        if cab is not None and ala is not None and max(abs(cab), abs(ala)) > p.vuelco_deg:
            cond['vuelco'] = {'cabeceo_deg': cab, 'alabeo_deg': ala, 'limite_deg': p.vuelco_deg}
        marg = estado.get('margen_vuelco_deg')
        if marg is not None and marg < p.margen_vuelco_deg and 'vuelco' not in cond:
            cond['vuelco'] = {'margen_vuelco_deg': marg, 'limite_margen_deg': p.margen_vuelco_deg}
        pend = estado.get('pendiente_deg')
        if pend is not None and pend > p.pendiente_deg:
            cond['pendiente'] = {'pendiente_deg': pend, 'limite_deg': p.pendiente_deg}
        if estado.get('energia_wh', 0.0) > p.energia_max_wh:
            cond['energia'] = {'energia_wh': estado['energia_wh'], 'limite_wh': p.energia_max_wh}
        bloqueado = abs(estado.get('v', 0.0)) < p.atasco_v and abs(v_orden) >= p.atasco_orden_v
        if bloqueado or estado.get('atascado'):
            if self._t_atasco is None:
                self._t_atasco = t
            if estado.get('atascado') or t - self._t_atasco >= p.atasco_s:
                cond['atasco'] = {'v': estado.get('v', 0.0), 'orden_v': v_orden, 'desde_t': self._t_atasco}
        else:
            self._t_atasco = None
        salida = []
        for tipo in TIPOS:
            if tipo in cond:
                if tipo not in self.activas or t - self._ult[tipo] >= p.repetir_s:
                    self._ult[tipo] = t
                    salida.append({'t': t, 'tipo': tipo, 'detalle': cond[tipo]})
                self.activas.add(tipo)
            else:
                self.activas.discard(tipo)
        return salida

    def debe_frenar(self):
        return any(a in self.activas for a in FRENAN)
