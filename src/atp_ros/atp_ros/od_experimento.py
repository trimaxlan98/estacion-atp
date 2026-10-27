"""Escenario de la determinacion de orbita: la "verdad" numerica, el TLE
envejecido, las estaciones, los pases y la evaluacion del apuntamiento.

Solo numpy. Lo usan `scripts/experimento_od.py` (la tabla de resultados), las
pruebas y el nodo `od_estimator` (para generar mediciones mientras no exista
receptor). Todo supuesto del escenario esta aqui y en docs/OD.md.

DISENO DEL EXPERIMENTO
  * Verdad: estado SGP4 del TLE en su epoca, mas un DESFASE de epoca (el
    error del propio TLE en la epoca: ~1 km a lo largo de la pista, ver
    orbit.py) y propagado con el propagador numerico J2+J3+J4 y arrastre con un
    coeficiente balistico que difiere en `factor_bc_verdad` (+25 %) del que
    CONOCE el estimador: la incertidumbre tipica de la densidad atmosferica es
    de ese orden (Vallado 2013, cap. 8) y es lo que hace crecer el error de un
    TLE viejo.
  * A priori conocido: el TLE envejecido N dias, propagado con SGP4.
  * Estimador: modelo J2 + arrastre nominal (menos fisica que la verdad: no
    comete el "crimen inverso" de estimar con el mismo modelo que genera los
    datos). Puede estimar el factor de arrastre `p`.
  * Medidas: las de la estacion de la casa (acimut, elevacion tras el
    autotrack, y opcionalmente rango y Doppler) con las sigmas de
    `medidas.SIGMAS_DEFECTO`, generadas de la verdad con ruido.
  * Metrica: angulo entre la direccion a la orbita predicha y a la verdad,
    en cada instante de los pases siguientes (grados): lo que la antena
    erraria si apuntara a la prediccion (program track).
"""
import math

import numpy as np

from atp_ros import medidas as M
from atp_ros import od
from atp_ros import orbit
from atp_ros import propagador as P

BC_FISICO_ISS = 0.0105   # Cd A / m (m2/kg): 2.2 * 1600 m2 / 335 000 kg
# Variacion de la densidad de la VERDAD (el estimador no la conoce): un
# termino de varios dias (actividad solar/geomagnetica, +-25 %) y uno al
# periodo orbital (el abultamiento diurno de la atmosfera, +-30 %). Ordenes
# de magnitud de Vallado y McClain (2013), cap. 8: la densidad real a 400 km
# varia ~un factor 2 entre dia y noche y +-25-50 % en dias. SUPUESTO: no hay
# un modelo de atmosfera completo (NRLMSISE-00), y estas cifras fijan cuanto
# peor es la verdad que cualquier modelo de arrastre constante.
VARIACION_DENSIDAD = ((0.25, 1.7 * 86400.0, 0.7), (0.30, 5545.0, 1.1))


def rtn_base(x):
    """Ejes (radial, a lo largo de la pista, normal) del estado x = [r, v]."""
    r, v = np.asarray(x[:3], float), np.asarray(x[3:6], float)
    rh = r / np.linalg.norm(r)
    hh = np.cross(r, v)
    hh /= np.linalg.norm(hh)
    return rh, np.cross(hh, rh), hh


def calibrar_p(x0, modelo, reloj, tle, horas=24.0, paso_s=300.0, iters=14):
    """Factor `p` del coeficiente balistico que minimiza el rms de la
    diferencia de posicion entre el propagador numerico (desde el estado
    SGP4 `x0` de la epoca) y SGP4, en `horas`. Busqueda por seccion aurea en
    [0.1, 1.5] (la funcion es unimodal: el error de fase es cuadratico en el
    error de semieje). Es el "BC efectivo": absorbe que SGP4 arrastra con su
    atmosfera de ley de potencias y el B* del TLE, no con la exponencial de
    Vallado. Devuelve (p, rms_km)."""
    sg = od.EfemerideSGP4(tle, reloj)
    ts = np.arange(0.0, horas * 3600.0 + 1.0, paso_s)
    rs, _ = sg.estado(ts)

    def costo(p):
        Y = P.propagar(P.derivadas(modelo), np.r_[x0, p], 0.0, ts, metodo='rk4', h=30.0)
        return math.sqrt(float(np.mean(np.sum((Y[:, :3] - rs) ** 2, axis=1))))
    a, b = 0.1, 1.5
    phi = (math.sqrt(5.0) - 1.0) / 2.0
    c, d = b - phi * (b - a), a + phi * (b - a)
    fc, fd = costo(c), costo(d)
    for _ in range(iters):
        if fc < fd:
            b, d, fd = d, c, fc
            c = b - phi * (b - a)
            fc = costo(c)
        else:
            a, c, fc = c, d, fd
            d = a + phi * (b - a)
            fd = costo(d)
    p = 0.5 * (a + b)
    return p, costo(p)


class Escenario:
    """Verdad + a priori + estaciones. Ver el docstring del modulo.

    estaciones: dict clave -> `orbit.Estacion`. `p_cal`: factor de BC efectivo
    (None = calibrarlo contra SGP4, ~15 s). `desfase_pos_km` y
    `desfase_vel_mm_s`: error del TLE en la epoca en ejes (R, T, N); el radial
    de posicion es 0 a proposito: un error radial de 1 km cambiaria el
    semieje 2 km y el TLE derivaria 100 km/dia, no el 1-3 km/dia
    documentado."""

    def __init__(self, tle, estaciones, dias=8.0, bc_fisico=BC_FISICO_ISS,
                 factor_bc_verdad=1.25, p_cal=None, desfase_pos_km=(0.0, 1.0, 0.3),
                 desfase_vel_mm_s=(0.3, 2.0, 0.3), grado_verdad=4, grado_est=2,
                 dt_efemeride_s=30.0, variacion_densidad=VARIACION_DENSIDAD):
        self.tle = tle
        self.x_tle, self.reloj = od.estado_tle(tle)
        self.sgp4 = od.EfemerideSGP4(tle, self.reloj)
        self.claves = list(estaciones)
        self.sitios = [M.sitio(estaciones[k]) for k in self.claves]
        self.bc_fisico = bc_fisico
        self.p_cal = p_cal
        if p_cal is None:
            self.p_cal, self.rms_cal_km = calibrar_p(
                self.x_tle, P.Modelo(grado=grado_verdad, cd_a_m=bc_fisico), self.reloj, tle)
        self.modelo_est = P.Modelo(grado=grado_est, cd_a_m=bc_fisico * self.p_cal)
        self.modelo_verdad = P.Modelo(grado=grado_verdad,
                                      cd_a_m=bc_fisico * self.p_cal * factor_bc_verdad,
                                      rho_var=tuple(variacion_densidad))
        rh, th, hh = rtn_base(self.x_tle)
        dp = desfase_pos_km[0] * rh + desfase_pos_km[1] * th + desfase_pos_km[2] * hh
        dv = (desfase_vel_mm_s[0] * rh + desfase_vel_mm_s[1] * th
              + desfase_vel_mm_s[2] * hh) * 1e-6
        self.x_verdad0 = self.x_tle + np.r_[dp, dv]
        self.dias = dias
        self.ef = od.efemeride(self.modelo_verdad, self.x_verdad0, 0.0, 0.0,
                               dias * 86400.0, dt_efemeride_s)

    # ----------------------------------------------------------- consultas
    def isit(self, clave):
        return self.claves.index(clave)

    def pases(self, clave, t_ini, t_fin, el_min_deg=0.0):
        """Pases VERDADEROS de la estacion en [t_ini, t_fin] con elevacion
        maxima >= el_min_deg (cada dict ademas lleva la `clave`)."""
        out = [p for p in od.pases_efemeride(self.ef, self.reloj, self.sitios[self.isit(clave)],
                                             t_ini, t_fin)
               if p['el_max_deg'] >= el_min_deg and not p['aos_truncado']
               and not p['los_truncado']]
        for p in out:
            p['clave'] = clave
        return out

    def pases_red(self, claves, t_ini, t_fin, el_min_deg=0.0):
        out = []
        for c in claves:
            out += self.pases(c, t_ini, t_fin, el_min_deg)
        return sorted(out, key=lambda p: p['aos'])

    def medidas(self, pase, canales=M.CANALES, rng=None, dt_s=5.0, sigmas=None,
                sesgo_ang_deg=(0.0, 0.0), el_min_deg=3.0):
        return od.medidas_de_pase(self.ef, self.reloj, self.sitios, self.isit(pase['clave']),
                                  pase, dt_s, sigmas, rng, canales, el_min_deg,
                                  sesgo_ang_deg=sesgo_ang_deg)

    def estado_sgp4(self, t):
        r, v = self.sgp4.estado(np.array([float(t)]))
        return np.r_[r[0], v[0]]

    def estado_verdad(self, t):
        r, v = self.ef.estado(np.array([float(t)]))
        return np.r_[r[0], v[0]]

    # ----------------------------------------------------------- estimacion
    def ajustar(self, lista_medidas, canales, prior=True, estima_p=False, sigma_p=0.5,
                sigmas=None, max_iter=12, sigma_pos_km_dia=2.0, sigma_deriva_km_dia=2.0):
        """OD por lotes con una lista de `Medidas` (de uno o varios pases).
        Estado en el instante de la primera medida; estimacion inicial = SGP4
        en ese instante; prior = TLE con la edad que corresponda (o ninguno)."""
        med = lista_medidas[0]
        for m in lista_medidas[1:]:
            med = med.concatenar(m)
        med = med.solo(canales)
        t0 = float(med.t[0])
        x0 = self.estado_sgp4(t0)
        pri = (od.prior_de_tle(x0, t0 / 86400.0, sigma_pos_km_dia, estima_p=estima_p,
                               sigma_p=sigma_p, sigma_deriva_km_dia=sigma_deriva_km_dia)
               if prior else None)
        return od.ajuste_lotes(med, x0, t0, self.modelo_est, sigmas, estima_p, pri,
                               max_iter=max_iter)

    def filtrar(self, lista_medidas, canales, estima_p=False, sigma_p=0.5, sigmas=None,
                q_acc=3e-21, q_p=0.0, sigma_pos_km_dia=2.0, sigma_deriva_km_dia=2.0,
                iter_med=3):
        """EKF sobre las mismas medidas, con el mismo prior que `ajustar`."""
        med = lista_medidas[0]
        for m in lista_medidas[1:]:
            med = med.concatenar(m)
        med = med.solo(canales)
        t0 = float(med.t[0])
        x0 = self.estado_sgp4(t0)
        xb, Pb = od.prior_de_tle(x0, t0 / 86400.0, sigma_pos_km_dia, estima_p=estima_p,
                                 sigma_p=sigma_p, sigma_deriva_km_dia=sigma_deriva_km_dia)
        return od.ekf(med, xb, Pb, t0, self.modelo_est, sigmas, estima_p, q_acc, q_p=q_p,
                      iter_med=iter_med)

    # ----------------------------------------------------------- evaluacion
    def error_pase(self, ef_pred, pase, clave=None):
        """Desapuntamiento (deg) en un pase: dict aos/max/rms/p95/medio."""
        sit = self.sitios[self.isit(clave or pase['clave'])]
        return od.resumen_error_pase(ef_pred, self.ef, self.reloj, sit, pase)

    def efemeride_de(self, res, t_fin, dt_s=30.0):
        return od.propagar_resultado(res, t_fin, dt_s)

    def error_posicion_km(self, res, t=None):
        """|r_estimada - r_verdad| (km) en el instante `t` (por defecto la
        epoca de la solucion) y el desglose (R, T, N)."""
        t = res.t0 if t is None else t
        x = self.estado_verdad(t)
        if t != res.t0:
            ef = self.efemeride_de(res, t + 60.0)
            r, v = ef.estado(np.array([t]))
            xe = np.r_[r[0], v[0]]
        else:
            xe = res.x[:6]
        rh, th, hh = rtn_base(x)
        d = xe[:3] - x[:3]
        return float(np.linalg.norm(d)), np.array([d @ rh, d @ th, d @ hh])


# ---------------------------------------------------------------- seleccion de pases (planificador)

def candidatos_planificador(escenario, claves, t_ini, horas, el_min_deg=20.0):
    """Ventanas candidatas para OD tal como las vera un operador: las del
    PLANIFICADOR (`planificador.construir_ventanas`) sobre el TLE envejecido,
    en segundos del reloj del escenario. Los tiempos son los PREDICHOS con el
    TLE (no los verdaderos): sirven para decidir, no para medir. Cada
    candidato: dict clave, aos, los, el_max_deg, id (de la ventana)."""
    from atp_ros import planificador as PL
    est = {k: escenario.sitios[escenario.isit(k)].est for k in claves}
    t0_utc = escenario.reloj.utc(t_ini)
    vs = PL.construir_ventanas([escenario.tle], est, t0_utc, horas, el_min_deg=el_min_deg,
                               dt_s=10.0)
    out = []
    for v in vs:
        out.append({'clave': v.estacion, 'aos': escenario.reloj.segundos(v.aos),
                    'los': escenario.reloj.segundos(v.los), 'el_max_deg': float(v.el_max_deg),
                    'id': v.id})
    return out


class Linealizacion:
    """Estado y STM de la orbita de referencia `x_lin` (en t0, propagada con
    el modelo del estimador) en los instantes pedidos, calculados con UNA
    integracion. Sirve para evaluar la informacion de muchos pases candidatos
    y la sensibilidad en el pase objetivo sin reintegrar."""

    def __init__(self, esc, t0, x_lin, tiempos):
        self.esc = esc
        self.ts = np.unique(np.concatenate([np.atleast_1d(np.asarray(t, float))
                                            for t in tiempos]))
        f = P.derivadas_stm(esc.modelo_est, False)
        Y = P.propagar(f, P.y0_stm(x_lin), t0, self.ts, **od.INT_STM)
        self.X, self.Phi = Y[:, :6], Y[:, 6:].reshape(len(self.ts), 6, 6)

    def en(self, t):
        i = np.searchsorted(self.ts, np.asarray(t, float))
        return self.X[i], self.Phi[i]


def tiempos_candidato(cand, dt_s=10.0):
    return np.arange(cand['aos'], cand['los'], dt_s)


def info_candidato(lin, cand, canales, sigmas=None, dt_s=10.0, el_min_deg=3.0,
                   el_max_deg=88.0):
    """Informacion (matriz 6x6 sobre el estado en t0) que aportaria un pase
    candidato: sum H^T W H con H = J(h) Phi(t, t0), linealizada sobre la
    orbita de `lin`."""
    esc = lin.esc
    sg = dict(M.SIGMAS_DEFECTO)
    sg.update(sigmas or {})
    t = tiempos_candidato(cand, dt_s)
    X, Phi = lin.en(t)
    sit = esc.sitios[esc.isit(cand['clave'])]
    el = np.degrees(M.observables(esc.reloj, sit, t, X[:, :3], X[:, 3:6])['el'])
    ok = (el >= el_min_deg) & (el <= el_max_deg)
    I = np.zeros((6, 6))
    if not ok.any():
        return I
    J = M.jacobiano(esc.reloj, sit, t[ok], X[ok, :3], X[ok, 3:6], tuple(canales))
    for k, c in enumerate(canales):
        H = np.einsum('mj,mji->mi', J[:, k, :], Phi[ok])
        I += (H.T @ H) / sg[c] ** 2
    return I


def sensibilidad_objetivo(lin, pase_obj, dt_s=20.0):
    """B (m, 2, 6): d(az cos el, el)/d(estado en t0) en m instantes del pase
    objetivo (rad por unidad de estado), sobre la orbita de `lin`."""
    esc = lin.esc
    t = np.arange(pase_obj['aos'], pase_obj['los'], dt_s)
    X, Phi = lin.en(t)
    sit = esc.sitios[esc.isit(pase_obj['clave'])]
    h = M.observables(esc.reloj, sit, t, X[:, :3], X[:, 3:6])
    J = M.jacobiano(esc.reloj, sit, t, X[:, :3], X[:, 3:6], ('az', 'el'))
    B = np.einsum('mij,mjk->mik', J, Phi)
    B[:, 0, :] *= np.cos(h['el'])[:, None]
    return B


def sigma_apuntamiento(B, Pcov):
    """sigma (deg, rms sobre el pase) del desapuntamiento que dejaria una
    orbita con covarianza formal `Pcov` (6x6, en t0): traza de B P B^T,
    promediada. Es FORMAL: no incluye el error de modelo (arrastre), que
    crece con el intervalo entre el dato y el pase."""
    C = np.einsum('mij,jk,mlk->mil', B, Pcov, B)
    tr = np.trace(C, axis1=1, axis2=2)
    return math.degrees(math.sqrt(float(np.mean(tr))))


def seleccionar_pases_od(esc, candidatos, primero, n_extra, pase_obj, canales,
                         sigmas=None, dt_s=10.0):
    """Eleccion golosa de `n_extra` pases (de `candidatos`, posteriores al
    `primero`, ya fijado, y anteriores a `pase_obj`) que MINIMIZAN el
    desapuntamiento formal esperado sobre `pase_obj` (`sigma_apuntamiento`).
    Criterio de informacion de Fisher sobre el modelo linealizado: favorece
    pases de OTRAS estaciones y geometrias distintas (base larga), que es lo
    que el planificador de la red puede ofrecer. Linealiza sobre la orbita
    SGP4 en la epoca del primer pase (el unico conocimiento previo del
    operador) con el prior del TLE. Devuelve ([primero, ...elegidos] en
    orden de tiempo, tabla [(clave, aos, sigma_deg)])."""
    t0 = primero['aos']
    x_lin = esc.estado_sgp4(t0)
    pool = [c for c in candidatos if c is not primero and c['aos'] > primero['los']
            and c['los'] < pase_obj['aos']]
    tiempos = [tiempos_candidato(c, dt_s) for c in [primero] + pool]
    tiempos.append(np.arange(pase_obj['aos'], pase_obj['los'], 20.0))
    lin = Linealizacion(esc, t0, x_lin, tiempos)
    B = sensibilidad_objetivo(lin, pase_obj)
    info = {id(c): info_candidato(lin, c, canales, sigmas, dt_s) for c in [primero] + pool}
    suma = np.linalg.inv(od.prior_de_tle(x_lin, t0 / 86400.0)[1]) + info[id(primero)]
    elegidos, tabla = [primero], []
    for _ in range(n_extra):
        mejor, mejor_s = None, None
        for c in pool:
            s = sigma_apuntamiento(B, np.linalg.inv(suma + info[id(c)]))
            if mejor_s is None or s < mejor_s:
                mejor, mejor_s = c, s
        if mejor is None:
            break
        elegidos.append(mejor)
        suma = suma + info[id(mejor)]
        pool.remove(mejor)
        tabla.append((mejor['clave'], mejor['aos'], mejor_s))
    return sorted(elegidos, key=lambda c: c['aos']), tabla


def pase_verdadero(esc, cand):
    """El pase VERDADERO que corresponde a un candidato del planificador (la
    misma estacion, el que se solapa mas en el tiempo). None si no hay."""
    mejor, mejor_sol = None, 0.0
    for p in esc.pases(cand['clave'], cand['aos'] - 600.0, cand['los'] + 600.0):
        sol = min(p['los'], cand['los']) - max(p['aos'], cand['aos'])
        if sol > mejor_sol:
            mejor, mejor_sol = p, sol
    return mejor


# ---------------------------------------------------------------- consistencia (Monte Carlo)

def monte_carlo_nees(esc, pases, canales, n=40, semilla=1, modelo_verdad=None,
                     estima_p=False, dt_s=5.0, sigma_p=0.5):
    """NEES de la OD con Monte Carlo sobre uno o varios pases (lista de dicts
    de `Escenario.pases`, con `clave`). En cada corrida: el estado verdadero
    en el primer instante se sortea del PRIOR del TLE (`prior_de_tle`; con
    `estima_p` el factor de arrastre verdadero se sortea de N(1, sigma_p)),
    se propaga con `modelo_verdad` (por defecto el del estimador: modelo
    EXACTO) y se miden los pases con ruido de las sigmas por defecto. Se
    ajusta por lotes y con el EKF, ambos con el mismo prior.

    Devuelve dict con arrays `lotes` y `ekf` (NEES de (r, v) en la epoca de
    cada estimador, 6 grados de libertad) y su media. Con el modelo exacto la
    media debe valer 6 (consistente); con un modelo de verdad distinto y
    arcos de horas, la media se dispara: la covarianza formal solo cubre
    el ruido de medicion, no el error de modelo."""
    rng = np.random.default_rng(semilla)
    modelo_verdad = modelo_verdad or esc.modelo_est
    ts_pase = [np.arange(p['aos'], p['los'], dt_s) for p in pases]
    t0 = float(ts_pase[0][0])
    xb, Pb = od.prior_de_tle(esc.estado_sgp4(t0), t0 / 86400.0, estima_p=estima_p,
                             sigma_p=sigma_p)
    L = np.linalg.cholesky(Pb)
    todos = np.unique(np.concatenate(ts_pase))
    nl, nk = [], []
    for _ in range(int(n)):
        xv = xb + L @ rng.standard_normal(len(xb))
        mv = modelo_verdad
        if estima_p:       # el factor verdadero escala el arrastre verdadero
            mv = modelo_verdad.con(cd_a_m=modelo_verdad.cd_a_m * xv[6])
        Y = P.propagar(P.derivadas(mv), xv[:6], t0, todos, metodo='rk4', h=15.0)
        meds = []
        for p, t in zip(pases, ts_pase):
            i = np.searchsorted(todos, t)
            meds.append(M.generar(esc.reloj, esc.sitios, esc.isit(p['clave']), t,
                                  Y[i, :3], Y[i, 3:6], rng=rng, canales=canales))
        med = meds[0]
        for m in meds[1:]:
            med = med.concatenar(m)
        res = od.ajuste_lotes(med, xb, t0, esc.modelo_est, None, estima_p, (xb, Pb))
        nl.append(od.nees(res.x[:6] - xv[:6], res.P[:6, :6]))
        kf = od.ekf(med, xb, Pb, t0, esc.modelo_est, None, estima_p)
        i_fin = np.searchsorted(todos, kf.t)       # verdad en el ultimo instante del filtro
        nk.append(od.nees(kf.x[:6] - Y[i_fin, :6], kf.P[:6, :6]))
    return {'lotes': np.array(nl), 'ekf': np.array(nk),
            'media_lotes': float(np.mean(nl)), 'media_ekf': float(np.mean(nk)),
            'limites_95': od.chi2_limites(6, int(n), 0.95)}
