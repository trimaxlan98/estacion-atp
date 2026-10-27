"""Navegacion del rover (R2), Python puro sin ROS. CONTRATO sec. 18, docs/ROVER.md.

Interfaz `Terreno` (duck-type): `altura(x, y) -> m` y `normal(x, y) -> (nx, ny, nz)` unitaria, z arriba.
Estado: el JSON de `/rover/estado` como dict (x, y, rumbo_deg, v, w, deslizamiento, ...).
Todo determinista: sin azar ni reloj; solo depende de las entradas.
"""
import heapq
import math
from dataclasses import dataclass


@dataclass
class ParamsNav:
    v_max: float = 1.0            # m/s en llano liso
    v_min: float = 0.1            # m/s: piso mientras haya ruta
    w_max: float = 0.8            # rad/s
    lookahead: float = 1.5        # m
    tol_meta: float = 0.3         # m
    pend_max_deg: float = 20.0    # >= esto es intransitable (A*) y v -> v_min
    pend_blando_deg: float = 5.0  # desde aqui empieza a bajar la velocidad
    rug_max: float = 0.15         # m (desv. tipica de altura local) que da v_min / intransitable
    rug_radio: float = 0.5        # m
    k_pend: float = 3.0           # peso de la pendiente en el costo del A*
    k_rug: float = 5.0            # peso de la rugosidad en el costo del A*
    res: float = 0.5              # m, celda del A*
    margen: float = 8.0           # m de holgura del dominio del A* alrededor de inicio-meta
    giro_sitio_deg: float = 80.0  # si el objetivo queda a mas de esto, gira en sitio (R4: el lazo usa 45)
    # --- R6. Todo apagado por defecto: con estos valores el Navegador se comporta como el de R2/R4.
    frenar_pend: bool = True      # R6: False = la pendiente no baja la velocidad (con control de rueda proporcional, ir mas lento en cuesta da menos par)
    graduado: bool = False        # costo graduado: pendiente/rugosidad por encima del limite CUESTAN mas, no prohiben
    pend_dura_deg: float = 35.0   # en modo graduado, esto o mas si es intransitable (el rover no puede subirlo)
    k_dura: float = 6.0           # costo extra por exceso: k_dura * (exceso / limite) (pendiente y rugosidad)
    k_lat: float = 1.5            # peso de la pendiente LATERAL (de costado) en el costo graduado
    replan_s: float = 0.0         # si > 0, replanifica cada tanto (tiempo del estado) y al desviarse
    desvio_replan_m: float = 2.0  # distancia a la ruta que obliga a replanificar
    ki_vel: float = 0.0           # PI de velocidad sobre la v medida: orden = v_deseada + kp e + integral(ki e)
    kp_vel: float = 0.0
    boost_max: float = 0.0        # m/s: tope del termino integral (el rover real tiene un limite de par)
    ff_desliz: bool = False       # prealimentacion: orden = v_deseada / (1 - deslizamiento medio medido)
    lat_umbral_deg: float = 4.0   # pendiente de costado a partir de la cual se cuida el agarre lateral: sin prealimentacion y con umbral_desliz_lat
    umbral_desliz_lat: float = 0.5  # umbral del antideslizamiento en esa situacion (con alto deslizamiento el circulo de friccion deja sin agarre lateral)
    s_ff_max: float = 0.65        # tope del deslizamiento usado en la prealimentacion (evita 1/(1-s) -> infinito)
    s_giro: float = 1.5           # deslizamiento medio a partir del cual se suelta la orden extra (1.5 = nunca: en este modelo mas orden siempre da mas empuje)
    cab_exceso_deg: float = 8.0   # cabeceo sobre el del terreno a partir del cual se considera encabritado (poca gravedad)
    cmd_extra_max: float = 0.9    # m/s: la orden nunca pasa de v_deseada + esto
    acc_max: float = 0.0          # m/s2: limite de aceleracion de la orden (0 = sin limite)
    vaiven_max: int = 1000        # maniobras de vaiven por meta antes de declararla inalcanzable
    vaiven: bool = False          # maniobra de vaiven (atras-adelante) al detectar atasco
    vaiven_v: float = 0.5         # m/s de la maniobra
    vaiven_atras_s: float = 2.5
    vaiven_adelante_s: float = 3.0
    vaiven_ciclos: int = 2        # ciclos atras/adelante antes de replanificar evitando el sitio
    evitar_radio_m: float = 2.0   # tras un vaiven fallido, ese sitio suma costo en la siguiente ruta
    evitar_costo: float = 8.0
    # --- R10 (apagado por defecto: con 0.0 el Navegador es el de R6, mismos bytes).
    pendiente_huella_m: float = 0.0   # > 0: la alarma `pendiente` usa la normal MEDIA de una rejilla 3x3 de este paso (m) y no la del punto bajo el CM


def pendiente_deg(terr, x, y):
    nz = max(-1.0, min(1.0, terr.normal(x, y)[2]))
    return math.degrees(math.acos(nz))


def rugosidad(terr, x, y, r=0.5):
    """Rugosidad local (m): rms de h(p)+h(-p)-2h(0) sobre 4 pares opuestos de un anillo de radio r, /2.
    La combinacion elimina el plano medio, asi que una rampa lisa tiene rugosidad 0."""
    h0 = terr.altura(x, y)
    acc = 0.0
    for k in range(4):
        a = k * math.pi / 4
        dx, dy = r * math.cos(a), r * math.sin(a)
        e = terr.altura(x + dx, y + dy) + terr.altura(x - dx, y - dy) - 2.0 * h0
        acc += e * e
    return math.sqrt(acc / 4.0) / 2.0


def limite_velocidad(terr, x, y, p):
    """Velocidad maxima permitida en (x, y) por pendiente y rugosidad."""
    pend = pendiente_deg(terr, x, y)
    rug = rugosidad(terr, x, y, p.rug_radio)
    f_p = 1.0 - (pend - p.pend_blando_deg) / max(1e-9, p.pend_max_deg - p.pend_blando_deg)
    f_p = max(0.0, min(1.0, f_p))
    if not p.frenar_pend:
        f_p = 1.0
    f_r = max(0.0, min(1.0, 1.0 - rug / p.rug_max))
    return max(p.v_min, p.v_max * min(f_p, f_r))


def _ang(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class PurePursuit:
    """Sigue una polilinea con pure pursuit; v segun terreno (si hay) y bajando cerca de la meta."""

    def __init__(self, ruta, params=None, terreno=None):
        self.p = params or ParamsNav()
        self.ruta = [tuple(q) for q in ruta]
        self.terreno = terreno
        self.i = 0          # segmento actual (monotono)
        self.llegado = False

    def _punto_lookahead(self, x, y):
        r, L = self.ruta, self.p.lookahead
        while self.i < len(r) - 2:
            ax, ay = r[self.i]
            bx, by = r[self.i + 1]
            dx, dy = bx - ax, by - ay
            n2 = dx * dx + dy * dy
            t = ((x - ax) * dx + (y - ay) * dy) / n2 if n2 > 0 else 1.0
            if t >= 1.0:
                self.i += 1
            else:
                break
        for j in range(self.i, len(r) - 1):
            ax, ay = r[j]
            bx, by = r[j + 1]
            dx, dy = bx - ax, by - ay
            fx, fy = ax - x, ay - y
            a = dx * dx + dy * dy
            if a == 0:
                continue
            b = 2 * (fx * dx + fy * dy)
            c = fx * fx + fy * fy - L * L
            disc = b * b - 4 * a * c
            if disc < 0:
                continue
            t = (-b + math.sqrt(disc)) / (2 * a)
            if 0.0 <= t <= 1.0:
                return (ax + t * dx, ay + t * dy)
        return r[-1]

    def comando(self, estado):
        """-> (v, w). Marca `llegado` al entrar en tol_meta de la ultima meta."""
        p = self.p
        x, y = estado['x'], estado['y']
        gx, gy = self.ruta[-1]
        dist = math.hypot(gx - x, gy - y)
        if dist <= p.tol_meta or self.llegado:
            self.llegado = True
            return 0.0, 0.0
        tx, ty = self._punto_lookahead(x, y)
        rumbo = math.radians(estado.get('rumbo_deg', 0.0))
        alfa = _ang(math.atan2(ty - y, tx - x) - rumbo)
        vlim = p.v_max if self.terreno is None else limite_velocidad(self.terreno, x, y, p)
        vlim = min(vlim, max(p.v_min, 0.8 * dist))
        if abs(alfa) > math.radians(p.giro_sitio_deg):          # objetivo detras: girar en sitio
            return 0.0, math.copysign(p.w_max, alfa)
        v = vlim * max(0.2, math.cos(alfa))
        L = max(1e-6, math.hypot(tx - x, ty - y))
        w = max(-p.w_max, min(p.w_max, v * 2.0 * math.sin(alfa) / L))
        return v, w


def planificar_ruta(terr, inicio, meta, params=None, evitar=(), lim_sube=None):
    """A* 8-conexo sobre rejilla de costo. Costo de celda = 1 + k_pend*pend/pend_max + k_rug*rug/rug_max;
    celdas con pendiente >= pend_max_deg o rugosidad >= rug_max son intransitables.
    -> lista de (x, y) de inicio a meta (con atajos libres) o None si no hay ruta.
    R6: con `params.graduado` se usa `planificar_ruta_graduada` (costo direccional; solo lo extremo se prohibe)."""
    p = params or ParamsNav()
    if p.graduado:
        return planificar_ruta_graduada(terr, inicio, meta, p, evitar, lim_sube)
    res = p.res
    x0 = min(inicio[0], meta[0]) - p.margen
    y0 = min(inicio[1], meta[1]) - p.margen
    nx = int(math.ceil((abs(inicio[0] - meta[0]) + 2 * p.margen) / res)) + 1
    ny = int(math.ceil((abs(inicio[1] - meta[1]) + 2 * p.margen) / res)) + 1

    def celda(q):
        return (int(round((q[0] - x0) / res)), int(round((q[1] - y0) / res)))

    def centro(c):
        return (x0 + c[0] * res, y0 + c[1] * res)

    cache = {}

    def costo(c):
        if c not in cache:
            x, y = centro(c)
            pend = pendiente_deg(terr, x, y)
            rug = rugosidad(terr, x, y, p.rug_radio)
            if pend >= p.pend_max_deg or rug >= p.rug_max:
                cache[c] = None
            else:
                cache[c] = 1.0 + p.k_pend * pend / p.pend_max_deg + p.k_rug * rug / p.rug_max
        return cache[c]

    s, g = celda(inicio), celda(meta)
    if costo(g) is None:
        return None                      # la meta misma es intransitable
    if costo(s) is None:
        cache[s] = 1.0                   # donde ya estamos, se puede salir
    vec = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    abierto = [(0.0, 0.0, s)]
    gcost = {s: 0.0}
    padre = {}
    cerrado = set()
    hallado = False
    while abierto:
        _, gc, c = heapq.heappop(abierto)
        if c in cerrado:
            continue
        if c == g:
            hallado = True
            break
        cerrado.add(c)
        for d in vec:
            n = (c[0] + d[0], c[1] + d[1])
            if not (0 <= n[0] < nx and 0 <= n[1] < ny) or n in cerrado:
                continue
            cn = costo(n)
            if cn is None:
                continue
            ng = gc + res * math.hypot(*d) * 0.5 * (cn + costo(c))
            if ng < gcost.get(n, 1e18):
                gcost[n] = ng
                padre[n] = c
                h = res * math.hypot(g[0] - n[0], g[1] - n[1])
                heapq.heappush(abierto, (ng + h, ng, n))
    if not hallado:
        return None
    cam = [g]
    while cam[-1] != s:
        cam.append(padre[cam[-1]])
    cam.reverse()
    pts = [tuple(inicio)] + [centro(c) for c in cam[1:-1]] + [tuple(meta)]

    def libre(a, b):
        n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / (res * 0.5)))
        for k in range(n + 1):
            t = k / n
            if costo(celda((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))) is None:
                return False
        return True

    out = [pts[0]]
    i = 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1 and not libre(pts[i], pts[j]):
            j -= 1
        out.append(pts[j])
        i = j
    return out


def planificar_ruta_graduada(terr, inicio, meta, p, evitar=(), lim_sube=None):
    """A* 8-conexo con costo GRADUADO y DIRECCIONAL (R6). Nada se prohibe salvo lo extremo (pendiente local >= `pend_dura_deg` o
    rugosidad >= 3 `rug_max`), de modo que una meta en una cuesta dificil ya no da `sin_ruta`: la ruta pasa por donde cuesta menos.
    Por arista de longitud L y direccion unitaria u, con g = gradiente del terreno (promedio de las dos celdas):
        sube = atan(g.u) si > 0, baja = atan(-g.u) si > 0, lat = atan(|g x u|)
        costo = L (1 + k_pend sube/lim + k_lat lat/lim + 0.5 baja/lim + k_rug rug/rug_max + k_dura exceso)
    con lim = pend_max_deg y exceso = (sube - lim)+/lim + (lat - lim)+/lim + (rug - rug_max)+/rug_max; `evitar` = [(x, y, r, costo)]
    suma costo por metro cerca de sitios donde el rover ya se atasco. Los atajos solo se aceptan si no cuestan mas de un 8 % que el
    tramo original. -> lista de (x, y) o None (solo si la meta o el cerco son extremos)."""
    res = p.res
    lim = p.pend_max_deg if lim_sube is None else lim_sube      # limite de subida (puede ser el aprendido)
    x0 = min(inicio[0], meta[0]) - p.margen
    y0 = min(inicio[1], meta[1]) - p.margen
    nx = int(math.ceil((abs(inicio[0] - meta[0]) + 2 * p.margen) / res)) + 1
    ny = int(math.ceil((abs(inicio[1] - meta[1]) + 2 * p.margen) / res)) + 1

    def celda(q):
        return (int(round((q[0] - x0) / res)), int(round((q[1] - y0) / res)))

    def centro(c):
        return (x0 + c[0] * res, y0 + c[1] * res)

    cache = {}

    def datos(c):
        if c not in cache:
            x, y = centro(c)
            nxx, nyy, nzz = terr.normal(x, y)
            nzz = max(1e-6, nzz)
            pend = math.degrees(math.acos(max(-1.0, min(1.0, nzz))))
            rug = rugosidad(terr, x, y, p.rug_radio)
            if pend >= p.pend_dura_deg or rug >= 3.0 * p.rug_max:
                cache[c] = None
            else:
                extra = sum(ce for ex_, ey_, er_, ce in evitar if math.hypot(x - ex_, y - ey_) < er_)
                cache[c] = (-nxx / nzz, -nyy / nzz, rug, extra)
        return cache[c]

    def arista(dc, dn, ux, uy, L):
        gx = 0.5 * (dc[0] + dn[0])
        gy = 0.5 * (dc[1] + dn[1])
        along = gx * ux + gy * uy
        cross = abs(gx * uy - gy * ux)
        sube = math.degrees(math.atan(max(0.0, along)))
        baja = math.degrees(math.atan(max(0.0, -along)))
        lat = math.degrees(math.atan(cross))
        rug = 0.5 * (dc[2] + dn[2])
        ex = max(0.0, sube - lim) / lim + max(0.0, lat - lim) / lim + max(0.0, rug - p.rug_max) / p.rug_max
        c = 1.0 + p.k_pend * sube / lim + p.k_lat * lat / lim + 0.5 * baja / lim + p.k_rug * rug / p.rug_max + p.k_dura * ex
        return L * (c + 0.5 * (dc[3] + dn[3]))

    s, g = celda(inicio), celda(meta)
    if datos(g) is None:
        return None
    if datos(s) is None:
        cache[s] = (0.0, 0.0, 0.0, 0.0)
    vec = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
    abierto = [(0.0, 0.0, s)]
    gcost = {s: 0.0}
    padre = {}
    cerrado = set()
    hallado = False
    while abierto:
        _, gc, c = heapq.heappop(abierto)
        if c in cerrado:
            continue
        if c == g:
            hallado = True
            break
        cerrado.add(c)
        dc = datos(c)
        for d in vec:
            n = (c[0] + d[0], c[1] + d[1])
            if not (0 <= n[0] < nx and 0 <= n[1] < ny) or n in cerrado:
                continue
            dn = datos(n)
            if dn is None:
                continue
            ln = math.hypot(*d)
            ng = gc + arista(dc, dn, d[0] / ln, d[1] / ln, res * ln)
            if ng < gcost.get(n, 1e18):
                gcost[n] = ng
                padre[n] = c
                h = res * math.hypot(g[0] - n[0], g[1] - n[1])
                heapq.heappush(abierto, (ng + h, ng, n))
    if not hallado:
        return None
    cam = [g]
    while cam[-1] != s:
        cam.append(padre[cam[-1]])
    cam.reverse()
    pts = [tuple(inicio)] + [centro(c) for c in cam[1:-1]] + [tuple(meta)]
    acum = [gcost[c] for c in cam]            # costo acumulado hasta cada celda; pts[k] <-> cam[k]

    def costo_recta(a, b):
        dist = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, int(dist / (res * 0.5)))
        ux, uy = (b[0] - a[0]) / max(dist, 1e-9), (b[1] - a[1]) / max(dist, 1e-9)
        tot = 0.0
        prev = datos(celda(a))
        for k in range(1, n + 1):
            t = k / n
            cur = datos(celda((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))))
            if cur is None or prev is None:
                return None
            tot += arista(prev, cur, ux, uy, dist / n)
            prev = cur
        return tot

    out = [pts[0]]
    i = 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1:
            cr = costo_recta(pts[i], pts[j])
            if cr is not None and cr <= 1.08 * (acum[j] - acum[i]) + 1e-9:
                break
            j -= 1
        out.append(pts[j])
        i = j
    return out


@dataclass
class ParamsTraccion:
    umbral_desliz: float = 0.3    # |deslizamiento| por rueda a partir del cual se reduce el par
    ganancia: float = 2.0         # factor objetivo = 1 - ganancia*exceso
    factor_min: float = 0.2
    recuperacion: float = 0.05    # subida por paso cuando ya no hay exceso


class ControlTraccion:
    """Anti-deslizamiento: factor global en [factor_min, 1] que baja al exceder el umbral cualquier rueda y
    se recupera poco a poco. `pares` aplica ademas un recorte extra a la rueda que patina."""

    def __init__(self, params=None):
        self.p = params or ParamsTraccion()
        self.factor = 1.0

    def actualizar(self, desliz):
        p = self.p
        exceso = max((abs(s) for s in desliz), default=0.0) - p.umbral_desliz
        if exceso > 0:
            self.factor = min(self.factor, max(p.factor_min, 1.0 - p.ganancia * exceso))
        else:
            self.factor = min(1.0, self.factor + p.recuperacion)
        return self.factor

    def pares(self, pares_nm, desliz):
        f = self.actualizar(desliz)
        return [t * (max(self.p.factor_min, f * 0.5) if abs(s) > self.p.umbral_desliz else f)
                for t, s in zip(pares_nm, desliz)]


class Navegador:
    """Nucleo completo sin ROS: meta -> ruta (A*) -> pure pursuit -> traccion -> seguridad.
    `paso(estado)` devuelve ((v, w), [alarmas]). Ante vuelco/pendiente/energia ordena parar (0, 0).
    R6 (opcional, ver ParamsNav): costo graduado, replanificacion, PI de velocidad y vaiven ante atasco."""

    def __init__(self, terreno=None, params=None, seguridad=None, traccion=None):
        from atp_rover.seguridad import MonitorSeguridad
        self.terreno = terreno
        self.p = params or ParamsNav()
        self.seg = seguridad or MonitorSeguridad()
        self.trac = traccion or ControlTraccion()
        self._umbral_plano = self.trac.p.umbral_desliz
        self.pp = None
        self.ruta = None
        self.meta = None
        self.sin_ruta = False
        self.inalcanzable = False
        self._v_cmd = 0.0
        self._t_enc = -1.0
        self._v_orden = 0.0
        self._boost = 0.0
        self._sl_f = 0.0
        self._t_prev = None
        self._t_plan = 0.0
        self._evitar = []
        self._lim_sube = None        # limite de subida APRENDIDO (grados): baja al atascarse en una cuesta
        self._vaiven = None          # None o dict con la fase de la maniobra
        self.vaivenes = 0            # maniobras iniciadas
        self.replanes = 0

    @property
    def llegado(self):
        return self.pp is not None and self.pp.llegado

    def _planificar(self, estado, meta):
        ini = (estado['x'], estado['y'])
        if self.terreno is None:
            ruta = [ini, tuple(meta)]
        else:
            ruta = planificar_ruta(self.terreno, ini, meta, self.p, self._evitar, self._lim_sube)
        return ruta

    def nueva_meta(self, estado, meta):
        self.sin_ruta = False
        self.inalcanzable = False
        self.vaivenes_meta = 0
        self.meta = tuple(meta)
        self._evitar = []
        self._lim_sube = None
        self._vaiven = None
        self._boost = 0.0
        self._t_plan = estado.get('t', 0.0)
        self.ruta = self._planificar(estado, meta)
        if self.ruta is None:
            self.sin_ruta = True
            self.pp = None
        else:
            self.pp = PurePursuit(self.ruta, self.p, self.terreno)

    def _replanificar(self, estado):
        """Nueva ruta desde donde estamos; si no la hay, se conserva la anterior."""
        self._t_plan = estado.get('t', 0.0)
        ruta = self._planificar(estado, self.meta)
        if ruta is not None:
            self.ruta = ruta
            self.pp = PurePursuit(ruta, self.p, self.terreno)
            self.replanes += 1

    def _aprender_limite(self, estado):
        """Atascado subiendo una cuesta de `a` grados: el rover no puede con eso en este suelo; el planificador baja su limite de subida
        a `a - 2` grados (piso 5) y, si hay otra ruta, la toma. No se vuelve a subir en la misma meta (conservador)."""
        if self.terreno is None:
            return
        nxx, nyy, nzz = self.terreno.normal(estado['x'], estado['y'])
        r = math.radians(estado.get('rumbo_deg', 0.0))
        sube = math.degrees(math.atan((-nxx * math.cos(r) - nyy * math.sin(r)) / max(1e-6, nzz)))
        if sube > 4.0:
            lim = max(5.0, sube - 2.0)
            self._lim_sube = lim if self._lim_sube is None else min(self._lim_sube, lim)

    def _pendiente_huella(self, estado):
        """Pendiente (grados) de la normal media de una rejilla 3x3 de paso `pendiente_huella_m` centrada en el rover. La normal puntual de un
        heightmap con rocas de borde vertical da > 25 grados en el borde de una roca de 20 cm aunque el rover la pase sin problema; la media sobre
        la huella (~ el ancho de las ruedas) filtra eso y conserva las paredes largas (cráter, cuestas)."""
        h = self.p.pendiente_huella_m
        sx = sy = sz = 0.0
        for i in (-1, 0, 1):
            for j in (-1, 0, 1):
                nxx, nyy, nzz = self.terreno.normal(estado['x'] + i * h, estado['y'] + j * h)
                sx += float(nxx)
                sy += float(nyy)
                sz += float(nzz)
        n = math.sqrt(sx * sx + sy * sy + sz * sz)
        return math.degrees(math.acos(max(-1.0, min(1.0, sz / max(n, 1e-9)))))

    def _desvio(self, estado):
        r = self.ruta
        x, y = estado['x'], estado['y']
        mejor = 1e18
        for (ax, ay), (bx, by) in zip(r, r[1:]):
            dx, dy = bx - ax, by - ay
            n2 = dx * dx + dy * dy
            t = 0.0 if n2 == 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / n2))
            mejor = min(mejor, math.hypot(x - (ax + t * dx), y - (ay + t * dy)))
        return mejor

    def _orden_vaiven(self, estado):
        """Maniobra de vaiven: atras `vaiven_atras_s`, adelante `vaiven_adelante_s`, `vaiven_ciclos` veces. -> (v, w) o None si acabo."""
        p, vb = self.p, self._vaiven
        t = estado.get('t', 0.0)
        dt = t - vb['t0']
        ciclo = p.vaiven_atras_s + p.vaiven_adelante_s
        if dt >= ciclo * p.vaiven_ciclos:
            return None
        f = dt % ciclo
        return (-p.vaiven_v, 0.0) if f < p.vaiven_atras_s else (p.vaiven_v, 0.0)

    def _exceso_cabeceo(self, estado):
        """Cabeceo del rover menos el de la cuesta bajo el: > 0 es morro arriba MAS que el terreno (encabritado)."""
        cab = estado.get('cabeceo_deg')
        if cab is None:
            return 0.0
        pit = 0.0
        if self.terreno is not None:
            nxx, nyy, nzz = self.terreno.normal(estado['x'], estado['y'])
            r = math.radians(estado.get('rumbo_deg', 0.0))
            pit = math.degrees(math.atan((-nxx * math.cos(r) - nyy * math.sin(r)) / max(1e-6, nzz)))
        return cab - pit

    def _lateral_deg(self, estado):
        """Pendiente de costado (grados) bajo el rover: componente del gradiente perpendicular al rumbo."""
        if self.terreno is None:
            return 0.0
        nxx, nyy, nzz = self.terreno.normal(estado['x'], estado['y'])
        r = math.radians(estado.get('rumbo_deg', 0.0))
        return math.degrees(math.atan(abs(-nxx * -math.sin(r) + -nyy * math.cos(r)) / max(1e-6, nzz)))

    def _compensar(self, estado, v_des, dt, t):
        """Orden de velocidad (m/s) para lograr `v_des` MEDIDA: prealimentacion por deslizamiento + PI sobre la v medida.
        En este modelo el deslizamiento lo fija la fuerza que pide la cuesta, no la orden: v = (1 - s) * v_rueda. Se suelta (no se sigue
        subiendo la orden) si las ruedas ya giran en falso (s >= s_giro) o si el rover se encabrita (cabeceo mayor que el del terreno),
        que es lo que pasa con poca gravedad."""
        p = self.p
        sl = estado.get('deslizamiento') or [0.0]
        self._sl_f += 0.2 * (max(0.0, sum(sl) / len(sl)) - self._sl_f)          # media de ruedas, filtrada (~0.5 s)
        if self._exceso_cabeceo(estado) > p.cab_exceso_deg:
            self._t_enc = t + 2.0
        enc = t < self._t_enc
        de_costado = self._lateral_deg(estado) > p.lat_umbral_deg
        self.trac.p.umbral_desliz = p.umbral_desliz_lat if de_costado else self._umbral_plano
        suelta = enc or self._sl_f >= p.s_giro
        e = v_des - estado.get('v', 0.0)
        if suelta:
            self._boost *= 0.8
        else:
            self._boost = max(0.0, min(p.boost_max, self._boost + p.ki_vel * e * dt))
        cmd = v_des
        if p.ff_desliz and not suelta and not de_costado and e > 0.0:       # solo mientras falta velocidad (si no, el transitorio de arranque sobrepasa)
            cmd = v_des / max(0.3, 1.0 - min(self._sl_f, p.s_ff_max))
        if not enc:
            cmd += p.kp_vel * e + self._boost
        cmd = min(cmd, v_des + p.cmd_extra_max)
        if enc:
            cmd = min(cmd, v_des)
        return max(0.0, cmd)

    def _salida(self, v, w, dt, alarmas):
        """Limita la aceleracion de la orden (acc_max): en poca gravedad un escalon de orden voltea al rover por el par de reaccion."""
        if self.p.acc_max > 0:
            paso_v = self.p.acc_max * dt
            v = max(self._v_cmd - paso_v, min(self._v_cmd + paso_v, v))
        self._v_cmd = v
        self._v_orden = v
        return (v, w), alarmas

    def paso(self, estado):
        p = self.p
        t = estado.get('t', 0.0)
        dt = 0.1 if self._t_prev is None else max(0.0, min(1.0, t - self._t_prev))
        self._t_prev = t
        if p.pendiente_huella_m > 0 and self.terreno is not None:
            estado = {**estado, 'pendiente_deg': self._pendiente_huella(estado)}
        alarmas = self.seg.evaluar(estado, self._v_orden)
        v = w = 0.0
        if self.pp is not None and not self.seg.debe_frenar():
            if self._vaiven is not None:
                ov = self._orden_vaiven(estado)
                if ov is None:                       # fin de la maniobra: replanificar evitando este sitio si no se libero
                    vb = self._vaiven
                    self._vaiven = None
                    self.seg._t_atasco = None
                    if math.hypot(estado['x'] - vb['x'], estado['y'] - vb['y']) < 0.5:
                        self._evitar.append((vb['x'], vb['y'], p.evitar_radio_m, p.evitar_costo))
                    self._replanificar(estado)
                else:
                    return self._salida(ov[0], ov[1], dt, alarmas)
            if p.vaiven and 'atasco' in self.seg.activas and not self.pp.llegado:
                if self.vaivenes_meta >= p.vaiven_max:      # ya se intento: la meta no se puede alcanzar con este rover y este suelo
                    self.inalcanzable = True
                    self.pp = None
                    return self._salida(0.0, 0.0, dt, alarmas)
                self._vaiven = {'t0': t, 'x': estado['x'], 'y': estado['y']}
                self.vaivenes += 1
                self.vaivenes_meta += 1
                self._aprender_limite(estado)
                self._boost = 0.0
                return self._salida(-p.vaiven_v, 0.0, dt, alarmas)
            if p.replan_s > 0 and self.meta is not None and not self.pp.llegado:
                if t - self._t_plan >= p.replan_s or self._desvio(estado) > p.desvio_replan_m:
                    self._replanificar(estado)
            v, w = self.pp.comando(estado)
            if p.ki_vel > 0 or p.kp_vel > 0 or p.ff_desliz:
                if v > 1e-6:
                    v = self._compensar(estado, v, dt, t)
                else:
                    self._boost = 0.0
            v *= self.trac.actualizar(estado.get('deslizamiento', []))
        else:
            self._v_cmd = 0.0
        return self._salida(v, w, dt, alarmas)
