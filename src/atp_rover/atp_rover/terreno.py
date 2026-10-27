"""Terreno por heightmap determinista (CONTRATO sec. 18): mismo `tipo` + `semilla` + parametros = mismos bytes.

Rejilla de `res` = 0.1 m sobre un cuadrado de `extension` m centrado en el origen. `altura`, `normal` y `suelo`
aceptan escalares o arreglos (interpolacion bilinear; fuera de la rejilla se repite el borde).

Tipos: `llano`, `rocoso` (ruido fractal + rocas gaussianas), `dunas` (crestas sinusoidales + rizado),
`crater` (cuenco parabolico con borde, centro en (cx, cy)), `pendiente` (plano inclinado `angulo_deg` hacia +x; con
`direccion_deg` el plano sube hacia ese rumbo: 90 = hacia +y, es decir, una pendiente LATERAL para un rover que avanza por +x).
R6: `rocas` (rocas discretas con la distribucion de tamanos de Golombek y Rapp 1997 para Marte; ver `campo_de_rocas`) y
`escalones` (gradas de `altura` m con borde vertical, para medir la altura maxima que supera la rueda).
Todo el azar sale de `numpy.random.default_rng(semilla)` (PCG64) y no se usa tiempo ni estado global.

Suelo por zona: la rejilla `zona` marca 0 = suelo base del planeta, 1 = suelto (arena de duna, fondo de crater),
2 = roca. Los multiplicadores de cada zona son SUPUESTOS de modelado, no medidos.
"""
import numpy as np

# Multiplicadores sobre el suelo base: (kphi, c, delta_phi_deg, K)  -- SUPUESTOS de modelado
ZONAS = {
    0: (1.0, 1.0, 0.0, 1.0),
    1: (0.7, 0.3, -4.0, 1.3),      # suelto: mas blando, menos cohesion, menos friccion, mas deformacion
    2: (10.0, 5.0, 8.0, 0.4),      # roca: casi rigida, rugosa
}
_ZT = np.array([[ZONAS[k][c] for k in range(3)] for c in range(4)])
TIPOS = ('llano', 'rocoso', 'dunas', 'crater', 'pendiente', 'rocas', 'escalones')


def _suave(f):
    return f * f * (3.0 - 2.0 * f)


def _ruido_valor(rng, x, y, paso):
    """Ruido de valores (lattice aleatorio, interpolacion smoothstep) en la rejilla x (N,), y (N,) -> (N, N)."""
    nx = int(np.ceil((x[-1] - x[0]) / paso)) + 3
    ny = int(np.ceil((y[-1] - y[0]) / paso)) + 3
    red = rng.uniform(-1.0, 1.0, size=(ny, nx))
    u = (x - x[0]) / paso
    v = (y - y[0]) / paso
    i0 = np.floor(u).astype(int)
    j0 = np.floor(v).astype(int)
    fu = _suave(u - i0)[None, :]
    fv = _suave(v - j0)[:, None]
    a = red[np.ix_(j0, i0)]
    b = red[np.ix_(j0, i0 + 1)]
    c = red[np.ix_(j0 + 1, i0)]
    d = red[np.ix_(j0 + 1, i0 + 1)]
    return (a * (1 - fu) + b * fu) * (1 - fv) + (c * (1 - fu) + d * fu) * fv


def _fractal(rng, x, y, escala, octavas):
    z = np.zeros((len(y), len(x)))
    amp, norma = 1.0, 0.0
    for o in range(octavas):
        z += amp * _ruido_valor(rng, x, y, escala / (2 ** o))
        norma += amp
        amp *= 0.5
    return z / norma


def distribucion_rocas(k=0.10, d_min=0.15, d_max=0.80, n=400):
    """Distribucion de tamanos de rocas tipo Marte (Golombek y Rapp 1997, citada de memoria, sin recontrastar contra el articulo):
    la fraccion de area cubierta por rocas de diametro >= D es F(D) = k exp(-q D), con q = 1.79 + 0.152 / k (D en m, k = abundancia
    total, 0.05 a 0.20 en los sitios de aterrizaje de Viking, Pathfinder y los MER). La fraccion de area entre D y D + dD es
    k q exp(-q D) dD y una roca de diametro D ocupa pi D^2 / 4, asi que su densidad numerica es n(D) = k q exp(-q D) / (pi D^2 / 4).
    -> (D (n,), dens (n,) rocas por m2 y por m, total rocas por m2 entre d_min y d_max)."""
    q = 1.79 + 0.152 / k
    D = np.linspace(d_min, d_max, n)
    dens = k * q * np.exp(-q * D) / (np.pi * D * D / 4.0)
    total = float(np.sum(0.5 * (dens[1:] + dens[:-1]) * np.diff(D)))
    return D, dens, total


def campo_de_rocas(rng, z, zona, X, Y, x0, res, k, d_min, d_max, radio, alt_rel=(0.30, 0.60), libre_m=3.0):
    """Siembra rocas discretas (semi-elipsoides con borde vertical) en el cuadrado |x|,|y| <= radio, salvo r < libre_m del origen.
    Determinista por `rng`. Altura = D * U(alt_rel) (SUPUESTO de forma). Devuelve el numero de rocas y la altura maxima."""
    D, dens, total = distribucion_rocas(k, d_min, d_max)
    cdf = np.concatenate([[0.0], np.cumsum(0.5 * (dens[1:] + dens[:-1]) * np.diff(D))])
    cdf /= cdf[-1]
    n_roc = int(rng.poisson(total * (2.0 * radio) ** 2))
    n = z.shape[0]
    hmax = 0.0
    puestas = 0
    for _ in range(n_roc):
        dd = float(np.interp(rng.uniform(), cdf, D))
        rx, ry = rng.uniform(-radio, radio, 2)
        if np.hypot(rx, ry) < libre_m:
            rng.uniform(size=3)
            continue
        b_a = rng.uniform(0.6, 1.0)
        ang = rng.uniform(0.0, np.pi)
        h = dd * rng.uniform(*alt_rel)
        a = dd / 2.0
        i0 = max(int((rx - a - x0) / res), 0)
        i1 = min(int((rx + a - x0) / res) + 2, n)
        j0 = max(int((ry - a - x0) / res), 0)
        j1 = min(int((ry + a - x0) / res) + 2, n)
        if i1 <= i0 or j1 <= j0:
            continue
        dx = X[j0:j1, i0:i1] - rx
        dy = Y[j0:j1, i0:i1] - ry
        u = dx * np.cos(ang) + dy * np.sin(ang)
        v = -dx * np.sin(ang) + dy * np.cos(ang)
        rho2 = (u / a) ** 2 + (v / (a * b_a)) ** 2
        dentro = rho2 < 1.0
        if not dentro.any():
            continue
        base = float(np.mean(z[j0:j1, i0:i1][dentro]))
        perfil = base + h * np.sqrt(np.maximum(1.0 - rho2, 0.0))
        ventana = z[j0:j1, i0:i1]
        ventana[dentro] = np.maximum(ventana[dentro], perfil[dentro])
        zona[j0:j1, i0:i1][dentro] = 2
        hmax = max(hmax, h)
        puestas += 1
    return puestas, hmax


class Terreno:
    def __init__(self, tipo='llano', semilla=0, suelo_base=None, extension=120.0, res=0.1, **p):
        if tipo not in TIPOS:
            raise ValueError(f'tipo de terreno desconocido: {tipo!r} (hay: {", ".join(TIPOS)})')
        self.tipo, self.semilla, self.res, self.extension = tipo, int(semilla), float(res), float(extension)
        self.suelo_base = dict(suelo_base) if suelo_base is not None else None
        self.parametros = dict(p)
        n = int(round(extension / res)) + 1
        self.x0 = -extension / 2.0
        xs = self.x0 + res * np.arange(n)
        self._xs = xs
        X, Y = np.meshgrid(xs, xs)
        rng = np.random.default_rng(self.semilla)
        z = np.zeros((n, n))
        zona = np.zeros((n, n), dtype=np.uint8)
        if tipo == 'pendiente':
            ang = np.radians(p.get('angulo_deg', 15.0))
            dir_deg = p.get('direccion_deg', 0.0)
            z = np.tan(ang) * X if dir_deg == 0.0 else np.tan(ang) * (X * np.cos(np.radians(dir_deg)) + Y * np.sin(np.radians(dir_deg)))
        elif tipo == 'rocas':
            z = 0.01 * _fractal(rng, xs, xs, 3.0, 3)
            self.n_rocas, self.altura_roca_max = campo_de_rocas(
                rng, z, zona, X, Y, self.x0, res, p.get('abundancia', 0.10), p.get('d_min', 0.15), p.get('d_max', 0.80),
                p.get('radio_rocas', 30.0), libre_m=p.get('libre_m', 3.0))
        elif tipo == 'escalones':
            h = p.get('altura', 0.15)
            x_ini, sep, nesc = p.get('x_inicio', 3.0), p.get('separacion', 6.0), int(p.get('n', 1))
            for i in range(nesc):
                z = z + h * (X >= x_ini + i * sep)
            zona[z > 0.5 * h] = 2 if p.get('roca', False) else 0
        elif tipo == 'dunas':
            th = rng.uniform(0, np.pi)
            lam = rng.uniform(20.0, 30.0)
            amp = rng.uniform(0.6, 1.0) * p.get('amplitud', 1.0)
            fase = rng.uniform(0, 2 * np.pi)
            s = X * np.cos(th) + Y * np.sin(th)
            z = amp * np.sin(2 * np.pi * s / lam + fase) + 0.03 * _fractal(rng, xs, xs, 1.5, 3)
            zona[:] = 1
        elif tipo == 'crater':
            cx, cy = p.get('cx', 25.0), p.get('cy', 0.0)
            R, prof = p.get('radio', 15.0), p.get('profundidad', 3.0)
            r = np.hypot(X - cx, Y - cy)
            z = np.where(r < R, -prof * (1.0 - (r / R) ** 2), 0.0)
            z = z + 0.8 * np.exp(-(((r - R) / 2.0) ** 2)) + 0.02 * _fractal(rng, xs, xs, 2.0, 3)
            zona[r < 0.5 * R] = 1
        elif tipo == 'rocoso':
            z = 0.25 * p.get('rugosidad', 1.0) * _fractal(rng, xs, xs, 8.0, 4)
            nroc = int(p.get('densidad_rocas', 0.01) * extension ** 2)
            hmax = p.get('altura_roca_max', 0.30)
            for _ in range(nroc):
                rx, ry = rng.uniform(-extension / 2, extension / 2, 2)
                h = rng.uniform(0.05, hmax)
                s = rng.uniform(0.2, 0.5)
                i0 = max(int((rx - 3 * s - self.x0) / res), 0)
                i1 = min(int((rx + 3 * s - self.x0) / res) + 1, n)
                j0 = max(int((ry - 3 * s - self.x0) / res), 0)
                j1 = min(int((ry + 3 * s - self.x0) / res) + 1, n)
                if i1 <= i0 or j1 <= j0:
                    continue
                w = np.exp(-((X[j0:j1, i0:i1] - rx) ** 2 + (Y[j0:j1, i0:i1] - ry) ** 2) / (2 * s * s))
                z[j0:j1, i0:i1] += h * w
                zona[j0:j1, i0:i1][w > 0.5] = 2
            # zona de arranque llana (r < 2 m) con transicion suave hasta 5 m
            rr = np.hypot(X, Y)
            k = _suave(np.clip((rr - 2.0) / 3.0, 0.0, 1.0))
            z *= k
            zona[rr < 3.0] = 0
        self.z = z
        self.zona = zona

    # ---- consulta -------------------------------------------------------------------------------------------
    def altura(self, x, y):
        x = np.asarray(x, dtype=float)
        y = np.asarray(y, dtype=float)
        n = self.z.shape[0]
        u = np.minimum(np.maximum((x - self.x0) / self.res, 0.0), n - 1.000001)
        v = np.minimum(np.maximum((y - self.x0) / self.res, 0.0), n - 1.000001)
        i = u.astype(int)
        j = v.astype(int)
        fu = u - i
        fv = v - j
        z = self.z
        return ((z[j, i] * (1 - fu) + z[j, i + 1] * fu) * (1 - fv)
                + (z[j + 1, i] * (1 - fu) + z[j + 1, i + 1] * fu) * fv)

    def altura_normal(self, x, y):
        """(h, n) con n unitaria (..., 3); diferencias centrales a 0.1 m sobre la bilinear."""
        x = np.asarray(x, dtype=float)
        y = np.asarray(y, dtype=float)
        e = self.res
        xs = np.stack([x, x + e, x - e, x, x])
        ys = np.stack([y, y, y, y + e, y - e])
        a = self.altura(xs, ys)
        h = a[0]
        dzdx = (a[1] - a[2]) / (2 * e)
        dzdy = (a[3] - a[4]) / (2 * e)
        n = np.stack([-dzdx, -dzdy, np.ones_like(dzdx)], axis=-1)
        n /= np.sqrt(np.einsum('...i,...i->...', n, n))[..., None]
        return h, n

    def normal(self, x, y):
        return self.altura_normal(x, y)[1]

    def _zona(self, x, y):
        n = self.zona.shape[0]
        i = np.minimum(np.maximum(np.rint((np.asarray(x) - self.x0) / self.res).astype(int), 0), n - 1)
        j = np.minimum(np.maximum(np.rint((np.asarray(y) - self.x0) / self.res).astype(int), 0), n - 1)
        return self.zona[j, i]

    def suelo_arr(self, x, y):
        """Parametros Bekker-Wong como arreglos con la forma de x: kc, kphi, n, c, phi_deg, k_deslizamiento."""
        if self.suelo_base is None:
            raise ValueError('el terreno no tiene suelo_base (pasar el del planeta)')
        z = self._zona(x, y)
        mk, mc, dp, mK = _ZT[0][z], _ZT[1][z], _ZT[2][z], _ZT[3][z]
        b = self.suelo_base
        return {'kc': b['kc'] * mk, 'kphi': b['kphi'] * mk, 'n': np.full(z.shape, b['n'], dtype=float),
                'c': b['c'] * mc, 'phi_deg': b['phi_deg'] + dp, 'k_deslizamiento': b['k_deslizamiento'] * mK}

    def suelo(self, x, y):
        """Suelo en un punto (dict de escalares)."""
        a = self.suelo_arr(np.asarray(x, dtype=float), np.asarray(y, dtype=float))
        return {k: float(v) for k, v in a.items()}

    def pendiente_deg(self, x, y):
        n = self.normal(x, y)
        return np.degrees(np.arccos(np.clip(n[..., 2], -1, 1)))

    def bytes_huella(self):
        """Huella para la prueba de determinismo: los bytes del heightmap y de las zonas."""
        return self.z.tobytes() + self.zona.tobytes()
