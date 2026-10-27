#!/usr/bin/env python3
"""Genera los meshes simples de la montura (STL ASCII, metros, solo stdlib).

  python3 scripts/gen_meshes.py meshes/

Piezas: pedestal (cilindro con brida), yugo (base + dos brazos), plato
(paraboloide de 3 m, f/D = 0.4) y brazo del alimentador. Son geometria de
VISUALIZACION: la colision del URDF usa primitivas (cilindros y cajas), que
son baratas y robustas en cualquier motor de fisica.
"""
import math
import os
import sys


def _normal(a, b, c):
    ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
    n = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx)
    m = math.sqrt(sum(x * x for x in n)) or 1.0
    return tuple(x / m for x in n)


class Malla:
    def __init__(self):
        self.tri = []

    def triangulo(self, a, b, c):
        self.tri.append((a, b, c))

    def cuadro(self, a, b, c, d):
        self.triangulo(a, b, c)
        self.triangulo(a, c, d)

    def guardar(self, ruta, nombre):
        with open(ruta, 'w') as f:
            f.write(f'solid {nombre}\n')
            for a, b, c in self.tri:
                n = _normal(a, b, c)
                f.write(f' facet normal {n[0]:.6f} {n[1]:.6f} {n[2]:.6f}\n  outer loop\n')
                for v in (a, b, c):
                    f.write(f'   vertex {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n')
                f.write('  endloop\n endfacet\n')
            f.write(f'endsolid {nombre}\n')


def cilindro(m, r, h, z0=0.0, n=32):
    """Cilindro vertical de radio r y altura h con base en z0."""
    for i in range(n):
        a0, a1 = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
        p0 = (r * math.cos(a0), r * math.sin(a0))
        p1 = (r * math.cos(a1), r * math.sin(a1))
        m.cuadro((p0[0], p0[1], z0), (p1[0], p1[1], z0), (p1[0], p1[1], z0 + h), (p0[0], p0[1], z0 + h))
        m.triangulo((0, 0, z0 + h), (p0[0], p0[1], z0 + h), (p1[0], p1[1], z0 + h))
        m.triangulo((0, 0, z0), (p1[0], p1[1], z0), (p0[0], p0[1], z0))


def caja(m, x0, x1, y0, y1, z0, z1):
    v = [(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]
    p = lambda i, j, k: v[i * 4 + j * 2 + k]  # noqa: E731
    m.cuadro(p(0, 0, 0), p(0, 1, 0), p(0, 1, 1), p(0, 0, 1))
    m.cuadro(p(1, 0, 0), p(1, 0, 1), p(1, 1, 1), p(1, 1, 0))
    m.cuadro(p(0, 0, 0), p(0, 0, 1), p(1, 0, 1), p(1, 0, 0))
    m.cuadro(p(0, 1, 0), p(1, 1, 0), p(1, 1, 1), p(0, 1, 1))
    m.cuadro(p(0, 0, 0), p(1, 0, 0), p(1, 1, 0), p(0, 1, 0))
    m.cuadro(p(0, 0, 1), p(0, 1, 1), p(1, 1, 1), p(1, 0, 1))


def plato(m, d=3.0, f_sobre_d=0.4, nr=8, na=40):
    """Paraboloide de revolucion con el eje en +X (el plato apunta a +X), vertice
    en el origen: x = r^2 / (4 f), f = f_sobre_d * d. Cascara de 2 cm."""
    f = f_sobre_d * d
    R = d / 2.0
    esp = 0.02

    def punto(i, j, dx):
        r = R * i / nr
        a = 2 * math.pi * j / na
        return (r * r / (4 * f) + dx, r * math.cos(a), r * math.sin(a))

    for i in range(nr):
        for j in range(na):
            for dx, orient in ((0.0, 1), (-esp, -1)):
                a, b = punto(i, j, dx), punto(i + 1, j, dx)
                c, e = punto(i + 1, j + 1, dx), punto(i, j + 1, dx)
                if orient > 0:
                    m.cuadro(a, b, c, e)
                else:
                    m.cuadro(a, e, c, b)
    for j in range(na):  # borde
        a, b = punto(nr, j, 0.0), punto(nr, j + 1, 0.0)
        c, e = punto(nr, j + 1, -esp), punto(nr, j, -esp)
        m.cuadro(a, e, c, b)


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else 'meshes'
    os.makedirs(out, exist_ok=True)
    # pedestal: fuste de 1 m (z 0..1) y brida
    m = Malla()
    cilindro(m, 0.40, 0.90, 0.0)
    cilindro(m, 0.55, 0.10, 0.90)
    m.guardar(os.path.join(out, 'pedestal.stl'), 'pedestal')
    # yugo (marco az_link, origen en el eje az): base y dos brazos hasta el eje de elevacion (z = 0.9)
    m = Malla()
    caja(m, -0.30, 0.30, -0.80, 0.80, 0.0, 0.25)
    caja(m, -0.10, 0.10, -0.80, -0.65, 0.25, 0.95)
    caja(m, -0.10, 0.10, 0.65, 0.80, 0.25, 0.95)
    m.guardar(os.path.join(out, 'yugo.stl'), 'yugo')
    # plato de 3 m apuntando a +X con el vertice en el origen del el_link
    m = Malla()
    plato(m)
    m.guardar(os.path.join(out, 'plato.stl'), 'plato')
    # brazo del alimentador: cilindro largo a lo largo de +X (se genera vertical y se
    # deja que el URDF lo gire), de 1.2 m, mas el bloque del alimentador
    m = Malla()
    cilindro(m, 0.03, 1.20, 0.0, n=24)
    cilindro(m, 0.09, 0.12, 1.20, n=24)
    m.guardar(os.path.join(out, 'alimentador.stl'), 'alimentador')
    print('meshes en', out)


if __name__ == '__main__':
    main()
