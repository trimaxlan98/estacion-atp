"""Genera data/tle/README.md con la epoca de cada TLE (lo llama actualizar_tle.sh)."""
import datetime as dt
import glob
import os

filas = []
for f in sorted(glob.glob('data/tle/*.tle')):
    lin = [x.rstrip() for x in open(f) if x.strip()]
    for i in range(0, len(lin) - 2, 3):
        nombre, l1 = lin[i].strip(), lin[i + 1]
        if not l1.startswith('1 '):
            continue
        yy, dia = int(l1[18:20]), float(l1[20:32])
        anio = 2000 + yy if yy < 57 else 1900 + yy
        ep = (dt.datetime(anio, 1, 1, tzinfo=dt.timezone.utc)
              + dt.timedelta(days=dia - 1))
        filas.append((os.path.basename(f), nombre, l1[2:7],
                      ep.strftime('%Y-%m-%d %H:%M UTC')))

ahora = dt.datetime.now(dt.timezone.utc)
with open('data/tle/README.md', 'w') as o:
    o.write('# TLE de la Estacion\n\n'
            'Fuente: CelesTrak (https://celestrak.org/NORAD/elements/), TLE de 3 lineas.\n'
            f'Generado por `scripts/actualizar_tle.sh` el {ahora:%Y-%m-%d %H:%M UTC}.\n\n'
            'Un TLE solo vale cerca de su epoca: el error de SGP4 crece del orden de\n'
            '1-3 km/dia en LEO (mas con arrastre alto). Para predecir pases, refrescar.\n\n'
            '| archivo | objeto | NORAD | epoca |\n|---|---|---|---|\n')
    for r in filas:
        o.write('| ' + ' | '.join(r) + ' |\n')
print('README.md escrito:', len(filas), 'objetos')
