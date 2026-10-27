#!/usr/bin/env python3
"""Puntuacion de mutacion de las pruebas de `atp_dron` (P7): 20 mutantes escritos a mano, cada uno un cambio pequeno y plausible
(un signo, un exponente, un indice, una constante, un orden). Un mutante esta MUERTO si la suite falla con el.

Uso:  python3 scripts/mutantes_dron.py [original|completa] [--json salida.json]
  original  solo los archivos de prueba que ya existian antes de P7 (la referencia de "hoy")
  completa  todos (los de P7 incluidos)
Cada mutante se aplica a una COPIA del paquete en un directorio temporal: el arbol de trabajo no se toca. Se corre con
`pytest -x` (para en la primera falla), asi que un mutante muerto cuesta poco y uno vivo cuesta la suite completa.
Es una herramienta de verificacion de las pruebas, no un estudio: no hay semillas ni estadistica. Los mutantes que sobreviven se
reportan tal cual (algunos pueden ser equivalentes o de efecto menor que la tolerancia de las pruebas existentes).
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
from concurrent.futures import ThreadPoolExecutor

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PAQ = os.path.join(RAIZ, 'src', 'atp_dron')
NUEVAS = ('test_propiedades.py', 'test_eskf_nees.py', 'test_determinismo.py')

# (nombre, archivo, texto original [debe aparecer], reemplazo) — se cambia solo la PRIMERA aparicion
MUTANTES = [
    ('giroscopico_signo', 'dron.py', "wd = self.Iinv @ (tau - _cruz(w, self.I @ w))", "wd = self.Iinv @ (tau + _cruz(w, self.I @ w))"),   # P8 cambio np.cross por _cruz
    ('par_guinada_signo', 'dron.py', "-(S_GIRO * self.P['k_guinada_m']) @ T", "(S_GIRO * self.P['k_guinada_m']) @ T"),
    ('rk4_k4_medio_paso', 'dron.py', "k4 = self._deriv(s + dt * k3, om_cmd, wind)", "k4 = self._deriv(s + 0.5 * dt * k3, om_cmd, wind)"),
    ('ou_varianza', 'dron.py', "self.sigma * np.sqrt(1 - a * a)", "self.sigma * np.sqrt(1 - a)"),
    ('potencia_exponente', 'dron.py', "self.empuje ** 1.5", "self.empuje ** 1.4"),
    ('efecto_suelo_exponente', 'rotor_fisica.py', "(radio / (4.0 * max(z, radio / 2.0))) ** 2", "(radio / (4.0 * max(z, radio / 2.0))) ** 1"),   # P8 unifico la ruta en rotor_fisica
    ('apoyado_desliza', 'dron.py', "            self.v[:] = 0.0\n", "            self.v[:] *= 1.0\n"),
    ('cuaternion_sin_normalizar', 'dron.py', "s[6:10] / np.linalg.norm(s[6:10])", "s[6:10]"),
    ('motor_constante_tiempo', 'dron.py', "(om_cmd - om) / self.P['tau_motor_s']", "(om_cmd - om) / (2.0 * self.P['tau_motor_s'])"),
    ('eskf_Q_x250', 'estimador.py', "dt_r = DT * DT if S.get('q_corregida') else DT", "dt_r = DT"),
    ('eskf_F_cruz_signo', 'estimador.py', "F[3:6, 6:9] = -R @ _cruz(am) * DT", "F[3:6, 6:9] = R @ _cruz(am) * DT"),
    ('eskf_H_baro_indice', 'estimador.py', "H[0, 2] = 1.0", "H[0, 1] = 1.0"),
    ('eskf_H_mag_sin_cos_pitch', 'estimador.py', "np.array([np.sin(r_), np.cos(r_)]) / np.cos(p_)", "np.array([np.sin(r_), np.cos(r_)]) / 1.0"),
    ('eskf_joseph_sin_ruido', 'estimador.py', "+ K @ Rm @ K.T", "+ 0.0 * (K @ Rm @ K.T)"),
    ('eskf_sesgo_giro_signo', 'estimador.py', "self.bg += dx[12:15]", "self.bg -= dx[12:15]"),
    ('sensor_sesgo_acel_signo', 'sensores.py', "self.d.fuerza_especifica() + self.sesgo_a", "self.d.fuerza_especifica() - self.sesgo_a"),
    ('sensor_rw_sesgo_sin_raiz_dt', 'sensores.py', "self.sesgo_a += P['acel_sesgo_rw'] * raiz *", "self.sesgo_a += P['acel_sesgo_rw'] *"),
    ('control_sin_proyeccion_empuje', 'control.py', "T = self.T_des * max(0.0, float(zb @ R[:, 2]))", "T = self.T_des"),
    ('lockstep_gyro_sin_sesgo', 'lockstep.py', "self.vista.w = g_m - self.est.bg", "self.vista.w = g_m"),
]


def correr_mutante(args):
    i, (nombre, archivo, viejo, nuevo), suite = args
    t0 = time.time()
    with tempfile.TemporaryDirectory() as tmp:
        os.makedirs(os.path.join(tmp, 'src'))
        dst = os.path.join(tmp, 'src', 'atp_dron')
        shutil.copytree(PAQ, dst, ignore=shutil.ignore_patterns('__pycache__', '.pytest_cache', '*.egg-info'))
        os.symlink(os.path.join(os.path.dirname(PAQ), 'atp_ros'), os.path.join(tmp, 'src', 'atp_ros'))
        os.symlink(os.path.join(RAIZ, 'scripts'), os.path.join(tmp, 'scripts'))      # las pruebas cargan scripts/dron_lockstep.py
        if suite == 'original':
            for n in NUEVAS:
                p = os.path.join(dst, 'test', n)
                if os.path.exists(p):
                    os.remove(p)
        ruta = os.path.join(dst, 'atp_dron', archivo)
        with open(ruta) as f:
            txt = f.read()
        if viejo is not None:
            if txt.count(viejo) < 1:
                return {'mutante': nombre, 'error': f'texto no encontrado en {archivo}'}
            txt = txt.replace(viejo, nuevo, 1)
            with open(ruta, 'w') as f:
                f.write(txt)
        r = subprocess.run([sys.executable, '-m', 'pytest', '-x', '-q', '-p', 'no:cacheprovider', 'test'], cwd=dst,
                           capture_output=True, text=True, timeout=1800)
        muerto = r.returncode != 0
        falla = next((l for l in r.stdout.splitlines() if l.startswith('FAILED') or l.startswith('ERROR')), '')
        return {'mutante': nombre, 'archivo': archivo, 'muerto': muerto, 'prueba_que_lo_mata': falla[:160] if muerto else '',
                's': round(time.time() - t0, 1)}


def main(argv):
    suite = next((a for a in argv if a in ('original', 'completa')), 'completa')
    salida = argv[argv.index('--json') + 1] if '--json' in argv else None
    # sanidad: sin mutar la suite debe pasar
    base = correr_mutante((0, ('sin_mutar', 'dron.py', None, None), suite))
    print('sin mutar:', 'pasa' if not base['muerto'] else 'FALLA', f"({base['s']} s)", flush=True)
    if base['muerto']:
        print(base)
        return 2
    with ThreadPoolExecutor(4) as ex:
        res = list(ex.map(correr_mutante, [(i, m, suite) for i, m in enumerate(MUTANTES)]))
    for r in res:
        print(('MUERTO  ' if r.get('muerto') else 'VIVO    ') + r['mutante'], r.get('prueba_que_lo_mata', r.get('error', '')), flush=True)
    muertos = sum(1 for r in res if r.get('muerto'))
    print(f'puntuacion de mutacion ({suite}): {muertos}/{len(res)}')
    if salida:
        with open(salida, 'w') as f:
            json.dump({'suite': suite, 'muertos': muertos, 'total': len(res), 'mutantes': res}, f, indent=1)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
