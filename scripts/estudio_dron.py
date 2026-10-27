#!/usr/bin/env python3
"""Estudio del dron (ronda D2): escalon, viento, carga util, cuerpos, rafagas Monte Carlo y efecto suelo.

    python3 scripts/estudio_dron.py [--rapido] [--salida docs/dron_resultados]

Escribe estudio_dron.json (las cifras) y estudio_dron.md (las tablas). Mismos supuestos y semillas -> mismas cifras.
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'atp_dron'))

from atp_dron import estudios as E   # noqa: E402


def tabla(filas, cols):
    out = ['| ' + ' | '.join(cols) + ' |', '|' + '---|' * len(cols)]
    for f in filas:
        out.append('| ' + ' | '.join('—' if f.get(c) is None else str(f.get(c)) for c in cols) + ' |')
    return '\n'.join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rapido', action='store_true')
    ap.add_argument('--salida', default='docs/dron_resultados')
    a = ap.parse_args()
    n_mc = 6 if a.rapido else 30
    res = {'supuestos': 'cuadrirrotor 1.5 kg, 4 helices de 5", empuje/peso 2.2 (Tierra); ver docs/DRON.md'}
    res['escalon_5m'] = E.respuesta_escalon()
    res['viento'] = E.barrido_viento(velocidades=(0, 4, 8, 12, 16, 20, 24), duracion_s=30.0 if a.rapido else 40.0)
    res['viento_rafagas'] = E.barrido_viento(velocidades=(0, 6, 12), duracion_s=40.0, rafaga_sigma=1.5, semilla=7)
    res['carga'] = E.barrido_carga()
    res['carga_marte_x14'] = E.barrido_carga(cuerpo_nombre='marte', params={'rotor_escala': 14.0})
    res['cuerpos'] = E.comparar_cuerpos({'marte': 14.0})
    res['montecarlo_rafagas'] = E.montecarlo_rafagas(n=n_mc)
    res['efecto_suelo'] = E.efecto_suelo_hover()
    res['estimador'] = E.estimador_montecarlo(n=4 if a.rapido else 20)
    res['perdida_gps'] = E.perdida_gps()
    os.makedirs(a.salida, exist_ok=True)
    with open(os.path.join(a.salida, 'estudio_dron.json'), 'w') as f:
        json.dump(res, f, sort_keys=True, indent=1)
    mc = res['montecarlo_rafagas']
    md = ['# Estudio del dron (D2)', '', '_En simulación, con estos supuestos; no es un dron concreto._', '',
          '## Escalón de posición (5 m)', '', '```', json.dumps(res['escalon_5m']), '```', '',
          '## Viento constante, mantener posición a 10 m', '',
          tabla(res['viento'], ['viento_ms', 'desviacion_max_m', 'inclinacion_max_deg', 'energia_Wh', 'choque']), '',
          '## Carga útil (Tierra, batería de 60 Wh)', '',
          tabla(res['carga'], ['carga_kg', 'empuje_peso', 'p_hover_w', 'autonomia_min']), '',
          '## Carga útil en Marte (rotores ×14, SUPUESTO sin fuente)', '',
          tabla(res['carga_marte_x14'], ['carga_kg', 'empuje_peso', 'puede_volar', 'p_hover_w', 'autonomia_min']), '',
          '## Mismo dron en cada cuerpo', '',
          tabla(res['cuerpos'], ['cuerpo', 'rotor_escala', 'empuje_peso', 'rotor_escala_min_tw1.3', 'p_hover_w']), '',
          f'## Ráfagas (σ = 1 m/s, viento 2 m/s), {mc["n"]} semillas', '',
          f'Llegaron {mc["llegaron"]}/{mc["n"]}, choques {mc["choques"]}. Error transversal rms: {mc["error_transversal_rms_m"]}. '
          f'Energía: {mc["energia_Wh"]}.', '',
          f'## Estimador ESKF con IMU, GPS, barómetro y magnetómetro, {res["estimador"]["n"]} semillas', '',
          f'Llegaron {res["estimador"]["llegaron"]}/{res["estimador"]["n"]}. Error de posición rms {res["estimador"]["error_pos_rms_m"]} m; '
          f'actitud {res["estimador"]["error_actitud_rms_deg"]} grados; error transversal con estimación '
          f'{res["estimador"]["error_transversal_rms_m"]} m.', '',
          '## Pérdida de GPS (mantener posición a 10 m, 60 s)', '',
          tabla(res['perdida_gps'], ['corte_gps_s', 'est_error_pos_final_m', 'pos_verdadera_final_m', 'est_error_actitud_rms_deg']), '',
          '## Efecto suelo (Cheeseman–Bennett)', '',
          tabla(res['efecto_suelo'], ['z_m', 'z_sobre_R', 'T_sobre_Tinf', 'P_sobre_Pinf_a_igual_T']), '']
    with open(os.path.join(a.salida, 'estudio_dron.md'), 'w') as f:
        f.write('\n'.join(md))
    print('\n'.join(md))
    return 0


if __name__ == '__main__':
    sys.exit(main())
