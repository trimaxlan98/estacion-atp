#!/usr/bin/env python3
"""Estudio P8: fisica del rotor (coherencia par-potencia, Reynolds, momentum, Glauert, efecto suelo unico).

Uso: python3 scripts/estudio_rotor.py [--salida docs/dron_resultados/estudio_rotor.json]
Todo es "en simulacion, con estos supuestos"; cada cifra del documento sale de este JSON.
"""
import argparse
import hashlib
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'atp_dron'))
from atp_dron import rotor_fisica as RF                       # noqa: E402
from atp_dron.cuerpos import cuerpo, viscosidad               # noqa: E402
from atp_dron.dron import Dron, PARAMS, rot                   # noqa: E402

R = PARAMS['radio_rotor_m']


def hover_dinamico(cuerpo_nombre, params, t_s=2.0):
    """Hover real en el simulador: potencia (W) medida en el ultimo paso, y FoM implicita con T y om simulados."""
    d = Dron(cuerpo_nombre, params=params, pos=(0, 0, 1000.0))
    Th = d.m * d.g / 4.0
    n = int(t_s / 0.004)
    for _ in range(n):
        d.paso(np.full(4, Th))
    e0 = d.energia_j
    d.paso(np.full(4, Th))
    p = (d.energia_j - e0) / 0.004
    return {'potencia_w': float(p), 'T': float(d.empuje[0]), 'om': float(d.om[0]), 'k_m': d.P['k_guinada_m'],
            'fom_implicita': RF.fom_implicita(d.empuje[0], d.om[0], d.P['k_guinada_m'], d.rho, d.P['radio_rotor_m'])}


def potencia_formula(cuerpo_nombre, fom=0.6, eta=0.8, p_av=15.0, m=1.5):
    c = cuerpo(cuerpo_nombre)
    return float(4 * RF.potencia_ideal(m * c['g'] / 4.0, c['rho'], R) / (fom * eta) + p_av)


def h1():
    out = {}
    for c in ('tierra', 'titan'):
        leg = hover_dinamico(c, {})
        coh = hover_dinamico(c, {'modo_rotor': 'coherente'})
        par = hover_dinamico(c, {'modo_rotor': 'energia_par'})
        out[c] = {
            'formula_hover_w': potencia_formula(c),
            'legacy': leg, 'coherente': coh, 'energia_par': par,
            'error_fom_legacy': abs(leg['fom_implicita'] - 0.6),
            'error_fom_coherente': abs(coh['fom_implicita'] - 0.6),
            'error_rel_potencia_coherente_vs_formula': abs(coh['potencia_w'] - potencia_formula(c)) / potencia_formula(c),
            'energia_par_sobre_legacy': par['potencia_w'] / leg['potencia_w'],
        }
    # antes de eficiencia (Tierra, solo los 4 rotores, sin aviónica): 4 P_ideal/FoM vs 4 kM T Omega
    t = out['tierra']['legacy']
    out['tierra']['antes_de_eficiencia_w'] = {
        'fom_0.6': 4 * RF.potencia_ideal(t['T'], 1.225, R) / 0.6,
        'par_kM_T_Omega': 4 * 0.016 * t['T'] * t['om'],
    }
    # coherencia a otros empujes (rho fijo): P = kM T Omega escala igual que T^1.5
    d = Dron('tierra', params={'modo_rotor': 'coherente'})
    ratios = []
    for f in (0.5, 0.8, 1.0, 1.3, 1.8):
        T = f * d.m * d.g / 4
        ratios.append(RF.fom_implicita(T, np.sqrt(T / d.kT), d.P['k_guinada_m'], d.rho, R))
    out['fom_implicita_vs_empuje_coherente'] = ratios
    return out


def h2():
    out = {}
    for c in ('tierra', 'marte', 'titan'):
        cu = cuerpo(c)
        d = Dron(c)
        out[c] = {'empuje_peso_efectivo': d.empuje_peso_efectivo, 'mu_pa_s': viscosidad(c)}
    cu = cuerpo('marte')
    mu = viscosidad('marte')
    out['marte']['escala_min_solo_densidad'] = RF.escala_minima_hover(cu['rho'], cu['g'])
    n_ct = RF.exponente_desde_caida(0.26)
    n_fm = RF.exponente_desde_caida(0.36)
    out['exponentes_desde_resumen'] = {'exp_kT': n_ct, 'exp_fom': n_fm}
    esc = {}
    for nombre, (nk, nf) in {'sin_Re (exponente 0)': (0.0, 0.0), 'mitad': (n_ct / 2, n_fm / 2),
                             'nominal (-26 %/-36 %)': (n_ct, n_fm), 'doble': (2 * n_ct, 2 * n_fm)}.items():
        e = {}
        for geo in ('fija', 'escalada'):
            s = RF.escala_minima_hover(cu['rho'], cu['g'], mu, expo_kT=nk, geometria=geo)
            e['escala_min_' + geo] = s
        s = e['escala_min_fija']
        e['hover_en_escala_min_fija'] = RF.hover_con_re(cu['rho'], cu['g'], mu, s * 1.0000001, expo_kT=nk, expo_fom=nf)
        s0 = out['marte']['escala_min_solo_densidad']
        e['hover_en_escala_min_solo_densidad'] = RF.hover_con_re(cu['rho'], cu['g'], mu, s0 * 1.0000001, expo_kT=nk,
                                                                 expo_fom=nf)
        esc[nombre] = e
    out['marte']['barrido_exponentes'] = esc
    re_max = float(RF.reynolds(cu['rho'], mu, PARAMS['om_max'], R, PARAMS['cuerda_m']))
    out['marte']['re_en_om_max'] = re_max
    out['marte']['empuje_peso_con_re_en_escala_solo_densidad'] = (
        out['marte']['empuje_peso_efectivo'] * out['marte']['escala_min_solo_densidad']
        * float(RF.f_reynolds(re_max, PARAMS['re_ref'], n_ct)))
    ch = {}
    for c_m in (0.02, 0.04, 0.08, 0.13):
        re_c = float(RF.reynolds(cu['rho'], mu, PARAMS['om_max'], R, c_m))
        ch[str(c_m)] = {'re_om_max': re_c, 'escala_min_fija_nominal': RF.escala_minima_hover(
            cu['rho'], cu['g'], mu, cuerda=c_m, expo_kT=n_ct), 'f_kT': float(RF.f_reynolds(re_c, PARAMS['re_ref'], n_ct))}
    out['marte']['barrido_cuerda'] = ch
    p_dens = 4 * RF.potencia_ideal(PARAMS['masa_kg'] * cu['g'] / 4, cu['rho'], R) / (0.6 * 0.8) + 15.0
    out['marte']['potencia_hover_solo_densidad_w'] = float(p_dens)
    # Tierra: f(Re) = 1 en hover y byte a byte con condicion inicial en hover
    ct = cuerpo('tierra')
    d = Dron('tierra', params={'f_reynolds': True})
    Th = d.m * d.g / 4
    om_h = float(np.sqrt(Th / d.kT))
    re_h = RF.reynolds(ct['rho'], viscosidad('tierra'), om_h, R, PARAMS['cuerda_m'])
    om_re1 = PARAMS['re_ref'] * viscosidad('tierra') / (ct['rho'] * R * PARAMS['cuerda_m'])

    def corre(flag):
        d = Dron('tierra', params={'f_reynolds': flag}, pos=(0, 0, 100.0), viento=(2, 1, 0), rafaga_sigma=1.0, semilla=3)
        d.om[:] = om_h
        h = hashlib.sha256()
        rng = np.random.default_rng(1)
        for k in range(1500):
            d.paso(Th * (1 + 0.1 * np.sin(k * 0.01) + 0.05 * rng.standard_normal(4)))
            h.update(np.concatenate([d.p, d.v, d.q, d.w, d.om, [d.energia_j]]).tobytes())
        return h.hexdigest()[:16], float(d.om.min())
    h0, m0 = corre(False)
    h1_, m1 = corre(True)
    out['tierra_re'] = {'re_hover': float(re_h), 'om_minimo_para_f1_rad_s': float(om_re1), 'om_hover': om_h,
                        'hash_sin_f': h0, 'hash_con_f': h1_, 'identico': h0 == h1_, 'om_min_vista': min(m0, m1)}
    # control negativo: arrancando desde reposo (om bajo) f<1 y los bytes cambian
    def corre_reposo(flag):
        d = Dron('tierra', params={'f_reynolds': flag}, pos=(0, 0, 100.0))
        for _ in range(300):
            d.paso(np.full(4, Th))
        return hashlib.sha256(np.concatenate([d.p, d.v, d.om]).tobytes()).hexdigest()[:16]
    out['tierra_re']['control_negativo_desde_reposo_distinto'] = corre_reposo(False) != corre_reposo(True)
    # Marte dinamico: empuje maximo con f(Re) vs prediccion
    dm = Dron('marte', params={'f_reynolds': True, 'rotor_escala': 11.0})
    dm.om[:] = dm.P['om_max']
    f_pred = float(RF.f_reynolds(RF.reynolds(dm.rho, mu, dm.P['om_max'], R, PARAMS['cuerda_m']), PARAMS['re_ref'], PARAMS['exp_kT']))
    Te = dm._empuje_efectivo(dm.om, 10.0)
    out['marte_dinamico'] = {'f_kT_en_om_max': f_pred, 'T_sobre_t_max': float(Te[0] / dm.t_max)}
    return out


def h3():
    vh = float(RF.v_hover(1.5 * 9.80665 / 4, 1.225, R))
    res = []
    for vc in (0.0, 0.5, 1.0, 2.0, 5.0, 10.0, -2 * vh, -12.0, -20.0):
        vi = RF.induccion_axial(vc, vh)
        signo = 1.0 if vc >= 0 else -1.0       # ascenso: vi(vi+vc)=vh^2; freno de molino: vi(vi+vc)=-vh^2 (T = -2 rho A vi (vc+vi))
        res.append({'vc': vc, 'vi': float(vi), 'residuo_momentum': float(abs(vi * (vi + vc) - signo * vh ** 2) / vh ** 2),
                    'rel_formula_ascenso': float(abs(vi - (-vc / 2 + np.sqrt((vc / 2) ** 2 + vh ** 2))) / vi) if vc >= 0 else None})
    vrs_err = []
    for vc in (-0.1 * vh, -vh, -1.9 * vh):
        try:
            RF.induccion_axial(vc, vh)
            vrs_err.append({'vc': vc, 'lanza': False})
        except NotImplementedError as e:
            vrs_err.append({'vc': vc, 'lanza': True})
    return {'vh_tierra_m_s': vh, 'rama_momentum': res, 'banda_vrs': vrs_err,
            'continuidad_en_-2vh': float(abs(RF.induccion_axial(-2 * vh, vh) - vh)),
            'maximo_residuo_momentum': max(r['residuo_momentum'] for r in res),
            'estado_vrs': 'NO IMPLEMENTADO: coeficientes de Johnson NO VERIFICADO (PDF no accesible desde el sandbox)'}


def h4():
    cd = cuerpo('tierra')
    Th = 1.5 * cd['g'] / 4
    vh = float(RF.v_hover(Th, cd['rho'], R))
    om_h = float(np.sqrt(Th / Dron('tierra').kT))
    Vs = np.linspace(0.0, 30.0, 61)
    cda = PARAMS['cda_m2']
    tot, ind, per, par, ratio = [], [], [], [], []
    for V in Vs:
        p, t = RF.potencia_avance(Th, V, cd['rho'], R, 0.6, om_h, 4.65, cda)
        tot.append(4 * p), ind.append(4 * t['inducida']), per.append(4 * t['perfil']), par.append(4 * t['parasita'])
        ratio.append(RF.razon_induccion_avance(V, vh))
    tot, ind = np.array(tot), np.array(ind)
    i_min = int(np.argmin(tot))
    # Newton vs cerrada
    err_it = max(abs(RF.glauert_iterativa(V, vh)[0] - RF.glauert_cerrada(V, vh)) / vh for V in Vs)
    iters = max(RF.glauert_iterativa(V, vh)[1] for V in Vs)
    # mu->0
    p0, _ = RF.potencia_avance(Th, 0.0, cd['rho'], R, 0.6, om_h, 4.65, cda)
    hover = RF.potencia_ideal(Th, cd['rho'], R) / 0.6
    # alta velocidad: induccion -> vh^2/V ; parasita exacta
    Vh = 200.0
    asint = float(RF.glauert_cerrada(Vh, vh) / (vh ** 2 / Vh))
    # control negativo: sin Glauert (induccion de hover constante), no hay curva en U
    sin_glauert = [4 * (RF.potencia_ideal(Th, cd['rho'], R) + RF.potencia_ideal(Th, cd['rho'], R) * (1 / 0.6 - 1)
                        * (1 + 4.65 * (V / (om_h * R)) ** 2) + 0.5 * cda * cd['rho'] * V ** 3 / 4) for V in Vs]
    # dinamico: energia del simulador con la bandera, V = 10 m/s relativa
    d = Dron('tierra', params={'avance': True})
    d.v = np.array([10.0, 0, 0]); d.om[:] = om_h; d.empuje = d.kT * d.om ** 2
    e0 = d.energia_j; d._energia(1.0)
    p_sim = d.energia_j - e0
    p_esp = 4 * RF.potencia_avance(Th, 10.0, cd['rho'], R, 0.6, om_h, 4.65, cda)[0] / 0.8 + 15.0
    d2 = Dron('tierra', params={'avance': True})
    d2.om[:] = om_h; d2.empuje = d2.kT * d2.om ** 2
    e0 = d2.energia_j; d2._energia(1.0)
    p_v0 = d2.energia_j - e0
    return {'vh_m_s': vh, 'V_m_s': Vs.tolist(), 'potencia_rotores_w': tot.tolist(), 'inducida_w': ind.tolist(),
            'perfil_w': per, 'parasita_w': par, 'razon_vi_vh': ratio,
            'inducida_monotona_decreciente': bool(np.all(np.diff(ind) < 0)),
            'V_potencia_minima_m_s': float(Vs[i_min]), 'potencia_minima_rotores_w': float(tot[i_min]),
            'potencia_V0_rotores_w': float(tot[0]), 'curva_en_U': bool(0 < i_min < len(Vs) - 1),
            'maximo_error_newton_vs_cerrada_rel': float(err_it), 'iteraciones_max': iters,
            'mu_cero_vs_hover_rel': float(abs(p0 - hover) / hover),
            'razon_inducida_alta_velocidad_vs_vh2_V': asint,
            'control_negativo_sin_glauert': {'creciente': bool(np.all(np.diff(sin_glauert) > 0)),
                                             'curva_en_U': bool(0 < int(np.argmin(sin_glauert)) < len(Vs) - 1)},
            'parasita_vs_formula_rel': float(abs(RF.potencia_avance(Th, 30.0, cd['rho'], R, 0.6, om_h, 4.65, cda)[1]['parasita'] * 4
                                                  - 0.5 * cda * cd['rho'] * 30.0 ** 3) / (0.5 * cda * cd['rho'] * 30.0 ** 3)),
            'simulador_V10_w': float(p_sim), 'esperado_V10_w': float(p_esp),
            'simulador_V0_w': float(p_v0), 'hover_formula_w': potencia_formula('tierra'),
            'k_mu': 4.65, 'nota_k_mu': 'PARAMETRICO (fuente secundaria, texto de Leishman no leido)'}


def h5():
    rng = np.random.default_rng(7)
    max_dT, max_da, n = 0.0, 0.0, 0
    for z in (0.02, 0.05, 0.0635, 0.1, 0.2, 0.5, 1.0, 3.0):
        for _ in range(20):
            d = Dron('tierra', params={'efecto_suelo': True}, pos=(0, 0, z))
            d.q = rng.standard_normal(4); d.q /= np.linalg.norm(d.q)
            d.v = rng.standard_normal(3) * 2; d.om = 600 + 100 * rng.random(4)
            d.en_suelo = False
            wind = np.array([1.0, -2.0, 0.5])
            d.viento_medio = wind
            s = np.concatenate([d.p, d.v, d.q, d.w, d.om])
            a_din = d._deriv(s, d.om, wind)[3:6] + np.array([0, 0, d.g])
            a_body = rot(d.q).T @ a_din
            a_acel = d.fuerza_especifica()
            max_da = max(max_da, float(np.abs(a_body - a_acel).max()))
            Td = d._empuje_efectivo(d.om, z)
            Ta = d.kT * d.om * d.om
            Ta = RF.aplicar_suelo(Ta, z, R)
            max_dT = max(max_dT, float(np.abs(Td - Ta).max()))
            n += 1
    # control negativo: alterar una sola ruta (radio 1 % mayor solo en el acelerometro)
    class DronRoto(Dron):
        def fuerza_especifica(self):
            R_ = rot(self.q)
            T = RF.aplicar_suelo(self.kT * self.om * self.om, self.p[2], R * 1.01)
            vr = self.v - self.viento
            arr = -0.5 * self.rho * self.P['cda_m2'] * np.linalg.norm(vr) * vr
            return (np.array([0.0, 0.0, T.sum()]) + R_.T @ arr) / self.m
    d = DronRoto('tierra', params={'efecto_suelo': True}, pos=(0, 0, 0.1))
    d.om[:] = 650.0; d.en_suelo = False
    s = np.concatenate([d.p, d.v, d.q, d.w, d.om])
    roto = float(np.abs(rot(d.q).T @ (d._deriv(s, d.om, d.viento)[3:6] + np.array([0, 0, d.g])) - d.fuerza_especifica()).max())
    return {'casos': n, 'max_dif_empuje_n': max_dT, 'max_dif_aceleracion_m_s2': max_da,
            'control_negativo_ruta_alterada_m_s2': roto,
            'factor_suelo_en_z_0.1': float(RF.factor_suelo(0.1, R)), 'tope': float(RF.factor_suelo(0.0, R))}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--salida', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'docs',
                                                     'dron_resultados', 'estudio_rotor.json'))
    a = ap.parse_args()
    res = {'H1_coherencia_par_potencia': h1(), 'H2_reynolds_marte': h2(), 'H3_momentum_vrs': h3(),
           'H4_avance_glauert': h4(), 'H5_efecto_suelo_unico': h5(),
           'supuestos': {'radio_rotor_m': R, 'cuerda_m': PARAMS['cuerda_m'], 'masa_kg': PARAMS['masa_kg'],
                         'T_Marte_K': 210.0, 'T_Tierra_K': 288.15,
                         'viscosidad': 'Sutherland aire/CO2 con constantes de extractos de buscador; Titan sin viscosidad (NO VERIFICADO)'}}
    os.makedirs(os.path.dirname(os.path.abspath(a.salida)), exist_ok=True)
    with open(a.salida, 'w') as f:
        json.dump(res, f, indent=1, sort_keys=True)
        f.write('\n')
    print('escrito', a.salida)


if __name__ == '__main__':
    main()
