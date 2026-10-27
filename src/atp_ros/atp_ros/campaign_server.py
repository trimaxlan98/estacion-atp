"""Servidor de la accion /atp/run_campaign (Monte Carlo de apuntamiento).

Modo "analitico": cuadratura de terminos (milisegundos, miles de corridas).
Modo "lazo": simula PID + montura de cada pase, vectorizado, con feedback.
Publica el resultado tambien en /atp/campaign_result para el grabador.
"""
import json
import time

import numpy as np
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from std_msgs.msg import String

from atp_interfaces.action import RunCampaign
from atp_ros import model
from atp_ros.nodo import correr


class CampaignServer(Node):
    def __init__(self):
        super().__init__('campaign_server')
        self.server = ActionServer(self, RunCampaign, '/atp/run_campaign',
                                   self.execute)
        self.result_pub = self.create_publisher(String, '/atp/campaign_result', 10)
        self.get_logger().info('listo: /atp/run_campaign')

    def execute(self, handle):
        g = handle.request
        n = g.n or 500
        kw = dict(n=n, semilla=g.seed or 2029,
                  sigma_viento_nm=g.sigma_wind_nm or 0.010,
                  sigma_ruido_deg=g.sigma_noise_deg or 0.02,
                  sesgo_max_deg=g.bias_max_deg or 0.03,
                  latencia_max_s=g.latency_max_s or 0.045,
                  kp=g.kp or 25.0, mascara_deg=g.mask_deg or model.MASCARA_DEG,
                  vel_max_deg_s=g.vel_max_deg_s or 6.0)
        mode = g.mode or 'analitico'
        self.get_logger().info(f'campana {mode} n={n}')
        t0 = time.monotonic()
        fb = RunCampaign.Feedback()

        def progreso(i, total):
            fb.done, fb.total = int(i * n / total), n
            handle.publish_feedback(fb)

        if mode == 'lazo':
            out = model.campana_lazo(progreso=progreso, **kw)
        elif mode == 'analitico':
            out = model.campana_analitica(**kw)
        else:
            handle.abort()
            return RunCampaign.Result()
        pc = model.percentiles(out['rms'])
        res = RunCampaign.Result()
        res.rms_deg = [float(x) for x in out['rms']]
        res.el_max_deg = [float(x) for x in out['el_max']]
        res.p50_deg, res.p95_deg, res.p99_deg = pc['p50'], pc['p95'], pc['p99']
        res.worst_deg = pc['peor']
        res.meets_objective = pc['pasa']
        res.objective_deg = model.OBJETIVO_DEG
        res.p95_uncertainty_pts = model.incertidumbre_percentil(n)
        res.wall_time_s = time.monotonic() - t0
        fb.done, fb.total, fb.p95_running_deg = n, n, pc['p95']
        handle.publish_feedback(fb)
        handle.succeed()
        self.get_logger().info(
            f'CAMPANA {mode} n={n} p50={pc["p50"]:.4f} p95={pc["p95"]:.4f} '
            f'p99={pc["p99"]:.4f} peor={pc["peor"]:.4f} deg | '
            f'{"CUMPLE" if pc["pasa"] else "NO CUMPLE"} p95<{model.OBJETIVO_DEG} '
            f'(+-{res.p95_uncertainty_pts:.1f} pts) | {res.wall_time_s:.1f} s')
        self.result_pub.publish(String(data=json.dumps({
            'mode': mode, 'n': n, **pc, 'wall_time_s': res.wall_time_s,
            'rms_deg': list(res.rms_deg), 'el_max_deg': list(res.el_max_deg)})))
        return res


def main():
    correr(CampaignServer)
