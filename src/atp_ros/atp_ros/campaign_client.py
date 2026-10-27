"""Cliente de la accion /atp/run_campaign: lanza una campana Monte Carlo desde
parametros ROS, imprime el feedback y escribe campaign.json (CONTRATO 4).

  ros2 run atp_ros campaign_client --ros-args -p mode:=lazo -p n:=300 -p output_dir:=/tmp/c

Un campo numerico en 0 deja que el servidor use su valor por defecto.
"""
import json
import os

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from atp_interfaces.action import RunCampaign
from atp_ros.params import declare_num

# campos del goal (ademas de mode y n)
CAMPOS = ('seed', 'sigma_wind_nm', 'sigma_noise_deg', 'bias_max_deg',
          'latency_max_s', 'kp', 'mask_deg', 'vel_max_deg_s')


class CampaignClient(Node):
    def __init__(self):
        super().__init__('campaign_client')
        self.declare_parameter('mode', 'analitico')
        self.declare_parameter('output_dir', '')
        declare_num(self, 'n', 500)
        for c in CAMPOS:
            declare_num(self, c, 0.0)
        declare_num(self, 'server_timeout_s', 30.0)
        self.client = ActionClient(self, RunCampaign, '/atp/run_campaign')

    def goal(self):
        g = RunCampaign.Goal()
        p = lambda k: self.get_parameter(k).value  # noqa: E731
        g.mode, g.n, g.seed = str(p('mode')), int(p('n')), int(p('seed'))
        for c in CAMPOS[1:]:
            setattr(g, c, float(p(c)))
        return g

    def feedback(self, msg):
        f = msg.feedback
        self.get_logger().info(
            f'progreso {f.done}/{f.total} | p95 parcial {f.p95_running_deg:.4f} deg')

    def run(self):
        """Devuelve el codigo de salida del proceso."""
        if not self.client.wait_for_server(
                timeout_sec=float(self.get_parameter('server_timeout_s').value)):
            self.get_logger().error('no hay servidor /atp/run_campaign')
            return 1
        goal = self.goal()
        self.get_logger().info(f'campana {goal.mode} n={goal.n}')
        fut = self.client.send_goal_async(goal, feedback_callback=self.feedback)
        rclpy.spin_until_future_complete(self, fut)
        handle = fut.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('meta rechazada')
            return 1
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        wrap = res_fut.result()
        if wrap.status != 4:  # GoalStatus.STATUS_SUCCEEDED
            self.get_logger().error(f'campana fallida (estado {wrap.status}): '
                                    f'modo desconocido?')
            return 1
        r = wrap.result
        out = {'mode': goal.mode, 'n': len(r.rms_deg), 'p50': r.p50_deg,
               'p95': r.p95_deg, 'p99': r.p99_deg, 'peor': r.worst_deg,
               'pasa': bool(r.meets_objective), 'objective_deg': r.objective_deg,
               'p95_uncertainty_pts': r.p95_uncertainty_pts,
               'wall_time_s': r.wall_time_s,
               'rms_deg': list(r.rms_deg), 'el_max_deg': list(r.el_max_deg)}
        self.get_logger().info(
            f'CAMPANA {goal.mode} n={out["n"]} p50={out["p50"]:.4f} p95={out["p95"]:.4f} '
            f'p99={out["p99"]:.4f} peor={out["peor"]:.4f} deg | '
            f'{"CUMPLE" if out["pasa"] else "NO CUMPLE"}')
        d = str(self.get_parameter('output_dir').value)
        if d:
            os.makedirs(d, exist_ok=True)
            path = os.path.join(d, 'campaign.json')
            with open(path + '.tmp', 'w', encoding='utf-8') as f:
                json.dump(out, f)
            os.replace(path + '.tmp', path)
            self.get_logger().info(f'campaign.json escrito en {d}')
        return 0


def main():
    rclpy.init()
    node = CampaignClient()
    code = 1
    try:
        code = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(code)
