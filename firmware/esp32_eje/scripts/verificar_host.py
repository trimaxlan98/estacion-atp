#!/usr/bin/env python3
"""Verifica, con ROS 2 de verdad, el nodo micro-ROS del eje (eje_host en Linux
o el ESP32) a traves del agente micro-ROS.

Hace de "Estacion": publica /atp/reference (20 Hz, name=[az, el], rampa de
9 deg/s en az) y comprueba, leyendo lo que el micro-ROS publica:

  1. nace DESARMADO y no obedece (posicion quieta aunque llegue referencia);
  2. cmd/enable=true  -> ACTIVO y sigue la rampa (error vs la referencia);
  3. se corta la referencia -> RETENCION a ~0.5 s y ESPERA a ~2.5 s (watchdog);
  4. vuelve la referencia -> ACTIVO otra vez (retoma desde donde esta);
  5. cmd/enable=false -> DESARMADO;
  6. /joint_states fusionado por atp_mcu (si esta corriendo mcu_merge).

Uso (dentro del contenedor ROS, con el agente y eje_host ya en marcha):
    python3 verificar_host.py [--eje az] [--vel-deg-s 9]
Sale con codigo 0 si todo pasa.
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray

ESTADOS = ('DESARMADO', 'ESPERA', 'ACTIVO', 'RETENCION', 'FALLA')


class Verificador(Node):
    def __init__(self, eje, vel_deg_s):
        super().__init__('verificador_mcu')
        self.eje = eje
        self.v = math.radians(vel_deg_s)
        self.ref_activa = True
        self.r_base = 0.2          # rad: referencia constante hasta que se pide la rampa
        self.t_rampa0 = None       # None: sin rampa
        self.ref_ult = self.r_base
        self.js = []          # (t, pos, vel, effort, nombres)
        self.diag = None
        self.diag_n = 0
        self.fusion = []
        self.pub_ref = self.create_publisher(JointState, '/atp/reference', 10)
        self.pub_en = self.create_publisher(Bool, f'/atp/mcu/{eje}/cmd/enable', 10)
        self.create_subscription(JointState, f'/atp/mcu/{eje}/joint_states', self.on_js, 50)
        self.create_subscription(Float32MultiArray, f'/atp/mcu/{eje}/diag', self.on_diag,
                                 qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states', self.on_fusion, 50)
        self.create_timer(0.05, self.publicar_ref)

    def ahora(self):
        return time.monotonic()

    def ref_en(self, t):
        if self.t_rampa0 is None:
            return self.r_base
        return self.r_base + self.v * (t - self.t_rampa0)

    def iniciar_rampa(self):
        """La rampa de 9 deg/s sale de la referencia actual, sin saltar."""
        self.r_base = self.ref_ult
        self.t_rampa0 = self.ahora()

    def publicar_ref(self):
        if not self.ref_activa:
            return
        self.ref_ult = self.ref_en(self.ahora())
        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = ['az', 'el']
        m.position = [0.0, self.ref_ult] if self.eje == 'el' else [self.ref_ult, 0.0]
        self.pub_ref.publish(m)

    def on_js(self, m):
        self.js.append((self.ahora(), m.position[0], m.velocity[0] if m.velocity else 0.0,
                        m.effort[0] if m.effort else 0.0, list(m.name)))

    def on_diag(self, m):
        self.diag = list(m.data)
        self.diag_n += 1

    def on_fusion(self, m):
        self.fusion.append((self.ahora(), list(m.name), list(m.position)))

    def estado(self):
        return ESTADOS[int(self.diag[0])] if self.diag else None

    def girar(self, s):
        fin = self.ahora() + s
        while self.ahora() < fin:
            rclpy.spin_once(self, timeout_sec=0.02)

    def esperar_estado(self, nombre, timeout):
        fin = self.ahora() + timeout
        while self.ahora() < fin:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.estado() == nombre:
                return self.ahora() - (fin - timeout)
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--eje', default='az')
    ap.add_argument('--vel-deg-s', type=float, default=9.0)
    ap.add_argument('--espera-nodo', type=float, default=40.0)
    a = ap.parse_args()

    rclpy.init()
    n = Verificador(a.eje, a.vel_deg_s)
    fallos = []

    def check(nombre, ok, detalle=''):
        print(f"{'OK   ' if ok else 'FALLA'} {nombre}" + (f'  [{detalle}]' if detalle else ''), flush=True)
        if not ok:
            fallos.append(nombre)

    # 0. el nodo del microcontrolador aparece (el agente lo crea al conectar)
    fin = n.ahora() + a.espera_nodo
    while n.ahora() < fin and n.diag is None:
        rclpy.spin_once(n, timeout_sec=0.05)
    check('el micro-ROS publica ~/diag (conexion con el agente)', n.diag is not None,
          f'tras {a.espera_nodo - (fin - n.ahora()):.1f} s')
    if n.diag is None:
        rclpy.shutdown()
        return 1
    n.girar(1.0)
    n.ref_activa = True

    # 1. desarmado: no obedece
    n.girar(1.5)
    pos0 = n.js[-1][1] if n.js else None
    check('nace DESARMADO', n.estado() == 'DESARMADO', f'estado={n.estado()}')
    n.girar(1.5)
    check('desarmado no se mueve aunque llegue referencia',
          abs(n.js[-1][1] - pos0) < 1e-3, f'pos {pos0:.5f} -> {n.js[-1][1]:.5f} rad, ref {n.ref_ult:.3f} rad')
    check('joint_states trae name=[<eje>], position/velocity/effort',
          n.js and n.js[-1][4] == [a.eje], f'name={n.js[-1][4] if n.js else None}')

    # 2. armar y seguir la rampa
    n.pub_en.publish(Bool(data=True))
    t = n.esperar_estado('ACTIVO', 5.0)
    check('cmd/enable=true -> ACTIVO', t is not None, f'en {t:.2f} s' if t else f'estado={n.estado()}')
    n.girar(4.0)                                   # alcanza la referencia fija (arranque sin salto, <= 12 deg/s)
    check('llega a la referencia fija de 0.2 rad', abs(n.js[-1][1] - n.r_base) < math.radians(0.3),
          f'pos={n.js[-1][1]:.4f} rad')
    n.iniciar_rampa()
    n.girar(5.0)                                   # transitorio de enganche de la rampa
    i0 = len(n.js)
    n.girar(8.0)
    err = []
    for (t_s, pos, _v, _e, _nm) in n.js[i0:]:
        err.append(abs(pos - n.ref_en(t_s)))
    if err:
        rms = math.sqrt(sum(e * e for e in err) / len(err))
        mx = max(err)
        check('sigue la rampa (regimen): error max < 0.5 deg',
              mx < math.radians(0.5), f'n={len(err)} rms={math.degrees(rms):.4f} deg max={math.degrees(mx):.4f} deg')
    else:
        check('hay joint_states en regimen', False)
    rate = (len(n.js) - i0) / 8.0
    check('joint_states ~50 Hz', 35.0 <= rate <= 60.0, f'{rate:.1f} Hz')
    check('diag publica (~2 Hz)', n.diag_n >= 8, f'{n.diag_n} mensajes')
    d = n.diag
    print(f"     diag: t_lazo={d[9]:.0f} us max={d[10]:.0f} us jitter_max={d[11]:.0f} us "
          f"perdidos={d[12]:.0f} refs={d[13]:.0f}", flush=True)

    # 6. fusion de atp_mcu (si corre)
    if n.fusion:
        nombres = n.fusion[-1][1]
        check('/joint_states fusionado: name=[az, el]', nombres == ['az', 'el'], f'name={nombres}')
    else:
        print('SALTA /joint_states fusionado (mcu_merge no esta corriendo)', flush=True)

    # 3. se corta la referencia
    n.ref_activa = False
    t_corte = n.ahora()
    t_ret = n.esperar_estado('RETENCION', 3.0)
    check('referencia cortada -> RETENCION (~0.5 s)', t_ret is not None and 0.3 <= t_ret <= 1.2,
          f'{t_ret:.2f} s' if t_ret else f'estado={n.estado()}')
    t_esp = n.esperar_estado('ESPERA', 5.0)
    total = n.ahora() - t_corte
    check('sigue sin referencia -> ESPERA, driver apagado (~2.5 s)', t_esp is not None and 2.0 <= total <= 4.0,
          f'{total:.2f} s desde el corte' if t_esp else f'estado={n.estado()}')
    check('en ESPERA el mando es 0', abs(n.js[-1][3]) < 1e-6, f'effort={n.js[-1][3]}')

    # 4. vuelve la referencia
    n.iniciar_rampa()                              # la rampa sigue donde iba, sin saltar
    n.ref_activa = True
    t = n.esperar_estado('ACTIVO', 5.0)
    check('vuelve la referencia -> ACTIVO', t is not None, f'en {t:.2f} s' if t else f'estado={n.estado()}')

    # 5. desarmar
    n.pub_en.publish(Bool(data=False))
    t = n.esperar_estado('DESARMADO', 3.0)
    check('cmd/enable=false -> DESARMADO', t is not None, f'estado={n.estado()}')

    print(f"\n{'TODO OK' if not fallos else 'FALLARON: ' + ', '.join(fallos)}", flush=True)
    n.destroy_node()
    rclpy.shutdown()
    return 1 if fallos else 0


if __name__ == '__main__':
    sys.exit(main())
