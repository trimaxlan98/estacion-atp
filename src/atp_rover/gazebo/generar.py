#!/usr/bin/env python3
"""Mundos de Gazebo Harmonic (gz sim 8) del rover planetario: terreno por heightmap, gravedad del
planeta, colores de cielo/suelo por planeta y el modelo SDF del rover de 6 ruedas rocker-bogie.

Equipo R3 (docs/ROVER.md, CONTRATO §18). No depende de ROS ni de planetas.py/terreno.py de R1: el
terreno que se dibuja aqui es PARA MIRAR (Gazebo no lee terreno.py); la fisica fina es la de R1.
Determinista: misma (planeta, terreno, semilla) -> mismos bytes.

    python3 generar.py <salida> [--planeta marte] [--terreno rocoso] [--semilla 1] [--modo diffdrive|estado]
                       [--prefijo-uri file:///ws/runs/x]   # como ve el contenedor la carpeta de salida
Escribe en <salida>: terreno.png (heightmap 16 bit), color.png (textura con el sombreado horneado),
normal.png, rover.sdf (modelo), mundo.sdf, gui.config.
"""
import argparse
import json
import math
import os
import struct
import zlib

import numpy as np

# gravedad (m/s^2) y paleta por planeta: cielo/suelo en hex sRGB (el SDF los lleva en lineal: Gazebo no
# aplica gamma, igual que scripts/gz_demo.sh); `sol` = (intensidad, elevacion_deg) de la luz.
PLANETAS = {
    'luna': {'g': 1.62, 'cielo': '#03050A', 'suelo': '#7C7D82', 'sol': (1.0, 22), 'ambiente': 0.10, 'nombre': 'Luna'},
    'marte': {'g': 3.71, 'cielo': '#B8794F', 'suelo': '#A04A28', 'sol': (0.95, 30), 'ambiente': 0.30, 'nombre': 'Marte'},
    'titan': {'g': 1.35, 'cielo': '#7A5A22', 'suelo': '#6B5A42', 'sol': (0.6, 40), 'ambiente': 0.7, 'nombre': 'Titán'},
    'ceres': {'g': 0.28, 'cielo': '#0A1030', 'suelo': '#7A746A', 'sol': (1.0, 25), 'ambiente': 0.3, 'nombre': 'Ceres'},
    'tierra': {'g': 9.81, 'cielo': '#6FA8DC', 'suelo': '#5E7F3C', 'sol': (1.0, 45), 'ambiente': 0.40, 'nombre': 'Tierra'},
}
TERRENOS = ('llano', 'rocoso', 'dunas', 'crater', 'pendiente')
LADO = 60.0          # m, el heightmap es cuadrado y centrado en el origen
N = 257              # 2^8 + 1 (requisito de Gazebo); 0.234 m por celda
PLANO_R = 3.5        # m alrededor del origen siempre llano (el rover nace ahi)


# ---------------------------------------------------------------- color (igual que atp_gz.launch.py)
def lineal(hex_, a=1.0, k=1.0):
    c = [int(hex_[i:i + 2], 16) / 255.0 for i in (1, 3, 5)]
    lin = [((v + 0.055) / 1.055) ** 2.4 if v > 0.04045 else v / 12.92 for v in c]
    return ' '.join(f'{v * k:.4f}' for v in lin) + f' {a}'


# Paleta Co.De Aerospace (marca): el rover va en plata, ambar y cian como las demas escenas.
M = {'plata': lineal('#F2F3F4', k=0.85), 'tenue': lineal('#8E99A6'), 'ambar': lineal('#F59E0B'),
     'cian': lineal('#00D9FF'), 'panel': lineal('#0B3A66'), 'rueda': lineal('#1B2733')}


# ---------------------------------------------------------------- terreno
def _ruido(rng, n, celdas):
    """Ruido de valor suave (interpolacion coseno) de `celdas` celdas de lado, en n x n."""
    g = rng.rand(celdas + 2, celdas + 2)
    t = np.linspace(0, celdas, n, endpoint=False)
    i = np.floor(t).astype(int)
    f = t - i
    f = (1 - np.cos(f * math.pi)) / 2
    a = g[i][:, i] * (1 - f)[None, :] * (1 - f)[:, None]
    a += g[i + 1][:, i] * (1 - f)[None, :] * f[:, None]
    a += g[i][:, i + 1] * f[None, :] * (1 - f)[:, None]
    a += g[i + 1][:, i + 1] * f[None, :] * f[:, None]
    return a - 0.5


def altura(tipo, semilla, n=N, lado=LADO):
    """Alturas en m (n x n; fila = y creciente, columna = x creciente). Siempre 0 en el origen."""
    rng = np.random.RandomState(int(semilla) & 0x7FFFFFFF)
    xs = np.linspace(-lado / 2, lado / 2, n)
    X, Y = np.meshgrid(xs, xs)
    R = np.hypot(X, Y)
    if tipo == 'llano':
        h = 0.02 * _ruido(rng, n, 24)
    elif tipo == 'rocoso':
        h = sum(a * _ruido(rng, n, c) for a, c in ((1.1, 4), (0.55, 9), (0.28, 20), (0.14, 44)))
        for _ in range(70):   # rocas: montículos de ~0.3-0.9 m
            cx, cy = rng.uniform(-lado / 2 + 2, lado / 2 - 2, 2)
            r, a = rng.uniform(0.35, 0.9), rng.uniform(0.15, 0.55)
            h = h + a * np.exp(-((X - cx) ** 2 + (Y - cy) ** 2) / (2 * (r / 2) ** 2))
    elif tipo == 'dunas':
        fase = 2.2 * _ruido(rng, n, 5) * 6
        h = 0.55 * np.sin((X * 0.9 + Y * 0.35) * 0.55 + fase) + 0.12 * _ruido(rng, n, 30)
        h = h * (0.75 + 0.5 * (_ruido(rng, n, 3) + 0.5))   # crestas de altura distinta
    elif tipo == 'crater':
        cx, cy, rc = 13.0, 4.0, 9.0
        d = np.hypot(X - cx, Y - cy) / rc
        cuenco = -2.2 * (1 - d ** 2) * (d < 1)
        borde = 0.7 * np.exp(-((d - 1.05) / 0.22) ** 2)
        h = cuenco + borde + 0.18 * _ruido(rng, n, 14)
        h = h + 0.5 * np.exp(-((np.hypot(X + 16, Y - 14)) / 2.5) ** 2)   # un montículo de referencia
    elif tipo == 'pendiente':
        a = math.radians(14.0)
        h = np.clip(X - 4.0, 0, None) * math.tan(a) + 0.06 * _ruido(rng, n, 30)
    else:
        raise ValueError(f'terreno desconocido: {tipo} (usa {TERRENOS})')
    # plano alrededor del origen con transición suave y cero exacto en (0,0)
    w = np.clip((R - PLANO_R) / 3.0, 0, 1)
    w = w * w * (3 - 2 * w)
    h = h * w
    h = h - h[n // 2, n // 2]      # el origen siempre en z = 0
    return h


def _png(ruta, filas, ancho, alto, modo):
    """PNG sin dependencias: modo 'L16' (gris 16 bit) o 'RGB' (8 bit). `filas`: bytes por fila."""
    bits, ctipo = (16, 0) if modo == 'L16' else (8, 2)

    def chunk(t, d):
        c = struct.pack('>I', len(d)) + t + d
        return c + struct.pack('>I', zlib.crc32(t + d) & 0xFFFFFFFF)
    raw = b''.join(b'\x00' + f for f in filas)
    with open(ruta, 'wb') as fh:
        fh.write(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', ancho, alto, bits, ctipo, 0, 0, 0))
                 + chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b''))


def escribir_terreno(salida, tipo, semilla, planeta):
    h = altura(tipo, semilla)
    hmin, hmax = float(h.min()), float(h.max())
    rango = max(hmax - hmin, 0.05)
    u = np.round((h - hmin) / rango * 65535).astype('>u2')
    # Gazebo lee la imagen con la fila 0 = norte (y mayor): se voltea
    _png(os.path.join(salida, 'terreno.png'), [r.tobytes() for r in u[::-1]], N, N, 'L16')
    # textura: color del suelo del planeta con sombreado de relieve (luz del sol) y tinte por altura
    base = np.array([int(PLANETAS[planeta]['suelo'][i:i + 2], 16) for i in (1, 3, 5)], float) / 255
    gy, gx = np.gradient(h, LADO / (N - 1))
    az = math.radians(215.0)
    sombra = np.clip(0.62 + 1.4 * (-gx * math.cos(az) - gy * math.sin(az)) / np.sqrt(1 + gx ** 2 + gy ** 2), 0.25, 1.15)
    alt = (h - hmin) / rango
    ruido = 0.06 * _ruido(np.random.RandomState(int(semilla) + 11), N, 60)
    k = sombra * (0.82 + 0.3 * alt) + ruido
    rgb = np.clip(base[None, None, :] * k[:, :, None], 0, 1)
    img = np.round(rgb * 255).astype(np.uint8)[::-1]
    _png(os.path.join(salida, 'color.png'), [r.tobytes() for r in np.ascontiguousarray(img)], N, N, 'RGB')
    return hmin, hmax


# ---------------------------------------------------------------- modelo del rover
RUEDA_R, RUEDA_W = 0.15, 0.10
CHASIS = (0.90, 0.56, 0.22)
Z_CHASIS = 0.60   # altura del centro del chasis sobre el suelo en reposo
PIVOTE_ROCKER = (0.0, 0.0)       # (x, z) respecto al chasis; y = lado * Y_ROCKER
PIVOTE_BOGIE = (-0.34, -0.26)
RUEDAS = {'d': (0.52, -0.45), 'm': (-0.06, -0.45), 't': (-0.62, -0.45)}   # delantera, media, trasera (x, z)
Y_RUEDA, Y_ROCKER = 0.46, 0.31
RIGIDEZ_ROCKER, RIGIDEZ_BOGIE = 120, 30   # N m/rad
MASAS = {'chasis': 14.0, 'rocker': 1.2, 'bogie': 0.9, 'rueda': 0.7}


def _ci(m, r, ln):      # inercia de un cilindro (eje z)
    return m * (3 * r * r + ln * ln) / 12, m * r * r / 2


def _inercia(m, ixx, iyy, izz):
    return (f'<inertial><mass>{m}</mass><inertia><ixx>{ixx:.5f}</ixx><iyy>{iyy:.5f}</iyy><izz>{izz:.5f}</izz>'
            f'<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>')


def _mat(c, emisivo=None):
    e = f'<emissive>{emisivo}</emissive>' if emisivo else ''
    return f'<material><ambient>{c}</ambient><diffuse>{c}</diffuse>{e}</material>'


def _barra(p0, p1, r, mat, nombre):
    """Cilindro visual de radio r entre p0 y p1 (en el marco del enlace)."""
    v = np.array(p1, float) - np.array(p0, float)
    ln = float(np.linalg.norm(v))
    c = (np.array(p0, float) + np.array(p1, float)) / 2
    pitch = math.acos(v[2] / ln)
    yaw = math.atan2(v[1], v[0])
    geo = f'<geometry><cylinder><radius>{r}</radius><length>{ln:.4f}</length></cylinder></geometry>'
    return (f'<visual name="{nombre}"><pose>{c[0]:.4f} {c[1]:.4f} {c[2]:.4f} 0 {pitch:.5f} {yaw:.5f}</pose>'
            f'{geo}{_mat(mat)}</visual>')


def _caja(pose, size, mat, nombre, colision=False):
    g = f'<geometry><box><size>{size[0]} {size[1]} {size[2]}</size></box></geometry>'
    s = f'<visual name="{nombre}"><pose>{pose}</pose>{g}{_mat(mat)}</visual>'
    if colision:
        s += f'<collision name="{nombre}_c"><pose>{pose}</pose>{g}</collision>'
    return s


def sdf_rover(modo='diffdrive', nombre='rover', pose='0 0 0 0 0 0'):
    """Modelo SDF del rover. `modo`: 'diffdrive' (dinamico, plugin DiffDrive en /rover/cmd_vel: skid-steer
    de las 6 ruedas) o 'estado' (estatico: lo coloca rover_estado_gz.py con /rover/estado de R1; las
    articulaciones no se mueven)."""
    estatico = modo == 'estado'
    cx, cy, cz = CHASIS
    visual = (_caja('0 0 0 0 0 0', CHASIS, M['plata'], 'caja', colision=True)
              + _caja('-0.30 0 0.17 0 0 0', (0.46, 0.50, 0.10), M['panel'], 'panel')
              + _caja('0.30 0 0.18 0 0 0', (0.08, 0.30, 0.12), M['tenue'], 'electronica')
              + f'<visual name="mastil"><pose>0.36 0 0.42 0 0 0</pose><geometry><cylinder><radius>0.025</radius>'
                f'<length>0.60</length></cylinder></geometry>{_mat(M["tenue"])}</visual>'
              + _caja('0.38 0 0.76 0 0 0', (0.14, 0.22, 0.09), M['ambar'], 'cabeza')
              + f'<visual name="antena"><pose>-0.45 0.12 0.30 0 0 0</pose><geometry><sphere><radius>0.07</radius>'
                f'</sphere></geometry>{_mat(M["cian"], M["cian"])}</visual>')
    mc = MASAS['chasis']
    L = [f'<link name="chasis">{_inercia(mc, mc * (cy ** 2 + cz ** 2) / 12, mc * (cx ** 2 + cz ** 2) / 12, mc * (cx ** 2 + cy ** 2) / 12)}{visual}</link>']
    juntas, izq, der = [], [], []
    rel = lambda p, o: (p[0] - o[0], p[1] - o[1], p[2] - o[2])   # noqa: E731
    for lado, tag in ((1, 'i'), (-1, 'd')):
        # balancin (rocker): pivota en el chasis; pata delantera a la rueda delantera y trasera al pivote del bogie
        pr = (PIVOTE_ROCKER[0], lado * Y_ROCKER, PIVOTE_ROCKER[1])
        yb = lado * Y_RUEDA
        pb = (PIVOTE_BOGIE[0], yb, PIVOTE_BOGIE[1])
        rd = (RUEDAS['d'][0], yb, RUEDAS['d'][1])
        vis = (_barra((0, 0, 0), rel(rd, pr), 0.022, M['tenue'], 'pata_del')
               + _barra((0, 0, 0), rel(pb, pr), 0.022, M['tenue'], 'pata_tra')
               + _barra((0, 0, 0), (0, lado * (Y_RUEDA - Y_ROCKER), 0), 0.03, M['tenue'], 'eje'))
        L.append(f'<link name="rocker_{tag}"><pose>{pr[0]} {pr[1]} {pr[2]} 0 0 0</pose>'
                 f'{_inercia(MASAS["rocker"], 0.03, 0.03, 0.01)}{vis}</link>')
        juntas.append(f'<joint name="rocker_{tag}_j" type="revolute"><parent>chasis</parent><child>rocker_{tag}</child>'
                      f'<axis><xyz>0 1 0</xyz><limit><lower>-0.5</lower><upper>0.5</upper></limit>'
                      f'<dynamics><damping>6</damping><friction>0.05</friction></dynamics></axis></joint>')
        # bogie: pivota en la parte trasera del balancin; lleva las ruedas media y trasera
        rm = (RUEDAS['m'][0], yb, RUEDAS['m'][1])
        rt = (RUEDAS['t'][0], yb, RUEDAS['t'][1])
        vis = (_barra((0, 0, 0), rel(rm, pb), 0.022, M['tenue'], 'pata_med')
               + _barra((0, 0, 0), rel(rt, pb), 0.022, M['tenue'], 'pata_tra'))
        L.append(f'<link name="bogie_{tag}"><pose>{pb[0]} {pb[1]} {pb[2]} 0 0 0</pose>'
                 f'{_inercia(MASAS["bogie"], 0.02, 0.02, 0.008)}{vis}</link>')
        juntas.append(f'<joint name="bogie_{tag}_j" type="revolute"><parent>rocker_{tag}</parent><child>bogie_{tag}</child>'
                      f'<axis><xyz>0 1 0</xyz><limit><lower>-0.6</lower><upper>0.6</upper></limit>'
                      f'<dynamics><damping>4</damping><friction>0.05</friction></dynamics></axis></joint>')
        # ruedas: giran sobre su eje (y); skid-steer: friccion lateral (mu, a lo largo de fdir1 = eje del cilindro)
        # menor que la de rodadura (mu2)
        for k, (rx, rz) in RUEDAS.items():
            padre = f'rocker_{tag}' if k == 'd' else f'bogie_{tag}'
            nom = f'rueda_{tag}{k}'
            ic, iz = _ci(MASAS['rueda'], RUEDA_R, RUEDA_W)
            geo = f'<geometry><cylinder><radius>{RUEDA_R}</radius><length>{RUEDA_W}</length></cylinder></geometry>'
            surf = ('<surface><friction><ode><mu>0.55</mu><mu2>1.1</mu2><fdir1>0 0 1</fdir1></ode></friction>'
                    '<contact><ode><kp>1e6</kp><kd>100</kd></ode></contact></surface>')
            taco = ''.join(
                f'<visual name="taco{j}"><pose>{RUEDA_R * math.cos(a):.4f} 0 {RUEDA_R * math.sin(a):.4f} 0 0 0</pose>'
                f'<geometry><box><size>0.05 {RUEDA_W + 0.012} 0.025</size></box></geometry>{_mat(M["tenue"])}</visual>'
                for j, a in enumerate(math.radians(x * 45) for x in range(8)))
            L.append(f'<link name="{nom}"><pose>{rx} {yb} {rz} 0 0 0</pose>{_inercia(MASAS["rueda"], ic, ic, iz)}'
                     f'<visual name="llanta"><pose>0 0 0 {math.pi / 2:.5f} 0 0</pose>{geo}{_mat(M["rueda"])}</visual>'
                     f'<collision name="c"><pose>0 0 0 {math.pi / 2:.5f} 0 0</pose>{geo}{surf}</collision>{taco}</link>')
            juntas.append(f'<joint name="{nom}_j" type="revolute"><parent>{padre}</parent><child>{nom}</child>'
                          f'<axis><xyz>0 1 0</xyz><dynamics><damping>0.02</damping></dynamics></axis></joint>')
            (izq if tag == 'i' else der).append(f'{nom}_j')
    plugin = ''
    if not estatico:
        plugin = ('<plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">'
                  + ''.join(f'<left_joint>{j}</left_joint>' for j in izq)
                  + ''.join(f'<right_joint>{j}</right_joint>' for j in der)
                  + f'<wheel_separation>{2 * Y_RUEDA}</wheel_separation><wheel_radius>{RUEDA_R}</wheel_radius>'
                  '<topic>/rover/cmd_vel</topic><odom_topic>/rover/odom</odom_topic>'
                  '<frame_id>odom</frame_id><child_frame_id>chasis</child_frame_id>'
                  '<max_linear_velocity>1.5</max_linear_velocity><max_angular_velocity>1.5</max_angular_velocity>'
                  '<max_linear_acceleration>1.0</max_linear_acceleration><max_angular_acceleration>2.0</max_angular_acceleration>'
                  '<odom_publish_frequency>20</odom_publish_frequency></plugin>')
        # Sin la barra diferencial de un rover real, los dos balancines (mismo eje y) dejarian al chasis
        # cabecear libremente. Se sustituye por una rigidez de resorte-amortiguador (controlador PD de posicion
        # alrededor de 0): el balancin y el bogie siguen el terreno, y el chasis se mantiene a nivel.
        for j, kp, kd in (('rocker_i_j', RIGIDEZ_ROCKER, 25), ('rocker_d_j', RIGIDEZ_ROCKER, 25),
                          ('bogie_i_j', RIGIDEZ_BOGIE, 8), ('bogie_d_j', RIGIDEZ_BOGIE, 8)):
            plugin += (f'<plugin filename="gz-sim-joint-position-controller-system" '
                       f'name="gz::sim::systems::JointPositionController"><joint_name>{j}</joint_name>'
                       f'<topic>/rover/{j}_cmd</topic><use_actuator_msg>false</use_actuator_msg>'
                       f'<p_gain>{kp}</p_gain><i_gain>0</i_gain><d_gain>{kd}</d_gain></plugin>')
    return (f'<model name="{nombre}"><pose>{pose}</pose><static>{"true" if estatico else "false"}</static>'
            f'{"".join(L)}{"".join(juntas)}{plugin}</model>')


# ---------------------------------------------------------------- mundo
def sdf_mundo(planeta, tipo, hmin, hmax, uri, modo='diffdrive', semilla=1):
    p = PLANETAS[planeta]
    inten, elev = p['sol']
    el = math.radians(elev)
    az = math.radians(215.0)
    dx, dy, dz = -math.cos(el) * math.cos(az), -math.cos(el) * math.sin(az), -math.sin(el)
    rango = max(hmax - hmin, 0.05)
    plugins = ''.join(f'<plugin filename="gz-sim-{s}-system" name="gz::sim::systems::{c}"/>' for s, c in (
        ('physics', 'Physics'), ('user-commands', 'UserCommands'), ('scene-broadcaster', 'SceneBroadcaster')))
    return f"""<?xml version="1.0"?>
<!-- Mundo del rover en {p['nombre']} (g = {p['g']} m/s^2), terreno {tipo}, semilla {semilla}. Generado por
     src/atp_rover/gazebo/generar.py: no se edita a mano. -->
<sdf version="1.9">
  <world name="rover_{planeta}">
    <physics name="2ms" type="dart"><max_step_size>0.002</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    {plugins}
    <gravity>0 0 {-p['g']}</gravity>
    <scene>
      <ambient>{lineal('#FFFFFF', k=p['ambiente'])}</ambient>
      <background>{lineal(p['cielo'])}</background>
      <grid>false</grid><shadows>true</shadows>
    </scene>
    <light type="directional" name="sol">
      <cast_shadows>true</cast_shadows><pose>0 0 30 0 0 0</pose>
      <diffuse>{lineal('#FFF4E0', k=min(inten, 1.0))}</diffuse><specular>{lineal('#FFFFFF', k=0.15)}</specular>
      <direction>{dx:.4f} {dy:.4f} {dz:.4f}</direction>
    </light>
    <model name="terreno">
      <static>true</static>
      <link name="l">
        <collision name="c"><geometry><heightmap><uri>{uri}/terreno.png</uri><size>{LADO} {LADO} {rango:.4f}</size></heightmap></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
        <visual name="v"><geometry><heightmap><uri>{uri}/terreno.png</uri><size>{LADO} {LADO} {rango:.4f}</size>
          <texture><diffuse>{uri}/color.png</diffuse><normal>{uri}/normal.png</normal><size>{LADO}</size></texture>
          </heightmap></geometry></visual>
      </link>
    </model>
    {sdf_rover(modo, pose=f'0 0 {Z_CHASIS + 0.02 - hmin:.4f} 0 0 0')}
  </world>
</sdf>
"""


def gui_config(res, planeta):
    w, h = res.split('x')
    cielo = lineal(PLANETAS[planeta]['cielo'])
    cfg = f"""<?xml version="1.0"?>
<window><width>{w}</width><height>{h}</height>
  <style material_theme="Dark" material_primary="#080F15" material_accent="#00D9FF"
         toolBarColorLight="#080F15" toolBarColorDark="#080F15" toolBarTextLight="#F2F3F4"
         toolBarTextDark="#F2F3F4" plugin_toolbar_color_light="#080F15" plugin_toolbar_color_dark="#080F15"
         plugin_toolbar_text_light="#F2F3F4" plugin_toolbar_text_dark="#F2F3F4"/>
</window>
<plugin filename="MinimalScene" name="3D View"><gz-gui><title>Rover</title>
  <property type="bool" key="showTitleBar">false</property></gz-gui>
  <engine>ogre2</engine><scene>scene</scene><ambient_light>0.3 0.3 0.3</ambient_light>
  <background_color>{cielo}</background_color>
  <camera_pose>-5.5 -4.5 2.6 0 0.30 0.68</camera_pose></plugin>
"""
    for filtro, nombre in (('GzSceneManager', 'Scene Manager'), ('InteractiveViewControl', 'Interactive view control'),
                           ('CameraTracking', 'Camera tracking')):
        cfg += f"""<plugin filename="{filtro}" name="{nombre}"><gz-gui>
  <anchors target="3D View"><line own="right" target="right"/><line own="top" target="top"/></anchors>
  <property key="resizable" type="bool">false</property>
  <property key="width" type="double">5</property><property key="height" type="double">5</property>
  <property key="state" type="string">floating</property>
  <property key="showTitleBar" type="bool">false</property></gz-gui></plugin>
"""
    return cfg


def escribir(salida, planeta='marte', terreno='rocoso', semilla=1, modo='diffdrive', uri=None, res='1920x1080'):
    if planeta not in PLANETAS:
        raise ValueError(f'planeta desconocido: {planeta} (usa {tuple(PLANETAS)})')
    os.makedirs(salida, exist_ok=True)
    uri = uri or ('file://' + os.path.abspath(salida))
    hmin, hmax = escribir_terreno(salida, terreno, semilla, planeta)
    # normal plana (128,128,255) 4x4: el sombreado ya va horneado en color.png
    _png(os.path.join(salida, 'normal.png'), [bytes([128, 128, 255] * 4)] * 4, 4, 4, 'RGB')
    with open(os.path.join(salida, 'mundo.sdf'), 'w') as fh:
        fh.write(sdf_mundo(planeta, terreno, hmin, hmax, uri, modo, semilla))
    # Gazebo (heightmap) no respeta de forma fiable un desplazamiento vertical del terreno entre visual y
    # colision: el terreno va de z = 0 a z = hmax - hmin y el origen del rover queda a z0 = -hmin.
    with open(os.path.join(salida, 'origen.json'), 'w') as fh:
        json.dump({'z0': round(-hmin, 4), 'hmin': round(hmin, 4), 'hmax': round(hmax, 4), 'lado': LADO}, fh)
    with open(os.path.join(salida, 'rover.sdf'), 'w') as fh:
        fh.write(f'<?xml version="1.0"?><sdf version="1.9">{sdf_rover(modo)}</sdf>\n')
    with open(os.path.join(salida, 'gui.config'), 'w') as fh:
        fh.write(gui_config(res, planeta))
    return {'mundo': os.path.join(salida, 'mundo.sdf'), 'world': f'rover_{planeta}', 'hmin': hmin, 'hmax': hmax}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('salida')
    ap.add_argument('--planeta', default='marte', choices=sorted(PLANETAS))
    ap.add_argument('--terreno', default='rocoso', choices=TERRENOS)
    ap.add_argument('--semilla', type=int, default=1)
    ap.add_argument('--modo', default='diffdrive', choices=('diffdrive', 'estado'))
    ap.add_argument('--prefijo-uri', default=None)
    ap.add_argument('--res', default='1920x1080')
    a = ap.parse_args()
    r = escribir(a.salida, a.planeta, a.terreno, a.semilla, a.modo, a.prefijo_uri, a.res)
    print(f"mundo={r['mundo']} world={r['world']} z=[{r['hmin']:.2f},{r['hmax']:.2f}] m")


if __name__ == '__main__':
    main()
