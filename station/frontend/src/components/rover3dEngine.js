// Motor del visor 3D del rover (three.js, sin Gazebo; equipo R3, CONTRATO §18). Dibuja el terreno (relieve con
// el color del suelo del planeta y sombreado), el rover de seis ruedas rocker-bogie y su trayectoria a partir de las
// muestras de /rover/estado (ver lib/rover3d.js). Es reproduccion cinematica: no calcula fisica, solo coloca el
// rover donde dice el estado (como el video de scripts/gz_rover.sh en modo `estado`).
//
// Todo se construye en ENU (x este, y norte, z arriba) dentro de un grupo girado -90 grados sobre x, que lo pasa a
// los ejes de three (y arriba). Las articulaciones del balancin y del bogie NO vienen en el estado: se dibujan
// aproximadas a partir del terreno bajo las ruedas (indicativas).
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import { PLANETAS, alturaEn } from '../lib/rover3d.js'

const D2R = Math.PI / 180
const CIAN = '#00D9FF'
const AMBAR = '#F59E0B'
const PLATA = '#D6D9DC'
const TENUE = '#8E99A6'
export const VELOCIDADES = [0.5, 1, 2, 5, 10, 20]

// geometria (la misma del modelo SDF de Gazebo, src/atp_rover/gazebo/generar.py)
const RUEDA_R = 0.15, RUEDA_W = 0.1
const Y_RUEDA = 0.46, Y_ROCKER = 0.31
const PIV_BOGIE = [-0.34, -0.26]
const RUEDAS = { d: [0.52, -0.45], m: [-0.06, -0.45], t: [-0.62, -0.45] }
const Z_CHASIS = 0.62

const mezclar = (hex, otro, f) => new THREE.Color(hex).lerp(new THREE.Color(otro), f)

function barra(p0, p1, r, mat) {
  const a = new THREE.Vector3(...p0), b = new THREE.Vector3(...p1)
  const v = b.clone().sub(a)
  const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, v.length(), 8), mat)
  m.position.copy(a.clone().add(b).multiplyScalar(0.5))
  m.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), v.normalize())
  return m
}

function construirRover() {
  const std = (color, extra = {}) => new THREE.MeshStandardMaterial({ color, roughness: 0.65, metalness: 0.2, ...extra })
  const mPlata = std(PLATA), mTenue = std(TENUE), mRueda = std('#1B2733', { roughness: 0.9 }), mPanel = std('#1c5f9e', { metalness: 0.5 })
  const mAmbar = std(AMBAR, { emissive: AMBAR, emissiveIntensity: 0.25 }), mCian = new THREE.MeshBasicMaterial({ color: CIAN })
  const g = new THREE.Group() // ENU: x adelante, y izquierda, z arriba; origen = centro del chasis
  const caja = (sx, sy, sz, x, y, z, mat) => { const m = new THREE.Mesh(new THREE.BoxGeometry(sx, sy, sz), mat); m.position.set(x, y, z); g.add(m); return m }
  caja(0.9, 0.56, 0.22, 0, 0, 0, mPlata)
  caja(0.46, 0.5, 0.1, -0.3, 0, 0.17, mPanel)
  caja(0.08, 0.3, 0.12, 0.3, 0, 0.18, mTenue)
  const mastil = new THREE.Mesh(new THREE.CylinderGeometry(0.025, 0.025, 0.6, 8), mTenue)
  mastil.rotation.x = Math.PI / 2
  mastil.position.set(0.36, 0, 0.42)
  g.add(mastil)
  caja(0.14, 0.22, 0.09, 0.38, 0, 0.76, mAmbar)
  const ant = new THREE.Mesh(new THREE.SphereGeometry(0.07, 12, 8), mCian)
  ant.position.set(-0.45, 0.12, 0.3)
  g.add(ant)

  const ruedas = []
  const art = { rocker: [], bogie: [] } // grupos que se inclinan (eje y)
  for (const lado of [1, -1]) {
    const yb = lado * Y_RUEDA
    const rocker = new THREE.Group()
    rocker.position.set(0, lado * Y_ROCKER, 0)
    const pb = [PIV_BOGIE[0], yb - lado * Y_ROCKER, PIV_BOGIE[1]]
    const rd = [RUEDAS.d[0], yb - lado * Y_ROCKER, RUEDAS.d[1]]
    rocker.add(barra([0, 0, 0], rd, 0.022, mTenue), barra([0, 0, 0], pb, 0.022, mTenue), barra([0, 0, 0], [0, lado * (Y_RUEDA - Y_ROCKER), 0], 0.03, mTenue))
    const bogie = new THREE.Group()
    bogie.position.set(...pb)
    for (const k of ['m', 't']) bogie.add(barra([0, 0, 0], [RUEDAS[k][0] - pb[0], 0, RUEDAS[k][1] - pb[2]], 0.022, mTenue))
    rocker.add(bogie)
    g.add(rocker)
    art.rocker.push(rocker)
    art.bogie.push(bogie)
    for (const k of ['d', 'm', 't']) {
      const padre = k === 'd' ? rocker : bogie
      const base = k === 'd' ? rd : [RUEDAS[k][0] - pb[0], 0, RUEDAS[k][1] - pb[2]]
      const w = new THREE.Group()
      w.position.set(...base)
      const llanta = new THREE.Mesh(new THREE.CylinderGeometry(RUEDA_R, RUEDA_R, RUEDA_W, 20), mRueda) // eje y de three = eje y ENU tras girar
      const giro = new THREE.Group()
      giro.add(llanta)
      for (let j = 0; j < 8; j++) {
        const a = (j / 8) * Math.PI * 2
        const taco = new THREE.Mesh(new THREE.BoxGeometry(0.05, RUEDA_W + 0.012, 0.025), mTenue)
        taco.position.set(RUEDA_R * Math.cos(a), 0, RUEDA_R * Math.sin(a))
        taco.rotation.y = -a
        giro.add(taco)
      }
      w.add(giro)
      padre.add(w)
      ruedas.push({ giro, lado })
    }
  }
  return { grupo: g, ruedas, art }
}

function colorTerreno(ter, planeta) {
  const { n, lado, z } = ter
  const base = new THREE.Color(planeta.suelo)
  let lo = Infinity, hi = -Infinity
  for (let i = 0; i < z.length; i++) { lo = Math.min(lo, z[i]); hi = Math.max(hi, z[i]) }
  const rango = Math.max(hi - lo, 0.05)
  const col = new Float32Array(n * n * 3)
  const paso = lado / (n - 1)
  const az = 215 * D2R
  const lx = Math.cos(az), ly = Math.sin(az) // hacia donde mira la luz
  const c = new THREE.Color()
  for (let i = 0; i < n; i++) {
    for (let j = 0; j < n; j++) {
      const gx = (z[i * n + Math.min(j + 1, n - 1)] - z[i * n + Math.max(j - 1, 0)]) / (paso * (j === 0 || j === n - 1 ? 1 : 2))
      const gy = (z[Math.min(i + 1, n - 1) * n + j] - z[Math.max(i - 1, 0) * n + j]) / (paso * (i === 0 || i === n - 1 ? 1 : 2))
      const sombra = Math.min(Math.max(0.62 + (1.4 * (-gx * lx - gy * ly)) / Math.sqrt(1 + gx * gx + gy * gy), 0.25), 1.15)
      const alt = (z[i * n + j] - lo) / rango
      const k = sombra * (0.82 + 0.3 * alt)
      c.copy(base).multiplyScalar(k)
      const o = (i * n + j) * 3
      col[o] = c.r; col[o + 1] = c.g; col[o + 2] = c.b
    }
  }
  return col
}

function construirTerreno(ter, planeta) {
  const { n, lado, z } = ter
  const pos = new Float32Array(n * n * 3)
  for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) {
    const o = (i * n + j) * 3
    pos[o] = -lado / 2 + (j / (n - 1)) * lado
    pos[o + 1] = -lado / 2 + (i / (n - 1)) * lado
    pos[o + 2] = z[i * n + j]
  }
  const idx = new Uint32Array((n - 1) * (n - 1) * 6)
  let q = 0
  for (let i = 0; i < n - 1; i++) for (let j = 0; j < n - 1; j++) {
    const a = i * n + j, b = a + 1, c = a + n, d = c + 1
    idx.set([a, b, d, a, d, c], q)
    q += 6
  }
  const geo = new THREE.BufferGeometry()
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3))
  geo.setAttribute('color', new THREE.BufferAttribute(colorTerreno(ter, planeta), 3))
  geo.setIndex(new THREE.BufferAttribute(idx, 1))
  geo.computeVertexNormals()
  return new THREE.Mesh(geo, new THREE.MeshLambertMaterial({ vertexColors: true }))
}

/**
 * Crea el visor. `datos` viene de prepararRover(). Devuelve un controlador imperativo. Lanza si no hay WebGL.
 * onHud(estado): se llama ~10 veces por segundo mientras algo cambia.
 */
export function createRoverEngine({ host, datos, onHud }) {
  const pl = PLANETAS[datos.planeta] || PLANETAS.marte
  const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'low-power', preserveDrawingBuffer: true })
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2))
  host.appendChild(renderer.domElement)

  const scene = new THREE.Scene()
  scene.background = new THREE.Color(pl.cielo)
  scene.fog = new THREE.Fog(pl.niebla, 35, 90)
  const camera = new THREE.PerspectiveCamera(50, 1, 0.1, 400)
  const controls = new OrbitControls(camera, renderer.domElement)
  controls.enableDamping = true
  controls.dampingFactor = 0.12
  controls.maxPolarAngle = Math.PI * 0.495

  const hemi = new THREE.HemisphereLight('#ffffff', pl.suelo, pl.ambiente)
  const sol = new THREE.DirectionalLight('#fff4e0', pl.sol)
  sol.position.set(-Math.cos(215 * D2R) * 30, 18, Math.sin(215 * D2R) * 30)
  scene.add(hemi, sol)

  const mundo = new THREE.Group() // ENU -> three
  mundo.rotation.x = -Math.PI / 2
  scene.add(mundo)
  mundo.add(construirTerreno(datos.terreno, pl))

  // trayectoria: completa (tenue) y recorrida (cian), a ras del suelo
  const nPts = datos.n
  const arr = new Float32Array(Math.max(nPts, 1) * 3)
  datos.pts.forEach((p, i) => { arr[i * 3] = p.x; arr[i * 3 + 1] = p.y; arr[i * 3 + 2] = alturaEn(datos.terreno, p.x, p.y) + 0.07 })
  const attr = new THREE.BufferAttribute(arr, 3)
  const geoTodo = new THREE.BufferGeometry(); geoTodo.setAttribute('position', attr)
  const geoVivo = new THREE.BufferGeometry(); geoVivo.setAttribute('position', attr)
  geoVivo.setDrawRange(0, 0)
  const lineaTodo = new THREE.Line(geoTodo, new THREE.LineBasicMaterial({ color: '#F2F3F4', transparent: true, opacity: 0.4 }))
  const lineaVivo = new THREE.Line(geoVivo, new THREE.LineBasicMaterial({ color: CIAN }))
  mundo.add(lineaTodo, lineaVivo)
  if (nPts) {
    const fin = datos.pts[nPts - 1]
    const aro = new THREE.Mesh(new THREE.TorusGeometry(0.45, 0.04, 8, 32), new THREE.MeshBasicMaterial({ color: AMBAR }))
    aro.position.set(fin.x, fin.y, alturaEn(datos.terreno, fin.x, fin.y) + 0.08)
    mundo.add(aro)
  }

  const rover = construirRover()
  const roverEnu = new THREE.Group()
  roverEnu.add(rover.grupo)
  mundo.add(roverEnu)

  const state = { t: datos.t0, playing: false, speed: 1, modo: 'seguir' }
  const objetivo = new THREE.Vector3()
  let raf = 0, last = 0, dirty = true, hudAt = 0, fpsN = 0, fpsT = 0, fps = 0, muerto = false
  const cur = datos.en(state.t)

  function colocar(m) {
    if (!m) return
    roverEnu.position.set(m.x, m.y, m.z)
    roverEnu.rotation.set(m.alabeo * D2R, -m.cabeceo * D2R, m.rumbo * D2R, 'ZYX')
    // articulaciones aproximadas: pendiente del suelo delante/detras respecto del cabeceo del chasis
    const r = m.rumbo * D2R
    const hF = alturaEn(datos.terreno, m.x + Math.cos(r) * 0.52, m.y + Math.sin(r) * 0.52)
    const hT = alturaEn(datos.terreno, m.x - Math.cos(r) * 0.62, m.y - Math.sin(r) * 0.62)
    const pend = Math.atan2(hF - hT, 1.14) / D2R
    const th = Math.max(-0.5, Math.min(0.5, (pend - m.cabeceo) * D2R * 0.6))
    for (const g of rover.art.rocker) g.rotation.y = -th
    for (const g of rover.art.bogie) g.rotation.y = th * 1.2
    const giro = m.dist / RUEDA_R
    for (const w of rover.ruedas) w.giro.rotation.y = giro
    if (m.i !== undefined) geoVivo.setDrawRange(0, Math.min(m.i + 1, nPts))
  }

  function aplicarCamara(m) {
    if (!m) return
    objetivo.set(m.x, m.z, -m.y)
    if (state.modo === 'libre') return
    const d = objetivo.clone().sub(controls.target)
    camera.position.add(d)
    controls.target.add(d)
  }

  function vista(nombre) {
    const m = datos.en(state.t) || { x: 0, y: 0, z: 0.6, rumbo: 0 }
    const c = new THREE.Vector3(m.x, m.z, -m.y)
    const r = m.rumbo * D2R
    controls.target.copy(c)
    if (nombre === 'cenit') camera.position.set(c.x, c.y + 28, c.z + 0.01)
    else if (nombre === 'mapa') { state.modo = 'libre'; controls.target.set(0, 0, 0); camera.position.set(0, 48, 40) }
    else if (nombre === 'cerca') camera.position.set(c.x - Math.cos(r) * 3.2 + Math.sin(r) * 1.2, c.y + 1.6, c.z + Math.sin(r) * 3.2 + Math.cos(r) * 1.2)
    else camera.position.set(c.x - Math.cos(r) * 7 + Math.sin(r) * 3, c.y + 4.2, c.z + Math.sin(r) * 7 + Math.cos(r) * 3)
    if (nombre !== 'mapa') state.modo = nombre === 'cenit' ? 'seguir' : state.modo
    controls.update()
    dirty = true
  }

  function resize() {
    const w = host.clientWidth || 640, h = host.clientHeight || 400
    renderer.setSize(w, h, false)
    renderer.domElement.style.width = '100%'
    renderer.domElement.style.height = '100%'
    camera.aspect = w / h
    camera.updateProjectionMatrix()
    dirty = true
  }
  const ro = new ResizeObserver(resize)
  ro.observe(host)
  resize()
  controls.addEventListener('change', () => { dirty = true })
  controls.addEventListener('start', () => { dirty = true })

  function emitir(now) {
    if (now - hudAt < 90) return
    hudAt = now
    const m = datos.en(state.t)
    onHud?.({
      t: state.t, t0: datos.t0, t1: datos.t1, playing: state.playing, speed: state.speed, modo: state.modo, fps,
      m, dist: m?.dist ?? 0, total: datos.dist,
    })
  }

  function frame(now) {
    if (muerto) return
    raf = requestAnimationFrame(frame)
    const dt = last ? Math.min((now - last) / 1000, 0.1) : 0
    last = now
    if (state.playing) {
      state.t += dt * state.speed
      if (state.t >= datos.t1) { state.t = datos.t1; state.playing = false }
      dirty = true
    }
    if (dirty) {
      const m = datos.en(state.t)
      colocar(m)
      if (state.modo === 'seguir') aplicarCamara(m)
      controls.update()
      renderer.render(scene, camera)
      dirty = false
      fpsN++
      if (now - fpsT > 1000) { fps = Math.round((fpsN * 1000) / (now - fpsT)); fpsN = 0; fpsT = now }
    }
    emitir(now)
  }

  colocar(cur)
  vista('seguir')
  raf = requestAnimationFrame(frame)

  return {
    state,
    play() { if (state.t >= datos.t1) state.t = datos.t0; state.playing = true; dirty = true; last = 0 },
    pause() { state.playing = false },
    seek(t) { state.t = Math.min(Math.max(t, datos.t0), datos.t1); dirty = true; hudAt = 0 },
    setSpeed(v) { state.speed = v },
    setModo(m) { state.modo = m; dirty = true },
    vista,
    captura() { renderer.render(scene, camera); return renderer.domElement.toDataURL('image/png') },
    dispose() {
      muerto = true
      cancelAnimationFrame(raf)
      ro.disconnect()
      controls.dispose()
      scene.traverse((o) => { o.geometry?.dispose?.(); const m = o.material; if (m) [].concat(m).forEach((x) => x.dispose?.()) })
      renderer.dispose()
      renderer.domElement.remove()
    },
  }
}
