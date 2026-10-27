import { useEffect, useMemo, useRef, useState } from 'react'
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'
import { C } from '../colors.js'
import { nf } from '../format.js'
import { Legend, Panel } from './ui.jsx'

const D2R = Math.PI / 180
const PIVOT_Y = 1.8 // altura del eje de elevacion sobre el suelo
const LOS_LEN = 7

/** Direccion unitaria (ENU -> escena: x = este, y = arriba, z = -norte) de un acimut/elevacion en grados. */
export function dirOf(az, el) {
  const a = az * D2R, e = el * D2R
  return new THREE.Vector3(Math.sin(a) * Math.cos(e), Math.sin(e), -Math.cos(a) * Math.cos(e))
}

/** Ultima muestra <= i con los campos pedidos (la 1a muestra solo trae ref_*: el contrato omite lo que no hay). */
function lastWith(samples, i, ...keys) {
  for (let k = Math.min(i, samples.length - 1); k >= 0; k--) {
    const s = samples[k]
    if (keys.every((q) => s[q] != null)) return s
  }
  return null
}

function label(text, color) {
  const c = document.createElement('canvas')
  c.width = c.height = 64
  const g = c.getContext('2d')
  g.font = '700 40px monospace'
  g.fillStyle = color
  g.textAlign = 'center'
  g.textBaseline = 'middle'
  g.fillText(text, 32, 34)
  const tex = new THREE.CanvasTexture(c)
  const spr = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, transparent: true, depthTest: false }))
  spr.scale.set(0.9, 0.9, 1)
  return spr
}

/** Monta la escena: suelo, base, anillo de acimut, brazos y plato de elevacion, y las dos lineas. */
function buildRig() {
  const root = new THREE.Group()
  const std = (color, extra = {}) => new THREE.MeshStandardMaterial({ color, roughness: 0.6, metalness: 0.25, ...extra })

  const grid = new THREE.GridHelper(14, 28, C.furniture, C.furniture)
  grid.material.transparent = true
  grid.material.opacity = 0.45
  root.add(grid)
  for (const [t, x, z] of [['N', 0, -6.6], ['E', 6.6, 0], ['S', 0, 6.6], ['O', -6.6, 0]]) {
    const s = label(t, t === 'N' ? C.cyan : C.gray)
    s.position.set(x, 0.3, z)
    root.add(s)
  }

  const base = new THREE.Mesh(new THREE.CylinderGeometry(1.0, 1.25, 0.6, 40), std('#3a4b5c'))
  base.position.y = 0.3
  root.add(base)

  const az = new THREE.Group() // gira alrededor de Y
  az.position.y = 0.6
  root.add(az)
  const ring = new THREE.Mesh(new THREE.CylinderGeometry(0.85, 0.85, 0.2, 48), std('#4d6277'))
  ring.position.y = 0.1
  az.add(ring)
  const notch = new THREE.Mesh(new THREE.BoxGeometry(0.18, 0.24, 0.3), std(C.cyan, { emissive: C.cyan, emissiveIntensity: 0.35 }))
  notch.position.set(0, 0.12, -0.78) // marca la direccion "norte" del anillo
  az.add(notch)
  for (const sx of [-0.62, 0.62]) {
    const post = new THREE.Mesh(new THREE.BoxGeometry(0.14, PIVOT_Y - 0.8, 0.14), std('#566b80'))
    post.position.set(sx, 0.2 + (PIVOT_Y - 0.8) / 2, 0)
    az.add(post)
  }

  const el = new THREE.Group() // gira alrededor de X, en el eje de elevacion
  el.position.y = PIVOT_Y - 0.6
  az.add(el)
  const axle = new THREE.Mesh(new THREE.CylinderGeometry(0.07, 0.07, 1.4, 16), std('#8795a6'))
  axle.rotation.z = Math.PI / 2
  el.add(axle)
  // plato parabolico: la curva y = r^2 / 4f gira alrededor de Y; la boca queda hacia -Z (boresight)
  const f = 0.95
  const pts = []
  for (let i = 0; i <= 24; i++) {
    const r = (i / 24) * 1.15
    pts.push(new THREE.Vector2(r, (r * r) / (4 * f)))
  }
  const dishGeo = new THREE.LatheGeometry(pts, 48)
  dishGeo.rotateX(-Math.PI / 2)
  const dish = new THREE.Mesh(dishGeo, std('#cfe9f1', { side: THREE.DoubleSide, emissive: C.cyan, emissiveIntensity: 0.12 }))
  dish.position.z = 0.45
  el.add(dish)
  const feed = new THREE.Mesh(new THREE.SphereGeometry(0.1, 16, 12), std(C.cyan, { emissive: C.cyan, emissiveIntensity: 0.6 }))
  feed.position.z = 0.45 - f
  el.add(feed)
  const mast = new THREE.Mesh(new THREE.CylinderGeometry(0.03, 0.03, f, 8), std('#8795a6'))
  mast.rotation.x = Math.PI / 2
  mast.position.z = 0.45 - f / 2
  el.add(mast)

  const bore = new THREE.Line(
    new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, -LOS_LEN)]),
    new THREE.LineBasicMaterial({ color: C.cyan }),
  )
  el.add(bore)

  const losGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3(0, 0, -1)])
  const los = new THREE.Line(losGeo, new THREE.LineBasicMaterial({ color: C.amber, transparent: true, opacity: 0.85 }))
  los.frustumCulled = false
  const sat = new THREE.Mesh(new THREE.SphereGeometry(0.17, 20, 14), std(C.amber, { emissive: C.amber, emissiveIntensity: 0.7 }))
  root.add(los, sat)

  const hemi = new THREE.HemisphereLight('#cfe3ff', '#0b121c', 0.9)
  const sun = new THREE.DirectionalLight('#ffffff', 1.1)
  sun.position.set(5, 9, 6)
  root.add(hemi, sun)
  return { root, az, el, los, sat }
}

/**
 * Montura 3D (three.js empaquetado, sin CDN): base, anillo de acimut, brazos y plato de elevacion
 * orientados con la telemetria (`az`, `el` reales), y la linea de vista al satelite (`ref_az`,
 * `ref_el`, a un radio fijo: no esta a escala). Ligera: dibuja SOLO la muestra elegida y SOLO cuando
 * algo cambia (muestra, camara o tamano), asi que 10 000 muestras cuestan lo mismo que una.
 * En vivo sigue la ultima muestra; con el deslizador se puede ir a cualquier instante y reproducir.
 */
export default function Mount3D({ samples, live }) {
  const host = useRef(null)
  const gl = useRef(null) // {renderer, scene, camera, rig, request, frames}
  const [noGL, setNoGL] = useState(false)
  const [idx, setIdx] = useState(null) // null = seguir la ultima
  const [playing, setPlaying] = useState(false)
  const n = samples.length
  const i = idx == null ? n - 1 : Math.min(idx, n - 1)

  const pose = useMemo(() => {
    if (n === 0) return null
    const a = lastWith(samples, i, 'az', 'el')
    const r = lastWith(samples, i, 'ref_az', 'ref_el')
    const e = lastWith(samples, i, 'e_pt')
    return { t: samples[i].t, az: a?.az ?? 0, el: a?.el ?? 0, rAz: r?.ref_az ?? a?.az ?? 0, rEl: r?.ref_el ?? a?.el ?? 0, e: e?.e_pt ?? null, has: !!a }
  }, [samples, i, n])

  // escena: una sola vez
  useEffect(() => {
    const el = host.current
    let renderer
    try {
      renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false, powerPreference: 'low-power' })
    } catch {
      setNoGL(true)
      return undefined
    }
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2))
    renderer.setClearColor('#0b121c')
    el.appendChild(renderer.domElement)
    renderer.domElement.setAttribute('aria-label', 'Montura de la antena en 3D')
    const scene = new THREE.Scene()
    const camera = new THREE.PerspectiveCamera(42, 1, 0.1, 100)
    camera.position.set(5.2, 3.9, 6.0)
    const controls = new OrbitControls(camera, renderer.domElement)
    controls.target.set(0, 1.9, -1.2)
    controls.enableDamping = false // sin amortiguacion: no hace falta un bucle continuo
    controls.minDistance = 4
    controls.maxDistance = 22
    controls.maxPolarAngle = Math.PI * 0.495
    controls.update()
    const rig = buildRig()
    scene.add(rig.root)
    const st = { renderer, scene, camera, rig, frames: 0, raf: 0, dead: false }
    st.render = () => {
      st.raf = 0
      if (st.dead) return
      renderer.render(scene, camera)
      st.frames++
      el.dataset.frames = String(st.frames)
    }
    st.request = () => { if (!st.raf && !st.dead) st.raf = requestAnimationFrame(st.render) }
    controls.addEventListener('change', st.request)
    const resize = () => {
      const w = Math.max(200, el.clientWidth)
      const h = Math.round(Math.max(260, Math.min(w * 0.72, 460)))
      renderer.setSize(w, h)
      camera.aspect = w / h
      camera.updateProjectionMatrix()
      st.request()
    }
    const ro = new ResizeObserver(resize)
    ro.observe(el)
    resize()
    gl.current = st
    return () => {
      st.dead = true
      cancelAnimationFrame(st.raf)
      ro.disconnect()
      controls.dispose()
      scene.traverse((o) => {
        o.geometry?.dispose?.()
        const mats = Array.isArray(o.material) ? o.material : o.material ? [o.material] : []
        mats.forEach((m) => { m.map?.dispose?.(); m.dispose() })
      })
      renderer.dispose()
      renderer.domElement.remove()
      gl.current = null
    }
  }, [])

  // pose: cambia con la muestra elegida
  useEffect(() => {
    const st = gl.current
    if (!st || !pose) return
    const { rig } = st
    rig.az.rotation.y = -pose.az * D2R
    rig.el.rotation.x = pose.el * D2R
    const d = dirOf(pose.rAz, pose.rEl)
    const p0 = new THREE.Vector3(0, PIVOT_Y, 0)
    const p1 = p0.clone().addScaledVector(d, LOS_LEN)
    const pos = rig.los.geometry.attributes.position // en sitio: sin crear buffers nuevos por muestra
    pos.setXYZ(0, p0.x, p0.y, p0.z)
    pos.setXYZ(1, p1.x, p1.y, p1.z)
    pos.needsUpdate = true
    rig.los.geometry.computeBoundingSphere()
    rig.sat.position.copy(p1)
    st.request()
  }, [pose])

  // reproducir una corrida terminada
  useEffect(() => {
    if (!playing || n < 2) return undefined
    const stride = Math.max(1, Math.ceil(n / 360))
    const t = setInterval(() => {
      setIdx((cur) => {
        const nx = (cur == null ? 0 : cur) + stride
        if (nx >= n - 1) { setPlaying(false); return n - 1 }
        return nx
      })
    }, 45)
    return () => clearInterval(t)
  }, [playing, n])

  const following = idx == null
  return (
    <Panel
      title="Montura 3D"
      sub="Antena orientada con la telemetría; la línea ámbar va al satélite (no está a escala)"
      right={<Legend items={[{ color: C.cyan, label: 'eje de la antena' }, { color: C.amber, label: 'línea de vista al satélite' }]} />}
    >
      <div
        ref={host}
        className="mount3d"
        data-testid="mount3d"
        data-n={n}
        data-t={pose?.t ?? ''}
        data-az={pose ? pose.az : ''}
        data-el={pose ? pose.el : ''}
        data-ref-az={pose ? pose.rAz : ''}
        data-ref-el={pose ? pose.rEl : ''}
      >
        {noGL && <div className="empty"><p className="muted">WebGL no está disponible en este navegador: la montura 3D no se puede dibujar.</p></div>}
      </div>
      <div className="mount3d__bar">
        <button type="button" className="btn btn--sm" disabled={n < 2 || live} onClick={() => { if (!playing && (following || i >= n - 1)) setIdx(0); setPlaying((p) => !p) }}>
          {playing ? 'Pausa' : 'Reproducir'}
        </button>
        <input
          type="range" min="0" max={Math.max(0, n - 1)} step="1" value={i} disabled={n < 2}
          onChange={(e) => { setPlaying(false); setIdx(Number(e.target.value)) }}
          aria-label="Instante de la corrida"
        />
        <button type="button" className="btn btn--sm btn--ghost" disabled={following} onClick={() => { setPlaying(false); setIdx(null) }}>
          {live ? 'Volver al vivo' : 'Ir al final'}
        </button>
      </div>
      <p className="mount3d__read muted small" data-testid="mount3d-read">
        {pose
          ? <>t = {nf(pose.t, 1)} s · az {nf(pose.az, 2)}° · el {nf(pose.el, 2)}° · satélite az {nf(pose.rAz, 2)}° el {nf(pose.rEl, 2)}°{pose.e != null && <> · error {nf(pose.e, 3)}°</>}{following && live ? ' · siguiendo en vivo' : ''}</>
          : 'Sin muestras todavía'}
      </p>
    </Panel>
  )
}
