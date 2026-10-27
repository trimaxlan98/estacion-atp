// Antena para el visor 3D web: la misma montura que Mount3D.jsx (base, anillo de acimut con marca norte,
// brazos, eje y plato parabolico de elevacion), pero con la geometria FUSIONADA por nivel de articulacion
// (fija / acimut / elevacion) y con color por vertice: 3 mallas por antena en vez de 9, y la geometria y
// los materiales se comparten entre antenas (2-4 antenas = 6-12 llamadas de dibujo).
import * as THREE from 'three'
import { mergeGeometries } from 'three/examples/jsm/utils/BufferGeometryUtils.js'

export const PIVOT_Y = 1.8 // altura del eje de elevacion sobre el suelo (misma que Mount3D)
const AZ_Y = 0.6
const EL_Y = PIVOT_Y - AZ_Y

let cache = null

function tint(geo, hex) {
  const c = new THREE.Color(hex)
  const n = geo.attributes.position.count
  const arr = new Float32Array(n * 3)
  for (let i = 0; i < n; i++) { arr[i * 3] = c.r; arr[i * 3 + 1] = c.g; arr[i * 3 + 2] = c.b }
  geo.setAttribute('color', new THREE.BufferAttribute(arr, 3))
  return geo
}

function part(geo, hex, { pos = [0, 0, 0], rotX = 0, rotZ = 0 } = {}) {
  const g = geo.index ? geo.toNonIndexed() : geo
  if (rotX) g.rotateX(rotX)
  if (rotZ) g.rotateZ(rotZ)
  g.translate(pos[0], pos[1], pos[2])
  g.deleteAttribute('uv')
  return tint(g, hex)
}

function build() {
  const base = mergeGeometries([part(new THREE.CylinderGeometry(1.0, 1.25, 0.6, 28), '#3a4b5c', { pos: [0, 0.3, 0] })])

  const azParts = [
    part(new THREE.CylinderGeometry(0.85, 0.85, 0.2, 32), '#4d6277', { pos: [0, 0.1, 0] }),
    part(new THREE.BoxGeometry(0.18, 0.24, 0.3), '#00D9FF', { pos: [0, 0.12, -0.78] }), // marca "norte" del anillo
  ]
  for (const sx of [-0.62, 0.62]) azParts.push(part(new THREE.BoxGeometry(0.14, PIVOT_Y - 0.8, 0.14), '#566b80', { pos: [sx, 0.2 + (PIVOT_Y - 0.8) / 2, 0] }))
  const az = mergeGeometries(azParts)

  const f = 0.95
  const pts = []
  for (let i = 0; i <= 14; i++) {
    const r = (i / 14) * 1.15
    pts.push(new THREE.Vector2(r, (r * r) / (4 * f)))
  }
  const dish = new THREE.LatheGeometry(pts, 32)
  dish.rotateX(-Math.PI / 2) // la boca queda hacia -Z (boresight)
  const el = mergeGeometries([
    part(new THREE.CylinderGeometry(0.07, 0.07, 1.4, 10), '#8795a6', { rotZ: Math.PI / 2 }),
    part(dish, '#cfe9f1', { pos: [0, 0, 0.45] }),
    part(new THREE.SphereGeometry(0.1, 10, 8), '#00D9FF', { pos: [0, 0, 0.45 - f] }),
    part(new THREE.CylinderGeometry(0.03, 0.03, f, 6), '#8795a6', { rotX: Math.PI / 2, pos: [0, 0, 0.45 - f / 2] }),
  ])

  const mat = new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.6, metalness: 0.25 })
  const matDish = new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.55, metalness: 0.3, side: THREE.DoubleSide, emissive: '#00D9FF', emissiveIntensity: 0.08 })
  return { base, az, el, mat, matDish }
}

/**
 * Antena: `{root, az, el, pivotY}`. root = Group en la posicion de la antena; `az.rotation.y = -acimut`,
 * `el.rotation.x = elevacion` (misma convencion que Mount3D); el eje de la antena es -Z de `el`.
 */
export function buildAntenna(scale = 1) {
  cache ||= build()
  const root = new THREE.Group()
  root.scale.setScalar(scale)
  root.add(new THREE.Mesh(cache.base, cache.mat))
  const az = new THREE.Group()
  az.position.y = AZ_Y
  az.add(new THREE.Mesh(cache.az, cache.mat))
  const el = new THREE.Group()
  el.position.y = EL_Y
  el.add(new THREE.Mesh(cache.el, cache.matDish))
  az.add(el)
  root.add(az)
  return { root, az, el, pivotY: PIVOT_Y * scale }
}

/** Libera la geometria compartida (al desmontar el visor). */
export function disposeAntennaCache() {
  if (!cache) return
  for (const k of ['base', 'az', 'el']) cache[k].dispose()
  cache.mat.dispose()
  cache.matDish.dispose()
  cache = null
}
