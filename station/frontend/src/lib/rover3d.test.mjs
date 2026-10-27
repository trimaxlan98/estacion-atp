// Datos del visor 3D del rover (CONTRATO §18): terreno determinista, estados normalizados, interpolacion y mock.
import test from 'node:test'
import assert from 'node:assert/strict'
import {
  PLANETAS, TERRENOS, alturaEn, generarTerreno, mockRover, normalizarTerreno, parsearEstados, pendienteEn, prepararRover, terrenoDeEscena,
} from './rover3d.js'

const iguales = (a, b) => a.length === b.length && a.every((v, i) => v === b[i])

test('generarTerreno: determinista por semilla y distinto con otra', () => {
  const a = generarTerreno('rocoso', 7, 65), b = generarTerreno('rocoso', 7, 65), c = generarTerreno('rocoso', 8, 65)
  assert.ok(iguales(a.z, b.z))
  assert.ok(!iguales(a.z, c.z))
  assert.equal(a.z.length, 65 * 65)
})

test('todos los tipos: z = 0 en el origen, llano alrededor del origen y valores finitos', () => {
  for (const tipo of TERRENOS) {
    const t = generarTerreno(tipo, 3, 129)
    assert.equal(alturaEn(t, 0, 0), 0, tipo)
    for (const [x, y] of [[0.5, 0.5], [-1, 1.5], [2, -2], [-2.4, 0]]) assert.ok(Math.abs(alturaEn(t, x, y)) < 0.05, `${tipo} llano en (${x},${y})`)
    assert.ok(t.z.every(Number.isFinite), tipo)
  }
  assert.throws(() => generarTerreno('hielo'))
})

test('los tipos tienen el relieve que dicen', () => {
  const rango = (t) => { let lo = Infinity, hi = -Infinity; for (const v of t.z) { lo = Math.min(lo, v); hi = Math.max(hi, v) } return hi - lo }
  assert.ok(rango(generarTerreno('llano', 1)) < 0.1)
  assert.ok(rango(generarTerreno('rocoso', 1)) > 1.2)
  assert.ok(rango(generarTerreno('dunas', 1)) > 0.8)
  const cr = generarTerreno('crater', 1)
  assert.ok(alturaEn(cr, 13, 4) < -1.5, 'el centro del crater esta hundido')
  assert.ok(alturaEn(cr, 13 + 9.5, 4) > 0.3, 'el borde sobresale')
  const pe = generarTerreno('pendiente', 1)
  const p = pendienteEn(pe, 15, 0).deg
  assert.ok(Math.abs(p - 14) < 1.5, `pendiente ${p}`)
  assert.ok(pendienteEn(pe, -10, 0).deg < 1)
})

test('alturaEn: exacta en los nodos, bilineal entre ellos y acotada fuera del mapa', () => {
  const n = 5
  const z = Float32Array.from({ length: n * n }, (_, k) => (k % n) + 10 * Math.floor(k / n))
  const t = { n, lado: 4, z }
  assert.equal(alturaEn(t, -2, -2), 0)
  assert.equal(alturaEn(t, 2 - 1e-9, -2) > 3.99, true)
  assert.ok(Math.abs(alturaEn(t, 0, 0) - 22) < 1e-9)
  assert.ok(Math.abs(alturaEn(t, 0.5, 0) - 22.5) < 1e-9)
  assert.ok(Number.isFinite(alturaEn(t, 1e6, -1e6)))
})

test('normalizarTerreno acepta el JSON de un backend y rechaza lo mal formado', () => {
  const t = normalizarTerreno({ tipo: 'dunas', lado_m: 20, z: [0, 1, 2, 3] })
  assert.equal(t.n, 2)
  assert.equal(t.lado, 20)
  assert.equal(t.z[3], 3)
  assert.equal(normalizarTerreno({ z: [1, 2, 3] }), null)
  assert.equal(normalizarTerreno(null), null)
})

test('parsearEstados: JSONL, lineas rotas, campos omitidos, orden y repetidos', () => {
  const txt = [
    '{"t":0.2,"x":1,"y":2,"rumbo_deg":10,"v":0.5}',
    'esto no es json',
    '',
    '{"t":0.1,"x":0,"y":0}',
    '{"t":0.2,"x":9,"y":9}',      // t repetido: se descarta
    '{"x":1,"y":1}',              // sin t
    '{"t":0.3,"x":"a","y":1}',    // x no numerico
    '{"t":0.4,"x":2,"y":3,"deslizamiento":[0.1,null,0.3],"atascado":true}',
  ].join('\n')
  const m = parsearEstados(txt)
  assert.deepEqual(m.map((s) => s.t), [0.1, 0.2, 0.4])
  assert.equal(m[0].rumbo, 0)
  assert.equal(m[0].z, undefined, 'un campo sin dato se omite')
  assert.equal(m[1].v, 0.5)
  assert.equal(m[2].atascado, true)
  assert.deepEqual(m[2].desliz, [0.1, undefined, 0.3])
  assert.deepEqual(parsearEstados(null), [])
  assert.deepEqual(parsearEstados([{ t: 1, x: 1, y: 1 }]).length, 1)
})

test('prepararRover: interpola, rumbo por el camino corto y completa z/cabeceo con el terreno', () => {
  const terreno = generarTerreno('pendiente', 1)
  const r = prepararRover({
    terreno,
    estados: [{ t: 0, x: 10, y: 0, rumbo_deg: 350 }, { t: 1, x: 12, y: 0, rumbo_deg: 10 }],
  })
  assert.equal(r.n, 2)
  assert.equal(r.t1, 1)
  const m = r.en(0.5)
  assert.ok(Math.abs(m.x - 11) < 1e-9)
  assert.ok(Math.abs(m.rumbo) < 1e-9 || Math.abs(m.rumbo - 360) < 1e-9, `rumbo ${m.rumbo}`)
  assert.ok(Math.abs(m.dist - 1) < 1e-9)
  // el estado minimo se coloca sobre el relieve: mirando cuesta arriba (+x) el morro sube
  assert.ok(m.z > 0.62, `z ${m.z}`)
  assert.ok(Math.abs(r.en(0).cabeceo) > 5, 'cabeceo desde el terreno')
  assert.equal(r.en(-5).t, -5, 'fuera de rango se acota a la primera muestra')
  assert.ok(Math.abs(r.dist - 2) < 1e-9)
  assert.equal(prepararRover({ estados: [] }).en(0), null)
})

test('prepararRover respeta lo que trae el estado (z, cabeceo, alabeo)', () => {
  const r = prepararRover({ estados: [{ t: 0, x: 0, y: 0, z: 3, cabeceo_deg: 5, alabeo_deg: -2 }, { t: 1, x: 1, y: 0, z: 4, cabeceo_deg: 7, alabeo_deg: -4 }] })
  const m = r.en(0.5)
  assert.equal(m.z, 3.5)
  assert.equal(m.cabeceo, 6)
  assert.equal(m.alabeo, -3)
})

test('parsearEstados: el deslizamiento por rueda da medio y maximo; un escalar vale como una rueda', () => {
  const [a, b, c] = parsearEstados([
    { t: 0, x: 0, y: 0, deslizamiento: [0.2, 0.4, 0.6, 0.2, 0.4, 0.6] },
    { t: 1, x: 1, y: 0, deslizamiento: 0.3 },
    { t: 2, x: 2, y: 0 },
  ])
  assert.ok(Math.abs(a.deslizMedio - 0.4) < 1e-12)
  assert.equal(a.deslizMax, 0.6)
  assert.equal(a.desliz.length, 6)
  assert.equal(b.deslizMedio, 0.3)
  assert.equal(b.deslizMax, 0.3)
  assert.equal(b.desliz.length, 1)
  assert.equal(c.deslizMedio, undefined)
  assert.equal(c.desliz, undefined)
})

test('prepararRover: columnas presentes, atascos como flancos de subida y deslizamiento interpolado', () => {
  const est = [0, 1, 2, 3, 4, 5, 6].map((t) => ({ t, x: t, y: 0, atascado: (t >= 2 && t <= 3) || t === 5, deslizamiento: [t / 10, t / 10], potencia_w: 50 + t }))
  const d = prepararRover({ estados: est, terreno: generarTerreno('llano', 1) })
  assert.deepEqual(d.atascos.map((a) => a.t), [2, 5], 'un atasco continuo (t = 2 y 3) cuenta una vez')
  assert.deepEqual(d.tiene, { energia: false, potencia: true, desliz: true })
  const m = d.en(1.5)
  assert.ok(Math.abs(m.deslizMedio - 0.15) < 1e-12, 'el deslizamiento se interpola')
  assert.equal(m.potencia, 51, 'la potencia es la de la muestra anterior, sin interpolar')
  assert.equal(d.en(2.5).atascado, true)
  assert.equal(d.en(4.5).atascado, false)
  const sin = prepararRover({ estados: [{ t: 0, x: 0, y: 0 }, { t: 1, x: 1, y: 0 }] })
  assert.deepEqual(sin.atascos, [])
  assert.deepEqual(sin.tiene, { energia: false, potencia: false, desliz: false })
  assert.equal(sin.en(0.5).deslizMedio, undefined)
})

test('mockRover: determinista, 10 Hz, dentro del mapa y subir cuesta frena', () => {
  const a = mockRover({ planeta: 'luna', terreno: 'dunas', semilla: 4, dur: 30 })
  const b = mockRover({ planeta: 'luna', terreno: 'dunas', semilla: 4, dur: 30 })
  assert.deepEqual(a.estados, b.estados)
  assert.equal(a.estados.length, 301)
  assert.ok(a.estados.every((e, i) => i === 0 || Math.abs(e.t - a.estados[i - 1].t - 0.1) < 1e-9))
  assert.ok(a.estados.every((e) => Math.abs(e.x) < 30 && Math.abs(e.y) < 30))
  assert.ok(a.estados.at(-1).energia_wh > 0)
  const r = prepararRover({ estados: a.estados, terreno: a.terreno, planeta: 'luna' })
  assert.ok(r.dist > 10 && r.vMax <= 0.65)
  // una pendiente lenta al frente reduce la velocidad media
  const llano = prepararRover(mockRover({ terreno: 'llano', dur: 30 }))
  const cuesta = prepararRover(mockRover({ terreno: 'pendiente', dur: 30 }))
  assert.ok(cuesta.dist < llano.dist + 1e-9 || cuesta.pendMax > llano.pendMax)
})

test('planetas: los cinco del contrato con la gravedad y colores', () => {
  assert.deepEqual(Object.keys(PLANETAS).sort(), ['ceres', 'luna', 'marte', 'tierra', 'titan'])
  assert.equal(PLANETAS.luna.g, 1.62)
  assert.equal(PLANETAS.marte.g, 3.71)
  assert.equal(PLANETAS.titan.g, 1.35)
  assert.equal(PLANETAS.ceres.g, 0.28)
  assert.equal(PLANETAS.tierra.g, 9.81)
  const cielos = new Set(Object.values(PLANETAS).map((p) => p.cielo))
  assert.equal(cielos.size, 5, 'cada planeta con su cielo')
  for (const p of Object.values(PLANETAS)) assert.match(p.suelo, /^#[0-9A-F]{6}$/i)
})

test('terrenoDeEscena: pendiente con la formula del simulador (angulo y direccion), sin arranque llano', () => {
  const rad = (g) => (g * Math.PI) / 180
  const t = terrenoDeEscena('pendiente', 1, { angulo_deg: 20, direccion_deg: 90 })
  assert.equal(t.exacto, true)
  assert.equal(alturaEn(t, 0, 0), 0)
  assert.ok(Math.abs(alturaEn(t, 0, 10) - Math.tan(rad(20)) * 10) < 1e-3, 'direccion 90: sube con y')
  assert.ok(Math.abs(alturaEn(t, 10, 0)) < 1e-3, 'direccion 90: no sube con x')
  assert.ok(Math.abs(alturaEn(t, 5, -5) + Math.tan(rad(20)) * 5) < 1e-3, 'y negativa: por debajo del origen')
  const d = terrenoDeEscena('pendiente', 1)                       // por omision, 15 grados hacia +x (como terreno.py)
  assert.ok(Math.abs(alturaEn(d, 10, 0) - Math.tan(rad(15)) * 10) < 1e-3)
  assert.ok(alturaEn(d, 2, 0) > 0.4, 'sin la zona llana del relieve del visor')
})

test('terrenoDeEscena: con la telemetria REAL de rover-tierra-lateral el chasis queda a una altura constante del suelo', () => {
  // (x, y, z) de la corrida real (pendiente de 20 grados, direccion 90): z = suelo + 0.41 m de chasis en todas las muestras.
  // Con el relieve propio del visor, que ignoraba los parametros, el rover quedaba enterrado a ~2 m.
  const filas = [[0, -0.26, 0.317], [4.19, -0.89, 0.089], [7.81, -1.07, 0.025], [9.6, -0.99, 0.052]]
  const t = terrenoDeEscena('pendiente', 1, { angulo_deg: 20, direccion_deg: 90 })
  const desfases = filas.map(([x, y, z]) => z - alturaEn(t, x, y))
  for (const f of desfases) assert.ok(Math.abs(f - desfases[0]) < 0.01, `desfase ${f}`)
  assert.ok(Math.abs(desfases[0] - 0.41) < 0.02)
  const enterrado = filas.map(([x, y, z]) => z - alturaEn(generarTerreno('pendiente', 1), x, y))
  assert.ok(Math.min(...enterrado) < -1, 'el relieve del visor dejaba al rover bajo el suelo')
})

test('terrenoDeEscena: escalones exactos, rocas sembradas y el resto aproximado', () => {
  const e = terrenoDeEscena('escalones', 1, { altura: 0.18, x_inicio: 3 })
  assert.equal(e.exacto, true)
  assert.equal(alturaEn(e, 1, 0), 0)
  assert.ok(Math.abs(alturaEn(e, 5, 7) - 0.18) < 1e-6)
  const dos = terrenoDeEscena('escalones', 1, { altura: 0.2, x_inicio: 2, separacion: 5, n: 2 })
  assert.ok(Math.abs(alturaEn(dos, 4, 0) - 0.2) < 1e-6 && Math.abs(alturaEn(dos, 9, 0) - 0.4) < 1e-6)
  const a = terrenoDeEscena('rocas', 3), b = terrenoDeEscena('rocas', 3), c = terrenoDeEscena('rocas', 4)
  assert.ok(iguales(a.z, b.z) && !iguales(a.z, c.z), 'determinista por semilla')
  assert.equal(a.exacto, false)
  let max = 0
  for (const v of a.z) max = Math.max(max, v)
  assert.ok(max > 0.1 && max < 0.6, `altura maxima de roca ${max}`)
  assert.ok(Math.abs(alturaEn(a, 0, 0)) < 0.05 && Math.abs(alturaEn(a, 1, -1)) < 0.05, 'zona libre de rocas alrededor del origen')
  assert.equal(terrenoDeEscena('hielo', 2).exacto, false, 'tipo desconocido: relieve llano aproximado, no falla')
  assert.equal(terrenoDeEscena('rocoso', 1, { loquesea: 1 }).exacto, false)
  assert.equal(terrenoDeEscena('pendiente', 1, null).exacto, true, 'parametros ausentes o nulos: los de omision')
})
