// Formato de los argumentos de una corrida (cabecera de la corrida y pantalla de comparar).
import test from 'node:test'
import assert from 'node:assert/strict'
import { argTxt, argsTxt } from './format.js'

test('argTxt: los diccionarios y las listas van en JSON, no como «[object Object]» ni «10,0»', () => {
  assert.equal(argTxt({ angulo_deg: 20, direccion_deg: 90 }), '{"angulo_deg":20,"direccion_deg":90}')
  assert.equal(argTxt([[10, 0]]), '[[10,0]]')
  assert.equal(argTxt(['a', 'b']), '["a","b"]')
  assert.equal(argTxt(0.5), '0.5')
  assert.equal(argTxt('luna'), 'luna')
  assert.equal(argTxt(true), 'true')
  assert.equal(argTxt(null), 'null')
  assert.equal(argTxt(undefined), 'undefined')
})

test('argsTxt: clave=valor separados por · y con claves omitidas', () => {
  const args = { planeta: 'tierra', terreno_params: { angulo_deg: 20 }, ruta: [[10, 0]], sats: [1, 2, 3] }
  assert.equal(argsTxt(args, ['sats']), 'planeta=tierra · terreno_params={"angulo_deg":20} · ruta=[[10,0]]')
  assert.ok(!argsTxt(args).includes('[object Object]'))
  assert.equal(argsTxt(undefined), '')
  assert.equal(argsTxt({}), '')
})
