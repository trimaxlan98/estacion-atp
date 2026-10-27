// Paleta por ROL (curso ATP): el color dice el papel, no la decoracion.
export const C = {
  cyan: '#00D9FF', // cifra calculada y la antena
  amber: '#f59e0b', // el satelite / referencia y el p95
  violet: '#a78bfa', // cielo / predicho
  red: '#f43f5e', // keyhole, saturacion, fuera de presupuesto
  green: '#34d399', // dentro de presupuesto / enlace cerrado
  furniture: '#31414f', // mobiliario (ejes, rejillas)
  gray: '#94a0b0', // dato no calculado
  pale: '#cbd5e1', // segunda corrida en comparaciones (no tiene rol)
}

// Modos de mision del ADCS (bandas de fondo): el color dice QUE se apunta.
// DETUMBLE = satelite girando (ambar), NADIR = hacia la Tierra (violeta, "cielo"),
// TRACK_GS = antena a la estacion (cian, "antena").
export const MODE_COLOR = { DETUMBLE: C.amber, NADIR: C.violet, TRACK_GS: C.cyan }
export const MODE_NAMES = ['DETUMBLE', 'NADIR', 'TRACK_GS']

// Satelites del plan de contactos: aqui el color es IDENTIDAD (categorica), no rol, asi que se
// evitan el rojo y el verde (estado) y el cian (antena/estacion). La ISS, ambar. Con mas de 8
// satelites los colores se reciclan en orden y la leyenda/tooltip siguen dando el nombre.
export const SAT_COLORS = ['#f59e0b', '#60a5fa', '#f472b6', '#c4b5fd', '#fb923c', '#818cf8', '#e879f9', '#d6c28f']
export const satColor = (i) => SAT_COLORS[i % SAT_COLORS.length]
