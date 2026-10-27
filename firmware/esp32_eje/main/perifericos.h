/* Perifericos del eje: encoder, motor (puente H), parada de emergencia,
 * sensor de corriente y LED de estado. Solo ESP32. */
#ifndef PERIFERICOS_H
#define PERIFERICOS_H

#include <stdbool.h>
#include <stdint.h>

/* Todas las funciones con `e` toman el eje (0 | 1); los `*_iniciar` preparan todos los ejes.
 * ---- encoder: cuentas acumuladas desde el arranque (int32) ---- */
int encoder_iniciar(void);
int32_t encoder_leer(int e);         /* eje `e`; nunca bloquea mas de unas decenas de us */
uint32_t encoder_errores(int e);     /* lecturas invalidas (solo el SPI)        */

/* ---- motor: LEDC (PWM) + habilitacion ---- */
int motor_iniciar(void);
/* duty -1..1 firmado; habilitar=false deja el puente en alta impedancia */
void motor_aplicar(int e, bool habilitar, float duty);
/* version minima para ISR y para el controlador de apagado: solo baja EN de TODOS los ejes */
void motor_apagar_rapido(void);

/* ---- parada de emergencia ---- */
int estop_iniciar(void);
bool estop_abierto(void);            /* nivel actual O disparo enclavado por la ISR */
void estop_limpiar(void);            /* quita el enclavamiento si el contacto ya cerro */

/* ---- sensor de corriente (BTS7960 R_IS / L_IS) ---- */
int corriente_iniciar(void);
float corriente_leer_a(int e);       /* A del eje `e`; NAN si no esta habilitado */

/* ---- LED de estado (GPIO2): patron segun el estado del eje ---- */
void led_iniciar(void);
void led_estado(int estado_eje, uint32_t tick);

#endif
