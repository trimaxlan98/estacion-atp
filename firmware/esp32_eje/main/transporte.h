#ifndef TRANSPORTE_H
#define TRANSPORTE_H

/* Registra el transporte de micro-ROS (serie o UDP segun menuconfig).
 * En UDP arranca el Wi-Fi y espera la IP. Llamar antes de eje_uros_correr. */
void transporte_configurar(void);

#endif
