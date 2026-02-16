// Brujula.h

#ifndef BRUJULA_H
#define BRUJULA_H

#include <Arduino.h>
#include <Wire.h>
#include <ICM20948_WE.h>


#define ICM20948_ADDR 0x68   // Dirección I2C del ICM20948

extern float yaw;        // Ángulo de orientación (grados 0–360)
extern float vx, vy;     // Velocidades lineales estimadas (m/s)
extern float alphaYaw;   // Filtro complementario para yaw
extern float alphaVel;   // Factor de suavizado de velocidad


/**
 * @brief Inicializa la brújula (IMU ICM20948) y configura sus rangos.
 * 
 * - Inicializa comunicación I2C y sensor.
 * - Configura rangos de acelerómetro y giroscopio.
 * - Inicializa y calibra el magnetómetro.
 * 
 * @return true si la inicialización fue exitosa.
 */
bool inicializarBrujula();

/**
 * @brief Actualiza las mediciones del IMU.
 * 
 * - Calcula yaw (rumbo) en grados, compensado por inclinación.
 * - Estima la magnitud de la velocidad lineal horizontal (v = √(vx²+vy²)).
 * 
 * @return float Velocidad lineal actual en m/s.
 */
float actualizarBrujula();

/**
 * @brief Devuelve el valor actual del ángulo de orientación (yaw).
 * 
 * @return float Rumbo en grados (0–360°).
 */
float obtenerYaw();

#endif // BRUJULA_H
