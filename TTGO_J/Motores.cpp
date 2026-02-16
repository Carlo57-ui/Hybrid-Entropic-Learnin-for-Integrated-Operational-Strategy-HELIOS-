// Motores.cpp

#include "Motores.h"
#include <Arduino.h>

const int ENA = 2;   // PWM motor derecho
const int ENB = 13;  // PWM motor izquierdo
const int IN1 = 14;  // Dirección motor derecho (entrada 1)
const int IN2 = 25;  // Dirección motor derecho (entrada 2)
const int IN3 = 15;  // Dirección motor izquierdo (entrada 1)
const int IN4 = 35;  // Dirección motor izquierdo (entrada 2)

// Parámetros de configuración del PWM
const int PWM_FREQ = 1000; // Frecuencia PWM (1 kHz)
const int PWM_RES = 8;     // Resolución de 8 bits → valores 0–255
const float U_MAX = 1.0;   // Voltaje máximo lógico normalizado (±1.0)

// Configura los pines de los motores y activa el PWM.
void inicializarMotores() {
  // Define los pines de dirección como salidas
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Define los pines PWM como salida
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  ledcAttach(ENA, PWM_FREQ, PWM_RES); // PWM canal motor derecho
  ledcAttach(ENB, PWM_FREQ, PWM_RES); // PWM canal motor izquierdo
}

// Función: aplicarPWM(float uR, float uL)
// Aplica las señales PWM (derecha e izquierda) según los
// valores de voltaje/velocidad deseados.
void aplicarPWM(float uR, float uL) {
  // Escala las señales a rango 0–255 y evita saturación
  int pwm_r = constrain((int)(fabs(uR) / U_MAX * 255.0), 0, 255);
  int pwm_l = constrain((int)(fabs(uL) / U_MAX * 255.0), 0, 255);

  // -------------------------------------------------------
  // Dirección del motor derecho
  // Si uR ≥ 0 → hacia adelante, si no → hacia atrás
  // -------------------------------------------------------
  if (uR >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }

  // -------------------------------------------------------
  // Dirección del motor izquierdo
  // Si uL ≥ 0 → hacia adelante, si no → hacia atrás
  // -------------------------------------------------------
  if (uL >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

  // -------------------------------------------------------
  // Aplicar PWM a cada motor 
  // duty_cycle ∈ [0, 255] porque resolución = 8 bits
  // -------------------------------------------------------
  ledcWrite(ENA, pwm_r); // Motor derecho
  ledcWrite(ENB, pwm_l); // Motor izquierdo
}

