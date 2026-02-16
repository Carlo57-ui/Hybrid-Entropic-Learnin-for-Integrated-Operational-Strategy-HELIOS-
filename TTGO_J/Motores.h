// Motores.h

#ifndef MOTORES_H
#define MOTORES_H

#include <Arduino.h>

// Pines de motor
extern const int ENA;
extern const int IN1;
extern const int IN2;
extern const int IN3;
extern const int IN4;
extern const int ENB;

// Constantes PWM
extern const int PWM_CH_R;
extern const int PWM_CH_L;
extern const int PWM_FREQ;
extern const int PWM_RES;

// Funciones
void inicializarMotores();
void aplicarPWM(float uR, float uL); // uR, uL en voltios +/- U_MAX

#endif
