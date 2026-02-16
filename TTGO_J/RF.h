#ifndef RF_H
#define RF_H

#include <Arduino.h>

// Inicializa el módulo LoRa
void inicializarRF();

// Envía las coordenadas actuales del robot (q)
void enviarQ(double latitud, double longitud);

// Recibe las coordenadas meta (q_meta)
// Retorna true si se recibió un mensaje válido con coordenadas
bool recibirQmeta(double &lat_meta, double &lon_meta);

#endif
