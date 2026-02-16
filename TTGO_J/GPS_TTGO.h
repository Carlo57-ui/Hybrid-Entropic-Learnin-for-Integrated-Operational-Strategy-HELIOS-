#ifndef GPS_TTGO_H
#define GPS_TTGO_H

#include <Arduino.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

// Pines del GPS
extern const int GPS_RX;
extern const int GPS_TX;

// Funciones
void inicializarGPS();
bool leerGPS(double &latitud, double &longitud);

#endif
