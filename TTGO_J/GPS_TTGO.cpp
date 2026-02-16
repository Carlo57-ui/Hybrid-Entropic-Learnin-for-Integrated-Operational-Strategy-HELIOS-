#include "GPS_TTGO.h"

TinyGPSPlus gps;
HardwareSerial ss(1);

// Pines de conexión 
const int GPS_RX = 34;  // RX ESP32 conectado al TX del GPS
const int GPS_TX = 12;  // TX ESP32 conectado al RX del GPS

void inicializarGPS() {
  Serial.println("Iniciando módulo GPS...");
  ss.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
}

bool leerGPS(double &latitud, double &longitud) {
  while (ss.available() > 0) {
    gps.encode(ss.read());
  }

  if (gps.location.isUpdated()) {
    latitud = gps.location.lat();
    longitud = gps.location.lng();

    // Imprime para depuración
    //Serial.print("Lat: "); Serial.println(latitud, 6);
    //Serial.print("Lon: "); Serial.println(longitud, 6);
    //Serial.print("Satélites: "); Serial.println(gps.satellites.value());
    //Serial.println("------------------------");

    return true;  // datos válidos
  }

  return false;  // aún no hay actualización
}
