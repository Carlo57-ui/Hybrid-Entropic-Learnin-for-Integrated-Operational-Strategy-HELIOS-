/* TTGO_J.ino
   TTGO_J para comunicaciones con Jetson + TTGO_L + control de agente
   - Envía q (lat, lon) a TTGO_L por LoRa cada 250 ms
   - Recibe q_meta por LoRa (payload "lat,lon")
   - Mide teta (yaw) en grados con filtro de Kalman
   - Recibe a_h por Serial (v,w)
   - Envía por Serial JSON con: {"q":[lat,lon],"teta":deg,"a_h":[v,w],"q_meta":[lat,lon]}
   - Control de motores (sin encoders) con estimación ωR, ωL
*/

#include <Arduino.h>
#include <ArduinoJson.h>
#include "Motores.h"
#include "Brujula.h"
#include "GPS_TTGO.h"
#include "RF.h"

// Variables globales
double lat = 0.0, lon = 0.0;
double lat_meta = 0.0, lon_meta = 0.0;
bool have_fix = false, have_q_meta = false;
float yaw_deg = 0.0;
float v = 0.0, w = 0.0; // velocidades recibidas por Serial

// --- Filtro de Kalman para yaw ---
struct Kalman {
  float q;      // proceso
  float r;      // medición
  float x;      // valor estimado
  float p;      // covarianza
  float k;      // ganancia de Kalman
};
Kalman kYaw = {0.01, 0.5, 0.0, 1.0, 0.0};

float filtroKalman(float medicion) {
  // Predicción
  kYaw.p += kYaw.q;
  // Actualización
  kYaw.k = kYaw.p / (kYaw.p + kYaw.r);
  kYaw.x += kYaw.k * (medicion - kYaw.x);
  kYaw.p *= (1 - kYaw.k);
  return kYaw.x;
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Inicializar módulos
  inicializarMotores();
  inicializarGPS();
  inicializarBrujula();
  inicializarRF();

  Serial.println("TTGO_J inicializado");
}

void loop() {
  // Actualizar yaw
  yaw_deg = actualizarBrujula();          // función de Brujula.h
  yaw_deg = filtroKalman(yaw_deg);        // filtrar para estabilidad

  // Actualizar GPS
  have_fix = leerGPS(lat, lon);

  // Recibir q_meta por LoRa
  have_q_meta = recibirQmeta(lat_meta, lon_meta);

  // Leer velocidades v,w desde Serial (formato [v,w] o v,w)
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    // Si viene en formato JSON [v,w]
    if (line.startsWith("[")) {
      line.replace("[", "");
      line.replace("]", "");
    }

    int sep = line.indexOf(',');
    if (sep > 0) {
      v = line.substring(0, sep).toFloat();
      w = line.substring(sep + 1).toFloat();
    }
  }

  // Comprobación de la llegada a la meta
    
  if (have_q_meta && have_fix) {
    float R = 6371000.0; // radio de la Tierra
    float dLat = radians(lat_meta - lat);
    float dLon = radians(lon_meta - lon);
    float a = sin(dLat/2) * sin(dLat/2) +
              cos(radians(lat)) * cos(radians(lat_meta)) *
              sin(dLon/2) * sin(dLon/2);
    float c = 2 * atan2(sqrt(a), sqrt(1 - a));
    float distancia = R * c; // en metros

    // Si está cerca de la meta (<2 m), detener motores
    if (distancia < 2.0) {
      aplicarPWM(0, 0);    // detener ambos motores
      v = 0;
      w = 0;
      Serial.println("Meta alcanzada: robot detenido.");
      delay(500);
      return;  // saltar el resto del loop
    }
  }

  // Control de motores (LQR usando yaw de Brujula)
{
  // --- Constantes físicas del robot ---
  const float L = 0.28;   // distancia entre ruedas (m)
  const float R = 0.14;   // radio de rueda (m)
  const float U_MAX = 255.0;  // PWM máximo

  // --- Matriz de ganancia LQR ---
  float K[2][2] = { {3320.8, -2796.0},
                    {-2796.0, 3320.8} };

  // --- Variables de referencia (enviadas desde Jetson) ---
  float v_ref = v;
  float w_ref = w;

  // --- Medición actual del yaw desde Brujula ---
  float yaw_rad = radians(yaw_deg);   // yaw_deg filtrado
  static float yaw_prev = yaw_rad;
  float w_fis = (yaw_rad - yaw_prev) / (0.05);  // derivada aprox. rad/s (asumiendo loop 50 ms)
  yaw_prev = yaw_rad;

  // --- Estimar velocidades reales de rueda ---
  float v_est = 0.0;  // sin encoder, opcional: asumir v_est ≈ v_ref si quieres estabilidad
  float wR_est = (2.0 * v_est + w_fis * L) / (2.0 * R);
  float wL_est = (2.0 * v_est - w_fis * L) / (2.0 * R);

  // --- Convertir referencias a rueda ---
  float wR_ref = (2.0 * v_ref + w_ref * L) / (2.0 * R);
  float wL_ref = (2.0 * v_ref - w_ref * L) / (2.0 * R);

  // --- Error de seguimiento ---
  float eR = wR_est - wR_ref;
  float eL = wL_est - wL_ref;

  // --- Control LQR ---
  float uR = -(K[0][0] * eR + K[0][1] * eL);
  float uL = -(K[1][0] * eR + K[1][1] * eL);

  // --- Saturar y aplicar PWM ---
  uR = constrain(uR, -U_MAX, U_MAX);
  uL = constrain(uL, -U_MAX, U_MAX);
  aplicarPWM(uR, uL);
}


  // Construir JSON
  StaticJsonDocument<256> doc;
  JsonArray q = doc.createNestedArray("q");
  q.add(lat);
  q.add(lon);
  doc["teta"] = yaw_deg;
  JsonArray a_h = doc.createNestedArray("a_h");
  a_h.add(v);
  a_h.add(w);
  JsonArray q_meta_json = doc.createNestedArray("q_meta");
  q_meta_json.add(lat_meta);
  q_meta_json.add(lon_meta);

  // Enviar JSON por Serial
  serializeJson(doc, Serial);
  Serial.println();

  // Enviar q por LoRa cada 250 ms
  static unsigned long lastSend = 0;
  if (millis() - lastSend > 250) {
    lastSend = millis();
    enviarQ(lat, lon);
  }

  delay(50);
}
