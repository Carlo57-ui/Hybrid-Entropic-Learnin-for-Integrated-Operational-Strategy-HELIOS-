#include "RF.h"
#include <LoRa.h>
#include <SPI.h>

// ---------------- Pines del módulo LoRa ----------------
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_CS 18
#define LORA_RST 14
#define LORA_IRQ 26

// ---------------- Parámetros de comunicación LoRa ----------------
#define LORA_FREQUENCY 915E6       // Frecuencia 915 MHz
#define LORA_BANDWIDTH 125E3       // Ancho de banda
#define LORA_SPREADING_FACTOR 7
#define LORA_CODING_RATE 5         // 
#define LORA_SYNC_WORD 0x34
#define LORA_TX_POWER 20           // dBm

// ---------------------------------------------------------------

void inicializarRF() {
  Serial.println("Iniciando módulo LoRa...");

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);

  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("Error: no se pudo inicializar LoRa.");
    while (true); // detiene el sistema
  }

  LoRa.setSignalBandwidth(LORA_BANDWIDTH);
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.setTxPower(LORA_TX_POWER);

  Serial.println("LoRa inicializado correctamente.");
}

// ---------------- Envía coordenadas actuales (q) ----------------
void enviarQ(double latitud, double longitud) {
  String mensaje = String(latitud, 6) + "," + String(longitud, 6);
  
  LoRa.beginPacket();
  LoRa.print(mensaje);
  LoRa.endPacket();

  Serial.print("Enviado q → ");
  Serial.println(mensaje);
}

// ---------------- Recibe coordenadas meta (q_meta) ----------------
bool recibirQmeta(double &lat_meta, double &lon_meta) {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String msg = "";
    while (LoRa.available()) {
      msg += (char)LoRa.read();
    }

    Serial.print("Mensaje recibido: ");
    Serial.println(msg);

    // Espera formato: "lat,lon"
    int sep = msg.indexOf(',');
    if (sep != -1) {
      lat_meta = msg.substring(0, sep).toDouble();
      lon_meta = msg.substring(sep + 1).toDouble();
      Serial.print("q_meta → ");
      Serial.print(lat_meta, 6);
      Serial.print(", ");
      Serial.println(lon_meta, 6);
      return true;
    } else {
      Serial.println("Formato de q_meta inválido.");
    }
  }
  return false;
}
