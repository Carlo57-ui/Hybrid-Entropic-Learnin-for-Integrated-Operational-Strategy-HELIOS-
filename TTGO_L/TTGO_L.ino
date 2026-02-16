#include <LoRa.h>
#include <SPI.h>

#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_CS 18
#define LORA_RST 14
#define LORA_IRQ 26
#define LORA_BAND 915E6

String q_meta = "";         // último q_meta recibido
unsigned long lastSend = 0; // para temporizar envíos

void setup() {
  Serial.begin(115200);

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);

  if (!LoRa.begin(LORA_BAND)) {
    Serial.println("LoRa init failed!");
    while (1);
  }
  Serial.println("TTGO_L listo.");
}

void loop() {
  // ------------------- Recibir q_meta de la laptop -------------------
  if (Serial.available()) {
    String incoming = Serial.readStringUntil('\n');
    incoming.trim();
    if (incoming.length() > 0) {
      q_meta = incoming; // guardar último q_meta recibido
      //Serial.print("q_meta guardado: ");
      Serial.println(q_meta);
    }
  }

  // ------------------- Reenviar q_meta al TTGO_J -------------------
  if (q_meta.length() > 0 && millis() - lastSend > 250) {
    lastSend = millis();
    LoRa.beginPacket();
    LoRa.print(q_meta);
    LoRa.endPacket();

    //Serial.print("Enviado q_meta → ");
    Serial.println(q_meta);
  }

  // ------------------- Recibir q desde TTGO_J -------------------
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String q = "";
    while (LoRa.available()) {
      q += (char)LoRa.read();
    }
    q.trim();
    if (q.length() > 0) {
      // Retransmitir q a la laptop
      //Serial.print("q recibido de TTGO_J → ");
      Serial.println(q);
    }
  }
}
