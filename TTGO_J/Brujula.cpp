// Brujula.cpp
// Implementación de las funciones de lectura del ICM20948
// para obtener yaw (teta) y velocidad lineal.

#include "Brujula.h"

// Instancia del sensor ICM20948
ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);

// Variables globales
float yaw = 0.0;
float vx = 0.0, vy = 0.0;
float alphaYaw = 0.98;  // Filtro complementario (mezcla giroscopio/magnetómetro)
float alphaVel = 0.9;   // Suavizado exponencial para velocidad
unsigned long lastTime = 0;

// Inicialización del sensor IMU
bool inicializarBrujula() {
  Wire.begin();
  delay(100);

  if(!myIMU.init()){
    Serial.println("ICM20948 no responde");
    return false;
  }
  Serial.println("ICM20948 conectado correctamente");

  if (!myIMU.initMagnetometer()) {
    Serial.println("Magnetómetro no responde");
  } else {
    Serial.println("Magnetómetro conectado");
  }

  // Auto-calibración y configuración
  myIMU.autoOffsets();
  myIMU.setAccRange(ICM20948_ACC_RANGE_2G);
  myIMU.setGyrRange(ICM20948_GYRO_RANGE_250);
  myIMU.setAccDLPF(ICM20948_DLPF_6);
  myIMU.setGyrDLPF(ICM20948_DLPF_6);

  lastTime = millis();
  return true;
}

// Actualiza lecturas y calcula orientación y velocidad
float actualizarBrujula() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  // Leer datos del sensor
  myIMU.readSensor();

  xyzFloat g, mag, gyro, acc;
  myIMU.getGValues(&g);
  myIMU.getMagValues(&mag);
  myIMU.getGyrValues(&gyro);
  myIMU.getAccRawValues(&acc);

  // Calcular Pitch y Roll (para compensar inclinación)
  float roll  = atan2(g.y, g.z);
  float pitch = atan(-g.x / sqrt(g.y * g.y + g.z * g.z));

  // Compensar magnetómetro
  float Xh = mag.x * cos(pitch) + mag.z * sin(pitch);
  float Yh = mag.x * sin(roll) * sin(pitch) + mag.y * cos(roll) - mag.z * sin(roll) * cos(pitch);
  float yawMag = atan2(Yh, Xh) * 180.0 / PI;
  if (yawMag < 0) yawMag += 360.0;

  // Integración del giroscopio (Z)
  float yawGyro = yaw + gyro.z * dt;

  // Filtro complementario
  yaw = alphaYaw * yawGyro + (1 - alphaYaw) * yawMag;
  if (yaw < 0) yaw += 360;
  if (yaw > 360) yaw -= 360;

  // Convertir aceleraciones de g → m/s²
  float ax = g.x * 9.81;
  float ay = g.y * 9.81;
  float az = g.z * 9.81;

  // Quitar componente de gravedad (plano XY)
  ax -= sin(pitch) * 9.81;
  ay -= -sin(roll) * 9.81;

  // Integrar velocidad (filtrada)
  vx = alphaVel * (vx + ax * dt);
  vy = alphaVel * (vy + ay * dt);

  float v = sqrt(vx * vx + vy * vy);  // Magnitud de la velocidad

  return v;
}

// Obtener orientación actual
float obtenerYaw() {
  return yaw;
}
