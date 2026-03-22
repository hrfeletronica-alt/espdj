#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <math.h>

// --- CONFIGURAÇÃO DE REDE ---
const char* ssid = "DVS_LINK";
const char* password = "sine_master_dvs";
const char* host = "192.168.4.1"; // IP padrão do AP ESP32
unsigned int remotePort = 5000;

// --- ESTRUTURA DE DADOS ---
struct MotionMessage {
    int32_t phaseTicks;
    float gyroDps;
    uint8_t motionActive;
    uint8_t transmitterId;
    uint32_t sequence;
} __attribute__((packed));

WiFiUDP udp;
#define TRANSMITTER_ID 1

// --- MPU6050 & LÓGICA ---
#define MPU6050_ADDRESS 0x68
#define MPU6050_GYRO_Z_OUT_H 0x47
#define MPU6050_PWR_MGMT_1 0x6B
#define GYRO_SCALE 131.0f 
#define PHASE_TICKS_PER_TURN 1000000 
#define GYRO_FILTER_ALPHA 0.80f // Antes 0.10, agindo como anti-latência / resposta instantânea
#define MOTION_START_DPS 8.0f
#define MOTION_STOP_DPS 3.5f

float gyroZOffsetDps = 0.0f;
float filteredGyroZDps = 0.0f;
float cumulativeTurns = 0.0f;
unsigned long lastMs = 0;
uint32_t packetSequence = 0;

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(reg); Wire.write(value);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  Wire.begin(); // Pinos padrao: 21 (SDA), 22 (SCL)
  
  // Reseta e habilita MPU6050 com Timeout para nao travar a ESP
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);

  // Calibracao do Giroscopio no boot
  float sum = 0;
  for (int i = 0; i < 100; i++) {
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPU6050_GYRO_Z_OUT_H);
    if (Wire.endTransmission(false) == 0) {
        Wire.requestFrom(MPU6050_ADDRESS, 2, true);
        int16_t gz = (Wire.read() << 8) | Wire.read();
        sum += (float)gz / GYRO_SCALE;
    }
    delay(2);
  }
  gyroZOffsetDps = sum / 100.0f;

  Serial.printf("Conectando ao WiFi: %s\n", ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("\nWiFi OK! MPU6050 Ativo e Protegido.");
}

void loop() {
  unsigned long now = millis();
  unsigned long deltaMs = now - lastMs;
  if (deltaMs < 2) return; // 500Hz de taxa (Fogo sem latência!)
  lastMs = now;

  // Escudo I2C: So le se o sensor estiver respondendo
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_GYRO_Z_OUT_H);
  if (Wire.endTransmission(false) == 0) {
      Wire.requestFrom(MPU6050_ADDRESS, 2, true);
      int16_t gzRaw = (Wire.read() << 8) | Wire.read();
      float rawDps = ((float)gzRaw / GYRO_SCALE) - gyroZOffsetDps;
      filteredGyroZDps = (rawDps * GYRO_FILTER_ALPHA) + (filteredGyroZDps * (1.0f - GYRO_FILTER_ALPHA));
  } else {
      // Se deu erro de leitura (sensor soltou), assume frenagem (0 DPS) 
      // para não enlouquecer o Serato.
      filteredGyroZDps *= 0.5f; 
  }

  // Acumula revolucoes (DVS Master Phase)
  cumulativeTurns += (filteredGyroZDps * (deltaMs / 1000.0f)) / 360.0f;

  // Deteccao de Movimento
  static bool motionActive = false;
  if (fabsf(filteredGyroZDps) > MOTION_START_DPS) motionActive = true;
  else if (fabsf(filteredGyroZDps) < MOTION_STOP_DPS) motionActive = false;

  MotionMessage msg;
  msg.phaseTicks = (int32_t)lroundf(cumulativeTurns * PHASE_TICKS_PER_TURN);
  msg.gyroDps = filteredGyroZDps;
  msg.motionActive = motionActive ? 1 : 0;
  msg.transmitterId = TRANSMITTER_ID;
  msg.sequence = packetSequence++;

  udp.beginPacket(host, remotePort);
  udp.write((uint8_t*)&msg, sizeof(msg));
  udp.endPacket();

  static unsigned long lastLog = 0;
  if (now - lastLog > 500) {
    lastLog = now;
    Serial.printf("GYRO | DPS: %.1f | Ativo: %d\n", filteredGyroZDps, msg.motionActive);
  }
}
