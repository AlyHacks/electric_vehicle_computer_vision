// ============================================================
// IMU Debug Tool v7 - R4 + Direct I2C registers + DLPF
// Uses same register config that gave 0.11deg/30s on R3
// ============================================================

#include <Wire.h>

// --- ICM-20948 registers ---
#define ICM_ADDR        0x69
#define ICM_REG_BANK    0x7F
#define ICM_PWR_MGMT_1  0x06
#define ICM_GYRO_CFG    0x01
#define ICM_GYRO_ZOUT_H 0x37
#define ICM_WHO_AM_I    0x00

float gyroZ_offset = 0.0;
float yawAngle     = 0.0;
unsigned long lastTime = 0;

// ============================================================
// Direct I2C helpers (same as R3 version that worked)
// ============================================================
void icmSelectBank(byte bank) {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(ICM_REG_BANK);
  Wire.write(bank << 4);
  Wire.endTransmission();
}

void icmWriteRegister(byte bank, byte reg, byte value) {
  icmSelectBank(bank);
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

int16_t icmReadGyroZ() {
  icmSelectBank(0);
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(ICM_GYRO_ZOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)ICM_ADDR, (uint8_t)2);
  byte high = Wire.available() ? Wire.read() : 0;
  byte low  = Wire.available() ? Wire.read() : 0;
  return (int16_t)((high << 8) | low);
}

void initI2C() {
  Wire.end();   // R4 specific reset
  delay(100);
  Wire.begin();
  Wire.setClock(100000); // 100kHz for stability
  Serial.println("I2C initialized at 100kHz");
}

bool icmInit() {
  for (int attempt = 1; attempt <= 3; attempt++) {
    Serial.print("ICM-20948 attempt "); Serial.print(attempt); Serial.println("/3...");

    icmSelectBank(0);
    Wire.beginTransmission(ICM_ADDR);
    Wire.write(ICM_WHO_AM_I);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)ICM_ADDR, (uint8_t)1);
    byte whoAmI = Wire.available() ? Wire.read() : 0xFF;
    Serial.print("WHO_AM_I: 0x"); Serial.println(whoAmI, HEX); // Should be EA

    if (whoAmI == 0xEA) {
      // Wake up from sleep, use best clock
      icmWriteRegister(0, ICM_PWR_MGMT_1, 0x01); delay(100);

      // Enable DLPF at 5.7Hz - this is the key setting that fixed R3
      // 0x31 = FCHOICE=1 (enable DLPF), DLPF_CFG=6 (5.7Hz), 250dps range
      icmWriteRegister(2, ICM_GYRO_CFG, 0x31); delay(50);

      Serial.println("ICM-20948 OK! DLPF enabled at 5.7Hz");
      return true;
    }

    Serial.println("Failed, re-initializing I2C...");
    initI2C();
    delay(500);
  }
  return false;
}

void calibrate() {
  Serial.println("Calibrating... keep still for 3 seconds!");

  // Discard 200 warmup readings to let DLPF fully settle
  for (int i = 0; i < 200; i++) { icmReadGyroZ(); delay(5); }

  long  sum    = 0;
  int16_t minVal =  32767;
  int16_t maxVal = -32768;

  for (int i = 0; i < 500; i++) {
    int16_t raw = icmReadGyroZ();
    sum += raw;
    if (raw < minVal) minVal = raw;
    if (raw > maxVal) maxVal = raw;
    delay(5);
  }

  gyroZ_offset = sum / 500.0;
  yawAngle     = 0.0;
  lastTime     = millis();

  Serial.println("--- Calibration Report ---");
  Serial.print("Offset: "); Serial.println(gyroZ_offset, 2);
  Serial.print("Spread: "); Serial.println(maxVal - minVal);
  // On R3 with DLPF enabled, spread was 24 - expect similar here
  if      (maxVal - minVal < 30)  Serial.println("Stability: GOOD");
  else if (maxVal - minVal < 80)  Serial.println("Stability: OK");
  else                            Serial.println("Stability: POOR");
  Serial.println("--------------------------");
  Serial.println("Commands: 'r' = recalibrate | 'z' = zero yaw");
  Serial.println();
  Serial.println("Raw Z  | Corrected(dps) | Yaw(deg)");
  Serial.println("-------|----------------|----------");
}

float icmGetGyroZ_dps() {
  return (icmReadGyroZ() - gyroZ_offset) / 131.0;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  Serial.println("=== IMU Debug Tool v7 (R4 + Direct Registers + DLPF) ===");

  initI2C();

  if (!icmInit()) {
    Serial.println("ICM-20948 FAILED! Check wiring.");
    while (true);
  }

  // Wait 3 seconds for DLPF to fully settle before calibrating
  Serial.println("Waiting 3000ms for DLPF to settle...");
  delay(3000);
  calibrate();
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'r') { Serial.println("\n>>> Recalibrating..."); calibrate(); }
    if (cmd == 'z') { yawAngle = 0.0; Serial.println("\n>>> Yaw zeroed."); }
  }

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  int16_t rawZ    = icmReadGyroZ();
  float corrected = (rawZ - gyroZ_offset) / 131.0;
  yawAngle       += corrected * dt;

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    Serial.print(rawZ);         Serial.print("\t | ");
    Serial.print(corrected, 4); Serial.print("\t | ");
    Serial.println(yawAngle, 2);
  }
}