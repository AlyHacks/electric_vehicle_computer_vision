// ============================================================
// Scheme 1: Open-loop distance control
// Runs on Arduino UNO R4 WiFi
// Uses FspTimer hardware ISR for step generation (replaces FastAccelStepper)
// + Direct ICM-20948 I2C registers + ArduinoBLE
// ============================================================

#include <Wire.h>
#include <FspTimer.h>
#include <ArduinoBLE.h>

// --- Pin Definitions ---
#define LEFT_STEP_PIN  9
#define LEFT_DIR_PIN   12
#define LEFT_EN_PIN    8
#define RIGHT_STEP_PIN 10
#define RIGHT_DIR_PIN  6
#define RIGHT_EN_PIN   7
#define BUTTON_PIN     2

// --- ICM-20948 registers ---
#define ICM_ADDR        0x69
#define ICM_REG_BANK    0x7F
#define ICM_PWR_MGMT_1  0x06
#define ICM_GYRO_CFG    0x01
#define ICM_GYRO_ZOUT_H 0x37
#define ICM_WHO_AM_I    0x00

// --- Physical Parameters ---
const float WHEEL_DIAMETER_MM      = 76.2;
const float STEPS_PER_REV          = 200.0;
const int   MICROSTEP              = 8;
const float TARGET_DIST_MM         = 7000.0;
const unsigned long TARGET_TIME_MS = 10000;

// ============================================================
// TUNING PARAMETERS
// ============================================================
const float KP_HEADING         = 10.0;
const float BIAS                = 5.0;
const float DIST_CORRECTION_MM = 0.0;
const float ACCEL_TIME_SEC      = 2.0;
// ============================================================

// --- Calculated Steps and Speed ---
const float MM_PER_REV     = PI * WHEEL_DIAMETER_MM;
const float ACTUAL_DIST_MM = TARGET_DIST_MM + DIST_CORRECTION_MM;
const long  TOTAL_STEPS    = (long)((ACTUAL_DIST_MM / MM_PER_REV) * STEPS_PER_REV * MICROSTEP);
const float BASE_SPEED     = TOTAL_STEPS / (TARGET_TIME_MS / 1000.0); // steps/sec

// ============================================================
// FspTimer stepper engine
// Timer runs at TIMER_FREQ Hz, each motor has a counter
// When counter reaches its interval, one step pulse is generated
// Acceleration is handled by gradually changing the interval
// ============================================================
#define TIMER_FREQ 50000 // 50kHz = 20us per tick, max ~25000 steps/sec per motor

FspTimer stepTimer;

// Volatile because modified in ISR and read in main loop
volatile long  leftPosition      = 0;
volatile long  rightPosition     = 0;
volatile long  leftTarget        = 0;
volatile long  rightTarget       = 0;
volatile float leftIntervalF     = 0; // Fractional ticks between steps
volatile float rightIntervalF    = 0;
volatile float leftCounter       = 0; // Accumulator
volatile float rightCounter      = 0;
volatile bool  leftRunning       = false;
volatile bool  rightRunning      = false;

// Target speed in steps/sec - set from main loop
volatile float leftTargetSpeed   = 0;
volatile float rightTargetSpeed  = 0;
volatile float leftCurrentSpeed  = 0; // Current speed for acceleration
volatile float rightCurrentSpeed = 0;

// Acceleration in steps/sec^2
float ACCEL = 0;

// ISR called at TIMER_FREQ Hz
void stepISR(timer_callback_args_t *args) {

  // --- Acceleration: ramp current speed toward target speed ---
  float accelStep = ACCEL / TIMER_FREQ; // Speed change per tick

  if (leftCurrentSpeed < leftTargetSpeed)
    leftCurrentSpeed = min(leftCurrentSpeed + accelStep, leftTargetSpeed);
  else if (leftCurrentSpeed > leftTargetSpeed)
    leftCurrentSpeed = max(leftCurrentSpeed - accelStep, leftTargetSpeed);

  if (rightCurrentSpeed < rightTargetSpeed)
    rightCurrentSpeed = min(rightCurrentSpeed + accelStep, rightTargetSpeed);
  else if (rightCurrentSpeed > rightTargetSpeed)
    rightCurrentSpeed = max(rightCurrentSpeed - accelStep, rightTargetSpeed);

  // --- Left motor step ---
  if (leftRunning && leftPosition < leftTarget && leftCurrentSpeed > 0) {
    leftCounter += leftCurrentSpeed / TIMER_FREQ; // Fraction of a step per tick
    if (leftCounter >= 1.0) {
      leftCounter -= 1.0;
      digitalWrite(LEFT_STEP_PIN, HIGH);
      digitalWrite(LEFT_STEP_PIN, LOW);
      leftPosition++;
      if (leftPosition >= leftTarget) {
        leftRunning = false;
        leftCurrentSpeed = 0;
      }
    }
  }

  // --- Right motor step ---
  if (rightRunning && rightPosition < rightTarget && rightCurrentSpeed > 0) {
    rightCounter += rightCurrentSpeed / TIMER_FREQ;
    if (rightCounter >= 1.0) {
      rightCounter -= 1.0;
      digitalWrite(RIGHT_STEP_PIN, HIGH);
      digitalWrite(RIGHT_STEP_PIN, LOW);
      rightPosition++;
      if (rightPosition >= rightTarget) {
        rightRunning = false;
        rightCurrentSpeed = 0;
      }
    }
  }
}

// Set speed for both motors (called from main loop)
void setMotorSpeeds(float leftSpd, float rightSpd) {
  leftTargetSpeed  = constrain(leftSpd,  BASE_SPEED * 0.8, BASE_SPEED * 1.2);
  rightTargetSpeed = constrain(rightSpd, BASE_SPEED * 0.8, BASE_SPEED * 1.2);
}

// Start both motors moving to TOTAL_STEPS
void startMotors() {
  leftPosition  = 0;
  rightPosition = 0;
  leftTarget    = TOTAL_STEPS;
  rightTarget   = TOTAL_STEPS;
  leftCounter   = 0;
  rightCounter  = 0;
  leftCurrentSpeed  = 0;
  rightCurrentSpeed = 0;
  leftTargetSpeed   = BASE_SPEED;
  rightTargetSpeed  = BASE_SPEED;
  leftRunning   = true;
  rightRunning  = true;
}

void stopMotors() {
  leftRunning  = false;
  rightRunning = false;
  leftCurrentSpeed  = 0;
  rightCurrentSpeed = 0;
}

// ============================================================
// Heading and state
// ============================================================
float headingAngle      = 0.0;
float gyroZ_offset      = 0.0;
unsigned long lastTime  = 0;
unsigned long startTime = 0;
bool vehicleStarted     = false;

// ============================================================
// BLE Setup
// ============================================================
BLEService vehicleService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEStringCharacteristic statusChar(
  "19B10001-E8F2-537E-4F6C-D104768A1214",
  BLERead | BLENotify,
  100
);
unsigned long lastBLESend = 0;

// ============================================================
// Direct I2C helpers
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

float icmGetGyroZ_dps() {
  return (icmReadGyroZ() - gyroZ_offset) / 131.0;
}

void initI2C() {
  Wire.end();
  delay(100);
  Wire.begin();
  Wire.setClock(100000);
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
    Serial.print("WHO_AM_I: 0x"); Serial.println(whoAmI, HEX);

    if (whoAmI == 0xEA) {
      icmWriteRegister(0, ICM_PWR_MGMT_1, 0x01); delay(100);
      icmWriteRegister(2, ICM_GYRO_CFG,   0x31); delay(50);
      Serial.println("ICM-20948 OK!");
      return true;
    }
    initI2C();
    delay(500);
  }
  return false;
}

void icmCalibrateGyro() {
  Serial.println("Calibrating gyro... keep vehicle still!");
  for (int i = 0; i < 200; i++) { icmReadGyroZ(); delay(5); }
  long sum = 0;
  for (int i = 0; i < 500; i++) { sum += icmReadGyroZ(); delay(5); }
  gyroZ_offset = sum / 500.0;
  Serial.print("Gyro Z offset: "); Serial.println(gyroZ_offset, 2);
  Serial.println("Calibration done.");
}

// ============================================================
// BLE status broadcast
// ============================================================
void sendBLEStatus(float heading, float leftSpd, float rightSpd,
                   float correction, float distMM) {
  if (!BLE.connected()) return;
  if (millis() - lastBLESend < 200) return;
  lastBLESend = millis();
  char buf[100];
  snprintf(buf, sizeof(buf),
    "H:%.1f L:%.0f R:%.0f C:%.1f D:%.0f",
    heading, leftSpd, rightSpd, correction, distMM
  );
  statusChar.writeValue(buf);
}

// ============================================================
// Unified stop
// ============================================================
void stopVehicle(const char* reason) {
  stopMotors();
  digitalWrite(LEFT_EN_PIN,  HIGH);
  digitalWrite(RIGHT_EN_PIN, HIGH);

  float finalDist = leftPosition * MM_PER_REV / (STEPS_PER_REV * MICROSTEP);
  Serial.print("Stopped! Reason: "); Serial.println(reason);
  Serial.print("Time elapsed: ");
  Serial.print((millis() - startTime) / 1000.0, 2); Serial.println(" sec");
  Serial.print("Steps completed: "); Serial.println(leftPosition);
  Serial.print("Estimated distance: "); Serial.print(finalDist, 1); Serial.println(" mm");

  if (BLE.connected()) {
    char buf[100];
    snprintf(buf, sizeof(buf), "STOP:%s D:%.0f", reason, finalDist);
    statusChar.writeValue(buf);
  }
  while (true);
}

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  Serial.println("=== System Starting ===");

  pinMode(LEFT_STEP_PIN,  OUTPUT); digitalWrite(LEFT_STEP_PIN,  LOW);
  pinMode(RIGHT_STEP_PIN, OUTPUT); digitalWrite(RIGHT_STEP_PIN, LOW);
  pinMode(LEFT_DIR_PIN,   OUTPUT); digitalWrite(LEFT_DIR_PIN,   HIGH); // Direction
  pinMode(RIGHT_DIR_PIN,  OUTPUT); digitalWrite(RIGHT_DIR_PIN,  LOW);  // Inverted
  pinMode(LEFT_EN_PIN,    OUTPUT); digitalWrite(LEFT_EN_PIN,    HIGH); // Disabled
  pinMode(RIGHT_EN_PIN,   OUTPUT); digitalWrite(RIGHT_EN_PIN,   HIGH); // Disabled
  pinMode(BUTTON_PIN, INPUT);
  Serial.println("[1] Pins OK");

  // Calculate acceleration: steps/sec^2
  ACCEL = BASE_SPEED / ACCEL_TIME_SEC;

  // Start FspTimer at 50kHz
  uint8_t timerType = AGT_TIMER;
  int8_t  timerCh   = FspTimer::get_available_timer(timerType);
  if (timerCh < 0) {
    timerType = GPT_TIMER;
    timerCh   = FspTimer::get_available_timer(timerType);
  }
  stepTimer.begin(TIMER_MODE_PERIODIC, timerType, timerCh,
                  TIMER_FREQ, 0.0f, stepISR);
  stepTimer.setup_overflow_irq();
  stepTimer.open();
  stepTimer.start();
  Serial.println("[2] Steppers OK (FspTimer at 50kHz)");

  // ICM-20948
  initI2C();
  Serial.println("[3] I2C OK");

  if (!icmInit()) {
    Serial.println("ICM-20948 FAILED! Halting.");
    while (true);
  }
  Serial.println("[4] ICM-20948 OK");

  Serial.println("Waiting 3000ms for DLPF to settle...");
  delay(3000);
  icmCalibrateGyro();
  Serial.println("[5] Calibration OK");

  // ArduinoBLE
  if (!BLE.begin()) {
    Serial.println("BLE FAILED! Halting.");
    while (true);
  }
  BLE.setLocalName("SciolyEV");
  BLE.setAdvertisedService(vehicleService);
  vehicleService.addCharacteristic(statusChar);
  BLE.addService(vehicleService);
  BLE.advertise();
  Serial.println("[6] BLE OK - advertising as 'SciolyEV'");

  Serial.println("---");
  Serial.print("Base speed:   "); Serial.print(BASE_SPEED, 1);              Serial.println(" steps/sec");
  Serial.print("Accel time:   "); Serial.print(ACCEL_TIME_SEC, 1);          Serial.println(" sec");
  Serial.print("Total steps:  "); Serial.println(TOTAL_STEPS);
  Serial.print("Actual dist:  "); Serial.print(ACTUAL_DIST_MM);             Serial.println(" mm");
  Serial.print("Target time:  "); Serial.print(TARGET_TIME_MS / 1000.0, 1); Serial.println(" sec");
  Serial.print("KP_HEADING:   "); Serial.println(KP_HEADING, 2);
  Serial.print("BIAS:         "); Serial.println(BIAS, 1);
  Serial.println("---");
  Serial.println("Connect to 'SciolyEV' then press button to start.");
}

// ============================================================
// Loop - only handles logic, motors run in ISR
// ============================================================
void loop() {

  // --- Gyro read first, before BLE ---
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  if (vehicleStarted) {
    headingAngle += icmGetGyroZ_dps() * dt;
  }

  // --- BLE poll every 50ms ---
  static unsigned long lastBLEPoll = 0;
  if (millis() - lastBLEPoll >= 50) {
    lastBLEPoll = millis();
    BLE.poll();
  }

  // --- Wait for button press ---
  if (!vehicleStarted) {
    if (BLE.connected()) {
      static unsigned long lastIdle = 0;
      if (millis() - lastIdle > 1000) {
        lastIdle = millis();
        statusChar.writeValue("STATUS:Waiting for button...");
      }
    }

    if (digitalRead(BUTTON_PIN) == HIGH) {
      delay(50);
      if (digitalRead(BUTTON_PIN) == HIGH) {
        vehicleStarted = true;
        headingAngle   = 0.0;
        startTime      = millis();
        lastTime       = millis();

        digitalWrite(LEFT_EN_PIN,  LOW);
        digitalWrite(RIGHT_EN_PIN, LOW);
        delay(10);

        startMotors();

        Serial.println("Button pressed! Vehicle starting...");
        if (BLE.connected()) statusChar.writeValue("STATUS:Running!");
        while (digitalRead(BUTTON_PIN) == HIGH);
      }
    }
    return;
  }

  // --- Speed correction ---
  float kpCorrection = KP_HEADING * headingAngle;
  float leftSpd  = BASE_SPEED + kpCorrection + BIAS;
  float rightSpd = BASE_SPEED - kpCorrection - BIAS;
  setMotorSpeeds(leftSpd, rightSpd);

  // --- Serial debug every 200ms ---
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 200) {
    lastPrint = millis();
    float distMM = leftPosition * MM_PER_REV / (STEPS_PER_REV * MICROSTEP);
    Serial.print("H:"); Serial.print(headingAngle, 2);
    Serial.print(" L:"); Serial.print(leftSpd, 0);
    Serial.print(" R:"); Serial.print(rightSpd, 0);
    Serial.print(" D:"); Serial.println(distMM, 0);
  }

  // --- BLE status ---
  float distMM = leftPosition * MM_PER_REV / (STEPS_PER_REV * MICROSTEP);
  sendBLEStatus(headingAngle, leftSpd, rightSpd, kpCorrection, distMM);

  // --- Stop condition 1: Steps reached ---
  if (!leftRunning && !rightRunning) {
    stopVehicle("Target steps reached");
  }

  // --- Stop condition 2: Time limit reached ---
  if (millis() - startTime >= TARGET_TIME_MS) {
    stopVehicle("Time limit reached");
  }
}